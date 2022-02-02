/*
 * configuration.c
 *
 *  Created on: Jan 31, 2022
 *      Author: Dukalis
 */

#include "main.h"

KALIBR_STRUCT kalibr =
{ 0 };
CONFIGURATION_STRUCT config =
{ 0 };

void Configurator_Serv(void)
{
  if (config.status == CONFIGURATION_DOESNT_LOADED)
    Read_Configuration();

  if (config.pars_index != rs485.rx_index)
  {
    LLS_Parsing(rs485.buf_in[config.pars_index]);
    if (++config.pars_index == RS485_BUF_SIZE)
      config.pars_index = 0;
  }
}
void Read_Configuration(void)
{
  load_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
  if ((kalibr.crc8 != crc_str((char*) &kalibr, sizeof(kalibr) - 1))
      || (!kalibr.crc8))
  {
    kalibr.threshold = 400;
    kalibr.threshold_type = ACC_DETECTOR_DISTANCE_THRESHOLD_TYPE_FIXED;
    kalibr.cfar_threshold_only_lower_distance = 1;
    kalibr.guard = 0.06f;
    kalibr.window = 0.02f;
    kalibr.threshold_sensitivity = 0.8f;
    kalibr.start_m = 0.2f;
    kalibr.length_m = 5.0f;
    kalibr.average = 1.0f;
    kalibr.profile = ACC_SERVICE_PROFILE_2;
    kalibr.gain = 0.1f;
    kalibr.meas_freq = 0.10f;
    kalibr.peak_num = 5;
    kalibr.peak_sorting = ACC_DETECTOR_DISTANCE_PEAK_SORTING_CLOSEST_FIRST;
    kalibr.sweep_averaging = 1;
    kalibr.hw_acc_samples = 63;
    write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
  }
  config.status = CONFIGURATION_READY;
}
char crcc(char dat_crc, char crc8)
{
  char j;
  j = dat_crc ^ crc8;
  crc8 = 0;
  if (j & 0x01)
    crc8 ^= 0x5e;
  if (j & 0x02)
    crc8 ^= 0xbc;
  if (j & 0x04)
    crc8 ^= 0x61;
  if (j & 0x08)
    crc8 ^= 0xc2;
  if (j & 0x10)
    crc8 ^= 0x9d;
  if (j & 0x20)
    crc8 ^= 0x23;
  if (j & 0x40)
    crc8 ^= 0x46;
  if (j & 0x80)
    crc8 ^= 0x8c;
  return crc8;
}
char crc_str(char *str6, int lens)
{
  uint8_t crct = 0;
  uint16_t ltt = 0;

  for (ltt = 0; ltt < lens; ltt++)
  {
    crct = crcc(*(str6 + ltt), crct);
  }

  return crct;
}
void LLS_Parsing(uint8_t b)
{
  switch (config.pars_cnt)
  {
    case PREFIX:
      if (b == PREF_IN)
      {
        config.pars_cnt = NETNUM;
        config.crc_in = 0;
      }
      break;
    case NETNUM:
      if (b == OPROS_NET)
        config.pars_cnt = OPROS_NET_STATE;
      else
      {
        if ((b == kalibr.net_num)
            || ((b == kalibr.net_num + 1) && (kalibr.peak_num > 1))
            || ((b == kalibr.net_num + 2) && (kalibr.peak_num > 2))
            || ((b == kalibr.net_num + 3) && (kalibr.peak_num > 3))
            || ((b == kalibr.net_num + 4) && (kalibr.peak_num > 4)))
        {
          config.addr = b - kalibr.net_num;
          config.pars_cnt = LLS_CMD;
        }
        else
          config.pars_cnt = PREFIX;
      }
      break;
    case OPROS_NET_STATE:
      if (b == config.crc_in)
      {
        memset(rs485.buf_out, 0, 9);
        rs485.buf_out[0] = kalibr.net_num;
        for (uint8_t cc = 1; cc < kalibr.peak_num; cc++)
          rs485.buf_out[cc] = kalibr.net_num + cc;
        pusk_485(rs485.buf_out, strlen((char*) rs485.buf_out));
      }
      break;
    case LLS_CMD:
      config.lls_cmd = b;
      config.data_cnt = 0;
      config.pars_cnt = LLS_CMD_PROCESSING;
      break;
    case LLS_CMD_PROCESSING:
      config.res = LLS_Cmd_Processing(b);
      config.data_cnt++;
      if (config.res == PROCESSING_FINISHED)
        config.pars_cnt = PREFIX;
      break;
    default:
      break;
  }
  config.crc_in = crcc(b, config.crc_in);
}
LLS_CMD_PROCESSING_RESULT LLS_Cmd_Processing(uint8_t n)
{
  LLS_CMD_PROCESSING_RESULT res;
  switch (config.lls_cmd)
  {
    case out_DATA:
      if (n == config.crc_in)
      {
        out_d();
        return PROCESSING_FINISHED;
      }
      break;
    case out_full:
      if (n == config.crc_in)
      {
        out_td();
        return PROCESSING_FINISHED;
      }
      break;
    case set_NET:
      if (!config.data_cnt)
        config.net_num = n;
      else if (n == config.crc_in)
      {
        kalibr.net_num = config.net_num;
        write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
        return PROCESSING_FINISHED;
      }
      break;
    case set_LO:
      switch (config.data_cnt)
      {
        case 0:
          config.start_m = n << 8;
          break;
        case 1:
          config.start_m += n;
          break;
        case 2:
          if (n == config.crc_in)
          {
            kalibr.start_m = 0.01 * (float) config.start_m;
            write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
            distance.status = DISTANCE_DETECTOR_NOT_ACTIVATED;
            return PROCESSING_FINISHED;
          }
          break;
        default:
          return PROCESSING_FINISHED;
          break;
      }
      break;
    case set_HI:
      switch (config.data_cnt)
      {
        case 0:
          config.length_m = n << 8;
          break;
        case 1:
          config.length_m += n;
          break;
        case 2:
          if (n == config.crc_in)
          {
            kalibr.length_m = 0.01 * (float) config.length_m;
            kalibr.length_m -= kalibr.start_m;
            if (kalibr.length_m > 0)
            {
              write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
              distance.status = DISTANCE_DETECTOR_NOT_ACTIVATED;
            }
            return PROCESSING_FINISHED;
          }
          break;
        default:
          return PROCESSING_FINISHED;
          break;
      }
      break;
    case get_assembly_result:
      if (n == config.crc_in)
      {
        memset(rs485.buf_out, 0, RS485_BUF_SIZE);
        rs485.buf_out[0] = pref_out;
        rs485.buf_out[1] = kalibr.net_num + config.addr;
        rs485.buf_out[2] = get_assembly_result;
        rs485.buf_out[3] = distance.assembly_result;
        rs485.buf_out[4] = crc_str((char*) rs485.buf_out, 4);
        pusk_485(rs485.buf_out, 5);
      }
      return PROCESSING_FINISHED;
      break;
    case envelope_ON:
      if (n == config.crc_in)
      {
        distance.envelope = 1;
        distance.status = DISTANCE_DETECTOR_NOT_ACTIVATED;
      }
      return PROCESSING_FINISHED;
      break;
    case envelope_OFF:
      if (n == config.crc_in)
      {
        distance.envelope = 0;
        distance.status = DISTANCE_DETECTOR_NOT_ACTIVATED;
      }
      return PROCESSING_FINISHED;
      break;
    case get_envelope_cmd:
      if (n == config.crc_in)
        distance.get_envelope = 1;
      return PROCESSING_FINISHED;
      break;
    case set_AVERAGE:
      if (config.data_cnt == 1)
      {
        config.average = n;
      }
      else if (config.data_cnt == 2)
      {
        if (n == config.crc_in)
        {
          kalibr.average = 0.1f * (float) config.average;
          write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
        }
        return PROCESSING_FINISHED;
      }
      break;
    case set_PR:
      switch (config.data_cnt)
      {
        case 0:
          config.threshold = n << 8;
          break;
        case 1:
          config.threshold += n;
          break;
        case 2:
          if (n == config.crc_in)
          {
            kalibr.threshold = config.threshold;
            write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
            distance.status = DISTANCE_DETECTOR_NOT_ACTIVATED;
            return PROCESSING_FINISHED;
          }
          break;
        default:
          return PROCESSING_FINISHED;
          break;
      }
      break;
    case set_SETTINGS:
      res = LLS_Set_Settings(n);
      return res;
      break;
    case out_SER:
      if (n == config.crc_in)
      {
        rs485.buf_out[0] = pref_out;
        rs485.buf_out[1] = kalibr.net_num + config.addr;
        rs485.buf_out[2] = out_SER;
        rs485.buf_out[3] = (uint8_t) kalibr.sn;
        rs485.buf_out[4] = (uint8_t) (kalibr.sn >> 8);
        rs485.buf_out[5] = (uint8_t) (kalibr.sn >> 16);
        rs485.buf_out[6] = (uint8_t) (kalibr.sn >> 24);
        rs485.buf_out[7] = (uint8_t) VERSION;
        rs485.buf_out[8] = (uint8_t) (VERSION >> 8);
        rs485.buf_out[9] = 0;
        rs485.buf_out[10] = 0;
        rs485.buf_out[11] = crc_str((char*) rs485.buf_out, 11);
        pusk_485(rs485.buf_out, 12);
      }
      return PROCESSING_FINISHED;
      break;
    case set_SER:
      if (config.data_cnt < 4)
        config.sn = (config.sn << 8) + n;
      else if (config.data_cnt == 4)
      {
        kalibr.sn=config.sn;
        write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
        return PROCESSING_FINISHED;
      }
      break;
    default:
      return PROCESSING_FINISHED;
      break;
  }
  return PROCESSING_GOING_ON;
}
LLS_CMD_PROCESSING_RESULT LLS_Set_Settings(uint8_t c)
{
  switch (config.data_cnt)
  {
    case 0:
      config.auto_treshold = c;
      break;
    case 1:
      config.profile = c;
      break;
    case 2:
      config.auto_profile = c;
      break;
    case 3:
      config.meas_freq = (uint16_t) c;
      break;
    case 4:
      config.meas_freq |= ((uint16_t) c) << 8;
      break;
    case 5:
      config.peaks = c;
      break;
    case 6:
      config.peak_sorting = c;
      break;
    case 7:
      config.sweep_averaging = c;
      break;
    case 8:
      config.threshold_type = c;
      break;
    case 9:
      config.guard = c;
      break;
    case 10:
      config.window = c;
      break;
    case 11:
      config.threshold_sensitivity = c;
      break;
    case 12:
      config.hw_acc_samples = c;
      break;
    case 13:
      if (c == config.crc_in)
      {
        kalibr.auto_treshold = config.auto_treshold;
        kalibr.profile = config.profile;
        kalibr.auto_profile = config.auto_profile;
        kalibr.meas_freq = config.meas_freq;
        kalibr.peak_num = config.peaks;
        kalibr.peak_sorting = config.peak_sorting;
        kalibr.sweep_averaging = config.sweep_averaging;
        kalibr.guard = config.guard;
        kalibr.window = config.window;
        kalibr.threshold_sensitivity = config.threshold_sensitivity;
        kalibr.threshold_type = config.threshold_type;
        kalibr.hw_acc_samples = config.hw_acc_samples;
        write_variables((char*) &kalibr, sizeof(kalibr), KALIBR_ADDR);
        distance.status = DISTANCE_DETECTOR_NOT_ACTIVATED;
        return PROCESSING_FINISHED;
      }
      break;
    default:
      return PROCESSING_FINISHED;
      break;
  }
  return PROCESSING_GOING_ON;
}
void out_d(void)
{
  uint16_t dist = 0, ampl = 0;
  dist = distance.peak[0];
  ampl = distance.amplitude[0];
  memset(rs485.buf_out, 0, RS485_BUF_SIZE);
  rs485.buf_out[0] = pref_out;
  rs485.buf_out[1] = kalibr.net_num + config.addr;
  rs485.buf_out[2] = out_DATA;
  rs485.buf_out[3] = (uint8_t) 25; //temperature
  rs485.buf_out[4] = (uint8_t) (dist);
  rs485.buf_out[5] = (uint8_t) (dist >> 8);
  rs485.buf_out[6] = (uint8_t) (ampl);
  rs485.buf_out[7] = (uint8_t) (ampl >> 8);
  rs485.buf_out[8] = crc_str((char*) rs485.buf_out, 8);
  pusk_485(rs485.buf_out, 9);
}
void out_td(void)
{
  uint16_t dist = 0, ampl = 0, t = 0;
  dist = distance.peak[0];
  ampl = distance.amplitude[0];
  memset(rs485.buf_out, 0, RS485_BUF_SIZE);
  rs485.buf_out[0] = pref_out;
  rs485.buf_out[1] = kalibr.net_num + config.addr;
  rs485.buf_out[2] = out_full;
  rs485.buf_out[3] = (uint8_t) 25; //temperature
  rs485.buf_out[4] = (uint8_t) (dist);
  rs485.buf_out[5] = (uint8_t) (dist >> 8);
  rs485.buf_out[6] = 0;
  rs485.buf_out[7] = 0;
  rs485.buf_out[8] = (uint8_t) (ampl);
  rs485.buf_out[9] = (uint8_t) (ampl >> 8);

  t = (uint16_t) (kalibr.start_m / 10);
  rs485.buf_out[10] = 0;
  rs485.buf_out[11] = 0;
  rs485.buf_out[12] = (uint8_t) (t);
  rs485.buf_out[13] = (uint8_t) (t >> 8);

  t = (uint16_t) ((kalibr.start_m + kalibr.length_m) / 10);
  rs485.buf_out[14] = 0;
  rs485.buf_out[15] = 0;
  rs485.buf_out[16] = (uint8_t) (t);
  rs485.buf_out[17] = (uint8_t) (t >> 8);

  t = (uint16_t) (kalibr.average);
  rs485.buf_out[18] = (uint8_t) (t);
  rs485.buf_out[19] = (uint8_t) (t >> 8);

  rs485.buf_out[20] = 0;

  t = (uint16_t) (kalibr.threshold);
  rs485.buf_out[21] = (uint8_t) (t);
  rs485.buf_out[22] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.peak[1]);
  rs485.buf_out[23] = (uint8_t) (t);
  rs485.buf_out[24] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.amplitude[1]);
  rs485.buf_out[25] = 0;
  rs485.buf_out[26] = 0;
  rs485.buf_out[27] = (uint8_t) (t);
  rs485.buf_out[28] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.peak[2]);
  rs485.buf_out[29] = (uint8_t) (t);
  rs485.buf_out[30] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.amplitude[2]);
  rs485.buf_out[31] = 0;
  rs485.buf_out[32] = 0;
  rs485.buf_out[33] = (uint8_t) (t);
  rs485.buf_out[34] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.peak[3]);
  rs485.buf_out[35] = (uint8_t) (t);
  rs485.buf_out[36] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.amplitude[3]);
  rs485.buf_out[37] = 0;
  rs485.buf_out[38] = 0;
  rs485.buf_out[39] = (uint8_t) (t);
  rs485.buf_out[40] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.peak[4]);
  rs485.buf_out[41] = (uint8_t) (t);
  rs485.buf_out[42] = (uint8_t) (t >> 8);

  t = (uint16_t) (distance.amplitude[4]);
  rs485.buf_out[43] = 0;
  rs485.buf_out[44] = 0;
  rs485.buf_out[45] = (uint8_t) (t);
  rs485.buf_out[46] = (uint8_t) (t >> 8);

    t = (uint16_t) (100*distance.max_freq);
  rs485.buf_out[47] = (uint8_t) (t);
  rs485.buf_out[48] = (uint8_t) (t >> 8);

  t = (uint16_t) (100 * kalibr.meas_freq);
  rs485.buf_out[49] = (uint8_t) (t);
  rs485.buf_out[50] = (uint8_t) (t >> 8);

  rs485.buf_out[51] = (uint8_t) (kalibr.profile);
  rs485.buf_out[51] = (uint8_t) (kalibr.auto_treshold);
  rs485.buf_out[52] = (uint8_t) (kalibr.auto_profile);
  rs485.buf_out[53] = (uint8_t) (kalibr.peak_num);
  rs485.buf_out[54] = (uint8_t) (kalibr.peak_sorting);
  rs485.buf_out[55] = (uint8_t) (kalibr.sweep_averaging);
  rs485.buf_out[56] = (uint8_t) (kalibr.threshold_type);
  rs485.buf_out[57] = (uint8_t) (kalibr.guard);
  rs485.buf_out[58] = (uint8_t) (kalibr.window);
  rs485.buf_out[59] = (uint8_t) (kalibr.threshold_sensitivity);
  rs485.buf_out[60] = (uint8_t) (distance.status);
  rs485.buf_out[61] = (uint8_t) (distance.get_envelope);
  rs485.buf_out[62] = (uint8_t) (kalibr.hw_acc_samples);

  rs485.buf_out[63] = crc_str((char*) rs485.buf_out, 63);
  pusk_485(rs485.buf_out, 64);
}
