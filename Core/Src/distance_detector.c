/*
 * distance_detector.c
 *
 *  Created on: Jan 31, 2022
 *      Author: Dukalis
 */
#include "main.h"
acc_detector_distance_handle_t distance_handle;
DISTANCE_DETECTOR_STRUCT distance =
{ 0 };
bool success = true;
acc_detector_distance_result_t result[PEAK_NUMBER];
acc_detector_distance_result_info_t result_info;

void Distance_Detector_Serv(void)
{
  if (distance.status == DISTANCE_DETECTOR_NOT_ACTIVATED)
    Distance_Detector_Init();
  if (distance.start_measure)
    Measure_Distance();
}
void Measure_Distance(void)
{
  distance.start_measure = 0;
  if (distance.status != DISTANCE_DETECTOR_READY)
    return;
  distance.meas_period = Toc1();
  Tic1();
  Tic2();
  success = acc_detector_distance_get_next(distance_handle, result,
      kalibr.peak_num, &result_info);
  distance.meas_time = Toc2();
  if (success)
  {
    for (uint16_t i = 0; i < result_info.number_of_peaks; i++)
    {
      distance.peak[i] = (uint16_t) (result[i].distance_m * 1000);
      distance.amplitude[i] = (uint16_t) result[i].amplitude;
    }
    distance.new_data_available = 1;
  }
  distance.max_freq = 1000.0f / (float) (distance.meas_time + 15);
}
void Distance_Detector_Init(void)
{
  LL_TIM_DisableIT_UPDATE(RADAR_SYNC_TIMER);
  LL_TIM_DisableCounter(RADAR_SYNC_TIMER);
  const acc_hal_t *hal = acc_hal_integration_get_implementation();

  if (!acc_rss_activate(hal))
    return;

  acc_detector_distance_configuration_t distance_configuration =
      acc_detector_distance_configuration_create();

  if (distance_configuration == NULL)
  {
    acc_rss_deactivate();
    return;
  }

  acc_detector_distance_configuration_power_save_mode_set(
      distance_configuration, ACC_POWER_SAVE_MODE_SLEEP);
  acc_detector_distance_configuration_requested_start_set(
      distance_configuration, kalibr.start_m);
  acc_detector_distance_configuration_requested_length_set(
      distance_configuration, kalibr.length_m);
  acc_detector_distance_configuration_service_profile_set(
      distance_configuration, kalibr.profile);
  acc_detector_distance_configuration_downsampling_factor_set(
      distance_configuration, 4);
  acc_detector_distance_configuration_sweep_averaging_set(
      distance_configuration, kalibr.sweep_averaging);
  acc_detector_distance_configuration_threshold_type_set(distance_configuration,
      kalibr.threshold_type);
  acc_detector_distance_configuration_cfar_threshold_guard_set(
      distance_configuration, kalibr.guard);
  acc_detector_distance_configuration_cfar_threshold_window_set(
      distance_configuration, kalibr.window);
  acc_detector_distance_configuration_cfar_threshold_only_lower_distance_set(
      distance_configuration, kalibr.cfar_threshold_only_lower_distance);
  acc_detector_distance_configuration_threshold_sensitivity_set(
      distance_configuration, kalibr.threshold_sensitivity);

  if (kalibr.threshold_type == ACC_DETECTOR_DISTANCE_THRESHOLD_TYPE_FIXED)
    acc_detector_distance_configuration_fixed_threshold_set(
        distance_configuration, kalibr.threshold);
  acc_detector_distance_configuration_peak_sorting_set(distance_configuration,
      kalibr.peak_sorting);
  acc_detector_distance_configuration_receiver_gain_set(distance_configuration,
      kalibr.gain);
  acc_detector_distance_configuration_hw_accelerated_average_samples_set(
      distance_configuration, kalibr.hw_acc_samples);
  acc_detector_distance_configuration_mur_set(distance_configuration,
      ACC_SERVICE_MUR_9);

  if (distance.get_envelope)
    acc_detector_distance_configuration_service_data_callback_set(
        distance_configuration, Envelope_Data_Ready_Callback);

  distance_handle = acc_detector_distance_create(distance_configuration);

  if (distance_handle == NULL)
  {
    acc_detector_distance_configuration_destroy(&distance_configuration);
    acc_rss_deactivate();
    return;
  }

  acc_detector_distance_configuration_destroy(&distance_configuration);

  if (!acc_detector_distance_activate(distance_handle))
  {
    acc_detector_distance_destroy(&distance_handle);
    acc_rss_deactivate();
    return;
  }

  LL_TIM_EnableIT_UPDATE(RADAR_SYNC_TIMER);
  LL_TIM_EnableCounter(RADAR_SYNC_TIMER);

  distance.status = DISTANCE_DETECTOR_READY;
}
void Radar_Sync_Timer_Handler(void)
{
  distance.start_measure = 1;
}
void Envelope_Data_Ready_Callback(const uint16_t *data, uint16_t data_length)
{
//  uint16_t data_l = 0, buf_out_index = 0, packet_num = 0;
//  uint8_t crc = 0;
//  if (flag_of.report)
//  {
//    flag_of.report = 0;
//    Delay_Millisec(250);
//    sprintf((char*) mb_struc.buf_ModBus_out, "Envelope data:");
//    //отправляем строку out_buf_485 на передачу
//    pusk_485(mb_struc.buf_ModBus_out, strlen(mb_struc.buf_ModBus_out));
//    Delay_Millisec(500);
//    while (data_length > data_l)
//    {
//      crc = 0;
//      buf_out_index = 0;
//      mb_struc.buf_ModBus_out[buf_out_index] = 0x3E;
//      crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//      mb_struc.buf_ModBus_out[buf_out_index] = 0x01;
//      crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//      mb_struc.buf_ModBus_out[buf_out_index] = 0xE4;
//      crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//      mb_struc.buf_ModBus_out[buf_out_index] = (uint8_t) (data_length >> 8);
//      crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//      mb_struc.buf_ModBus_out[buf_out_index] = (uint8_t) data_length;
//      crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//      mb_struc.buf_ModBus_out[buf_out_index] = (uint8_t) (packet_num >> 8);
//      crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//      mb_struc.buf_ModBus_out[buf_out_index] = (uint8_t) packet_num;
//      crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//      for (uint16_t ibah = 0; ibah < ENVELOPE_PACKET_SIZE; ibah++)
//      {
//        if (data_l < data_length)
//        {
//          mb_struc.buf_ModBus_out[buf_out_index] = data[data_l] >> 8;
//          crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//          mb_struc.buf_ModBus_out[buf_out_index] = data[data_l++];
//          crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//        }
//        else
//        {
//          mb_struc.buf_ModBus_out[buf_out_index] = 0;
//          crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//          mb_struc.buf_ModBus_out[buf_out_index] = 0;
//          crc = crcc(mb_struc.buf_ModBus_out[buf_out_index++], crc);
//          data_l++;
//        }
//      }
//      packet_num = data_l;
//      mb_struc.buf_ModBus_out[buf_out_index++] = crc;
//      //отправляем строку out_buf_485 на передачу
//      pusk_485(mb_struc.buf_ModBus_out, buf_out_index);
////          pusk_485(mb_struc.buf_ModBus_out, strlen(mb_struc.buf_ModBus_out));
//      while (!mb_struc.usart_ready)
//      {
//      };
//      Delay_Millisec(300);
//    }
//    sprintf(mb_struc.buf_ModBus_out, "\r\nEnd\r\n");
//    //отправляем строку out_buf_485 на передачу
//    pusk_485(mb_struc.buf_ModBus_out, strlen(mb_struc.buf_ModBus_out));
//  }
}
