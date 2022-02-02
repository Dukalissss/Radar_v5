/*
 * configuration.h
 *
 *  Created on: Jan 31, 2022
 *      Author: Dukalis
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#define VERSION                     500

#define KALIBR_ADDR                 0x0801F800

#define PREF_IN                     0x31
#define OPROS_NET                   0xFF
#define out_DATA                    0x06
#define pref_out                    0x3E
#define out_full                    0xF0
#define set_NET                     0xFB
#define set_LO                      0xFA
#define set_HI                      0xF5
#define get_assembly_result         0xE5
#define envelope_ON                 0xE2
#define envelope_OFF                0xE3
#define get_envelope_cmd            0xE4
#define set_AVERAGE                 0xE9
#define set_PR                      0xEA
#define set_SETTINGS                0xE0
#define out_SER                     0xFC
#define set_SER                     0xF6



typedef struct
{
  uint8_t                         net_num;
  uint32_t                        sn;
  acc_service_profile_t           profile;
  float                           start_m;
  float                           length_m;
  uint16_t                        threshold;
  float                           average;
  uint8_t                         threshold_type;
  float                           guard;
  float                           window;
  float                           threshold_sensitivity;
  uint8_t                         cfar_threshold_only_lower_distance;
  uint8_t                         peak_num;
  uint8_t                         peak_sorting;
  uint8_t                         sweep_averaging;
  uint8_t                         hw_acc_samples;
  float                           gain;
  float                           meas_freq;
  uint8_t                         auto_treshold;
  uint8_t                         auto_profile;
  uint8_t                         crc8;
} KALIBR_STRUCT;

extern KALIBR_STRUCT kalibr;


typedef enum
{
  CONFIGURATION_DOESNT_LOADED=0,
  CONFIGURATION_READY
} CONFIGURATION_STATUS;

typedef enum
{
  PREFIX=0,
  NETNUM,
  LLS_CMD,
  LLS_CMD_PROCESSING,
  OPROS_NET_STATE
} LLS_ENUM;

typedef enum
{
  PROCESSING_FINISHED=0,
  PROCESSING_GOING_ON
} LLS_CMD_PROCESSING_RESULT;


typedef struct
{
  CONFIGURATION_STATUS      status;
  uint16_t                  pars_index;
  LLS_ENUM                  pars_cnt;
  uint8_t                   crc_in;
  uint8_t                   addr;
  uint8_t                   lls_cmd;
  LLS_CMD_PROCESSING_RESULT res;
  uint8_t                   data_cnt;
  uint8_t                   net_num;
  uint16_t                  start_m;
  uint16_t                  length_m;
  uint8_t                   average;
  uint16_t                  threshold;
  uint8_t                   auto_treshold;
  uint8_t                   profile;
  uint8_t                   auto_profile;
  uint16_t                  meas_freq;
  uint8_t                   peaks;
  uint8_t                   peak_sorting;
  uint8_t                   sweep_averaging;
  uint8_t                   threshold_type;
  uint8_t                   guard;
  uint8_t                   window;
  uint8_t                   threshold_sensitivity;
  uint8_t                   hw_acc_samples;
  uint32_t                  sn;

} CONFIGURATION_STRUCT;

extern CONFIGURATION_STRUCT config;


void Configurator_Serv(void);
void Read_Configuration(void);
char crc_str(char *str6, int lens);
char crcc(char dat_crc, char crc8);
void LLS_Parsing(uint8_t b);
LLS_CMD_PROCESSING_RESULT LLS_Cmd_Processing(uint8_t cmd);
void out_d(void);
void out_td(void);
LLS_CMD_PROCESSING_RESULT LLS_Set_Settings(uint8_t c);



#endif /* INC_CONFIGURATION_H_ */
