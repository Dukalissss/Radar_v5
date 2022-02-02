/*
 * distance_detector.h
 *
 *  Created on: Jan 31, 2022
 *      Author: Dukalis
 */

#ifndef INC_DISTANCE_DETECTOR_H_
#define INC_DISTANCE_DETECTOR_H_


#define RADAR_SYNC_TIMER        TIM15
#define PEAK_NUMBER             5

typedef enum
{
  DISTANCE_DETECTOR_NOT_ACTIVATED=0,
  DISTANCE_DETECTOR_READY
} DETECTOR_STATUS;

typedef struct
{
  DETECTOR_STATUS                 status;
  uint8_t                         start_measure;
  uint16_t                        peak[PEAK_NUMBER];
  uint16_t                        amplitude[PEAK_NUMBER];
  uint8_t                         new_data_available;
  uint8_t                         assembly_result;
  uint8_t                         envelope;
  uint8_t                         get_envelope;
  uint16_t                        meas_period;
  uint16_t                        meas_time;
  float                           max_freq;
} DISTANCE_DETECTOR_STRUCT;

extern DISTANCE_DETECTOR_STRUCT distance;

void Distance_Detector_Serv(void);
void Distance_Detector_Init(void);
void Radar_Sync_Timer_Handler(void);
void Measure_Distance(void);
void Envelope_Data_Ready_Callback(const uint16_t *data, uint16_t data_length);



#endif /* INC_DISTANCE_DETECTOR_H_ */
