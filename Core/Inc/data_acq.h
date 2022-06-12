/*
 * data_acq.h
 *
 *  Created on: Jun 11, 2022
 *      Author: kon
 */

#ifndef INC_DATA_ACQ_H_
#define INC_DATA_ACQ_H_

#include <user.h>



enum adc_ch { I_STATOR, TORQ_SENS, V_OUT, I_ROTOR, PWM_ROTOR, SPEED_IN, AUTO_RUN, FILE_RUN, SEND_FILE, CLEAR_FLASH};



#define SPEED (pulse_t!=0?MAX_RPM/pulse_t:0)
#define ROTOR_CURR (-((float)adc_data[I_ROTOR]-(float)adc_zero[I_ROTOR])/271.75)
#define STATOR_CURR (-((float)adc_data[I_STATOR]-(float)adc_zero[I_STATOR])/49.636)
#define STATOR_V ((float)adc_data[V_OUT]-(float)adc_zero[V_OUT])/223.77
#define TORQUE ((float)adc_data[TORQ_SENS]-(float)adc_zero[TORQ_SENS])/399.022
#define N_SAMPLES 500



extern volatile uint32_t adc_data[4];
extern volatile uint32_t adc_zero[4];
extern volatile uint32_t pulse_t;
extern volatile uint32_t adc_raw[N_SAMPLES*4];

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern ADC_HandleTypeDef hadc1;



void send_data( unsigned time, unsigned speed, float torq, float i_r, float i_s, float v_s );


HAL_StatusTypeDef init_data_acq();
void HAL_GPIO_EXTI_Callback( uint16_t pin );
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim );
void ADC_get_zero_points();
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *hadc );

#endif /* INC_DATA_ACQ_H_ */
