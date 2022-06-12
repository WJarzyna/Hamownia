/*
 * logic.h
 *
 *  Created on: Jun 12, 2022
 *      Author: kon
 */

#ifndef INC_LOGIC_H_
#define INC_LOGIC_H_

#include <stdio.h>
#include <string.h>
#include <user.h>
#include <data_acq.h>
#include <quadspi.h>

#include "main.h"
#include "stm32l476g_discovery_glass_lcd.h"


extern ADC_HandleTypeDef hadc1;
extern QSPI_HandleTypeDef hqspi;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
extern volatile uint32_t adc_data[4];
extern volatile uint32_t adc_raw[N_SAMPLES*4];
extern volatile uint32_t adc_zero[4];
extern volatile uint8_t enc_state;
extern volatile uint32_t pulse_t;




uint32_t crc32b( const unsigned char* message);
void send_data( unsigned time, unsigned speed, float torq, float i_r, float i_s, float v_s );
void manual_PWM( void );
void auto_run();
void auto_run_file();
void send_file();
void destroy_all_evidence();
uint32_t find_file( uint32_t file_id );

#endif /* INC_LOGIC_H_ */
