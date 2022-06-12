/*
 * user.h
 *
 *  Created on: Jun 11, 2022
 *      Author: kon
 */

#ifndef INC_USER_H_
#define INC_USER_H_

#include <stdio.h>

#include "stm32l476g_discovery_glass_lcd.h"



extern volatile uint8_t enc_state;



#define MAX_RPM 1200000.0
#define MIN_ENC_T 20
#define ENC_BT_Pin GPIO_PIN_0
#define ENC_BT_GPIO_Port GPIOD
#define ENC_2_Pin GPIO_PIN_8
#define ENC_2_GPIO_Port GPIOE
#define ENC_1_Pin GPIO_PIN_2
#define ENC_1_GPIO_Port GPIOB



enum enc_event { NOP, BUTTON, LEFT, RIGHT};



int menu( const char* entries[], const unsigned entries_no );
void lcd_put_v( float voltage );
void lcd_put_a( float curr );
void lcd_put_nm( float torq );
void lcd_put_speed( float rpm );
void BSP_LCD_GLASS_ExitableScrollSentence(uint8_t *ptr, uint16_t nScroll, uint16_t ScrollSpeed);



#endif /* INC_USER_H_ */
