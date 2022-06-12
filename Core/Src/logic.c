/*
 * logic.c
 *
 *  Created on: Jun 12, 2022
 *      Author: kon
 */

#include "logic.h"

uint32_t crc32b( const unsigned char* message)
{
   int i, j;
   unsigned int byte, crc, mask;

   i = 0;
   crc = 0xFFFFFFFF;
   while (message[i] != 0)
   {
      byte = message[i];
      crc = crc ^ byte;
      for (j = 7; j >= 0; j--)
      {
         mask = -(crc & 1);
         crc = (crc >> 1) ^ (0xEDB88320 & mask);
      }
      i = i + 1;
   }
   return ~crc;
}

void send_data( unsigned time, unsigned speed, float torq, float i_r, float i_s, float v_s )
{
	static uint8_t out_msg[256];
	uint32_t crc, out_msg_len;

	out_msg_len = sprintf( (char*)out_msg, "0:%u,%u,%1.3f,%1.3f,%1.3f,%1.3f:", time, speed, torq, i_r, i_s, v_s );
	crc = crc32b(out_msg);
	out_msg_len = sprintf( (char*)out_msg, "%s%lu\n", out_msg, crc );
	HAL_UART_Transmit( &huart2, out_msg, out_msg_len, 100);
}

void manual_PWM( void )
{
	uint8_t text[8];
	static int32_t pwm = 0;

	switch( enc_state )
	{
	case LEFT: ++pwm; break;
	case RIGHT: --pwm; break;
	default: break;
	}
	enc_state = NOP;

	if( pwm < 0 ) pwm = 0;
	pwm %= 101;
	__HAL_TIM_SET_COMPARE( &htim17, TIM_CHANNEL_1, pwm*50 );

	sprintf( (char*)text, "%3lu", pwm );
	BSP_LCD_GLASS_DisplayString(text);
}

void auto_run()
{
	uint8_t text[8];
	uint32_t t0 = HAL_GetTick();

	HAL_TIM_Base_Stop_IT(&htim16);

	for( unsigned pwm = 0; pwm < 101 && enc_state != BUTTON; ++pwm)
	{
		__HAL_TIM_SET_COMPARE( &htim17, TIM_CHANNEL_1, pwm*50 );
		sprintf( (char*)text, "%3u", pwm );
		BSP_LCD_GLASS_DisplayString(text);
		HAL_Delay(50);
		send_data( HAL_GetTick()-t0, SPEED, TORQUE, ROTOR_CURR, STATOR_CURR, STATOR_V );
	}

	__HAL_TIM_SET_COMPARE( &htim17, TIM_CHANNEL_1, 0 );
	HAL_TIM_Base_Start_IT(&htim16);
	enc_state = BUTTON;
}

void auto_run_file()
{
	uint8_t file_head[5] = {};
	static uint8_t mem_line[256];
	uint32_t mem_line_len = 0, file_ptr, file_id = 0, data_ptr = 0;
	uint8_t text[10];
	uint32_t t0 = HAL_GetTick(), time;

	for( uint32_t tmp_ptr = 0; tmp_ptr != UINT32_MAX; ++file_id )
	{
		file_ptr = tmp_ptr;
		tmp_ptr = find_file( file_id );
	}
	--file_id;

	sprintf( (char*)text, " Plik %lu ", file_id );
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)text, 1, 100);

	if( CSP_QSPI_Read(file_head, file_ptr, 5) != HAL_OK ) Error_Handler();
	file_ptr = (file_head[1]<<24) + (file_head[2]<<16) + (file_head[3]<<8) + file_head[4];
	if( file_ptr == UINT32_MAX ) file_ptr = 0;
	data_ptr = file_ptr + 5;

	BSP_LCD_GLASS_Clear();

	HAL_TIM_Base_Stop_IT(&htim16);

	for( unsigned pwm = 0; pwm < 101 && enc_state != BUTTON; ++pwm)
	{
		__HAL_TIM_SET_COMPARE( &htim17, TIM_CHANNEL_1, pwm*50 );
		sprintf( (char*)text, "%3u", pwm );
		BSP_LCD_GLASS_DisplayString(text);
		HAL_Delay(50);
		time = HAL_GetTick()-t0;
		send_data( time, SPEED, TORQUE, ROTOR_CURR, STATOR_CURR, STATOR_V );

		mem_line_len = sprintf( (char*)mem_line, "%lu,%u,%1.3f,%1.3f,%1.3f,%1.3f\n",\
				time, (unsigned)SPEED, TORQUE, ROTOR_CURR, STATOR_CURR, STATOR_V );

		if( CSP_QSPI_Write( mem_line, data_ptr, mem_line_len) != HAL_OK ) Error_Handler();
		data_ptr += mem_line_len;
	}

	file_head[0] = file_id;
	file_head[1] = (data_ptr>>24) & 0xFF;
	file_head[2] = (data_ptr>>16) & 0xFF;
	file_head[3] = (data_ptr>>8) & 0xFF;
	file_head[4] = data_ptr & 0xFF;
	if( CSP_QSPI_Write( file_head, file_ptr, 5) != HAL_OK ) Error_Handler();

	__HAL_TIM_SET_COMPARE( &htim17, TIM_CHANNEL_1, 0 );
	HAL_TIM_Base_Start_IT(&htim16);
	enc_state = BUTTON;
}

void send_file()
{
	uint8_t file_head[5];
	uint8_t text[8];
	unsigned flag = 0;
	int selection = 0, prev_sel = 1;
	uint32_t file_ptr = 0, end_ptr = 0;

	if( find_file( 0 ) == UINT32_MAX )
	{
		BSP_LCD_GLASS_ScrollSentence((uint8_t*)" Brak plikow ", 1, 100);
		enc_state = BUTTON;
		return;
	}

	BSP_LCD_GLASS_ScrollSentence((uint8_t*)" Wybierz plik do wyslania ", 1, 100);

	BSP_LCD_GLASS_Clear();

	while( !flag )
	{
		sprintf( (char*)text, "%d", selection);
		BSP_LCD_GLASS_DisplayString( text );

		switch( enc_state )
		{
		case NOP: break;
		case BUTTON: flag = 1; break;
		case LEFT: --selection; break;
		case RIGHT: ++selection; break;
		default: break;
		}
		selection = selection < 0 ? 0 : selection;
		selection %= 255;
		enc_state = NOP;

		if( selection != prev_sel ) file_ptr = find_file( (unsigned)selection );
		if( file_ptr == UINT32_MAX ) selection = prev_sel;
		prev_sel = selection;
	}

	uint8_t buf;
	if( CSP_QSPI_Read(file_head, file_ptr, 5) != HAL_OK ) Error_Handler();
	end_ptr = (file_head[1]<<24) + (file_head[2]<<16) + (file_head[3]<<8) + file_head[4];

	for( file_ptr +=5; file_ptr < end_ptr; ++file_ptr )
	{
		if( CSP_QSPI_Read(&buf, file_ptr, 1) != HAL_OK ) Error_Handler();
		HAL_UART_Transmit(&huart2, &buf, 1, 5);
	}

	enc_state = BUTTON;
}

void destroy_all_evidence()
{
	BSP_LCD_GLASS_ScrollSentence((uint8_t*)" Na pewno wyczyscic pamiec? ", 1, 100);
	while( !enc_state );
	if( enc_state == BUTTON )
	{
		BSP_LCD_GLASS_ScrollSentence((uint8_t*)" Czyszczenie pamieci ", 1, 100);
		if( CSP_QSPI_Erase_Chip() != HAL_OK ) Error_Handler();
		BSP_LCD_GLASS_ScrollSentence((uint8_t*)" Pamiec wyczyszczona ", 1, 100);
	}
	else BSP_LCD_GLASS_ScrollSentence((uint8_t*)" Anulowano ", 1, 100);
	HAL_Delay(1000);
	enc_state = BUTTON;
}

uint32_t find_file( uint32_t file_id )
{
	uint8_t file_head[5] = {};
	uint32_t ptr = 0;

	do
	{
		ptr = (file_head[1]<<24) + (file_head[2]<<16) + (file_head[3]<<8) + file_head[4];
		if( CSP_QSPI_Read(file_head, ptr, 5) != HAL_OK ) Error_Handler();

	} while( file_head[0] != file_id && file_head[0] != 255 );

	return file_head[0] == 255 ? UINT32_MAX : ptr;
}
