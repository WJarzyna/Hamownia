/*
 * data_acq.c
 *
 *  Created on: Jun 11, 2022
 *      Author: kon
 */

#include <data_acq.h>

HAL_StatusTypeDef init_data_acq()
{
	if( HAL_TIM_Base_Start_IT(&htim4) != HAL_OK ) return HAL_ERROR;
	if( HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1) != HAL_OK ) return HAL_ERROR;

	if( HAL_TIM_Base_Start_IT(&htim16) != HAL_OK ) return HAL_ERROR;
	if( HAL_TIM_Base_Start_IT(&htim15) != HAL_OK ) return HAL_ERROR;

	if( HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK ) return HAL_ERROR;
	if( HAL_ADC_Start_DMA(&hadc1, adc_raw, N_SAMPLES*4) != HAL_OK ) return HAL_ERROR;

	HAL_Delay(200);

	ADC_get_zero_points();
	return HAL_OK;
}

void HAL_GPIO_EXTI_Callback( uint16_t pin )
{
	static uint8_t state, butt_state;
	static uint32_t t;

	if( t - HAL_GetTick() > MIN_ENC_T )
	{
		if( pin == GPIO_PIN_0 )
		{
			if( !HAL_GPIO_ReadPin(ENC_BT_GPIO_Port, ENC_BT_Pin) )
			{
				if( butt_state == 1 ) enc_state = BUTTON;
				butt_state = 0;
			}
			else butt_state = 1;
		}
		else
		{
			state &= 0x0F;
			state = HAL_GPIO_ReadPin(ENC_1_GPIO_Port, ENC_1_Pin) \
					+ (HAL_GPIO_ReadPin(ENC_2_GPIO_Port, ENC_2_Pin) << 1) \
					+ (state << 4);

			switch( state )
			{
			case 0x10: enc_state = LEFT; break;
			case 0x20: enc_state = RIGHT; break;
			default: break;
			}
		}
	}
	t = HAL_GetTick();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t aux;
	if( htim == &htim4 )
	{
		aux = HAL_TIM_ReadCapturedValue( htim, TIM_CHANNEL_1);

		if( aux > 240 )
		{
			pulse_t = aux;
			__HAL_TIM_SET_COUNTER( htim, 0);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim )
{
	static unsigned i = 0;
	if( htim == &htim4 )
	{
		pulse_t = 0;
	}
	else if( htim == &htim16 )
	{
		send_data( i++, SPEED, TORQUE, ROTOR_CURR, STATOR_CURR, STATOR_V);
		i %= 7000;
	}
	else if( htim == &htim15 ) moving_avg();
}

void ADC_get_zero_points()
{
	for( unsigned i = 0; i < 4; ++i )
	{
		adc_zero[i] = 0;

		for( unsigned j = 0; j < 1000; ++j )
		{
			HAL_Delay(1);
			adc_zero[i] += adc_data[i];
		}
		adc_zero[i] /= 1000;

		lcd_put_v( 3.3*((float)adc_zero[i]/4096.0) );
	}
}

void moving_avg()
{
	uint32_t aux_data[4] = {};
	for( unsigned i = 0; i < N_SAMPLES*4; i+=4)
	{
		aux_data[0] += adc_raw[i];
		aux_data[1] += adc_raw[i+1];
		aux_data[2] += adc_raw[i+2];
		aux_data[3] += adc_raw[i+3];
	}
	adc_data[0] = aux_data[0]/N_SAMPLES;
	adc_data[1] = aux_data[1]/N_SAMPLES;
	adc_data[2] = aux_data[2]/N_SAMPLES;
	adc_data[3] = aux_data[3]/N_SAMPLES;
}
