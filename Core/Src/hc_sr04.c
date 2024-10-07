//Drivers para sensor ultrasonico
#include "tim.h"
#include "hc_sr04.h"

uint32_t t_ini = 0;
uint32_t t_end = 0;
uint32_t t_time = 0;
uint16_t dist = 0;
uint8_t flag_captured = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(flag_captured == 0)
		{
			t_ini = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			flag_captured = 1;
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if(flag_captured == 1)
		{
			t_end = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			__HAL_TIM_SetCounter(htim, 0);

			if(t_end > t_ini){
				t_time = t_end - t_ini;
			}
			else if(t_ini > t_end){
				t_time = (0xFFFF - t_ini) + t_end;
			}

			dist = (uint16_t)((t_time*0.034)/2);
			flag_captured = 0;

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void HCSR04_Init(void)
{
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
}

uint16_t HCSR04_Get_Distance(void)
{
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_SET);
	__HAL_TIM_SetCounter(&htim1, 0);
	while (__HAL_TIM_GetCounter(&htim1) < 10);
	HAL_GPIO_WritePin(Trigger_GPIO_Port, Trigger_Pin, GPIO_PIN_RESET);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	return dist;
}

