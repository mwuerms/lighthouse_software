/**
 * Martin Egli
 * 2024-11-13
 * buttons for light house
 */

// - includes ------------------------------------------------------------------
#include <gpios.h>
#include "main.h"
#include "scheduler.h"

// - private functions ---------------------------------------------------------
/*Configure GPIO pins : BTN0_Pin BTN1_Pin BTN2_Pin BTN3_Pin
					   BTN4_Pin */
/*
GPIO_InitStruct.Pin = BTN0_Pin|BTN1_Pin|BTN2_Pin|BTN3_Pin
					  |BTN4_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if(GPIO_Pin == BTN0_Pin) {
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		scheduler_send_event(main_tid, MAIN_EV_BUTTON0, NULL);
	}
	if(GPIO_Pin == BTN1_Pin) {
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		scheduler_send_event(main_tid, MAIN_EV_BUTTON1, NULL);
	}
	if(GPIO_Pin == BTN2_Pin) {
		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
		scheduler_send_event(main_tid, MAIN_EV_BUTTON2, NULL);
	}
	if(GPIO_Pin == BTN3_Pin) {
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		scheduler_send_event(main_tid, MAIN_EV_BUTTON3, NULL);
	}
	if(GPIO_Pin == BTN4_Pin) {
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		scheduler_send_event(main_tid, MAIN_EV_BUTTON4, NULL);
	}

	if(GPIO_Pin == VBUS_SENSE_Pin) {
		if(HAL_GPIO_ReadPin(VBUS_SENSE_GPIO_Port, VBUS_SENSE_Pin) == GPIO_PIN_SET) {
			// VBUS plugged in
			scheduler_send_event(vbus_tid, VBUS_EV_PLUGGED_IN, NULL);
		}
		else  {
			// VBUS pulled out
			scheduler_send_event(vbus_tid, VBUS_EV_PULLED_OUT, NULL);
		}

	}
}

// - public functions ----------------------------------------------------------
void gpios_init(void) {

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	return;
}

void gpios_button_enable_irq(void) {
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
}

void gpios_button_disable_irq(void) {
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
}

void gpios_vbus_enable_irq(void) {
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void gpios_vbus_disable_irq(void) {
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}
