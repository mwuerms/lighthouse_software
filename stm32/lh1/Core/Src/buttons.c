/**
 * Martin Egli
 * 2024-11-13
 * buttons for light house
 */

// - includes ------------------------------------------------------------------
#include "main.h"
#include "buttons.h"
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
		scheduler_send_event(main_tid, MAIN_EV_BUTTON0, NULL);
	}
	if(GPIO_Pin == BTN1_Pin) {
		scheduler_send_event(main_tid, MAIN_EV_BUTTON1, NULL);
	}
	if(GPIO_Pin == BTN2_Pin) {
		scheduler_send_event(main_tid, MAIN_EV_BUTTON2, NULL);
	}
	if(GPIO_Pin == BTN3_Pin) {
		scheduler_send_event(main_tid, MAIN_EV_BUTTON3, NULL);
	}
	if(GPIO_Pin == BTN4_Pin) {
		scheduler_send_event(main_tid, MAIN_EV_BUTTON4, NULL);
	}

	if(GPIO_Pin == USR_BTN_Pin) {
		scheduler_send_event(main_tid, MAIN_EV_USR_BUTTON, NULL);
	}
}

// - public functions ----------------------------------------------------------
void buttons_init(void) {

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);


	return;
}

void buttons_enable_irq(void) {
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void buttons_disable_irq(void) {
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
}
