/*
 * SubSys_USART_ReceiveIT_CallBacks_Driver.h
 *
 *  Created on: Aug 17, 2024
 *      Author: yunus
 */

#ifndef INC_SUBSYS_USART_RECEIVEIT_CALLBACKS_DRIVER_H_
#define INC_SUBSYS_USART_RECEIVEIT_CALLBACKS_DRIVER_H_

#include "main.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_SUBSYS_USART_RECEIVEIT_CALLBACKS_DRIVER_H_ */
