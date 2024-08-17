/*
 * SubSys_USART_ReceiveIT_CallBacks_Driver.h
 *
 *  Created on: Aug 17, 2024
 *      Author: yunus
 */

#ifndef INC_SUBSYS_USART_RECEIVEIT_CALLBACKS_DRIVER_H_
#define INC_SUBSYS_USART_RECEIVEIT_CALLBACKS_DRIVER_H_

#include "main.h"
#include "SubSys_WirelessCommunication_Telemetry_Driver.h"

extern SubSys_WirelessCom_APP_HandleTypeDef dev_WirelessComApp;

extern char UsbTTL2EmbeddedGS[9];
extern char EmbeddedGS2Payload[30];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif /* INC_SUBSYS_USART_RECEIVEIT_CALLBACKS_DRIVER_H_ */
