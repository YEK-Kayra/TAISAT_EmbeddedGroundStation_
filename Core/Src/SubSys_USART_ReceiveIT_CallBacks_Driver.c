#include "SubSys_USART_ReceiveIT_CallBacks_Driver.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	/*!
	 * The data coming from USART2 are the complete telemetry packets
	 * sent by the payload to the ground station.
	 */
	if(huart->Instance == USART2)
	{
		SubSys_WirelessCom_Telemetry_Receive_From_To(Sat_Payload, GroundStation, &dev_WirelessComApp);
	}

	/*!
	 * The data coming from USART1 are for the telecommand packets and
	 * the full telemetry packet that we need to receive&send to the ground station.
	 */

	if(huart->Instance == USART1)
	{

		/*! Ground Station PC want a telemetry packet from embedded station*/
		if(UsbTTL2EmbeddedGS[0] == '*')
		{
			HAL_UART_Transmit(&huart1, dev_WirelessComApp.Buffer.Rx, 200, 2000);
		}



		/*! Ground Station PC send a RHRH packet for color filtering*/
		if((UsbTTL2EmbeddedGS[3] !='?') && (UsbTTL2EmbeddedGS[4] !='?') && (UsbTTL2EmbeddedGS[5] !='?') && (UsbTTL2EmbeddedGS[5] !='6'))
		{
			dev_WirelessComApp.Variable.PAY_dataRHRH[0] = UsbTTL2EmbeddedGS[3];
			dev_WirelessComApp.Variable.PAY_dataRHRH[1] = UsbTTL2EmbeddedGS[4];
			dev_WirelessComApp.Variable.PAY_dataRHRH[2] = UsbTTL2EmbeddedGS[5];
			dev_WirelessComApp.Variable.PAY_dataRHRH[3] = UsbTTL2EmbeddedGS[6];
		}
		else
		{
			dev_WirelessComApp.Variable.PAY_dataRHRH[0] = '?';
			dev_WirelessComApp.Variable.PAY_dataRHRH[1] = '?';
			dev_WirelessComApp.Variable.PAY_dataRHRH[2] = '?';
			dev_WirelessComApp.Variable.PAY_dataRHRH[3] = '?';
		}



		/*! Ground Station PC send a separation command*/
		if(UsbTTL2EmbeddedGS[6] !='+')
		{
			dev_WirelessComApp.Variable.PAY_SeparationCommand = '+';
		}
		else
		{
			dev_WirelessComApp.Variable.PAY_SeparationCommand = '-';
		}


		/*! We need to send this packet to the Payload from Embedded ground station to Payload of Satellite */
		SubSys_WirelessCom_Telemetry_Transfer_From_To(GroundStationMcu, Sat_Payload, &dev_WirelessComApp);

		/*! Start listening to the ground station again on the UART1 channel,
		 * 	and trigger an interrupt if data is received.
		 */
		HAL_UART_Receive_IT(&huart1, UsbTTL2EmbeddedGS, sizeof(UsbTTL2EmbeddedGS));


	}


}
