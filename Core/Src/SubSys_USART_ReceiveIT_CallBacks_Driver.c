#include "SubSys_USART_ReceiveIT_CallBacks_Driver.h"



extern	  	char 								 EmbeddedGS2Payload[30];
extern 		SubSys_WirelessCom_APP_HandleTypeDef dev_WirelessComApp;

	/*! CommandPC(GroundStation) send a packet :
	 * e.g ==> *G<3R7B+> or *G<????->
	 * Let's we explain what are they
	 * 			 '*' 			 -> CommandPC(GroundStation) wants a telepacket
	 * 			 'G' 			 -> We put 'G' char into the telemetry packet that we will send to the payload
	 * 			 '3' 'R' '7' 'B' -> Chars number and letter for Color Filtering
	 * 			 -,+ 			 ->	Manuel Deploy command, + manuel deploy active, - manuel deploy deactive
	 *
	 */
extern	  	char								  UsbTTL2EmbeddedGS[9];

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
		if((UsbTTL2EmbeddedGS[3] !='?') && (UsbTTL2EmbeddedGS[4] !='?') && (UsbTTL2EmbeddedGS[5] !='?') && (UsbTTL2EmbeddedGS[5] !='?'))
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
		if(UsbTTL2EmbeddedGS[6] =='+')
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
