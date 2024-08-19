
/******************************************************************************
         				#### WIRELESSCOM INCLUDES ####
******************************************************************************/
#include "SubSys_WirelessCommunication_Telemetry_Driver.h"

#include "stdio.h"
#include "stdlib.h"
/******************************************************************************
         				#### WIRELESSCOM VARIABLES ####
******************************************************************************/
uint16_t Written_Bytes; /* is for save number of total converted buffer's characters*/
uint8_t cnt;


/******************************************************************************
         				#### WIRELESSCOM  EXTERNS ####
******************************************************************************/
extern	  	char 								 EmbeddedGS2Payload[30];

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

/******************************************************************************
         				#### WIRELESSCOM  FUNCTIONS ####
******************************************************************************/




/**
  * @brief Decimal, float and other formats are converted as character and save them into the TX buffer.
  * 		When TX buffer is fulfilled , it is sent by UART interface.
  * @note  Follow the transmitting rules, each if and else if has a why head for using
  * @param MissionUnit From_X, Packet type used for your specific purpose. Where do you  want to take it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param MissionUnit To_Y, Packet type used for your specific purpose. Where do you want to send it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
  * @retval NONE
  */
void SubSys_WirelessCom_Telemetry_Transfer_From_To(MissionUnit From_X, MissionUnit To_Y, SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp){


	SubSys_WirelessCom_Telemetry_Create_Packet_For(GroundStationMcu, dev_WirelessComApp);

			/*! We continue create a telemetry packet for transmitting to payload*/
			Written_Bytes = sprintf(dev_WirelessComApp->Buffer.Temp,
																	"G<%c%c%c%c%c><%.2f>",
																					   dev_WirelessComApp->Variable.PAY_dataRHRH[0],
																					   dev_WirelessComApp->Variable.PAY_dataRHRH[1],
																					   dev_WirelessComApp->Variable.PAY_dataRHRH[2],
																					   dev_WirelessComApp->Variable.PAY_dataRHRH[3],
																					   dev_WirelessComApp->Variable.PAY_SeparationCommand,
																					   dev_WirelessComApp->Variable.PAY_IOT_Temperature);

			/*! Fill Embedded ground station command and ambiance temperature info */
			for(cnt = 0 ; cnt < Written_Bytes ; cnt++)
			{

				dev_WirelessComApp->Buffer.Tx[cnt+3] = dev_WirelessComApp->Buffer.Temp[cnt]; /*End of the array has \n character*/

			}

			/*! Fill gaps with character '*', So we create a 30bytes buffer */
			for(uint8_t j=(cnt+3) ; j < SizeOf_Wireless_TX_Buff_PAYLOAD ; j++)
			{

				dev_WirelessComApp->Buffer.Tx[j] = '*';

			}

			/*! Transmit all Tx buffer(uint8_t) to the Payload of Satellite
			 * Total Size 30byte
			 */
			HAL_UART_Transmit(dev_WirelessComApp->huartX, dev_WirelessComApp->Buffer.Tx , SizeOf_Wireless_TX_Buff_PAYLOAD, 1000);
			//HAL_UART_Transmit(dev_WirelessComApp->huartX, (uint8_t *)dev_WirelessComApp->Buffer.Tx , SizeOf_Wireless_TX_Buff_PAYLOAD, 1000);
}




/**
 * @brief  : Creates 3 types packet for Carrier, Payload and Ground Station
 * @note   : Where do you want to send packet, select that, For example you are a courier and
 * 			 carry the packet to the selected home(MissionUnite x)
 *
 * @param  : MissionUnit x, Packet type used for your specific purpose @arg 0 : Sat_Carrier
 * 																	   @arg 1 : Sat_Payload
 * 																	   @arg 2 : Ground_Sation
 * @param  : SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
 * @retval NONE
 */
void SubSys_WirelessCom_Telemetry_Create_Packet_For(MissionUnit x,SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp){

	/*-------------TARGET DEVICE ADDRESS AND CHANNEL INFO----------------*/
				/*! Target device will be Satellite's Payload*/
				dev_WirelessComApp->Buffer.Tx[0] = dev_WirelessComApp->Target_ADDH;
				dev_WirelessComApp->Buffer.Tx[1] = dev_WirelessComApp->Target_ADDL;
				dev_WirelessComApp->Buffer.Tx[2] = dev_WirelessComApp->Target_Ch;

				/*-------------YOUR DEVICE VARIABLE THAT YOU WİLL SEND----------------*/
				/*! We need to get temperature IOT data from the Embedded ground station and save into the variable */

				dev_WirelessComApp->Variable.PAY_IOT_Temperature = MS5611_Temp;

				dev_WirelessComApp->Variable.PAY_dataRHRH[0]	   =  UsbTTL2EmbeddedGS[3];
				dev_WirelessComApp->Variable.PAY_dataRHRH[1] 	   =  UsbTTL2EmbeddedGS[4];
				dev_WirelessComApp->Variable.PAY_dataRHRH[2] 	   =  UsbTTL2EmbeddedGS[5];
				dev_WirelessComApp->Variable.PAY_dataRHRH[3] 	   =  UsbTTL2EmbeddedGS[6];
				dev_WirelessComApp->Variable.PAY_SeparationCommand =  UsbTTL2EmbeddedGS[7];



}
void SubSys_WirelessCom_Telemetry_Receive_From_To(MissionUnit From_X, MissionUnit To_Y, SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp){

	/**
	 *  "The Buffer.Rx will be filled with the payload's telemetry packet.
	 *   The payload must send a 200-byte telemetry packet for the IT function to work properly.
	 */
	HAL_UART_Receive_IT(dev_WirelessComApp->huartX, (uint8_t *)dev_WirelessComApp->Buffer.Rx, sizeof(dev_WirelessComApp->Buffer.Rx));

}

