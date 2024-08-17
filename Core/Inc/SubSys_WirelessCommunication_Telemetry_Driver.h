
#ifndef SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_TELEMETRY_H
#define SAT_PAYLOAD_SUBSYS_DRIVERS_WIRELESSCOMMUNICATION_TELEMETRY_H


/******************************************************************************
         				#### WIRELESSCOM INCLUDES ####
******************************************************************************/
#include "main.h"

/******************************************************************************
         				#### WIRELESSCOM DEFINITIONS ####
******************************************************************************/

/*! 200 is the number of packets the LoRa module can send in a single transmission */
#define SizeOf_Wireless_TX_Buff_PAYLOAD 	30
#define SizeOf_Wireless_GroundStation	 	200
/******************************************************************************
         				#### WIRELESSCOM ENUMS ####
******************************************************************************/
typedef enum{

	Sat_Carrier = 1,
	Sat_Payload,
	GroundStation,
	GroundStationMcu

}MissionUnit;


/******************************************************************************
         				#### WIRELESSCOM EXTERNS ####
******************************************************************************/

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern float  MS5611_Temp;			/*! Temperature data variable 		*/
/******************************************************************************
         				#### WIRELESSCOM STRUCTS ####
******************************************************************************/

typedef struct{

	/*####################### GROUND STATION UNIT VARIABLES #######################*/

			float PAY_IOT_Temperature;					/* Unit : float(4Byte) e.g => 23.45°    	  */
			char PAY_dataRHRH[4];						/* Unit : char(1Byte)  e.g => '3','G','7','B' */

}SubSys_WirelessCom_VariableTypeDef;

typedef struct{
	uint8_t Tx[SizeOf_Wireless_TX_Buff_PAYLOAD];	 /*! Buffer for Datas that send to Lora 	 */
	char 	Temp[SizeOf_Wireless_TX_Buff_PAYLOAD];	 /*! Buffer for Datas that send to Lora		 */
	char	Rx[SizeOf_Wireless_GroundStation];	 	 /*! Buffer for Datas that receive from Lora */
}SubSys_WirelessCom_BufferTypeDef;

typedef struct{

	/*! Inner structs */
	SubSys_WirelessCom_BufferTypeDef Buffer;
	SubSys_WirelessCom_VariableTypeDef	Variable;

	/*! Used device DMA and Usart interface settings */
	UART_HandleTypeDef *huartX;
	DMA_HandleTypeDef *hdma_usartX_rx;
	DMA_HandleTypeDef *hdma_usartX_tx;

	/*! Target device Address and Channel settings */
	uint8_t Target_ADDH;
	uint8_t Target_ADDL;
	uint8_t Target_Ch;

}SubSys_WirelessCom_APP_HandleTypeDef;


/******************************************************************************
         			#### WIRELESSCOM PROTOTYPES OF FUNCTIONS ####
******************************************************************************/
/**
  * @brief Decimal, float and other formats are converted as character and save them into the TX buffer.
  * 		When TX buffer is fulfilled , it is sent by UART interface.
  * @note  Follow the transmitting rules, each if and else has a reason
  * @param MissionUnit From_X, Packet type used for your specific purpose. Where do you  want to take it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  *
  * @param MissionUnit To_Y, Packet type used for your specific purpose. Where do you want to send it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
  * @retval NONE
  */
void SubSys_WirelessCom_Telemetry_Transfer_From_To(MissionUnit From_X, MissionUnit To_Y, SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp);


/**
  * @brief
  * @note  Follow the transmitting rules, each if and else has a reason
  * @param MissionUnit From_X, Packet type used for your specific purpose. Where do you  want to take it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  *
  * @param MissionUnit To_Y, Packet type used for your specific purpose. Where do you want to send it, select that.
  * 																	  @arg 0 : Sat_Carrier
  * 																 	  @arg 1 : Sat_Payload
  * 																  	  @arg 2 : Ground_Sation
  * @param SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
  * @retval NONE
  */

void SubSys_WirelessCom_Telemetry_Receive_From_To(MissionUnit From_X, MissionUnit To_Y, SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp);


/**
 * @brief  : Creates 3 types packet for Carrier, Payload and Ground Station
 * @note   : Where do you want to send the packet, select that, For example you are a courier and
 * 			 carry the packet to the selected home(MissionUnite x)
 *
 * @param  : MissionUnit x, Packet type used for your specific purpose @arg 0 : Sat_Carrier
 * 																	   @arg 1 : Sat_Payload
 * 																	   @arg 2 : Ground_Sation
 *
 * @param  : SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp , created object for wireless communication
 * @retval NONE
 */
void SubSys_WirelessCom_Telemetry_Create_Packet_For(MissionUnit x,SubSys_WirelessCom_APP_HandleTypeDef *dev_WirelessComApp);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
#endif /* SAT_CARRIER_SUBSYS_DRIVERS_SUBSYS_INC_SUBSYS_WIRELESSCOMMUNICATION_TELEMETRY_DRIVER_H_ */
