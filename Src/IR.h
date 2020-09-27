#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "string.h"


#define LED_STOP 			   		 0U
#define LED_LEARN_MODE  	   1U
#define LED_SERVER_C 		     2U
#define LED_SERVER_D    		 3U
#define LED_TRANSMIT    		 4U
#define LED_IDLE		    		 5U
#define LED_LOCAL_C					 6U
#define LED_LOCAL_IDLE			 7U
#define LED_HOTSPOT_C				 8U

typedef enum
{
  IR_Sending_Burst             = 0x0U,    /*!< IR_Transmitter is sending the burst signal  															*/
															
  IR_Sending_LowLevel          = 0x1U,    /*!< IR_Transmitter is sending low level		      														*/
	
	IR_Initializing 			       = 0x2U,    /*!< IR_Transmitter is initializing (It is in idle mode(Sending Low Level))		*/
	
	IR_Sending_Done			         = 0x3U,    /*!< IR_Transmitter reached the end of packet		     													*/

}IR_Transmitter_StateTypeDef;

typedef enum
{
  IR_BurstMaker_BurstMode      = 0x0U,    /*!< IR_Transmitter Mode = Send Burst  			*/
	
  IR_BurstMaker_LowLevelMode   = 0x1U,    /*!< IR_Transmitter Mode =	Send low level	*/
	
}IR_BurstMaker_StateTypeDef;

typedef enum
{
	IR_Receiver_Disable					 = 0x0U,    /*!<	IR_Receiver is disable (This state is used to avoid disabling IRQ)	*/
		
	IR_Receiver_Idle	           = 0x1U,    /*!< 	IR_Receiver is waiting for the first edge to start the timer	     	*/
		
	IR_Receiver_IsGetting 			 = 0x2U,    /*!< 	IR_Receiver is active. i.e. it is saving the lvl durations					*/
		
	IR_Receiving_LVL_Timeout		 = 0x4U,    /*!<	Duration of a lvl is more than threshold														*/
																															
	IR_Receiving_Len_Overflow		 = 0x5U,    /*!<	The packet got full																									*/
	
}IR_Receiver_StateTypeDef;

typedef struct
{
	uint16_t																	packet_len_max;		/*!<	The length of packet array should be written here (Handle the overflow of the packet)*/
	uint16_t																	len;					 		/*!<	(Length) length of the packet																												 */
	
  uint16_t																	pckt[514];		 		/*!<	(Packet) data array																																	 */
	
}IR_PacketTypeDef;

typedef struct
{
	uint16_t																	packet_len_max;		/*!<	The length of packet array should be written here (Handle the overflow of the packet)*/
	uint16_t																	len;					 		/*!<	(Length) length of the packet																												 */
	
	char 																			pckt[512];		 		/*!<	(Packet) data array																																	 */
	
}USART_PacketTypeDef;


typedef struct
{
	TIM_HandleTypeDef													*htim_Cntrllr;								/*!<	The pointer to the controller timer's handler																				*/
	uint32_t 																	Cntrllr_channel;							/*!<	The channle of the controller timer configured as Output Compare	No Output					*/
	TIM_HandleTypeDef 												*htim_BMkr;										/*!<	The pointer to the handler of the timer which makes the bursts											*/
	uint32_t 																	BMkr_channel;									/*!<	The channel of the Burst maker controller configured as PWM Generator								*/
	IR_PacketTypeDef													*target_packet;
												
	volatile	IR_Transmitter_StateTypeDef 		state;												/*!<	Handles the transmition steps																												*/
	volatile	uint16_t												cntr;					 								/*!<	(Counter) Counter of the packet array																								*/
	volatile	uint16_t												packet_nmbr;
	
}IR_Transmitter_HandleTypeDef;

typedef struct
{
	IRQn_Type 																irqn;													/*!<	The IRq connected to IR receiver sensor																							*/
	TIM_HandleTypeDef													*htim_Capture;								/*!<	The pointer to the capturer timer's handler																					*/
	uint32_t 																	LVL_Timeout_Handler_channel;	/*!<	The channle of the capturer timer configured as Output Compare	No Output	which..		*/

	uint32_t																	lvl_timeout;									/*!<	The maximum duraton of a lvl in received signal																			*/
	IR_PacketTypeDef													*target_packet;
	
	volatile	IR_Receiver_StateTypeDef 				state;												/*!<	Handles the Receiving steps																													*/
	volatile	uint32_t												lvl_nmbr;											/*!<	(Level Number)Counter of the packet array																						*/
				
}IR_Receiver_HandleTypeDef;

	

typedef enum
{
  IR_UART_Idle             	= 0x0U,    /*!<					*/
															
  IR_UART_T_Mode						= 0x1U,    /*!<					*/
	
	IR_UART_R_Mode 			    	= 0x2U,    /*!< 				*/
	
	IR_UART_Invld 			    	= 0x3U,    /*!< 				*/

}IR_UART_StateTypeDef;

typedef struct
{
	uint8_t																		u_data_len;
	uint8_t																		u_data1[5];
					
	volatile	IR_UART_StateTypeDef						u_state;
	
}IR_UART_MsgTypeDef;

uint32_t							IR_Msg_Find_Nmbr				(	IR_UART_MsgTypeDef *u_msg );
HAL_StatusTypeDef 	ATR_IR_Transmit	( IR_Transmitter_HandleTypeDef *irth, IR_PacketTypeDef *packet, uint16_t timeout );
void ATR_IR_Recive									( void );
void CorrectInputData								( void );
void ATR_IR_T_OC_Callback						( IR_Transmitter_HandleTypeDef *irth, IR_PacketTypeDef *packet, TIM_HandleTypeDef *htim );
void ATR_IR_R_OC_Callback						( IR_Receiver_HandleTypeDef *irrh, TIM_HandleTypeDef *htim );
void ATR_IR_R_EXI_Callback					( IR_Receiver_HandleTypeDef *irrh, IR_PacketTypeDef *packet, uint16_t GPIO_Pin );
HAL_StatusTypeDef		ATR_TIM_SetMode	(	IR_Transmitter_HandleTypeDef *irth, IR_BurstMaker_StateTypeDef	mode );
void CallbackTIMERRTOS							(void const * argument);

