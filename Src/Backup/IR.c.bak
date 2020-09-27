#include "IR.h"
	

extern	IR_PacketTypeDef		packet1[1];
extern	uint16_t						index_IR  ;	
extern	TIM_HandleTypeDef htim1;
extern	TIM_HandleTypeDef htim2;
extern char t_str[512]; //buffer for transmit UART

extern UART_HandleTypeDef huart2;
extern osTimerId myTimer01Handle;
extern int32_t S_LED;			//signal related to LED blinking ,when Reciving the packet
extern osThreadId ControlTaskHandle;
extern int32_t S_R_U1; //signal UART 1

HAL_StatusTypeDef	ATR_IR_Transmit( IR_Transmitter_HandleTypeDef *irth, IR_PacketTypeDef *packet, uint16_t timeout ){
	
	irth->target_packet = packet;
	////// Timer initialization (Future Updates!): /////////
	(irth->state) = IR_Initializing;
	//Stop Burst Maker (Send Low Level)	
	ATR_TIM_SetMode( irth, IR_BurstMaker_LowLevelMode );
	HAL_TIM_PWM_Start_IT( irth->htim_BMkr, irth->BMkr_channel);								//	Burst Maker Start	
	HAL_TIM_OC_Start_IT( irth->htim_Cntrllr, irth->Cntrllr_channel);					//	Controller  Start	
	//_____________________________________________
	irth->cntr = 0;
	irth->state = IR_Sending_LowLevel;
	
	return HAL_OK;
}

void	ATR_IR_T_OC_Callback( IR_Transmitter_HandleTypeDef *irth, IR_PacketTypeDef *packet, TIM_HandleTypeDef *htim ){
	if( htim == (irth->htim_Cntrllr) )
	{
		switch (irth->state)
		{
			case IR_Initializing:
			{		
				break;
			}
			case IR_Sending_Burst:
			{
				if( (irth->cntr) == (packet->len) )
				{
					(irth->state) = IR_Sending_Done;
					HAL_TIM_PWM_Stop_IT( irth->htim_BMkr, irth->BMkr_channel);
					HAL_TIM_OC_Stop_IT( irth->htim_Cntrllr, irth->Cntrllr_channel);
					break;
				}
				__HAL_TIM_SET_COUNTER( irth->htim_Cntrllr , 0);

				ATR_TIM_SetMode( irth, IR_BurstMaker_LowLevelMode );
				
				__HAL_TIM_SET_COMPARE( irth->htim_Cntrllr, irth->Cntrllr_channel, packet->pckt[irth->cntr]);
				(irth->cntr)++;
				
				irth->state = IR_Sending_LowLevel;
				
				break;
			}
			case IR_Sending_LowLevel:
			{
				if( (irth->cntr) == (packet->len) )
				{
					(irth->state) = IR_Sending_Done;
					HAL_TIM_PWM_Stop_IT( irth->htim_BMkr, irth->BMkr_channel);
					HAL_TIM_OC_Stop_IT( irth->htim_Cntrllr, irth->Cntrllr_channel);	
					break;
				}
				__HAL_TIM_SET_COUNTER( irth->htim_Cntrllr , 0);

				ATR_TIM_SetMode( irth, IR_BurstMaker_BurstMode );			
				
				__HAL_TIM_SET_COMPARE( irth->htim_Cntrllr, irth->Cntrllr_channel, packet->pckt[irth->cntr]);
				(irth->cntr)++;		
				(irth->state) = IR_Sending_Burst;
				break;
			}
			default:
				break;
		}
	}
}
	
HAL_StatusTypeDef ATR_TIM_SetMode( IR_Transmitter_HandleTypeDef *irth, IR_BurstMaker_StateTypeDef	mode ){
	switch( mode )
	{
		case IR_BurstMaker_LowLevelMode:
		{
			//Stop Burst Maker (Send Low Level)	
			(irth->htim_BMkr)->Instance->CCMR1 &= (~0X0020);					/*	CCMR1[6..4] == 110 -->  PWM_1
																																						== 100 -->  Forced Inactive	*/
			return HAL_OK;
		}
		case IR_BurstMaker_BurstMode:
		{
			//Start Burst Maker	(Send Burst)
			(irth->htim_BMkr)->Instance->CCMR1 |= 0X0020;					/*	CCMR1[6..4] == 110 -->  PWM_1
																																						== 100 -->  Forced Inactive	*/
			return HAL_OK;
		}
		default:
		{
			break;
		}
	}
	return HAL_ERROR;
}


void ATR_IR_Recive(void){
	index_IR=0;
//	sprintf(t_str, "recive Start 5s\n");
//	HAL_UART_Transmit(&huart1,(unsigned char*)t_str ,strlen(t_str),100);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	for(int i = 0 ; i< 2000 && S_LED == LED_LEARN_MODE ; i++)
	{
		osDelay(10);
	}
	HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);
if(htim1.State!=HAL_TIM_STATE_RESET){
//	sprintf(t_str, "recive End5\n");
//	HAL_UART_Transmit(&huart1,(unsigned char*)t_str ,strlen(t_str),100);
	}
}
void CorrectInputData(void){
		//________________creat format for transmit_________________________________________________________________________
		if(index_IR>0 && index_IR<512)
		{
			packet1->len=index_IR-1;//set the frame len (first one is not corect (cycle))
		}
		else
		{
			packet1->len=0;
		}		
		packet1->packet_len_max = 511;//set the max len of frame
		
		for(int i=0;i<packet1->len;i++)
		{
			packet1->pckt[i]=packet1->pckt[i+1];//remove the first data and put it in paket type
		}

		for(int k=0;k<packet1->len;k++)
		{
			if((k%2)==1)
				packet1->pckt[k]=(packet1->pckt[k] - packet1->pckt[k-1]);					
		}//make the format of array in some way , which can send 
//		for(int j=0;j<packet1->len;j=j+2){
//			sprintf(t_str, "%d-B:%d,A:%d\n",j,packet1->pckt[j],packet1->pckt[j+1]);
//			HAL_UART_Transmit(&huart1,(unsigned char*)t_str,strlen(t_str),100);
//		}//print the frame, B is time of low , A is high
				
}

