
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdlib.h"
#include <stdio.h>

#include "IR.h"
#include "ROM.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//_______________________param_related_to_ :Learn
uint16_t index_IR=0;	//Index of BUffer IR
int32_t S_LED;			//signal related to LED blinking ,when Reciving the packet
uint8_t packet_overflow_flag = 0;

//_______________________param_related_to_transmit_IR
IR_PacketTypeDef									packet1[1]; //Packet Type to transmit			
IR_Transmitter_HandleTypeDef 			irth;		//IR Handler

//_______________________param_related_to_Uart
char t_str[60]; //buffer for transmit UART
uint8_t data_u1[1] ;	//reg for recive from USART1
uint8_t data_u2[1] ;	//reg for recive from USART2
int TimeOut_Recive =200;
USART_PacketTypeDef BUF; 		//bufer for collecting Data

const int serial_buffer_size = 512;
char serial_read_buffer[serial_buffer_size];
uint16_t read_ptr = 0;
uint16_t write_ptr = 0;

uint8_t  start_Recive=0;					//flag for mak frame 
uint32_t index_URAT=0;		//index for data_u

int32_t S_R_U1=0; //signal control task

int32_t S_R_U2=0;	//signal UART 2

char buffer[10],*token; //parsing buffer
int Buffer;							//decimaler
int l=0;								//index of parser
int esp_alive_flag = 0;

osThreadId mainTaskHandle;
osThreadId ControlTaskHandle;
osThreadId serialTaskHandle;
osThreadId esp_watcherHandle;
osTimerId myTimer01Handle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void ATR_IR_Recive(void);
void CorrectInputData(void);

void mainTask(void const * argument);
void Control_task(void const * argument);
void serial_task(void const * argument);
void esp_watcher_task(void const * argument);
void CallbackTIMERRTOS(void const * argument);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


	/*	IR_P Initializations 'BEGIN'	*/
		
		packet1->len = 0;
		packet1->packet_len_max = 512;
	
	
	/*	IR_P Initializations 'END'	*/
	
	/*	IR_T Initializations 'BEGIN'	*/
	irth.htim_BMkr = &htim2;
	irth.BMkr_channel = TIM_CHANNEL_1;
	irth.htim_Cntrllr = &htim3;
	irth.Cntrllr_channel = TIM_CHANNEL_1;
	
	/*	IR_T Initializations 'END'	*/

	
  /* Create the timer(s) */
  /* definition and creation of myTimer01 */
  osTimerDef(myTimer01, CallbackTIMERRTOS);
  myTimer01Handle = osTimerCreate(osTimer(myTimer01), osTimerPeriodic, NULL);

/* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(_mainTask, mainTask, osPriorityNormal, 0, 128);
  mainTaskHandle = osThreadCreate(osThread(_mainTask), NULL);

/* definition and creation of ControlTask */
  osThreadDef(ControlTask, Control_task, osPriorityNormal, 0, 128);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);
	
	osThreadDef(SerialTask, serial_task, osPriorityHigh, 0, 128);
  serialTaskHandle = osThreadCreate(osThread(SerialTask), NULL);

	osThreadDef(esp_watcher_Task, esp_watcher_task, osPriorityNormal, 0, 128);
  esp_watcherHandle = osThreadCreate(osThread(esp_watcher_Task), NULL);
	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* USER CODE BEGIN 4 */

void mainTask(void const * argument)
{
	S_LED = LED_SERVER_D;
	BUF.packet_len_max=512;
	packet1->packet_len_max=512;
	osDelay(500);
	HAL_UART_Receive_IT(&huart2,data_u2,1);
	HAL_UART_Transmit(&huart2,(unsigned char*)"starting...\n",12,100);
  for(;;){
		osEvent result=osSignalWait(S_R_U2,20000);
		if(result.status == osEventSignal){
			if(strcmp(BUF.pckt,"reset") == 0)
			{
				HAL_UART_Transmit(&huart2,(unsigned char*)"reseok",7,100);
				NVIC_SystemReset();
			}
			else
			{	
				switch (BUF.pckt[0])
				{
					//___________________Transmiting IR packet from ROM
					case 't':
						sscanf(BUF.pckt,"%*c,%d",&Buffer);	
							if(Buffer<128&&Buffer>30){
								load_packet(Buffer, packet1->pckt);	
								packet1->len= ROM_read_HWORD( Buffer , 0 );
								if(packet1->len > 511)
								{
									packet1->len = 511;
								}
								S_LED=LED_TRANSMIT;
								ATR_IR_Transmit( &irth, packet1,0);
								sprintf(t_str, "@d,%d.",packet1->len);
								HAL_UART_Transmit(&huart2,(unsigned char*)t_str,strlen(t_str),100);
	//							for(int j=0;j<packet1->len;j++)
	//							{
	//								if (j==packet1->len-1)
	//									{
	//										sprintf(t_str, "%d.",packet1->pckt[j]);
	//										HAL_UART_Transmit(&huart2,(unsigned char*)t_str,strlen(t_str),100);
	//									}
	//									else
	//									{
	//										sprintf(t_str, "%d,",packet1->pckt[j]);
	//										HAL_UART_Transmit(&huart2,(unsigned char*)t_str,strlen(t_str),100);
	//									}
	//							}							
								
							}
							else
							{
								HAL_UART_Transmit(&huart2,(unsigned char*)"@e:1." ,5,100);
							}
							osDelay(250);
					break;
					//___________________Reciving IR packet from ROM
					case 'r':
						sscanf(BUF.pckt,"%*c,%d",&Buffer);	
						if(Buffer<128&&Buffer>30)
						{
							S_LED = LED_LEARN_MODE;
							ATR_IR_Recive();// takes 20 seconds
							S_LED = LED_STOP;
							CorrectInputData();
							osDelay(400);
							if(packet1->len > 0 && packet1->len < 512)
							{
								save_packet( Buffer,packet1->pckt ,packet1->len );		
								//transmit packet which recived to ESP
								sprintf(t_str, "@d,%d.",packet1->len);
								HAL_UART_Transmit(&huart2,(unsigned char*)t_str,strlen(t_str),100);
	//							for(int j=0;j<packet1->len+1;j++) // save packet puts len at fist so +1 is to show all packet
	//							{
	//								if (j==packet1->len)
	//									{
	//										sprintf(t_str, "%d.",packet1->pckt[j]);
	//										HAL_UART_Transmit(&huart2,(unsigned char*)t_str,strlen(t_str),100);
	//									}
	//									else
	//									{
	//										sprintf(t_str, "%d,",packet1->pckt[j]);
	//										HAL_UART_Transmit(&huart2,(unsigned char*)t_str,strlen(t_str),100);
	//									}
	//							}							
							}			
							else	
							{
								if(packet_overflow_flag == 0)
								{
									sprintf(t_str, "@e:0.");
									HAL_UART_Transmit(&huart2,(unsigned char*)t_str,strlen(t_str),100);
								}
								else if(packet_overflow_flag == 1)
								{
									packet_overflow_flag = 0;
									sprintf(t_str, "@e:512.");
									HAL_UART_Transmit(&huart2,(unsigned char*)t_str ,strlen(t_str),100);
								}		
							}							
						}
						else
						{
							HAL_UART_Transmit(&huart2,(unsigned char*)"@e:1." ,5,100);
						}
					break;
					//Recive pacet from ESP and transmit it
					case 'd':
						token = strtok(BUF.pckt, ",");
						while( token != NULL ) {
							sscanf(token,"%d",&Buffer);
							packet1->pckt[l]=Buffer;
							l++;					 
							token = strtok(NULL, ",");
							}
						packet1->len=l;
						l=0;
						for(int k=0;k<packet1->len;k++)						//remove the firs one
							packet1->pckt[k]=packet1->pckt[k+1];
						packet1->len--;

							S_LED=LED_TRANSMIT;
							ATR_IR_Transmit( &irth, packet1,0);
							HAL_Delay(400);

					break;
					case 'c':
						sscanf(BUF.pckt,"%*c,%d",&Buffer);	
						switch (Buffer){
							case 0:
								//disconnected from server
							S_LED=LED_SERVER_D;
														
							break;
							case 1:
								//connected to server
								if(S_LED != LED_IDLE)
								{
									S_LED=LED_SERVER_C;
									osSignalSet(ControlTaskHandle,S_R_U1);
								}
							break;
							case 2:
								if(S_LED != LED_LOCAL_IDLE)
								{
									S_LED=LED_LOCAL_C;
									osSignalSet(ControlTaskHandle,S_R_U1);
								}
							break;
							case 3:
								S_LED=LED_HOTSPOT_C;
								osSignalSet(ControlTaskHandle,S_R_U1);
							break;
							default:
								break;
						}
						osDelay(250);
					break;
							
					default:
						sprintf(t_str, "@e:3.");		
						HAL_UART_Transmit(&huart2,(unsigned char*)t_str ,strlen(t_str),100);
					break;
				}
			}
		}
		osDelay(10);
	}
}	

void Control_task(void const * argument)
{
  /* Infinite loop */
	uint8_t local_state_flag = 0;
  for(;;)
  {
		switch (S_LED){
    	case	LED_SERVER_D :
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
			break;
			case	LED_HOTSPOT_C :
					HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
					osDelay(1000);	
					HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_SET);
					HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
					osDelay(1000);			
			break;	
    	case	LED_SERVER_C :
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
				for(int i=0;i<4;i++){
					HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_SET);
					osDelay(100);	
					HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
					osDelay(100);	
				}
				osDelay(1000);
				local_state_flag = 0;
				S_LED=LED_IDLE;				
			break;	
			case	LED_LOCAL_C :
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
				for(int i=0;i<4;i++){
					HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_SET);
					osDelay(100);	
					HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
					osDelay(100);	
				}
				osDelay(1000);
				local_state_flag = 1;
				S_LED=LED_LOCAL_IDLE;
			break;
			case LED_LEARN_MODE:
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_TogglePin(state_1_GPIO_Port, state_1_Pin);
				osDelay(150);
			break;
    	case LED_TRANSMIT:
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_SET);
				osDelay(600);	
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
				osDelay(600);
				S_LED = LED_STOP;
			break;
    	case LED_STOP:
				if(local_state_flag == 1)
				{
					S_LED = LED_LOCAL_IDLE;
				}
				else
				{
					S_LED = LED_IDLE;
				}
			break;
    	case LED_IDLE:
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_SET);
				osDelay(200);	
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
				for(int i = 0; i< 100 && S_LED == LED_IDLE ; i++)
				{
					 osDelay(50);
				}
			break;	
			case LED_LOCAL_IDLE:
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_SET);
				osDelay(200);	
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
				for(int i = 0; i< 100 && S_LED == LED_LOCAL_IDLE ; i++)
				{
					 osDelay(50);
				}
			break;		
			default:
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
			break;
			}
		
		if(HAL_GPIO_ReadPin(CMD_BUTTON_GPIO_Port,CMD_BUTTON_Pin) == 0)
		{
			int i=0;
			for(i=0; i<10 && HAL_GPIO_ReadPin(CMD_BUTTON_GPIO_Port,CMD_BUTTON_Pin) == 0; i++)
			{
				HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_SET);
				osDelay(200);
			}
			HAL_GPIO_WritePin(state_1_GPIO_Port, state_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(state_2_GPIO_Port, state_2_Pin, GPIO_PIN_RESET);
			if(i>=10)
			{
				osDelay(2000);
				HAL_UART_Transmit(&huart2,(unsigned char*)"@setting_reset.",15,100);	
				osDelay(100);
				NVIC_SystemReset();
			}
		}
		osDelay(10);
		HAL_UART_Receive_IT(&huart2,data_u2,1);
  }
}

void serial_task(void const * argument)
{
  /* Infinite loop */
	
	char char_buff = 0;
  for(;;)
	{
		if(read_ptr != write_ptr)
		{
			char_buff = serial_read_buffer[read_ptr];
			if(char_buff=='@'){
				start_Recive=1;//set 
				BUF.len=0;
			}	

			if(start_Recive==1)
			{
				if( (char_buff!='.') && char_buff!='@' && (BUF.len<BUF.packet_len_max))
				{
					BUF.pckt[BUF.len]=char_buff;
					BUF.pckt[BUF.len + 1]=0;
					BUF.len++;
				}
				if(char_buff=='.' )
				{
					if(strcmp(BUF.pckt,"alive") == 0)
					{
						esp_alive_flag = 1;
						start_Recive=0;
					}
					else
					{
						start_Recive=0;	
						osSignalSet(mainTaskHandle,S_R_U2);//set the signal of rtos set.	
					}
				}
				if(BUF.len >= BUF.packet_len_max )
				{//noise
					sprintf(t_str, "@e:2.");		
					HAL_UART_Transmit(&huart2,(unsigned char*)t_str ,strlen(t_str),100);	
					BUF.len=0;
					start_Recive=0;
				}
			}

			if(read_ptr < serial_buffer_size - 1 )
			{
				read_ptr ++ ;
			}
			else
			{
				read_ptr = 0;
			}
		}
		osDelay(1);
	}
}


void esp_watcher_task(void const * argument)
{
	osDelay(40000);
	for(;;)
	{
		esp_alive_flag = 0;
		HAL_UART_Transmit(&huart2,(unsigned char*)"@alive.",7,100);
		osDelay(2000);
		if(esp_alive_flag == 1)
		{
			esp_alive_flag = 0;
		}
		else
		{
			HAL_GPIO_WritePin(esp_reset_GPIO_Port,esp_reset_Pin,GPIO_PIN_RESET);
			osDelay(100);
			NVIC_SystemReset();
		}
			
		osDelay(10000);
	}
}

void CallbackTIMERRTOS(void const * argument)
{
  osTimerStop(myTimer01Handle);
	S_LED = LED_STOP;
	HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
		
		osTimerStart(myTimer01Handle,TimeOut_Recive);
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{//cycle
			packet1->pckt[index_IR] = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
			index_IR++;
		}
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{//duty
			packet1->pckt[index_IR] = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
			index_IR++;
		}	
		if(index_IR>511)
		{
			HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_1);
			HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_2);
			S_LED = LED_STOP;
			index_IR=0;
			for(int i=0;i<512;i++)
			{
				packet1->pckt[i]=0;
			}
			packet_overflow_flag = 1;	
		}
}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	ATR_IR_T_OC_Callback( &irth, irth.target_packet, htim );
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
		serial_read_buffer[write_ptr] = *data_u2;
		if(write_ptr < serial_buffer_size - 1 )
		{
			write_ptr ++ ;
		}
		else
		{
			write_ptr = 0;
		}	 
		HAL_UART_Receive_IT(&huart2,data_u2,1);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */


  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
