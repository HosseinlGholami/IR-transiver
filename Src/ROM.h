#include "stm32f1xx_hal.h"

#define ROM_Byte             0x01U   /*!<Program  a  byte 			(8-bit) at a specified address.*/
#define ROM_HALFWORD         0x02U   /*!<Program a  half-word  (16-bit) at a specified address.*/
#define ROM_WORD             0x01U   /*!<Program a word 			  (32-bit) at a specified address.*/
	

uint32_t getpage(uint8_t page);	

void ROM_erase_page( uint8_t page);
uint8_t ROM_read_Byte  ( uint8_t page , uint16_t index );
uint16_t ROM_read_HWORD( uint8_t page , uint16_t index );
uint32_t ROM_read_WORD ( uint8_t page , uint16_t index );
HAL_StatusTypeDef ROM_Write_HWORD(uint8_t page	,uint16_t VirtAddress	,uint16_t *Data	,int16_t length	);
HAL_StatusTypeDef ROM_Write_WORD(uint8_t page	,uint16_t VirtAddress	,uint32_t *Data	,int16_t length	);

void save_packet(uint8_t page,uint16_t *Data,uint16_t len);
void load_packet(uint8_t page,uint16_t *Data);
