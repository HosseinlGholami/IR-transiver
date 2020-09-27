#include "stm32f1xx_hal.h"
#include "ROM.h"
#include "usart.h"
#include "string.h"
uint16_t BUFFER[512];

uint32_t getpage(uint8_t page){
return 0x08000000+1024*page;
}
void ROM_erase_page( uint8_t page){
	
	FLASH_EraseInitTypeDef hd;
	uint32_t eh;
	hd.TypeErase=FLASH_TYPEERASE_PAGES;
	hd.Banks=FLASH_BANK_1;
	hd.NbPages=1;
	hd.PageAddress=getpage(page);
	HAL_FLASH_Unlock();
	HAL_StatusTypeDef res=HAL_FLASHEx_Erase(&hd,&eh);
	if(res==HAL_OK)
		HAL_FLASH_Lock();
		
}	

uint8_t ROM_read_Byte  ( uint8_t page , uint16_t index ){
	
	return  *(__IO uint8_t*)(getpage(page)+index);

}
uint16_t ROM_read_HWORD( uint8_t page , uint16_t index ){
	
	return  *(__IO uint16_t*)(getpage(page)+2*index);

}
uint32_t ROM_read_WORD ( uint8_t page , uint16_t index ){
	return  *(__IO uint32_t*)(getpage(page)+4*index);
}
HAL_StatusTypeDef ROM_Write_HWORD(uint8_t page	,uint16_t VirtAddress	,uint16_t *Data	,int16_t length	){
	uint16_t Size = FLASH_PAGE_SIZE/2; //512
	HAL_StatusTypeDef res;	
	char t_str[10];
	for(int i=0;i<Size;i++)
	{
		BUFFER[i]=ROM_read_HWORD(page,i);
	}

	if((length+VirtAddress)<Size)
	{
		for(uint16_t i=0 ;i<length ;i++)
		{		
			BUFFER[i+VirtAddress] = Data[i];
		}
		
		ROM_erase_page(page);
		HAL_FLASH_Unlock();
		for(uint16_t i=0;i<Size;i++)
		{
			res=HAL_FLASH_Program( FLASH_TYPEPROGRAM_HALFWORD, ( getpage(page) + 2*i ), BUFFER[i]);
		}
		HAL_FLASH_Lock();
		return res;
	
	}else{
	return HAL_ERROR;
	
	}		
}
HAL_StatusTypeDef ROM_Write_WORD(uint8_t page	,uint16_t VirtAddress	,uint32_t *Data	,int16_t length	){
	//ino daghigh check nakardam _ vali ROM_Write_HWORD kamel doroste !
	// nafar bahdi ke mikhay estefade koni check kon bad estefade kon :)))) 
	volatile uint16_t Size =FLASH_PAGE_SIZE/4;
	volatile uint32_t BUF[Size];
	volatile HAL_StatusTypeDef res;
	
	for(int i=0;i<Size;i++)
		BUF[i]=ROM_read_WORD(page,i);
		
	ROM_erase_page(page);
	
	if(((length+VirtAddress)*4)<FLASH_PAGE_SIZE){
		for(int i=VirtAddress;i<(VirtAddress+length);i++)
			BUF[i]=Data[i];
		HAL_FLASH_Unlock();
		for(int i=0;i<Size;i++)		
			res=HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, getpage(page), BUF[i]);
		HAL_FLASH_Unlock();
		return res;
	}else{
		return HAL_ERROR;
		}
}


void save_packet(uint8_t page,uint16_t *Data,uint16_t len){
	if(len<512)
	{
		for(int i=(len-1);i>=0;i--)
		{
			Data[i+1]=Data[i];
		}
			
		Data[0]=len;
		ROM_Write_HWORD( page	, 0	, Data	, len + 1	);
	}
}

void load_packet(uint8_t page,uint16_t *Data){
	uint16_t len=ROM_read_HWORD( page , 0 );
	if(len<512)
	{
		for(uint16_t i=0 ; i<len ; i++)
			Data[i] =ROM_read_HWORD( page, i+1 );		
	}
	else
	{
		len=511;
		for(uint16_t i=0 ; i<len ; i++)
			Data[i] =0;		
	}
}	
