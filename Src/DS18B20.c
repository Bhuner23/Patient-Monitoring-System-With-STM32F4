#include "stm32f4xx_hal.h"


/* Kullanilan port ve pine göre burada gerekli degisiklikleri yapiyoruz.*/
#define DS_PORT GPIOB
#define DS_PIN GPIO_PIN_7

float Temperature = 0.0;
uint8_t Presence = 0;
uint8_t LS_Byte = 0;
uint8_t MS_Byte = 0;
uint8_t LS_Byte_Temp = 0;
uint8_t MS_Byte_Temp = 0;
uint16_t Word = 0;


#include <DS18B20.h>

uint32_t DWT_Delay_Init(void)
{
  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

     /* 3 NO OPERATION instructions */
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");
     __ASM volatile ("NOP");

  /* Check if clock cycle counter has started */
     if(DWT->CYCCNT)
     {
       return 0; /*clock cycle counter started*/
     }
     else
  {
    return 1; /*clock cycle counter not started*/
  }
}

__STATIC_INLINE void delay(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;

  /* Go to number of cycles for system */
  microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Speed = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	DWT_Delay_Init();
	Set_Pin_Output(DS_PORT, DS_PIN);   																// Pin'i output olarak ayarladik.
	HAL_GPIO_WritePin (DS_PORT, DS_PIN, 0);  														// Pin'i LOW'a cektik.
	delay (480);   																					// Datasheet'e gore gerekli gecikmeyi verdik.
	Set_Pin_Input(DS_PORT, DS_PIN);    																// Pin'i input olarak ayarladik.
	delay (80);    																					// Datasheet'e gore gerekli gecikmeyi verdik.
	if (!(HAL_GPIO_ReadPin (DS_PORT, DS_PIN)))
		Response = 1;    																			// Eger pin LOW ise presence puls'i algilanmistir.
	else
		Response = -1;
	delay (400); 																					// Toplamda 400 - 480 us kadar bir gecikme veriyoruz. .
	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS_PORT, DS_PIN);  																// Output olarak ayarladik.
	for (uint8_t i = 0; i < 8; i++){
		if ((data & (1 << i)) != 0){  																// Eger bit HIGH ise;
			// write 1
			Set_Pin_Output(DS_PORT, DS_PIN);  														// Output olarak ayarladik.
			HAL_GPIO_WritePin (DS_PORT, DS_PIN, 0);  												// Pin'i LOW'a cektik.
			delay (1);  																			// 1 us bekledik.
			Set_Pin_Input(DS_PORT, DS_PIN);  														// Input olarak ayarladik.
			delay (60);  																			// 50-60 us gecikme verdik.
		}
		else{  																						// Eger bit LOW ise;
			// write 0
			Set_Pin_Output(DS_PORT, DS_PIN);
			HAL_GPIO_WritePin (DS_PORT, DS_PIN, 0);  												// Pin'i LOW'a cektik.
			delay (60);  																			// 50-60 us bekledik.
			Set_Pin_Input(DS_PORT, DS_PIN);
		}
	}
}


uint8_t DS18B20_Read(void)
{
	uint8_t value = 0;
	Set_Pin_Input(DS_PORT, DS_PIN);
	for (uint8_t i = 0; i < 8; i++){
		Set_Pin_Output(DS_PORT, DS_PIN);   															// Output olarak ayarladikk.
		HAL_GPIO_WritePin (DS_PORT, DS_PIN, 0);  													// Pin'i LOW'a cektik.
		delay (2);  																				// 2 us gecikme verdikk.
		Set_Pin_Input(DS_PORT, DS_PIN);  															// Input olarak ayarladik.
		if (HAL_GPIO_ReadPin (DS_PORT, DS_PIN)){  													// Eğer pin HIGH ise;
			value |= 1 << i;  																		// Okuma sonucu 1
		}
		delay (60);  																				// 50-60 us bekliyoruz.
	}
	return value;
}

void DS_GetData (DS_DataTypedef *DS_Data)
{
	  Presence = DS18B20_Start();
	  HAL_Delay(1);
	  DS18B20_Write(0xCC); 																			// Skip ROM
	  DS18B20_Write(0x44); 																			// Convert t
	  //HAL_Delay(800);

	  Presence = DS18B20_Start();
	  HAL_Delay(1);
	  DS18B20_Write(0xCC); 																			// Skip ROM
	  DS18B20_Write(0xBE); 																			// Read Scratch-pad

	  LS_Byte = DS18B20_Read();
	  MS_Byte = DS18B20_Read();

	  LS_Byte_Temp = LS_Byte;
	  MS_Byte_Temp = MS_Byte;

	  Word = ((uint16_t)MS_Byte_Temp << 8) | (uint16_t)LS_Byte_Temp;
	  DS_Data->T_Reel = (float)Word / 16;

	  if ((MS_Byte >> 3) == 0){
		  DS_Data->Sign = 1;																		// Sicaklik verisi + ise 1
		  DS_Data->T_Integer = ((MS_Byte & 0x07) << 4) | (LS_Byte_Temp >> 4);
		  DS_Data->T_Fractional = (float)((LS_Byte & 0x0F) * 0.0625);
	  }
	  else{
		  DS_Data->Sign = 0;																		// Sicaklik verisi - ise 0
		  /* Negatif sıcaklık değeri gerekmediği icin hesaplanmadi */
	  }


}


