
#ifndef _MAX30102_H_
#define _MAX30102_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include "algorithm.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX30102_ADDRESS 0xAE	// (0x57<<1)


#define MAX30102_MEASUREMENT_SECONDS 		5		// saniye cinsinden ölçüm süresi (varsayılan 5s)
#define MAX30102_SAMPLES_PER_SECOND			100 	// saniyede numune (varsayılan 100) aşağıdaki değerlere sahip olabilir 50, 100, 200, 400, 800, 100, 1600, 3200 numune derecelendirmesi
#define MAX30102_FIFO_ALMOST_FULL_SAMPLES 	17		// kesme çağrısı için ölçüm sayısı (maksimum 32 örnek, varsayılan 17)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
//	Status enum
//
typedef enum{
	MAX30102_ERROR,	
	MAX30102_OK	
} MAX30102_STATUS;


typedef enum{
	MAX30102_STATE_BEGIN,
	MAX30102_STATE_CALIBRATE,
	MAX30102_STATE_CALCULATE_HR,
	MAX30102_STATE_COLLECT_NEXT_PORTION
}MAX30102_STATE;

	
#define MAX30102_BUFFER_LENGTH	((MAX30102_MEASUREMENT_SECONDS+1)*MAX30102_SAMPLES_PER_SECOND)

//
//	Calibration
//

//-----------------------------------------------------------------
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch
  
#define MAX30102_IR_LED_CURRENT_LOW				0x02
#define MAX30102_RED_LED_CURRENT_LOW			0x02
#define MAX30102_GREEN_LED_CURRENT_LOW			0x02

#define MAX30102_IR_LED_CURRENT_HIGH			0x24
#define MAX30102_RED_LED_CURRENT_HIGH			0x24
#define MAX30102_GREEN_LED_CURRENT_HIGH			0x24

//------------------------------------------------------------------

#define MAX30102_IR_VALUE_FINGER_ON_SENSOR 		1600
#define MAX30102_IR_VALUE_FINGER_OUT_SENSOR 	50000
//
//	Register addresses
//
#define REG_INTR_STATUS_1 		0x00	// Kesme durumu 1
#define REG_INTR_STATUS_2 		0x01	// Kesme durumu 2 (Sadece sicaklik icin gerekli)
#define REG_INTR_ENABLE_1 		0x02	// Kesme durumu 1
#define REG_INTR_ENABLE_2 		0x03	// Kesme durumu 2 (Sadece sicaklik icin gerekli)
#define REG_FIFO_WR_PTR 		0x04
#define REG_OVF_COUNTER 		0x05
#define REG_FIFO_RD_PTR 		0x06
#define REG_FIFO_DATA 			0x07
#define REG_FIFO_CONFIG 		0x08
#define REG_MODE_CONFIG 		0x09
#define REG_SPO2_CONFIG 		0x0A
#define REG_LED1_PA 			0x0C	// IR
#define REG_LED2_PA 			0x0D	// RED
#define REG_LED3_PA 			0x0E	// GREEN
#define REG_PILOT_PA 			0x10
#define REG_MULTI_LED_CTRL1 	0x11	// slot 1 ( 0,1,2 byte ) ve slot 2 ( 5,6,7 byte )
#define REG_MULTI_LED_CTRL2 	0x12	// slot 3 ( 0,1,2 byte ) ve slot 4 ( 5,6,7 byte )
#define REG_TEMP_INTR 			0x1F
#define REG_TEMP_FRAC 			0x20
#define REG_TEMP_CONFIG 		0x21
#define REG_PROX_INT_THRESH 	0x30
#define REG_REV_ID 				0xFE
#define REG_PART_ID 			0xFF

//
//	Interrupt Status 1 (0x00)
//	Interrupt Status 2 (0x01)
//	Interrupt Enable 1 (0x02)
//	Interrupt Enable 2 (0x03)
//
#define	INT_A_FULL_BIT			7	// SpO2 ve HR modlarında, bu kesinti, FIFO yazma işaretçisinde belirli bir miktarda boş alan kaldığında tetiklenir.
#define	INT_PPG_RDY_BIT			6	// SpO2 ve HR modlarında, bu kesme FIFO verisinde yeni bir örnek alındığında tetiklenir.
#define	INT_ALC_OVF_BIT			5	// Bu kesinti, SpO2/HR fotodiyotun ortam ışığı bastırma işlevi maksimum sınırına ulaştığında tetiklenir ve bu nedenle ortam ışığı ADC'nin çıkışını etkiler.
#define	INT_DIE_TEMP_RDY_BIT	1	// Matrisin dahili sıcaklık dönüşümü tamamlandığında, işlemcinin sıcaklık veri kayıtlarını okumasına izin vermek için bu kesinti tetiklenir.
#define	INT_PWR_RDY_BIT			0	// Açılışta veya bir güç kapatma durumundan sonra, VDD besleme gerilimi düşük gerilim engelleme geriliminden (UVLO) gerilime değiştiğinde
									// UVLO voltajını aşarsa, modülün açık ve çalışmaya hazır olduğunu bildiren bir güç kesintisi tetiklenir. veri topla.

//
//	FIFO Configuration (0x08)
//
#define FIFO_CONF_SMP_AVE_BIT 			7
#define FIFO_CONF_SMP_AVE_LENGHT 		3
#define FIFO_CONF_FIFO_ROLLOVER_EN_BIT 	4
#define FIFO_CONF_FIFO_A_FULL_BIT 		3
#define FIFO_CONF_FIFO_A_FULL_LENGHT 	4

// örnek ortalama (SMP_AVE)
#define FIFO_SMP_AVE_1		0	// ortalaması alınmadan
#define FIFO_SMP_AVE_2		1
#define FIFO_SMP_AVE_4		2
#define FIFO_SMP_AVE_8		3
#define FIFO_SMP_AVE_16		4
#define FIFO_SMP_AVE_32		5

//
//	Mode Configuration (0x09)
//
#define MODE_SHDN_BIT		7
#define MODE_RESET_BIT		6
#define MODE_MODE_BIT		2
#define MODE_MODE_LENGTH	3

// modlar
#define MODE_HEART_RATE_MODE	2	// Kalp atış hızı modu Yalnızca kırmızı
#define MODE_SPO2_MODE			3	// SpO2 Modu Kırmızı ve IR
#define MODE_MULTI_LED_MODE		7	// Çoklu LED Modu Kırmızı ve IR

//
//	SpO2 Configuration (0x0A)
//
#define SPO2_CONF_ADC_RGE_BIT		6
#define SPO2_CONF_ADC_RGE_LENGTH	2
#define SPO2_CONF_SR_BIT			4
#define SPO2_CONF_SR_LENGTH			3
#define SPO2_CONF_LED_PW_BIT		1
#define SPO2_CONF_LED_PW_LENGTH		2

// Bu kayıt, SpO2 sensörünün ADC'sinin tüm aralığını ayarlar.
#define	SPO2_ADC_RGE_2048	0
#define	SPO2_ADC_RGE_4096	1
#define	SPO2_ADC_RGE_8192	2
#define	SPO2_ADC_RGE_16384	3

// SpO 2 Numune hızı kontrolü
#define	SPO2_SAMPLE_RATE_50		0
#define	SPO2_SAMPLE_RATE_100	1
#define	SPO2_SAMPLE_RATE_200	2
#define	SPO2_SAMPLE_RATE_400	3
#define	SPO2_SAMPLE_RATE_800	4
#define	SPO2_SAMPLE_RATE_1000	5
#define	SPO2_SAMPLE_RATE_1600	6
#define	SPO2_SAMPLE_RATE_3200	7

#if(MAX30102_SAMPLES_PER_SECOND == 50)
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_50
#elif((MAX30102_SAMPLES_PER_SECOND == 100))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_100
#elif((MAX30102_SAMPLES_PER_SECOND == 200))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_200
#elif((MAX30102_SAMPLES_PER_SECOND == 400))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_400
#elif((MAX30102_SAMPLES_PER_SECOND == 800))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_800
#elif((MAX30102_SAMPLES_PER_SECOND == 1000))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_1000
#elif((MAX30102_SAMPLES_PER_SECOND == 1600))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_1600
#elif((MAX30102_SAMPLES_PER_SECOND == 3200))
#define SPO2_SAMPLE_RATE SPO2_SAMPLE_RATE_3200
#else
#error "Wrong Sample Rate value. Use 50, 100, 200, 400, 800, 1000, 1600 or 3200."
#endif

// LED darbe genişliği kontrolü ve ADC çözünürlüğü
#define	SPO2_PULSE_WIDTH_69			0
#define	SPO2_PULSE_WIDTH_118		1
#define	SPO2_PULSE_WIDTH_215		2
#define	SPO2_PULSE_WIDTH_411		3


// Slots
#define	SLOT_RED_LED 		0x01
#define	SLOT_IR_LED			0x02
#define	SLOT_GREEN_LED		0x03

//
//	Functions
//
MAX30102_STATUS Max30102_Init(I2C_HandleTypeDef *i2c);
/*
	DİKKAT!!! genellikle Max30102 ve Max30105 modüllerinde, ir_led ve red_led LED'lerinin kanalları karıştırılır,
	bu nedenle veri okumaları doğru değilse, MAX30102.c ve MAX30102.h dosyalarındaki Max30102_ReadFifo fonksiyonunda
	giriş parametreleri degistirilmelidir. . Aşağıda bir örnek verilmiştir:

	MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_red_led, volatile uint32_t *pun_ir_led){}	// Secenek 1
	MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_ir_led, volatile uint32_t *pun_red_led){}	// Secenek 2
*/
MAX30102_STATUS Max30102_ReadFifo(volatile uint32_t *pun_ir_led, volatile uint32_t *pun_red_led);
MAX30102_STATUS Max30102_WriteReg(uint8_t uch_addr, uint8_t uch_data);
MAX30102_STATUS Max30102_ReadReg(uint8_t uch_addr, uint8_t *puch_data);
//
//	Interrupts
//
MAX30102_STATUS Max30102_ReadInterruptStatus(uint8_t *Status);
MAX30102_STATUS Max30102_SetIntAlmostFullEnabled(uint8_t Enable);
MAX30102_STATUS Max30102_SetIntFifoDataReadyEnabled(uint8_t Enable);
MAX30102_STATUS Max30102_SetIntAmbientLightCancelationOvfEnabled(uint8_t Enable);

MAX30102_STATUS Max30102_SetIntInternalTemperatureReadyEnabled(uint8_t Enable);


void Max30102_InterruptCallback(void);
//
//	FIFO Configuration
//
MAX30102_STATUS Max30102_FifoWritePointer(uint8_t Address);
MAX30102_STATUS Max30102_FifoOverflowCounter(uint8_t Address);
MAX30102_STATUS Max30102_FifoReadPointer(uint8_t Address);
MAX30102_STATUS Max30102_FifoSampleAveraging(uint8_t Value);
MAX30102_STATUS Max30102_FifoRolloverEnable(uint8_t Enable);
MAX30102_STATUS Max30102_FifoAlmostFullValue(uint8_t Value); // 17-32 samples ready in FIFO
//
//	Mode Configuration
//
MAX30102_STATUS Max30102_ShutdownMode(uint8_t Enable);
MAX30102_STATUS Max30102_Reset(void);
MAX30102_STATUS Max30102_SetMode(uint8_t Mode);
MAX30102_STATUS Max30102_Slot(uint8_t slotNumber, uint8_t Value);
//
//	SpO2 Configuration
//
MAX30102_STATUS Max30102_SpO2AdcRange(uint8_t Value);
MAX30102_STATUS Max30102_SpO2SampleRate(uint8_t Value);
MAX30102_STATUS Max30102_SpO2LedPulseWidth(uint8_t Value);
//
//	LEDs Pulse Amplitute Configuration
//
MAX30102_STATUS Max30102_Led1PulseAmplitude(uint8_t Value);
MAX30102_STATUS Max30102_Led2PulseAmplitude(uint8_t Value);
MAX30102_STATUS Max30102_Led3PulseAmplitude(uint8_t Value);
//
//	Usage functions
//
MAX30102_STATUS Max30102_IsFingerOnSensor(void);

void Max30102_Task(void);
int32_t Max30102_GetHeartRate(void);
int32_t Max30102_GetSpO2Value(void);
float Max30102_ReadTemperature(void);

#ifdef __cplusplus
}
#endif

#endif /* _MAX30102_H_ */
