/* USER CODE BEGIN Header */
//********************************************************************************************************************/
// Summary    	:
// Date       	: 12.05.2023
// Author     	: Project Group
// Version    	: 1.00 - 10.01.2023  :Initial version
// Sources    	:
// Libraries  	:
// Notes      	: MAX30102 Ayarlar:
//				: I2C1 modulunu aktif ediyoruz. I2C Fast Mode (400kHz),
//				: MAX30102'den kesme alabilmek için bir pini GPIO_EXTIx olarak ayarliyoruz.
//			  	: Bu pinin modunu Falling Edge ve pull-up olarak konfigure ediyoruz.
//				: Interrupt alinabilmesi ici NVIC kesme onay kutusu isaretlenmis olmalidir.
//				: DS18B20 Ayarlar:
//				: DS1820B.c dosyasinda DQ sinyalini bagladigimiz port ve pin'i tanimliyoruz.
//				: LCD Display ayarları:
//				: I2C3 modülünü aktif ediyoruz. I2C Standart Mode (100kHz),
//				: liquidcrystal_i2c.c dosyasında "extern I2C_HandleTypeDef hi2c3" olarak düzenliyoruz.
//				: liquidcrystal_i2c.c dosyasında ExpanderWrite fonksiyonunuda da HAL_I2C_Master_Transmit
//				: fonksiyonu ilk parametresini &hi2c3 olarak degistiriyoruz.
//********************************************************************************************************************/
//********************************************************************************************************************/
// PD0 -> PR_PATIENT_OCCUPANCY
// PD1 -> PR_EMERGENCY_CALL


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MAX30102.h"
#include "DS18B20.h"
#include "EEPROM.h"
#include "liquidcrystal_i2c.h"

#include "string.h"
#include "stdlib.h"																					// atoi fonksiyonu kullanılacaksa bu kütüphane dosyasına ihtiyaç var (şu an için kullanmadık).
#include "stdio.h"																					// printf, sprintf fonksiyonları kullanılacaksa bu kütüphane dosyasına ihtiyaç var.
#include "stdbool.h"																				// boolean type tanımları için...

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/*------ Lag Free Delay Variables ------*/
uint32_t last_time = 0;
uint32_t present_time = 0;
uint16_t process_period_timer = 0;																	// Analog Okuma, DHT ve UART için işlemlerin uygylanma periyodunu belirleyen zamanlama değişkeni

/*--------- Debounce variables ---------*/
/* Page Button */
uint8_t button_state1;
uint8_t last_button_state1 = 0;
uint32_t last_debounce_time1 = 0;
uint32_t debounce_delay1 = 50;
/* Shift Button */
uint8_t button_state2;
uint8_t last_button_state2 = 0;
uint32_t last_debounce_time2 = 0;
uint32_t debounce_delay2 = 50;
/* Acknowledge Button */
uint8_t button_state3;
uint8_t last_button_state3 = 0;
uint32_t last_debounce_time3 = 0;
uint32_t debounce_delay3 = 50;

/*--- MAX30102 Variables ---*/
uint8_t finger_on = 0;
int32_t hr_old = 1, hr_new = 1;
int32_t sp_old = 1, sp_new = 1;
float T = 0.0;

/*---- DS18B20 Variables ---*/
DS_DataTypedef DS_Data;
uint8_t temp_integer = 0;																			// Sıcaklık verisinin tam sayı kısmı,
uint8_t temp_sign = 0;
float temp_fractional = 0.0;
float temp_full = 0.0;

/*--- LCD Display Variables ---*/
char char_array[16];																				// sprintf fonksiyonunda string oluşturmak için gerekli char dizi.
char row0_cursor = '>';
char row1_cursor = ' ';
uint8_t row0_value = 0;
uint8_t row1_value = 0;
uint8_t page_number = 0;																			// Display görüntüleri arasındaki geçiş için kullanılan counter değişkeni
uint8_t shift_counter = 0;																			// Satırlar arasında geçiş için kullanılan counter değişkeni

/*--- ADC Threshold Set Variables ---*/
uint16_t hr_adc_threshold = 0;																		// Potansiyometre ile ayarlanan Heart Rate değeri
uint16_t bt_adc_threshold = 0;																		// Potansiyometre ile ayarlanan Body Temperature değeri

/*--- Occupancy Detector and CAll Button Variables ---*/
bool emergency_button_flag = false;
uint8_t po_button_state;
uint8_t po_last_button_state = 0;
uint32_t po_last_debounce_time = 0;
uint32_t po_debounce_delay = 50;

/*--- EEPROM Variables ---*/
uint8_t threshold_values[4];

/*------ Buzzer Control Variables ------*/
uint16_t buzzer_timing_counter1 = 0;																// Buzzer'ın kesikli veya kısa süreli beep yapması için Timer kesmesi içinde artırılan zamanlama değişkeni
uint16_t buzzer_timing_counter2 = 0;
bool alarm_buzzer_enable = false;
bool acknowledge_buzzer_enable = false;

/*---- UART Communication Variables ----*/
uint8_t transmit_data[2] = {0, 0};
uint8_t receive_data[2] = {0, 0}; 																	// Veri almıyoruz.o sebeple kullanılmıyor.
bool command1_flag = false;
bool command2_flag = false;
bool command3_flag = false;
bool command4_flag = false;
bool command5_flag = false;
bool command6_flag = false;

/*--- General Variables ---*/
bool clear_once = false;
bool one_shot_hr = true;
bool one_shot_bt = true;
bool system_alarm_enable = true;																	// Sistemin alarm fonksiyonları devrede;
uint16_t test_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*================= GPIO Input olarak konfigure edilmis pinlerden gelen kesmeler =================*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == PR_MAX30102_INT_Pin){
		Max30102_InterruptCallback();
	}

	if (GPIO_Pin == PR_EMERGENCY_CALL_BUT_Pin){														// Kesme Emergency Call Button kaynakli ise;
		emergency_button_flag = true;
	}
}

/*========================= Her 1 ms'de bir kere calisan Timer 6 kesmesi =========================*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance){															// Gelen Timer kesemsi Timer6 kaynakli ise;
		if (alarm_buzzer_enable == true){															// Alarm aktif olduysa;
			if (buzzer_timing_counter1 == 500){														// 500 ms sure ile
				HAL_GPIO_TogglePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin);								// Buzzer'in bagli oldugu pini 1 ve 0 yap.
				buzzer_timing_counter1 = 0;
			}
			buzzer_timing_counter1 ++;
		}

		if (acknowledge_buzzer_enable == true){														// Acknowledge butonuna basildiysa;
			if (buzzer_timing_counter2 == 200){														// 200 ms sure sonunda
				HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 0);							// Buzzer'i sustur.
				buzzer_timing_counter2 = 0;
				acknowledge_buzzer_enable = false;													// 1 kes beep yapmasini istediğimizden flag'i false yaptik.
			}
			buzzer_timing_counter2 ++;
		}
	}
}

/*============ ESP-01'den UART verisi geldiginde tetiklenen interrupt (kullanilmiyor) ============*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART3){
	/* Alınan veri burada isleniyor */
	}
	// HAL_UART_Receive_IT(&huart3, receive_data, 2);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  //EEPROM_Write_NUM (0, 0, dataw);
  //HAL_Delay(50);
  //datar = EEPROM_Read_NUM (0, 0);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_I2C3_Init();

  MX_USART3_UART_Init();

  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  LCD_Init(2);																						// LCD display 2 satir oldugu icin fonksiyon parametresine 2 degerini yazdik.
  LCD_Clear();

  Max30102_Init(&hi2c1);																			// Aktif edilen I2C1 struct'ının adresini veriyoruz.
  Max30102_ShutdownMode(1);																			// Uyku moduna aldik.
  HAL_Delay(1000);
  Max30102_ShutdownMode(0);																			// Uyku modundan ciktik.
  HAL_Delay(1000);

  DS_GetData(&DS_Data);																				// Sicaklik sensorunun olctugu veriyi aliyoruz.
  temp_integer = DS_Data.T_Integer;																	// Elde edilen verinin tam sayi kismi
  HAL_Delay(100);

  EEPROM_Read(0, 0, threshold_values, 4);															// Program baslarken EEPROM'dan set degerlerini okuyoruz.

  HAL_TIM_Base_Start_IT(&htim6);																	// Timer 6'yi baslattik.

  HAL_GPIO_WritePin(PR_SYSTEM_ON_LED_GPIO_Port, PR_SYSTEM_ON_LED_Pin, 1);							// Baslangicta sistemin alarm fonksiyonlarinin aktif oldugunu gosteren yesil LED'i aktif ediyoruz.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	Max30102_Task();																				// Library fonksiyonlarinin calisabilmesi icin while icerisinde surekli islenmelidir.
	//char Buffer[32];

	/*-------------------------- SCHEDULED TASKS IN LAG FREE DELAY Begin -------------------------*/
	present_time = HAL_GetTick();																	// HAL_GetTick() fonksiyonu sistem basladiginden itibaren gecen sureyi milisaniye olarak dondurur.
	if (present_time - last_time > 1){
		last_time = present_time;

		/* Analog value reading */
		if (process_period_timer == 0){
			/*--- Heart Rate Potentiometer ---*/
			HAL_ADC_Start(&hadc1);																	// ADC modulunu baslatarak analog'tan dijitale cevrim surecine start verdik.
			while(HAL_ADC_PollForConversion(&hadc1, 300));											// Cevrim bitene kadar bekliyoruz
			hr_adc_threshold = (int)(HAL_ADC_GetValue(&hadc1)/2);
			HAL_ADC_Stop(&hadc1);																	// ADC modulunu durdurduk.

			/*--- Body Temperature Potentiometer ---*/
			HAL_ADC_Start(&hadc2);
			while(HAL_ADC_PollForConversion(&hadc2, 300));
			bt_adc_threshold = HAL_ADC_GetValue(&hadc2);
			HAL_ADC_Stop(&hadc2);
		}


		/* DS18B20 reading */
		if (process_period_timer == 100){
			DS_GetData(&DS_Data);																	// Sicaklik sensorunun olctugu veriyi aliyoruz.
			temp_integer = DS_Data.T_Integer;														// Elde edilen verinin tam sayi kismi
			temp_fractional = DS_Data.T_Fractional;													// Elde edilen verinin virgulden sonraki (ondalikli) kismi
			temp_full = DS_Data.T_Reel;																// Verinin butun hali (float type variable)
			temp_sign = DS_Data.Sign;																// Sicaklik verisinin isareti (> 0 ise 1, < 0 ise 0)
		}

		/* MAX30102 reading */
		if (process_period_timer == 300){
			finger_on = Max30102_IsFingerOnSensor();												// Sensore parmak yaklasip yaklasmadigina bakiyoruz.
			hr_new = Max30102_GetHeartRate();														// Kalp atis hizini aliyoruz.
			sp_new = Max30102_GetSpO2Value();														// Oksijen oranini aliyoruz (O2 saturasyon - max: 100)
			if (finger_on == MAX30102_OK){ 															// Parmak var, ve oncekilerden farkli bir veri elde edilmisse;
				if (hr_new > 30){
					if ((hr_old != hr_new) || (sp_old != sp_new)){
						hr_old = hr_new;															// Nabiz degerini tekrar okuyarak yeni bir degiskene aldik.
						sp_old = sp_new;															// Oksijen oranini tekrar okuyarak yeni bir degiskene aldik.
					}
				}
			}
		}

		/* Sending "Command1: Heart Rate alarm LED on/off data" via UART */
		if (process_period_timer == 1200){
			if (command1_flag){
				transmit_data[0] = 0x01;															// Command
				transmit_data[1] = 0x01;															// LED on
				HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
				command1_flag = false;
			}
		}

		/* Sending "Command2: Heart Rate value" via UART */
		if (process_period_timer == 1300){
			transmit_data[0] = 0x02;																// Command
			transmit_data[1] = (uint8_t)hr_old;														// Heart Rate value
			HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
		}

		/* Sending "Command3: Body Temp. alarm LED on/off data" via UART */
		if (process_period_timer == 1400){
			if (command3_flag){
				transmit_data[0] = 0x03;															// Command
				transmit_data[1] = 0x01;															// LED on
				HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
				command3_flag = false;
			}
		}

		/* Sending "Command5: Emergency Call alarm LED on/off data" via UART*/
		if (process_period_timer == 1500){
			if (command5_flag){
				transmit_data[0] = 0x05;															// Command
				transmit_data[1] = 0x01;															// LED on
				HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
				command5_flag = false;
			}
		}

		/* Sending "Command6: Pat. Occu. warning LED on/off data" via UART */
		if (process_period_timer == 1600){
			if (command6_flag){
				transmit_data[0] = 0x06;																// Command
				transmit_data[1] = 0x01;																// LED on
				HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
				command6_flag = false;
			}
		}

		/* Display Control Lines */
		if (process_period_timer == 1700){
			/* Heart Rate and Temperature screen */
			if (page_number == 0){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}
				if ((hr_old > 0) && (finger_on == MAX30102_OK)){									// Sensor üzerinde parmak varsa ve hatalı olcum yapilmiyorsa (örn: -999)
					sprintf(char_array, "Hr.Rate:%3d bpm", (int)hr_old);							// %3d: 3 basamaklı decimal sayi ve saga yasli
					LCD_SetCursor(0, 0);															// LCD_SetCursor(column, row)
					LCD_PrintStr(char_array);														// Birlestirilen karakterleri ekrana basiyoruz.
				}
				else{																				// Parmak yoksa veya hatali okuma gerceklesmisse...
					LCD_SetCursor(0, 0);															// LCD_SetCursor(column, row)
					LCD_PrintStr("Hr.Rate: Error! ");												// Ekrana "Error" mesaji basiyoruz.
				}
				sprintf(char_array, "Bd.Temp:%2d.%d %cC",
						(int)temp_integer,
						(int)(temp_fractional * 10),
						223);
				LCD_SetCursor(0, 1);
				LCD_PrintStr(char_array);
			}

			/* SpO2 screen*/
			if (page_number == 1){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}
				if ((sp_old > 0) && (finger_on == MAX30102_OK)){
					sprintf(char_array, "SpO2 Level: %3d", (int)sp_old);
					LCD_SetCursor(0, 0);
					LCD_PrintStr(char_array);
				}
				else{
					LCD_SetCursor(0, 0);															// LCD_SetCursor(column, row)
					LCD_PrintStr("SpO2 Level:Error");
				}

				LCD_SetCursor(0, 1);																// LCD_SetCursor(column, row)
				LCD_PrintStr("----------------");
			}

			/* Heart Rate threshold max. min. adjustment screen */
			if (page_number == 2){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				if (shift_counter == 0){
					row0_cursor = ' ';
					row1_cursor = ' ';
					row0_value = (int)threshold_values[0];
					row1_value = (int)threshold_values[1];
				}

				if (shift_counter == 1){
					row0_cursor = '>';
					row1_cursor = ' ';
					row0_value = hr_adc_threshold;
					row1_value = (int)threshold_values[1];
				}

				if (shift_counter == 2){
					row0_cursor = ' ';
					row1_cursor = '>';
					row0_value = (int)threshold_values[0];
					row1_value = hr_adc_threshold;
				}

				sprintf(char_array, "HR.Th.Min.:%c%3d", row0_cursor, row0_value);
				LCD_SetCursor(0, 0);
				LCD_PrintStr(char_array);

				sprintf(char_array, "HR.Th.Max.:%c%3d", row1_cursor, row1_value);
				LCD_SetCursor(0, 1);
				LCD_PrintStr(char_array);
			}

			/* Body Temperature threshold max. min. adjustment screen  */
			if (page_number == 3){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				if (shift_counter == 0){
					row0_cursor = ' ';
					row1_cursor = ' ';
					row0_value = (int)threshold_values[2];
					row1_value = (int)threshold_values[3];
				}

				if (shift_counter == 1){
					row0_cursor = '>';
					row1_cursor = ' ';
					row0_value = bt_adc_threshold;
					row1_value = (int)threshold_values[3];
				}

				if (shift_counter == 2){
					row0_cursor = ' ';
					row1_cursor = '>';
					row0_value = (int)threshold_values[2];
					row1_value = bt_adc_threshold;
				}

				sprintf(char_array, "BT.Th.Min.:%c%3d", row0_cursor, row0_value);
				LCD_SetCursor(0, 0);
				LCD_PrintStr(char_array);

				sprintf(char_array, "BT.Th.Max.:%c%3d", row1_cursor, row1_value);
				LCD_SetCursor(0, 1);
				LCD_PrintStr(char_array);
			}

			/* Alarm screens  */
			if (page_number == 10){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				LCD_SetCursor(0, 0);
				LCD_PrintStr(" LOW HEART RATE ");

				LCD_SetCursor(0, 1);
				LCD_PrintStr("     ALARM!     ");
			}

			if (page_number == 11){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				LCD_SetCursor(0, 0);
				LCD_PrintStr("HIGH HEART RATE ");

				LCD_SetCursor(0, 1);
				LCD_PrintStr("     ALARM!     ");
			}

			if (page_number == 12){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				LCD_SetCursor(0, 0);
				LCD_PrintStr("LOW TEMPERATURE ");

				LCD_SetCursor(0, 1);
				LCD_PrintStr("     ALARM!     ");
			}

			if (page_number == 13){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				LCD_SetCursor(0, 0);
				LCD_PrintStr("HIGH TEMPERATURE");

				LCD_SetCursor(0, 1);
				LCD_PrintStr("     ALARM!     ");
			}

			if (page_number == 14){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				LCD_SetCursor(0, 0);
				LCD_PrintStr("   EMERGENCY    ");

				LCD_SetCursor(0, 1);
				LCD_PrintStr("     ALARM!     ");
			}

			if (page_number == 15){
				if (clear_once == true){
					LCD_Clear();
					clear_once = false;
				}

				LCD_SetCursor(0, 0);
				LCD_PrintStr(" PATIENT IS NOT ");

				LCD_SetCursor(0, 1);
				LCD_PrintStr("     IN BED     ");
			}
		}

		/* Period Completion */
		if (process_period_timer == 1800){
			process_period_timer = -1;
			test_counter ++;
		}
		process_period_timer ++;
	}
	/*--------------------------- SCHEDULED TASKS IN LAG FREE DELAY End --------------------------*/

	/*----------------------------- PERIPHERAL DEVICES CONTROL Begin -----------------------------*/

	/* Heart Rate threshold values control */
	if ((finger_on == MAX30102_OK) && system_alarm_enable){
		if (hr_old < threshold_values[0]){
			if (one_shot_hr){
				HAL_GPIO_WritePin(PR_HEART_RATE_LED_GPIO_Port, PR_HEART_RATE_LED_Pin, 1);
				buzzer_timing_counter1 = 0;
				alarm_buzzer_enable = true;
				page_number = 10;
				clear_once = true;
				command1_flag = true;
				one_shot_hr = false;
			}
		}
		else if (hr_old > threshold_values[1]){
			if (one_shot_hr){
				HAL_GPIO_WritePin(PR_HEART_RATE_LED_GPIO_Port, PR_HEART_RATE_LED_Pin, 1);
				buzzer_timing_counter1 = 0;
				alarm_buzzer_enable = true;
				page_number = 11;
				clear_once = true;
				command1_flag = true;
				one_shot_hr = false;
			}
		}
	}

	/* Body Temperature threshold values control */
	if (system_alarm_enable){
		if (temp_integer < threshold_values[2]){
			if (one_shot_bt){
				HAL_GPIO_WritePin(PR_BODY_TEMPERATURE_LED_GPIO_Port, PR_BODY_TEMPERATURE_LED_Pin, 1);
				buzzer_timing_counter1 = 0;
				alarm_buzzer_enable = true;
				page_number = 12;
				clear_once = true;
				command3_flag = true;
				one_shot_bt = false;
			}
		}
		if (temp_integer >= threshold_values[3]){
			if (one_shot_bt){
				HAL_GPIO_WritePin(PR_BODY_TEMPERATURE_LED_GPIO_Port, PR_BODY_TEMPERATURE_LED_Pin, 1);
				buzzer_timing_counter1 = 0;
				alarm_buzzer_enable = true;
				page_number = 13;
				clear_once = true;
				command3_flag = true;
				one_shot_bt = false;
			}
		}
	}

	/* Emergency Call button signal processing & action */
	if (system_alarm_enable){
		if (emergency_button_flag){
			HAL_GPIO_WritePin(PR_EMERGENCY_CALL_LED_GPIO_Port, PR_EMERGENCY_CALL_LED_Pin, 1);
			buzzer_timing_counter1 = 0;
			alarm_buzzer_enable = true;
			page_number = 14;
			clear_once = true;
			command5_flag = true;
			emergency_button_flag = false;
		}
	}

	/* Patient Occupancy Detector signal processing & action (Debounced) */
	if (system_alarm_enable){
		uint8_t po_reading;
		po_reading = HAL_GPIO_ReadPin(PR_PATIENT_OCCUPANCY_DET_GPIO_Port, PR_PATIENT_OCCUPANCY_DET_Pin);
		if (po_reading != po_last_button_state){
			po_last_debounce_time = HAL_GetTick();
		}
		if ((HAL_GetTick() - po_last_debounce_time) > po_debounce_delay){
			if (po_reading != po_button_state){
				po_button_state = po_reading;
				/*--------------- Job Section Begin ---------------*/
				if (po_button_state == 0){															// Buton (pull-up bağlı ise) basılı iken çalışan blok
					/*** Pressed ***/

				}
				else{																				// Buton (pull-up bağlı ise) basılı değil iken çalışan blok
					/*** Released ***/
					HAL_GPIO_WritePin(PR_PATIENT_OCCUPANCY_LED_GPIO_Port, PR_PATIENT_OCCUPANCY_LED_Pin, 1);
					buzzer_timing_counter1 = 0;
					alarm_buzzer_enable = true;
					page_number = 15;
					clear_once = true;
					command6_flag = true;
				}
				/*---------------- Job Section End ----------------*/
			}
		}
		po_last_button_state = po_reading;
	}
	/*------------------------------ PERIPHERAL DEVICES CONTROL End ------------------------------*/

	/*----------------------------- SYSTEM RESET BUTTON CONTROL Begin ----------------------------*/
	if (HAL_GPIO_ReadPin(PR_SYSTEM_RESET_BUTTON_GPIO_Port, PR_SYSTEM_RESET_BUTTON_Pin)){
		HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 0);									// Buzzer pin'i aktif kalmissa off yap.
		HAL_GPIO_WritePin(PR_SYSTEM_ON_LED_GPIO_Port, PR_SYSTEM_ON_LED_Pin, 0);						// Sistemin aktif oldugunu gosteren LED'i sondur.
		HAL_GPIO_WritePin(PR_HEART_RATE_LED_GPIO_Port, PR_HEART_RATE_LED_Pin, 0);
		HAL_GPIO_WritePin(PR_BODY_TEMPERATURE_LED_GPIO_Port, PR_BODY_TEMPERATURE_LED_Pin, 0);
		HAL_GPIO_WritePin(PR_EMERGENCY_CALL_LED_GPIO_Port, PR_EMERGENCY_CALL_LED_Pin, 0);
		HAL_GPIO_WritePin(PR_PATIENT_OCCUPANCY_LED_GPIO_Port, PR_PATIENT_OCCUPANCY_LED_Pin, 0);

		system_alarm_enable = false;
		alarm_buzzer_enable = false;
		acknowledge_buzzer_enable = false;

		one_shot_hr = true;
		one_shot_bt = true;
		page_number = 0;
		clear_once = true;

		transmit_data[0] = 0x01;																// Command
		transmit_data[1] = 0x00;																// LED on
		HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
		HAL_Delay(200);

		transmit_data[0] = 0x03;																// Command
		transmit_data[1] = 0x00;																// LED on
		HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
		HAL_Delay(200);

		transmit_data[0] = 0x05;																// Command
		transmit_data[1] = 0x00;																// LED on
		HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
		HAL_Delay(200);

		transmit_data[0] = 0x06;																// Command
		transmit_data[1] = 0x00;																// LED on
		HAL_UART_Transmit(&huart3, transmit_data, 2, 200);
		HAL_Delay(200);
	}
	/*------------------------------ SYSTEM RESET BUTTON CONTROL End -----------------------------*/

	/*--------------------- CONTROL & MONITORING LAYER BUTTONS CONTROL Begin ---------------------*/
	/* Page Button (Debounced) */
	uint8_t reading1;
	reading1 = HAL_GPIO_ReadPin(PR_PAGE_BUTTON_GPIO_Port, PR_PAGE_BUTTON_Pin);
	if (reading1 != last_button_state1){
		last_debounce_time1 = HAL_GetTick();
	}
	if ((HAL_GetTick() - last_debounce_time1) > debounce_delay1){
		if (reading1 != button_state1){
			button_state1 = reading1;
			/*--------------- Job Section Begin ---------------*/
			if (button_state1 == 0){																// Buton (pull-up bağlı ise) basılı iken çalışan blok
				/*** Pressed ***/
				page_number ++;
				clear_once = true;
				shift_counter = 0;
				if (page_number == 4){
					page_number = 0;
				}
			}
			else{																					// Buton (pull-up bağlı ise) basılı değil iken çalışan blok
				/*** Released ***/
			}
			/*---------------- Job Section End ----------------*/
		}
 	}
	last_button_state1 = reading1;

	/* Shift Button (Debounced) */
	uint8_t reading2;
	reading2 = HAL_GPIO_ReadPin(PR_SHIFT_BUTTON_GPIO_Port, PR_SHIFT_BUTTON_Pin);
	if (reading2 != last_button_state2){
		last_debounce_time2 = HAL_GetTick();
	}
	if ((HAL_GetTick() - last_debounce_time2) > debounce_delay2){
		if (reading2 != button_state2){
			button_state2 = reading2;
			/*--------------- Job Section Begin ---------------*/
			if (button_state2 == 0){																// Buton (pull-up bağlı ise) basılı iken çalışan blok
				/*** Pressed ***/
				shift_counter ++;
				if (shift_counter == 3){
					shift_counter = 0;
				}
			}
			else{																					// Buton (pull-up bağlı ise) basılı değil iken çalışan blok
				/*** Released ***/
			}
			/*---------------- Job Section End ----------------*/
		}
	}
	last_button_state2 = reading2;

	/* Acknowledge Button (Debounced) */
	uint8_t reading3;
	reading3 = HAL_GPIO_ReadPin(PR_ACKNOWLEDGE_BUTTON_GPIO_Port, PR_ACKNOWLEDGE_BUTTON_Pin);
	if (reading3 != last_button_state3){
		last_debounce_time3 = HAL_GetTick();
	}
	if ((HAL_GetTick() - last_debounce_time3) > debounce_delay3){
		if (reading3 != button_state3){
			button_state3 = reading3;
			/*--------------- Job Section Begin ---------------*/
			if (button_state3 == 0){																// Buton (pull-up bağlı ise) basılı iken çalışan blok
				/*** Pressed ***/
				if ((page_number == 2) && (shift_counter == 1)){
					threshold_values[0] = hr_adc_threshold;
					EEPROM_Write(0, 0, (threshold_values + 0), 1);
					shift_counter = 2;
					/* Beep */
					acknowledge_buzzer_enable = true;
					buzzer_timing_counter2 = 0;
					HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 1);
				}
				else if ((page_number == 2) && (shift_counter == 2)){
					threshold_values[1] = hr_adc_threshold;
					EEPROM_Write(0, 1, (threshold_values + 1), 1);
					shift_counter = 0;
					/* Beep */
					acknowledge_buzzer_enable = true;
					buzzer_timing_counter2 = 0;
					HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 1);
				}
				else if ((page_number == 3) && (shift_counter == 1)){
					threshold_values[2] = bt_adc_threshold;
					EEPROM_Write(0, 2, (threshold_values + 2), 1);
					shift_counter = 2;
					/* Beep */
					acknowledge_buzzer_enable = true;
					buzzer_timing_counter2 = 0;
					HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 1);
				}
				else if ((page_number == 3) && (shift_counter == 2)){
					threshold_values[3] = bt_adc_threshold;
					EEPROM_Write(0, 3, (threshold_values + 3), 1);
					shift_counter = 0;
					/* Beep */
					acknowledge_buzzer_enable = true;
					buzzer_timing_counter2 = 0;
					HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 1);
				}
				else{
					if (system_alarm_enable){
						HAL_GPIO_WritePin(PR_SYSTEM_ON_LED_GPIO_Port, PR_SYSTEM_ON_LED_Pin, 0);		// Sistemin aktif oldugunu gosteren LED'i sondur.
						system_alarm_enable = false;
						acknowledge_buzzer_enable = true;
						buzzer_timing_counter2 = 0;
						HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 1);
					}
					else{
						HAL_GPIO_WritePin(PR_SYSTEM_ON_LED_GPIO_Port, PR_SYSTEM_ON_LED_Pin, 1);		// Sistemin aktif oldugunu gosteren LED'i sondur.
						system_alarm_enable = true;
						acknowledge_buzzer_enable = true;
						buzzer_timing_counter2 = 0;
						HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, 1);
					}
				}
			}
			else{																					// Buton (pull-up bağlı ise) basılı değil iken çalışan blok
				/*** Released ***/
			}
			/*---------------- Job Section End ----------------*/
		}
	}
	last_button_state3 = reading3;
	/*---------------------- CONTROL & MONITORING LAYER BUTTONS CONTROL End ----------------------*/
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_6B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16799;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 4;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PR_BUZZER_GPIO_Port, PR_BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |PR_SYSTEM_ON_LED_Pin|PR_HEART_RATE_LED_Pin|PR_BODY_TEMPERATURE_LED_Pin|Audio_RST_Pin
                          |PR_EMERGENCY_CALL_LED_Pin|PR_PATIENT_OCCUPANCY_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PR_PAGE_BUTTON_Pin PR_SHIFT_BUTTON_Pin PR_ACKNOWLEDGE_BUTTON_Pin */
  GPIO_InitStruct.Pin = PR_PAGE_BUTTON_Pin|PR_SHIFT_BUTTON_Pin|PR_ACKNOWLEDGE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin PR_BUZZER_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|PR_BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PR_SYSTEM_RESET_BUTTON_Pin */
  GPIO_InitStruct.Pin = PR_SYSTEM_RESET_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PR_SYSTEM_RESET_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PR_PATIENT_OCCUPANCY_DET_Pin */
  GPIO_InitStruct.Pin = PR_PATIENT_OCCUPANCY_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PR_PATIENT_OCCUPANCY_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PR_EMERGENCY_CALL_BUT_Pin */
  GPIO_InitStruct.Pin = PR_EMERGENCY_CALL_BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(PR_EMERGENCY_CALL_BUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOOT1_Pin PR_DS18B20_DATA_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin|PR_DS18B20_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           PR_SYSTEM_ON_LED_Pin PR_HEART_RATE_LED_Pin PR_BODY_TEMPERATURE_LED_Pin Audio_RST_Pin
                           PR_EMERGENCY_CALL_LED_Pin PR_PATIENT_OCCUPANCY_LED_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |PR_SYSTEM_ON_LED_Pin|PR_HEART_RATE_LED_Pin|PR_BODY_TEMPERATURE_LED_Pin|Audio_RST_Pin
                          |PR_EMERGENCY_CALL_LED_Pin|PR_PATIENT_OCCUPANCY_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PR_MAX30102_INT_Pin */
  GPIO_InitStruct.Pin = PR_MAX30102_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PR_MAX30102_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
