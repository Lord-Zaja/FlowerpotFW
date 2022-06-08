/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include "sht3x.h"
#include <stdlib.h>
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

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
volatile char RXBuffer[128];
volatile char RXByte[8];
volatile bool RXDone = false;	// true, pokud nechci poslouchat, false pokud chci nový řádek
char RXLineBuffer[128];
char RXLine[128];
// Kalibrační konstanty teploty
float k0_temp = 0;
float k1_temp = 1;

//volatile bool echo = false;
/*
volatile uint16_t adcDMABuffer[2];	// můžu klidně skladovat více hodnot .. třeba 1000 .. jednotlivé kanalý se pak skládají za sebou podle ranku
volatile int adcConversionComplete = 0;
*/
//1. 2. 1. 2. ..
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void myprintf(const char *fmt, ...);
sht3x_handle_t setupSHT();
void readData(sht3x_handle_t *handle, float *temp, float *hum, uint8_t *wcup, uint8_t *wrez, float *soil1, float *soil2);
bool openConfigFile(uint8_t *mode, uint8_t *hum, uint8_t *temp);
char* removeSpaces(char *str);
bool writeToFile(float temp, float hum, uint8_t wcup, uint8_t wrez, float soil1, float soil2);
void getLine();
uint8_t bcdToDec(uint8_t val);
uint8_t decToBcd(uint8_t val);
bool saveConfig(uint8_t mode, uint8_t temp, uint8_t hum);
void sendMyData(float temp, float hum, uint8_t wcup, uint8_t wrez, float soil1, float soil2);
void sendConfig(uint8_t modeset,uint8_t humset,uint8_t tempset);
void zalij(int time_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float getRawTemp(sht3x_handle_t *handle){
	float temperature,hum;
	sht3x_read_temperature_and_humidity(handle, &temperature, &hum);
	return temperature;
}
bool saveCalibration(float k0, float k1){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);// orange led to indicate data logging
	//save config to config file
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations

	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_mount(NULL, "", 0);
		return false;
	}

	DWORD free_clusters, free_sectors, total_sectors;
	FATFS* getFreeFs;
	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK) {
		myprintf("f_getfree error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		return false;
	}

	//Formula comes from ChaN's documentation
	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	free_sectors = free_clusters * getFreeFs->csize;
	myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

	fres = f_open(&fil, "cal.txt", FA_WRITE | FA_CREATE_ALWAYS);
	if(fres == FR_OK) {
		myprintf("I was able to open 'cal.txt' for writing\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_close(&fil);
		f_mount(NULL, "", 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		return false;
	}

	BYTE readBuf[128];
	memset(readBuf,0,128);

	char *write_buffer = (char*)malloc(128);
	*write_buffer = '\0';
	sprintf(write_buffer+strlen(write_buffer),"k0 = %e \n\r",k0);
	sprintf(write_buffer+strlen(write_buffer),"k1 = %e \n\r\0",k1);

	//Copy in a string
	strncpy((char*)readBuf, write_buffer, strlen(write_buffer));
	UINT bytesWrote;
	myprintf(readBuf);
	fres = f_write(&fil, readBuf, strlen(write_buffer), &bytesWrote);
	if(fres == FR_OK) {
		myprintf("Wrote %i bytes to 'cal.txt'!\r\n", bytesWrote);
	}else {
		myprintf("f_write error (%i)\r\n",fres);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_close(&fil);
		f_mount(NULL, "", 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		return false;
	}
	f_close(&fil);
	f_mount(NULL, "", 0);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);// orange led to indicate data logging
	return true;
}
bool openCalibrationFile(float *k0,float *k1){
	//!!!!!!!!!!!!! POZOR .. v souboru nesmí být na začátku prázdné řádky!!!!!
	// .. teda .. je třeba odzkoušet .. možná jsem to akorát opravil
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);// orange led to indicate SD activity
    FATFS FatFs; 	//Fatfs handle
    FIL fil; 		//File handle
    FRESULT fres; //Result after operations

    //Open the file system
    fres = f_mount(&FatFs, "", 1); //1=mount now
    if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_mount(NULL, "", 0);
		return false;
    }

    DWORD free_clusters, free_sectors, total_sectors;
    FATFS* getFreeFs;
    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
		myprintf("f_getfree error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
  	  	return false;
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;
    myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    fres = f_open(&fil, "cal.txt", FA_READ);
    if (fres != FR_OK) {
		myprintf("f_open error (%i)\r\n");
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_close(&fil);
		f_mount(NULL, "", 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
  	  	return false;
    }
    myprintf("I was able to open 'cal.txt' for reading!\r\n");

    //Read 30 bytes from "test.txt" on the SD card
    BYTE readBuf[30];

    for(int i = 0; i<3; i++){
    	TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);	// přečte celý řádek, pokud nezaplní buffer
		if(rres != 0) {
			myprintf("Read string from 'cal.txt' contents: %s\n\r", readBuf);
			char *data = &readBuf[0];
			char d[strlen(data)];
			strcpy(d,data);
			data = removeSpaces(d);
			strcpy(d,data);
			int delic = strcspn(d,"=");// počet znaků před =

			if (delic != strlen(d)){	// příkaz rozeznán
				char *key = (char*)malloc(delic+1);	// na konci přidám jene znak pro ukončení strungu
				memcpy(key,&d,delic);
				key[delic] = '\0';
				char *value = (char*)malloc(sizeof(d)-delic+1);
				memcpy(value,&d[delic+1],sizeof(d)-delic);
				value[sizeof(d)-delic] = '\0';
				if(strcmp(key,"k0") == 0){
					*k0 = atof(value);
				}else if(strcmp(key, "k1") == 0){
					*k1 = atof(value);
				}else{
					myprintf("\n\rKey \"%s\" not recognized!\n\r",key);
				}
			}else{
				myprintf("\n\rDidnt found \"=\" on line %i",i);
				/*
				f_close(&fil);
				f_mount(NULL, "", 0);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
				return false;
				*/
			}
		} else {
			myprintf("f_gets error (%i)\r\n", fres);
			f_close(&fil);
			f_mount(NULL, "", 0);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
			return false;
		}
    }
    f_close(&fil);
    f_mount(NULL, "", 0);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);// orange led to indicate SD activity
    return true;
}
bool saveConfig(uint8_t mode, uint8_t temp, uint8_t hum){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);// orange led to indicate data logging
	//save config to config file
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations

	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_mount(NULL, "", 0);
		return false;
	}

	DWORD free_clusters, free_sectors, total_sectors;
	FATFS* getFreeFs;
	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK) {
		myprintf("f_getfree error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		return false;
	}

	//Formula comes from ChaN's documentation
	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	free_sectors = free_clusters * getFreeFs->csize;
	myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

	fres = f_open(&fil, "config.txt", FA_WRITE | FA_CREATE_ALWAYS);
	if(fres == FR_OK) {
		/*fres = f_lseek(&fil,f_size(&fil));	// ukáže na konec souboru
		if(fres != FR_OK){
			myprintf("f_lseek error (%i)\r\n", fres);
			f_close(&fil);
			f_mount(NULL, "", 0);
		}*/
		myprintf("I was able to open 'config.txt' for writing\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_close(&fil);
		f_mount(NULL, "", 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		return false;
	}

	BYTE readBuf[128];
	memset(readBuf,0,128);
	//float *temp, float *hum, uint8_t *wcup, uint8_t *wrez, uint16_t *soil1, uint16_t *soil2
	char *write_buffer = (char*)malloc(128);
	*write_buffer = '\0';
	sprintf(write_buffer+strlen(write_buffer),"mode = %i \n",mode);
	sprintf(write_buffer+strlen(write_buffer),"humidity = %i \n",hum);
	sprintf(write_buffer+strlen(write_buffer),"temperature = %i \n\r\0",temp);

	//Copy in a string
	strncpy((char*)readBuf, write_buffer, strlen(write_buffer));
	UINT bytesWrote;
	myprintf(readBuf);
	fres = f_write(&fil, readBuf, strlen(write_buffer), &bytesWrote);
	if(fres == FR_OK) {
		myprintf("Wrote %i bytes to 'config.txt'!\r\n", bytesWrote);
	}else {
		myprintf("f_write error (%i)\r\n",fres);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_close(&fil);
		f_mount(NULL, "", 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		return false;
	}
	f_close(&fil);
	f_mount(NULL, "", 0);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);// orange led to indicate data logging
	return true;
}
uint8_t bcdToDec(uint8_t val){
	return RTC_Bcd2ToByte(val);
	return((val/10*10)+(val%16));
}
uint8_t decToBcd(uint8_t val){
	return RTC_ByteToBcd2(val);
	return ((val/10*10)+(val%10));
}
void getLine(){
	RXDone = false;
	//while?
	/*
	if(RXDone && huart1.RxState != HAL_UART_STATE_BUSY_RX){
		RXBuffer[0] = '\0';
		RXDone = false;
		HAL_UART_Receive_IT(&huart1,&RXByte,1);
	}else{
		myprintf("UART is RX busy!");
	}*/
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		// !!!!!!!!!!! POZOR .. nikdy nesmím v callbacku pro RX vysílat TX !!!!!!!!
		// jinak se podělají zprávy posílané z stm
		//HAL_UART_Transmit(&huart1, (uint8_t*)RXByte, 1, -1);//echo
		strcat(&RXBuffer,&RXByte);
		HAL_UART_Receive_IT(&huart1,&RXByte,1);
	}
}
void uart_buffering(){
	// ze všech nasbíraných dat se vezme pouze jeden řádek
	// předpokládám, že během průchodu jedné smyčky nepošlu dva příkazy
	if(strlen(RXLineBuffer)+strlen(RXBuffer) > 128){
		myprintf("Buffer overflow!\n\r");
		RXLineBuffer[0] = '\0';
	}
	strcat(&RXLineBuffer,&RXBuffer);
	RXBuffer[0] = '\0';

	int index = strcspn(RXLineBuffer,"\n\r");// počet znaků před
	if(index != strlen(RXLineBuffer)){
		//buffer obsahuje \n\r
		if(!RXDone){
			strcpy(&RXLine,&RXLineBuffer);
			RXDone = true;
			//myprintf(RXLine);
		}
		RXLineBuffer[0] = '\0';
	}
	/*
	if(strlen(RXBuffer) >= 32){
		RXBuffer[32] = '\n';
		RXBuffer[33] = '\0';
		myprintf("UART buffer přetekl!\r\n");
		if(!RXDone){
			strcpy(&RXLine,&RXBuffer);
		}
		RXDone = true;
		RXBuffer[0] = '\0';
		//HAL_UART_AbortReceive(&huart1);
	}*/
}

void myprintf(const char *fmt, ...) {
  static char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  int len = strlen(buffer);
  HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, -1);

}
sht3x_handle_t setupSHT(){
	// Create the handle for the sensor.
	sht3x_handle_t handle = {
	  .i2c_handle = &hi2c1,
	  .device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_HIGH
	};

	// Initialise sensor (tests connection by reading the status register).
	if (!sht3x_init(&handle)) {
	  myprintf("SHT3x access failed.\n\r");
	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
	}
	return handle;
}
void readData(sht3x_handle_t *handle, float *temp, float *hum, uint8_t *wcup, uint8_t *wrez, float *soil1, float *soil2){
	// teplota ve stupních, hum v %
	sht3x_read_temperature_and_humidity(handle, temp, hum);
	if(abs(*temp) < 100){
		*temp = k0_temp + k1_temp*(*temp);	// kalibřaní křivka
	}
	// digitální vstupy - miska, rezervoár
	// nejdříve se musí zapnout napáení květináče
	HAL_GPIO_WritePin(pwr_GPIO_Port,pwr_Pin,SET);
	HAL_Delay(50);// pro jistotu chvilku počkám, aby vše naběhlo
	if(!HAL_GPIO_ReadPin(cup_med_GPIO_Port,cup_med_Pin)){
		if(!HAL_GPIO_ReadPin(cup_high_GPIO_Port,cup_high_Pin)){
			*wcup = 2;
		}else{
			*wcup = 1;
		}
	}else{
		*wcup = 0;
	}
	if(!HAL_GPIO_ReadPin(rez_med_GPIO_Port,rez_med_Pin)){
		if(!HAL_GPIO_ReadPin(rez_high_GPIO_Port,rez_high_Pin)){
			*wrez = 2;
		}else{
			*wrez = 1;
		}
	}else{
		*wrez = 0;
	}

	// Soil adc humidities
	// Get ADC value
	int adc1;
	int adc2;
	if( HAL_ADC_Start(&hadc1) != HAL_OK){
		myprintf("HAL ADC1 Start fucked up!\n\r");
		return;
	}
	if( HAL_ADC_Start(&hadc2) != HAL_OK){
			myprintf("HAL ADC2 Start fucked up!\n\r");
			return;
	}
	if(HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK){
		myprintf("HAL ADC1 Poll fucked up!\n\r");
		return;
	}
	adc1 = HAL_ADC_GetValue(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK){
		myprintf("HAL ADC2 Poll fucked up!\n\r");
		return;
	}
	adc2 = HAL_ADC_GetValue(&hadc2);
	float u_meas_1 = (adc1/4096.0f)*3.3f;	// !!!!!!!!! musím označit, že násobím flouty!!!!! jinak tady program zmarzne
	float u_meas_2 = (adc2/4096.0f)*3.3f;
	float u_2 = 2.0f;		// h = 0%
	float u_1 = 0.6f;	// h = 100%
	float a = 100.0f/(u_1 - u_2);	// rovnice přímky
	float b = 100.0f-a*u_1;
	*soil1 = a*u_meas_1 + b;
	*soil2 = a*u_meas_2 + b;

	HAL_GPIO_WritePin(pwr_GPIO_Port,pwr_Pin,SET);
	/*
	// Enable heater for two seconds.
	sht3x_set_header_enable(&handle, true);
	HAL_Delay(2000);
	sht3x_set_header_enable(&handle, false);*/
}
char* removeSpaces(char *str){
    int i =0,j=0;
    while(str[i]){
        if(str[i] != ' '){
            str[j++] = str[i];
        }
        i++;
    }
    str[j] = '\0';
    return str;
}
bool openConfigFile(uint8_t *mode, uint8_t *hum, uint8_t *temp){
	//!!!!!!!!!!!!! POZOR .. v souboru nesmí být na začátku prázdné řádky!!!!!
	// .. teda .. je třeba odzkoušet .. možná jsem to akorát opravil
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);// orange led to indicate SD activity
    FATFS FatFs; 	//Fatfs handle
    FIL fil; 		//File handle
    FRESULT fres; //Result after operations

    //Open the file system
    fres = f_mount(&FatFs, "", 1); //1=mount now
    if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_mount(NULL, "", 0);
		return false;
    }

    DWORD free_clusters, free_sectors, total_sectors;
    FATFS* getFreeFs;
    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
		myprintf("f_getfree error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
  	  	return false;
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;
    myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    fres = f_open(&fil, "config.txt", FA_READ);
    if (fres != FR_OK) {
		myprintf("f_open error (%i)\r\n");
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_close(&fil);
		f_mount(NULL, "", 0);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
  	  	return false;
    }
    myprintf("I was able to open 'config.txt' for reading!\r\n");

    //Read 30 bytes from "test.txt" on the SD card
    BYTE readBuf[30];

    for(int i = 0; i<3; i++){
    	TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);	// přečte celý řádek, pokud nezaplní buffer
		if(rres != 0) {
			myprintf("Read string from 'config.txt' contents: %s\n\r", readBuf);
			char *data = &readBuf[0];
			char d[strlen(data)];
			strcpy(d,data);
			data = removeSpaces(d);
			strcpy(d,data);
			int delic = strcspn(d,"=");// počet znaků před =

			if (delic != strlen(d)){	// příkaz rozeznán
				char *key = (char*)malloc(delic+1);	// na konci přidám jene znak pro ukončení strungu
				memcpy(key,&d,delic);
				key[delic] = '\0';
				//strncpy(key,d,delic);
				char *value = (char*)malloc(sizeof(d)-delic+1);
				memcpy(value,&d[delic+1],sizeof(d)-delic);
				value[sizeof(d)-delic] = '\0';
				//strcpy(value,d[delic+1]);
				if(strcmp(key,"mode") == 0){
					*mode = atoi(value);
				}else if(strcmp(key, "humidity") == 0){
					*hum = atoi(value);
				}else if(strcmp(key, "temperature") == 0){
					*temp = atoi(value);
				}else{
					myprintf("\n\rKey \"%s\" not recognized!\n\r",key);
				}
			}else{
				myprintf("\n\rDidnt found \"=\" on line %i",i);
				/*
				f_close(&fil);
				f_mount(NULL, "", 0);
				HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
				return false;
				*/
			}
		} else {
			myprintf("f_gets error (%i)\r\n", fres);
			f_close(&fil);
			f_mount(NULL, "", 0);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
			return false;
		}
    }
    f_close(&fil);
    f_mount(NULL, "", 0);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);// orange led to indicate SD activity
    return true;
}
bool writeToFile(float temp, float hum, uint8_t wcup, uint8_t wrez, float soil1, float soil2){
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_SET);// orange led to indicate SD activity
	FATFS FatFs; 	//Fatfs handle
	FIL fil; 		//File handle
	FRESULT fres; //Result after operations

	//Open the file system
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		myprintf("f_mount error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		f_mount(NULL, "", 0);
		return false;
	}

	DWORD free_clusters, free_sectors, total_sectors;
	FATFS* getFreeFs;
	fres = f_getfree("", &free_clusters, &getFreeFs);
	if (fres != FR_OK) {
		myprintf("f_getfree error (%i)\r\n", fres);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
		return false;
	}

	//Formula comes from ChaN's documentation
	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	free_sectors = free_clusters * getFreeFs->csize;
	myprintf("SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    fres = f_open(&fil, "data.txt", FA_WRITE | FA_OPEN_ALWAYS);
    if(fres == FR_OK) {
    	fres = f_lseek(&fil,f_size(&fil));	// ukáže na konec souboru
    	if(fres != FR_OK){
    		myprintf("f_lseek error (%i)\r\n", fres);
    		f_close(&fil);
			f_mount(NULL, "", 0);
    	}
    	myprintf("I was able to open 'data.txt' for writing\r\n");
    } else {
    	myprintf("f_open error (%i)\r\n", fres);
    	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
    	f_close(&fil);
		f_mount(NULL, "", 0);
		return false;
    }

    BYTE readBuf[128];
    memset(readBuf,0,128);
	HAL_RTC_GetTime(&hrtc,&sTime,RTC_FORMAT_BCD);
    HAL_RTC_GetDate(&hrtc,&sDate,RTC_FORMAT_BCD);
    //float *temp, float *hum, uint8_t *wcup, uint8_t *wrez, uint16_t *soil1, uint16_t *soil2
	char *write_buffer = (char*)malloc(128);
	*write_buffer = '\0';
	sprintf(write_buffer+strlen(write_buffer),"\n");
	sprintf(write_buffer+strlen(write_buffer)," \t%02d.%02d.%02d\t ;",bcdToDec(sDate.Date),bcdToDec(sDate.Month),bcdToDec(sDate.Year));
	sprintf(write_buffer+strlen(write_buffer)," \t%02d:%02d:%02d\t ;",bcdToDec(sTime.Hours),bcdToDec(sTime.Minutes),bcdToDec(sTime.Seconds));
	sprintf(write_buffer+strlen(write_buffer)," \t%.2f\t ;",temp);
	sprintf(write_buffer+strlen(write_buffer)," \t%.2f\t ;",hum);
	sprintf(write_buffer+strlen(write_buffer)," \t%i\t ;",wcup);
	sprintf(write_buffer+strlen(write_buffer)," \t%i\t ;",wrez);
	sprintf(write_buffer+strlen(write_buffer)," \t%.2f\t ;",soil1);
	sprintf(write_buffer+strlen(write_buffer)," \t%.2f\t ;\0",soil2);

    //Copy in a string
    strncpy((char*)readBuf, write_buffer, strlen(write_buffer));
    UINT bytesWrote;
    myprintf(readBuf);
    fres = f_write(&fil, readBuf, strlen(write_buffer), &bytesWrote);
    if(fres == FR_OK) {
    	myprintf("Wrote %i bytes to 'data.txt'!\r\n", bytesWrote);
    }else {
    	myprintf("f_write error (%i)\r\n",fres);
    	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);
    	HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,GPIO_PIN_SET);// red led
    	f_close(&fil);
		f_mount(NULL, "", 0);
		return false;
    }
    f_close(&fil);
	f_mount(NULL, "", 0);
	HAL_GPIO_WritePin(GPIOE,GPIO_PIN_10,GPIO_PIN_RESET);// orange led to indicate SD activity
	return true;
}
void sendMyData(float temp, float hum, uint8_t wcup, uint8_t wrez, float soil1, float soil2){
	char data[128];
	sprintf(data,"{%.2f;%.2f;%i;%i;%.2f;%.2f}\n\r",temp,hum,wcup,wrez,soil1,soil2);
	HAL_UART_Transmit(&huart1,(uint8_t*)data,strlen(data),-1);
}
void sendConfig(uint8_t modeset,uint8_t humset,uint8_t tempset){
	char data[128];
	sprintf(data,"{%i;%i;%i}\n\r",modeset,tempset,humset);
	HAL_UART_Transmit(&huart1,(uint8_t*)data,strlen(data),-1);
}
void zalij(int time_ms){
	myprintf("{watter_on}\n\r");
	HAL_GPIO_WritePin(motor_GPIO_Port,motor_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(LD7_GPIO_Port,LD7_Pin,GPIO_PIN_SET);
	HAL_Delay(time_ms);
	HAL_GPIO_WritePin(LD7_GPIO_Port,LD7_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor_GPIO_Port,motor_Pin,GPIO_PIN_RESET);
	myprintf("{watter_off}\n\r");
}
void sendStatus(bool zalevam, bool ohrivam){
	char data[128];
	uint8_t watter,heat;
	if(zalevam){
		watter = 1;
	}else{
		watter = 0;
	}
	if(ohrivam){
		heat = 1;
	}else{
		heat = 0;
	}
	sprintf(data,"{%i;%i}\n\r",watter,heat);
	HAL_UART_Transmit(&huart1,(uint8_t*)data,strlen(data),-1);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_RTC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  /*
	// -- Enables ADC DMA request
	if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcDMABuffer, 2) != HAL_OK){
		myprintf("HAL ADC Start DMA fucked up!\n\r");
		return;
	}
	while(adcConversionComplete == 0){}
	adcConversionComplete = 0;
	*/
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay(1000);
  sht3x_handle_t handle = setupSHT();

  myprintf("Flowerpot started\r\n");
  float temperature, humidity;
  uint8_t watter_cup, watter_rez;
  float soil1, soil2;
  uint8_t modeset;
	uint8_t humset;
	uint8_t tempset;
  if(!openConfigFile(&modeset, &humset, &tempset)){
  	myprintf("Something fucked up during opening of config file!\n\r");
  }else{
  	myprintf("Modeset: %i, humset: %i, tempset: %i\n\r",modeset,humset,tempset);
  }
  if(!openCalibrationFile(&k0_temp, &k1_temp)){
    	myprintf("Something fucked up during opening of calibration file!\n\r");
  }

  // start listening on serial port
  HAL_UART_Receive_IT(&huart1,&RXByte,1);
  //getLine();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    int index;
    char tmp[3],tmp_char[32];
    tmp[2] = '\0';// jo jo .. mohl bych použít jen tmp_char .. ale už se mi to nechce předělávat :D
    // nula ukončí string .. používá se u nastavení času, bez toho je čas nesmysl
    bool timeset = false;
    uint8_t log_perioda = 1; // hodina
    int time_hours = -1;
    int time = -1;
    int stop_day = -1;
    int stop_time = -1;
    bool zalevam = false;
    bool ohrivam = false;
    int watter_period = 10;	//minut mezi zaléváním

    myprintf("Waitig for time sequence: \n\r");
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	uart_buffering();
// smyčka pro úvodnmí nastaevgní času
    	while(false){// zkouška jestli funguje uart
    		if(RXDone){
    			myprintf(RXLine);
    			RXDone = false;
    		}
    	}

		while(!timeset){// wait to receive current date and time
			uart_buffering();
			if(RXDone){
				myprintf(RXLine);
				myprintf("\n\r");
				//030622
				//205420
				if(strlen(RXLine) != 13){// včetně ukončovacího znaku
					myprintf("Wrong structure!\n\r");
					RXDone = false;
					//getLine();
				}else{
					myprintf("Time received!\n\r");
					myprintf("Copying time to memory..");
					memcpy(tmp,&RXLine[0],2);
					sDate.Date = decToBcd(atoi(tmp));
					memcpy(tmp,&RXLine[2],2);
					sDate.Month = decToBcd(atoi(tmp));
					memcpy(tmp,&RXLine[4],2);
					sDate.Year = decToBcd(atoi(tmp));
					memcpy(tmp,&RXLine[6],2);
					sTime.Hours = decToBcd(atoi(tmp));
					memcpy(tmp,&RXLine[8],2);
					sTime.Minutes = decToBcd(atoi(tmp));
					memcpy(tmp,&RXLine[10],2);
					sTime.Seconds = decToBcd(atoi(tmp));
					HAL_RTC_SetTime(&hrtc,&sTime,RTC_FORMAT_BCD);
					HAL_RTC_SetDate(&hrtc,&sDate,RTC_FORMAT_BCD);
					myprintf("Done!\n\r");
					timeset = true;
					break;
				}
			}
		}// smyčka pro nastavení času

		// zapni sledování uartu
		if(RXDone){
			// hledám packet s nastavením: {mode; temp; hum}
			uint8_t left = strcspn(RXLine,"{");
			uint8_t right = strcspn(RXLine,"}");
			uint8_t tmp_idx;
			uint8_t tmp_idx2;
			if(left < right && right != strlen(RXLine)){
				myprintf("Packet captured!\n\r");
				tmp_idx = strcspn(RXLine,";");
				tmp_idx2 = strcspn(&RXLine[tmp_idx+1],";") + tmp_idx + 1;//druhý středník => mám vše
				if(tmp_idx == strlen(RXLine) || tmp_idx2 >= strlen(RXLine)){
					if(tmp_idx != strlen(RXLine)){// mám jen jeden středník, pak předpokládám, že přišla kalibrační data
						memcpy(tmp_char,&RXLine[left+1],tmp_idx-left-1);
						tmp_char[tmp_idx-left-1] = '\0';
						k0_temp = (float)atof(tmp_char);
						memcpy(tmp_char,&RXLine[tmp_idx+1],right-tmp_idx-1);// počítám s tím, že { nemusí být na nulové pozici
						tmp_char[right-tmp_idx-1] = '\0';
						k1_temp = (float)atof(tmp_char);
						myprintf("Calibration set!\n\r");
						myprintf("k0: %f\t k1: %e\n\r",k0_temp,k1_temp);
						myprintf("Saving..\n\r");
						if(saveCalibration(k0_temp,k1_temp)){
							myprintf("Calibration saved!\n\r");
						}
					}else{
						myprintf("Not enough items inside the packet!");
					}
				}else{
					memcpy(tmp_char,&RXLine[left+1],tmp_idx-left-1);
					tmp_char[tmp_idx-left-1] = '\0';
					modeset = atoi(tmp_char);
					memcpy(tmp_char,&RXLine[tmp_idx+1],tmp_idx2-tmp_idx-1);
					tmp_char[tmp_idx2-tmp_idx-1] = '\0';
					tempset = atoi(tmp_char);
					memcpy(tmp_char,&RXLine[tmp_idx2+1],right-tmp_idx2-1);
					tmp_char[right-tmp_idx2-1] = '\0';
					humset = atoi(tmp_char);
					myprintf("Variables set!\n\r");
					myprintf("Modeset: %i\t Tempset: %i\t Humset: %i\n\r",modeset,tempset,humset);
					myprintf("Saving..\n\r");
					if(saveConfig(modeset, tempset, humset)){
						myprintf("Variables saved!\n\r");
					}
				}
			}else if(strstr(RXLine,"getdata") != NULL){
				myprintf("\n\r");
				readData(&handle, &temperature, &humidity, &watter_cup, &watter_rez, &soil1, &soil2);
				//writeToFile(temperature, humidity, watter_cup, watter_rez, soil1, soil2);
				sendMyData(temperature, humidity, watter_cup, watter_rez, soil1, soil2);
			}else if(strstr(RXLine,"getconfig") != NULL){
				myprintf("\n\r");
				sendConfig(modeset,humset,tempset);
			}else if(strstr(RXLine,"getstatus") != NULL){
				myprintf("\n\r");
				sendStatus(zalevam,ohrivam);
			}else if(strstr(RXLine,"gettime") != NULL){
				myprintf("\n\r");
				myprintf("Date: %02d.%02d.%02d\t",bcdToDec(sDate.Date),bcdToDec(sDate.Month),bcdToDec(sDate.Year));
				myprintf("Time: %02d:%02d:%02d\r\n",bcdToDec(sTime.Hours),bcdToDec(sTime.Minutes),bcdToDec(sTime.Seconds));
			}else if(strstr(RXLine,"settime") != NULL){
				myprintf("\n\r");
				timeset = false;
			}else if(strstr(RXLine,"getconnection") != NULL){
				myprintf("\n\r");
				myprintf("connected\n\r");
			}else if(strstr(RXLine,"getcalibration") != NULL){
				myprintf("\n\r");
				myprintf("{%e;%e}\n\r",k0_temp,k1_temp);
			}else if(strstr(RXLine,"getrawtemp") != NULL){
				myprintf("\n\r");
				float temp = getRawTemp(&handle);
				myprintf("{%f}\n\r",temp);
			}
			RXDone = false;
			//getLine();
		}//sledování uartu

		//Řízení
		readData(&handle, &temperature, &humidity, &watter_cup, &watter_rez, &soil1, &soil2);
		// čas v minutách
		time = bcdToDec(sTime.Hours)*60 + bcdToDec(sTime.Minutes) + bcdToDec(sTime.Seconds)/60;
		if(watter_rez != 0){// není prázdná nádrž
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);// red led
			switch(modeset){//režimy zalévání
			case 0:
				//plná miska
				//udržuje misku mezi max a med hodnotou

				if(!zalevam && watter_cup < 1){
					// voda klesla pod med => zalij
					zalevam = true;
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);// blue led
					//zalij(1000);
				}else if(zalevam && watter_cup < 2){
					//voda není na max => dále zalévám
					//zalij(1000);
				}else if(zalevam){
					// vypni zalévání
					zalevam = false;
					HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_RESET);// blue led
				}
				break;
			}//switch
			// proces zalévání
			// zalévá se pouze krátkce v časových intervalech
			if(stop_day == -1 && zalevam){// vynulováno pak zapni stopky
				stop_day = bcdToDec(sDate.Date);
				stop_time = time;
				// před zalitím uložím data, abych mohl přímo pozorovat změny zapřčíčiněné zaléváním
				writeToFile(temperature, humidity, watter_cup, watter_rez, soil1, soil2);
				if(watter_rez < 2){
					zalij(2000);
				}else{
					zalij(1000);
				}

			}else if(zalevam){
				if(bcdToDec(sDate.Date) != stop_day){// při změně dne je třeba přičíst (odečíst) den
					stop_time = stop_time - 24*60;
				}
				if(time - stop_time > watter_period){// každých pět minut na 1.5 sekundu zalij
					stop_day = -1;// vynulování stopek
				}
			}else{// nezalévám .. pak vynuluj stopky
				// pokud nastane zákmit .. nesmím hned začít pumpovat vodu ..
				// voda se může pumpovat jen v 5 min intervalech
				if(time - stop_time > watter_period){// každých pět minut na 1.5 sekundu zalij
					stop_day = -1;// vynulování stopek
					stop_time = -1;
				}
			}
		}else{
			zalevam = false;
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);// red led
		}
		// ohřívání
		if(tempset > 0){
			if(temperature < tempset - 1 && !ohrivam){
				HAL_GPIO_WritePin(lamp_GPIO_Port,lamp_Pin,GPIO_PIN_SET);
				ohrivam = true;
				myprintf("{heating_on}\n\r");
			}else if(ohrivam && temperature > tempset + 0.5f){	// překročení požadované teploty .. vypni ohřívání
				HAL_GPIO_WritePin(lamp_GPIO_Port,lamp_Pin,GPIO_PIN_RESET);
				ohrivam = false;
				myprintf("{heating_off}\n\r");
			}
		}else{
			ohrivam = false;
		}


		//Logování dat
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);//!!!!! POZOR .. čas se zasekne, pokud hned potom nevolám getDate!!!!!!!!!§
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
		if(time_hours != bcdToDec(sTime.Hours)){	// čas se změnil, zkontroluj zda nemám logovat data
			time_hours = bcdToDec(sTime.Hours);
			if(time_hours%log_perioda == 0){
				readData(&handle, &temperature, &humidity, &watter_cup, &watter_rez, &soil1, &soil2);
				writeToFile(temperature, humidity, watter_cup, watter_rez, soil1, soil2);
			}
		}
		//myprintf("test\n\r");
    	//myprintf("Date: %02d.%02d.%02d\t",bcdToDec(sDate.Date),bcdToDec(sDate.Month),bcdToDec(sDate.Year));
		//myprintf("Time: %02d:%02d:%02d\r\n\r",bcdToDec(sTime.Hours),bcdToDec(sTime.Minutes),bcdToDec(sTime.Seconds));
		HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
    	//HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);//!!!!! POZOR .. čas se zasekne, pokud hned potom nevolám getDate!!!!!!!!!§
    	//HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
		HAL_Delay(100);
    }//while smyčka
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  //HAL_NVIC_SetPriority(USART1_IRQn,0,0);
  //HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, pwr_Pin|lamp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(motor_GPIO_Port, motor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DRDY_Pin MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin
                           MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = DRDY_Pin|MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin
                          |MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin LD5_Pin
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|LD5_Pin
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : cup_med_Pin cup_high_Pin rez_med_Pin rez_high_Pin */
  GPIO_InitStruct.Pin = cup_med_Pin|cup_high_Pin|rez_med_Pin|rez_high_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : pwr_Pin lamp_Pin */
  GPIO_InitStruct.Pin = pwr_Pin|lamp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : motor_Pin */
  GPIO_InitStruct.Pin = motor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(motor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

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

