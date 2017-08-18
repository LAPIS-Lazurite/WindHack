/* FILE NAME: WindHack_BLE.c
 * The MIT License (MIT)
 * 
 * Copyright (c) 2017  Lapis Semiconductor Co.,Ltd.
 * All rights reserved.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//
// debug options
//
//#define LIB_DEBUG
//#define BREAK_MODE
//#define DEBUG

//
// include files
//
#include "WindHack_BLE_ide.h"		// Additional Header
#include <stdlib.h>
#include <string.h>
#include "driver_extirq.h"
#include "driver_gpio.h"

//
// version
//
#define VERSION				("WindHack F/W Version 0.99B\r\n")

//
// macro switch
//
//#define GPS_NOT_USE			// define if GPS module is not attached
//#define BIN_FORMAT			// define if sensor data is treated as binary format

//
// macro definition
//
// pin definition
#define PWR_SW				( 2 )						// Power SW
#define RF_SW				( 3 )						// RF SW
#define BLUE_LED			( 4 )						// BLE transfer LED
#define ORANGE_LED			( 5 )						// SD Card Access LED
#define RED_LED		 		( 6 )						// Power LED
#define LDO3V_EN			( 9 )						// LDO3V_EN pin
#define SDSPI_SS_PIN		( 10 )						// SPI SS pin
// led
#define ALL_LED_OFF			( 0 )
#define BLUE_LED_OFF		( 1 )
#define ORANGE_LED_OFF		( 2 )
#define RED_LED_OFF			( 3 )
#define ALL_LED_ON			( 4 )
#define BLUE_LED_ON			( 5 )
#define ORANGE_LED_ON		( 6 )
#define RED_LED_ON			( 7 )
// led pattern
#define LED_LOW_BATTERY		( 0 )
#define LED_POWER_ON		( 1 )
#define LED_POWER_OFF		( 2 )
#define LED_GPRMC_ACTIVE	( 3 )
#define LED_SDCARD_ERROR	( 4 )
#define LED_MAX_PATTERN		( 8 )
// ldo control
#define LDO3V_OFF			( 0 )
#define LDO3V_ON			( 1 )
// sensor
#define SENSOR_BUF_SIZE  	( 100 )
#define KXG03_BUF_SIZE   	( 60 )
#define BM1422_BUF_SIZE  	( 30 )
#define SAMPLING_RATE_RF_ON		KXG03_ODR_6_25HZ		// 160ms sampling rate
#define SAMPLING_RATE_RF_OFF	KXG03_ODR_50HZ			// 20ms sampling rate
// low battery threshold
#define LOWBAT_TIMER_THRESLD	 ( VLS_2_800 )
#define LOWBAT_NOT_TIMER_THRESLD ( VLS_3_068 )
// gps parameter
#define GPS_SEN_SIZE		( 6 )
#define NMEA_GPRMC			( "$GPRMC" )
#define NMEA_GPZDA			( "$GPZDA" )
#define NMEA_VALID_GPRMC	( "A" )
#define NMEA_INVALID_YEAR	( "1980" )
#define GPS_BUF_SIZE		( 160 )
#define GPZDA_BUF_SIZE		( 50 )
// sd
#define LOW_BIT				( 0 )
#define HIGH_BIT			( 1 )
// file status
#define FILE_CLOSED			( 0 )
#define FILE_OPEN_REQ		( 1 )
#define FILE_OPENED			( 2 )
#define FILE_NOT_USE		( 3 )
// ble
#define CONNECTION_INT_FAST	( 10 )						// testing purpose only

//
// typedef
//
typedef __packed struct {
	uint8_t sig[2];
	uint16_t addr;
	uint16_t seq_num;
	uint16_t deltaT;
	signed short acc_data[3];		// the data multiplied by 1024
	signed long gyro_data[3];		// the data multiplied by 1024
	signed short mag_data[3];		// the data multiplied by 1024
	uint32_t baro_data;				// the data multiplied by 1024
} BIN_DATA;

typedef struct {
	uint8_t cycle;
	uint8_t sequence[LED_MAX_PATTERN];
	uint16_t sequence_time[LED_MAX_PATTERN];
} LED_PARAM;

typedef struct {
	bool	state;
	uint8_t cycle;
	uint8_t *sequence;
	uint16_t *sequence_time;
	uint32_t event_time;
} LED_CTRL;

const LED_PARAM led_param[5] =
{
	{8,{RED_LED_ON,RED_LED_OFF,RED_LED_ON,RED_LED_OFF,RED_LED_ON,RED_LED_OFF,RED_LED_ON,RED_LED_OFF},{500,500,500,500,500,500,500,50}},	// low battery
	{4,{RED_LED_ON,RED_LED_OFF,RED_LED_ON,RED_LED_OFF},{150,150,150,10}},		// power on
	{2,{RED_LED_ON,RED_LED_OFF},{1000,10}},										// power off
	{2,{RED_LED_ON,RED_LED_OFF},{150,10}},										// GPRMC active
	{2,{ORANGE_LED_ON,ORANGE_LED_OFF},{4000,10}}								// sd card error
};

typedef enum {
	STATE_BOOT_INIT = 0,
	STATE_POWER_OFF,
	STATE_POWER_ON
} WIND_HACK_STATE;

typedef struct {
	WIND_HACK_STATE func_mode;
	uint8_t	file;
	LED_CTRL led;
	uint32_t current_time;
} WIND_HACK_PARAM;

WIND_HACK_PARAM wind_hack_param;

WIND_HACK_STATE func_boot_init(void);
WIND_HACK_STATE func_power_off(void);
WIND_HACK_STATE func_power_on(void);

WIND_HACK_STATE (*functions[])(void) = {
	func_boot_init,
	func_power_off,
	func_power_on
};

//
// global variable
//
// interrupt flag
bool		kxg03_irq = false; // true = kxg03 interrupt occured, false = interrupt cleared
bool		bat_check_irq = false; // true = battery check interrupt occured, false = interrupt cleared
bool		pwr_sw_irq = false; // true = power switch interrupt occured, false = interrupt cleared
// gps
bool		rtc_gps_sync_req;
// sd
#ifdef BIN_FORMAT
uint8_t		filename[] = "YYMMDDNN.BIN";
uint8_t		filename2[] = "YYMMDDNN.BIN";
#else
uint8_t		filename[] = "YYMMDDNN.CSV";
uint8_t		filename2[] = "YYMMDDNN.CSV";
#endif
st_File_v	myFile;

//
// sd
//
static uint8_t hex_to_char(uint8_t hex, uint8_t bit)
{
	return bit ? (hex / 10) + '0' : (hex % 10) + '0';	// hex = 0~99
	// No problem : Warning : W5023 : Conversion between different integral types
}

static void make_filename(void)
{
	uint8_t num = 0, n;

	n = RTC.getYear();									// YY
	filename[0] = hex_to_char(n, HIGH_BIT);
	filename[1] = hex_to_char(n, LOW_BIT);
	n = RTC.getMonth();									// MM
	filename[2] = hex_to_char(n, HIGH_BIT);
	filename[3] = hex_to_char(n, LOW_BIT);
	n = RTC.getDay();									// DD
	filename[4] = hex_to_char(n, HIGH_BIT);
	filename[5] = hex_to_char(n, LOW_BIT);
	do {												// NN
		filename[6] = hex_to_char(num, HIGH_BIT);
		filename[7] = hex_to_char(num, LOW_BIT);
		if (num >= 100) {
			filename[0] = NULL;
			break;
		} else {
			num++;
		}
	} while (SD.exists(filename));
#ifdef DEBUG
	if (filename) {
		Serial.print("make_filename() : ");
		Serial.println(filename);
	} else {
		Serial.println("make_filename() : filename error. NULL pointer.");
	}
#endif
	strcpy(filename2, filename);
}

//
// sensor
//
static void kxg03_isr(void)
{
	kxg03_irq = true;
}

static void kxg03_sync_init2(uint8_t slave_addr,uint8_t odr_rate,void (*func)(void))
{
	uint8_t rc;
	uint8_t data;

	rc = kxg03.init(slave_addr);

	// set KXG03 to stand-by
	data = 0xEF;
	rc = kxg03.write(KXG03_STDBY,&data,1);
	
	// clear int1
	rc = kxg03.read(KXG03_INT1_L,&data,1);

	// opem intmask of ACC and GYRO
	data = 0x03;
	rc = kxg03.write(KXG03_INT_MASK1,&data,1);

	// set interrupt mode
	data = 0x0E;
	rc = kxg03.write(KXG03_INT_PIN_CTL,&data,1);
	
	// Enable interrupt from ACC, GYRO
	data = 0x03;
	rc = kxg03.write(KXG03_INT_PIN1_SEL,&data,1);

	// set ACC ODR
//	data = odr_rate | (kxg03_acc_avr<<4);
	data = odr_rate | (KXG03_ACC_AVR_128<<4);
	rc = kxg03.write(KXG03_ACCEL_ODR_WAKE ,&data,1);
	
	// set GYRO ODR to 25Hz
//	data = odr_rate | (kxg03_gyro_range<<6) | (kxg03_gyro_bw<<4);
	data = odr_rate | (0<<6) | (0<<4);
	rc = kxg03.write(KXG03_GYRO_ODR_WAKE,&data,1);
	
		// set ACC_RANGE
//	data = (kxg03_acc_range<<2);
	data = (0<<2);
	rc = kxg03.write(KXG03_ACCEL_CTL,&data,1);
	
	// set Interrupt
	drv_pinMode(10, INPUT_PULLDOWN);
	drv_attachInterrupt(10, 6, func, RISING, false, false);

	// Start sensor
	data = 0xec;
	rc = kxg03.write(KXG03_STDBY,&data,1);
}

#ifdef BIN_FORMAT
static void kxg03_conv_reg_hex(uint8_t *reg, signed short *acc, signed long *gyro)
{
	acc[0] = (((signed short)reg[7] << 8) | reg[6]) >> 4;
	acc[1] = (((signed short)reg[9] << 8) | reg[8]) >> 4;
	acc[2] = (((signed short)reg[11] << 8) | reg[10]) >> 4;
	gyro[0] = (signed long)(((signed short)reg[1] << 8) | reg[0]) << 3;
	gyro[1] = (signed long)(((signed short)reg[3] << 8) | reg[2]) << 3; 
	gyro[2] = (signed long)(((signed short)reg[5] << 8) | reg[4]) << 3;
}

static void kxg03_get_val_hex(signed short *acc, signed long *gyro)
{
	uint8_t rc;
	uint8_t reg[12];

	rc = kxg03.read(KXG03_GYRO_XOUT_L, reg, 12);
	if(rc!=12) {
#ifdef DEBUG
		Serial.print("I2C ERROR=");
		Serial.println_long((long)rc,DEC);
#endif
	} else {
		kxg03_conv_reg_hex(reg, acc, gyro);
	}
}

static void bm1422_conv_reg_hex(uint8_t *reg, signed short *mag)
{
	mag[0] = ((signed short)reg[1] << 8) | (reg[0]);
	mag[1] = ((signed short)reg[3] << 8) | (reg[2]);
	mag[2] = ((signed short)reg[5] << 8) | (reg[4]);
	mag[0] = (signed short)((float) mag[0] * 0.042);
	mag[1] = (signed short)((float) mag[1] * 0.042);
	mag[2] = (signed short)((float) mag[2] * 0.042);
}

static void bm1422_get_val_hex(signed short *mag)
{
	byte rc;
	unsigned char reg[6];

	rc = bm1422.get_rawval(reg);
	if (rc != 0) {
#ifdef DEBUG
		Serial.print("I2C ERROR=");
		Serial.println_long((long)rc,DEC);
#endif
	} else {
		bm1422_conv_reg_hex(reg, mag);
	}
}

static void bm1383_conv_reg_hex(uint8_t* reg, uint32_t *baro)
{
	*baro = (((unsigned long)reg[0] << 16) | ((unsigned long)reg[1] << 8) | reg[2]&0xFC) >> 2;
	*baro /= 2;
}

static void bm1383_get_val_hex(uint32_t *baro)
{
	unsigned char reg[5];

	bm1383.get_rawtemppressval(reg);
	bm1383_conv_reg_hex(reg, baro);
}
#else // BIN_FORMAT
const int fixed_table[] = {
0,4,8,12,16,20,23,27,31,35,39,43,47,51,55,59,63,66,70,74,78,82,86,90,94,98,102,105,
109,113,117,121,125,129,133,137,141,145,148,152,156,160,164,168,172,176,180,184,188,
191,195,199,203,207,211,215,219,223,227,230,234,238,242,246,250,254,258,262,266,270,
273,277,281,285,289,293,297,301,305,309,313,316,320,324,328,332,336,340,344,348,352,
355,359,363,367,371,375,379,383,387,391,395,398,402,406,410,414,418,422,426,430,434,
438,441,445,449,453,457,461,465,469,473,477,480,484,488,492,496,500,504,508,512,516,
520,523,527,531,535,539,543,547,551,555,559,563,566,570,574,578,582,586,590,594,598,
602,605,609,613,617,621,625,629,633,637,641,645,648,652,656,660,664,668,672,676,680,
684,688,691,695,699,703,707,711,715,719,723,727,730,734,738,742,746,750,754,758,762,
766,770,773,777,781,785,789,793,797,801,805,809,813,816,820,824,828,832,836,840,844,
848,852,855,859,863,867,871,875,879,883,887,891,895,898,902,906,910,914,918,922,926,
930,934,938,941,945,949,953,957,961,965,969,973,977,980,984,988,992,996
};

// Note: "," is attached at the end of string.
static void print_fixedpoint(char *buf, short num, unsigned char point, unsigned char digit)
{
	volatile long integer;

	Print.init(buf,10);

	if((num < 0) && (num != -1 << 15)) {
		Print.p("-");
		num *= -1;
	}
	
	integer = (num >> point);
	Print.l(integer,DEC);
	if(digit == 0) return;
	
	Print.p(".");

	integer = integer << point;
	integer = num - integer;
	
	if (point > 8) {
		integer = integer >> (point - 8);
	} else {
		integer = integer << (8 - point);
	}

	integer = fixed_table[integer];

	switch(digit)
	{
	case 1:
		integer /= 100;
		Print.l(integer,DEC);
		break;
	case 2:
		integer /= 10;
		if (integer < 10) {
			Print.p("0");
		}
		Print.l(integer,DEC);
		break;
	case 3:
		if (integer < 10) {
			Print.p("00");
		} else if (integer < 100) {
			Print.p("0");
		}
		Print.l(integer,DEC);
		break;
	default:
		break;
	}
	Print.p(",");
}

static void kxg03_conv_reg_str(uint8_t* reg, uint8_t* str)
{
	uint16_t data[6];
	uint8_t buf[10];

	data[0] = ((signed short)reg[1] << 8) | reg[0];
	data[1] = ((signed short)reg[3] << 8) | reg[2];
	data[2] = ((signed short)reg[5] << 8) | reg[4];
	data[3] = ((signed short)reg[7] << 8) | reg[6];
	data[4] = ((signed short)reg[9] << 8) | reg[8];
	data[5] = ((signed short)reg[11] << 8) | reg[10];

	print_fixedpoint(buf, data[3], 14, 2);
	strcpy(str, buf);
	print_fixedpoint(buf, data[4], 14, 2);
	strcat(str, buf);
	print_fixedpoint(buf, data[5], 14, 2);
	strcat(str, buf);
	print_fixedpoint(buf, data[0], 7, 2);
	strcat(str, buf);
	print_fixedpoint(buf, data[1], 7, 2);
	strcat(str, buf);
	print_fixedpoint(buf, data[2], 7, 2);
	strcat(str, buf);
}

static uint8_t kxg03_get_val_str(uint8_t* str)
{
	uint8_t rc;
	uint8_t reg[12];

	rc = kxg03.read(KXG03_GYRO_XOUT_L, reg, 12);
	if(rc!=12) {
#ifdef DEBUG
		Serial.print("I2C ERROR=");
		Serial.println_long((long)rc,DEC);
#endif
	}

	kxg03_conv_reg_str(reg, str);

	return rc;
}

static void bm1422_conv_reg_str(uint8_t* reg, uint8_t* str)
{
	signed short mag[3];
	uint8_t buf[10];
	signed long data[3];

	mag[0] = ((signed short)reg[1] << 8) | (reg[0]);
	mag[1] = ((signed short)reg[3] << 8) | (reg[2]);
	mag[2] = ((signed short)reg[5] << 8) | (reg[4]);

	// 56371 = 27 bit shift left of 0.042/100
	// 16 bit right shift
	// 11 bit right shift for converting 0 point
	data[0] = (signed long)((mag[0] * 56371) >> 16);
	print_fixedpoint(buf, (signed short)data[0], 11, 3);
	strcpy(str, buf);
	data[1] = (signed long)((mag[1] * 56371) >> 16);
	print_fixedpoint(buf, (signed short)data[1], 11, 3);
	strcat(str, buf);
	data[2] = (signed long)((mag[2] * 56371) >> 16);
	print_fixedpoint(buf, (signed short)data[2], 11, 3);
	strcat(str, buf);
}

static byte bm1422_get_val_str(uint8_t* str)
{
	byte rc;
	unsigned char reg[6];

	rc = bm1422.get_rawval(reg);
	if (rc != 0) {
		return (rc);
	}

	bm1422_conv_reg_str(reg, str);

	return (rc);  
}

static void bm1383_conv_reg_data(uint8_t* reg, float *data)
{
	unsigned long rawbaro;

	rawbaro = (((unsigned long)reg[0] << 16) | ((unsigned long)reg[1] << 8) | reg[2]&0xFC) >> 2;
	*data = (float)rawbaro / 2048;
}
#endif // BIN_FORMAT

//
// LED
//
static void led_update(uint8_t value)
{
	switch(value)
	{
	case ALL_LED_OFF:
		digitalWrite(BLUE_LED, HIGH),digitalWrite(ORANGE_LED, HIGH),digitalWrite(RED_LED, HIGH);
		break;
	case BLUE_LED_OFF:
		digitalWrite(BLUE_LED, HIGH);
		break;
	case ORANGE_LED_OFF:
		digitalWrite(ORANGE_LED, HIGH);
		break;
	case RED_LED_OFF:
		digitalWrite(RED_LED, HIGH);
		break;
	case ALL_LED_ON:
		digitalWrite(BLUE_LED, LOW),digitalWrite(ORANGE_LED, LOW),digitalWrite(RED_LED, LOW);
		break;
	case BLUE_LED_ON:
		digitalWrite(BLUE_LED, LOW);
		break;
	case ORANGE_LED_ON:
		digitalWrite(ORANGE_LED, LOW);
		break;
	case RED_LED_ON:
		digitalWrite(RED_LED, LOW);
		break;
	default:
		break;
	}
}

static void led_clear_sequence(void)
{
	wind_hack_param.led.state = false;
	wind_hack_param.led.cycle = 0;
	wind_hack_param.led.sequence = NULL;
	wind_hack_param.led.sequence_time = NULL;
	led_update(ALL_LED_OFF);
}

static void led_ctrl(void)
{
	if ((wind_hack_param.led.cycle == 0) || (wind_hack_param.led.sequence == NULL) || (wind_hack_param.led.sequence_time == NULL)) return;

	if (wind_hack_param.led.state == false) {
		wind_hack_param.led.state = true;
		led_update(*wind_hack_param.led.sequence);
		wind_hack_param.led.event_time = wind_hack_param.current_time;
	}
	if (wind_hack_param.current_time - wind_hack_param.led.event_time >= *wind_hack_param.led.sequence_time) {
		wind_hack_param.led.cycle--;
		if(wind_hack_param.led.cycle == 0)
		{
			led_clear_sequence();
		}
		else
		{
			wind_hack_param.led.sequence++;
			wind_hack_param.led.sequence_time++;
			wind_hack_param.led.event_time = wind_hack_param.current_time;
		}
		led_update(*wind_hack_param.led.sequence);
	}
}

static void led_set_sequence(uint8_t led_mode)
{
	wind_hack_param.led.state			= false;
	wind_hack_param.led.cycle			= led_param[led_mode].cycle;
	wind_hack_param.led.sequence		= led_param[led_mode].sequence;
	wind_hack_param.led.sequence_time	= led_param[led_mode].sequence_time;
}

//
// LDO
//
static void ldo_update(uint8_t value)
{
	(value == LDO3V_OFF) ? digitalWrite(LDO3V_EN, LOW) : digitalWrite(LDO3V_EN, HIGH);
}

//
// rtc
//
static uint8_t char_to_dec(uint8_t *chr)
{
	return (uint8_t)((chr[0] - '0') * 10 + (chr[1] - '0'));
}

static void rtc_adjustment(uint8_t* time, uint8_t* dd, uint8_t* mm, uint8_t* yyyy)
{
	uint8_t hour, min, sec, day, month, year;

	hour = char_to_dec(time); time++; time++;
	min = char_to_dec(time); time++; time++;
	sec = char_to_dec(time);

	day = char_to_dec(dd);
	month = char_to_dec(mm);
	yyyy++; yyyy++;
	year = char_to_dec(yyyy);

	RTC.setTime(hour, min, sec);
	RTC.setDate(day, month, year);
#ifdef DEBUG
	Serial.println("rtc_adjustment()");
#endif
}

//
// gps
//
#ifdef GPS_NOT_USE
#else
static void handle_gprmc(uint8_t* cmdbuf)
{
	uint8_t i = 0;
	static uint8_t* pparam[20];

	if (!rtc_gps_sync_req) {
		// command sprit
		pparam[i] = strtok(cmdbuf,",");
		do {
			i++;
		} while((pparam[i] = strtok(NULL,",")) != NULL);
		// No problem : Warning : W5028 : Assignment within conditional expression

		if (strcmp(pparam[2], NMEA_VALID_GPRMC) == 0) {
			led_set_sequence(LED_GPRMC_ACTIVE);
		}
	}
}

static bool rtc_gps_sync(uint8_t* cmdbuf)
{
	uint8_t i = 0;
	static uint8_t* pparam[20];

	pparam[i] = strtok(cmdbuf,",.");					// command sprit
	do {
		i++;
	} while((pparam[i] = strtok(NULL,",.")) != NULL);
	// No problem : Warning : W5028 : Assignment within conditional expression

	if (strcmp(pparam[5], NMEA_INVALID_YEAR) != 0) {
		rtc_adjustment(pparam[1], pparam[3], pparam[4], pparam[5]);
		return true;
	} else {
		return false;
	}
}

static void command_decoder(uint8_t* cmdbuf)
{
	static uint8_t cmdbuf2[GPS_SEN_SIZE+1];
	static uint8_t cmdbuf3[GPS_BUF_SIZE];
#ifdef DEBUG
	uint8_t rtcdata[20];
#endif

	strtok(cmdbuf,"\r\n");
	strncpy(cmdbuf2, cmdbuf, GPS_SEN_SIZE);
	cmdbuf2[GPS_SEN_SIZE] = NULL;

	if (strcmp(cmdbuf2, NMEA_GPRMC) == 0) {
#ifdef DEBUG
		Serial.println(cmdbuf);
#endif // DEBUG
		strcpy(cmdbuf3, cmdbuf);
		handle_gprmc(cmdbuf3);
		if (wind_hack_param.file == FILE_OPENED) {
			led_update(ORANGE_LED_ON);
#ifdef BIN_FORMAT
			File.write(&myFile, cmdbuf, strlen(cmdbuf));
#else
			File.write(&myFile, strcat(cmdbuf, "\r\n"), strlen(cmdbuf));
#endif // BIN_FORMAT
			led_update(ORANGE_LED_OFF);
		} else if (wind_hack_param.file == FILE_NOT_USE) {
// ble
#ifdef DEBUG
#else
			if((BLE.getStatus() == SERIAL_CONNECT) && (BLE.tx_available() == BLE_TX_BUF_SIZE)) {
				led_update(BLUE_LED_ON);
#ifdef BIN_FORMAT
				BLE.write(cmdbuf, strlen(cmdbuf));
#else
				BLE.write(strcat(cmdbuf, "\r\n"), strlen(cmdbuf));
#endif // BIN_FORMAT
				led_update(BLUE_LED_OFF);
			}
#endif // DEBUG
		}
	} else if (rtc_gps_sync_req && (strcmp(cmdbuf2, NMEA_GPZDA) == 0)) {
#ifdef DEBUG
		Serial.println(cmdbuf);
#endif
		if (rtc_gps_sync(cmdbuf)) {
#ifdef DEBUG
			Serial.println("rtc is sync'd with gps.");
			Print.init(rtcdata,sizeof(rtcdata));
			Print.l(RTC.getYear(), 10);
			Print.p("/");
			Print.l(RTC.getMonth(), 10);
			Print.p("/");
			Print.l(RTC.getDay(), 10);
			Print.p(" ");
			Print.l(RTC.getHours(), 10);
			Print.p(":");
			Print.l(RTC.getMinutes(), 10);
			Print.p(":");
			Print.l(RTC.getSeconds(), 10);
			Print.ln();
			Serial.print(rtcdata);
#endif
			rtc_gps_sync_req = false;
		} else {
#ifdef DEBUG
			Serial.println("rtc sync failed.");
#endif
		}
	} else {
		// do nothing
	}
}
#endif // GPS_NOT_USE

//
// battery check
//
static void low_bat_check_isr(void)
{
	bat_check_irq = true;
}

// return true, if battery is lower than threshold
static bool low_bat_check(uint8_t threshold)
{
	if (voltage_check(threshold) < threshold) {
		return true;
	} else {
		return false;
	}
}

//
// switch
//
static void power_switch_isr(void)
{
	pwr_sw_irq = true;
}

static void power_off2on(void)
{
	if (digitalRead(RF_SW) == LOW) {					// RF SW is ON
		kxg03_sync_init2(KXG03_DEVICE_ADDRESS_4E, SAMPLING_RATE_RF_ON, kxg03_isr);
	} else {											// RF SW is OFF
		// set next alarm for low battery check
		RTC.attachInterrupt(low_bat_check_isr);			// attach low battery check interrupt handler
		RTC.setAlarmTime(0, 0, (uint8_t)((RTC.getSeconds()+30) % 60));		// every 30 seconds
		RTC.enableAlarm(MATCH_SS);
		kxg03_sync_init2(KXG03_DEVICE_ADDRESS_4E, SAMPLING_RATE_RF_OFF, kxg03_isr);
	}

	bm1383.init(BM1383GLV_DEVICE_ADDRESS);
	bm1422.init(BM1422_DEVICE_ADDRESS_0E);
#ifdef GPS_NOT_USE
#else
	Serial2.flush();						// refresh serial buffer data for gps
#endif // GPS_NOT_USE
}

static void power_on2off(void)
{
	// close file
	File.close(&myFile);
	// disable low battery check
	RTC.disableAlarm();
	// stop sensors
	kxg03.standby();									// make it enter stand-by mode
	bm1383.power_down();								// make it enter power down mode
	bm1422.power_down();								// make it enter power down mode
}

//
// state machine
//
static WIND_HACK_STATE func_boot_init(void)
{
	WIND_HACK_STATE mode = STATE_BOOT_INIT;

	if (rtc_gps_sync_req) {
		led_update(ALL_LED_ON);
	} else {
		led_update(ALL_LED_OFF);
		if (digitalRead(PWR_SW) == LOW) {
#ifdef DEBUG
			Serial.println("power switch is already ON in STATE_BOOT_INIT.");
#endif
			if (digitalRead(RF_SW) == LOW) {	// RF_SW is on
				wind_hack_param.file = FILE_NOT_USE;
			} else {							// RF_SW is off, use sd card
				wind_hack_param.file = FILE_OPEN_REQ;
			}
			led_set_sequence(LED_POWER_ON);
		}
		mode = STATE_POWER_OFF;							// go to next state
#ifdef DEBUG
		Serial.println("go to STATE_POWER_OFF.");
#endif
	}

	return mode;
}

static WIND_HACK_STATE func_power_off(void)
{
	WIND_HACK_STATE mode = STATE_POWER_OFF;

	if 	(wind_hack_param.led.state == false) {
		switch (wind_hack_param.file) {
		case FILE_OPEN_REQ:
			if (SD.begin(SDSPI_SS_PIN)) {				// Is sd initialization ok?
#ifdef DEBUG
				Serial.println("sd initializing successed.");
#endif
				make_filename();
				if (SD.open(filename, FILE_WRITE, &myFile)) {
#ifdef DEBUG
					Serial.println("file open successed and go to STATE_POWER_ON.");
#endif
					power_off2on();
					wind_hack_param.file = FILE_OPENED;
					mode = STATE_POWER_ON;
				} else {
#ifdef DEBUG
					Serial.println("file open failed and stay STATE_POWER_OFF.");
#endif
					led_set_sequence(LED_SDCARD_ERROR);
					wind_hack_param.file = FILE_CLOSED;
				}
			} else {
#ifdef DEBUG
				Serial.println("sd initializing failed. ldo will be off.");
#endif
				led_set_sequence(LED_SDCARD_ERROR);
				wind_hack_param.file = FILE_CLOSED;
			}
			break;
		case FILE_OPENED:								// just in case
			power_on2off();
			ldo_update(LDO3V_OFF);
			wind_hack_param.file = FILE_CLOSED;
			break;
		case FILE_NOT_USE:
			power_off2on();
			mode = STATE_POWER_ON;
			break;
		case FILE_CLOSED:
			wait_event(&pwr_sw_irq);
			delay(5);					// for debounce
			if (digitalRead(PWR_SW) == LOW) {
#ifdef DEBUG
				Serial.println("power switch is changed to ON in STATE_POWER_OFF.");
#endif
				if (low_bat_check(LOWBAT_NOT_TIMER_THRESLD)) {	// low battery
					led_set_sequence(LED_LOW_BATTERY);
				} else {
					if (digitalRead(RF_SW) == LOW) {	// RF_SW is on
						wind_hack_param.file = FILE_NOT_USE;
					} else {							// RF_SW is off, use sd card
						wind_hack_param.file = FILE_OPEN_REQ;
					}
					led_set_sequence(LED_POWER_ON);
				}
			}
			break;
		default:
			break;
		}
	}

	return mode;
}

static WIND_HACK_STATE func_power_on(void)
{
	WIND_HACK_STATE mode = STATE_POWER_ON;
	static uint32_t	p_pwr_sw_ts = 0;					// previous time stamp for debounce
	static bool	deb_start = false;						// debounce start flag

#ifdef DEBUG
#else
	 if ((wind_hack_param.file == FILE_NOT_USE) && (BLE.getStatus() == DEEPSLEEP)) {
		BLE.advertise(true);
	}
#endif

	if (pwr_sw_irq) {
		if (!deb_start) {
			deb_start = true;
			p_pwr_sw_ts = millis();
		}
		if ((millis() - p_pwr_sw_ts) >= 5) {
			deb_start = false;
			pwr_sw_irq = false;
			if (digitalRead(PWR_SW) == HIGH) {
#ifdef DEBUG
				Serial.println("power switch is changed to OFF in STATE_POWER_ON.");
#endif
				power_on2off();
#ifdef DEBUG
#else
				if (wind_hack_param.file == FILE_NOT_USE) BLE.advertise(false);
#endif
				led_update(ALL_LED_OFF);
				led_set_sequence(LED_POWER_OFF);
				wind_hack_param.file = FILE_CLOSED;
				mode = STATE_POWER_OFF;
#ifdef DEBUG
				Serial.println("go to STATE_POWER_OFF.");
#endif
			}
		}
	}

	if (bat_check_irq) {
		bat_check_irq = false;
#ifdef DEBUG
		Serial.println("battery check event occured.");
#endif
		if (low_bat_check(LOWBAT_TIMER_THRESLD)) {		// low battery check
			led_set_sequence(LED_LOW_BATTERY);
			power_on2off();
			ldo_update(LDO3V_OFF);
			wind_hack_param.file = FILE_CLOSED;
			mode = STATE_POWER_OFF;
#ifdef DEBUG
			Serial.println("go to STATE_POWER_OFF.");
#endif
		} else {
			RTC.setAlarmTime(0, 0, (uint8_t)((RTC.getSeconds()+30) % 60));		// every 30 seconds
			File.close(&myFile);
			if (SD.exists(filename2) && SD.open(filename, FILE_WRITE_APPEND, &myFile)) {
#ifdef DEBUG
				Serial.println("file reopen successed and stay STATE_FILE_OPENED.");
#endif
			}
		}
	}

	return mode;
}

//
// setup
//
void setup()
{
	const uint8_t gps_cmd[] = {"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*28"};   // GPRMC only
	static struct BLE_LED_CONFIG ble_led_conf = {
	{				// sleep
		0,			// cycle
		BLUE_LED,	// pin
		1,			// on time
		100			// off time
	},
	{				// advertise
		3,			// cycle
		BLUE_LED,	// pin
		1,		// on time
		100			// off time
	},
	{				// connect
		1,			// cycle
		BLUE_LED,	// pin
		200,			// on time
		100			// off time
	},
	{				// serial_connect
		0,			// cycle
		BLUE_LED,	// pin
		10,			// on time
		100			// off time
	}
};

#ifdef DEBUG
	Serial.begin(115200);
	Serial.println(VERSION);
#endif

	pinMode(BLUE_LED, OUTPUT);
	pinMode(ORANGE_LED, OUTPUT);
	pinMode(RED_LED, OUTPUT);
	led_clear_sequence();

	pinMode(LDO3V_EN, OUTPUT);
	ldo_update(LDO3V_ON);

	pinMode(PWR_SW, INPUT_PULLUP);
	pinMode(RF_SW, INPUT_PULLUP);

	RTC.begin();                           				// start RTC
	Wire.begin();

// ble
#ifdef DEBUG
#else
	BLE.begin(NULL);
	BLE.setLedConfig(&ble_led_conf);
	BLE.setTxFlowCtrl(true);
//	BLE.setConnectInterval(CONNECTION_INT_FAST,1000,3000);	// testing purpose only
#endif

	wind_hack_param.func_mode	= STATE_BOOT_INIT;
	wind_hack_param.file		= FILE_CLOSED;

	attachInterrupt(0, power_switch_isr, CHANGE);		// attach power switch interrupt handler

	File.init(&myFile);

	do {
		sleep(1000);
	}
	while (low_bat_check(LOWBAT_NOT_TIMER_THRESLD));

#ifdef GPS_NOT_USE
	rtc_gps_sync_req = false;
#else
	Serial2.begin(9600L);
	Serial2.println(gps_cmd);
	rtc_gps_sync_req = true;
#endif // GPS_NOT_USE
}

//
// loop
//
void loop()
{
	uint32_t tmp, deltaT;
	static uint32_t p_sensor_ts = 0;
	static uint16_t seq_num = 0;
#ifdef BIN_FORMAT
	BIN_DATA motion_data = {{'$','M'},0,};
#else
 	static uint8_t txdata[SENSOR_BUF_SIZE];
	static uint8_t kxg03_buf[KXG03_BUF_SIZE];
	static uint8_t bm1422_buf[BM1422_BUF_SIZE];
	static float bm1383_data[2];
#endif // BIN_FORMAT

#ifdef GPS_NOT_USE
#else
	static uint8_t bufp = 0;
	static uint8_t gps_buf[GPS_BUF_SIZE];
	uint8_t data;

	// gps data
	if (Serial2.available() > 0) {
		data = (uint8_t) Serial2.read();
		if ((data == 0x0D) || (data == 0x0A)) {			// if end of line
			if ((bufp > 0) && (bufp < GPS_BUF_SIZE)) {
				gps_buf[bufp] = NULL;
				command_decoder(gps_buf);
			}
			bufp = 0;
		} else if (bufp < GPS_BUF_SIZE) {
			gps_buf[bufp++] = data;
		}
	}
#endif // GPS_NOT_USE

	// sensor data
	if (kxg03_irq) {
		kxg03_irq = false;

		tmp = millis();
		deltaT = tmp - p_sensor_ts;
		p_sensor_ts = tmp;

#ifdef BIN_FORMAT
		motion_data.seq_num = seq_num++;
		motion_data.deltaT = (uint16_t)deltaT;
		kxg03_get_val_hex(motion_data.acc_data, motion_data.gyro_data);
		bm1422_get_val_hex(motion_data.mag_data);
		bm1383_get_val_hex(&motion_data.baro_data);
#else // BIN_FORMAT
		kxg03_get_val_str(kxg03_buf);
		bm1422_get_val_str(bm1422_buf);
		bm1383.get_val(bm1383_data);

		Print.init(txdata,sizeof(txdata));
		Print.p("$MOTION,,");
		Print.l((uint32_t)seq_num++, DEC);				// sequence number
		Print.p(",");
		Print.l((uint32_t)deltaT, DEC);					// delta t
		Print.p(",");
		Print.p(kxg03_buf);								// accel & gyro
		Print.p(bm1422_buf);							// mag
		Print.f((float)bm1383_data[1], 3);				// baro
		Print.ln();
#ifdef DEBUG
		Serial.print(txdata);
#endif // DEBUG
#endif // BIN_FORMAT
		if (wind_hack_param.file == FILE_OPENED) {
			led_update(ORANGE_LED_ON);
#ifdef BIN_FORMAT
			File.write(&myFile, (const uint8_t*)(&motion_data), sizeof(BIN_DATA));
#else
			File.write(&myFile, txdata, strlen(txdata));
#endif // BIN_FORMAT
			led_update(ORANGE_LED_OFF);
		} else if (wind_hack_param.file == FILE_NOT_USE) {
// ble
#ifdef DEBUG
#else
			if((BLE.getStatus() == SERIAL_CONNECT) && (BLE.tx_available() == BLE_TX_BUF_SIZE)) {
				led_update(BLUE_LED_ON);
#ifdef BIN_FORMAT
				BLE.write((uint8_t*)(&motion_data), sizeof(BIN_DATA));
#else
				BLE.print(txdata);
#endif // BIN_FORMAT
				led_update(BLUE_LED_OFF);
			}
#endif
		}
	}

	// main routine
	wind_hack_param.func_mode = functions[wind_hack_param.func_mode]();

	// led control
	wind_hack_param.current_time = millis();
	led_ctrl();
}
