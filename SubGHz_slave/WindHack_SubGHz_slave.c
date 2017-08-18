/* FILE NAME: WindHack_SubGHz_slave.c
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
#include "WindHack_SubGHz_slave_ide.h"		// Additional Header
#include <stdlib.h>
#include <string.h>

//
// macro switch
//

//
// macro definition
//
// pin definition
#define ORANGE_LED			( 25 )
#define BLUE_LED			( 26 )
// sensor
#define CMD_START			( "st" )
#define CMD_STOP			( "sp" )
#define SAMPLING_RATE		KXG03_ODR_25HZ
// subghz
#define SUBGHZ_CH			( 50 )
#define SUBGHZ_PANID		( 0xabcd )
#define SUBGHZ_BUF_SIZE		( 220 )
#define SUBGHZ_ADDR_CFG_TIMEOUT ( 10000 )
// communication
#define CMD_LINE_SIZE 		( 32 )

//
// typedef
//
typedef __packed struct {
	uint16_t seq_num;
	uint16_t deltaT;
	uint8_t kxg03_raw[12];
	uint8_t bm1422_raw[6];
	uint8_t bm1383_raw[5];
} TX_RAW;

//
// sensor
//
bool kxg03_irq = false; // true = kxg03 interrupt occured, false = interrupt cleared

static void kxg03_isr(void)
{
	kxg03_irq = true;
}

static uint8_t kxg03_get_raw_val(uint8_t* data)
{
	uint8_t rc;
	
	rc = kxg03.read(KXG03_GYRO_XOUT_L,data,12);
	if(rc!=12) {
		Serial.print("I2C ERROR=");
		Serial.println_long((long)rc,DEC);
	}
	return rc;
}

static void sensor_start(void)
{
	kxg03.sync_init(KXG03_DEVICE_ADDRESS_4E, SAMPLING_RATE, kxg03_isr);
	bm1383.init(BM1383GLV_DEVICE_ADDRESS);
	bm1422.init(BM1422_DEVICE_ADDRESS_0E);
}

static void sensor_stop(void)
{
	kxg03.standby();									// make it enter stand-by mode
	bm1383.power_down();								// make it enter power down mode
	bm1422.power_down();								// make it enter power down mode
}

//
// subghz
//
uint16_t my_addr, gw_addr, master_addr, slave0_addr, slave1_addr;
bool subghz_addr_check_irq = false; // true = subghz address check interrupt occured, false = interrupt cleared
bool user_input = false;
int cmd_num;
char cmd[CMD_LINE_SIZE];

// return false, if one of the SubGHz address is 0xFFFF.
static bool subghz_addr_check(void)
{
	int i = 0;
	uint16_t addr;
	bool res = false;

	addr = Flash.read(0,i++);
	if (addr != 0xFFFF) {
		gw_addr = addr;
		addr = Flash.read(0,i++);
		if (addr != 0xFFFF) {
			master_addr = addr;
			addr = Flash.read(0,i++);
			if (addr != 0xFFFF) {
				slave0_addr = addr;
				addr = Flash.read(0,i++);
				if (addr != 0xFFFF) {
					slave1_addr = addr;
					res = true;
				}
			}
		}
	}
	return res;
}

static void subghz_addr_check_isr(void)
{
	if (!user_input) subghz_addr_check_irq = true;
	timer2.stop();
}

static void reset_buffer(void)
{
	cmd_num=0;
	memset(cmd,0,CMD_LINE_SIZE);
}

/*
 * read command format: r/R
 *
 */
static void read_process(void)
{
	char *sp;
	int i;
	unsigned short data;
	
	sp = cmd;

	// check 'r'
	if((*sp != 'r') && (*sp != 'R'))  goto error;

	for (i=0;i<4;i++) {
		data = Flash.read(0,i);
		Serial.print("0x");
		Serial.print_long((long)data, HEX);
		if (i!=3) Serial.print(",");
	}
	Serial.println("");

	goto read_ok;
error:
	Serial.println("format error");
read_ok:
	reset_buffer();
}

static unsigned short hex_str_to_short(uint8_t* hex_str) {
	unsigned short data = 0;
	uint8_t c,*p;
	int i;
	bool error_flag = false;

	p = hex_str;
	p++; p++;											// skip '0x'

	for (i=3;i>=0;i--,p++) {
		if ((*p >= '0') && (*p <='9')) {				// 0-9
			c = *p - '0';
		} else if ((*p >= 'A') && (*p <='F')) {			// A-F
			c = *p - 'A' + 10;
		} else if ((*p >= 'a') && (*p <='f')) {			// a-f
			c = *p - 'a' + 10;
		} else {										// other
			error_flag = true;
		}
		data += (unsigned short)(c) << (4*i);
	}
	return error_flag ? 0xFFFF : data;
}
 
/*
 * write command format: w/W,(GW Address),(Master Address),(Slave1 Address),(Slave2 Address)
 *
 */
static void write_process(void)
{
	char *sp;
	int i;
	unsigned short data;
	
	sp = cmd;

	// check 'r'
	if((*sp != 'w') && (*sp != 'W'))  goto error;
	sp++;
	for (i=0;i<4;i++) {
		// check ','
		if(*sp != ',') goto error;
		sp++;											// skip ','
		// get address & check data
		data = hex_str_to_short(sp);
		if (data != 0xFFFF) Flash.write(0, i, data);
		sp = sp + 6;
	}

	goto write_ok;
error:
	Serial.println("format error");
write_ok:
	Serial.println("done");
	reset_buffer();
}

/*
 * erase command format: e/E
 *
 */
static void erase_process(void)
{
	char *sp;
	
	sp = cmd;

	// check 'e'
	if((*sp != 'e') && (*sp != 'E'))  goto error;

	Flash.erase(0);
	
	goto write_ok;
error:
	Serial.println("format error");
write_ok:
	Serial.println("done");
	reset_buffer();
}

//
// setup
//
TX_RAW tx_raw = {0,0,{0},{0},{0}};
bool subghz_addr_cfg_req;

void setup()
{
	Serial.begin(115200);

	pinMode(ORANGE_LED,OUTPUT);
	pinMode(BLUE_LED,OUTPUT);
	digitalWrite(ORANGE_LED,HIGH);
	digitalWrite(BLUE_LED,HIGH);

	Wire.begin();
	SubGHz.init();
	my_addr = SubGHz.getMyAddress();
	SubGHz.setAckReq(false);
	SubGHz.begin(SUBGHZ_CH, SUBGHZ_PANID, SUBGHZ_100KBPS, SUBGHZ_PWR_1MW);
	SubGHz.rxEnable(NULL);

	subghz_addr_cfg_req = true;
	timer2.set(SUBGHZ_ADDR_CFG_TIMEOUT, subghz_addr_check_isr);	// set subghz address config callback
	timer2.start();
}

//
// loop
//
void loop()
{
	uint32_t tmp, deltaT;
	static uint32_t p_sensor_ts = 0;
	int itmp;
	static SUBGHZ_MAC_PARAM mac;
	static uint8_t rx_data[SUBGHZ_BUF_SIZE];
	short rx_len;
	uint16_t addr;

	if (subghz_addr_cfg_req) {
		itmp = Serial.read();
		// receiving process
		if(itmp > 0) 
		{
			user_input = true;
			switch(itmp)
			{
			case '\r':
				switch(cmd[0])
				{
				case 'r':
				case 'R':
					read_process();
					reset_buffer();
					digitalWrite(ORANGE_LED,LOW);
					delay(500);
					digitalWrite(ORANGE_LED,HIGH);
					break;
				case 'w':
				case 'W':
					write_process();
					reset_buffer();
					digitalWrite(ORANGE_LED,LOW);
					delay(500);
					digitalWrite(ORANGE_LED,HIGH);
					break;
				case 'e':
				case 'E':
					erase_process();
					reset_buffer();
					digitalWrite(BLUE_LED,LOW);
					delay(500);
					digitalWrite(BLUE_LED,HIGH);
					break;
				default:
					reset_buffer();
					break;
				}
			case '\b':
				if(cmd_num != 0)	cmd_num--;
				break;
			default:
				cmd[cmd_num] = (unsigned char)itmp;
				if(cmd_num >= CMD_LINE_SIZE) cmd_num = CMD_LINE_SIZE;
				else cmd_num++;
				break;
			}
		}

		if (subghz_addr_check_irq) {
			subghz_addr_check_irq = false;

			if (subghz_addr_check()) {
				subghz_addr_cfg_req = false;
			} else {
				subghz_addr_cfg_req = true;		// if subghz address is invalid, keep waiting for input infinitely.
			}
		}
	} else {
		// sensor data
		if (kxg03_irq) {
			kxg03_irq = false;
			tmp = millis();
			deltaT = tmp - p_sensor_ts;
			p_sensor_ts = tmp;

			tx_raw.seq_num++;								// sequence number
			tx_raw.deltaT = deltaT;							// delta t
			kxg03_get_raw_val(tx_raw.kxg03_raw);			// accel & gyro
			bm1422.get_rawval(tx_raw.bm1422_raw);			// mag
			bm1383.get_rawtemppressval(tx_raw.bm1383_raw);	// baro
			digitalWrite(BLUE_LED,LOW);
			SubGHz.send(SUBGHZ_PANID, master_addr, (uint8_t*)&tx_raw, sizeof(TX_RAW), NULL);
			digitalWrite(BLUE_LED,HIGH);
		}

		rx_len = SubGHz.readData(rx_data,SUBGHZ_BUF_SIZE);
		if(rx_len>0)
		{
			SubGHz.decMac(&mac,rx_data,rx_len);
			addr = *((uint16_t*)mac.src_addr);
			if (addr == master_addr) {
				mac.payload[mac.payload_len+1] = NULL;
				if (strcmp(mac.payload, CMD_START) == 0) {
					if (my_addr == slave1_addr) delay(20);
					sensor_start();
				} else if (strcmp(mac.payload, CMD_STOP) == 0) {
					sensor_stop();
				} else {
					// do nothing
				}
			}
		}
	}
}
