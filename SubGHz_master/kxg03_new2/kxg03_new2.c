#include "lazurite.h"
#include "kxg03_new2.h"
#include "wire.h"
#include "driver_extirq.h"
#include "driver_gpio.h"
#include "math.h"

// basic access parameters
static uint8_t slave_addr;
#if 1
static uint8_t kxg03_acc_range=0;					// 2g
static uint8_t kxg03_acc_avr=KXG03_ACC_AVR_128;
static uint8_t kxg03_gyro_range=0;					// 256 deg/sec
static uint8_t kxg03_gyro_bw=0;						// 10Hz

// pry interface parameters
static int8_t g[3]={0,1,2};
static int8_t a[3]={0,1,2};
static int8_t gf[3]={1,1,1};
static int8_t af[3]={1,1,1};
static uint8_t angle_mode=0;
static float raw[6],tmp[6];

// output flags
static bool kxg03_out_flag=0;
#define KXG03_KALMAN_OUT	0x01
#define KXG03_ACC_OUT		0x02
#define KXG03_GYRO_OUT		0x04
#define KXG03_DEG_OUT		0x08

static KALMAN_PARAM k[2];
static const float Q_angle = 0.001;
static const float Q_bias = 0.003;
static const float R_measure = 0.03;

static const short acc_div[] =
{
	32768/2,
	32768/4,
	32768/8,
	32768/16,
};
static const short gyro_div[] =
{
	32768/256,
	32768/512,
	32768/1024,
	32768/2048,
};

static void kalman_init(KALMAN_PARAM *param)
{
	param->angle = 0.0;
	param->bias = 0.0;
	param->P[0][0] = 0.0;
	param->P[0][1] = 0.0;
	param->P[1][0] = 0.0;
	param->P[1][1] = 0.0;
	
	return;
}
static void kalman_set(KALMAN_PARAM *param,float newAngle)
{
	param->angle = newAngle;
	param->bias = 0.0;
	param->P[0][0] = 0.0;
	param->P[0][1] = 0.0;
	param->P[1][0] = 0.0;
	param->P[1][1] = 0.0;
	param->lastTime=millis();
	
	return;
}

static float kalman_cal(KALMAN_PARAM *param,float newAngle, float newRate){
	
	uint32_t now;
	float dt;
	float rate,y,S,K[2],P00_temp,P01_temp;
	
	// variances

	now = millis();;
	dt = (now - param->lastTime)/1000;
	
	// step 1
	rate = newRate - param->bias;
	param->angle += dt * rate;

	// step 2
	param->P[0][0] += dt * (dt * param->P[1][1] - param->P[0][1] - param->P[1][0] + Q_angle);
	param->P[0][1] -= dt * param->P[1][1];
	param->P[1][0] -= dt * param->P[1][1];
	param->P[1][1] += Q_bias * dt;

	// step 3
	y = newAngle - param->angle;

	// step 4
	S = param->P[0][0] + R_measure;

	// step 5
	K[0] = param->P[0][0] / S;
	K[1] = param->P[1][0] / S;

	// step 6
	param->angle += K[0] * y;
	param->bias += K[1] * y;

	// step 7
	P00_temp = param->P[0][0];
	P01_temp = param->P[0][1];
	param->P[0][0] -= K[0] * P00_temp;
	param->P[0][1] -= K[0] * P01_temp;
	param->P[1][0] -= K[1] * P00_temp;
	param->P[1][1] -= K[1] * P01_temp;
	
	param->lastTime = now;
	
	return param->angle;
}
#endif
static uint8_t kxg03_read(uint8_t addr,uint8_t* data,uint8_t size)
{
	byte rc;
	unsigned char cnt;
	
	Wire.beginTransmission(slave_addr);
	Wire.write_byte(addr);
	rc = Wire.endTransmission(false);
	if (rc != 0) {
//		Serial.print("read.endTransmission=0x");
//		Serial.print_long((long)addr,HEX);
//		Serial.print(",");
//		Serial.println_long((long)rc,HEX);
//		return (rc);
	}
	rc=Wire.requestFrom(slave_addr, size, true);
	cnt = 0;
	while(Wire.available())
	{
		data[cnt] = (uint8_t)Wire.read();
		cnt++;
	}
	return (rc);
}

static uint8_t kxg03_write(uint8_t addr,uint8_t* data, uint8_t size)
{
	byte rc;
	
	Wire.beginTransmission(slave_addr);
	Wire.write_byte(addr);
	Wire.write(data, size);
	rc = Wire.endTransmission(true);
	return (rc);
}

static uint8_t kxg03_init(uint8_t i2c_addr)
{
	byte rc;
	unsigned char reg;
	
	if(i2c_addr == KXG03_DEVICE_ADDRESS_4F)
	{
		slave_addr = KXG03_DEVICE_ADDRESS_4F;
	} else {
		slave_addr = KXG03_DEVICE_ADDRESS_4E;
//		slave_addr = 0x11;
	}
//	Wire.setTimeout(100);
	rc = kxg03_read(KXG03_WHO_AM_I, &reg, sizeof(reg));
	if (rc != 1) {
//		Serial.print_long((long)rc,DEC);
//		while(1){}
//			Serial.println("Can't access KXG03");
			return (rc);
	} 
//	Serial.print("KXG03_WHO_AM_I Register Value = 0x");
//	Serial.println_long((long)reg, HEX);

	if (reg != KXG03_WHO_AM_I_VAL) {
//		Serial.println("Can't find KXG03");
	// return (rc);
	}
	/*
	// rc = write(KXG03_ACCEL_ODR_WAKE, &reg, sizeof(reg));
	reg = 0xEC;
	rc = kxg03_write(KXG03_STDBY, &reg, sizeof(reg));
	if (rc != 0) {
		Serial.println("Can't write KXG03 STDBY Register");
		return (rc);
	} 
	*/
	return (rc);  
}
#if 1
static uint8_t kxg03_get_val(float* val)
{
	uint8_t rc;

	union {
		uint8_t d8[12];
		short d16[6];
	} data;
	
	rc = kxg03_read(KXG03_GYRO_XOUT_L,data.d8,12);
	if(rc!=12) {
//		Serial.print("I2C ERROR=");
//		Serial.println_long((long)rc,DEC);
	}
		
	val[0] = (float)data.d16[0]/gyro_div[kxg03_gyro_range];
	val[1] = (float)data.d16[1]/gyro_div[kxg03_gyro_range];
	val[2] = (float)data.d16[2]/gyro_div[kxg03_gyro_range];
	val[3] = (float)data.d16[3]/acc_div[kxg03_acc_range];
	val[4] = (float)data.d16[4]/acc_div[kxg03_acc_range];
	val[5] = (float)data.d16[5]/acc_div[kxg03_acc_range];
	
	return rc;
}

static uint8_t kxg03_get_acc(float* acc)
{
	uint8_t rc;
	union {
		uint8_t d8[6];
		short d16[3];
	} data;
	
	rc = kxg03_read(KXG03_ACCEL_XOUT_L,data.d8,6);
	if(!rc) return rc;

	acc[0] = (float)data.d16[0]/acc_div[kxg03_acc_range];
	acc[1] = (float)data.d16[1]/acc_div[kxg03_acc_range];
	acc[2] = (float)data.d16[2]/acc_div[kxg03_acc_range];
	
	return rc;
}

static uint8_t kxg03_get_gyro(float* gyro)
{
	uint8_t rc;
	union {
		uint8_t d8[6];
		short d16[3];
	} data;
	                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
	rc = kxg03_read(KXG03_GYRO_XOUT_L,data.d8,6);
	if(!rc) return rc;
	
	gyro[0] = (float)data.d16[0]/gyro_div[kxg03_gyro_range];
	gyro[1] = (float)data.d16[1]/gyro_div[kxg03_gyro_range];
	gyro[2] = (float)data.d16[2]/gyro_div[kxg03_gyro_range];
	
	return rc;
}


static const char kxg03_err_msg[][16] =
{
	"STDBY",
	"INT1_L",
	"INT_MASK1",
	"INT_PIN_CTL",
	"INT_PIN1_SEL",
	"ACCEL_ODR_WAKE",
	"GYRO_ODR_WAKE",
	"ACCEL_CTL"
};

void kxg03_err_msg_out(uint8_t id, uint8_t val)
{
//	Serial.print("KXG03_");
//	Serial.print(kxg03_err_msg[id]);
//	Serial.print("=");
//	Serial.println_long(val,DEC);
}

static int kxg03_set_acc_avr(uint8_t acc_avr)
{
	int ret;
	switch(acc_avr)
	{
	case KXG03_ACC_AVR_1:
	case KXG03_ACC_AVR_2:
	case KXG03_ACC_AVR_4:
	case KXG03_ACC_AVR_8:
	case KXG03_ACC_AVR_16:
	case KXG03_ACC_AVR_32:
	case KXG03_ACC_AVR_64:
	case KXG03_ACC_AVR_128:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_1:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_2:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_4:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_8:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_16:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_32:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_64:
	case KXG03_ACC_LPMODE | KXG03_ACC_AVR_128:
		kxg03_acc_avr = (acc_avr >> 4);
		ret = 0;
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}

static int kxg03_set_acc_range(uint8_t acc_range)
{
	int ret;
	switch(acc_range)
	{
	case KXG03_ACC_RANGE_2G:
	case KXG03_ACC_RANGE_4G:
	case KXG03_ACC_RANGE_8G:
	case KXG03_ACC_RANGE_16G:
		kxg03_acc_range = (acc_range >> 2);
		ret = 0;
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}


/*
// int kxg03_set_gyro_range(uint8_t gyro_range);
	parameters:
		#define KXG03_GYRO_RANGE_256	0x00
		#define KXG03_GYRO_RANGE_512	0x40
		#define KXG03_GYRO_RANGE_1024	0x80
		#define KXG03_GYRO_RANGE_2048	0xC0
	ret val:
		0:  success
		-1: wrong parameters
*/
static int kxg03_set_gyro_range(uint8_t gyro_range)
{
	int ret;
	switch(gyro_range)
	{
	case KXG03_GYRO_RANGE_256:
	case KXG03_GYRO_RANGE_512:
	case KXG03_GYRO_RANGE_1024:
	case KXG03_GYRO_RANGE_2048:
		kxg03_gyro_range = (gyro_range >> 6);
		ret = 0;
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}

/*
	static int kxg03_set_gyro_bw(uint8_t gyro_bw)
	parameters:
		#define KXG03_GYRO_BW_10HZ		0x00
		#define KXG03_GYRO_BW_10HZ		0x10
		#define KXG03_GYRO_BW_10HZ		0x20
		#define KXG03_GYRO_BW_10HZ		0x30
	ret_val:
		0:	success
		-1: wrong parameter
*/
static int kxg03_set_gyro_bw(uint8_t gyro_bw)
{
	int ret;
	switch(gyro_bw)
	{
	case KXG03_GYRO_BW_10HZ:
	case KXG03_GYRO_BW_20HZ:
	case KXG03_GYRO_BW_40HZ:
	case KXG03_GYRO_BW_160HZ:
		kxg03_gyro_bw = (gyro_bw >> 4);
		ret = 0;
		break;
	default:
		ret = -1;
		break;
	}
	return ret;
}
/*
static uint8_t kxg03_sync_init(uint8_t slave_addr,uint8_t odr_rate,void (*func)(void));
	parameters:
		slave_addr:
			#define KXG03_DEVICE_ADDRESS_4E   (0x4E)    // 7bit Addrss
			#define KXG03_DEVICE_ADDRESS_4F   (0x4F)    // 7bit Address
		odr_rate:
			#define KXG03_ODR_0_781HZ		0
			#define KXG03_ODR_1_563HZ		1
			#define KXG03_ODR_3_125HZ		2
			#define KXG03_ODR_6_25HZ		3
			#define KXG03_ODR_12_5HZ		4
			#define KXG03_ODR_25HZ			5
			#define KXG03_ODR_50HZ			6
			#define KXG03_ODR_100HZ			7
			#define KXG03_ODR_200HZ			8
			#define KXG03_ODR_400HZ			9
			#define KXG03_ODR_800HZ			10
			#define KXG03_ODR_1600HZ		11
			#define KXG03_ODR_3200HZ		12
			#define KXG03_ODR_6400HZ		13
			#define KXG03_ODR_12800HZ		14
			#define KXG03_ODR_51200HZ		15
		func:
			callback function by INT1
		
*/
static uint8_t kxg03_sync_init(uint8_t slave_addr,uint8_t odr_rate,void (*func)(void))
{
	uint8_t rc;
	uint8_t data;
	
	rc = kxg03.init(slave_addr);

	// set KXG03 to stand-by
	data = 0xEF;
	rc = kxg03.write(KXG03_STDBY,&data,1);
	if(rc) {
		// kxg03_err_msg_out(0,rc);
		return rc;
	}
	
	// clear int1
	rc = kxg03.read(KXG03_INT1_L,&data,1);
	if(rc!=1) {
		// kxg03_err_msg_out(1,rc);
		return rc;
	}

	// opem intmask of ACC and GYRO
	data = 0x03;
	rc = kxg03.write(KXG03_INT_MASK1,&data,1);
	if(rc) {
		// kxg03_err_msg_out(2,rc);
		return rc;
	}

	// set interrupt mode
	data = 0x0E;
	rc = kxg03.write(KXG03_INT_PIN_CTL,&data,1);
	if(rc) {
		// kxg03_err_msg_out(3,rc);
		return rc;
	}
	
	// Enable interrupt from ACC, GYRO
	data = 0x03;
	rc = kxg03.write(KXG03_INT_PIN1_SEL,&data,1);
	if(rc) {
		// kxg03_err_msg_out(4,rc);
		return rc;
	}

	// set ACC ODR
	data = odr_rate | (kxg03_acc_avr<<4);
	rc = kxg03.write(KXG03_ACCEL_ODR_WAKE ,&data,1);
	if(rc) {
		// kxg03_err_msg_out(5,rc);
		return rc;
	}
	
	// set GYRO ODR to 25Hz
	data = odr_rate | (kxg03_gyro_range<<6) | (kxg03_gyro_bw<<4);
	rc = kxg03.write(KXG03_GYRO_ODR_WAKE,&data,1);
	if(rc) {
		// kxg03_err_msg_out(6,rc);
		return rc;
	}
	
		// set ACC_RANGE
	data = (kxg03_acc_range<<2);
	rc = kxg03.write(KXG03_ACCEL_CTL,&data,1);
	if(rc) {
		// kxg03_err_msg_out(7,rc);
		return rc;
	}

	
	// set Interrupt
	drv_pinMode(9,INPUT);
	drv_attachInterrupt(9,6, func, RISING,false, false);

	// Start sensor
	data = 0xec;
	rc = kxg03.write(KXG03_STDBY,&data,1);
	if(rc) {
		// kxg03_err_msg_out(0,rc);
		return rc;
	}
	
	return rc;
}

/*
	static void kxg03_pry_init(uint8_t mode, uint8_t gx, uint8_t gy, uint8_t gz, uint8_t ax, uint8_t ay, uint8_t az);
	parameters:
		mode =	0x00: pitch, roll
					apitch = atan( ax / sqrt(ay^2+az^2)) * RAD_TO_DEG
					aroll  = atan( ay / abs( az )) * RAD_TO_DEG
					az > 0 : aroll + 90
					az < 0 : aroll - 90
				0x01: pitch, slope
					apitch = atan( ax / sqrt(ay^2+az^2)) * RAD_TO_DEG
					aslope = atan( ay / sqrt(ax^2+az^2)) * RAD_TO_DEG
		gx,gy,gz:  table of axis  0 = x, 1 = y, 2 = z
			(ex)  gx = 2   z axis data is used as x axis data for culcuration
		ax,ay,az:  table of axis  0 = x, 1 = y, 2 = z
			(ex)  ax = 2   z axis data is used as x axis data for culcuration
*/
static void kxg03_angle_init(int8_t mode, int8_t gx, int8_t gy, int8_t gz, int8_t ax, int8_t ay, int8_t az)
{
#if 0
	float adata[2];
	angle_mode = mode;
	g[0] = abs(gx)-1;
	g[1] = abs(gy)-1;
	g[2] = abs(gz)-1;
	a[0] = abs(ax)+2;
	a[1] = abs(ay)+2;
	a[2] = abs(az)+2;
	gf[0] = gx>0 ? 1 : -1;
	gf[1] = gy>0 ? 1 : -1;
	gf[2] = gz>0 ? 1 : -1;
	af[0] = ax>0 ? 1 : -1;
	af[1] = ay>0 ? 1 : -1;
	af[2] = az>0 ? 1 : -1;
	
	kalman_init(&k[0]);
	kalman_init(&k[1]);
	
	kxg03_get_val(tmp);
	raw[0] = tmp[g[0]]*gf[0];
	raw[1] = tmp[g[1]]*gf[1];
	raw[2] = tmp[g[2]]*gf[2];
	raw[3] = tmp[a[0]]*af[0];
	raw[4] = tmp[a[1]]*af[1];
	raw[5] = tmp[a[2]]*af[2];
	switch(angle_mode)
	{
	case KXG03_MODE_KALMAN | KXG03_MODE_PR:
		adata[0] = atan2(tmp[4]*-1,sqrt(sq(tmp[3])+sq(tmp[5]))) * RAD_TO_DEG;
		adata[1] = atan2(tmp[3],fabs(tmp[5])) * RAD_TO_DEG;
//		if((raw[5] < 0) && ( raw[4] > 0)) adata[1] = 180-adata[1];
//		if((raw[5] < 0) && ( raw[4] < 0)) adata[1] = -180-adata[1];
		kalman_set(&k[0],adata[0]);
		kalman_set(&k[1],adata[1]);
		break;
	case KXG03_MODE_KALMAN | KXG03_MODE_PS:
		adata[0] = atan2(raw[3],sqrt(sq(raw[4])+sq(raw[5]))) * RAD_TO_DEG;
		adata[1] = atan2(raw[4],sqrt(sq(raw[5])+sq(raw[3]))) * RAD_TO_DEG;
		if((raw[5] < 0) && ( raw[4] > 0)) adata[1] = 180-adata[1];
		if((raw[5] < 0) && ( raw[4] < 0)) adata[1] = -180-adata[1];
		kalman_set(&k[0],adata[0]);
		kalman_set(&k[1],adata[1]);
		break;
	}
	return;
#endif
}

/*
	static void kxg03_get_angle(float* kangle,float* ganlgle, float* aangle)
	parameters:
	kangle:  pointer of pitch and roll/slope angle with kalman filter
			in case of null, data is not output.
	gangle:  pointer of pitch and roll/slope angle of gyro raw data
			in case of null, data is not output.
	aangle:  pointer of pitch and roll/slope angle of calcurated data by acc
			in case of null, data is not output.
*/
static int kxg03_get_angle(float* angle)
{
#if 0
	float kdata[2],adata[2];
	int i=0;
	kxg03_get_val(tmp);
	raw[0] = tmp[g[0]]*gf[0];
	raw[1] = tmp[g[1]]*gf[1];
	raw[2] = tmp[g[2]]*gf[2];
	raw[3] = tmp[a[0]]*af[0];
	raw[4] = tmp[a[1]]*af[1];
	raw[5] = tmp[a[2]]*af[2];
	switch(angle_mode)
	{
	case KXG03_MODE_PR:
		adata[0] = atan2(raw[0],sqrt(sq(raw[1])+sq(raw[2]))) * RAD_TO_DEG;
		adata[1] = atan2(raw[1],fabs(raw[2])) * RAD_TO_DEG;
		break;
	case KXG03_MODE_PS:
		adata[0] = atan2(raw[0],sqrt(sq(raw[1])+sq(raw[2]))) * RAD_TO_DEG;
		adata[1] = atan2(raw[1],sqrt(sq(raw[0])+sq(raw[2]))) * RAD_TO_DEG;
		break;
	case KXG03_MODE_KALMAN | KXG03_MODE_PR:
		adata[0] = atan2(tmp[4]*-1,sqrt(sq(tmp[3])+sq(tmp[5]))) * RAD_TO_DEG;
		adata[1] = atan2(tmp[3],fabs(tmp[5])) * RAD_TO_DEG;
//		if((raw[5] < 0) && ( raw[4] > 0)) adata[1] = 180-adata[1];
//		if((raw[5] < 0) && ( raw[4] < 0)) adata[1] = -180-adata[1];
		kdata[0]=kalman_cal(&k[0],adata[0],raw[0]);
		kdata[1]=kalman_cal(&k[1],adata[1],raw[1]);
		break;
	case KXG03_MODE_KALMAN | KXG03_MODE_PS:
		adata[0] = atan2(raw[3],sqrt(sq(raw[4])+sq(raw[5]))) * RAD_TO_DEG;
		adata[1] = atan2(raw[4],sqrt(sq(raw[5])+sq(raw[3]))) * RAD_TO_DEG;
		kdata[0]=kalman_cal(&k[0],adata[0],raw[0]);
		kdata[1]=kalman_cal(&k[1],adata[1],raw[1]);
		if((raw[5] < 0) && ( raw[4] > 0)) adata[1] = 180-adata[1];
		if((raw[5] < 0) && ( raw[4] < 0)) adata[1] = -180-adata[1];
		break;
	}
	if(kxg03_out_flag & KXG03_KALMAN_OUT)
	{
		angle[i]=kdata[0],i++;
		angle[i]=kdata[1],i++;
	}
	if(kxg03_out_flag & KXG03_DEG_OUT)
	{
		angle[i]=adata[0],i++;
		angle[i]=adata[1],i++;
	}
	if(kxg03_out_flag & KXG03_GYRO_OUT)
	{
		angle[i]=raw[0],i++;
		angle[i]=raw[1],i++;
		angle[i]=raw[2],i++;
	}
	if(kxg03_out_flag & KXG03_ACC_OUT)
	{
		angle[i]=raw[3],i++;
		angle[i]=raw[4],i++;
		angle[i]=raw[5],i++;
	}
	return i;
#endif
}
void kxg03_set_kalman_out(bool kalman_out)
{
	if(kalman_out) kxg03_out_flag |= KXG03_KALMAN_OUT;
	else kxg03_out_flag &= ~KXG03_KALMAN_OUT;
}
void kxg03_set_acc_out(bool acc_out)
{
	if(acc_out) kxg03_out_flag |= KXG03_ACC_OUT;
	else kxg03_out_flag &= ~KXG03_ACC_OUT;
}
void kxg03_set_gyro_out(bool gyro_out)
{
	if(gyro_out) kxg03_out_flag |= KXG03_GYRO_OUT;
	else kxg03_out_flag &= ~KXG03_GYRO_OUT;
}
void kxg03_set_deg_out(bool deg_out)
{
	if(deg_out) kxg03_out_flag |= KXG03_DEG_OUT;
	else kxg03_out_flag &= ~KXG03_DEG_OUT;
}
#endif
static uint8_t kxg03_standby(void)
{
	uint8_t rc;
	uint8_t data;

	// set KXG03 to stand-by
	data = 0xEF;
	rc = kxg03.write(KXG03_STDBY,&data,1);
	// if(rc) {
		// kxg03_err_msg_out(0,rc);
	// }
	return rc;
}
#if 1
const KXG03 kxg03 ={
	kxg03_init,					//	uint8_t (*init)(uint8_t slave_addr);
	kxg03_sync_init,			//	uint8_t (*sync_init)(uint8_t slave_addr,uint8_t odr_rate,void (*func)(void));
	kxg03_read,					// 	uint8_t (*read)(uint8_t addr, uint8_t* data, uint8_t size);
	kxg03_write,				//	uint8_t (*write)(uint8_t addr, uint8_t* data, uint8_t size);
	kxg03_get_val,				//	uint8_t (*get_val)(float* val);
	kxg03_get_gyro,				//	uint8_t (*get_gyro)(float* gyro);
	kxg03_get_acc,				//	uint8_t (*get_acc)(float* acc);
	kxg03_set_acc_avr,			//	int (*set_acc_avr)(uint8_t acc_avr);
	kxg03_set_acc_range,		//	int (*set_acc_range)(uint8_t gyro_range);
	kxg03_set_gyro_range,		//	int (*set_gyro_range)(uint8_t gyro_range);
	kxg03_set_gyro_bw,			//	int (*set_gyro_bw)(uint8_t gyro_bw);
	kxg03_angle_init,			//	void (*angle_init)(uint8_t mode, uint8_t gx, uint8_t gy, uint8_t gz, uint8_t ax, uint8_t ay, uint8_t az);
	kxg03_get_angle,			//	int (*get_angle)(float* angle);
	kxg03_set_kalman_out,		// 	void (*set_kalman_out)(bool kalman_out);
	kxg03_set_acc_out,			//	void (*set_acc_out)(bool acc_out);
	kxg03_set_gyro_out,			//	void (*set_gyro_out)(bool gyro_out);
	kxg03_set_deg_out,			//	void (*set_deg_out)(bool deg_out);
	kxg03_standby				//  uint8_t (*standby)(void);
};

const KALMAN kalman = {
	kalman_init,
	kalman_set,
	kalman_cal
};
#endif
