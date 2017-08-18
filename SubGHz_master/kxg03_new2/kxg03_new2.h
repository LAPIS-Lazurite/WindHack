
#ifndef _KXG03_NEW_H
#define _KXG03_NEW_H

#include "lazurite.h"

#define KXG03_DEVICE_ADDRESS_4E   (0x4E)    // 7bit Addrss
#define KXG03_DEVICE_ADDRESS_4F   (0x4F)    // 7bit Address

#define KXG03_WHO_AM_I_VAL        (0x24)    // sample

// REGISTER
#define	KXG03_TEMP_OUT_L		0x00
#define	KXG03_TEMP_OUT_H		0x01
#define	KXG03_GYRO_XOUT_L		0x02
#define	KXG03_GYRO_XOUT_H		0x03
#define	KXG03_GYRO_YOUT_L		0x04
#define	KXG03_GYRO_YOUT_H		0x05
#define	KXG03_GYRO_ZOUT_L		0x06
#define	KXG03_GYRO_ZOUT_H		0x07
#define	KXG03_ACCEL_XOUT_L		0x08
#define	KXG03_ACCEL_XOUT_H		0x09
#define	KXG03_ACCEL_YOUT_L		0x0A
#define	XG03_ACCEL_YOUT_H		0x0B
#define	KXG03_ACCEL_ZOUT_L		0x0C
#define	KXG03_ACCEL_ZOUT_H		0x0D
#define	KXG03_AUX1_OUT1			0x0E
#define	KXG03_AUX1_OUT2			0x0F
#define	KXG03_AUX1_OUT3			0x10
#define	KXG03_AUX1_OUT4			0x11
#define	KXG03_AUX1_OUT5			0x12
#define	KXG03_AUX1_OUT6			0x13
#define	KXG03_AUX2_OUT1			0x14
#define	KXG03_AUX2_OUT2			0x15
#define	KXG03_AUX2_OUT3			0x16
#define	KXG03_AUX2_OUT4			0x17
#define	KXG03_AUX2_OUT5			0x18
#define	KXG03_AUX2_OUT6			0x19
#define	KXG03_WAKE_CNT_L		0x1A
#define	KXG03_WAKE_CNT_H		0x1B
#define	KXG03_SLEEP_CNT_L		0x1C
#define	KXG03_SLEEP_CNT_H		0x1D
#define	KXG03_BUF_SMPLEV_L		0x1E
#define	KXG03_BUF_SMPLEV_H		0x1F
#define	KXG03_BUF_PAST_L		0x20
#define	KXG03_BUF_PAST_H		0x21
#define	KXG03_AUX_STATUS		0x22
#define	KXG03_WHO_AM_I			0x30
#define	KXG03_SN1_MIR			0x31
#define	KXG03_SN2_MIR			0x32
#define	KXG03_SN3_MIR			0x33
#define	KXG03_SN4_MIR			0x34
#define	KXG03_STATUS1			0x36
#define	KXG03_INT1_SRC1			0x37
#define	KXG03_INT1_SRC2			0x38
#define	KXG03_INT1_L			0x39
#define	KXG03_STATUS2			0x3A
#define	KXG03_INT2_SRC1			0x3B
#define	KXG03_INT2_SRC2			0x3C
#define	KXG03_INT2_L			0x3D
#define	KXG03_ACCEL_ODR_WAKE	0x3E
#define	KXG03_ACCEL_ODR_SLEEP	0x3F
#define	KXG03_ACCEL_CTL			0x40
#define	KXG03_GYRO_ODR_WAKE		0x41
#define	KXG03_GYRO_ODR_SLEEP	0x42
#define	KXG03_STDBY				0x43
#define	KXG03_CTL_REG_1			0x44
#define	KXG03_INT_PIN_CTL		0x45
#define	KXG03_INT_PIN1_SEL		0x46
#define	KXG03_INT_PIN2_SEL		0x47
#define	KXG03_INT_MASK1			0x48
#define	KXG03_INT_MASK2			0x49
#define	KXG03_FSYNC_CTL			0x4A
#define	KXG03_WAKE_SLEEP_CTL1	0x4B
#define	KXG03_WAKE_SLEEP_CTL2	0x4C
#define	KXG03_WUF_TH			0x4D
#define	KXG03_WUF_COUNTER 		0x4E
#define	KXG03_BTS_TH			0x4F
#define	KXG03_BTS_COUNTER		0x50
#define	KXG03_AUX_I2C_CTRL_REG	0x51
#define	KXG03_AUX_I2C_SAD1		0x52
#define	KXG03_AUX_I2C_REG1		0x53
#define	KXG03_AUX_I2C_CTL1		0x54
#define	KXG03_AUX_I2C_BIT1		0x55
#define	KXG03_AUX_I2C_ODR1_W 	0x56
#define	KXG03_AUX_I2C_ODR1_S 	0x57
#define	KXG03_AUX_I2C_SAD2		0x58
#define	KXG03_AUX_I2C_REG2		0x59
#define	KXG03_AUX_I2C_CTL2		0x5A
#define	KXG03_AUX_I2C_BIT2		0x5b
#define	KXG03_AUX_I2C_ODR2_W	0x5C
#define	KXG03_AUX_I2C_ODR2_S	0x5D
#define	KXG03_BUF_WMITH_L		0x75
#define	KXG03_BUF_WMITH_H		0x76
#define	KXG03_BUF_TRIGTH_L		0x77
#define	KXG03_BUF_TRIGTH_H		0x78
#define	KXG03_BUF_CTL2			0x79
#define	KXG03_BUF_CTL3			0x7A
#define	KXG03_BUF_CTL4			0x7B
#define	KXG03_BUF_EN			0x7C
#define	KXG03_BUF_STATUS		0x7D
#define	KXG03_BUF_CLEAR			0x7E
#define	KXG03_BUF_READ			0x7F

// ODR PARAMETER
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

// ACC_AVERAGE
#define KXG03_ACC_AVR_1			0x00
#define KXG03_ACC_AVR_2			0x10
#define KXG03_ACC_AVR_4			0x20
#define KXG03_ACC_AVR_8			0x30
#define KXG03_ACC_AVR_16		0x40
#define KXG03_ACC_AVR_32		0x50
#define KXG03_ACC_AVR_64		0x60
#define KXG03_ACC_AVR_128		0x70
#define KXG03_ACC_LPMODE		0x80

// ACC_RANGE
#define KXG03_ACC_RANGE_2G		0x00
#define KXG03_ACC_RANGE_4G		0x04
#define KXG03_ACC_RANGE_8G		0x08
#define KXG03_ACC_RANGE_16G		0x0C

// GYRO_RANGE
#define KXG03_GYRO_RANGE_256	0x00
#define KXG03_GYRO_RANGE_512	0x40
#define KXG03_GYRO_RANGE_1024	0x80
#define KXG03_GYRO_RANGE_2048	0xC0

// GYRO_BAND_WIDTH
#define KXG03_GYRO_BW_10HZ		0x00
#define KXG03_GYRO_BW_20HZ		0x10
#define KXG03_GYRO_BW_40HZ		0x20
#define KXG03_GYRO_BW_160HZ		0x30

// MODE PARAMTERES
#define KXG03_MODE_RAW			0x00
#define KXG03_MODE_PR			0x01
#define KXG03_MODE_PS			0x02
#define KXG03_MODE_KALMAN		0x10

typedef struct {
	uint8_t addr;
	uint8_t data;
} KXG03_REG,*pKXG03_REG;

typedef struct {
	
	uint8_t (*init)(uint8_t slave_addr);
	uint8_t (*sync_init)(uint8_t slave_addr,uint8_t odr_rate,void (*func)(void));
	uint8_t (*read)(uint8_t addr, uint8_t* data, uint8_t size);
	uint8_t (*write)(uint8_t addr, uint8_t* data, uint8_t size);
	uint8_t (*get_val)(float* val);
	uint8_t (*get_gyro)(float* gyro);
	uint8_t (*get_acc)(float* acc);
	int (*set_acc_avr)(uint8_t acc_avr);
	int (*set_acc_range)(uint8_t gyro_range);
	int (*set_gyro_range)(uint8_t gyro_range);
	int (*set_gyro_bw)(uint8_t gyro_bw);
	void (*angle_init)(int8_t mode, int8_t gx, int8_t gy, int8_t gz, int8_t ax, int8_t ay, int8_t az);
	int (*get_angle)(float* angle);
	void (*set_kalman_out)(bool kalman_out);
	void (*set_acc_out)(bool acc_out);
	void (*set_gyro_out)(bool gyro_out);
	void (*set_deg_out)(bool deg_out);
	uint8_t (*standby)(void);
}KXG03;

typedef struct {
    float angle;
    float bias;
    float P[2][2];
	float newAngle;
	float newRate;
	float lastTime;
} KALMAN_PARAM;

extern const KXG03 kxg03;

typedef struct {
	void (*init)(KALMAN_PARAM *param);
	void (*set)(KALMAN_PARAM *param,float newAngle);
	float (*cal)(KALMAN_PARAM *param,float newAngle, float newRate);
} KALMAN;

extern const KALMAN kalman;

#endif

