#include "mpu6050.h"
#include "i2c.h"
#include "Time_Out.h"
#include "fusion.h"
#include "jediode_calibration.h"
#include "jediode_blade_swing.h"
#include "jediode_types.h"
#include "stdint.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef RAD2DEG
#define RAD2DEG (57.2957795130823208768f)
#endif
#ifndef DEG2RAD
#define DEG2RAD (1.0f / RAD2DEG)
#endif
#define MPU6050_DMA_FRAME_LEN 14  // accel(6) + gyro(6) + temp(2)

static uint8_t mpu_dma_rawbuf[MPU6050_DMA_FRAME_LEN];

static volatile MPU6050_Frame_t mpu_buf_front;    // latest readable
static volatile MPU6050_Frame_t mpu_buf_back;     // written by DMA callback
static volatile uint8_t mpu_new_data = 0;         // flag for user

//2- Accel & Gyro Scaling Factor
static float accelScalingFactor, gyroScalingFactor;
//3- Bias variables
static float A_X_Bias = 0.0f;
static float A_Y_Bias = 0.0f;
static float A_Z_Bias = 0.0f;

static float G_X_Bias = 0.0f;
static float G_Y_Bias = 0.0f;
static float G_Z_Bias = 0.0f;

static int16_t GyroRW[3];

uint32_t last_timestamp = 0;

static void euler_to_quaternion(float roll_deg, float pitch_deg, float yaw_deg,
                                float *qw, float *qx, float *qy, float *qz);

Attitude_Def att;          // Your orientation EKF/Complementary filter
ScaledData_Def acc;        // raw accel (g)
ScaledData_Def acc_lin;    // gravity-removed accel

MPU6050_Data g_mpu6050;    // <--- Required global (swing uses it)

void MPU6050_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;
	//Clock Source 
	//Reset Device

	i2c1_writeByte(MPU_ADDR, PWR_MAGT_1_REG, 0x80);

	delay(100);

	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register

	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	
	i2c1_writeByte(MPU_ADDR,PWR_MAGT_1_REG, Buffer);

	delay(100); // should wait 10ms after changeing the clock setting.

	//Set the Digital Low Pass Filter
	Buffer = 0;

	Buffer = config->CONFIG_DLPF & 0x07;
	
	i2c1_writeByte(MPU_ADDR,CONFIG_REG, Buffer);

	//Select the Gyroscope Full Scale Range
	Buffer = 0;

	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	
	i2c1_writeByte(MPU_ADDR,GYRO_CONFIG_REG, Buffer);

	//Select the Accelerometer Full Scale Range 
	Buffer = 0; 

	Buffer = (config->Accel_Full_Scale << 3) & 0x18;

	i2c1_writeByte(MPU_ADDR,ACCEL_CONFIG_REG, Buffer);

	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);
	
	
	//Accelerometer Scaling Factor, Set the Accelerometer and Gyroscope Scaling Factor
	switch (config->Accel_Full_Scale)
	{
		case AFS_SEL_2g:
			accelScalingFactor = (2000.0f/32768.0f);
			break;
		
		case AFS_SEL_4g:
			accelScalingFactor = (4000.0f/32768.0f);
				break;
		
		case AFS_SEL_8g:
			accelScalingFactor = (8000.0f/32768.0f);
			break;
		
		case AFS_SEL_16g:
			accelScalingFactor = (16000.0f/32768.0f);
			break;
		
		default:
			break;
	}
	//Gyroscope Scaling Factor 
	switch (config->Gyro_Full_Scale)
	{
		case FS_SEL_250:
			gyroScalingFactor = 250.0f/32768.0f;
			break;
		
		case FS_SEL_500:
				gyroScalingFactor = 500.0f/32768.0f;
				break;
		
		case FS_SEL_1000:
			gyroScalingFactor = 1000.0f/32768.0f;
			break;
		
		case FS_SEL_2000:
			gyroScalingFactor = 2000.0f/32768.0f;
			break;
		
		default:
			break;
	}
	
}

//5- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;
	
	Buffer = i2c1_readByte(MPU_ADDR,SMPLRT_DIV_REG);

	return Buffer;
}

void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
    i2c1_writeByte(MPU_ADDR, SMPLRT_DIV_REG, SMPRTvalue);
}


uint8_t MPU6050_Get_FSYNC(void)
{
	uint8_t Buffer = 0;
	
	Buffer = i2c1_readByte(MPU_ADDR,CONFIG_REG);
	Buffer &= 0x38; 
	return (Buffer>>3);
}

//8- Set External Frame Sync. 
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	Buffer = i2c1_readByte(MPU_ADDR,CONFIG_REG);
	Buffer &= ~0x38;
	
	Buffer |= (ext_Sync <<3); 
	i2c1_writeByte(MPU_ADDR,CONFIG_REG, Buffer);
	
}

//9- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef)
{
	uint8_t state;
	uint8_t AcceArr[6], GyroArr[6];
	
	state = i2c1_readByte(MPU_ADDR,INT_STATUS_REG);


	if((state & 0x01))
	{
		i2c1_readMulti(MPU_ADDR,ACCEL_XOUT_H_REG, 6,AcceArr);
		
		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8) + AcceArr[1]); // x-Axis
		rawDef->y = ((AcceArr[2]<<8) + AcceArr[3]); // y-Axis
		rawDef->z = ((AcceArr[4]<<8) + AcceArr[5]); // z-Axis
		//Gyro Raw Data
		i2c1_readMulti(MPU_ADDR,GYRO_XOUT_H_REG, 6,GyroArr);
		GyroRW[0] = ((GyroArr[0]<<8) + GyroArr[1]);
		GyroRW[1] = (GyroArr[2]<<8) + GyroArr[3];
		GyroRW[2] = ((GyroArr[4]<<8) + GyroArr[5]);
	}
}

//10- Get Accel scaled data (g unit of gravity, 1g = 9.81m/s2)
void MPU6050_Get_Accel_Scale(ScaledData_Def *scaledDef)
{

	RawData_Def AccelRData;
	MPU6050_Get_Accel_RawData(&AccelRData);
	
	//Accel Scale data 
	scaledDef->x = ((AccelRData.x+0.0f)*accelScalingFactor)/1000;
	scaledDef->y = ((AccelRData.y+0.0f)*accelScalingFactor)/1000;
	scaledDef->z = ((AccelRData.z+0.0f)*accelScalingFactor)/1000;
}

//11- Get Accel calibrated data
void MPU6050_Get_Accel_Cali(ScaledData_Def *CaliDef)
{
	ScaledData_Def AccelScaled;
	MPU6050_Get_Accel_Scale(&AccelScaled);
	
	//Accel Scale data 
	CaliDef->x = (AccelScaled.x) - A_X_Bias; // x-Axis
	CaliDef->y = (AccelScaled.y) - A_Y_Bias;// y-Axis
	CaliDef->z = (AccelScaled.z) - A_Z_Bias;// z-Axis
}

//12- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{
	
	//Accel Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = GyroRW[1];
	rawDef->z = GyroRW[2];
	
}

//13- Get Gyro scaled data
void MPU6050_Get_Gyro_Scale(ScaledData_Def *scaledDef)
{
    RawData_Def myGyroRaw;
    MPU6050_Get_Gyro_RawData(&myGyroRaw);

    scaledDef->x = (myGyroRaw.x)*gyroScalingFactor - G_X_Bias;
    scaledDef->y = (myGyroRaw.y)*gyroScalingFactor - G_Y_Bias;
    scaledDef->z = (myGyroRaw.z)*gyroScalingFactor - G_Z_Bias;
}

//14- Accel Calibration
void _Accel_Cali(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
	//1* X-Axis calibrate
	A_X_Bias		= (x_max + x_min)/2.0f;
	
	//2* Y-Axis calibrate
	A_Y_Bias		= (y_max + y_min)/2.0f;
	
	//3* Z-Axis calibrate
	A_Z_Bias		= (z_max + z_min)/2.0f;
}

void Gyro_Cali(void)
{
    ScaledData_Def g;
    const int N = 500;
    float sumx = 0, sumy = 0, sumz = 0;

    for (int i = 0; i < N; i++) {
        ScaledData_Def rawGyro;
        MPU6050_Get_Gyro_Scale(&rawGyro);  // or raw + manual scaling
        sumx += rawGyro.x;
        sumy += rawGyro.y;
        sumz += rawGyro.z;
        delay(2); // ~2 ms between samples
    }

    G_X_Bias = sumx / N;
    G_Y_Bias = sumy / N;
    G_Z_Bias = sumz / N;
}

void MPU6050_DMA_RequestRead(void)
{
    // 0x3B = ACCEL_XOUT_H register (first of 14 bytes)
    i2c1_dma_read(0x68,0x3B, mpu_dma_rawbuf, MPU6050_DMA_FRAME_LEN);
}

void MPU6050_DMA_DoneHandler(void)
{
    // Unpack accel (big-endian)
    mpu_buf_back.ax = (int16_t)((mpu_dma_rawbuf[0] << 8) | mpu_dma_rawbuf[1]);
    mpu_buf_back.ay = (int16_t)((mpu_dma_rawbuf[2] << 8) | mpu_dma_rawbuf[3]);
    mpu_buf_back.az = (int16_t)((mpu_dma_rawbuf[4] << 8) | mpu_dma_rawbuf[5]);

    // Temp
    mpu_buf_back.temp = (int16_t)((mpu_dma_rawbuf[6] << 8) | mpu_dma_rawbuf[7]);

    // Gyro
    mpu_buf_back.gx = (int16_t)((mpu_dma_rawbuf[8]  << 8) | mpu_dma_rawbuf[9]);
    mpu_buf_back.gy = (int16_t)((mpu_dma_rawbuf[10] << 8) | mpu_dma_rawbuf[11]);
    mpu_buf_back.gz = (int16_t)((mpu_dma_rawbuf[12] << 8) | mpu_dma_rawbuf[13]);

    // Store timestamp (from your Ticks_Init clock)
    mpu_buf_back.timestamp = get_Ticks();

    // Swap back-buffer into front-buffer atomically
    mpu_buf_front = mpu_buf_back;

    // Notify that new data available
    mpu_new_data = 1;
}

uint8_t MPU6050_DMA_GetLatest(MPU6050_Frame_t *out)
{
    if (!mpu_new_data)
        return 0; // no new frame yet

    // Copy the atomically-updated front buffer
    *out = mpu_buf_front;

    mpu_new_data = 0; // consume
    return 1;
}

void MPU6050_DMA_Init(void)
{
    i2c1_dma_init();               // from Option A driver
    mpu_new_data = 0;
}

static void euler_to_quaternion(float roll_deg, float pitch_deg, float yaw_deg,
                                float *qw, float *qx, float *qy, float *qz)
{
    float r = roll_deg  * DEG2RAD * 0.5f;
    float p = pitch_deg * DEG2RAD * 0.5f;
    float y = yaw_deg   * DEG2RAD * 0.5f;

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    *qw = cr*cp*cy + sr*sp*sy;
    *qx = sr*cp*cy - cr*sp*sy;
    *qy = cr*sp*cy + sr*cp*sy;
    *qz = cr*cp*sy - sr*sp*cy;
}
