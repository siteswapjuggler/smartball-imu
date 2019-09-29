/*
  MPU9250.h rewrite and upgrades
  16/09/2109 by Sylvain Garnavault <garnav@wanadoo.fr>

  Copyright (c) 2018 Sylvain GARNAVAULT

  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
  and associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute,
  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or
  substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
  DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SB_MPU9250_h
#define SB_MPU9250_h

#include "Arduino.h"
#include "helper_3dmath.h"		// Quaternion, vectors and rotation
#include "SPI.h"     			// SPI library

// MPU9250 registers

#define TEMP_OUT      	  0x41
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D
#define ACCEL_OFFSETS	  0x77
#define ACCEL_OUT         0x3B
#define GYRO_CONFIG       0x1B
#define GYRO_OFFSETS	  0x13
#define GYRO_OUT      	  0x43
#define CONFIG            0x1A
#define SMPDIV            0x19
#define PWR_MGMNT_1       0x6B
#define PWR_CYCLE         0x20
#define PWR_RESET         0x80
#define CLOCK_SEL_PLL     0x01
#define PWR_MGMNT_2       0x6C
#define SEN_ENABLE        0x00
#define DIS_GYRO          0x07
#define USER_CTRL         0x6A
#define I2C_MST_EN        0x20
#define I2C_MST_CLK       0x0D
#define I2C_MST_CTRL      0x24
#define I2C_SLV0_ADDR     0x25
#define I2C_SLV0_REG      0x26
#define I2C_SLV0_DO       0x63
#define I2C_SLV0_CTRL     0x27
#define I2C_SLV0_EN       0x80
#define I2C_READ_FLAG     0x80
#define EXT_SENS_DATA_00  0x49
#define WHO_AM_I          0x75

// AK8963 registers
#define AK8963_I2C_ADDR   0x0C
#define AK8963_HXL        0x03
#define AK8963_CNTL1      0x0A
#define AK8963_PWR_DOWN   0x00
#define AK8963_CNT_MEAS1  0x12
#define AK8963_CNT_MEAS2  0x16
#define AK8963_FUSE_ROM   0x0F
#define AK8963_CNTL2      0x0B
#define AK8963_RESET      0x01
#define AK8963_ASA        0x10
#define AK8963_WHO_AM_I   0x00

// SPI VALUES
#define SPI_READ      	  0x80
#define SPI_LS_CLOCK      1000000    // 1 MHz
#define SPI_HS_CLOCK      15000000   // 15 MHz

// TEMP SCALES
#define TEMP_SCALE 		  333.87f
#define TEMP_OFFSET 	  21.f

// CONSTANTS
#define GRAVITY 		  9.80665f
#define D2R 			  0.01745329251f

// MADGWICK FILTER PARAMETERS

#define GyroMeasError PI * (40.0f / 180.0f)      // gyroscope measurement error in rads/s (shown as 3 deg/s)
#define GyroMeasDrift PI * (0.0f / 180.0f)       // gyroscope measurement drift in rad/s/s (shown as 0.0 deg/s/s)

#define beta sqrt(3.0f / 4.0f) * GyroMeasError   // compute beta
#define zeta sqrt(3.0f / 4.0f) * GyroMeasDrift   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f                           // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

enum GyroRange
{
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
};
enum AccelRange
{
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G
};
enum AccelDlpfBandwidth
{
  ACCEL_DLPF_BANDWIDTH_460HZ = 0x00,    // delay 01.94 ms
  ACCEL_DLPF_BANDWIDTH_184HZ = 0x01,	// delay 05.80 ms
  ACCEL_DLPF_BANDWIDTH_92HZ  = 0x02,	// delay 07.80 ms
  ACCEL_DLPF_BANDWIDTH_41HZ  = 0x03,	// delay 11.80 ms
  ACCEL_DLPF_BANDWIDTH_20HZ  = 0x04,	// delay 19.80 ms
  ACCEL_DLPF_BANDWIDTH_10HZ  = 0x05,	// delay 35.70 ms
  ACCEL_DLPF_BANDWIDTH_5HZ   = 0x06,  	// delay 66.96 ms
//  ACCEL_DLPF_BANDWIDTH_460HZ = 0x07		// delay 01.94 ms
};

enum GyroDlpfBandwidth
{
  GYRO_DLPF_BANDWIDTH_250HZ,	 		// delay 00.97 ms
  GYRO_DLPF_BANDWIDTH_184HZ,	 		// delay 02.90 ms
  GYRO_DLPF_BANDWIDTH_92HZ,	 			// delay 03.90 ms
  GYRO_DLPF_BANDWIDTH_41HZ,	 			// delay 05.90 ms
  GYRO_DLPF_BANDWIDTH_20HZ,	 			// delay 09.90 ms	
  GYRO_DLPF_BANDWIDTH_10HZ,   			// delay 17.85 ms
  GYRO_DLPF_BANDWIDTH_5HZ,    			// delay 33.48 ms
  GYRO_DLPF_BANDWIDTH_3600HZ  			// delay 00.17 ms
};

class MPU9250 {
  public:
	MPU9250(SPIClass &bus,uint8_t csPin);
    void begin();

    void setAccelRange(byte range);
    void setAccelRange(AccelRange range);
    void setGyroRange(byte range);
    void setGyroRange(GyroRange range);
    void setAccelDlpfBandwidth(byte bandwidth);
    void setAccelDlpfBandwidth(AccelDlpfBandwidth bandwidth);
    void setGyroDlpfBandwidth(byte bandwidth);
    void setGyroDlpfBandwidth(GyroDlpfBandwidth bandwidth);
    void setSampleRateDivider(uint8_t samplerate);

    void readSensor();
	
	void madgwickUpdate();
	void realWorldUpdate();
	void resetQuaternion();
	
	float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();
    float getAccelN_mss();

    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();
    float getGyroN_rads();
    
	float getMagX_uT();
    float getMagY_uT();
    float getMagZ_uT();
    float getMagN_uT();
	
	float getQuatW();
	float getQuatX();
	float getQuatY();
	float getQuatZ();

	float getWorldX_mss();
    float getWorldY_mss();
    float getWorldZ_mss();
    
	float getTemperature_C();

    void  calibrateGyro();
    float getGyroBiasX_rads();
    float getGyroBiasY_rads();
    float getGyroBiasZ_rads();
    void  setGyroBiasX_rads(float bias);
    void  setGyroBiasY_rads(float bias);
    void  setGyroBiasZ_rads(float bias);

    void  calibrateAccel();
    float getAccelBiasX_mss();
    float getAccelScaleFactorX();
    float getAccelBiasY_mss();
    float getAccelScaleFactorY();
    float getAccelBiasZ_mss();
    float getAccelScaleFactorZ();
    void  setAccelCalX(float bias, float scaleFactor);
    void  setAccelCalY(float bias, float scaleFactor);
    void  setAccelCalZ(float bias, float scaleFactor);

    void  calibrateMag();
    float getMagBiasX_uT();
    float getMagScaleFactorX();
    float getMagBiasY_uT();
    float getMagScaleFactorY();
    float getMagBiasZ_uT();
    float getMagScaleFactorZ();
    void  setMagCalX(float bias, float scaleFactor);
    void  setMagCalY(float bias, float scaleFactor);
    void  setMagCalZ(float bias, float scaleFactor);

  private:
    // SPI
    SPIClass *_spi;
    uint8_t _csPin;
    bool _useSPIHS;
	
    // buffer for reading from sensor
    uint8_t _buffer[21];

	// config
	AccelRange         _accelRange;
	AccelDlpfBandwidth _accelBandwidth;
	GyroRange  	  	   _gyroRange;
	GyroDlpfBandwidth  _gyroBandwidth;
	byte          	   _samplerateDivider;
	
    // data buffer
    float _ax, _ay, _az, _an;
    float _gx, _gy, _gz, _gn;
    float _mx, _my, _mz, _mn;
	float _wx, _wy, _wz;
	float _q1, _q2, _q3, _q4;
    float _t;

	// time management
	float _deltaTime;
	long  _now, _previousTime;
	
    // scale factors
    float _accelScale;
    float _gyroScale;
    float _magScaleX, _magScaleY, _magScaleZ;

    // gyro bias estimation
    float _gxb, _gyb, _gzb;

    // accel bias and scale factor estimation
    float _axb, _ayb, _azb;
    float _axs = 1.0f;
    float _ays = 1.0f;
    float _azs = 1.0f;

    // magnetometer bias and scale factor estimation
    float _mxmin, _mymin, _mzmin;
    float _mxb, _myb, _mzb;
    float _mxs = 1.0f;
    float _mys = 1.0f;
    float _mzs = 1.0f;

    // transformation matrix
    // transform the accel and gyro axes to match the magnetometer axes
    const int16_t tX[3] = {0,  1,  0};
    const int16_t tY[3] = {1,  0,  0};
    const int16_t tZ[3] = {0,  0, -1};

    // private functions
    int whoAmI();
    int whoAmIAK8963();    
	void writeRegister(uint8_t subAddress, uint8_t data);
    void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    void writeAK8963Register(uint8_t subAddress, uint8_t data);
    void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
};
#endif
