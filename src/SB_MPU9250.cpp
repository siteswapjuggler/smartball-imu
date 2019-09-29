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

#include "Arduino.h"
#include "SB_MPU9250.h"

/*------------------------------------------------------------------------
 * MPU9250 object, input the SPI bus and chip select pin
 *----------------------------------------------------------------------*/

MPU9250::MPU9250(SPIClass &bus, uint8_t csPin) {
  _spi = &bus;      // SPI bus
  _csPin = csPin;   // chip select pin
}

/*------------------------------------------------------------------------
 * starts communication with the MPU-9250
 *----------------------------------------------------------------------*/

void MPU9250::begin() {
  // SPI COMMUNICATION
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);
  _spi->begin();
  _useSPIHS = false;

  // RESET BOTH DEVICES
  writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);      		// select clock source to gyro
  writeRegister(USER_CTRL, I2C_MST_EN);         		// enable I2C master mode
  writeRegister(I2C_MST_CTRL, I2C_MST_CLK);       		// set the I2C bus speed to 400 kHz
  writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);   // set AK8963 to Power Down
  writeRegister(PWR_MGMNT_1, PWR_RESET);        		// reset the MPU9250
  delay(1);                       						// wait for MPU-9250 to come back up
  writeAK8963Register(AK8963_CNTL2, AK8963_RESET);    	// reset the AK8963

  // BASIC MPU9250 CONFIGURATION
  writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL);      		// select clock source to gyro
  writeRegister(PWR_MGMNT_2, SEN_ENABLE);         		// enable accelerometer and gyro
  setSampleRateDivider(0);                        		// setting the sample rate divider to 0 as default

  // BASIC ACCELEROMETER CONFIGURATION
  setAccelRange(ACCEL_RANGE_16G);            			// setting accel range to 16G as default
  setAccelDlpfBandwidth(ACCEL_DLPF_BANDWIDTH_460HZ); 	// setting bandwidth to 460Hz as default
  
  //TODO Read the buffer and calculate bias
	
  // BASIC GYROSCOPE CONFIGURATION
  setGyroRange(GYRO_RANGE_2000DPS);		           		// setting the gyro range to 2000DPS as default
  setGyroDlpfBandwidth(GYRO_DLPF_BANDWIDTH_250HZ);      // setting bandwidth to 250Hz as default
  readRegisters(GYRO_OFFSETS,6,_buffer);				// read factory settings offsets
  
  _gxb = ((float)(_buffer[0]<<8|_buffer[1]) * 4.f * 500.) / ((float)(1<<_gyroRange) * 65536.f);
  _gyb = ((float)(_buffer[2]<<8|_buffer[3]) * 4.f * 500.) / ((float)(1<<_gyroRange) * 65536.f);
  _gzb = ((float)(_buffer[4]<<8|_buffer[5]) * 4.f * 500.) / ((float)(1<<_gyroRange) * 65536.f);
  
  // BASIC AK8963 CONFIGURATION
  writeRegister(USER_CTRL, I2C_MST_EN);         		// enable I2C master mode
  writeRegister(I2C_MST_CTRL, I2C_MST_CLK);       		// set the I2C bus speed to 400 kHz

  // MAGNETOMETER FACTORY CALIBRATION
  writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);   // set AK8963 to Power Down
  delay(100);                       					// long wait between AK8963 mode changes
  writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM);   // set AK8963 to FUSE ROM access
  delay(100);                       					// long wait between AK8963 mode changes
  readAK8963Registers(AK8963_ASA, 3, _buffer);      	// read the AK8963 ASA registers and compute magnetometer scale factors

  _magScaleX = (((float)_buffer[0] - 128.0f)/256.0f + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleY = (((float)_buffer[1] - 128.0f)/256.0f + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleZ = (((float)_buffer[2] - 128.0f)/256.0f + 1.0f) * 4912.0f / 32760.0f; // micro Tesla

  writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);   // set AK8963 to Power Down
  delay(100);                       					// long wait between AK8963 mode changes
  writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);  // set AK8963 to 16 bit resolution, 100 Hz update rate
  delay(100);                       					// long wait between AK8963 mode changes
  readAK8963Registers(AK8963_HXL, 7, _buffer);      	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate

  resetQuaternion();
}

/*------------------------------------------------------------------------
 * reset Orientation Quaternion
 *----------------------------------------------------------------------*/

void MPU9250::resetQuaternion() {
	_q1 = 1.0;
	_q2 = 0.0;
	_q3 = 0.0;
	_q4 = 0.0;
}

/*------------------------------------------------------------------------
 * sets the accelerometer full scale range to values other than default
 *----------------------------------------------------------------------*/

void MPU9250::setAccelRange(byte range) {
  setAccelRange((AccelRange)range);
}

void MPU9250::setAccelRange(AccelRange range) {
  range = (AccelRange)((byte)range & 3);
  _useSPIHS = false;
  writeRegister(ACCEL_CONFIG, range << 3);
  _accelScale = 4.f * GRAVITY * double(1<<range) / 65536.f;
  _accelRange = range;
}

/*-------------------------------------------------------------------
 * sets the gyro full scale range to values other than default
 *-----------------------------------------------------------------*/

void MPU9250::setGyroRange(byte range) {
  setGyroRange((GyroRange)range);
}

void MPU9250::setGyroRange(GyroRange range) {
  range = (GyroRange)((byte)range & 3);
  _useSPIHS = false;
  writeRegister(GYRO_CONFIG, range << 3);
  _gyroScale = 500.f * D2R * double(1<<range) / 65536.f;
  _gyroRange = range;
}

/*-------------------------------------------------------------------
 * sets the DLPF bandwidth to values other than default
 *-----------------------------------------------------------------*/

 void MPU9250::setAccelDlpfBandwidth(byte bandwidth) {
  setAccelDlpfBandwidth((AccelDlpfBandwidth)bandwidth);
}

void MPU9250::setAccelDlpfBandwidth(AccelDlpfBandwidth bandwidth) {
  _useSPIHS = false;
   writeRegister(ACCEL_CONFIG2, (byte)bandwidth);
  _accelBandwidth = bandwidth;
}

void MPU9250::setGyroDlpfBandwidth(byte bandwidth) {
  setGyroDlpfBandwidth((GyroDlpfBandwidth)bandwidth);
}

void MPU9250::setGyroDlpfBandwidth(GyroDlpfBandwidth bandwidth) {
  _useSPIHS = false;
   writeRegister(CONFIG, (byte)bandwidth);
  _gyroBandwidth = bandwidth;
}

/*-------------------------------------------------------------------
 * sets the sample rate divider to values other than default
 *-----------------------------------------------------------------*/

void MPU9250::setSampleRateDivider(uint8_t srd) {
  _useSPIHS = false;
  writeRegister(SMPDIV, 19);                				// setting the sample rate divider to 19 to facilitate setting up magnetometer
  if (srd > 9) {
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);   	// set AK8963 to Power Down
    delay(100);                       						// long wait between AK8963 mode changes
    writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS1);  	// set AK8963 to 16 bit resolution, 8 Hz update rate
    delay(100);                       						// long wait between AK8963 mode changes
    readAK8963Registers(AK8963_HXL, 7, _buffer);      		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  }
  else {
    writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN);   	// set AK8963 to Power Down
    delay(100);                       						// long wait between AK8963 mode changes
    writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2);  	// set AK8963 to 16 bit resolution, 100 Hz update rate
    delay(100);                       						// long wait between AK8963 mode changes
    readAK8963Registers(AK8963_HXL, 7, _buffer);        	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  }
  writeRegister(SMPDIV, srd);               				// setting the sample rate divider
  _samplerateDivider = srd;
}


/*-------------------------------------------------------------------
 * reads the most current data from MPU9250 and stores in buffer
 *-----------------------------------------------------------------*/

void MPU9250::readSensor() {
  _now          = micros();
  _deltaTime    = float(_now - _previousTime)/1000000.0f;
  _previousTime = _now;

  _useSPIHS = true;             			// use the high speed SPI for data readout
  readRegisters(ACCEL_OUT, 20, _buffer);  	// grab the data from the MPU9250

  // combine into 16 bit values
  int16_t _axcounts = _buffer[0]  << 8 | _buffer[1];
  int16_t _aycounts = _buffer[2]  << 8 | _buffer[3];
  int16_t _azcounts = _buffer[4]  << 8 | _buffer[5];
  int16_t _tcounts  = _buffer[6]  << 8 | _buffer[7];
  int16_t _gxcounts = _buffer[8]  << 8 | _buffer[9];
  int16_t _gycounts = _buffer[10] << 8 | _buffer[11];
  int16_t _gzcounts = _buffer[12] << 8 | _buffer[13];
  int16_t _mxcounts = _buffer[15] << 8 | _buffer[14];
  int16_t _mycounts = _buffer[17] << 8 | _buffer[16];
  int16_t _mzcounts = _buffer[19] << 8 | _buffer[18];

  // transform and convert to float values
  _ax = (((float)(tX[0] * _axcounts + tX[1] * _aycounts + tX[2] * _azcounts) * _accelScale) - _axb) * _axs;
  _ay = (((float)(tY[0] * _axcounts + tY[1] * _aycounts + tY[2] * _azcounts) * _accelScale) - _ayb) * _ays;
  _az = (((float)(tZ[0] * _axcounts + tZ[1] * _aycounts + tZ[2] * _azcounts) * _accelScale) - _azb) * _azs;
  _an = sqrt(_ax*_ax+_ay*_ay+_az*_az);
  
  _gx = ((float)(tX[0] * _gxcounts + tX[1] * _gycounts + tX[2] * _gzcounts) * _gyroScale) - _gxb;
  _gy = ((float)(tY[0] * _gxcounts + tY[1] * _gycounts + tY[2] * _gzcounts) * _gyroScale) - _gyb;
  _gz = ((float)(tZ[0] * _gxcounts + tZ[1] * _gycounts + tZ[2] * _gzcounts) * _gyroScale) - _gzb;
  _gn = sqrt(_gx*_gx+_gy*_gy+_gz*_gz);
  
  _mx = (((float)(_mxcounts) * _magScaleX) - _mxb) * _mxs;
  _my = (((float)(_mycounts) * _magScaleY) - _myb) * _mys;
  _mz = (((float)(_mzcounts) * _magScaleZ) - _mzb) * _mzs;
  _mn = sqrt(_mx*_mx+_my*_my+_mz*_mz);
  
  _t =  (((float)(_tcounts) - TEMP_OFFSET) / TEMP_SCALE) + TEMP_OFFSET;
}

/*-------------------------------------------------------------------
 * Update Madgwick Filter
 *-----------------------------------------------------------------*/

void MPU9250::madgwickUpdate() {
  float norm, ax = _ax, ay = _ay, az = _az, gx = _gx, gy = _gy, gz = _gz, mx = _mx, my = _my, mz = _mz;
  float hx, hy, _2bx, _2bz;
  float s1, s2, s3, s4;
  float qDot1, qDot2, qDot3, qDot4;

  // Auxiliary variables to avoid repeated arithmetic
  float _2q1mx;
  float _2q1my;
  float _2q1mz;
  float _2q2mx;
  float _4bx;
  float _4bz;
  float _2q1 = 2.0f * _q1;
  float _2q2 = 2.0f * _q2;
  float _2q3 = 2.0f * _q3;
  float _2q4 = 2.0f * _q4;
  float _2q1q3 = 2.0f * _q1 * _q3;
  float _2q3q4 = 2.0f * _q3 * _q4;
  float q1q1 = _q1 * _q1;
  float q1q2 = _q1 * _q2;
  float q1q3 = _q1 * _q3;
  float q1q4 = _q1 * _q4;
  float q2q2 = _q2 * _q2;
  float q2q3 = _q2 * _q3;
  float q2q4 = _q2 * _q4;
  float q3q3 = _q3 * _q3;
  float q3q4 = _q3 * _q4;
  float q4q4 = _q4 * _q4;

  // Normalise accelerometer measurement
  norm = _an;
  if (isnan(norm)) return; // handle NaN
  norm = 1.0f / norm;
  ax *= norm;
  ay *= norm;
  az *= norm;

  // Normalise magnetometer measurement
  norm = _mn;
  if (isnan(norm)) return; // handle NaN
  norm = 1.0f / norm;
  mx *= norm;
  my *= norm;
  mz *= norm;

  // Reference direction of Earth's magnetic field
  _2q1mx = 2.0f * _q1 * mx;
  _2q1my = 2.0f * _q1 * my;
  _2q1mz = 2.0f * _q1 * mz;
  _2q2mx = 2.0f * _q2 * mx;
  hx = mx * q1q1 - _2q1my * _q4 + _2q1mz * _q3 + mx * q2q2 + _2q2 * my * _q3 + _2q2 * mz * _q4 - mx * q3q3 - mx * q4q4;
  hy = _2q1mx * _q4 + my * q1q1 - _2q1mz * _q2 + _2q2mx * _q3 - my * q2q2 + my * q3q3 + _2q3 * mz * _q4 - my * q4q4;
  _2bx = sqrt(hx * hx + hy * hy);
  _2bz = -_2q1mx * _q3 + _2q1my * _q2 + mz * q1q1 + _2q2mx * _q4 - mz * q2q2 + _2q3 * my * _q4 - mz * q3q3 + mz * q4q4;
  _4bx = 2.0f * _2bx;
  _4bz = 2.0f * _2bz;

  // Gradient decent algorithm corrective step
  s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * _q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * _q4 + _2bz * _q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * _q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * _q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * _q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * _q3 + _2bz * _q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * _q4 - _4bz * _q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * _q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * _q3 - _2bz * _q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * _q2 + _2bz * _q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * _q1 - _4bz * _q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * _q4 + _2bz * _q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * _q1 + _2bz * _q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * _q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
  norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
  norm = 1.0f / norm;
  s1 *= norm;
  s2 *= norm;
  s3 *= norm;
  s4 *= norm;

  // Compute rate of change of quaternion
  qDot1 = 0.5f * (-_q2 * gx - _q3 * gy - _q4 * gz) - beta * s1;
  qDot2 = 0.5f * (_q1 * gx + _q3 * gz - _q4 * gy) - beta * s2;
  qDot3 = 0.5f * (_q1 * gy - _q2 * gz + _q4 * gx) - beta * s3;
  qDot4 = 0.5f * (_q1 * gz + _q2 * gy - _q3 * gx) - beta * s4;

  // Integrate to yield quaternion
  _q1 += qDot1 * _deltaTime;
  _q2 += qDot2 * _deltaTime;
  _q3 += qDot3 * _deltaTime;
  _q4 += qDot4 * _deltaTime;
  norm = sqrt(_q1 * _q1 + _q2 * _q2 + _q3 * _q3 + _q4 * _q4);    // normalise quaternion
  norm = 1.0f / norm;
  _q1 = _q1 * norm;
  _q2 = _q2 * norm;
  _q3 = _q3 * norm;
  _q4 = _q4 * norm;
}

/*-------------------------------------------------------------------
 * Calculate World Accel
 *-----------------------------------------------------------------*/

void MPU9250::realWorldUpdate() {
	Quaternion q(_q1,_q2,_q3,_q4);
	Quaternion p(0,_ax,_ay,_az);
	p = q.getProduct(p);
	p = p.getProduct(q.getConjugate());
	_wx = p.x;
	_wy = p.y;
	_wz = p.z;
}
 
/*-------------------------------------------------------------------
 * GET VALUES
 *-----------------------------------------------------------------*/

float MPU9250::getAccelX_mss()    { return _ax; }
float MPU9250::getAccelY_mss()    { return _ay; }
float MPU9250::getAccelZ_mss()    { return _az; }
float MPU9250::getAccelN_mss()    { return _an; }
float MPU9250::getGyroX_rads()    { return _gx; }
float MPU9250::getGyroY_rads()    { return _gy; }
float MPU9250::getGyroZ_rads()    { return _gz; }
float MPU9250::getGyroN_rads()    { return _gn; }
float MPU9250::getMagX_uT()       { return _mx; }
float MPU9250::getMagY_uT()       { return _my; }
float MPU9250::getMagZ_uT()       { return _mz; }
float MPU9250::getMagN_uT()       { return _mn; }
float MPU9250::getQuatW()         { return _q1; }
float MPU9250::getQuatX()         { return _q2; }
float MPU9250::getQuatY()         { return _q3; }
float MPU9250::getQuatZ()         { return _q4; }
float MPU9250::getWorldX_mss()    { return _wx; }
float MPU9250::getWorldY_mss()    { return _wy; }
float MPU9250::getWorldZ_mss()    { return _wz; }
float MPU9250::getTemperature_C() { return _t;  }

/*------------------------------------------------------------------------------------------------------------------
 * MPU9250 SPI COMMUNICATION
 *----------------------------------------------------------------------------------------------------------------*/

void MPU9250::writeRegister(uint8_t subAddress, uint8_t data) {
  _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE0));   // begin the transaction
  digitalWrite(_csPin, LOW);                          						// select the MPU9250 chip
  _spi->transfer(subAddress);                         						// write the register address
  _spi->transfer(data);                             						// write the data
  digitalWrite(_csPin, HIGH);                         						// deselect the MPU9250 chip
  _spi->endTransaction();                           						// end the transaction
}

void MPU9250::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
 int clock = _useSPIHS ? SPI_HS_CLOCK : SPI_LS_CLOCK;
 _spi->beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);                          						// select the MPU9250 chip
  _spi->transfer(subAddress | SPI_READ);                    				// specify the starting register address
  for (uint8_t i = 0; i < count; i++) {
    dest[i] = _spi->transfer(0x00);      					                // read the data
  }
  digitalWrite(_csPin, HIGH);                         						// deselect the MPU9250 chip
  _spi->endTransaction();                           						// end the transaction
}

/*------------------------------------------------------------------------------------------------------------------
 * AK8963 I2C COMMUNICATION
 *----------------------------------------------------------------------------------------------------------------*/

void MPU9250::writeAK8963Register(uint8_t subAddress, uint8_t data) {
  writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR);
  writeRegister(I2C_SLV0_REG, subAddress);
  writeRegister(I2C_SLV0_DO, data);
  writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | 1);
}

void MPU9250::readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG);
  writeRegister(I2C_SLV0_REG, subAddress);
  writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count);
  delay(1);
  readRegisters(EXT_SENS_DATA_00, count, dest);
}

/*------------------------------------------------------------------------------------------------------------------
 * WHO AM I
 *----------------------------------------------------------------------------------------------------------------*/

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int MPU9250::whoAmI() {
  readRegisters(WHO_AM_I, 1, _buffer);
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int MPU9250::whoAmIAK8963() {
  readAK8963Registers(AK8963_WHO_AM_I, 1, _buffer);
  return _buffer[0];
}

/*******************************************************************************************************************
 * CALIBRATIONS
 ******************************************************************************************************************/

/*------------------------------------------------------------------------------------------------------------------
 * ACCELEROMETER CALIBRATION
 * finds bias and scale factor calibration for the accelerometer,
 * this should be run for each axis in each direction (6 total) to find
 * the min and max values along each
 *----------------------------------------------------------------------------------------------------------------*/

void MPU9250::calibrateAccel() {
  setAccelRange(ACCEL_RANGE_2G);
  setAccelDlpfBandwidth(ACCEL_DLPF_BANDWIDTH_184HZ);
  setSampleRateDivider(19);
  
  // temp variable
  size_t _numSamples = 100;
  float _axmax, _aymax, _azmax;
  float _axmin, _aymin, _azmin;
  
  // take samples and find min / max
  double _axbD = 0;
  double _aybD = 0;
  double _azbD = 0;
  
  for (size_t i = 0; i < _numSamples; i++) {
    readSensor();
    _axbD += (getAccelX_mss() / _axs + _axb) / ((double)_numSamples);
    _aybD += (getAccelY_mss() / _ays + _ayb) / ((double)_numSamples);
    _azbD += (getAccelZ_mss() / _azs + _azb) / ((double)_numSamples);
    delay(20);
  }
  
  if (_axbD > 9.0f) _axmax = (float)_axbD;
  if (_aybD > 9.0f) _aymax = (float)_aybD;
  if (_azbD > 9.0f) _azmax = (float)_azbD;
  if (_axbD < -9.0f) _axmin = (float)_axbD;
  if (_aybD < -9.0f) _aymin = (float)_aybD;
  if (_azbD < -9.0f) _azmin = (float)_azbD;

  // find bias and scale factor
  if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f)) {
    _axb = (_axmin + _axmax) / 2.0f;
    _axs = GRAVITY / ((abs(_axmin) + abs(_axmax)) / 2.0f);
  }
  if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f)) {
    _ayb = (_axmin + _axmax) / 2.0f;
    _ays = GRAVITY / ((abs(_aymin) + abs(_aymax)) / 2.0f);
  }
  if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f)) {
    _azb = (_azmin + _azmax) / 2.0f;
    _azs = GRAVITY / ((abs(_azmin) + abs(_azmax)) / 2.0f);
  }

  // set the range, bandwidth, and srd back to what they were
  setAccelRange(_accelRange);
  setAccelDlpfBandwidth(_accelBandwidth);
  setSampleRateDivider(_samplerateDivider);
}

float MPU9250::getAccelBiasX_mss() { return _axb;}
float MPU9250::getAccelScaleFactorX() { return _axs;}
float MPU9250::getAccelBiasY_mss() { return _ayb;}
float MPU9250::getAccelScaleFactorY() { return _ays;}
float MPU9250::getAccelBiasZ_mss() { return _azb;}
float MPU9250::getAccelScaleFactorZ() { return _azs;}

void MPU9250::setAccelCalX(float bias, float scaleFactor) {
  _axb = bias;
  _axs = scaleFactor;
}

void MPU9250::setAccelCalY(float bias, float scaleFactor) {
  _ayb = bias;
  _ays = scaleFactor;
}

void MPU9250::setAccelCalZ(float bias, float scaleFactor) {
  _azb = bias;
  _azs = scaleFactor;
}

/*------------------------------------------------------------------------------------------------------------------
 * GYROSCOPE CALBRATION
 *
 * estimates the gyro biases
 *----------------------------------------------------------------------------------------------------------------*/

void MPU9250::calibrateGyro() {
  // set the range, bandwidth, and srd
  setGyroRange(GYRO_RANGE_250DPS);
  setGyroDlpfBandwidth(GYRO_DLPF_BANDWIDTH_20HZ);
  setSampleRateDivider(19);

  // take samples and find bias
  double _gxbD = 0;
  double _gybD = 0;
  double _gzbD = 0;

  size_t _numSamples = 100;
  
  for (size_t i = 0; i < _numSamples; i++) {
    readSensor();
    _gxbD += (getGyroX_rads() + _gxb) / ((double)_numSamples);
    _gybD += (getGyroY_rads() + _gyb) / ((double)_numSamples);
    _gzbD += (getGyroZ_rads() + _gzb) / ((double)_numSamples);
    delay(20);
  }
  _gxb = (float)_gxbD;
  _gyb = (float)_gybD;
  _gzb = (float)_gzbD;

  // set the range, bandwidth, and srd back to what they were
  setGyroRange(_gyroRange);
  setGyroDlpfBandwidth(_gyroBandwidth);
  setSampleRateDivider(_samplerateDivider);
}

float MPU9250::getGyroBiasX_rads() { return _gxb;}
float MPU9250::getGyroBiasY_rads() { return _gyb;}
float MPU9250::getGyroBiasZ_rads() { return _gzb;}

void MPU9250::setGyroBiasX_rads(float bias) { _gxb = bias;}
void MPU9250::setGyroBiasY_rads(float bias) { _gyb = bias;}
void MPU9250::setGyroBiasZ_rads(float bias) { _gzb = bias;}


/*----------------------------------------------------------------------------------------------------------------*/
/* MAG CALIBRATION
/* the sensor should be rotated in a figure 8 motion until complete
/*----------------------------------------------------------------------------------------------------------------*/

void MPU9250::calibrateMag() {
  // set the srd
  setSampleRateDivider(19);

  // local variables
  uint16_t _maxCounts = 1000;
  float _deltaThresh = 0.3f;
  uint8_t _coeff = 8;
  uint16_t _counter = 0;
  float _framedelta, _delta;
  float _mxfilt, _myfilt, _mzfilt;
  float _mxmax, _mymax, _mzmax;
  
  // get a starting set of data
  readSensor();
  _mxmax = getMagX_uT();
  _mxmin = getMagX_uT();
  _mymax = getMagY_uT();
  _mymin = getMagY_uT();
  _mzmax = getMagZ_uT();
  _mzmin = getMagZ_uT();

  // collect data to find max / min in each channel
    
  while (_counter < _maxCounts) {
    _delta = 0.0f;
    _framedelta = 0.0f;
    readSensor();
    _mxfilt = (_mxfilt * ((float)_coeff - 1) + (getMagX_uT() / _mxs + _mxb)) / ((float)_coeff);
    _myfilt = (_myfilt * ((float)_coeff - 1) + (getMagY_uT() / _mys + _myb)) / ((float)_coeff);
    _mzfilt = (_mzfilt * ((float)_coeff - 1) + (getMagZ_uT() / _mzs + _mzb)) / ((float)_coeff);
    
	if (_mxfilt > _mxmax) {
      _delta = _mxfilt - _mxmax;
      _mxmax = _mxfilt;
    }
    if (_delta > _framedelta) _framedelta = _delta;
	
    if (_myfilt > _mymax) {
      _delta = _myfilt - _mymax;
      _mymax = _myfilt;
    }
    if (_delta > _framedelta) _framedelta = _delta;
    
	if (_mzfilt > _mzmax) {
      _delta = _mzfilt - _mzmax;
      _mzmax = _mzfilt;
    }
    if (_delta > _framedelta) _framedelta = _delta;
    
	if (_mxfilt < _mxmin) {
      _delta = abs(_mxfilt - _mxmin);
      _mxmin = _mxfilt;
    }
    if (_delta > _framedelta) _framedelta = _delta;
    
	if (_myfilt < _mymin) {
      _delta = abs(_myfilt - _mymin);
      _mymin = _myfilt;
    }
    if (_delta > _framedelta) _framedelta = _delta;
	
    if (_mzfilt < _mzmin) {
      _delta = abs(_mzfilt - _mzmin);
      _mzmin = _mzfilt;
    }
    if (_delta > _framedelta) _framedelta = _delta;
    
	if (_framedelta > _deltaThresh) {
      _counter = 0;
    } else {
      _counter++;
    }
    delay(20);
  }

  // find the magnetometer bias
  _mxb = (_mxmax + _mxmin) / 2.0f;
  _myb = (_mymax + _mymin) / 2.0f;
  _mzb = (_mzmax + _mzmin) / 2.0f;

  // find the magnetometer scale factor
  _mxs = (_mxmax - _mxmin) / 2.0f;
  _mys = (_mymax - _mymin) / 2.0f;
  _mzs = (_mzmax - _mzmin) / 2.0f;
  float _avgs = (_mxs + _mys + _mzs) / 3.0f;
  _mxs = _avgs / _mxs;
  _mys = _avgs / _mys;
  _mzs = _avgs / _mzs;

  // set the srd back to what it was
  setSampleRateDivider(_samplerateDivider);
}

float MPU9250::getMagBiasX_uT() 	{ return _mxb;}
float MPU9250::getMagScaleFactorX() { return _mxs;}
float MPU9250::getMagBiasY_uT() 	{ return _myb;}
float MPU9250::getMagScaleFactorY() { return _mys;}
float MPU9250::getMagBiasZ_uT() 	{ return _mzb;}
float MPU9250::getMagScaleFactorZ() { return _mzs;}

void MPU9250::setMagCalX(float bias, float scaleFactor) {
  _mxb = bias;
  _mxs = scaleFactor;
}

void MPU9250::setMagCalY(float bias, float scaleFactor) {
  _myb = bias;
  _mys = scaleFactor;
}

void MPU9250::setMagCalZ(float bias, float scaleFactor) {
  _mzb = bias;
  _mzs = scaleFactor;
}
