
/*
 * Built on
 * https://github.com/kriswiner/MPU9250/blob/master/STM32F401/main.cpp
 * 
 * Connection 
 *  VDD  --- 3.3V 
 *  VDDI --- 3.3V 
 *  SDA  --- A4 
 *  SCL  --- A5 
 *  GND  --- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"
#include "wire.h"
#include <SoftwareSerial.h>

#define SerialDebug true
#define SAMPLES 7

float    ax[SAMPLES], ay[SAMPLES], az[SAMPLES], gx[SAMPLES], gy[SAMPLES], gz[SAMPLES];
int      ctr;
float    axSum, aySum, azSum, gxSum, gySum, gzSum;
uint32_t sumCount = 0;
float    sum = 0.0f;

// Accelero Gyro Magneto
MPU9250        AGM;
SoftwareSerial BTSerial(4, 5);



void setup()
{
  Wire.begin();
  Serial.begin(19200);
  BTSerial.begin(19200);

  BTSerial.write("AT+DEFAULT\r\n");
  BTSerial.write("AT+RESET\r\n");
  BTSerial.write("AT+NAME=Controller\r\n");
  BTSerial.write("AT+ROLE1\r\n");
  BTSerial.write("AT+TYPE1"); //Simple pairing

  // Read WHO_AM_I register
  byte c = AGM.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x73) // WHO_AM_I should always be 0x68
  {
    // Performing self test
    AGM.MPU9250SelfTest(AGM.SelfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    AGM.calibrateMPU9250(AGM.gyroBias, AGM.accelBias);

    AGM.initMPU9250();

    // Read WHO_AM_I register of the magnetometer
    byte d = AGM.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    // Get magnetometer calibration from AK8963 ROM
    AGM.initAK8963(AGM.magCalibration);

    if (SerialDebug)
    {
      //  Gyro and Accelometer Bias, Magnetometer calibration values
    }
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    BTSerial.write("AT+NAME=CricBatERROR\r\n");
    while(1) ; // Loop forever or reset?
  }
}



void loop()
{
  // Interrupt Pin goes high - Data ready; when all data registers have new data
  if (AGM.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    AGM.readAccelData(AGM.accelCount);  // Read the x/y/z adc values
    AGM.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    AGM.ax = (float)AGM.accelCount[0]*AGM.aRes; // - accelBias[0];
    AGM.ay = (float)AGM.accelCount[1]*AGM.aRes; // - accelBias[1];
    AGM.az = (float)AGM.accelCount[2]*AGM.aRes; // - accelBias[2];

    AGM.readGyroData(AGM.gyroCount);  // Read the x/y/z adc values
    AGM.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    AGM.gx = (float)AGM.gyroCount[0]*AGM.gRes;
    AGM.gy = (float)AGM.gyroCount[1]*AGM.gRes;
    AGM.gz = (float)AGM.gyroCount[2]*AGM.gRes;

    AGM.readMagData(AGM.magCount);  // Read the x/y/z adc values
    AGM.getMres();
    // User environmental x-axis correction in milliGauss.
    // https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration
    AGM.magbias[0] += 470.;
    AGM.magbias[1] += 120.;
    AGM.magbias[2] += 125.;

    // Calculate the magnetometer values in milliGauss
    AGM.mx = (float)AGM.magCount[0]*AGM.mRes*AGM.magCalibration[0] - AGM.magbias[0];
    AGM.my = (float)AGM.magCount[1]*AGM.mRes*AGM.magCalibration[1] - AGM.magbias[1];
    AGM.mz = (float)AGM.magCount[2]*AGM.mRes*AGM.magCalibration[2] - AGM.magbias[2];
  }

  // Must be called before updating quaternions!
  AGM.updateTime();
  sum += AGM.deltat; // sum for averaging filter update rate
  sumCount++;

  if(SerialDebug)
  {
      ctr++; ctr%=SAMPLES;
      axSum -= ax[ctr]; aySum -= ay[ctr]; azSum -= az[ctr];
      gxSum -= gx[ctr]; gySum -= gy[ctr]; gzSum -= gz[ctr];

      axSum += ax[ctr] = 1000*AGM.ax;
      aySum += ay[ctr] = 1000*AGM.ay;
      azSum += az[ctr] = 1000*AGM.az;
      gxSum += gx[ctr] = AGM.gx;
      gySum += gy[ctr] = AGM.gy;
      gzSum += gz[ctr] = AGM.gz;
 
      // Print acceleration values in milligs!
      Serial.print(axSum/SAMPLES); Serial.print("\t");
      Serial.print(aySum/SAMPLES); Serial.print("\t");
      Serial.print(azSum/SAMPLES); Serial.print("\t");

      BTSerial.print(axSum/SAMPLES); BTSerial.print("\t");
      BTSerial.print(aySum/SAMPLES); BTSerial.print("\t");
      BTSerial.print(azSum/SAMPLES); BTSerial.print("\t");

      // Print gyro values in degree/sec
      Serial.print(gxSum/SAMPLES, 3); Serial.print("\t");
      Serial.print(gySum/SAMPLES, 3); Serial.print("\t");
      Serial.print(gzSum/SAMPLES, 3); Serial.print("\t");

      BTSerial.print(gxSum/SAMPLES, 3); BTSerial.print("\t");
      BTSerial.print(gySum/SAMPLES, 3); BTSerial.print("\t");
      BTSerial.print(gzSum/SAMPLES, 3); BTSerial.print("\t");

      // Print mag values in degree/sec
      Serial.print(AGM.mx); Serial.print("\t");
      Serial.print(AGM.my); Serial.print("\t");
      Serial.print(AGM.mz); Serial.print("\t");

      BTSerial.print(AGM.mx); BTSerial.print("\t");
      BTSerial.print(AGM.my); BTSerial.print("\t");
      BTSerial.print(AGM.mz); BTSerial.print("\t");

      Serial.print(AGM.deltat); Serial.print("\t");
      Serial.print((float)sumCount/sum, 2); Serial.print("\n");

      BTSerial.print(AGM.deltat); BTSerial.print("\t");
      BTSerial.print((float)sumCount/sum, 2); BTSerial.print("\n");
      sumCount = 0;
      sum = 0;
      /*
       * Serial.print("X-mag field: "); Serial.print(AGM.mx);
       * Serial.print(" mG ");
       * Serial.print("Y-mag field: "); Serial.print(AGM.my);
       * Serial.print(" mG ");
       * Serial.print("Z-mag field: "); Serial.print(AGM.mz);
       * Serial.println(" mG");
      */
  } 
}
