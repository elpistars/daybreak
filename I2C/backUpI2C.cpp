#include </home/pi/I2C/LSM303DLHC.cpp>
#include <stdio.h>
#include <unistd.h>


/*
   getHeading() calculates a tilt-compensated heading.
   A float between 0 and 360 degrees is returned. You need
   to pass this function both a magneto and acceleration array.

   Headings are calculated as specified in AN3192:
   http://www.sparkfun.com/datasheets/Sensors/Magneto/Tilt%20Compensated%20Compass.pdf

*/

float getHeading(LSM303& lsm303dlhc);

#define PI 3.141592654


int main(void)
{
    uint8_t bajt;
    const char *fileN = "/dev/i2c-1";
    LSM303 lsm303dlhc(fileN);

    lsm303dlhc.enalbe();

    while(1)
    {
        lsm303dlhc.readAccelerationRaw();
        lsm303dlhc.readMagnetometerRaw();
        printf("acc [m/s^2]: \e[27;1;31m %f \e[m \e[27;1;32m %f \e[m \e[27;1;34m %f \e[m mag:  \e[27;1;31m %d \e[m  \e[27;1;32m %d \e[m \e[27;1;34m %d\e[m %fdeg\n",
               (int16_t)lsm303dlhc.acc_x_raw*0.00957,(int16_t)lsm303dlhc.acc_y_raw*0.00957,-(int16_t)lsm303dlhc.acc_z_raw*0.00957,
               (int16_t)lsm303dlhc.mag_x_raw, (int16_t)lsm303dlhc.mag_y_raw, (int16_t)lsm303dlhc.mag_z_raw,getHeading(lsm303dlhc));
        usleep(10);
       }

    }


float getHeading(LSM303& lsm303dlhc)
{
  float heading,pitch,roll,xh,yh,zh;
  // see section 1.2 in app note AN3192
  int magValue[3];
  float accelValue[3];
  magValue[0] = (int16_t)lsm303dlhc.mag_x_raw;
  magValue[1] = (int16_t)lsm303dlhc.mag_y_raw;
  magValue[2] = (int16_t)lsm303dlhc.mag_z_raw;
  accelValue[0] = (int16_t)lsm303dlhc.acc_x_raw*0.000976531;
  accelValue[1] = (int16_t)lsm303dlhc.acc_y_raw*0.000976531;
  accelValue[2] = -(int16_t)lsm303dlhc.acc_z_raw*0.000976531;

    // see appendix A in app note AN3192
  pitch = asin(-accelValue[0]);
  roll = asin(accelValue[1]/cos(pitch));

  xh = magValue[0] * cos(pitch) + magValue[2] * sin(pitch);
  yh = magValue[0] * sin(roll) * sin(pitch) + magValue[1] * cos(roll) - magValue[2] * sin(roll) * cos(pitch);
  zh = -magValue[0] * cos(roll) * sin(pitch) + magValue[1] * sin(roll) + magValue[2] * cos(roll) * cos(pitch);

  heading = 180*atan2(yh,xh)/PI;

  if (heading <0)
    heading += 360;

  return heading;
}
