/*
* The code is released under the GNU General Public License.
* Developed by Mark Williams
* A guide to this code can be found here; http://marks-space.com/2013/04/22/845/
* Created 28th April 2013
*/


#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/time.h>

#include <L3G.h>
#include <LSM303.h>
#include "/home/pi/I2C/sensor.c"
#include <i2c-dev.h>

#define X   0
#define Y   1
#define Z   2

#define DT 0.02         // [s/loop] loop period. 20ms
#define AA 0.98         // complementary filter constant

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.070     // [deg/s/LSB]
#define RAD_TO_DEG 57.29578
#define M_PI 3.14159265358979323846


// Enable accelerometer.
void enableACC()
{
	writeAccReg(LSM303_CTRL_REG1_A, 0b01010111); //  z,y,x axis enabled , 100Hz data rate
	writeAccReg(LSM303_CTRL_REG4_A, 0b00101000); // +/- 8G full scale: FS = 10 on DLHC, high resolution output mode
}

void enableMAG()
{
	writeMagReg(LSM303_MR_REG_M, 0x00);
}

int main()
{
	int  *Pacc_raw;
	int  *Pmag_raw;
	int  acc_raw[3];
	int  mag_raw[3];
	int i = 0;
	float AccYangle = 0.0;
	float AccXangle = 0.0;
	
	Pacc_raw=acc_raw;
	Pmag_raw=mag_raw;

	

	char filename[20];
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
        	printf("Unable to open I2C bus! \n");
        	exit(1);
	}
	printf("I2C Openned \n");

	enableACC();

	while(i<1000)
	{
		i++;
		readACC(Pacc_raw);
		AccXangle = (float) (atan2(*(acc_raw+1),*(acc_raw+2))+M_PI)*RAD_TO_DEG;
		AccYangle = (float) (atan2(*(acc_raw+2),*acc_raw)+M_PI)*RAD_TO_DEG;
		if (AccXangle >180){AccXangle -= (float)360.0;}
	        if (AccYangle >180){AccYangle -= (float)360.0;}
		printf ("   AccXangle \e[m %7.3f \t --- AccYangle %7.3f \t \n",AccYangle,AccXangle);
		usleep(100000);
	}

return 0;
}
