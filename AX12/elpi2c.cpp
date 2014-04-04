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
#include "sensor.c"
#include <i2c-dev.h>
#include "publicjoint.h"
#include "elpmove.cpp"

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

void conAction()
{
	if (condition == 1){
		//Tunggu class move
		//move=0;
		//while(ready==0){}
		printf("Saatnya Bangun depan");
		//bangun_depan();
		sleep(1);
		//siapJalan();
		sleep(5);
		move=1; //lanjutkan MOVE!

	}else if (condition == 2){
		//Tunggu class move
		//move=0;
		//while(ready==0){}
		printf("Saatnya Bangun belakang");
		//bangun_belakang();
		sleep(1);
		//siapJalan();
		sleep(5);
		move=1; //lanjutkan MOVE!
	}
}

void* getData(void* myParameter)
{
	int  *Pacc_raw;
	int  *Pmag_raw;
	int  acc_raw[3];
	int  mag_raw[3];
	float magXmax,magXmin;
	float magYmax,magYmin;
	float magZmax,magZmin;
	int i = 0;
	float AccXangle = 0.0;
	float AccYangle = 0.0;
	float MagX = 0.0;
	float MagY = 0.0;
	float MagZ = 0.0;
	float Xbefore = 250.0;
	float Ybefore = 250.0;
	float Zbefore = 250.0;
	Pacc_raw=acc_raw;
	Pmag_raw=mag_raw;
	int gyroYbuff,gyroXbuff;

	char filename[20];
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
        	printf("Unable to open I2C bus! \n");
        	exit(1);
	}
	printf("I2C Openned \n");

	enableACC();
	enableMAG();
	while(EnableSensor)
		{
		readACC(Pacc_raw);
		readMAG(Pmag_raw);
		AccXangle = (float) (atan2(*(acc_raw+1),*(acc_raw+2))+M_PI)*RAD_TO_DEG;
		AccYangle = (float) (atan2(*(acc_raw+2),*acc_raw)+M_PI)*RAD_TO_DEG;
		MagX= (float) *(mag_raw);
		MagZ= (float) *(mag_raw+1);
		MagY= (float) *(mag_raw+2);
		if (MagX<magXmin){magXmin=MagX;}else if(MagX>magXmax){magXmax=MagX;}
		if (MagY<magYmin){magYmin=MagY;}else if(MagY>magYmax){magYmax=MagY;}
		if (MagZ<magZmin){magZmin=MagZ;}else if(MagZ>magZmax){magZmax=MagZ;}
		printf ("   MagX \e[m %7.3f \t -- MagY %7.3f \t -- MagZ %7.3f \t \n",MagX,MagY,MagZ);
		printf ("   MinX \e[m %7.3f \t -- MinY %7.3f \t -- MinZ %7.3f \t \n",magXmin,magYmin,magZmin);
		printf ("   MaxX \e[m %7.3f \t -- MaxY %7.3f \t -- MaxZ %7.3f \t \n",magXmax,magYmax,magZmax);
		if ((AccYangle < 185) && (AccYangle > 100)) {condition=2;}
		else if ((AccYangle > 335)&&(AccYangle < 361)||((AccYangle >0)&& (AccYangle < 50))) {condition=1;}
		else {condition=0;}
		//printf ("   AccXangle \e[m %7.3f \t -- AccYangle %7.3f \t -- AccZangle %7.3f \t \n",AccXangle,AccYangle,AccZangle);
		if (i==100)
		{
		virtualGyroX=gyroXbuff;
		virtualGyroY=gyroYbuff;
		i=0;
		gyroXbuff=0;gyroYbuff=0;
		}
		else{
		gyroXbuff=gyroXbuff+(Xbefore-AccXangle);
		gyroYbuff=gyroYbuff+(Ybefore-AccYangle);
		virtualGyroX=0;virtualGyroY=0;}
		Xbefore=AccXangle; Ybefore=AccYangle;
		//conAction();
		usleep(10000);
		i++;
		}

return 0;
}
