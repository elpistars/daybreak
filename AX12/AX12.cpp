/********************************************************
 *             EL-PISTOLERO REV.2  (MAIN)
 * ******************************************************
 * BISMILLAH ^_^
 * Humanoid Soccer Division
 * Electronic And Robotic Research Group
 * Telkom University
 * 2014
 * ******************************************************/

#include "publicjoint.h"
//#include "elpdxl.cpp"
//#include "elpbasic.cpp"
//#include "elpsoccer.cpp"
#include "elpi2c.cpp"
//#include "elpmove.cpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>

/*******************************************************
 ** Motion Setting**
 *******************************************************
 * WARMING UP
 * jalan(100,9000,10,-20,-20);
 *
 * DRIBLE
 * jalan(100,1000,10,-20,-20);
 *
 * JALAN DOANK
 * jalan(100,30000,10,-20,-20);
 *
 * SPRINT
 * jalan(1000,100,100,-10,-10);
 *
 * PUTAR KIRI
 * putar_kiri(30000);
 *
 * PUTAR KANAN
 * putar_kanan(30000);
 *
 * *****************************************************/
int actHistory;
int threadSensorRet,threadMoveRet;
pthread_t threadSensor,threadMove;

void CallSensor()
{
	EnableSensor=1;
	threadSensorRet = pthread_create( &threadSensor, 0, getData, 0);
        //Thread DEBUG
       if (threadSensorRet!=0){printf("Failed to call thread! Error Code %d", threadSensorRet);}
}

void WaitSensor()
{
	condition=0;
	EnableSensor=0;
	pthread_join(threadSensor,0);
}

void CallMove()
{
	move=1;
	threadMoveRet = pthread_create( &threadMove, 0, setMove, 0);
        //Thread DEBUG
       if (threadMoveRet!=0){printf("Failed to call thread! Error Code %d", threadMoveRet);}
}
void WaitMove()
{
	condition=0;
	move=0;
	pthread_join(threadMove,0);
}

void actionMove(int actCode)
{
	if (actCode != actHistory){
		move=0; //keluar dari pengulangan aksi sebelumnya
		while(ready==0){}
		kodegerak=actCode;
		actHistory=actCode;
		move=1; //ACTION!
		usleep(1000);		}
	else{}
}

int main(void)
{
	//ELP_initialize(0,1); //PORT=ttyUSB0 Baud=1Mbps
	//ELP_StatReturnSet(2); //0 for no return, 1 for read only, 2 for return all
	//setAllSpeed(100);
	CallSensor();
	//CallMove();
	//siap();
	sleep(3);
	//siapJalan();

	walkParam1=-10;
	walkParam2=-10;
	while(1)
	{
		//actionMove(5);
		usleep(10000);
		//waitMove()
		//printf ("Condtion: %d \n",condition);
		//printf("VirtualGyroX = %d	----	VirtualGyroY = %d \n",virtualGyroX,virtualGyroY); 

	}
	return 0;
}
