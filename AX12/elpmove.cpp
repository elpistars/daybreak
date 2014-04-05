#include "elpdxl.cpp"
#include "elpsoccer.cpp"
#include "elpbasic.cpp"
#include "publicjoint.h"
#include <stdio.h>

void* setMove(void* myParameter)
{
	printf("move class started!");
	while(1)
	{
		if (move)
		{
		switch(kodegerak)
		{
			case 1 : //hadap kiri
			ready=0;
			while(move){putar_kiri(30000);}
			siapJalan();
			ready=1;
			break;

			case 2 : //hadap kanan
			ready=0;
	                while(move){putar_kanan(30000);}
	                siapJalan();
			ready=1;
			break;

			case 3 : //geser kiri
			ready=0;
        	        while(move){geser_kiri(30000);}
        	        siapJalan();
			ready=1;
			break;

			case 4 : //geser kanan
			ready=0;
        	        while(move){geser_kanan(30000);}
               		siapJalan();
			ready=1;
			break;

			case 5 : //maju
			ready=0;
        	        jalan (150,15000,walkParam1,walkParam2); //pengulangan ada di voidnya
        	        siapJalan();
			ready=1;
			break;

			case 6 : //tendang kiri
			ready=0;
        	        while(move){putar_kiri(30000);}
        	        siapJalan();
			ready=1;
			break;

			case 7 : //tendang kanan
			ready=0;
        	        while(move){putar_kiri(30000);}
        	        siapJalan();
			ready=1;
			break;

			case 10: //sprint
			ready=0;
               		while(move){putar_kiri(30000);}
                	siapJalan();
			ready=1;
			break;

			case 11: //maju omni kanan
			ready=0;
                	while(move){putar_kiri(30000);}
                	siapJalan();
			ready=1;
			break;

			case 12: //maju omni kiri 
			ready=0;
                	while(move){putar_kiri(30000);}
                	siapJalan();
			ready=1;
			break;

			case 20: //tangkap depan
			ready=0;
        	        while(move){putar_kiri(30000);}
                	siapJalan();
			ready=1;
			break;

			case 21: //tangkap kiri
			ready=0;
	                while(move){putar_kiri(30000);}
	                siapJalan();
			ready=1;
			break;

			case 22: //tangkap kanan
			ready=0;
        	        while(move){putar_kiri(30000);}
        	        siapJalan();
			ready=1;
			break;

			case 23: //bangun depan
			ready=0;
        	        while(move){putar_kiri(30000);}
        	        siapJalan();
			ready=1;
			break;

			case 24: //bangun belakang
			ready=0;
        	        while(move){putar_kiri(30000);}
               		siapJalan();
			ready=1;
			break;

			case 25: //scan bola
			ready=0;
			//scan_bola=true;
                	while(move){putar_kiri(30000);}
                	siapJalan();
			ready=1;
			break;

			default:
			ready=0;
			siapJalan();
			ready=1;
			break;
		}
		usleep(1000);
		}
	}
return 0;
}
