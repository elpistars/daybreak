/*****************************************************************
        ELPISTOLERO MOTION CLASS
        -> Rev 1


ELPISTOLERO HUMANOID ROBOT SOCCER TEAM
ELECTRONIC AND INTELLIGENCE ROBOTIC RESEARCH GROUP
TELKOM UNIVERSITY
-2014-
******************************************************************/
//BISMILLAH ^_^

#include "elpdxl.cpp"
#include "elpsoccer.cpp"
#include "elpbasic.cpp"
#include "publicjoint.h"
#include <stdio.h>

//variable backup array parameter
int bKa1,bKa2,bKi1,bKi2;

void setParameter(int Ka1, int Ki1, int Ka2, int Ki2)
{
        //********************   SET ALL ARRAY   *********************//
        for ( int Xid = 0; Xid < 7 ; Xid++)
        {
        FSL[Xid][IDservo[10]]=FSL[Xid][IDservo[10]]+Ka1; FSL[Xid][IDservo[11]]=FSL[Xid][IDservo[11]]-Ki1;
        FSL[Xid][IDservo[14]]=FSL[Xid][IDservo[14]]+Ka2; FSL[Xid][IDservo[15]]=FSL[Xid][IDservo[15]]-Ki2;

        FSL2[Xid][IDservo[10]]=FSL2[Xid][IDservo[10]]+Ka1; FSL2[Xid][IDservo[11]]=FSL2[Xid][IDservo[11]]-Ki1;
        FSL2[Xid][IDservo[14]]=FSL2[Xid][IDservo[14]]+Ka2; FSL2[Xid][IDservo[15]]=FSL2[Xid][IDservo[15]]-Ki2;

        FML[Xid][IDservo[10]]=FML[Xid][IDservo[10]]+Ka1; FML[Xid][IDservo[11]]=FML[Xid][IDservo[11]]-Ki1;
        FML[Xid][IDservo[14]]=FML[Xid][IDservo[14]]+Ka2; FML[Xid][IDservo[15]]=FML[Xid][IDservo[15]]-Ki2;

        FML2[Xid][IDservo[10]]=FML2[Xid][IDservo[10]]+Ka1; FML2[Xid][IDservo[11]]=FML2[Xid][IDservo[11]]-Ki1;
        FML2[Xid][IDservo[14]]=FML2[Xid][IDservo[14]]+Ka2; FML2[Xid][IDservo[15]]=FML2[Xid][IDservo[15]]-Ki2;

        FMR[Xid][IDservo[10]]=FMR[Xid][IDservo[10]]+Ka1; FMR[Xid][IDservo[11]]=FMR[Xid][IDservo[11]]-Ki1;
        FMR[Xid][IDservo[14]]=FMR[Xid][IDservo[14]]+Ka2; FMR[Xid][IDservo[15]]=FMR[Xid][IDservo[15]]-Ki2;

        FMR2[Xid][IDservo[10]]=FMR2[Xid][IDservo[10]]+Ka1; FMR2[Xid][IDservo[11]]=FMR2[Xid][IDservo[11]]-Ki1;
        FMR2[Xid][IDservo[14]]=FMR2[Xid][IDservo[14]]+Ka2; FMR2[Xid][IDservo[15]]=FMR2[Xid][IDservo[15]]-Ki2;

        FER[Xid][IDservo[10]]=FER[Xid][IDservo[10]]+Ka1; FER[Xid][IDservo[11]]=FER[Xid][IDservo[11]]-Ki1;
        FER[Xid][IDservo[14]]=FER[Xid][IDservo[14]]+Ka2; FER[Xid][IDservo[15]]=FER[Xid][IDservo[15]]-Ki2;

        FER2[Xid][IDservo[10]]=FER2[Xid][IDservo[10]]+Ka1; FER2[Xid][IDservo[11]]=FER2[Xid][IDservo[11]]-Ki1;
        FER2[Xid][IDservo[14]]=FER2[Xid][IDservo[14]]+Ka2; FER2[Xid][IDservo[15]]=FER2[Xid][IDservo[15]]-Ki2;
	}
	bKa1=Ka1;
	bKa2=Ka2;
	bKi1=Ki1;
	bKi2=Ki2;
}

void resetParameter()
{

        //********************   RESET ALL ARRAY   *********************//
        for ( int Xid = 0; Xid < 7 ; Xid++)
        {
        FSL[Xid][IDservo[10]]=FSL[Xid][IDservo[10]]-bKa1; FSL[Xid][IDservo[11]]=FSL[Xid][IDservo[11]]+bKi1;
        FSL[Xid][IDservo[14]]=FSL[Xid][IDservo[14]]-bKa2; FSL[Xid][IDservo[15]]=FSL[Xid][IDservo[15]]+bKi2;

        FSL2[Xid][IDservo[10]]=FSL2[Xid][IDservo[10]]-bKa1; FSL2[Xid][IDservo[11]]=FSL2[Xid][IDservo[11]]+bKi1;
        FSL2[Xid][IDservo[14]]=FSL2[Xid][IDservo[14]]-bKa2; FSL2[Xid][IDservo[15]]=FSL2[Xid][IDservo[15]]+bKi2;

        FML[Xid][IDservo[10]]=FML[Xid][IDservo[10]]-bKa1; FML[Xid][IDservo[11]]=FML[Xid][IDservo[11]]+bKi1;
        FML[Xid][IDservo[14]]=FML[Xid][IDservo[14]]-bKa2; FML[Xid][IDservo[15]]=FML[Xid][IDservo[15]]+bKi2;

        FML2[Xid][IDservo[10]]=FML2[Xid][IDservo[10]]-bKa1; FML2[Xid][IDservo[11]]=FML2[Xid][IDservo[11]]+bKi1;
        FML2[Xid][IDservo[14]]=FML2[Xid][IDservo[14]]-bKa2; FML2[Xid][IDservo[15]]=FML2[Xid][IDservo[15]]+bKi2;

        FMR[Xid][IDservo[10]]=FMR[Xid][IDservo[10]]-bKa1; FMR[Xid][IDservo[11]]=FMR[Xid][IDservo[11]]+bKi1;
        FMR[Xid][IDservo[14]]=FMR[Xid][IDservo[14]]-bKa2; FMR[Xid][IDservo[15]]=FMR[Xid][IDservo[15]]+bKi2;

        FMR2[Xid][IDservo[10]]=FMR2[Xid][IDservo[10]]-bKa1; FMR2[Xid][IDservo[11]]=FMR2[Xid][IDservo[11]]+bKi1;
        FMR2[Xid][IDservo[14]]=FMR2[Xid][IDservo[14]]-bKa2; FMR2[Xid][IDservo[15]]=FMR2[Xid][IDservo[15]]+bKi2;

        FER[Xid][IDservo[10]]=FER[Xid][IDservo[10]]-bKa1; FER[Xid][IDservo[11]]=FER[Xid][IDservo[11]]+bKi1;
        FER[Xid][IDservo[14]]=FER[Xid][IDservo[14]]-bKa2; FER[Xid][IDservo[15]]=FER[Xid][IDservo[15]]+bKi2;

        FER2[Xid][IDservo[10]]=FER2[Xid][IDservo[10]]-bKa1; FER2[Xid][IDservo[11]]=FER2[Xid][IDservo[11]]+bKi1;
        FER2[Xid][IDservo[14]]=FER2[Xid][IDservo[14]]-bKa2; FER2[Xid][IDservo[15]]=FER2[Xid][IDservo[15]]+bKi2;
        }
	bKa1=0;
	bKa2=0;
	bKi1=0;
	bKi2=0;
}

void* setMove(void* myParameter)
{
	printf("move class started!");//debug
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
			setParameter(-15,15,3,-3);
        	        jalan(100,16700); //pengulangan ada di voidnya
        	        siapJalan();
			resetParameter();
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
