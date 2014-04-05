#ifndef __ELPISTOLEROBASIC_HPP
#define __ELPISTOLEROBASIC_HPP

	int x,y;
	int IDservo [20];
	int walkReady[18];
	int FSL[7][18];
	int FSL2[7][18];
	int FSR[7][18];
	int FSR2[7][18];
	int FML[7][18];
	int FML2[7][18];
	int FMR[7][18];
	int FMR2[7][18];
	int FEL[7][18];
	int FEL2[7][18];
	int FER[7][18];
	int FER2[7][18];
	int RML[7][18];
	int RML2[7][18];
	int RMR[7][18];
	int RMR2[7][18];
	int LML[7][18];
	int LML2[7][18];
	int LMR[7][18];
	int LMR2[7][18];
	int LTML[7][18];
	int LTML2[7][18];
	int LTMR[7][18];
	int LTMR2[7][18];
	int RTML[7][18];
	int RTML2[7][18];
	int RTMR[7][18];
	int RTMR2[7][18];
	int LKICK[6][18];
	int RKICK[6][18];
	int SIT_DOWN[18];
	int UP_F [13][18];
	int delay_UP_F [13];
	int UP_B [26][18];
	int speed_UP_B[26];
	int delay_UP_B[26];
	void siapJalan();
	void jalan (int speed,int delayKu, int loop, int per11, int per12);
	void geser_kanan(int delayKu);
	void geser_kiri(int delayKu);
	void putar_kanan(int delayKu);
	void putar_kiri(int delayKu);
	void duduk();
	void bangun_depan();
	void bangun_belakang();
	void ELP_StatReturnSet(int level);
	void jalanDulu (int speed,int delayKu, int loop, int per11, int per12);

#endif
