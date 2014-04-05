//EDIT SOBIRIN 1 april 2014 19:49

#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include </home/pi/project_delta/src/raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <string>
#include <cmath>
#include <dynamixel.h>

//Control table address
#define P_GOAL_POSITION_L	30
#define P_GOAL_POSITION_H	31
#define P_GOAL_SPEED_L		32
#define	P_GOAL_SPEED_H		33
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING		46

//default setting
#define DEFAULT_PORTNUM		0	//COM 0
#define DEFAULT_BAUDNUM		1	//1 Mbps
#define NUM_ACTUATOR		2	// number of actuator
#define STEP_THETA		(PI/100.0f)//large value is more fast
#define CONTROL_PERIOD		(10)	//msec (large value is more slow)

#define PI 3.14159265

using namespace std;
using namespace cv;
bool doTestSpeedOnly=false;
//
/*
HSV GAWANG
7-60
91-229
63-256

22-34
83-201
131-179


HSV BOLA
6-14
112-256
98/187-256
*/

//range HSV GAWANG
int lowerH_G=22;	int upperH_G=34;
int lowerS_G=83;	int upperS_G=201;
int lowerV_G=131;	int upperV_G=179;

//range HSV BOLA
int lowerH_B=6;		int upperH_B=14;
int lowerS_B=112;	int upperS_B=256;
int lowerV_B=187;	int upperV_B=256;

//default capture width and height
const int FRAME_WIDTH  = 640;
const int FRAME_HEIGHT = 480;

//Global variables
Mat src, src_hsv, h_threshold;
int maxCorners=10;
int maxTrackbar=25;
float last_error_x, last_error_y, tx,ty;
int angguk_lokasi, toleh_lokasi;
bool scan_bola, scan_gawang, scan, track;
RNG rng(12345);
char* source_window = "Image";

//parse command line
//returns the index of a command line param in argv. If not found, return -1
int findParam ( string param,int argc,char **argv ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    return idx;

}
//parse command line
//returns the value of a command line param. If not found, defvalue is returned
float getParamVal ( string param,int argc,char **argv,float defvalue=-1 ) {
    int idx=-1;
    for ( int i=0; i<argc && idx==-1; i++ )
        if ( string ( argv[i] ) ==param ) idx=i;
    if ( idx==-1 ) return defvalue;
    else return atof ( argv[  idx+1] );
}

void processCommandLine ( int argc,char **argv,raspicam::RaspiCam_Cv &Camera ) {
    Camera.set ( CV_CAP_PROP_FRAME_WIDTH,  getParamVal ( "-w",argc,argv,320 ) );
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, getParamVal ( "-h",argc,argv,240 ) );
    Camera.set ( CV_CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );
    //Camera.set ( CV_CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
    //Camera.set ( CV_CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
    //Camera.set ( CV_CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );
    if ( findParam ( "-gr",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC1 );
    if ( findParam ( "-test_speed",argc,argv ) !=-1 )
        doTestSpeedOnly=true;
    if ( findParam ( "-ss",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_EXPOSURE, getParamVal ( "-ss",argc,argv )  );


//     Camera.setSharpness ( getParamVal ( "-sh",argc,argv,0 ) );
//     if ( findParam ( "-vs",argc,argv ) !=-1 )
//         Camera.setVideoStabilization ( true );
//     Camera.setExposureCompensation ( getParamVal ( "-ev",argc,argv ,0 ) );


}

void showUsage() {
    cout<<"Usage: "<<endl;
    cout<<"[-gr set gray color capture]\n";
    cout<<"[-test_speed use for test speed and no images will be saved]\n";
    cout<<"[-w width] [-h height] \n[-br brightness_val(0,100)]\n";
    cout<<"[-co contrast_val (0 to 100)]\n[-sa saturation_val (0 to 100)]";
    cout<<"[-g gain_val  (0 to 100)]\n";
    cout<<"[-ss shutter_speed (0 to 100)]\n";
    cout<<endl;
}

//function header
void goodFeaturesToTrack_Demo(int, void*);
void morphOps(Mat &thresh);
void cari_bola();
void DrawLine(Mat &frame);
void PID_Scan(float &c_pointX, float &c_pointY, float &last_error_x, float &last_error_y, int &scan_x, int &scan_y);
void tes_scan();
float getDistance(int &pos_servY);
Mat rotate(Mat src, double angle);


int main ( int argc,char **argv )
 {
   	if(dxl_initialize(DEFAULT_PORTNUM, DEFAULT_BAUDNUM) == 0)
	{
		printf("Failed to open USB2Dynamixel!\n");
		printf("Press any key to terminate...\n");
		return 0;
	}
	else
	{
		printf("Succeed to open USB2Dynamixel!\n");
	}
	cout<<"Bismillah..."<<endl;
    if ( argc==1 ) {
        cerr<<"Usage (-help for help)"<<endl;
    }
    if ( findParam ( "-help",argc,argv ) !=-1 ) {
        showUsage();
        return -1;
    }

    raspicam::RaspiCam_Cv camera;
    processCommandLine ( argc,argv,camera );
    cout<<"Connecting..."<<endl;
    if ( !camera.open() ) {
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
    cout<<"Connected to camera ="<<camera.getId() <<endl;

    Mat img;
    int nCount=100;
    int key = 0;
    last_error_x=0;
    last_error_y=0;
    angguk_lokasi=510;
    toleh_lokasi=210;
    scan_bola=false;
    scan_gawang=true;
    scan=true;
    while (1)
    {
	if(scan)
	{
		tes_scan();
	}
	camera.grab();
	camera.retrieve(src);
	//src = rotate(src, 270);
    	cvtColor(src, src_hsv, COLOR_BGR2HSV);
	if(scan_bola)
	{
		inRange(src_hsv, Scalar(lowerH_B, lowerS_B, lowerV_B), Scalar(upperH_B, upperS_B, upperV_B),h_threshold);
	}
	else
	{
		inRange(src_hsv, Scalar(lowerH_G, lowerS_G, lowerV_G), Scalar(upperH_G, upperS_G, upperV_G),h_threshold);
	}
	bool useMorphOps = true;
	if(useMorphOps)
		morphOps(h_threshold);

	if(scan_bola)
	{
		cout<<"cari_bola"<<endl;
		//scan_gawang=true;
		//scan_bola=false;
		scan_bola=true;
	}
	if(scan_gawang)
	{
		goodFeaturesToTrack_Demo(0,0);
		scan_gawang=true;
	}
	key=waitKey(3);
    }
	camera.release();
}

/*
	function goodFeaturesToTrack_Demo.cpp
	brief Apply Shi-Thomasi corner detector
*/


void goodFeaturesToTrack_Demo(int, void*)
{
	if(maxCorners < 1){ maxCorners = 1;}

	//parameter for Shi-Thomasi algorithm
	vector<Point2f> corners;
	double qualityLevel = 0.1; //0.01 semakin besar semakin akurat!!
	double minDistance = 40;   //semakin kecil semakin lemot prosesnya!!
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;
	int max_x, min_x, max_y, min_y;
	double delta, delta_x, delta_y;
	bool gawang;

	//copy the source image
	Mat copy;
	copy = src.clone();

	//apply corner detection
	goodFeaturesToTrack(	h_threshold,
				corners,
				maxCorners,
				qualityLevel,
				minDistance,
				Mat(),
				blockSize,
				useHarrisDetector,
				k);
/*
	//Draw corners detected
	cout<<"** Number of corners detected: "<<corners.size()<<endl;
	int r = 4;
	for(int i=0; i<corners.size(); i++)
	{ circle(copy, corners[i], r, Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)), -1, 8, 0);}
*/

	max_x=0;
	max_y=0;
	min_x=2600;
	min_y=2600;

	//write them down
	for(int i=0; i<corners.size(); i++)
	{
		//cout<<" -- Refined Corner ["<<i<<"]("<<corners[i].x","<<corners[i].y<<")"<<endl;
		//hitung nilai max_x, min_y, max_y, y1
		if(corners[i].x>=max_x){max_x=corners[i].x;}
		if(corners[i].x<=min_x){min_x=corners[i].x;}
		if(corners[i].y>=max_y){max_y=corners[i].y;}
		if(corners[i].y<=min_y){min_y=corners[i].y;}
	}

	//nilai tengah
	tx=min_x+((max_x-min_x)/2);
	ty=max_y;

	delta_x = max_x - min_x;	//max_y = piksel bawah, min_y = piksel atas
	delta_y = max_y - min_y;
	delta = delta_x/delta_y;	//lebar gawang dibandingkan dengan tinggi gawang

	if(tx!=0 && ty!=0)
	{
		gawang = true;
	}
	else
	{
		gawang = false;
		cout << "KOSONG" <<endl;
		scan_gawang = true;
		scan_bola = false;
		scan=true;
	}

	//cout << "titik tengah = "<<Point(tx,ty)<<endl;
	//cout << "delta	      = "<<delta<<endl;

	//tandain titik tengah gawang!!
	//circle( copy, Point(tx,ty), 5, Scalar(0), 2, 8, 0);

	if(gawang)
	{
		if((delta<1)&&(delta>-1))
		{
			cout<<"BUKAN GAWANG!!"<<endl;
			scan_gawang = true;
			scan_bola = false;
			scan=true;
		}
		else
		{
			//if(tx<160) maka ke kanan, selebihnya ke kiri
			//if(ty<120) maka ke bawah, selebihnya ke atas
			cout<<"GAWANG "<<endl;
			scan_gawang = true;
			scan_bola = false;
			scan=false;
			track=true;
			if(track)
			{
				PID_Scan(tx,ty,last_error_x,last_error_y,toleh_lokasi,angguk_lokasi);
				cout<<"track PID!!"<<endl;
			}
			else
			{
				track=false;
			}
			dxl_write_word(32,30,toleh_lokasi);
			dxl_write_word(31,30,angguk_lokasi);
		}
	}
	else
	{
		scan=true;
		track=false;
		scan_gawang=true;
	}
/*if(gawang)
  {	
	if((delta<1) && (delta>-1))
	  {
		  cout << "BUKAN GAWANG!!" << endl;scan_gawang=true;scan_bola=false;scan=true;
	  }
	  else
	  {
		  //if(tx<320) maka ke kanan, selebihnya ke kiri
		  //if(ty<240) maka ke bawah, selebihnya ke atas
		   cout << "GAWANG" << endl;scan_gawang=true;scan_bola=false; 
		   scan=false;
		   track=true;
		  
		  //track gawang
		   if(track)
		   {
			if(ty<80)//320 120
			{
				toleh_lokasi += 10;//72
				cout<<"ke kiri!!"<<endl;
				track=true;
			}
			else if(ty>160)//360
			{
				toleh_lokasi -= 10;//72
				cout<<"ke kanan!!!"<<endl;
				track=true;
			}
			else
			{
				cout<<"TENGAH X!!!"<<endl;
				cout<<"toleh  lokasi = "<<toleh_lokasi<<endl;
				track=false;
			}
			if (toleh_lokasi > 810) {toleh_lokasi = 810;} //looking to left ID:31
			if (toleh_lokasi < 210) {toleh_lokasi = 210;} //looking to right ID:31

/*			if(ty<220)
			{
				angguk_lokasi+=5;
				cout<<"ke bawah!!"<<endl;
				track=true;
			}
			else if(ty>260)
			{
				angguk_lokasi-=5;
				cout<<"ke atas!!!"<<endl;
				track=true;
			}
			else 
			{
				cout<<"TENGAH !!!"<<endl;
				cout<<"angguk lokasi = "<<angguk_lokasi<<endl;
				track=false;
			}
			if(angguk_lokasi > 510) {angguk_lokasi = 510;} //atas
			if(angguk_lokasi < 205) {angguk_lokasi = 205;} //bawah

			dxl_write_word(32,30,toleh_lokasi);
			dxl_write_word(31,30,angguk_lokasi);

		   }
		   else
		   {
			   scan=true;
			   track=false;
			   scan_gawang=true;
		   }

	}
  }*/

}

void morphOps(Mat &thresh)
{
	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3));
	//dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(5,5)); //(8,8)

	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);

	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

void PID_Scan(float &c_pointX, float &c_pointY, float &last_error_x, float &last_error_y, int &scan_x, int &scan_y)
{
	int mid_x = 160;//320
	int mid_y = 120;//240
	float P_x, P_y, Kp_x, Kd_x, Kp_y, Kd_y,rate_y, rate_x, D_x, D_y, MV_x,MV_y;
	float error_x,error_y;
	Kp_y = 0.1111;		//0.03125=10; 0.015625=5	PENTING!!!
	Kd_y = 0.8;		//0.05556			PENTING!!!
	Kp_x = 0.0625;		//	PENTING!!!
	Kd_x = 0.03125;		//0.020833,, 0.013333		PENTING!!!

	if(c_pointX !=mid_x)
	{
		error_x = mid_x - c_pointX;
		P_x = (Kp_x * error_x);
		rate_x = error_x - last_error_x;
		D_x = (rate_x * Kd_x);
		MV_x = P_x + D_x;
		last_error_x = error_x;
		scan_x = scan_x + MV_x;
		//buff_x = (scan_x);
		track=true;

		cout<<"error_x	= "<<error_x<<endl;
		cout<<"P_x		= "<<P_x<<endl;
		cout<<"rate_x	= "<<rate_x<<endl;
		cout<<"D_x		= "<<D_x<<endl;
		cout<<"MV_x		= "<<MV_x<<endl;
		cout<<"last_erx = "<<last_error_x<<endl;
	}
	else
	{
	   track=false;
	}

	if (scan_x > 810) {scan_x = 810;} //looking to left ID:31
	if (scan_x < 210) {scan_x = 210;} //looking to right ID:31
	/*if(c_pointY !=mid_y)
	{
		error_y = mid_y - ty;
		P_y = (Kp_y * error_y);

		rate_y = error_y - last_error_y;
		D_y = (rate_y * Kd_y);

		MV_y = P_y + D_y;
		last_error_y = error_y;

		scan_y = scan_y + MV_y;
		//buff_y = (scan_y);
		track=true;

	}else{ track = false;}
	if(scan_y > 510) {scan_y = 510;} //atas
	if(scan_y < 205) {scan_y = 205;} //bawah
*/
	/*
	cout<<"error_y	= "<<error_y<<endl;
	cout<<"P_y		= "<<P_y<<endl;
	cout<<"rate_y	= "<<rate_y<<endl;
	cout<<"D_y		= "<<D_y<<endl;
	cout<<"MV_y		= "<<MV_y<<endl;
	cout<<"last_ery = "<<last_error_y<<endl;*/
}

void tes_scan()
{
	angguk_lokasi = 510;
	toleh_lokasi += 20;	//72, 18
	usleep(900);		//9000

	if(toleh_lokasi > 810)
	{
		toleh_lokasi = 210; //kiri-kanan: 872-154
	}
	cout<<"toleh  lokasi = "<<toleh_lokasi<<endl;
	//cout<<"angguk lokasi = "<<angguk_lokasi<<endl;
	dxl_write_word(32,30,toleh_lokasi);
	dxl_write_word(31,30,angguk_lokasi);
}

/*
float getDistance(int &pos_servY)
{
	float param;
	unsigned int jarak;
	if (pos_servY >= 510)
	{
		param = 89;
		jarak = 35*tan(param*PI/180.0);
		return jarak;
	}else if (pos_servY <=205)
	{
		param = 1;
		jarak = 35*tan(param*PI/180.0);
		return jarak;
	}else{
	param = (pos_servY-204.8)*0.293;
	jarak = 35*tan( param * PI / 180.0 ); //pos_servY*(300/1023)
	return jarak;
	}
}*/

Mat rotate(Mat src, double angle)
{
    Mat dst;
    Point2f pt(src.cols/2., src.rows/2.);    //pt(src.cols/2., src.rows/2.);
    Mat r = getRotationMatrix2D(pt, angle, 1.0);
    warpAffine(src, dst, r, Size(src.cols, src.rows));
    return dst;
}
//http://docs.opencv.org/doc/tutorials/features2d/trackingmotion/corner_subpixeles/corner_subpixeles.html
