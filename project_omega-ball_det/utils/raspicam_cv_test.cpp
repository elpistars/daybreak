//EDIT SOBIRIN 1 april 2014 19:49
//EDIT PW 5 April 2014 10.43
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
#include <math.h>

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
int lowerH_G=7;	int upperH_G=60;
int lowerS_G=91;	int upperS_G=229;
int lowerV_G=63;	int upperV_G=256;

//range HSV BOLA
int lowerH_B=6;		int upperH_B=14;
int lowerS_B=112;	int upperS_B=256;
int lowerV_B=187;	int upperV_B=256;

//range HSV BOLA
int lowerH_B2=10;	int upperH_B2=14;
int lowerS_B2=164;	int upperS_B2=255;
int lowerV_B2=109;	int upperV_B2=255;

//default capture width and height
const int FRAME_WIDTH  = 320;
const int FRAME_HEIGHT = 240;

//Global variables
Mat src, src_hsv, h_threshold;
int maxCorners=10; //10
int maxTrackbar=25;
float last_error_x, last_error_y, tx,ty;
int angguk_lokasi, toleh_lokasi;
bool scan_bola, scan_gawang, scan, track;
int serX,serY,statX,statY,ii,x,y,lastX,lastY;
int temp,dcs;
bool ball,goalpost;
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
void act (int a) ;
void scan_ball ();
void trackBall(int dcs,  Mat &imgThresh);//int Mat imgThresh
void trackBall2 (int dcs, Mat &imgThresh);
float getDistance(int &pos_servY);
int pixelmap (int x, int y);

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
  //  int nCount=100;
    int key = 0;
    last_error_x=0;
    last_error_y=0;
    angguk_lokasi=510;
    toleh_lokasi=250;
    scan_bola=true;
    scan_gawang=false;
    scan=false;
	serX=512;
	serY=390;
    while (1)
    {
	camera.grab();
	camera.retrieve(src);
    	cvtColor(src, src_hsv, COLOR_BGR2HSV);
	if(scan_bola)
	{
		inRange(src_hsv, Scalar(lowerH_B2, lowerS_B2, lowerV_B2), Scalar(upperH_B2, upperS_B2, upperV_B2),h_threshold);
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
		//cout<<"cari_bola"<<endl;
		//scan_gawang=true;
		//scan_bola=false;
//	 	scan_bola=true;
		trackBall(dcs,h_threshold); 
		if (ball) scan_ball ();
//		act(dcs);
	}
	if (scan) {tes_scan();}
	if(scan_gawang)
	{
		goodFeaturesToTrack_Demo(0,0);
//		scan_gawang=true;
		//cout<<"angguk = "<<angguk_lokasi<<endl;
		//cout<<"jarak  = "<<getDistance(angguk_lokasi)<<endl;
	}

	if (goalpost) trackBall2 (dcs,h_threshold);
	//cout<<"serX = "<<serX<<endl;
	//cout<<"serY = "<<serY<<endl;
	//key=waitKey(3);
	act(dcs);
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
		if(goalpost){scan=false,scan_gawang=false;};
	}
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
	Kp_x = 0.0625;		//	10=0.0625,  5=0.03125  PENTING!!!
	Kd_x = 0.055;		//	10=0.03125, 5=0.015625 PENTING!!!
	Kp_y = 0,0417;	//	10=0.083333, 5=0,041666667  PENTING!!!
	Kd_y = 0.03;	//	10=0.041667, 5=0,020833333  PENTING!!!

	if(c_pointX !=mid_x)
	{
		error_x = mid_x - c_pointX;
		P_x = (Kp_x * error_x);
		rate_x = error_x - last_error_x;
		D_x = (rate_x * Kd_x);
		MV_x = P_x + D_x;
		last_error_x = error_x;
		scan_x = scan_x + MV_x; 
		track=true;

		cout<<"error_x	= "<<error_x<<endl;
		//cout<<"P_x		= "<<P_x<<endl;
		//cout<<"rate_x	= "<<rate_x<<endl;
		//cout<<"D_x		= "<<D_x<<endl;
		//cout<<"MV_x		= "<<MV_x<<endl;
		//cout<<"last_erx = "<<last_error_x<<endl;
	}
	else
	{
	   track=false;
	}

	if (scan_x > 770) {scan_x = 770;} //looking to left ID:31
	if (scan_x < 250) {scan_x = 250;} //looking to right ID:31
	if(c_pointY !=mid_y)
	{
		error_y = mid_y - ty;
		P_y = (Kp_y * error_y);

		rate_y = error_y - last_error_y;
		D_y = (rate_y * Kd_y);

		MV_y = P_y + D_y;
		last_error_y = error_y;

		scan_y = scan_y + MV_y;
		track=true;

	}else{ track = false;}
	if(scan_y > 510) {scan_y = 510;} //atas
	if(scan_y < 205) {scan_y = 205;} //bawah

	if (error_x>-30 && error_x<30 && error_y<30 && error_y>-30) {goalpost=true; scan_gawang = false; scan = false; scan_bola=false; ball=false;track=false; usleep (5000000);} 
//	cout<<"error_y	= "<<error_y<<endl;
	/*cout<<"P_y		= "<<P_y<<endl;
	cout<<"rate_y	= "<<rate_y<<endl;
	cout<<"D_y		= "<<D_y<<endl;
	cout<<"MV_y		= "<<MV_y<<endl;
	cout<<"last_ery = "<<last_error_y<<endl;*/
}

void tes_scan()
{
	angguk_lokasi = 510;
	toleh_lokasi += 20;	//72, 18
	usleep(500);		//9000 900

	if(toleh_lokasi > 770) //810
	{
		toleh_lokasi = 250; //kiri-kanan: 872-154   	210=512-312, 810=512+
	}
	cout<<"toleh  lokasi = "<<toleh_lokasi<<endl;
	//cout<<"angguk lokasi = "<<angguk_lokasi<<endl;
	dxl_write_word(32,30,toleh_lokasi);
	dxl_write_word(31,30,angguk_lokasi);
}

float getDistance(int &pos_servY)
{
	float param;
	unsigned int jarak;
	/*if (pos_servY >= 510)
	{
		param = 89;
		jarak = 35*tan(param*PI/180.0);
		return jarak;
	}else if (pos_servY <=205)
	{
		param = 1;
		jarak = 35*tan(param*PI/180.0);
		return jarak;
	}else{*/
	param = (pos_servY-205)*0.293;
	jarak = 35*tan( param * PI / 180.0 ); //pos_servY*(300/1023)
	return jarak;
	//}
}

//========================= FUNGSI SCAN BOLA ===========================//
int pixelmap (int x, int y)
{
	int matriksA;
	if (( x<=106)			  && (y<=80))              {matriksA = 11;} else
	if (((x<=212) && (x>106)) && (y<=80))			    {matriksA = 12;} else
	if (((x<=320) && (x>212)) && (y<=80))			    {matriksA = 13;} else

	if (( x<=106)			  && ((y>80) && (y<=160))) {matriksA = 21;} else
	if (((x<=212) && (x>106)) && ((y>80) && (y<=160))) {matriksA = 22;} else
	if (((x<=320) && (x>212)) && ((y>80) && (y<=160))) {matriksA = 23;} else

	if (( x<=106)    		  && ((y>160) && (y<=240))) {matriksA = 31;} else
	if (((x<=212) && (x>106)) && ((y>160) && (y<=240))) {matriksA = 32;} else
	if (((x<=320) && (x>212)) && ((y>160) && (y<=240))) {matriksA = 33;}

	return (matriksA);
}

void act (int a)
{
switch (a)
	{
		case 1 : printf ("hadap kiri \n"); break;
		case 2 : printf ("hadap kanan \n"); break;
		case 3 : printf ("geser kiri \n"); break;
		case 4 : printf ("geser kanan \n"); break;
		case 5 : printf ("maju \n"); break;
		case 6 : printf ("tendang kiri \n"); break;
		case 7 : printf ("tendang kanan \n"); break;
		case 10: printf ("sprint \n"); break;
		case 11: printf ("maju omni kanan \n"); break;
		case 12: printf ("maju omni kiri \n"); break;
		case 20: printf ("tangkap depan \n"); break;
		case 21: printf ("tangkap kiri \n"); break;
		case 22: printf ("tangkap kanan \n"); break;
		case 23: printf ("bangun depan \n"); break;
		case 24: printf ("bangun belakang \n"); break;
		case 25: printf ("scan bola \n"); scan_bola=true; scan_gawang=false; break;
		case 26: printf ("scan gawang \n"); scan_gawang=true; scan=true; scan_bola=false; break;
		case 27: printf ("scan bola2\n"); scan_gawang=false; scan=false; scan_bola=true; break;
	}
}

void scan_ball ()
{
	bool c;
	//cout<<"scanning"<<endl;
	dxl_write_word( 31, 30, 150);//32
	dxl_write_word( 32, 30, 150);

	if (ii<5)
	{
	serX=serX+statX; 
	serY=serY+statY;
	if (statX==0) statX=15;
	if (serY>510) {serY=510; statY=0;} else if (serY<250) {serY=250; statY=0;}
	if (serX<382) {ii=ii+1; serX=382; statX=statX*-1; if (statX>0) {serY=375+(ii*29);} else {serY=375-(ii*29);}} else 
	if (serX>642) {ii=ii+1; serX=642; statX=statX*-1; if (statX>0) {serY=375+(ii*29);} else {serY=375-(ii*29);}}
	}
	else {	serX=512;
		serY= 375; ii=0; dcs=2;}
	cout<<"scan bola"<<endl;
	cout<<"servo X = "<<serX<<endl;
	cout<<"servo Y = "<<serY<<endl;
//	cout<<"dcs     = "<<dcs<<endl;
//	cout<<"statX   = "<<statX<<endl;
	dxl_write_word( 31, 30, serY);
	dxl_write_word( 32, 30, serX);

	usleep (50000);//50000
}

void trackBall(int temp,Mat &imgThresh)//int Mat imgThresh) 
{
	int x,y;
	//max number of objects to be detected in frame
	//minimum and maximum object area
	double refArea = 1;
	Mat imgTracking;

	// Calculate the moments of 'imgThresh;
	Moments moment = moments(imgThresh); //We give the binary converted frames for calculating the moments
	double area = moment.m00; //Sum of all white color pixels
	if(area>refArea)
	{
		// calculate the position of the ball
		x = moment.m10/area;
		y = moment.m01/area;
		refArea = area;
		if (pixelmap(x,y) != 0)
		temp = pixelmap(x,y);
		switch (temp)
		{
			case 11 : statX= 15;  statY= 10; break;
			case 12 : statX=  0;  statY= 10; break;
			case 13	: statX=-15;  statY= 10; break;
			case 21 : statX= 15;  statY=  0; break;
			case 22 :{statX=  0;  statY=  0; if (serY>260){
						if (serX<482) {dcs=2;}
						else if (serX>542) {dcs=1;}
						else {dcs=5;}}
						else dcs=26; break;}
			case 23 : statX=-15; statY=  0; break;
			case 31 : statX= 15; statY=-10; break;
			case 32 : statX=  0; statY=-10; break;
			case 33 : statX=-15; statY=-10; break;
		}

		serY=serY+statY;
		serX=serX+statX;

		if (serX<382) {serX=382; dcs=2;} else if (serX>642) {serX=642; dcs=1;} else 
		if (serY>510) {serY=510; dcs=5;} else if (serY<259)
		{
			serY=260;
			if (serX<482) {dcs=2;}
			else if (serX>542) {dcs=1;}
			else dcs=26;
		}

		dxl_write_word( 31, 30, serY);
		dxl_write_word( 32, 30, serX);
		ball=false;
		lastX = x;
		lastY = y;
		ii=-3; cout<<"found"<<endl;
	} else
	 { cout<<"not found"<<endl; if (ball==false) {ii=ii+1;} if (ii>=0) {ball=true;}}

	//free(moments);
}

void trackBall2(int temp,Mat &imgThresh)//int Mat imgThresh) 
{
	int x,y;
	double refArea = 1;
	Mat imgTracking;

		dxl_write_word( 31, 30, 260);
		dxl_write_word( 32, 30, 512);
	ii=ii+1;
	Moments moment = moments(imgThresh); //We give the binary converted frames for calculating the moments
	double area = moment.m00; //Sum of all white color pixels
	if(area>refArea && ii<5)
	{
		cout<<"found 2"<<endl; 
		x = moment.m10/area;
		y = moment.m01/area;
		refArea = area;
		if (pixelmap(x,y) != 0)
		temp = pixelmap(x,y);
		switch (temp)
		{
			case 21 : dcs=5;break;
			case 22 : 	if (serX<482) {dcs=4;}
				   else if (serX>542) {dcs=3;}
				   else {dcs=5;} break;
			case 23 : dcs=5;  break;
			case 31 : dcs=6; goalpost=false; break;
			case 32 : 	if (serX<482) {dcs=4;}
				   else if (serX>542) {dcs=3;}
				   else {dcs=5;} break;
			case 33 :  dcs=7; goalpost = false; break;
		ii=ii-1;
		}


		ball=false;
		} else { cout<<"not found 2"<<endl;  if (ii>=5) {scan_bola=true; scan_gawang=false; scan=false; ball=true; goalpost=false;}}
}


//http://docs.opencv.org/doc/tutorials/features2d/trackingmotion/corner_subpixeles/corner_subpixeles.html

