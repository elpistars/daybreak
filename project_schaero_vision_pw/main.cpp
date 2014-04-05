/**	Program : El-pistolero Vision on goals detection
*	Author	: Arief Naibaho
*	Date	: 16 March 2014
*	Note	: This program created for participating in KRSBI 2014
*=====================Bismillahirrahmaannirrohim..======================*/

//Default setting
#define DEFAULT_PORTNUM			5 //COM3
#define DEFAULT_BAUDNUM			1 //1Mbps
#define NUM_ACTUATOR			2 // Number of actuator
#define STEP_THETA		(PI / 100.0f) // Large value is more fast
#define CONTROL_PERIOD	(10) // msec (Large value is more slow)

#include "elpistoleroVisionLib.h"
#include "dynamixel.h"

#pragma comment(lib, "dynamixel.lib")


/*
GAWANG:
#1 UTAMA	#2			#3
7-60		18-63		22-34
91-229		101-219		83-201
63-256		123-255		131-179

BOLA:
#1
6-14
112-256
98/187-256
*/

//initial min and max HSV filter values.
//these will be changed using trackbars
//RANGE HSV GAWANG
int lowerH_G=18;		int upperH_G = 63;
int lowerS_G=101;		int upperS_G = 219;
int lowerV_G=123;		int upperV_G = 255;

//RANGE HSV BOLA
int lowerH_B=6;			int upperH_B = 14;
int lowerS_B=112;		int upperS_B = 256;
int lowerV_B=187;		int upperV_B = 256;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

const string trackbarWindowName = "Trackbars";

/// Global variables
Mat src, src_gray,h_threshold;
int maxCorners = 10;
int maxTrackbar = 25;
int tx,ty,pos_goals_x, pos_goals_y, pos_ball_x, pos_ball_y;
int angguk_lokasi, toleh_lokasi;
int serX,serY,statX,statY,ii,x,y,lastX,lastY;
bool scan_bola, scan_gawang, scan,track;
RNG rng(12345);
char* source_window = "Image";

void act (int a) 
{
switch (a)
	{
		case 1 : printf ("hadap kiri"); break;
		case 2 : printf ("hadap kanan"); break;
		case 3 : printf ("geser kiri"); break;
		case 4 : printf ("geser kanan"); break;
		case 5 : printf ("maju"); break;
		case 6 : printf ("tendang kiri"); break;
		case 7 : printf ("tendang kanan"); break;
		case 10: printf ("sprint"); break;
		case 11: printf ("maju omni kanan"); break;
		case 12: printf ("maju omni kiri"); break;
		case 20: printf ("tangkap depan"); break;
		case 21: printf ("tangkap kiri"); break;
		case 22: printf ("tangkap kanan"); break;
		case 23: printf ("bangun depan"); break;
		case 24: printf ("bangun belakang"); break;
		case 25: printf ("scan bola"); scan_bola=true; scan_gawang=false; break;
		case 26: printf ("scan gawang"); scan_gawang=true; scan_bola=false; break;
	}
}

/// Function header
void gftt( int, void* );
/** @function main */
int main( int argc, char** argv )
{
	/*==================================== Local Variabel =====================================*/
	
	VideoCapture capture;

	/*=========================== Inisialisasi Nilai Awal Variabel ============================*/
	int key = 0;
	capture.open(0); // load image from webcam
	angguk_lokasi=510;
	toleh_lokasi=205;
	scan_bola = true;
	scan_gawang = false;
	scan = false;
	
	/*if(dxl_initialize(DEFAULT_PORTNUM, DEFAULT_BAUDNUM) == 0)
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press any key to terminate...\n" );
		return 0;
	}else{
		printf(" Succeed to open USB2Dynamixel!\n");
	}*/
	namedWindow( source_window, CV_WINDOW_AUTOSIZE );
	while(1)
		{	
			if(!capture.isOpened()){
				printf("Capture failure\n");
				return -1;
			}
			capture.read(src);
			cvtColor(src,src_gray,COLOR_BGR2HSV);
			if(scan_bola)
			{
				inRange(src_gray,Scalar(lowerH_B,lowerS_B,lowerV_B), Scalar(upperH_B,upperS_B,upperV_B),h_threshold);	
			}
			else if (scan_gawang)
			{
				inRange(src_gray,Scalar(lowerH_G,lowerS_G,lowerV_G),Scalar(upperH_G,upperS_G,upperV_G),h_threshold);
			}
			
			
			morphOps(h_threshold);
			
			
			if (scan_bola) 
			{
				scan_gawang=false;
				trackBall(serX,serY,lastX,lastY,temp,dcs, ball, h_threshold);
				if (ball) scan_ball (serX, serY, statX, ii); act(dcs);
			} 
			
			if (scan_gawang)
			{
				if(scan)
					{scan_goals(toleh_lokasi, angguk_lokasi);}
				gftt( 0, 0 );
				cout<<"Jarak = "<<getDistance(angguk_lokasi)<<endl;				
			}
//			createTrackbar( "Max  corners:", source_window, &maxCorners, maxTrackbar, gftt);
	//		imshow(source_window, src);
//			imshow("hasil threshold",h_threshold);
//			Sleep(100);
//			key = waitKey(3);
		}
		cvDestroyAllWindows();
		return(0);
}

/**
 * @function goodFeaturesToTrack.cpp
 * @brief Apply Shi-Tomasi corner detector
 */
void gftt( int, void* )
{
	if( maxCorners < 1 ) { maxCorners = 1; }

	/// Parameters for Shi-Tomasi algorithm
	vector<Point2f> corners;
	double qualityLevel = 0.1; //0.01
	double minDistance = 40;//10
	int blockSize = 3;
	bool useHarrisDetector = false;
	double k = 0.04;
	float max_x, min_x, max_y, min_y;
	double delta,delta_x,delta_y;
	bool gawang;

	/// Copy the source image
	Mat copy;
	copy = src.clone();

	/// Apply corner detection
	goodFeaturesToTrack( h_threshold,corners,maxCorners,qualityLevel,minDistance,Mat(),blockSize,useHarrisDetector,k );

	max_x=0;
	max_y=0;
	min_x=2600;
	min_y=2600;

	/// Write them down
	for( int i = 0; i < corners.size(); i++ )
    { 
		//cout<<" -- Refined Corner ["<<i<<"]  ("<<corners[i].x<<","<<corners[i].y<<")"<<endl;
		
		 //hitung nilai max_x,min_y,max_y,y1
		 if(corners[i].x>=max_x){max_x=corners[i].x;}
		 if(corners[i].x<=min_x){min_x=corners[i].x;}
		 if(corners[i].y>=max_y){max_y=corners[i].y;}
		 if(corners[i].y<=min_y){min_y=corners[i].y;}
	}
   
	//nilai tengah
	tx =min_x+((max_x-min_x)/2);
	ty =max_y;

	delta_x=max_x-min_x;
	delta_y=max_y-min_y;	//max_y=piksel bawah, min_y=piksel bawah
	delta=delta_x/delta_y;
  
	if(tx!=0 && ty!=0)
	{
		gawang=true;
	}
	else
	{
	  gawang=false;
	  cout << "TEU AYA " << endl;
	  scan_gawang=true;
	  scan_bola=false;
	  scan=true;
	} 
	cout << "titik tengah = "<<Point2d(tx,ty) << endl;
	//cout << "delta	    = "<< delta << endl;
  
	//tandain titik tengah gawang!!
	circle( copy, Point2f( tx, ty ), 5,  Scalar(0), 2, 8, 0 );
	if(gawang)
	{	
		if((delta<0.8) || (delta>-0.8))
		{
			cout << "BUKAN GAWANG!!" << endl;
			scan_gawang=true;
			scan_bola=false;
			scan=true;
		}
		else
		{
		  ///////////////////////////////////////////
		  //if(tx<320) maka ke kanan, selebihnya ke kiri
		  //if(ty<240) maka ke bawah, selebihnya ke atas
			cout << "GAWANG" << endl;
			scan_gawang=true;
			scan_bola=false; 
			scan=false;
			track=true;
			if(track)
			{
				trackGoals(tx,ty,toleh_lokasi,angguk_lokasi);   
			}
			else 
			{
				track=false;
			}
			cout<<"angguk lokasi = "<<angguk_lokasi<<endl;
			cout<<endl;
			
			dxl_write_word(31,30,toleh_lokasi);
			dxl_write_word(32,30,angguk_lokasi);

			
			//cout<<"Jarak = "<<getDistance(angguk_lokasi)<<endl;
		}
	}
	else
	{
	   scan=true;
	   track=false;
	   scan_gawang=true;
	}
	/// Show what you got
	namedWindow( source_window, CV_WINDOW_AUTOSIZE );
	imshow( source_window, copy );
}

//http://docs.opencv.org/doc/tutorials/features2d/trackingmotion/corner_subpixeles/corner_subpixeles.html