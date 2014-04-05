#include <iostream>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <raspicam_cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

#define MAX_NUM_OBJECTS =50;
#define MAX_OBJECT_AREA =150000;
#define MIN_OBJECT_AREA =70;


using namespace std;
using namespace cv;

//Global variables
Mat src, src_hsv, h_threshold;
int maxCorners=10;
int maxTrackbar=25;
bool scan_bola, scan_gawang,pw;
RNG rng(12345);
char* source_window = "Image";

bool doTestSpeedOnly=false;

//range HSV GAWANG
int lowerH_G=22;	int upperH_G=34;
int lowerS_G=83;	int upperS_G=201;
int lowerV_G=131;	int upperV_G=179;

//range HSV BOLA
int lowerH_B=6;		int upperH_B=14;
int lowerS_B=112;	int upperS_B=256;
int lowerV_B=187;	int upperV_B=256;

//range HSV bola-pw
int lowerH_P=6; 	int upperH_P=14;
int lowerS_P=137;	int upperS_P=255;
int lowerV_P=227;	int upperV_P=255;

int serX, serY, posX, posY;
int ii,statX,statY,temp;
int lastX=-1; int lastY=-1;
double area;


//pw
void scanpw ();
void trackobject();

//pw
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
    Camera.set ( CV_CAP_PROP_FRAME_HEIGHT, getParamVal ( "-h",argc,argv,180 ) );
    Camera.set ( CV_CAP_PROP_BRIGHTNESS,getParamVal ( "-br",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_CONTRAST ,getParamVal ( "-co",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_SATURATION, getParamVal ( "-sa",argc,argv,50 ) );
    Camera.set ( CV_CAP_PROP_GAIN, getParamVal ( "-g",argc,argv ,50 ) );
    if ( findParam ( "-gr",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_FORMAT, CV_8UC1 );
    if ( findParam ( "-test_speed",argc,argv ) !=-1 )
        doTestSpeedOnly=true;
    if ( findParam ( "-ss",argc,argv ) !=-1 )
        Camera.set ( CV_CAP_PROP_EXPOSURE, getParamVal ( "-ss",argc,argv )  );

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
//      cout<<"[-ex    exposure_mode (
}

//function header
void goodFeaturesToTrack_Demo(int, void*);
void morphOps(Mat &thresh);
//void cari_bola();


int main ( int argc,char **argv ) {
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
    cout<<"CONNECTED!"<<endl;

    Mat image,img;
    

    ii=0; serX=512; serY=490;
    statX= 10; statY=10; 
    scan_bola=true;
    scan_gawang=false;
    while (1)
    {
	camera.grab();
	camera.retrieve(image);
        inRange(src_hsv, Scalar(lowerH_P, lowerS_P, lowerV_P), Scalar(upperH_P, upperS_P, upperV_P),h_threshold);


    	//Convert image source to HSV color space
    	cvtColor(image, src_hsv, COLOR_BGR2HSV);
	if(scan_bola)
	{
		inRange(src_hsv, Scalar(lowerH_B, lowerS_B, lowerV_B), Scalar(upperH_B, upperS_B, upperV_B),h_threshold);
		scan_bola=true; trackobject();
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
		//fn<<"scan_bola.ppm";
		//scan_gawang=true;
		//scan_bola=false;
		scan_bola=true;
	}
	if(scan_gawang)
	{
		goodFeaturesToTrack_Demo(0,0);
		//cout<<"scan_gawang"<<endl;
		//fn<<"scan_gawang.ppm";
		scan_gawang=true;
	}

	if(!scan_gawang && !scan_bola)
	{
		cout<<"tendang!!"<<endl;
	}

	usleep (30);
	//imwrite( fn.str(),image);
    }
camera.release();
}

/*scan -> dapat bola -> cari gawang -> tendang*/



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
	int max_x, min_x, max_y, y1, tx, ty;
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
//corner subpix	

	max_x=0;
	max_y=0;
	min_x=2600;
	y1=0;

	//write them down
	for(int i=0; i<corners.size(); i++)
	{
		//cout<<" -- Refined Corner ["<<i<<"]("<<corners[i].x","<<corners[i].y<<")"<<endl;
		//hitung nilai max_x, min_y, max_y, y1
		if(corners[i].x>=max_x){max_x = corners[i].x; if(corners[i].y>=y1){y1 = corners[i].y;}};
		if(corners[i].x<=min_x){min_x = corners[i].x;}
		if(corners[i].x>=max_y){max_y = corners[i].y;}
	}


	//nilai tengah
	tx=min_x+((max_x-min_x)/2);
	//ty=y1+((max_y-y1)/2);
	ty=y1;
	delta_x = max_x - min_x;
	delta_y = max_y - y1;
	delta = delta_x/delta_y;	//lebar gawang dibandingkan dengan tinggi gawang

	if(tx!=0 && ty!=0)
	{
		gawang = true;
	}
	else
	{
		gawang = false;
		cout << "NOT FOUND" <<endl;
		scan_gawang = true;
		scan_bola = false;
	}

	cout << "titik tengah = "<<Point(tx,ty)<<endl;
	cout << "delta	      = "<<delta<<endl;

	//tandain titik tengah gawang!!
	circle( copy, Point(tx,ty), 5, Scalar(0), 2, 8, 0);

	if(gawang)
	{
		if((delta<1)&&(delta>-1))
		{
			cout<<"BUKAN GAWANG!!"<<endl;
			scan_gawang = true;
			//scan_bola = false;
		}
		else
		{
			cout<<"FOUND"<<endl;
			scan_gawang = true;
			//scan_bola = false;
		}
	
		//namedWindow( source_window, CV_WINDOW_AUTOSIZE);
//		imshow( source_window, copy);
	}
}
//tes
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

//http://docs.opencv.org/doc/tutorials/features2d/trackingmotion/corner_subpixeles/corner_subpixeles.html
/***********************************************************************************************/
void scanpw ()
{
//speed
//dxl_write_word( 31, 32, 150);
//dxl_write_word( 32, 32, 150);
if (ii<5)
	{serX=serX+statX;
	 if (serX<282) {serX=282; statX=statX*-1; ii=ii+1;
	 	if (statX>0) {serY=430;} else {serY=350;}} else 
	 if (serX>742) {serX=742; statX=statX*-1; ii=ii+1;
		if (statX>0) {serY=430;} else {serY=350;}}}
	else {serX=512; serY= 390; ii=0;}

//	dxl_write_word( 32, P_GOAL_POSITION_L, serY);
//	dxl_write_word( 31, P_GOAL_POSITION_L, serX);

	usleep (100);
	cout << "x = "<< serX;
	cout << "   y = "<<serY<<endl;

//printf ("x=%d    y=%d   ii=%d \n",serX,serY,ii);
}

void trackobject(){

//	Mat temp;
//	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
//	vector< vector<Point> > contours;
//	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
//	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	cout<<"lalala"<<endl;
	//use moments method to find our filtered object
/*	bool objectFound = false;
	if (hierarchy.size() > 0) {
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

               		if (area>70){posX=moment.m10/area;
				     posY=moment.m01/area;
				     objectFound = true;
	                       	    }
			else objectFound = false;
			}
			if(objectFound ==true){
				cout << "bola found" <<endl;} else
			if(objectFound ==false) {
				cout << "scan bola" << endl;}
*/
//	}
}

