/**********************************************************
***********************************************************/


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

using namespace std;
using namespace cv;
bool doTestSpeedOnly=false;


//range HSV GAWANG
int lowerH_G=22;	int upperH_G=34;
int lowerS_G=83;	int upperS_G=201;
int lowerV_G=131;	int upperV_G=179;

//range HSV BOLA
int lowerH_B=6;		int upperH_B=14;
int lowerS_B=112;	int upperS_B=256;
int lowerV_B=187;	int upperV_B=256;

//range HSV pw
int lowerH_P=6;		int upperH_P=14;
int lowerS_P=137;	int upperS_P=225;
int lowerV_P=227;	int upperV_P=255;


//Global variables
Mat src, src_hsv, h_threshold;
int maxCorners=10;
int maxTrackbar=25;
bool scan_bola, scan_gawang;
RNG rng(12345);
char* source_window = "Image";

int serX,serY,statX,statY,posX,posY,ii;

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
//      cout<<"[-ex    exposure_mode (
}

//function header
void goodFeaturesToTrack_Demo(int, void*);
void morphOps(Mat &thresh);
void trackFilteredObject(int &x, int &y, Mat threshold);
void trackFilteredObject2(int &x, int &y, Mat threshold);
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
    cout<<"Connecting to camera"<<endl;
    if ( !camera.open() ) {
        cerr<<"Error opening camera"<<endl;
        return -1;
    }
    cout<<"Connected to camera ="<<camera.getId() <<endl;

    Mat image,img;
    int nCount=100;
    int key = 0;
    scan_bola=true;
    scan_gawang=false;
    while (1)
    {
	camera.grab();
	//camera >> image;
	camera.retrieve(image);
    	//Convert image source to HSV color space
    	cvtColor(image, src_hsv, COLOR_BGR2HSV);
	if(scan_bola)
	{
		inRange(src_hsv, Scalar(lowerH_P, lowerS_P, lowerV_P), Scalar(upperH_P, upperS_P, upperV_P),h_threshold);
		scan_bola=true;
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
	  trackFilteredObject2(posX,posY,h_threshold);

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

	key=waitKey(3);
	//imwrite( fn.str(),image);
    }
	camera.release();
}

/*scan -> dapat bola -> cari gawang -> tendang*/

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
/*
	//Draw corners detected
	cout<<"** Number of corners detected: "<<corners.size()<<endl;
	int r = 4;
	for(int i=0; i<corners.size(); i++)
	{ circle(copy, corners[i], r, Scalar(rng.uniform(0,255),rng.uniform(0,255),rng.uniform(0,255)), -1, 8, 0);}
*/
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

/*
	//tampilkan nilai max dan min
	cout << "max_x = "<<max_x<<endl;
	cout << "min_x = "<<min_x<<endl;
	cout << "max_y = "<<max_y<<endl;
	cout << "min_y = "<<  y1 <<endl;
*/

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
			cout<<"GOALPOST"<<endl;
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
void trackFilteredObject(int &x, int &y, Mat threshold){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	cout<<"track ";
	bool objectFound = false;
			for (int index = 0; index >= 0; index = index+1) {
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;
		                if(area>150){
					x = moment.m10/area;
					y = moment.m01/area;
					objectFound = true;
				}else objectFound = false;
				}
			//let user know you found an object
			if(objectFound ==true){cout<<"found"<<endl;} else
			{cout<<"not found"<<endl;}

	 usleep (10);
}

void trackFilteredObject2(int &x, int &y, Mat threshold){

	Mat temp;
	threshold.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
//	findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
	//use moments method to find our filtered object
	double refArea = 0;
	bool objectFound = false;
	cout<<"..";
	imwrite ("tes.ppm",temp);
	sleep (1); cout<<"done";
	scan_bola=false;
}


