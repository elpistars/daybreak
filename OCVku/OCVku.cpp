
#include <stdio.h>
#include <iostream>
#include <termio.h>
#include <termios.h>
#include <unistd.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"
#include <time.h>
#include "dynamixel.h"

// Control table address
#define P_TORQUE_ENABLE			24
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46
#define P_GOAL_SPEED_L			32

// Defulat setting
#define DEFAULT_PORTNUM		1 // COM
#define DEFAULT_BAUDNUM		1 // 1Mbps
#define DEFAULT_ID			31

double area;
int posX,posY,a; //posisi
int speedx,speedy;
IplImage* imgTracking;
int lastX = -1;
int lastY = -1;
int serX;
int serY=490;
int speed,temp,ii;
int intX,intY,statX,statY,xx,yy;
unsigned int lastpost;
int kepalaX[5] = {256,384,512,640,768};
int kepalaY[3] = {490,360,230};
int dcs;
bool ball;
//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV){       
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    //cvInRangeS(imgHSV, cvScalar(160,160,60), cvScalar(179,256,256), imgThresh); 
	//cvInRangeS(imgHSV, cvScalar(15,0,0), cvScalar(22,256,256), imgThresh); 
	//cvInRangeS(imgHSV, cvScalar(0,63,163), cvScalar(36,234,256), imgThresh); 
    cvInRangeS(imgHSV, cvScalar(6,137,227), cvScalar(14,225,255), imgThresh); 
	//cvInRangeS(imgHSV, cvScalar(6,144,252), cvScalar(14,231,256), imgThresh); 
	//cvInRangeS(imgHSV, cvScalar(0,11,253), cvScalar(54,201,250), imgThresh); 
	
	return imgThresh;
}
//This function threshold the HSV image and create a binary image
int getch(void)
{
  int ch;
  struct termios oldt;
  struct termios newt;
  tcgetattr(STDIN_FILENO, &oldt); /*store old settings */
  newt = oldt; /* copy old settings to new settings */
  newt.c_lflag &= ~(ICANON | ECHO); /* make one change to old settings in new settings */
  tcsetattr(STDIN_FILENO, TCSANOW, &newt); /*apply the new settings immediatly */
  ch = getchar(); /* standard getchar call */
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt); /*reapply the old settings */
  return ch; /*return received char */
}

//buat mapping camera
int pixelmap (int x, int y)
{
	int matriksA;
	if (( x<=213)			  && (y<=160))              {matriksA = 11;} else 
	if (((x<=426) && (x>213)) && (y<=160))			    {matriksA = 12;} else 
	if (((x<=640) && (x>426)) && (y<=160))			    {matriksA = 13;} else 
	
	if (( x<=213)			  && ((y>160) && (y<=320))) {matriksA = 21;} else 
	if (((x<=426) && (x>213)) && ((y>160) && (y<=320))) {matriksA = 22;} else 
	if (((x<=640) && (x>426)) && ((y>160) && (y<=320))) {matriksA = 23;} else 
	
	if (( x<=213)    		  && ((y>320) && (y<=480))) {matriksA = 31;} else 
	if (((x<=426) && (x>213)) && ((y>320) && (y<=480))) {matriksA = 32;} else 
	if (((x<=640) && (x>426)) && ((y>320) && (y<=480))) {matriksA = 33;}  

	return (matriksA);
}

void act (int a) 
{
switch (a)
	{
		case 1 : printf ("hadap kiri"); break;
		case 2 : printf ("hadap kanan"); break;
		case 3 : printf ("geser kiri"); break;
		case 4 : printf ("geser kanan"); break;
		case 5 : printf ("maju"); break;
		case 6 : printf ("tendang"); break;
	}
}
void scan () 
{	

dxl_write_word( 31, 32, 150);
dxl_write_word( 32, 32, 150);
if (ii<5)
{serX=serX+statX; 
if (serX<382) {serX=382; statX=statX*-1; ii=ii+1;
	if (statX>0) {serY=430;} else {serY=350;}} else 
if (serX>642) {serX=642; statX=statX*-1; ii=ii+1;
	if (statX>0) {serY=430;} else {serY=350;}}}
else {serX=512; 
	  serY= 390; ii=0; dcs=2;}

	dxl_write_word( 32, P_GOAL_POSITION_L, serY);
	dxl_write_word( 31, P_GOAL_POSITION_L, serX); 

	sleep(1);}
	

void trackObject(IplImage* imgThresh){
// Calculate the moments of 'imgThresh'

CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
cvMoments(imgThresh, moments, 1);
double moment10 = cvGetSpatialMoment(moments, 1, 0);
double moment01 = cvGetSpatialMoment(moments, 0, 1);
area = cvGetCentralMoment(moments, 0, 0);

     // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
if(area>=100){ //aslinya area>1000
        // calculate the position of the ball

posX = moment10/area;
posY = moment01/area;  
	
//if (temp!=pixelmap(posX,posY) || a>1){	  
	//	printf ("matriks pixel = %d  x %d  y %d \n",pixelmap(posX,posY));} 
if (pixelmap(posX,posY) != 0)
	temp=pixelmap(posX,posY);
if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
{
// Draw a yellow line from the previous point to the current point
cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
}

 //printf("Ball! x=%d y=%d \n\r",posX,posY);
switch (temp) 
{
case 11 : statX=20; statY=15; break; 
case 12 : statX=0; statY=15; break;
case 13	: statX=-20; statY=15; break;
case 21 : statX=20; statY=0; break;
case 22 : {statX=0; statY=0; if (serY>260) { if (serX<482) {dcs=2;} else
									   if (serX>542) {dcs=1;} else 
									   {dcs=5;}} else dcs=6; break;}

case 23 : statX=-20; statY=0; break;
case 31 : statX=20; statY=-15; break;
case 32 : statX=0; statY=-15; break;
case 33 : statX=-20; statY=-15; break;
}

serY=serY+statY; serX=serX+statX;
if (serX<382) {serX=382; dcs=2;} else if (serX>642) {serX=642; dcs=1;}
if (serY>510) {serY=510;} else if (serY<259) {serY=259; 
									   if (serX<482) {dcs=2;} else
									   if (serX>542) {dcs=1;} else dcs=6;}
	  dxl_write_word( 32, 30, serY);
      dxl_write_word( 31, 30, serX);
ball=false;
lastX = posX; lastY = posY; a=0; ii=0;
} else 
	ball=true; free(moments);
}



//bikin garis-garis
void drawline (IplImage*a) 
{
cvLine(a,cvPoint(213,0),cvPoint(213,480),cvScalar (94,206,165,0),4,8,0);
cvLine(a,cvPoint(0,160),cvPoint(640,160),cvScalar (94,206,165,0),4,8,0);
cvLine(a,cvPoint(426,0),cvPoint(426,480),cvScalar (94,206,165,0),4,8,0);
cvLine(a,cvPoint(0,320),cvPoint(640,320),cvScalar (94,206,165,0),4,8,0);
}



int main(){
a=1; temp=0; intX=2, intY=1; statX=20;statY=0;
serX=512;
ball=true;
	  dxl_write_word( 32, 30, serY);
      dxl_write_word( 31, 30, serX);

if( dxl_initialize(DEFAULT_PORTNUM, DEFAULT_BAUDNUM) == 0)
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press any key to terminate...\n" );
		getch();
			
		return 0;
	}
	else{
		printf( "Succeed to open USB2Dynamixel!\n" ); 
		//dynamixel
		
		CvCapture* capture =0;       
      capture = cvCaptureFromCAM(1);
      if(!capture){
		printf("Capture failure\n");
		return -1;
      }
      //init
	  
      IplImage* frame=0;

      frame = cvQueryFrame(capture);           
      if(!frame)
      return -1;
  


     //create a blank image and assigned to 'imgTracking' which has the same size of original video
     imgTracking=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 3);
     cvZero(imgTracking); //covert the image, 'imgTracking' to black

     cvNamedWindow("Video");     
     cvNamedWindow("Ball");

      //iterate through each frames of the video     
      while(true){

            frame = cvQueryFrame(capture);           
            if(!frame) break;
            frame=cvCloneImage(frame); 
            
           cvSmooth(frame, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel
		   
	        IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
            cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
            IplImage* imgThresh = GetThresholdedImage(imgHSV);
          
		   	cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN,3,3); //smooth the binary image using Gaussian kernel
            
          //track the possition of the ball
          trackObject(imgThresh);

		  if (ball) {
dxl_write_word( 31, 32, 250);
dxl_write_word( 32, 32, 250);
if (ii<4)
{serX=serX+statX; 
if (serX<382) {serX=382; statX=statX*-1; ii=ii+1;
	if (statX>0) {serY=430+statY;} else {serY=350+statY;}} else 
if (serX>642) {serX=642; statX=statX*-1; ii=ii+1;
if (statX>0) {serY=430;} else {serY=350;}} dcs=0;}
else {serX=512; 
	  serY= 390; ii=0; statX=10; statY=10; dcs=1;}
if (statY==-10) {serY=serY-10; if (serY<=230) {serY=230; if (statX==0) {statX= 10; serY=390;}}} else
if (statY== 10) {serY=serY+10; if (serY>=510) {serY=510; if (statX==0) {statX=-10; serY=390;}}}
	dxl_write_word( 32, P_GOAL_POSITION_L, serY);
	dxl_write_word( 31, P_GOAL_POSITION_L, serX); 
	
}
		  act(dcs);
// Add the tracking image and the frame
          cvAdd(frame, imgTracking, frame);
		  drawline(frame);
           cvShowImage("Ball", imgThresh);           
           cvShowImage("Video", frame);
           
           //Clean up used images
           cvReleaseImage(&imgHSV);
           cvReleaseImage(&imgThresh);            
           cvReleaseImage(&frame);

            //Wait 10mS
            int c = cvWaitKey(10);
            //If 'ESC' is pressed, break the loop
            if((char)c==27 ) break;  
			
      }

      cvDestroyAllWindows() ;
      cvReleaseImage(&imgTracking);
      cvReleaseCapture(&capture);    

 dxl_write_word( 31, 32, 100);
 dxl_write_word( 32, 32, 100);

 dxl_write_word( 31, 30, 470);
 dxl_write_word( 32, 30, 512);

      return 0;
	}}
	
