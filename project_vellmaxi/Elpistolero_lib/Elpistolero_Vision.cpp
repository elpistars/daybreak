//#include "stdafx.h"
//#include "Elpistolero_DXL.cpp"
#include <time.h>
#include <stdio.h>
#include <string.h>
//#include <cv.h>
//#include <highgui.h>
//#include <cxcore.h>
//#include <cvaux.h>
#include <unistd.h>
#include <termio.h>
#include <termios.h>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/opencv.hpp"

int spy,spx,xkiri,xkanan,xtengah,jauh,dekat;

int posX,posY,tempx,tempy,xx,yy;
double area;
IplImage* imgTracking;
int lastX = -1;
int lastY = -1;
char buff[33];
//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV){       
    IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
    //cvInRangeS(imgHSV, cvScalar(160,160,60), cvScalar(179,256,256), imgThresh); 
	//cvInRangeS(imgHSV, cvScalar(15,0,0), cvScalar(22,256,256), imgThresh); 
	cvInRangeS(imgHSV, cvScalar(0,63,163), cvScalar(36,234,256), imgThresh); 
    return imgThresh;
}

void trackObject(IplImage* imgThresh){
// Calculate the moments of 'imgThresh'
CvMoments *moments = (CvMoments*)malloc(sizeof(CvMoments));
cvMoments(imgThresh, moments, 1);
double moment10 = cvGetSpatialMoment(moments, 1, 0);
double moment01 = cvGetSpatialMoment(moments, 0, 1);
area = cvGetCentralMoment(moments, 0, 0);

     // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
if(area>1000){
        // calculate the position of the ball
posX = moment10/area;
posY = moment01/area;  
        
       if(lastX>=0 && lastY>=0 && posX>=0 && posY>=0)
{
// Draw a yellow line from the previous point to the current point
cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY), cvScalar(0,0,255), 4);
}

 //printf("Ball! x=%d y=%d \n\r",posX,posY);
lastX = posX;
lastY = posY;
}

free(moments);
}

int getXY(){
  
      CvCapture* capture =0;       
      capture = cvCaptureFromCAM(0);
      if(!capture){
		printf("Capture failure\n");
		return -1;
      }
      
      IplImage* frame=0;
      frame = cvQueryFrame(capture);           
      if(!frame) return -1;
  
     //create a blank image and assigned to 'imgTracking' which has the same size of original video
     imgTracking=cvCreateImage(cvGetSize(frame),IPL_DEPTH_8U, 3);
     cvZero(imgTracking); //covert the image, 'imgTracking' to black

     //cvNamedWindow("Video");     
     //cvNamedWindow("Ball");

      //iterate through each frames of the video     
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
		  //kepala(posX,posY);
            // Add the tracking image and the frame
          cvAdd(frame, imgTracking, frame);

          //cvShowImage("Ball", imgThresh);           
           //cvShowImage("Video", frame);
           
           //Clean up used images
           cvReleaseImage(&imgHSV);
           cvReleaseImage(&imgThresh);            
           cvReleaseImage(&frame);

            //Wait 10mS
            int c = cvWaitKey(10);
            //If 'ESC' is pressed, break the loop
            if((char)c==27 ) break;  
			return (posX,posY);
      //cvDestroyAllWindows() ;
      //cvReleaseImage(&imgTracking);
      //cvReleaseCapture(&capture);     

      return 0;
}
