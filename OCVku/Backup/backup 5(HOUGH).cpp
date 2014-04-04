/******************************************
Program New Object Tracking
*******************************************/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//GlobalVariable
Mat ColorDetect,HoughOut,Clone,ImgBuff;
Mat canny_output;
int treshold=50;
int treshold_max=255;
RNG rng(12345);

//ColDetection
IplImage* GetThresholdedImage(IplImage* imgHSV){
       IplImage* imgThresh=cvCreateImage(cvGetSize(imgHSV),IPL_DEPTH_8U, 1);
       cvInRangeS(imgHSV, cvScalar(10,100,220,0), cvScalar(100,200,255,0), imgThresh); //x x x  //50 x x
       return imgThresh;}

//Labeling (gk wajib)
void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
        int fontface = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 1;
        int thickness = 2;
        int baseline = 0;

        cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
        cv::Rect r = cv::boundingRect(contour);

        cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
        cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
        cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
}

//Edgedetection
void GetEdge(int, void* );

//ShapeRecognition
void ShapeRecognition(int, void* );

int main(){
      	CvCapture* capture =0;

      	capture = cvCaptureFromCAM(1);
      	if(!capture){
		printf("Capture failure\n");
		return -1;
      	}
	IplImage* frame=0;
      	cvNamedWindow("Input Camera",2);
      	cvNamedWindow("ColourTracked!",2);
      	cvNamedWindow("Smooth!",2);
      //iterate through each frames of the video
      while(true){

		frame = cvQueryFrame(capture);
            	if(!frame) break;

            	frame=cvCloneImage(frame);
				cvShowImage("Input Camera", frame);
            	
           	cvSmooth(frame, frame, CV_GAUSSIAN,3,3); //smooth the original image using Gaussian kernel
				
            	IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
            	cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
            	IplImage* imgThresh = GetThresholdedImage(imgHSV);

           	cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN,3,3); //smooth the binary image using Gaussian kernel
            	cvShowImage("ColourTracked!", imgThresh);
            	cvShowImage("Smooth!", frame);
		ColorDetect = imgThresh;
		
		std::vector<cv::Point> approx;
        Clone = frame;
			
		GetEdge( 0, 0 );
		
		ShapeRecognition( 0, 0 );
	   	//Clean up used images
	   	/*
            	cvReleaseImage(&imgHSV);
            	cvReleaseImage(&imgThresh);
            	cvReleaseImage(&frame);
         */   	
            	//Wait 50mS
            	int c = cvWaitKey(10);
            	//If 'ESC' is pressed, break the loop
            	if((char)c==27 ) break;
      	}
	  
      cvDestroyAllWindows() ;
      cvReleaseCapture(&capture);

return 0; //End Of File o_O
}


//Edgedetection codes
void GetEdge(int, void* )
{
  	vector<vector<Point> > contours;
  	vector<Vec4i> hierarchy;

  /// Detect edges using canny
	Canny( ColorDetect, canny_output, treshold, treshold_max, 3 ); //kernel=3
	
  //### Prosesnya gk usah 2x, ntar di persingkat
  /// Find contours
  	findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  /// Draw contours
  	Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
  	for( int i = 0; i< contours.size(); i++ )
     	{
       		Scalar color = Scalar( 255, 255, 255 );
       		drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     	}
	ImgBuff= drawing;
  /// Show in a window
  	namedWindow( "Edge!", 2 );
  	imshow( "Edge!", canny_output );
	namedWindow( "Contours!", 2 );
	imshow( "Contours!", drawing );
}

void ShapeRecognition(int, void* )
{
	namedWindow( "Gotcha!", 2 );
	
    Mat Grayimg;
    cvtColor(ImgBuff, Grayimg, CV_BGR2GRAY );
    GaussianBlur( Grayimg, Grayimg, Size(9, 9), 2, 2 );
    vector<Vec3f> circles;
	HoughCircles(Grayimg, circles, CV_HOUGH_GRADIENT, 1, 10,
                 100, 30, 30, 70 // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles
                 );
    
    for( size_t i = 0; i < circles.size(); i++ )
	{
		//Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		Vec3i c = circles[i];
		// circle center
		circle( Clone, Point(c[0], c[1]), 2, Scalar(0,255,0), -1, 8, 0 );
		// circle outline
		circle( Clone, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, 8, 0 );
	}
	imshow("Gotcha!", Clone);
	waitKey();
    
}
