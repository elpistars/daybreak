/******************************************
Program New Object Tracking
*******************************************/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace cv;
using namespace std;

//GlobalVariable
Mat ColorDetect,HoughOut,unBlur,Blur,Clone;
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
void getSURF(int, void* );

int main(){
      	CvCapture* capture =0;

      	capture = cvCaptureFromCAM(0);
      	if(!capture){
		printf("Capture failure\n");
		return -1;
      	}
	IplImage* frame=0;
      	//cvNamedWindow("Input Camera",2);
      while(true){

		frame = cvQueryFrame(capture);
            	if(!frame) break;

            	frame=cvCloneImage(frame);
				//cvShowImage("Input Camera", frame);
        Clone = frame;
			
		//GetEdge( 0, 0 );
		getSURF( 0, 0 );
            	cvReleaseImage(&frame);

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

  /// Show in a window
  	namedWindow( "Edge!", 2 );
  	imshow( "Edge!", canny_output );
	namedWindow( "Contours!", 2 );
	imshow( "Contours!", drawing );
}

void getSURF(int, void* )
{
  Mat img_object = imread( "bolaku.png", IMREAD_GRAYSCALE );
  Mat img_scene = Clone;
  cvtColor(img_scene, img_scene, CV_BGR2GRAY );
  
////-- Step 1: Detect the keypoints using SURF Detector --////////////
  int minHessian = 1500;

  SurfFeatureDetector detector( minHessian );

  std::vector<KeyPoint> keypoints_object, keypoints_scene;

  detector.detect( img_object, keypoints_object );
  detector.detect( img_scene, keypoints_scene );
/////////////////////////////////////////////////////////////////////

////-- Step 2: Calculate descriptors (feature vectors)
  SurfDescriptorExtractor extractor;

  Mat descriptors_object, descriptors_scene;

  extractor.compute( img_object, keypoints_object, descriptors_object );
  extractor.compute( img_scene, keypoints_scene, descriptors_scene );
/////////////////////////////////////////////////////////////////////

  //-- Step 3: Matching descriptor vectors using FLANN matcher
  FlannBasedMatcher matcher;
  std::vector< DMatch > matches;
  matcher.match( descriptors_object, descriptors_scene, matches );

  double max_dist = 0; double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ )
  { double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptors_object.rows; i++ )
  { if( matches[i].distance < 3*min_dist )
    { good_matches.push_back( matches[i]); }
  }

  Mat img_matches;
  drawMatches( img_object, keypoints_object, img_scene, keypoints_scene,
               good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


  //-- Localize the object from img_1 in img_2
  std::vector<Point2f> obj;
  std::vector<Point2f> scene;

  for( size_t i = 0; i < good_matches.size(); i++ )
  {
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
    scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
  }

  Mat H = findHomography( obj, scene, RANSAC );

  //-- Get the corners from the image_1 ( the object to be "detected" )
  std::vector<Point2f> obj_corners(4);
  obj_corners[0] = Point(0,0); obj_corners[1] = Point( img_object.cols, 0 );
  obj_corners[2] = Point( img_object.cols, img_object.rows ); obj_corners[3] = Point( 0, img_object.rows );
  std::vector<Point2f> scene_corners(4);

  perspectiveTransform( obj_corners, scene_corners, H);


  //-- Draw lines between the corners (the mapped object in the scene - image_2 )
  Point2f offset( (float)img_object.cols, 0);
  line( img_matches, scene_corners[0] + offset, scene_corners[1] + offset, Scalar(0, 255, 0), 4 );
  line( img_matches, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[2] + offset, scene_corners[3] + offset, Scalar( 0, 255, 0), 4 );
  line( img_matches, scene_corners[3] + offset, scene_corners[0] + offset, Scalar( 0, 255, 0), 4 );

  //-- Show detected matches
  imshow( "Good Matches & Object detection", img_matches );
}

