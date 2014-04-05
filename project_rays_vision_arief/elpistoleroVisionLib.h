#ifndef ELPISTOLEROVISIONLIB_H_INCLUDED
#define ELPISTOLEROVISIONLIB_H_INCLUDED

/** Dynamixel Header */

//Control table address
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31
#define P_GOAL_SPEED_L			32
#define P_GOAL_SPEED_H			33
#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37
#define P_MOVING				46
/*========================================*/
#define PI 3.14159265

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
#include <time.h>
#include <cmath>

#include "dynamixel.h"

#pragma comment(lib, "dynamixel.lib")

using namespace std;
using namespace cv;

/**
 * @This function threshold the HSV image and create a binary image
 */
Mat* GetThresholdedImage(Mat* &imgHSV);
/**
 * @filter image with dilate and erode filtering
 */ 
void morphOps(Mat &thresh);
/**
 * @filter image with dilate and erode filtering
 */ 
void cari_bola();
/**
 * @Drawing 1x3 line for goals mapping
 */ 
void DrawLine(Mat &frame);
/**
 * @track center point of goals post using PID
 */ 
void trackGoals(int &c_pointX, int &c_pointY,int &scan_x, int &scan_y);
/**
 * @Scan goals post
 */ 
void scan_goals(int pos_servX, int pos_servY);
/**
 * @measure distance from cam to goals post
 */ 
unsigned int getDistance(int &pos_servY);
/**
 * @mapping camera track ball
 */
int pixelmap (int x, int y);
/**
 * @scan ball position
 */ 
void scan_ball (int serX, int serY, int statX, int ii);
/**
 * @Tracking ball position
 */
void trackBall(int &x, int &y,int serX, int serY, int lastX, int lastY, Mat imgThresh);
/**
 * @create trackbars for tuning HSV color space
 */ 
void createTrackbars();

void on_trackbar( int, void* );

#endif // ELPISTOLEROVISIONLIB_H_INCLUDED
