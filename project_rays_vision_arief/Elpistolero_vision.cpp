#include "elpistoleroVisionLib.h"

/**
 * @This function threshold the HSV image and create a binary image
 */
Mat GetThresholdedImage(Mat &imgHSV,int &lowerH, int &upperH, int &lowerS, int &upperS, int &lowerV, int &upperV)
{
	Mat imgThresh= Mat(imgHSV.rows, imgHSV.cols, CV_8UC1);
	inRange(imgHSV, Scalar(lowerH,lowerS,lowerV), Scalar(upperH,upperS,upperV), imgThresh);

	return imgThresh;
}
/**
 * @filter image with dilate and erode filtering
 */ 
void morphOps(Mat &thresh)
{

	//create structuring element that will be used to "dilate" and "erode" image.
	//the element chosen here is a 3px by 3px rectangle

	Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
	Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

	erode(thresh,thresh,erodeElement);
	erode(thresh,thresh,erodeElement);


	dilate(thresh,thresh,dilateElement);
	dilate(thresh,thresh,dilateElement);
}

/**
 * @track center point of goals post using PID
 */ 
void trackGoals(int &c_pointX, int &c_pointY,int &scan_x, int &scan_y)
{
	int mid_x = 320;//320
	int mid_y = 240;//240
	float P_x, P_y, Kp_x, Kd_x, Kp_y, Kd_y,rate_y, rate_x, D_x, D_y, MV_x,MV_y;
	float error_x,error_y, last_errX, last_errY;
	bool track;
	//last_errX=0;
	//last_errY=0;
	Kp_x = 0.03125;		//0.03125=10; 0.015625=5	PENTING!!!
	Kd_x = 0.02;		//0.0078125 ,,0.00390625	PENTING!!!
	Kp_y = 0.041667;	//	PENTING!!!
	Kd_y = 0.020833;	//0.020833,, 0.013333		PENTING!!!
	if(c_pointX !=mid_x)
	{
		last_errX = 0;
		error_x = mid_x - c_pointX;
		P_x = (Kp_x * error_x);			
		rate_x = error_x - last_errX;
		D_x = (rate_x * Kd_x);
		MV_x = P_x + D_x;
		last_errX = error_x;
		scan_x = scan_x + MV_x;
		//buff_x = (scan_x);
		track=true;
					
		/*cout<<"error_x	= "<<error_x<<endl;
		cout<<"P_x		= "<<P_x<<endl;
		cout<<"rate_x	= "<<rate_x<<endl;
		cout<<"D_x		= "<<D_x<<endl;
		cout<<"MV_x		= "<<MV_x<<endl;
		cout<<"last_erx = "<<last_error_x<<endl;*/
	}
	else 
	{
	   track=false;
	}

	if (scan_x > 819) {scan_x = 819;} //looking to left ID:31
	if (scan_x < 205) {scan_x = 205;} //looking to right ID:31
	if(c_pointY !=mid_y)
	{
		last_errY = 0;
		error_y = mid_y - c_pointY;
		P_y = (Kp_y * error_y);

		rate_y = error_y - last_errY;
		D_y = (rate_y * Kd_y);

		MV_y = P_y + D_y;
		last_errY = error_y;

		scan_y = scan_y + MV_y;
		//buff_y = (scan_y);
		track=true;
		
	}else{ track = false;}
	if(scan_y > 510) {scan_y = 510;} //atas
	if(scan_y < 205) {scan_y = 205;} //bawah
	/*
	cout<<"error_y	= "<<error_y<<endl;
	cout<<"P_y		= "<<P_y<<endl;
	cout<<"rate_y	= "<<rate_y<<endl;
	cout<<"D_y		= "<<D_y<<endl;
	cout<<"MV_y		= "<<MV_y<<endl;
	cout<<"last_ery = "<<last_error_y<<endl;*/
}

/**
 * @Scan goals post
 */ 
void scan_goals(int pos_servX, int pos_servY)
{
	pos_servX +=14;//72
	Sleep(18);//90
		
	if(pos_servX >= 819)
	{
		pos_servX -=14; //kiri-kanan: 872-154
	}
	//cout<<"toleh_lokasi = "<<toleh_lokasi<<endl;
	dxl_write_word(31,30,pos_servX);
	dxl_write_word(32,30,pos_servY); //atas-bawah: 510-205
}

/**
 * @measure distance from cam to goals post
 */ 
unsigned int getDistance(int &pos_servY)
{ 
	float param; 
	/*if (pos_servY >= 510)
	{
		param = 80;
		return 35*tan(param*PI/180.0);
	}else if (pos_servY <=103)
	{
		param = 1;
		return 35*tan(param*PI/180.0);
	}else{*/
	param = (pos_servY-205)*0.293;
	return 35*tanf( param * PI / 180.0 ); //pos_servY*(300/1023)
}
/**
 * @mapping camera track ball
 */
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
/**
 * @scan ball position
 */ 
void scan_ball (int serX, int serY, int statX, int ii) 
{	
	int dcs;
	dxl_write_word( 31, 32, 150);
	dxl_write_word( 32, 32, 150);
	if (ii<5)
	{serX=serX+statX; 
	if (serX<382) {serX=382; statX=statX*-1; ii=ii+1;
	if (statX>0) {serY=430;} else {serY=350;}} else 
	if (serX>642) {serX=642; statX=statX*-1; ii=ii+1;
	if (statX>0) {serY=430;} else {serY=350;}}}
	else {
		serX=512; 
		serY= 390; ii=0; dcs=2;}

	dxl_write_word( 32, P_GOAL_POSITION_L, serY);
	dxl_write_word( 31, P_GOAL_POSITION_L, serX); 

	Sleep (50);
}
/**
 * @Tracking ball position
 */
void trackBall(int &x, int &y,int serX, int serY, int lastX, int lastY, Mat imgThresh)
{
	//max number of objects to be detected in frame
	const int MAX_NUM_OBJECTS=50;
	//minimum and maximum object area
	const int MIN_OBJECT_AREA = 10*10;
	const int MAX_OBJECT_AREA = 480*640/1.5;
	double refArea = 0;
	Mat imgTracking;
	bool objectFound = false;
	bool ball;
	int temp, dcs, statX, statY, a, ii;
	

	// Calculate the moments of 'imgThresh'
	
	Moments moment = moments(imgThresh); //We give the binary converted frames for calculating the moments
	
	double area = moment.m00; //Sum of all white color pixels
	
    // if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
	if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea)
	{ 
		// calculate the position of the ball
		x = moment.m10/area;
		y = moment.m01/area;  
		objectFound = true;
		refArea = area;
		if (pixelmap(x,y) != 0)
		temp = pixelmap(x,y);
		if(lastX>=0 && lastY>=0 && x>=0 && y>=0)
		{
			// Draw a yellow line from the previous point to the current point
			line(imgTracking, Point(x, y), Point(lastX, lastY), Scalar(0,0,255), 4);
		}
		//printf("Ball! x=%d y=%d \n\r",posX,posY);
		switch (temp) 
		{
			case 11 : statX=20; statY=15; break; 
			case 12 : statX=0; statY=15; break;
			case 13	: statX=-20; statY=15; break;
			case 21 : statX=20; statY=0; break;
			case 22 : {statX=0; statY=0; if (serY>260){ 
						if (serX<482) {dcs=2;} 
						else if (serX>542) {dcs=1;} 
						else {dcs=5;}} 
						else dcs=6; break;}
			case 23 : statX=-20; statY=0; break;
			case 31 : statX=20; statY=-15; break;
			case 32 : statX=0; statY=-15; break;
			case 33 : statX=-20; statY=-15; break;
		}

		serY=serY+statY; 
		serX=serX+statX;
		if (serX<382) {serX=382; dcs=2;} 
		else if (serX>642) {serX=642; dcs=1;}
		if (serY>510) {serY=510;} 
		else if (serY<259) 
		{
			serY=259; 
			if (serX<482) {dcs=2;} 
			else if (serX>542) {dcs=1;} 
			else dcs=6;
		}
		dxl_write_word( 32, 30, serY);
		dxl_write_word( 31, 30, serX);
		ball=false;
		lastX = x; 
		lastY = y; 
		a=0; 
		ii=0;
	} else 
	ball=true; 
	free(moments);
}
/**
 * @create trackbars for tuning HSV color space
 */
void createTrackbars(int &lowerH, int &upperH, int &lowerS, int &upperS, int &lowerV, int &upperV)
{
	const string trackbarWindowName = "Trackbars";
	//create window for trackbars
	namedWindow(trackbarWindowName,0);
	//create memory to store trackbar name on window
	char TrackbarName[50];
	sprintf( TrackbarName, "H_MIN", lowerH);
	sprintf( TrackbarName, "H_MAX", upperH);
	sprintf( TrackbarName, "S_MIN", lowerS);
	sprintf( TrackbarName, "S_MAX", upperS);
	sprintf( TrackbarName, "V_MIN", lowerV);
	sprintf( TrackbarName, "V_MAX", upperV);
	//create trackbars and insert them into window
	//3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),

	createTrackbar( "H_MIN", trackbarWindowName, &lowerH, upperH, on_trackbar );
	createTrackbar( "H_MAX", trackbarWindowName, &upperH, upperH, on_trackbar );
	createTrackbar( "S_MIN", trackbarWindowName, &lowerS, upperS, on_trackbar );
	createTrackbar( "S_MAX", trackbarWindowName, &upperS, upperS, on_trackbar );
	createTrackbar( "V_MIN", trackbarWindowName, &lowerV, upperV, on_trackbar );
	createTrackbar( "V_MAX", trackbarWindowName, &upperV, upperV, on_trackbar );
}
void on_trackbar( int, void* )
{
	//This function gets called whenever a
	// trackbar position is changed
}
