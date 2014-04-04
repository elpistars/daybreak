#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <termios.h>
#include <time.h>

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

// Control table address
#define P_GOAL_POSITION_L 30
#define P_GOAL_POSITION_H 31
#define SPEED_L 32
#define SPEED_H 33
#define P_PRESENT_POSITION_L 36
#define P_PRESENT_POSITION_H 37
#define P_MOVING 46
#define PD 500



/** Motor IDs of all three motors motors */
#define MOTOR1      0
#define MOTOR2      1
//#define MOTOR3      15

// Id of the currently connected dynamixel
#define DEFAULT_BAUDNUM 1 // 1Mbps
#define DEFAULT_ID 0



void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);

void ELP_initialize(int portNum,int baudnum)
{
        /*
         * One digit of motor represents 0.3515625 degrees.
         * One degree represents 2.8444444 values of motor turn
         */
        int GoalPos[2] = {512, 512};
        /**
         * Middle Motor limits - [ -92.109, 101.25  ]
         */
        int GoalPosMotor2[2] = { 250, 800 };
        /**
         * Base Motor limits - [-180, 180 ];
         */
        int GoalPosMotor1[2] = { 0, 1024 };
        /**
         * Top Motor limits - [ 77.344, 125.859 ];
         */
        int index = 0;
        int deviceIndex = portNum; //port USBttyX
        int Moving, PresentPos;
        int CommStatus;

	/*int statX=20;
	int statY=10;
	int dcs;

	int serX = 512;
	int serY = 390;
	int ii=0;*/

        printf( "\n\nRead/Write example for Linux\n\n" );
        ///////// Open USB2Dynamixel ////////////
        if( dxl_initialize(deviceIndex, baudnum) == 0 )
        {
                printf( "Failed to open USB2Dynamixel!\n" );
                printf( "Press Enter key to terminate...\n" );
                getchar();
                //return 0;
        }
        else
                printf( "Succeed to open USB2Dynamixel!\n" );

}

/*void scan () 

{
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
}*/

void ELP_kill()
{
	    // Close device
        dxl_terminate();
        printf( "Press Enter key to terminate...\n" );
        getchar();
        //return 0;
}

/** Printing communication result */
void PrintCommStatus(int CommStatus)
{
        switch(CommStatus)
        {
        case COMM_TXFAIL:
                printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
                break;

        case COMM_TXERROR:
                printf("COMM_TXERROR: Incorrect instruction packet!\n");
                break;

        case COMM_RXFAIL:
                printf("COMM_RXFAIL: Failed get status packet from device!\n");
                break;

        case COMM_RXWAITING:
                printf("COMM_RXWAITING: Now recieving status packet!\n");
                break;

        case COMM_RXTIMEOUT:
                printf("COMM_RXTIMEOUT: There is no status packet!\n");
                break;

        case COMM_RXCORRUPT:
                printf("COMM_RXCORRUPT: Incorrect status packet!\n");
                break;

        default:
                printf("This is unknown error code!\n");
                break;
        }
}

/** Print error bit of status packet */
void PrintErrorCode()
{
        if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
                printf("Input voltage error!\n");

        if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
                printf("Angle limit error!\n");

        if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
                printf("Overheat error!\n");

        if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
                printf("Out of range error!\n");

        if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
                printf("Checksum error!\n");

        if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
                printf("Overload error!\n");

        if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
                printf("Instruction code error!\n");
}
