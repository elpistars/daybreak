#ifndef ELPDXL
#define ELPDXL

// Control table address


// Id of the currently connected dynamixel
#define NUM_SERVOS			18 // Number of actuators
int getch(void);
int id[NUM_SERVOS];
int CommStatus;
void PrintCommStatus(int CommStatus);
void PrintErrorCode(void);
void ELP_initialize(int portNum,int baudnum);
void ELP_kill();
void setAllSpeed( int speed);
void ELP_SyncWrite(int dataArray[]);
#endif
