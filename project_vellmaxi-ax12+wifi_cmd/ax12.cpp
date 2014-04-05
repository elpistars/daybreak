#include "Elpistolero_lib/Elpistolero_DXL.cpp"
#include "Elpistolero_lib/Elpistolero_Basic.cpp"
#include "Elpistolero_lib/Elpistolero_Soccer.cpp"
//#include "Elpistolero_lib/Elpistolero_Vision.cpp"

int main(void)
{
	ELP_initialize(0,1); //PORT=ttyUSB0 Baud=1Mbps
	
	sleep(5);
	ELP_kill();
}
