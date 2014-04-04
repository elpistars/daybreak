#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
int main(int argc,char *argv[])
{
    int handle = socket( AF_INET, SOCK_DGRAM, 0 );
    if(handle < 0){
      perror("socket");
      exit(1);
   }

   struct sockaddr_in servaddr;
   servaddr.sin_family = AF_INET;
   servaddr.sin_port=htons(3838);
   servaddr.sin_addr.s_addr= INADDR_ANY;

    if ( bind( handle, (struct sockaddr*)&servaddr, sizeof(servaddr) ) < 0 ){
   perror("bind");
   exit(1);
   }

    while ( 1 )
    {
      struct sockaddr_in cliaddr;
      char packet_data[116];

      socklen_t len= sizeof(cliaddr);
      int received_bytes = recvfrom( handle, packet_data, sizeof(packet_data),0, (struct sockaddr*)&cliaddr, &len );
      	if ( received_bytes > 0 ){
	if (packet_data[9]==3)
      	//printf("%x\n",packet_data[9]);
	printf("Play");
	}
	//FILE * pFile;
  	//char buffer[] = { 'x' , 'y' , 'z' };
  	//pFile = fopen ("myfile.bin", "wb");
  	//fwrite (packet_data , sizeof(char), sizeof(packet_data), pFile);
  	//fclose (pFile);
     }

     close(handle);


     return 0;

}
