#include <stdio.h>   
#include <string.h>  
#include <unistd.h>  
#include <fcntl.h>   
#include <errno.h>   
#include <termios.h> 
#include <time.h>

int main(void)
{
	time_t rawtime;
    struct tm *info;
    FILE * fp;
    fp = fopen ("Radiation_Data.csv","w");
    int fd = open("/dev/ttyACM0", O_RDONLY | O_NOCTTY);
    if (fd == -1)
        printf("open_port: Unable to open /dev/ttyACM0");

    char buffer[50];
    int n;
    time(&rawtime);
    info = localtime(&rawtime);
    fprintf(fp,"dayofyear,hour,minute,uvindex,solarradiation,ir,temperature,pressure,humidity\n");
    fprintf(fp,"Initial timestamp: %d,%d,%d,%d\n",info->tm_yday + 1,info->tm_hour,info->tm_min,info->tm_sec);
    fflush(fp);
    while(1){
    	n = read(fd, buffer, sizeof(buffer));
	    if (n < 0)
	        fputs("read failed!\n", stderr);
	    buffer[n] = '\0';
	    fprintf(fp,"%s",buffer);
	    fflush(fp);
	}
}