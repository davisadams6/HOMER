#include <iostream>
#include <string.h>
#include <bitset>
#include <unistd.h>
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>

#include "motor_command.h"

// Reset, send mode,stop,off both only require fd, no address due to echo
using namespace LibSerial ;

int SendDecel(int fd,unsigned char addr) {
  int ret,n;
  char buff[20];
  n = sprintf(buff,"%cX%c",128+addr,0x20);
  ret = write(fd,buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}

int SendStop(int fd) {
  int ret;
  unsigned char buff[] = {'S',0x20};
  ret = write(fd, buff, sizeof(buff));
  if(ret==sizeof(buff)) return EAN_OK;
  else return -EAN_ERR;
}

int SendOff(int fd,unsigned char addr) {
  int ret;
  unsigned char buff[] = {0x4F,0x46,0x46,0x20};
  ret = write(fd, buff, sizeof(buff));
  if(ret==sizeof(buff)) return EAN_OK;
  else return -EAN_ERR;
}

int SendReset(int fd,unsigned char addr) {
  int ret,n;
  char buff[20];
  n = sprintf(buff,"%cZ%c",128+addr,0x20);
  ret = write(fd,buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}

int SendGo(int fd, unsigned char addr) {
  int ret,n;
  char buff[20];
  n = sprintf(buff,"%cG%c",128+addr,0x20);
  ret = write(fd,buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}

int SendVel(int fd, unsigned char addr, int vel) {
  int ret,n;
  char buff[20];
  n = sprintf(buff,"%cVT=%d%c",128+addr,vel,0x20);
  ret = write(fd,buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}

int SendOrigin(int fd, unsigned char addr, int origin) {
  int ret,n;
  char buff[20];
  n = sprintf(buff, "%cO=%d%c",128+addr,origin,0x20);
  ret = write(fd, buff,n);
  if(ret==sizeof(buff)) return EAN_OK;
  else return -EAN_ERR;
}

void ReqPos(SerialPort& castor,int fd, int pos[2]) {
//  printf("castor: %d\n",fd-2);
  int ret;
  char buff[] = {42,129,'R','P','A',0x20};
  ret = write(fd,buff,sizeof(buff));
 if(ret != sizeof(buff)) printf("did not send rpa\n");

//  usleep(1000);
  char init_response[100];
  char *initptr;
  initptr = init_response;
//  printf("chars until *: \n");
  ret = 0;
  while(*initptr != 42){
     if(ret == 1) initptr++;
     ret = read(fd,initptr,1);
//     printf("%d ",*initptr);
  }
//  printf("\n");
  char init_response_rp[256] = {0};
  char *initptr_rp;
  initptr_rp = init_response_rp;
//  printf("chars of rpa:\n");
  ret = 0;
  while(*initptr_rp != 13 && *initptr_rp != 0x20) {
        if(ret==1) initptr_rp++;
	ret = read(fd,initptr_rp,1);
//        printf("%d ",*initptr_rp);
  }
//  printf("\n");


  char read_char[256]={0};
  char *bufptr;
  bufptr = read_char;
  int read_ctr = 0;
//  printf("position msg chars:\n");
  ret = 0;
  while(*bufptr != 13 && *bufptr != 0x20) {
  	if(ret == 1){
 		bufptr++;
		read_ctr++;
	}
	ret = read(fd,bufptr,1);
//        printf("%d %d\n",read_ctr,*bufptr);
// 	usleep(100); 
  }
//  printf("\n");
  int pos_data_ctr = 0;
  for (int i=0;i<read_ctr;i++) {
  	if(read_char[i] != 0 && read_char[i] != 13 && read_char[i] != 0x20) pos_data_ctr++;
  }

  char pos_msg[pos_data_ctr-1];
  pos_data_ctr = 0;

  for (int i=0; i<read_ctr; i++){
        if (read_char[i] != 0 && read_char[i] !=13 && read_char[i] != 0x20){
        	pos_msg[pos_data_ctr] = read_char[i];
                pos_data_ctr++;
        }
  }
  int pos_actual;
  sscanf(pos_msg,"%d", &pos_actual);
//  printf("Position of motor 1:\n");

//  printf("%d\n",pos_actual);
  memset(buff,0,sizeof(buff));
  pos[0] = pos_actual;

  char buff1[] = {42,130,'R','P','A',0x20};
  ret = write(fd,buff1,sizeof(buff1));
  if(ret != sizeof(buff)) printf("did not send rpa\n");

//  usleep(1000);
  char init_response1[100];
  char *initptr1;
  initptr1 = init_response1;
//  printf("chars until *: \n");
  ret = 0;
  while(*initptr1 != 42){
     if(ret == 1) initptr1++;
     ret = read(fd,initptr1,1);
//     printf("%d ",*initptr1);
  }
//  printf("\n");
  char init_response_rp1[256] = {0};
  char *initptr_rp1;
  initptr_rp1 = init_response_rp1;
//  printf("chars of rpa:\n");
  ret = 0;
  while(*initptr_rp1 != 13 && *initptr_rp1 != 0x20) {
        if(ret==1) initptr_rp1++;
        ret = read(fd,initptr_rp1,1);
//        printf("%d ",*initptr_rp1);
  }
//  printf("\n");

  char read_char1[256]={0};
  char *bufptr1;
  bufptr1 = read_char1;
  int read_ctr1 = 0;
//  printf("position msg chars:\n");
  ret = 0;
  while(*bufptr1 != 13 && *bufptr1 != 0x20) {
        if(ret == 1){
                bufptr1++;
                read_ctr1++;
        }
        ret = read(fd,bufptr1,1);
//        printf("%d %d\n",read_ctr1,*bufptr1);
//        usleep(100); 
  }
//  printf("\n");
  int pos_data_ctr1 = 0;
  for (int i=0;i<read_ctr1;i++) {
        if(read_char1[i] != 0 && read_char1[i] != 13 && read_char1[i] != 0x20) pos_data_ctr1++;
  }

  char pos_msg1[pos_data_ctr1-1];
  pos_data_ctr1 = 0;

  for (int i=0; i<read_ctr1; i++){
        if (read_char1[i] != 0 && read_char1[i] !=13 && read_char1[i] != 0x20){
                pos_msg1[pos_data_ctr1] = read_char1[i];
                pos_data_ctr1++;
        }
  }
  int pos_actual1;
  sscanf(pos_msg1,"%d", &pos_actual1);
//  printf("Position of motor 2:\n");

//  printf("%d\n",pos_actual1);
  memset(buff1,0,sizeof(buff1));
  pos[1] = pos_actual1;
}

int SetAccel(int fd, unsigned char addr, int accel) {
  int ret,n;
  char buff[20];
  n = sprintf(buff,"%cADT=%d%c",128+addr,accel,0x20);
  ret = write(fd,buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}
int SendTorque(int fd, unsigned char addr, int T) {
  int ret,n;
  char buff[20];
  n = sprintf(buff, "%cT=%d%c",128+addr,T,0x20);
  ret = write(fd, buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}

int ReqVoltage(int fd, unsigned char addr) {
  int ret,n;
  char buff[20];
  n = sprintf(buff,"%cv=UJA%cRv%c",128+addr,0x20,0x20);
  ret = write(fd,buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}

int SetTorqueMode(int fd, unsigned char addr) {
  int ret;
  char buff[] = {0x4D,0x54,0x20};
  ret = write(fd, buff, sizeof(buff));
  if(ret==sizeof(buff)) return EAN_OK;
  else return -EAN_ERR;
}

int SetVelMode(int fd,unsigned char addr) {
  int ret,n;
  char buff[20];
  n = sprintf(buff,"%cMV%c",128+addr,0x20);
  ret = write(fd,buff,n);
  if(ret==n) return EAN_OK;
  else return -EAN_ERR;
}  
int Echo_Off(int fd) {
  int ret;
  char buff[] = {'E','C','H','O','_','O','F','F',0x20};
  ret = write(fd,buff,sizeof(buff));
  if(ret==sizeof(buff)) return EAN_OK;
  else return -EAN_ERR;
}
int Echo(int fd) {
  int ret;
  char buff[] = {'E','C','H','O',0x20};
  ret = write(fd,buff,sizeof(buff));
  if(ret==sizeof(buff)) return EAN_OK;
  else return -EAN_ERR;
}


