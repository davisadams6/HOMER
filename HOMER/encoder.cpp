#include <iostream>
#include <string.h>
#include <bitset>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <math.h>

#include "encoder.h"

#define CALIBRATE 0


#define ENCODER_HEADER_BYTE 0xAA
#define ENCODER_BAUDRATE B38400
#define ENCODER_PORT "/dev/ttyUSB3"

/*
#pragma pack(1)
struct {
	unsigned char header;
	unsigned short int enc[3];
	unsigned char cksum;
} enc_packet;

#define ENCODER_PACKET_LENGTH sizeof(enc_packet)
*/

int len;	
unsigned char valid[3];
short int enc_offsets[3] = {ENC1_OFFSET,ENC2_OFFSET,ENC3_OFFSET};
int encoder_fd;

void cleanup_encoders(void) {
	close(encoder_fd);
}

int init_encoders();

void encoder_read(double theta[3]) {
	init_encoders();
	
	double encoder_angles[3];
	unsigned char buff[255];
	unsigned char cksum_calc = 0;
	unsigned char enc_header_byte;			
	unsigned char *ptr = buff;
	*ptr = 0;
				
	int i = 0;
	while(*ptr != ENCODER_HEADER_BYTE) {
        	len = read(encoder_fd,ptr,1);
      	}
	ptr++;

	while(len<ENCODER_PACKET_LENGTH) {
       		len += read(encoder_fd,ptr,ENCODER_PACKET_LENGTH-1);
     	}

	memcpy((void*)&enc_packet,buff,ENCODER_PACKET_LENGTH);

	unsigned short int enc1,enc2,enc3;	

	for(int i=1;i<ENCODER_PACKET_LENGTH-1;++i) {
		cksum_calc += buff[i];
	}

	memcpy((void*)&enc_packet,buff,ENCODER_PACKET_LENGTH);

	for(int i=0;i<3;++i) {
		if(enc_packet.enc[i]&(1<<13)) {
			valid[i] = 0;
		}
		else {
			valid[i] = 1;						
		}
		enc_packet.enc[i] &= (0x1FFF);
	}

	for(int i=0;i<3;i++) {
		if(valid[i]) {
			if(enc_offsets[i]>enc_packet.enc[i]) {
				enc_packet.enc[i] += 8192 - enc_offsets[i];
			}
			else {
				enc_packet.enc[i] -= enc_offsets[i];
			}
			encoder_angles[i] = enc_packet.enc[i]/8192.0*2*M_PI;
		}
	}

		enc_packet = {};
		theta[0] = encoder_angles[0];
		theta[1] = encoder_angles[1];
		theta[2] = encoder_angles[2];	
	close(encoder_fd);
}

int init_encoders() {

	struct termios options;
	int ret = 0;

	encoder_fd = open(ENCODER_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
	if (encoder_fd<0) {
		std::cerr << "The serial port did not open correctly!" << std::endl;
		return EXIT_FAILURE;
	}

	tcgetattr(encoder_fd,&options);
	options.c_cflag = ENCODER_BAUDRATE | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	options.c_cc[VMIN] = 0;
	options.c_cc[VTIME] = 1;
	tcflush(encoder_fd,TCIOFLUSH);
	tcsetattr(encoder_fd,TCSANOW,&options);


	return 0;
}

void cleanup_encoders(void);


