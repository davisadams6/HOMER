#include <iostream>
#include <string.h>
#include <bitset>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <math.h>

#include "encoder.h"

#define CALIBRATE 1


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

short int enc_offsets[3] = {ENC1_OFFSET,ENC2_OFFSET,ENC3_OFFSET};
int encoder_fd;


void cleanup_encoders(void) {
	
}

//int init_encoders();

int main() {
	
	
	
	while (true) {
		unsigned char buff[255];
		unsigned char *ptr = buff;
		memset(buff,0,sizeof(buff));
		std::cout << "Enter the number 1 to get encoder angles" << std::endl ;
		int num;
		std::cin >> num;
		if (num == 1) {
				//tcflush(encoder_fd,TCIOFLUSH);
				init_encoders();
				unsigned char enc_header_byte;
				double encoder_angles[3] = {0};
				
				unsigned char cksum_calc = 0;

				int len;	
				unsigned char valid[3];
				
				
				//delete ptr;
				//ptr = &buff;
				//*ptr = 0;
				//memset(buff,0,sizeof(buff));
				ptr = &buff[0];
				//ptr=0;
				int i = 0;
				while(*ptr != ENCODER_HEADER_BYTE) {
        				len = read(encoder_fd,ptr,1);
      				}
				ptr++;

				while(len<ENCODER_PACKET_LENGTH) {
       					len += read(encoder_fd,ptr,ENCODER_PACKET_LENGTH-1);
     				}
						
				if(len!=ENCODER_PACKET_LENGTH) {
        				std::cout << "invalid packet: received " << len << "bytes, expected " << ENCODER_PACKET_LENGTH << std::endl;
        				continue;
      				}

				memcpy((void*)&enc_packet,buff,ENCODER_PACKET_LENGTH);

				unsigned short int enc1,enc2,enc3;	

				for(int i=1;i<ENCODER_PACKET_LENGTH-1;++i) {
					cksum_calc += buff[i];
				}

				if (cksum_calc == buff[ENCODER_PACKET_LENGTH-1]) {
					std::cout << "The calculated and encoder check sum are equal!!" << std::endl;
				}
				else {
					std::cout << "The calculated and encoder check sum are not equal...." << std::endl;
				}

				memcpy((void*)&enc_packet,buff,ENCODER_PACKET_LENGTH);
				
				if(enc_packet.header!=ENCODER_HEADER_BYTE) {
					std::cerr << "Did not recieve the right header byte!!" << std::endl;
				}
				
				for(int i=0;i<3;++i) {
					if(enc_packet.enc[i]&(1<<13)) {
						valid[i] = 0;
					}
					else {
						valid[i] = 1;						
					}
					enc_packet.enc[i] &= (0x1FFF);
				}
			
				if (valid[0] + valid[1] + valid[2] < 3) {
					std::cout << "Not enough valid encoders" << std::endl; 
				}

				if(CALIBRATE) {
					std::cout << "Raw angles:" << std::endl;
					for(int i=0;i<3;i++) {
						encoder_angles[i] = enc_packet.enc[i];
						std::cout << "Encoder " << i+1 << " angle: " << encoder_angles[i] << std::endl;
					}				
		
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
	
				std::cout << "Calculated angles:" << std::endl;
				for(int i=0;i<3;i++) {
					std::cout << "Encoder " << i+1 << " angle: " << encoder_angles[i] << std::endl;
				}
				
				close(encoder_fd);
				std::cout << enc_packet.header << std::endl;
				enc_packet = {};
				std::cout << enc_packet.header << std::endl;
				std::cout << enc_packet.enc[0] << std::endl;
				std::cout << enc_packet.enc[1] << std::endl;
				std::cout << enc_packet.enc[2] << std::endl;
				std::cout << enc_packet.cksum << std::endl;
				//tcflush(encoder_fd,TCIOFLUSH);
		
				
			}
		
	}

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



