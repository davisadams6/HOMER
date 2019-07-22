#ifndef _ENCODER_H_
#define _ENCODER_H_



#define ENC1_OFFSET 2839
#define ENC2_OFFSET 2793
#define ENC3_OFFSET 296
/*
#define ENCODER_HEADER_BYTE 0xAA
#define ENCODER_BAUDRATE 38400
#define ENCODER_PORT "/dev/ttyUSB0"
*/
#pragma pack(1)
struct {
	unsigned char header;
	unsigned short int enc[3];
	unsigned char cksum;
} enc_packet;

#define ENCODER_PACKET_LENGTH sizeof(enc_packet)

void cleanup_encoders(void);
int init_encoders();
void encoder_read(double theta[3]);


#endif /* _ENCODER_H_ */
