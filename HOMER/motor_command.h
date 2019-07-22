
#ifndef _MOTOR_COMMAND_H_
#define _MOTOR_COMMAND_H_

#include <math.h>
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <fstream>
#include <unistd.h>


#define REV2POS (7*4000)
#define RPM2VEL (7*536.87633)
#define REVPS2VEL (7*32768)
#define RADPS2VEL (REVPS2VEL/2/M_PI)
#define RPSSQ2ACCEL (7*7.9166433)

enum { EAN_OK=0, EAN_ERR=1 };

typedef union {
  unsigned char bval[4];
  int ival;
  float fval;
} data32;

using namespace LibSerial ;

int SendDecel(int fd,unsigned char addr);
int SendStop(int fd);
int SendOff(int fd);
int SendReset(int fd,unsigned char addr);
int SendGo(int fd, unsigned char addr);
int SendVel(int fd, unsigned char addr, int vel);
int SendOrigin(int fd, unsigned char addr, int origin);
void ReqPos(SerialPort& castor,int fd, int pos[2]);
int SetAccel(int fd, unsigned char addr, int accel);
int SendTorque(int fd, unsigned char addr, int T);
int ReqVoltage(int fd, unsigned char addr);
int SetTorqueMode(int fd);
int SetVelMode(int fd,unsigned char addr);
int Echo_Off(int fd);
int Echo(int fd);

#endif /* _MOTOR_COMMAND_H_ */
