// When compiling must do: g++ joystick_command.cpp encoder.cpp motor_command.cpp joystick.cc wheel_vel.cpp castor.cpp local2global.cpp parameters.cpp odometry.cpp -o joystick -L/usr/local/lib -lserial

// axis max/min = +/-32768 units
#include <unistd.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <libserial/SerialPort.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

#include "joystick.hh"
#include "encoder.h"
#include "castor.h"
#include "motor_command.h"
#include "parameters.h"
/*
#define CASTOR_BAUDRATE 115200
#define CASTOR_PORT_1 "/dev/ttyUSB0"
#define CASTOR_PORT_2 "/dev/ttyUSB1"
#define CASTOR_PORT_3 "/dev/ttyUSB2"

const char *castor_ports[] = {CASTOR_PORT_1,CASTOR_PORT_2,CASTOR_PORT_3};
*/
#define VEL_CMD_MAX	0.2 // m/s
#define OMEGA_CMD_MAX	0.34907 // rad/s
#define ACC_CMD_MAX	0.5 // m/s/s
#define ACC_PSI_CMD_MAX 0.87266 // rad/s/s
#define axis_max 	32768
#define VELOCITY_MODE	1

using namespace LibSerial ;

void wheel_vel(double omega[6], double theta[3], double xdot, double ydot, double psidot, double psi_base);
int local2global(double BodyVel[3], double Pose[3], double* InertialVel);
int global2local(double InertialVel[3], double Pose[3], double* BodyVel);
int UpdatePose(paramset Param, int OldWheelCounts[3][2], int NewWheelCounts[3][2],double Theta[3],double *Pose,double* CastorHeading);

int main(int argc, char** argv)
{
	std::ofstream output;
	output.open("output1.csv");

	SerialPort castor[3];
	castor_params castors[3];
	int castor_fd[] = {castors[0].fd,castors[1].fd,castors[2].fd};

	unsigned char wheel_addr[] = {1,2};
	double xdot_new = 0.0f;
	double xdot_old = 0.0f;
	double ydot_new = 0.0f;
	double ydot_old = 0.0f;
	double psidot_new = 0.0f;
	double psidot_old = 0.0f;
	int oldwc[3][2] = {0};
	int newwc[3][2] = {0};
	paramset CalParam = NomParam();
	double theta[3] = {0.0f,0.0f,0.0f};
	double omega[6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
	double psi_base = 0.0f;
	double body_cmd[3] = {0};
	double ivels[3] = {0};
	double casthead[3] = {0};
	double pose[3] = {0};
	int global_flag = 1;

	init_castors(castor,castor_fd);
	castors[0].fd = castor_fd[0];
	castors[1].fd = castor_fd[1];
	castors[2].fd = castor_fd[2];
	printf("Castors initializized with fd's %d,%d,%d \n",castors[0].fd,castors[1].fd,castors[2].fd);

	// Create an instance of Joystick
  	Joystick joystick("/dev/input/js0");

  	// Ensure that it was found and that we can use it
  	if (!joystick.isFound()) {
    		printf("joystick open failed.\n");
    		exit(1);
  	}

	bool estop_button = false;


	if(VELOCITY_MODE){
		for(int i=0;i<3;i++){
			SendReset(castors[i].fd,wheel_addr[1]);
			SendReset(castors[i].fd,wheel_addr[0]);
		}
		sleep(2);
		for(int i=0;i<3;i++){
			SetVelMode(castors[i].fd,wheel_addr[0]);
			SetVelMode(castors[i].fd,wheel_addr[1]);
		}
	}
	usleep(1000);
	output << "Time,X Estimate [m],Y Estimate [m],Psi Estimate[m],xdot[m/s],ydot[m/s],psidot[rad/s],Castor 1: left_wc, Castor1: right_wc,Castor 2: left_wc, Castor2: right_wc,Castor 3: left_wc, Castor3: right_wc\n" ;
	output << "0,0,0,0,0,0,0,0,0,0,0,0,0\n" ;
	std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

 	while (true) {
    		// Restrict rate
//    		usleep(10);

    		// Attempt to sample an event from the joystick
    		JoystickEvent event;
    		if (joystick.sample(&event)) {
      			if (event.isAxis() && !estop_button) {
				if (event.number == 0) {
					xdot_new = VEL_CMD_MAX*(double(event.value)/double(axis_max));
				}
				if (event.number == 1) {
					ydot_new = -VEL_CMD_MAX*(double(event.value)/double(axis_max));
				}
				if (event.number == 2) {
					psidot_new = -OMEGA_CMD_MAX*(double(event.value)/double(axis_max));
				}
			}
			if (event.isButton()) {
				if (event.number == 7 && event.value == 1) { // estop pressed - start button
					printf("ESTOP BUTTON WAS HIT!!!\n");
					estop_button = true;
					xdot_new = 0.0;
					ydot_new = 0.0;
					psidot_new = 0.0;
				}
				else if(event.number == 0 && event.value == 1 && estop_button) { // resume button pressed - A button
					printf("RESUMING MOTION!!!\n");
					estop_button = false;
				}
				else if(event.number == 6 && event.value == 1) { // stop program and reset motors  - Select button
					printf("Ending program and shutting down motors...\n");
					for(int i=0;i<3;i++){
                        			SendDecel(castors[i].fd,wheel_addr[0]);
                        			SendDecel(castors[i].fd,wheel_addr[1]);
               				}
					sleep(1);
					for(int i=0;i<3;i++){
                        			SendReset(castors[i].fd,wheel_addr[1]);
                        			SendReset(castors[i].fd,wheel_addr[0]);
                			}
					sleep(2);
					break;
				}
			}
		}
		if(!estop_button){
//                        printf("Xdot = %f     Ydot = %f     Psidot = %f\n",xdot_new,ydot_new,psidot_new);
                }

		body_cmd[0] = xdot_new;
		body_cmd[1] = ydot_new;
		body_cmd[2] = psidot_new;
		encoder_read(theta);
		if(global_flag) {
			ivels[0] = body_cmd[0];
			ivels[1] = body_cmd[1];
			ivels[2] = body_cmd[2];
			global2local(ivels,pose,body_cmd);
		}

		wheel_vel(omega,theta,body_cmd[0],body_cmd[1],body_cmd[2],0);

		for(int i=0;i<3;i++){
			for(int j=0;j<2;j++){
				oldwc[i][j] = newwc[i][j];
			}
		}
		for(int i=0;i<3;i++) {
			ReqPos(castor[i],castors[i].fd,newwc[i]);
		}

		for(int i=0;i<3;i++){
			SendVel(castors[i].fd,wheel_addr[0],-int(floor(omega[2*i]*RADPS2VEL)));
			SendVel(castors[i].fd,wheel_addr[1],-int(floor(omega[2*i+1]*RADPS2VEL)));
			SetAccel(castors[i].fd,wheel_addr[0],100);
			SetAccel(castors[i].fd,wheel_addr[1],100);
			SendGo(castors[i].fd,wheel_addr[0]);
			SendGo(castors[i].fd,wheel_addr[1]);
		}
		UpdatePose(CalParam,oldwc,newwc,theta,pose,casthead);
		std::chrono::high_resolution_clock::time_point current_time = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> t = std::chrono::duration_cast<std::chrono::duration<double>>(current_time-start_time);
		output << t.count() << ',' << pose[0] << ',' <<  pose[1] << ',' <<  pose[2] << ','  << xdot_new << ',' << ydot_new << ',' << psidot_new << ',' << newwc[0][0] << ',' << newwc[0][1] << ',' << newwc[1][0] << ',' << newwc[1][1] << ',' << newwc[2][0] << ',' << newwc[2][1] << "\n";
		if(!estop_button) printf("X Estimate [m]: %f	Y Estimate [m]: %f	Psi Estimate [rad]: %f\n",pose[0],pose[1],pose[2]);
	}
	for(int i=0;i<3;i++){
		castor[i].Close();
	}
	output.close();
	return 0;
}
