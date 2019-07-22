//When compiling must do: g++ feedback_tracker.cpp encoder.cpp motor_command.cpp joystick.cc wheel_vel.cpp castor.cpp local2global.cpp parameters.cpp odometry.cpp csv_read_feedback.cpp vicon_rx.cpp -o feedback_tracker -Wno-psabi -L/usr/local/lib -lserial

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
#include "spline.h"
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
#define VICON_FEEDBACK  1
#define in2m		0.0254

using namespace LibSerial ;

void wheel_vel(double omega[6], double theta[3], double xdot, double ydot, double psidot, double psi_base);
int local2global(double BodyVel[3], double Pose[3], double* InertialVel);
int global2local(double InertialVel[3], double Pose[3], double* BodyVel);
int UpdatePose(paramset Param, int OldWheelCounts[3][2], int NewWheelCounts[3][2],double Theta[3],double *Pose,double* CastorHeading);
int csv_read(std::string &filename, double* traj_time, double* traj_s1, double* traj_s2, double* traj_s3, double* traj_s4, double* traj_s5, double* traj_s6, double* traj_s7, double* traj_s8, double* traj_s9, double* traj_v1, double* traj_v2, double* traj_v3);
void vicon_rx(double pose[3]);

int main(int argc, char** argv)
{
	// Read in trajectory
	double traj_time[10000]={0};
        double traj_s1[10000]={0};
        double traj_s2[10000]={0};
        double traj_s3[10000]={0};
	double traj_s4[10000];
	double traj_s5[10000];
	double traj_s6[10000];
	double traj_s7[10000];
	double traj_s8[10000];
	double traj_s9[10000];
	double traj_v1[10000];
	double traj_v2[10000];
	double traj_v3[10000];
        std::string traj = "feedback_circle_traj1.csv";
        csv_read(traj,traj_time,traj_s1,traj_s2,traj_s3,traj_s4,traj_s5,traj_s6,traj_s7,traj_s8,traj_s9,traj_v1,traj_v2,traj_v3);
        int i =0;
        while(traj_time[i+1]!=0){
//                printf("t=%f, s1=%f, s2=%f, s3=%f\n",traj_time[i],traj_s1[i],traj_s2[i],traj_s3[i]);
		i++;
        }
	i++;

	std::vector<double> t_data(i), s1(i), s2(i), s3(i), s4(i), s5(i), s6(i), s7(i), s8(i), s9(i), v1(i), v2(i), v3(i);
        for(int j=0;j<i;j++){
                t_data[j] = traj_time[j];
                s1[j] = traj_s1[j];
                s2[j] = traj_s2[j];
                s3[j] = traj_s3[j];
                s4[j] = traj_s4[j];
                s5[j] = traj_s5[j];
                s6[j] = traj_s6[j];
                s7[j] = traj_s7[j];
                s8[j] = traj_s8[j];
                s9[j] = traj_s9[j];
                v1[j] = traj_v1[j];
                v2[j] = traj_v2[j];
                v3[j] = traj_v3[j];
//              printf("%f      %f      %f      %f\n",t_data[j],s1[j],s2[j],s3[j]);
        }

	//Generate spline
	tk::spline s1_data;
    	s1_data.set_points(t_data,s1);
        tk::spline s2_data;
        s2_data.set_points(t_data,s2);
        tk::spline s3_data;
        s3_data.set_points(t_data,s3);
        tk::spline s4_data;
        s4_data.set_points(t_data,s4);
        tk::spline s5_data;
        s5_data.set_points(t_data,s5);
        tk::spline s6_data;
        s6_data.set_points(t_data,s6);
        tk::spline s7_data;
        s7_data.set_points(t_data,s7);
        tk::spline s8_data;
        s8_data.set_points(t_data,s8);
        tk::spline s9_data;
        s9_data.set_points(t_data,s9);
        tk::spline v1_data;
        v1_data.set_points(t_data,v1);
        tk::spline v2_data;
        v2_data.set_points(t_data,v2);
        tk::spline v3_data;
        v3_data.set_points(t_data,v3);

	//Generate output file
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
	double x0 = -1;
	double y0 = 0;
	double psi0 = 0;
	double old_pose[3];
	double x_offset_vicon = 11.9763*in2m;
	double y_offset_vicon = 5.11827*in2m;
	double pose[3] = {x0,y0,psi0};
	if(VICON_FEEDBACK) {
		vicon_rx(pose);
		pose[0] -= x_offset_vicon;
		pose[1] += y_offset_vicon;
	}
	int global_flag = 1;
	double S[3][3] = {0};
	double v[3] = {0};
	const double R_inv[3] = {1,1,1};
	const double B[3] = {1,1,1};
	double u[3] = {0};

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
	bool endprog_button = false;

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
//	output << "Time,X Estimate [m],Y Estimate [m],Psi Estimate[m],xdot[m/s],ydot[m/s],psidot[rad/s],Castor 1: left_wc, Castor1: right_wc,Castor 2: left_wc, Castor2: right_wc,Castor 3: left_wc, Castor3: right_wc\n" ;
	output << "Time,X Estimate [m],Y Estimate [m],Psi Estimate[m],xdot[m/s],ydot[m/s],psidot[rad/s]\n" ;
	output << "0,0,0,0,0,0,0\n" ;
	std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
	std::chrono::high_resolution_clock::time_point current_time = start_time;
	std::chrono::duration<double> t = std::chrono::duration_cast<std::chrono::duration<double>>(current_time-start_time);
	printf("t1 = %f\n",t.count());
	printf("end time = %f\n",t_data.back());

 	while (t.count()<t_data.back()) {
                // Attempt to sample an event from the joystick
                JoystickEvent event;
                if (joystick.sample(&event)) {
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
					endprog_button = true;
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
/*			xdot_new = xd_data(t.count());
			ydot_new = yd_data(t.count());
			psidot_new = psid_data(t.count()); 
*/
			S[0][0] = s1_data(t.count());
			S[1][0] = s2_data(t.count());
			S[2][0] = s3_data(t.count());
                        S[0][1] = s4_data(t.count());
                        S[1][1] = s5_data(t.count());
                        S[2][1] = s6_data(t.count());
                        S[0][2] = s7_data(t.count());
                        S[1][2] = s8_data(t.count());
                        S[2][2] = s9_data(t.count());
                        v[0] = v1_data(t.count());
                        v[1] = v2_data(t.count());
                        v[2] = v3_data(t.count());

			u[0] = -(S[0][0]*R_inv[0]*B[0]*pose[0]+S[0][1]*R_inv[0]*B[0]*pose[1]+S[0][2]*R_inv[0]*B[0]*pose[2])+(R_inv[0]*B[0]*v[0]);
			u[1] = -(S[1][0]*R_inv[1]*B[1]*pose[0]+S[1][1]*R_inv[1]*B[1]*pose[1]+S[1][2]*R_inv[1]*B[1]*pose[2])+(R_inv[1]*B[1]*v[1]);
			u[2] = -(S[2][0]*R_inv[2]*B[2]*pose[0]+S[2][1]*R_inv[2]*B[2]*pose[1]+S[2][2]*R_inv[2]*B[2]*pose[2])+(R_inv[2]*B[2]*v[2]);

			xdot_new = u[0];
			ydot_new = u[1];
			psidot_new = u[2];

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

/*			for(int i=0;i<3;i++){
				for(int j=0;j<2;j++){
					oldwc[i][j] = newwc[i][j];
				}
			}
			for(int i=0;i<3;i++) {
				ReqPos(castor[i],castors[i].fd,newwc[i]);
			}

*/			for(int i=0;i<3;i++){
				SendVel(castors[i].fd,wheel_addr[0],-int(floor(omega[2*i]*RADPS2VEL)));
				SendVel(castors[i].fd,wheel_addr[1],-int(floor(omega[2*i+1]*RADPS2VEL)));
				SetAccel(castors[i].fd,wheel_addr[0],100);
				SetAccel(castors[i].fd,wheel_addr[1],100);
				SendGo(castors[i].fd,wheel_addr[0]);
				SendGo(castors[i].fd,wheel_addr[1]);
			}

//			UpdatePose(CalParam,oldwc,newwc,theta,pose,casthead);
			old_pose[0] = pose[0];
			old_pose[1] = pose[1];
			old_pose[2] = pose[2];

			vicon_rx(pose);
			if(pose[0] == 0 && pose[1] == 0 && old_pose[0] != 0 && old_pose[1] != 0){
                        	pose[0] = old_pose[0];
                        	pose[1] = old_pose[1];
                        	pose[2] = old_pose[2];
			}
			if(VICON_FEEDBACK) {
				pose[0] -= x_offset_vicon;
				pose[1] += y_offset_vicon;
			}

			current_time = std::chrono::high_resolution_clock::now();
			t = std::chrono::duration_cast<std::chrono::duration<double>>(current_time-start_time);
//			output << t.count() << ',' << pose[0] << ',' <<  pose[1] << ',' <<  pose[2] << ','  << xdot_new << ',' << ydot_new << ',' << psidot_new << ',' << newwc[0][0] << ',' << newwc[0][1] << ',' << newwc[1][0] << ',' << newwc[1][1] << ',' << newwc[2][0] << ',' << newwc[2][1] << "\n";
			output << t.count() << ',' << pose[0] << ',' <<  pose[1] << ',' <<  pose[2] << ','  << xdot_new << ',' << ydot_new << ',' << psidot_new << "\n";
			if(!estop_button) printf("X Estimate [m]: %f	Y Estimate [m]: %f	Psi Estimate [rad]: %f\n",pose[0],pose[1],pose[2]);
		}
	}
	if(!endprog_button){
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
	}
	for(int i=0;i<3;i++){
		castor[i].Close();
	}
	output.close();
	return 0;
}
