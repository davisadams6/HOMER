#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
using namespace std ;
const char *path="/home/tinker/HOMER/vicon_data.txt";

void quat_mult(double *q, double *p) {
	double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3] ;
	double p0 = p[0], p1 = p[1], p2 = p[2], p3 = p[3] ;

	q[0] =  q1*p0 + q2*p3 - q3*p2 + q0*p1;
    	q[1] = -q1*p3 + q2*p0 + q3*p1 + q0*p2;
    	q[2] =  q1*p2 - q2*p1 + q3*p0 + q0*p3;
    	q[3] = -q1*p1 - q2*p2 - q3*p3 + q0*p0;
}

void vicon_rx(double pose[3]) {
	double r[3];
	double q[4];
	string line;
	ifstream file(path,ios::in| ios::ate);
	if(file.is_open()){
		file.seekg(-1,ios_base::end);			// go to one spot before the EOF
		bool keepLooping = true;
        	while(keepLooping) {
            		char ch;
            		file.get(ch);                            // Get current byte's data

            		if((int)file.tellg() <= 1) {             // If the data was at or before the 0th byte
                		file.seekg(0);                       // The first line is the last line
                		keepLooping = false;                // So stop there
            		}
            		else if(ch == '\n') {                   // If the data was a newline
              			keepLooping = false;                // Stop at the current position.
          		}
            		else {                                  // If the data was neither a newline nor at the 0 byte
               			file.seekg(-2,ios_base::cur);        // Move to the front of that data, then to the front of the data before it
            		}
     		}

        string lastLine;            
        getline(file,lastLine);                      // Read the current line
//        cout << "Result: " << lastLine << '\n';     // Display it
	string name = lastLine.substr(0,lastLine.find(","));
	size_t comma = lastLine.find(",");
	size_t comma2 = lastLine.find(",",comma);
	for (int i = 0;i<3;i++){
      		comma = comma2;
      		comma2 = lastLine.find(",",comma+1);
      		r[i] = stod(lastLine.substr(comma+1,comma2-comma));
   	}
	for (int i = 0;i<4;i++){
      		comma = comma2;
      		comma2 = lastLine.find(",",comma+1);
      		q[i] = stod(lastLine.substr(comma+1,comma2-comma));
    	}

	double p[4] = {0,0,1,0}; // unit j vector in quaternion (because vicon frame is wierd)

	double qw = q[0];
	double qx = q[1];
	double qy = q[2];
	double qz = q[3];

	double q1[4] = {qw,qx,qy,qz};
	double q1_inv[4] = {qw,-qx,-qy,-qz}; 

	quat_mult(q1_inv,p); // q_ * p
	double new_q[4];
	new_q[0] = q1_inv[0];
	new_q[1] = q1_inv[1];
	new_q[2] = q1_inv[2];
	new_q[3] = q1_inv[3];
	quat_mult(new_q,q1); // (q_*p) * q
 
	double yaw_angle_point = atan2(-new_q[2],new_q[1]); 
	double yaw_angle_HOMER;	
	yaw_angle_HOMER = yaw_angle_point + M_PI;

	pose[0] = r[0];
	pose[1] = r[1];
	pose[2] = yaw_angle_HOMER;

        file.close();
	}
}




