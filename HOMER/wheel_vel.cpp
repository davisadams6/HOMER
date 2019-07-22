#include <math.h>
#include <stdlib.h>
//#include "wheel_vel.h"
// added this to possibly help with multi-threaded gdb debugging
//#include <sys/types.h>
//#include <signal.h>
#include <unistd.h>

#define in2m 0.0254f
#define deg2rad (M_PI/180.0f)

void wheel_vel(double omega[6], double theta[3], double xdot, double ydot, double psidot, double  psi) {
  int i;
  const double d = 2*in2m; // lateral distance of wheel centerline from pivot
  const double rc = 2*in2m; // distance of wheel shaft from pivot
  const double L = 2*in2m; //
  const double rw = 1.5*in2m; // wheel radius
  const double R = 15.065*in2m; // distance of pivot point from body center
  const double phi[3] = {0*deg2rad, 120*deg2rad, 240*deg2rad};

  double pivot_xdot, pivot_ydot, castor_xdot, castor_ydot;
  
  for(i=0;i<3;++i) {
    pivot_xdot = xdot + -R*sin(phi[i])*psidot;
    pivot_ydot = ydot + +R*cos(phi[i])*psidot;
    castor_xdot = cos(theta[i])*pivot_xdot + sin(theta[i])*pivot_ydot;
//    castor_ydot = -sin(theta[i])*pivot_xdot + cos(theta[i])*pivot_ydot;  
    castor_ydot = +sin(theta[i])*pivot_xdot - cos(theta[i])*pivot_ydot; // original homer code
//    omega[2*i] = (castor_xdot - rc/d*castor_ydot)/rw;
    omega[2*i] = (castor_xdot + rc/d*castor_ydot)/rw; // original homer code
//    omega[2*i+1] = -(castor_xdot + rc/d*castor_ydot)/rw;
    omega[2*i+1] = (-castor_xdot + rc/d*castor_ydot)/rw; // original homer code
  }
}

