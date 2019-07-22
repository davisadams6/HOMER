#include<math.h>

int local2global(double BodyVel[3], double Pose[3], double* InertialVel)
{

  double cpsi = cos(Pose[2]);
  double spsi = sin(Pose[2]);
  InertialVel[0] = BodyVel[0]*cpsi - BodyVel[1]*spsi;
  InertialVel[1] = BodyVel[0]*spsi + BodyVel[1]*cpsi;
  InertialVel[2] = BodyVel[2];

  return 0;
}

int global2local(double InertialVel[3], double Pose[3], double* BodyVel)
{

  double cpsi = cos(Pose[2]);
  double spsi = sin(Pose[2]);
  BodyVel[0] = InertialVel[0]*cpsi + InertialVel[1]*spsi;
  BodyVel[1] = -InertialVel[0]*spsi + InertialVel[1]*cpsi;
  BodyVel[2] = InertialVel[2];

  return 0;
}
