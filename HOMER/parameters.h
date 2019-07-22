#include <math.h>
#define WHEEL_CPR (7*4000)
#define CASTOR_CPR 8192
#define CCount2Rad (2*M_PI/CASTOR_CPR)
#define WCount2Rad (2*M_PI/WHEEL_CPR)


typedef struct  {
  double PivotR[3];
  double PivotPHI[3];
  double ThetaOffset[3];
  double WheelRadius[3][2];
  double WheelBase[3][2];
  double AxleOffset[3];
} paramset;

paramset NomParam();


