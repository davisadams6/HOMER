#include "parameters.h"

paramset NomParam() {
  paramset Param;

  double PivotPHI[] = {0,2*M_PI/3,4*M_PI/3};
  double ThetaOffset[] = {0,0,0};
  int cast = 0;
  
  for(cast=0;cast<3;cast++) {
    Param.PivotR[cast] = 0.382651;
    Param.PivotPHI[cast] = PivotPHI[cast];
    Param.ThetaOffset[cast] = ThetaOffset[cast];
    Param.WheelRadius[cast][0] = 0.0381;
    Param.WheelRadius[cast][1] = 0.0381;
    Param.WheelBase[cast][0] = 0.0508;
    Param.WheelBase[cast][1] = 0.0508;
    Param.AxleOffset[cast] = 0.0508;
  }

  return Param;
};

