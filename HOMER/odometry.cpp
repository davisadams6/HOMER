#include <math.h>
#include "parameters.h"

int UpdatePose(paramset Param, int OldWheelCounts[3][2], int NewWheelCounts[3][2], double Theta[3], double* Pose, double* CastorHeading )
{

  double DeltaDist[2];
  double DU,DT,DX,DY,DeltaWheelBase;
  double OldCastorHeading[3];
  double cCH,sCH,cCHold,sCHold,RcosPpP,RsinPpP;
  double Hy[3] = {0,0,0};
  double m_x=0, m_y=0, sumR2=0;

  double cpsi = cos(Pose[2]);
  double spsi = sin(Pose[2]);
  int c1, c2, c3, c4;

  for(c1=0;c1<3;c1++) {
    // wheel counts come in opposite of sign convention --> Old-New instead of New-Old
    DeltaDist[0] = WCount2Rad*(OldWheelCounts[c1][0] - NewWheelCounts[c1][0])*Param.WheelRadius[c1][0]; // left wheel
    DeltaDist[1] = WCount2Rad*(OldWheelCounts[c1][1] - NewWheelCounts[c1][1])*Param.WheelRadius[c1][1]; // right wheel

    DU = (DeltaDist[0] - DeltaDist[1])/2;
    DT = -(DeltaDist[0] + DeltaDist[1])/(Param.WheelBase[c1][0]+Param.WheelBase[c1][1]);

    OldCastorHeading[c1] = CastorHeading[c1];  
    CastorHeading[c1] += DT;

    cCH = cos(CastorHeading[c1]);
    sCH = sin(CastorHeading[c1]);
    cCHold = cos(OldCastorHeading[c1]);
    sCHold = sin(OldCastorHeading[c1]);
   
    DeltaWheelBase = (Param.WheelBase[c1][1]-Param.WheelBase[c1][0])/2;
    DX = DU*cCH + Param.AxleOffset[c1]*(cCH-cCHold) + DeltaWheelBase*(sCH-sCHold);
    DY = DU*sCH + Param.AxleOffset[c1]*(sCH-sCHold) - DeltaWheelBase*(cCH-cCHold);

    RcosPpP = Param.PivotR[c1]*cos(Pose[2]+Param.PivotPHI[c1]);
    RsinPpP = Param.PivotR[c1]*sin(Pose[2]+Param.PivotPHI[c1]);

    Hy[0] += DX;
    Hy[1] += DY;
    Hy[2] += -DX*RsinPpP + DY*RcosPpP;

    m_x += Param.PivotR[c1]*cos(Param.PivotPHI[c1]);
    m_y += Param.PivotR[c1]*sin(Param.PivotPHI[c1]);
    sumR2 += pow(Param.PivotR[c1],2);
  }

  double a = 3;
  double b = -(cpsi*m_y + spsi*m_x);
  double c = cpsi*m_x - spsi*m_y;
  double d = sumR2;
  double den = (-a*d+pow(c,2)+pow(b,2));

  double invHH[3][3] = {{pow(c,2)/a-d, -c*b/a, b},
                        {-c*b/a, pow(b,2)/a-d, c},
                        {b, c, -a}};

  for(c2=0;c2<3;c2++)
    Hy[c2] = Hy[c2]/den;

  for(c3=0;c3<3;c3++) {
    Pose[0] += invHH[0][c3]*Hy[c3];
    Pose[1] += invHH[1][c3]*Hy[c3];
    Pose[2] += invHH[2][c3]*Hy[c3];
  }
    
  for(c4=0;c4<3;c4++) 
    CastorHeading[c4] = Pose[2] + Theta[c4];

  return 0;
}
