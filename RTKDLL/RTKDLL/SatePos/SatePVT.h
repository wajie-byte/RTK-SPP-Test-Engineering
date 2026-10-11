#ifndef SATEPVT_H
#define SATEPVT_H
#include"../RTKData/DefData.h"
#include"../OEMRead/Parse.h"


bool BDSClock_Cal(BDSEPHEMERIS eph, BDSTime BDStime, double& Clock_bias, double& Clock_vel);
bool BDSSatPVT(BDSEPHEMERIS eph, BDSTime BDStime, Point& Satpos, double& Clock_bias, double& Clock_vel);
void SatPVT(OBSS obss, vector<SATE>& sates, Point Recpos);


#endif // !SATEPVT_H
