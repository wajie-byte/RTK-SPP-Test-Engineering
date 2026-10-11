#ifndef SATEPOS_H
#define SATEPOS_H
#include"../RTKData/DefData.h"
// 卫星位置计算相关常量
const static double clight = 299792458.0; // 光速 (m/s)

const double OMEGA_E = 7.2921151467e-5;
const double GM_GPS = 3.986005e14;     // GPS引力常数 (m^3/s^2)
const double GM_BDS = 3.986004418e14;  // BDS引力常数 (m^3/s^2)
const double Ftr_GPS = -4.442807633e-10;
const double Ftr_BDS = -4.442807309e-10;
const double OMEGAeDOT_GPS= 7.2921151467e-5;
const double OMEGAeDOT_BDS= 7.2921150e-5;

double CalculateTimeDiffBDS(const BDSTime& obs_time, const BDSTime& eph_time);
void CriticalDiagnosis(vector<SatePosPara>& para);
double calculateApproximateRelativisticCorrection(const EphPara& eph, double dt_orbit);
double calculateTransmitTime(double receiverTime, double pseudorange, const EphPara& eph);
double computePreciseSatelliteClock(double transmitTime, const EphPara& eph);

bool Clock_Cal(int year, int month, int day, int hour, int min, double sec, EphPara eph, GPSTime GPStime, double& Clock_bias, double& Clock_vel);

bool GPSSatPVT(EphPara eph, GPSTime GPStime, Point& Satpos, double& Clock_bias, double& Clock_vel);
void EarRotCor(Point& Satpos, Point Recpos);
void BDSEarRotCor(Point& Satpos, Point Recpos);

double ComputeElevation(const Point& Satpos, const Point& Recpos);
void CalElevation(vector<Point>& Satpos, Point Recpos, vector<double>& elevation);
double ComputeAzimuth(const Point& Satpos, const Point& Recpos);

bool BDSClock_Cal(BDSEphPara eph, BDSTime BDStime, double& Clock_bias, double& Clock_vel);
bool BDSSatPVT(string BDSPRN, BDSEphPara eph, BDSTime BDStime, Point& Satpos, double& Clock_bias, double& Clock_vel);
#endif // !SATEPOS_H