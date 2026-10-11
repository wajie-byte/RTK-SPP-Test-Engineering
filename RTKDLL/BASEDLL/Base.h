#ifndef BASE_H
#define BASE_H
#include"../RTKData/DefData.h"
// 常量定义

const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_DEG = 180.0 / PI;

// WGS84椭球参数
const double WGS84_A = 6378137.0;          // 长半轴(m)
const double WGS84_F = 1.0 / 298.257223563; // 扁率
const double WGS84_E2 = 2 * WGS84_F - WGS84_F * WGS84_F; // 第一偏心率的平方
const double WGS84_E = sqrt(WGS84_E2);     // 第一偏心率





void XYZ2BLH(const double xyz[3], double blh[3]);
void BASELINE_XYZ2ENU(const double base_xyz[3], const double xyz[3], double enu[3]);


double gregorianToJulianDate(int year, int month, int day, int hour, int min, double sec);
bool julianDateToGPSTime(double jd, GPSTime& GPStime);
void gregorianToGPSTime(int year, int month, int day, int hour, int min, double sec, GPSTime& GPStime);
void handleDateCarry(int& year, int& month, int& day, int& hour, int& min, double& sec);
int daysInMonth(int year, int month);
bool isLeapYear(int year);
bool julianDateToBDSTime(double jd, BDSTime& BDStime);
void gregorianToBDSTime(int year, int month, int day, int hour, int min, double sec, BDSTime& BDStime);
YMDHMS BDT2UTC(const BDSTime bdt);
BDSTime gpsToBds(const GPSTime& gps_time);
GPSTime bdsToGps(const BDSTime& bds_time);

#endif // !BASE_H
