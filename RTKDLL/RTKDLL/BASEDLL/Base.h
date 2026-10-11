#ifndef BASE_H
#define BASE_H
#include"../RTKData/DefData.h"









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

#endif // !BASE_H
