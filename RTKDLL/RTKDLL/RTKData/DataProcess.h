#ifndef DATA_PROCESS_H
#define DATA_PROCESS_H
#include"DefData.h"
#include<map>
#include"../SatePos/SatePos.h"
// GPS时间常数
const double SECONDS_PER_WEEK = 604800.0;  // 7 * 24 * 3600
const double SECONDS_PER_DAY = 86400.0;    // 24 * 3600






double GregorianToGPSSOW(int year, int month, int day, int hour, int min, double sec);

double CalculateTimeDiffGPSSimple(int year1, int month1, int day1, int hour1, int min1, double sec1,
    int toe_week, double toe);
double CalculateTimeDiffGPSRobust(int year1, int month1, int day1, int hour1, int min1, double sec1,
    int toe_week, double toe);
map<string, vector<GPSEphData>> BuildEphMap(const vector<GPSEphData>& all_ephemeries);
void MatchSatePosPara(const GPSObsData obsdata, const map<string, vector<GPSEphData>> eph_map, SatePosPara& satepos);
void SateParaMatch(const vector<GPSEphData>& EphData, const vector<GPSObsData>& ObsData, vector<SatePosPara>& Satepospara);
double GregorianToBDSOW(int year, int month, int day, int hour, int min, double sec);
double CalculateTimeDiffBDSSimple(int year1, int month1, int day1, int hour1, int min1, double sec1,
	int toe_week, double toe);
double CalculateTimeDiffBDSRobust(int year1, int month1, int day1, int hour1, int min1, double sec1,
	int toe_week, double toe);
void BuildBDSEphMap(map<string, vector<BDSEphData>>& eph_map, const BDSEphData& eph_data);
map<string, vector<BDSEphData>> BuildBDSEphMap(const vector<BDSEphData>& all_ephemeries);
void MatchBDSSatePosPara(const BDSObsData obsdata, const map<string, vector<BDSEphData>> eph_map, BDSSatePosPara& satepos);
void BDSSateParaMatch(const vector<BDSEphData>& EphData, const vector<BDSObsData>& ObsData, vector<BDSSatePosPara>& Satepospara);
void TestTimeConversion();
#endif // !DATA_PROCESS_H
