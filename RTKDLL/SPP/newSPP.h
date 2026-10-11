#ifndef NEWSPP_H
#define NEWSPP_H
#include"../MATDLL/Mat.h"
#include"../OEMRead/Parse.h"
#include<map>
#include<set>

// 定义椭球参数结构体
struct EllipsoidParams
{
    double a;  // 长半轴 (m)
    double e2; // 第一偏心率平方
};

// 定义地理坐标结构体
struct Geodetic
{
    double lat; // 纬度 (度)
    double lon; // 经度 (度)
    double h;   // 高度 (m)
};


const EllipsoidParams WGS84 = { 6378137.0, 0.00669437999014 }; // WGS-84椭球参数


void LeastSquares(OBSS obss, vector<SATE>& Sates, IONUTC& ionutc,Point& Recpos, SPPResult& lsresult, int n_sys, SolveConfig solve_config);
bool LS_Line(Mat Recpos, const OBSS& obss, const vector<SATE>& Sates, const IONUTC& ionutc, vector<string> valid_obs_prn, Mat& B, Mat& L, Mat& W, int n_sys);
bool BDS_LS_Line_IF(Mat Recpos,const OBSS& obss, const vector<SATE>& Sates, vector<string> dual_freq_vec, Mat& B, Mat& L, Mat& W);
void SPP_BDS_SYS(const OBSS& obss, vector<SATE>& Sates,IONUTC& ionutc, Point& Recpos, SPPResult& lsresult, SolveConfig solve_config);
set<string> GetDualFreqSatellites(const OBSS& obss, const vector<SATE>& SATES, Point Recpos);
map<int, DualFreqData> GetPRN_OBS_WithSigma(const OBSS& obss);
double CalPR_B1I_B3I(double psr1, double psr3, double tgd1);
set<string> Sat_elev_check(const vector<SATE>& Sates, Point Recpos);
vector<string> Valid_obs_prn(const vector<SATE>& Sates, Point Recpos);
bool GetSPPResultSat(const vector<SATE>& Sates, const Point& Recpos, string sat, SPPResult& lsresult);
void disp_spp(const SPPResult& spp_result);
bool SPVLine(double* Recpos, const OBSS& obss, const vector<SATE>& Sates, vector<string> valid_obs_prn, Mat& B, Mat& L, Mat& W, int n_sys);
void SPV_LS(const OBSS& obss, const vector<SATE>& Sates, const Point& Recpos, SPPResult& lsresult, int n_sys);
void SPP_GPS_SYS(const OBSS& obss, vector<SATE>& Sates, IONUTC& ionutc, Point& Recpos, SPPResult& lsresult, SolveConfig solve_config);
void SPP_SYS(const OBSS& obss, vector<SATE>& Sates, IONUTC& ionutc, Point& Recpos, SPPResult& lsresult, int n_sys, SolveConfig solve_config);
bool GetValidSatIndex(const vector<SATE>& Sates, vector<string> valid_obs_prn, vector<int>& valid_index);
double KlobucharModel_GPS(double user_lat_rad, double user_lon_rad,
    double el_rad, double az_rad,
    const double alpha[4], const double beta[4],
    double gps_week_sec);
double HopfieldTroposphere(double lat_rad, double h_m, double el_rad,
    double T0 = 288.16, double P0 = 1013.25,
    double RH0 = 50.0, double h0_m = 0.0);
Geodetic XYZ2BLH(double X, double Y, double Z, const EllipsoidParams& ellip);
#endif // !NEWSPP_H
