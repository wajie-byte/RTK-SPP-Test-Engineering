#ifndef NEWSPP_H
#define NEWSPP_H
#include"../MATDLL/Mat.h"
#include"../OEMRead/Parse.h"
#include<map>
#include<set>



void LeastSquares(OBSS obss, vector<SATE>& Sates, Point& Recpos, LSResult& lsresult, int mode);
void BDS_LS_Line(Mat Recpos, OBSS obss, const vector<SATE>& Sates, vector<string> valid_obs_prn, Mat& B, Mat& L, Mat& W);
void BDS_LS_Line_IF(Mat Recpos, OBSS obss, const vector<SATE>& Sates, set<int> dual_freq_sats, Mat& B, Mat& L, Mat& W);
void SPP_BDS_SYS(const OBSS& obss, vector<SATE>& Sates, Point& Recpos, LSResult& lsresult, int mode);
set<string> GetDualFreqSatellites(const OBSS& obss, const vector<SATE>& SATES, Point Recpos);
map<int, DualFreqData> GetPRN_OBS_WithSigma(const OBSS& obss);
double CalPR_B1I_B3I(double psr1, double psr3, double tgd1);
set<string> Sat_elev_check(const vector<SATE>& Sates, Point Recpos);
vector<string> Valid_obs_prn(const vector<SATE>& Sates, Point Recpos);
bool GetLSResultSat(const vector<SATE>& Sates, const Point& Recpos, string sat, LSResult& lsresult);
#endif // !NEWSPP_H
