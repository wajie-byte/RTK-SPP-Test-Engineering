#ifndef SPP_H
#define SPP_H
#include"../RTKData/DefData.h"
#include"../SatePos/SatePos.h"
#include"../MATDLL/Mat.h"
const static double minPseudorange = 10000000.0;
const static double maxPseudorange = 30000000.0;

void SatPVTsignal(SatePosPara& para, Point Recpos);
void SPP(SatePosPara& para, SPPResult& lsresult,Point& Recpos);
void SPPLine(Mat Recpos, double num, SatePosPara& valid, Mat& B, Mat& L, Mat& W);
void GPS_SPP(vector<SatePosPara>& satePospara, vector<SPPResult>& LSresult);
void GetValidSat(SatePosPara para, SatePosPara& valid);
void LeastSquares(SatePosPara& valid, SPPResult& lsresult, Point& Recpos);
Mat computeElevationWeight(const vector<Point>& satPositions, const Point& recPos);
bool GetBDSValidSat(BDSSatePosPara para, BDSSatePosPara& valid);
void BDSSatPVTsignal(BDSSatePosPara& para, Point Recpos);
void BDS_SPPLine(Mat Recpos, double num, BDSSatePosPara& valid, Mat& B, Mat& L, Mat& W);
void BDS_LeastSquares(BDSSatePosPara& valid, SPPResult& lsresult, Point& Recpos);
void bds_SPP(BDSSatePosPara& para, SPPResult& lsresult, Point& Recpos, BDSSatePosPara& valid);
void BDS_SPP(vector<BDSSatePosPara>& satePospara, vector<SPPResult>& LSresult);
#endif // !SPP_H
