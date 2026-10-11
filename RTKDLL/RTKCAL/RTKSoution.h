#ifndef PTKSol_H
#define RTKSol_H
#include"../RTKData/DefData.h"
#include"../MATDLL/Mat.h"

const double Time_Tolerance = 1e-3; //时间容差，单位秒

//构建双差观测值
void GPSBDS_DDObsBuilder(vector<BDSSatePosPara>& Basedata, vector<BDSObsData>& Roverdata, vector<BDS_DDObs>& GPSDD);
// 按时间排序观测数据
void SortObsDataByTime(vector<GPSObsData>& obs_data);
//时间匹配判断函数
bool IsTimeMatched(const BDSSatePosPara& base, const BDSObsData& rover);
//判断基准站时间是否早于流动站
bool IsBaseEarlier(const BDSSatePosPara& base, const BDSObsData& rover);
//处理匹配的观测数据
void ProcessMatch(BDSSatePosPara& base, const BDSObsData& rover_obs, Point& base_recpos, BDS_DDObs& ddobs);
//  查找共视卫星
vector<string> FindCommonSatellites(const BDSSatePosPara& base, const BDSObsData& rover);
// 排序后使用双指针匹配
void MatchObservations(vector<BDSSatePosPara>& epocheph, const vector<BDSObsData>& obs, vector<BDS_DDObs> GPSDD);
// 计算双差观测值
void CalDoubleDif(const BDSSatePosPara& base_obs, const BDSObsData& rover_obs, BDS_DDObs& ddobs, double ref_index);
// 按时间分类星历数据
void ClassifyEphDataByTime(const vector<GPSEphData>& ephDataList, vector<EpochEph>& epochList);
void ClassifyBDSEphDataByTime(const vector<BDSEphData>& ephDataList, vector<BDSEpochEph>& epochList);
//匹配星历和观测数据
// 时间匹配函数（检查两个时间是否相同）

// 时间比较函数（检查eph时间是否早于obs时间）
bool TimeEarlier(const EpochEph& eph, const GPSObsData& obs);
// 处理匹配的历元数据
void ProcessPara(const EpochEph& ephEpoch, const GPSObsData& obsData, SatePosPara& para);
//寻找参考星
int FindReferenceSatellite(const vector<double>& elevations);
// 根据PRN号过滤基准站数据
void FilterBaseDataByPRN(const BDSSatePosPara& base, const vector<string>& commonPRN, BDSSatePosPara& base_common);
//根据PRN号过滤流动站数据
void FilterRoverDataByPRN(const BDSObsData& rover, const vector<string>& commonPRN, BDSObsData& rover_common);

void DDMatBuilder(const BDS_DDObs& ddobs, Mat& A, Mat& L, Mat& P);
void RTKSolve(const BDS_DDObs& ddobs);
//RTK解算
void RTKSolution(const vector<BDSEphData>& BDSbaseEph, const vector<BDSObsData>& BDSbaseObs, const vector<BDSObsData>& BDSroverObs);
void UpdatePositionWithFixedAmbiguity(Mat& X, const Mat& N_inv, const double* F_fixed, int n_ambi);
bool Match_Base_Rover(const BDSSatePosPara satepos, const vector<BDSObsData>& BDSroverObs, int& Rover_index);
#endif // !PTKSol_H
