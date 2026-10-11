#include"FileRead/RTKDataRead.h"
#include"RTKCAL/RTKSoution.h"
#include"RTKData/DefData.h"
#include"SatePos/SatePos.h"
#include"SPP/SPP.h"	
#include"RTKData/DataProcess.h"
#include"OEMRead/BinRead.h"
#include"OEMRead/Parse.h"
#include"RealTime/RealTime.h"
int main()
{

	//------------------------------------文件流---------------------------------------
	
	/*
	
	
	

	*/

	//文件路径
	string basenav = "D:\\visual studio\\RTK\\RINEX\\base.nav";
	string baseobs = "D:\\visual studio\\RTK\\RINEX\\base.obs";
	string rovenav = "D:\\visual studio\\RTK\\RINEX\\rove.nav";
	string roveobs = "D:\\visual studio\\RTK\\RINEX\\rove.obs";
	string fixed_basenav = "D:\\visual studio\\RTK\\RINEX\\fixed_base.nav";
	string fixed_rovenav = "D:\\visual studio\\RTK\\RINEX\\fixed_rove.nav";

	string testnav = "D:\\GNSS\\Data\\Data\\first\\NovatelOEM20211114-01-1.nav";
	string fixed_testnav = "D:\\GNSS\\Data\\Data\\first\\fixed_NovatelOEM20211114-01.nav";
	string testobs = "D:\\GNSS\\Data\\Data\\first\\NovatelOEM20211114-01-1.obs";
	//定义数据容器
	fix_File(basenav, fixed_basenav);
	vector<GPSEphData> baseEph;
	vector<GPSObsData> baseObs;
	vector<BDSEphData> BDSbaseEph;
	vector<BDSObsData> BDSbaseObs;
	vector<BDSObsData> BDSroverObs;
	//数据读取
	baseEph.clear();
	baseObs.clear();
	//GPSObsDataRead(baseobs,baseObs);
	//GPSEphDataRead(fixed_basenav,baseEph);
	BDSbaseObs.clear();
	BDSbaseEph.clear();
	BDSroverObs.clear();
	BDSObsDataRead(baseobs, BDSbaseObs);
	BDSObsDataRead(roveobs, BDSroverObs);
	BDSEphDataRead(fixed_basenav, BDSbaseEph);
	//卫星定位参数获取
	vector<SatePosPara> satepospara;
	vector<EpochEph> baseEphEpoch;
	//ClassifyEphDataByTime(baseEph,baseEphEpoch);
	vector<BDSSatePosPara> BDSsatepospara;
	vector<BDSEpochEph> BDSbaseEphEpoch;
	//ClassifyBDSEphDataByTime(BDSbaseEph, BDSbaseEphEpoch);

	//匹配星历和观测数据
	//SateParaMatch(baseEph, baseObs, satepospara);
	//BDSSateParaMatch(BDSbaseEph, BDSbaseObs, BDSsatepospara);

	//存储最小二乘定位结果
	vector<LSResult> LSResult;
	//GPS_SPP(satepospara, LSResult);
	//BDS_SPP(BDSsatepospara, LSResult);

	//
	vector<BDS_DDObs>BDS_DDobs;
	BDS_DDobs.clear();
	//MatchObservations(BDSsatepospara, BDSroverObs, BDS_DDobs);

	RTKSolution(BDSbaseEph, BDSbaseObs, BDSroverObs);
	
	

	


	
	//-------------------------------------RealTime---------------------------------------

	/*
	string rfilename = "D:\\visual studio\\RTK\\Bin\\NovatelOEM20211114-01.log";
	fstream rfile(rfilename, ios::in | ios::binary);
	vector<BinMessage> messages;
	//(rfilename);
	//实时
	//pos_mode , 0代表北斗双频无组合，1代表北斗双频消电离层组合
	int pos_mode = 1;
	//OEMRead_Universal(CeHuiXueYuanIP, pos_mode, true, CeHuiXueYuanPort1);
	//OEMRead_Universal(GongXueBuIP, true, GongXueBuPort);
	//OEMRead_Universal(rfilename, pos_mode, false, -1);
	//RealTime_RTK(CeHuiXueYuanIP, CeHuiXueYuanPort1, CeHuiXueYuanIP, CeHuiXueYuanPort2, pos_mode);
	//RealTime_RTK(CeHuiXueYuanIP, CeHuiXueYuanPort1, GongXueBuIP, GongXueBuPort, pos_mode);
	const char* filename1 = "D:\\visual studio\\RTK\\RTKData\\test1.txt";
	//运行四个小时
	int run_time_seconds = 4 * 60 * 60;
	RTK_RealTime(CeHuiXueYuanIP, CeHuiXueYuanPort1, GongXueBuIP, GongXueBuPort, pos_mode, filename1, run_time_seconds);
	*/
	

}