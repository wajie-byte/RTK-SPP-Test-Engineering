#include"FileRead/RTKDataRead.h"
#include"RTKCAL/RTKSoution.h"
#include"RTKData/DefData.h"
#include"SatePos/SatePos.h"
#include"SPP/SPP.h"	
#include"RTKData/DataProcess.h"
#include"OEMRead/BinRead.h"
#include"OEMRead/Parse.h"
#include"RealTime/RealTime.h"
#include"Config/config.h"
int main()
{

	GNSSConfig gnssconfig;
	gnssconfig.InitConfig();


	string rfilename = "D:\\visual studio\\RTK\\Bin\\NovatelOEM20211114-01.log";
	fstream rfile(rfilename, ios::in | ios::binary);
	vector<BinMessage> messages;
	//(rfilename);
	//实时
	//pos_mode , 0代表北斗双频无组合，1代表北斗双频消电离层组合,2代表GPS双频无组合
	int pos_mode = 2;
	
	
	//事后仿实时
	const char* base_file = "D:\\visual studio\\RTK\\LOG\\base.log";
	const char* rover_file = "D:\\visual studio\\RTK\\LOG\\rove.log";
	const char* rtk_file = "D:\\visual studio\\RTK\\RTKData\\rtk_result.txt";
	//RTK_FILE_OEM7(gnssconfig);
	

	
	//spp实时解算
	//OEMRead_Universal(GongXueBuIP, pos_mode, true, GongXueBuPort);
	//OEMRead_Universal(CeHuiXueYuanIP, pos_mode, true, CeHuiXueYuanPort1);
	//OEMRead_Universal(base_file, pos_mode, false, 0);
	


	//rtk实时采集四个小时
	int run_time_seconds = 4 * 60 * 60;
	const char* filename1 = "D:\\visual studio\\RTK\\RTKData\\test2.txt";
	//RTK_RealTime(CeHuiXueYuanIP, CeHuiXueYuanPort1, GongXueBuIP, GongXueBuPort, pos_mode, filename1, run_time_seconds);
	//RTK_RealTime(CeHuiXueYuanIP, CeHuiXueYuanPort1, CeHuiXueYuanIP, CeHuiXueYuanPort2, pos_mode, filename1, run_time_seconds);


	//----------------------------------------------组合导航RTK解算----------------------------------------
	// 
	// 
	//已知基站精确坐标
	//首先进行RTK解算得到基线向量，然后通过基线向量和基站坐标计算出流动站坐标

	/*
	double base_BLH[3] = { 28.1825754300,112.9965413397,63.0388 };// [deg.deg, m]
	*/
	
	//RTKPOST数据转换成指定格式
	/*
	const char* rtkpost_b = "D:\\CombineNav\\data\\data\\pos\\MOVb.pos";
	const char* rtkpost_c = "D:\\CombineNav\\data\\data\\pos\\MOVc.pos";
	const char* rtkpost_d = "D:\\CombineNav\\data\\data\\pos\\MOVd.pos"; 
	const char* gnsstxt = "D:\\CombineNav\\data\\data\\pos\\GNSS.txt";
	RTKPOST2GNSSPOS(rtkpost_b, gnsstxt);
	RTKPOST2GNSSPOS(rtkpost_c, gnsstxt);
	RTKPOST2GNSSPOS(rtkpost_d, gnsstxt);
	*/
	/*const char* rtkpost_b = "D:\\CombineNav\\data\\data\\plus_vel\\MOVb.pos";
	const char* rtkpost_c = "D:\\CombineNav\\data\\data\\plus_vel\\MOVc.pos";
	const char* rtkpost_d = "D:\\CombineNav\\data\\data\\plus_vel\\MOVd.pos";
	const char* gnsstxt = "D:\\CombineNav\\data\\data\\plus_vel\\GNSSPLUSVEL_gap1s.txt";*/
	//RTKPOST2GNSSPOS_PLUSVEL(rtkpost_b, gnsstxt);
	//RTKPOST2GNSSPOS_PLUSVEL(rtkpost_c, gnsstxt);
	//RTKPOST2GNSSPOS_PLUSVEL(rtkpost_d, gnsstxt);
	// 
	/*const char* rtkpost = "D:\\2026大三下DHGC实习\\6.5\\gnss_20260605_090950_582671.pos";
	const char* gnsstxt = "D:\\2026大三下DHGC实习\\6.5\\GNSSPLUSVEL.txt";
	RTKPOST2GNSSPOS_PLUSVEL(rtkpost, gnsstxt);*/
	const char* rtkpost = "D:\\2026大三下DHGC实习\\6.5\\gnss_20260605_090950_582671.pos";
	const char* gnsstxt = "D:\\2026大三下DHGC实习\\6.5\\GNSSPLUSVEL.txt";
	POSMIND2GNSSPOS_PLUSVEL(rtkpost, gnsstxt);
	//---------------------------------------------Test----------------------------------------
	
	return 0;

}