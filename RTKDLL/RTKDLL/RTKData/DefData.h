#ifndef DD_H
#define DD_H
#include<iostream>
using namespace std;
#include<vector>
#include<string>

//---------------------常数定义---------------------
const static double Freq_C2I = 1561.098e6;
#define PI  3.141592653589793


//GPS时间
struct GPSTime
{
	int week;
	double sow;
	GPSTime()
	{
		week = 0;
		sow = 0.0;
	}
};

//BDS时间
struct BDSTime
{
	int week;
	double sow;
	BDSTime()
	{
		week = 0;
		sow = 0.0;
	}
};

struct YMDHMS
{
	int year; int month; int day;
	int hour; int min; double sec;
	YMDHMS()
	{
		year = month = day = 0;
		hour = min = 0;
		sec = 0.0;
	}
};



//Point
struct Point
{
	double x; double y; double z;
	double vx; double vy; double vz;
	Point()
	{
		x = 0.0; y = 0.0; z = 0.0;
		vx = vy = vz = 0.0;
	}
};

//GPS星历参数
struct EphPara
{
	double a0; double a1; double a2;
	// 第2行数据
	double IODE; double Crs; double deltan; double M0;
	// 第3行数据
	double Cuc; double e; double Cus; double sqrtA;
	// 第4行数据
	double Toe; double Cic; double OMEGA; double Cis;
	// 第5行数据
	double i0; double Crc; double omega; double OMEGADOT;
	// 第6行数据
	double IDOT; double L2; double Toe_week; double L2P;
	// 第7行数据
	double precision; double health; double TGD; double IDOC;
	// 第8行数据
	double Toc; 
	EphPara()
	{
		a0 = 0.0; a1 = 0.0; a2 = 0.0;
		IODE = 0.0; Crs = 0.0; deltan = 0.0; M0 = 0.0;
		Cuc = 0.0; e = 0.0; Cus = 0.0; sqrtA = 0.0;
		Toe = 0.0; Cic = 0.0; OMEGA = 0.0; Cis = 0.0;
		i0 = 0.0; Crc = 0.0; omega = 0.0; OMEGADOT = 0.0;
		IDOT = 0.0; L2 = 0.0; Toe_week = 0.0; L2P = 0.0;
		precision = 0.0; health = 0.0; TGD = 0.0; IDOC = 0.0;
		Toc = 0.0; 
	}
};

//GPSEphData
struct GPSEphData
{
	// 第1行数据
	string PRN;//卫星号
	//时间
	int year;int month; int day;
	int hour; int min; double sec;
	EphPara ephpara;
	//构造函数
	GPSEphData()
	{
		PRN = "Unknown";
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
	}
};

//同一历元的GPS星历数据
struct EpochEph
{
	int year; int month; int day;
	int hour; int min; double sec;
	vector<GPSEphData> eph;
	EpochEph()
	{
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		eph = vector<GPSEphData>();
	}
};

//GPSObsData
struct GPSObsData
{
	vector<string> PRN;//卫星号
	//时间
	int year; int month; int day;
	int hour; int min; double sec;
	//观测值
	vector<double> C1C; vector<double> L1C; 
	//构造函数
	GPSObsData()
	{
		PRN = vector<string>();
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		C1C = vector<double>();
		L1C = vector<double>();
	}
	// 重置时间并清空观测数据
	void ResetTime(int y, int m, int d, int h, int mi, double s)
	{
		year = y; month = m; day = d;
		hour = h; min = mi; sec = s;
		PRN.clear();
		C1C.clear();
		L1C.clear();
	}
	// 添加卫星观测数据
	void AddObsData(const string& prn, double c1c, double l1c) 
	{
		PRN.push_back(prn);
		C1C.push_back(c1c);
		L1C.push_back(l1c);
	}
};

//DDObs
struct BDS_DDObs
{
	//时间
	int year; int month; int day;
	int hour; int min; double sec;
	
	vector<string> commonPRN;//共视卫星
	vector<double>elevation;//卫星高度角
	vector<Point> SatPos;
	Point BasePos;
	int ref_index;
	string refPRN;//参考星
	vector<string> otherPRN;//其他星
	vector<double> DDC2I;//双差伪距
	vector<double> DDL2I;//双差载波，单位周
	vector<double> DDL2I_m;//双差载波，单位m
	BDS_DDObs()
	{
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		SatPos = vector<Point>();
		BasePos = Point();
		commonPRN = vector<string>();
		elevation = vector<double>();
		refPRN = "Unknown";
		ref_index = 0;
		otherPRN = vector<string>();
		DDC2I = vector<double>();
		DDL2I = vector<double>();
		DDL2I_m = vector<double>();
	}
};



//SatePosPara
struct SatePosPara
{
	//需更新为信号发射时间
	int year; int month; int day;
	int hour; int min; double sec;
	GPSTime GPStime;
	vector<string> commonPRN;//共同卫星
	vector<EphPara> ephpara;
	vector<double> elevation;//仰角
	vector<double> C1C;//伪距
	vector<double> L1C;//载波
	vector<double>Clock_bias;
	vector<double>Clock_vel;
	vector<Point> Pos;
	SatePosPara()
	{
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		commonPRN = vector<string>();
		C1C = vector<double>();
		L1C = vector<double>();
		Clock_bias = vector<double>();
		Clock_vel = vector<double>();
		ephpara = vector<EphPara>();
		Pos = vector<Point>();
		GPStime = GPSTime();
	}
};

struct RecPosPara
{
	int year; int month; int day;
	int hour; int min; double sec;
	GPSTime GPStime;
	vector<string> commonPRN;//共同卫星
	vector<double> C1C;//伪距
	vector<double> fixed_C1C;
	vector<double> Clock_bias;
	
	vector<Point>SatePos;
	RecPosPara()
	{
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		commonPRN = vector<string>();
		C1C = vector<double>();
		SatePos = vector<Point>();
		fixed_C1C = vector<double>();
		Clock_bias = vector<double>();
		GPStime = GPSTime();
	}
};

// 最小二乘定位结果结构体
struct LSResult
{
	GPSTime GPStime;
	BDSTime BDStime;
	double est[4];//估计参数
	double var[4];//估计参数方差
	vector<string> sat_prns;//使用到的卫星
	vector<double> sat_elevs;//卫星高度角，单位度
	vector<Point> sat_pos;
	int numIterations;// 迭代次数
	bool converged;// 是否收敛
	double gdop;  //几何精度因子
	double pdop;  //位置精度因子

	LSResult()
	{
		for (int i = 0; i < 4; i++)
		{
			est[i] = 0.0;
			var[i] = 0.0;
		}
		numIterations = 0;
		converged = false;
		gdop = 0.0;
		pdop = 0.0;
	}
};



//BDS星历参数
struct BDSEphPara
{
	double a0; double a1; double a2;
	// 第2行数据
	double AODE; double Crs; double deltan; double M0;
	// 第3行数据
	double Cuc; double e; double Cus; double sqrtA;
	// 第4行数据
	double Toe; double Cic; double OMEGA0; double Cis;
	// 第5行数据
	double i0; double Crc; double omega; double OMEGADOT;
	// 第6行数据
	double IDOT; double spare1; double BDT_week; double spare2;
	// 第7行数据
	double accuracy; double health; double TGD1; double TGD2;
	// 第8行数据
	double Toc; double spare3;
	BDSEphPara()
	{
		a0 = 0.0; a1 = 0.0; a2 = 0.0;
		AODE = 0.0; Crs = 0.0; deltan = 0.0; M0 = 0.0;
		Cuc = 0.0; e = 0.0; Cus = 0.0; sqrtA = 0.0;
		Toe = 0.0; Cic = 0.0; OMEGA0 = 0.0; Cis = 0.0;
		i0 = 0.0; Crc = 0.0; omega = 0.0; OMEGADOT = 0.0;
		IDOT = 0.0; spare1 = 0.0; BDT_week = 0.0; spare2 = 0.0;
		accuracy = 0.0; health = 0.0; TGD1 = 0.0; TGD2 = 0.0;
		Toc = 0.0; spare3 = 0.0;
	}
};

struct BDSEphData
{
	// 第1行数据
	string PRN;//卫星号
	//时间
	int year;int month; int day;
	int hour; int min; double sec;
	BDSTime BDStime;
	BDSEphPara ephpara;
	//构造函数
	BDSEphData()
	{
		PRN = "Unknown";
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
	}
};

struct BDSEpochEph
{
	int year; int month; int day;
	int hour; int min; double sec;
	vector<BDSEphData> eph;
	BDSEpochEph()
	{
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		eph = vector<BDSEphData>();
	}
};

struct BDSObsData
{
	vector<string> PRN;//卫星号
	//时间
	int year; int month; int day;
	int hour; int min; double sec;
	//观测值
	vector<double> C2I; vector<double> L2I; 
	//构造函数
	BDSObsData()
	{
		PRN = vector<string>();
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		C2I = vector<double>();
		L2I = vector<double>();
	}
	// 重置时间并清空观测数据
	void ResetTime(int y, int m, int d, int h, int mi, double s)
	{
		year = y; month = m; day = d;
		hour = h; min = mi; sec = s;
		PRN.clear();
		C2I.clear();
		L2I.clear();
	}
	// 添加卫星观测数据
	void AddObsData(const string& prn, double c2i, double l2i) 
	{
		PRN.push_back(prn);
		C2I.push_back(c2i);
		L2I.push_back(l2i);
	}
};

struct BDSSatePosPara
{
	//需更新为信号发射时间
	int year; int month; int day;
	int hour; int min; double sec;
	BDSTime BDStime;
	vector<string> commonPRN;//共同卫星
	vector<BDSEphPara> ephpara;
	vector<double> elevation;//仰角
	vector<double> C2I;//伪距
	vector<double> L2I;//载波
	vector<double>Clock_bias;
	vector<double>Clock_vel;
	vector<Point> Pos;//卫星位置
	BDSSatePosPara()
	{
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		commonPRN = vector<string>();
		C2I = vector<double>();
		L2I = vector<double>();
		Clock_bias = vector<double>();
		Clock_vel = vector<double>();
		ephpara = vector<BDSEphPara>();
		Pos = vector<Point>();
		BDStime = BDSTime();
	}
};

struct BDSRecPosPara
{
	int year; int month; int day;
	int hour; int min; double sec;
	BDSTime BDStime;
	vector<string> commonPRN;//共同卫星
	vector<double> C2I;//伪距
	vector<double> L2I;//载波
	vector<double> elevation;//仰角
	
	vector<Point>SatePos;
	BDSRecPosPara()
	{
		year = 0; month = 0; day = 0;
		hour = 0; min = 0; sec = 0.0;
		BDStime = BDSTime();
		commonPRN = vector<string>();
		C2I = vector<double>();
		L2I = vector<double>();
		elevation = vector<double>();
		SatePos = vector<Point>();
	}
};




//
void PosDataGet(vector<SatePosPara>& satepospara, vector<RecPosPara>& recPospara);

#endif // !DD_H
