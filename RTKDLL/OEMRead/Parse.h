#ifndef PARSE_H
#define PARSE_H
#include"../RTKData/DefData.h"
#include<map>
#include "../Config/config.h"

#define GPS_SYS 0
#define BDS_SYS 4

#define GPS_L1C 0
#define GPS_L2P 5
#define GPS_L2P_Y 9

#define BDS_B1I_D1 0
#define BDS_B2I_D1 1
#define BDS_B3I_D1 2

#define GPS_L1_FREQ 1575.42e6
#define GPS_L2_FREQ 1227.6e6

#define BDS_B1_FREQ 1561.098e6
#define BDS_B3_FREQ 1268.52e6

#define CLIGHT 299792458.0

#define BDS_B1_WAVELEN (CLIGHT/BDS_B1_FREQ)
#define BDS_B3_WAVELEN (CLIGHT/BDS_B3_FREQ)
#define GPS_L1_WAVELEN (CLIGHT/GPS_L1_FREQ)
#define GPS_L2_WAVELEN (CLIGHT/GPS_L2_FREQ)

#pragma pack(push, 1)
struct BDSEPHEMERIS
{
	unsigned long ID;
	unsigned long Week;
	double URA;
	unsigned long health1;
	double tgd1;
	double tgd2;
	unsigned long AODC;
	unsigned long toc;
	double a0;
	double a1;
	double a2;
	unsigned long AODE;
	unsigned long toe;
	double RootA;
	double ecc;
	double omega;
	double deltaN;
	double M0;
	double Omega0;
	double OmegaDot;
	double i0;
	double IDOT;
	double Cuc;
	double Cus;
	double Crc;
	double Crs;
	double Cic;
	double Cis;
	// 默认构造函数 - 全零初始化
	BDSEPHEMERIS()
		: ID(0), Week(0), URA(0), health1(0), tgd1(0), tgd2(0),
		AODC(0), toc(0), a0(0), a1(0), a2(0), AODE(0), toe(0),
		RootA(0), ecc(0), omega(0), deltaN(0), M0(0), Omega0(0),
		OmegaDot(0), i0(0), IDOT(0), Cuc(0), Cus(0), Crc(0),
		Crs(0), Cic(0), Cis(0)
	{}


};
#pragma pack(pop)


#pragma pack(push, 1)
struct GPSEPHEM
{
	unsigned long PRN;
	double tow;
	unsigned long health;
	unsigned long IODE1;
	unsigned long IODE2;
	unsigned long week;
	unsigned long z_week;
	double toe;
	double A;
	double deltaN;
	double M0;
	double ecc;
	double omega;
	double cuc;
	double cus;
	double crc;
	double crs;
	double cic;
	double cis;
	double I0;
	double IDot;
	double omega0;
	double omegaDot;
	unsigned long iodc;
	double toc;
	double tgd;
	double af0;
	double af1;
	double af2;
	long AS;
	double N;
	double URA;
	
	
	// 默认构造函数 - 全零初始化
	GPSEPHEM()
		: PRN(0), tow(0.0), health(0), IODE1(0), IODE2(0),
		week(0), z_week(0), toe(0.0), A(0.0), deltaN(0.0),
		M0(0.0), ecc(0.0), omega(0.0), cuc(0.0), cus(0.0),
		crc(0.0), crs(0.0), cic(0.0), cis(0.0), I0(0.0),
		IDot(0.0), omega0(0.0), omegaDot(0.0), iodc(0),
		toc(0.0), tgd(0.0), af0(0.0), af1(0.0), af2(0.0),
		AS(0), N(0.0), URA(0.0)
	{
	}
	
		


};
#pragma pack(pop)

#pragma pack(push, 1)
struct Range
{
	unsigned short PRN_slot;
	unsigned short glofreq;
	double psr;
	float psr_sigma;
	double adr;
	float adr_sigma;
	float dopp;
	float C_No;
	float locktime;
	unsigned long ch_tr_status;
	Range()
		: PRN_slot(0),
		glofreq(0),
		psr(0),
		psr_sigma(0),
		adr(0),
		adr_sigma(0),
		dopp(0),
		C_No(0),
		locktime(0),
		ch_tr_status(0)
	{
	}
};
#pragma pack(pop)



#pragma pack(push,1)
struct IONUTC
{
	double a0; double a1; double a2; double a3;
	double b0; double b1; double b2; double b3;
	unsigned long utc_wn;
	unsigned long tot; 
	double A0; double A1;
	unsigned long wn_lsf;
	unsigned long dn; 
	long deltat_ls;
	long deltat_lsf;
	long Reserved;
	IONUTC()
		: a0(0), a1(0), a2(0), a3(0),
		b0(0), b1(0), b2(0), b3(0),
		utc_wn(0), tot(0), A0(0), A1(0),
		wn_lsf(0), dn(0), deltat_ls(0),
		deltat_lsf(0), Reserved(0)
	{
	}
};
#pragma pack(pop)


#pragma pack(push, 1)
struct RANGE
{
	unsigned long n_obs;
	vector<Range> obs;
	RANGE()
	{
		n_obs = 0;
	}
};
#pragma pack(pop)

struct OBS
{
	int system;
	int signal_type;
	int PRN;
	double psr;
	double psr_sigma;
	double adr_sigma;
	double adr;
	double dopp;
	double C_NO;
	OBS()
	{
		system = -1;
		signal_type = -1;
		PRN = -1;
		psr = 0.0;
		psr_sigma = 0.0;
		adr = 0.0;
		adr_sigma = 0.0;
		dopp = 0.0;
		C_NO = 0.0;
	}
};

struct OBSS
{
	GPSTime GPStime;
	BDSTime BDStime;
	int n_obs;
	vector<OBS> obs;
	OBSS()
	{
		n_obs = 0;
	}
};

struct SATE
{
	int sys;
	GPSTime GPStime;
	BDSTime BDStime;
	YMDHMS UTC;
	Point satepos;
	double Clock_bias;
	double Clock_vel;
	BDSEPHEMERIS BDSEPH;
	GPSEPHEM GPSEPH;
	
	SATE()
	{
		sys = -1;
		Clock_bias = 0.0;
		Clock_vel = 0.0;
	}
};

// 定义结构体存储双频观测值和误差
struct DualFreqData 
{
	double psr_b1;      // B1I伪距观测值
	double psr_b3;      // B3I伪距观测值
	double sigma_b1;    // B1I测距误差（标准差）
	double sigma_b3;    // B3I测距误差（标准差）
	bool has_b1;        // 是否有B1观测
	bool has_b3;        // 是否有B3观测

	DualFreqData() : psr_b1(0.0), psr_b3(0.0),
		sigma_b1(0.0), sigma_b3(0.0),
		has_b1(false), has_b3(false) {}
};

//存储RTK解算得到的解算状态，基线和卫星等信息
struct RTKResult
{
	GPSTime GPStime; // GPS时间
	int status; // 解算状态,如果为-1表示解算失败，0表示浮点解，1表示固定解
	double ratio; // 定位精度比率
	double baseline_Float[3]; // 基线向量浮点解
	double baseline_Fixed[3]; // 基线向量固定解
	double sigma0_squared; // 单位权方差,浮点解计算得到
	double Float_std[3]; // 浮点解标准差
	double Fixed_std[3]; // 固定解标准差
	string ref_sat; // 参考卫星PRN
	double ref_elev; // 参考卫星仰角
	vector<string> other_prns; // 其他卫星PRN列表
	vector<double> other_elevs; // 卫星仰角列表
	vector<double> N_Float; // 浮点模糊度列表,数量为卫星数量两倍，对应双频
	vector<double> N_Fixed; // 固定模糊度列表,数量为卫星数量两倍，对应双频
	RTKResult() : status(-1)
	{
		ref_sat = "Unknown";
		ref_elev = 0.0;
		ratio = 0.0;
		sigma0_squared = 0.0;
		baseline_Float[0] = 0.0; baseline_Float[1] = 0.0; baseline_Float[2] = 0.0;
		baseline_Fixed[0] = 0.0; baseline_Fixed[1] = 0.0; baseline_Fixed[2] = 0.0;
		Float_std[0] = 0.0; Float_std[1] = 0.0; Float_std[2] = 0.0;
		Fixed_std[0] = 0.0; Fixed_std[1] = 0.0; Fixed_std[2] = 0.0;
	}
};


struct RTKPLOT
{
	int status;
	double GPSsec;
	double ratio;
	double baseline_xyz[3];
	double baseline_enu[3];
	double sat_num;
	RTKPLOT()
	{
		GPSsec = 0.0;
		ratio = 0.0;
		status = -1;
		baseline_xyz[0] = 0.0; baseline_xyz[1] = 0.0; baseline_xyz[2] = 0.0;
		baseline_enu[0] = 0.0; baseline_enu[1] = 0.0; baseline_enu[2] = 0.0;
		sat_num = 0.0;
	}
};



BDSEPHEMERIS parseBDSEPHEMERIS(const vector<uint8_t>& body, bool& is_parse);
GPSEPHEM parseGPSEPHEM(const vector<uint8_t>& body, bool& is_parse);
OBSS parseRANGE(const vector<uint8_t>& body, bool& is_parse);
IONUTC parseIONUTC(const vector<uint8_t>& body, bool& is_parse);
bool decode_ch_tr_status(uint32_t ch_tr_status, int& system, int& signal_type);
OBS decodeRANGE(const Range range, int system, int signal_type);
BDSEphData Trans_BDSEPHEMERIS(const BDSEPHEMERIS BDSeph);
bool Match_GPS_OBS_AND_EPH(const OBSS& obss, const map<string, vector<GPSEPHEM>>& eph_map, vector<SATE>& Sates, OBSS& valid_obss);
int Match_BDS_GPS_OBS_AND_EPH(const OBSS& obss, const map<string, vector<BDSEPHEMERIS>>& bds_eph_map, const map<string, vector<GPSEPHEM>>& gps_eph_map
	, vector<SATE>& Sates, OBSS& valid_obss);
BDSSatePosPara Match_BDSOBS_AND_BDSEPH(const OBSS obss, const map<string, vector<BDSEphData>> eph_map);
vector<BDSSatePosPara> BDSMatch(const vector<OBSS> OBSEpoch, const map<string, vector<BDSEphData>> eph_map);
bool Match_BDS_OBS_AND_EPH(const OBSS& obss, const map<string, vector<BDSEPHEMERIS>>& eph_map, vector<SATE>& Sates, OBSS& valid_obss, const ParamConfig param_config);
string GetSatPRN(int system, int PRN);
void RTKResultWrite(const RTKResult& rtk_result, ofstream& fout);
#endif // !PARSE_H
