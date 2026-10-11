#ifndef CONFIG_H
#define CONFIG_H
#include<string>

#define POSMODE_SINGLE 0
#define POSMODE_RTK 1
// 算法
enum class SolveType {
    SPP,    // 单点定位
    RTK     // 实时动态
};

// 卫星
enum class SatSys {
    GPS_ONLY,
    BDS_ONLY,
    GPS_BDS,
    ALL
};

// 频率直接带系统信息
enum class FreqType {
	// GPS
	GPS_L1,
	GPS_L2,
	GPS_L5,
	GPS_L1L2,   // 双频组合
	GPS_L1L5,

	// BDS
	BDS_B1,
	BDS_B2,
	BDS_B3,
	BDS_B1B2,
	BDS_B1B3,

	// GPS+BDS混合（SPP常用）
	GPS_L1_BDS_B1,   // GPS用L1，BDS用B1
	GPS_L1L2_BDS_B1B2
};

enum class IonoHandle {
	NONE,           // 不处理（单频用模型）
	MODEL,          // 模型改正（单频）
	ELIMINATE,      // 消电离层组合（双频）
	ESTIMATE        // 作为参数估计
};

// 配置
class SolveConfig {
public:
    SolveType type;      // 算法
    SatSys system;       // 系统
    FreqType freq;       // 频率
	IonoHandle iono;          // 电离层处理方式（隐含是否消电离层）
};



class FileConfig
{
public:
    //SPP
	//read from file
	std::string obs_file;
	std::string nav_file;
	std::string bin_file;
	//write to file
	std::string spp_result_file;
	//RTK
	//read from file
	std::string base_obs_file;
	std::string base_nav_file;
	std::string rover_obs_file;
	std::string rover_nav_file;
	//write to file
	std::string rtk_result_file;

	//RealTime
	bool is_network;
	//SPP
	std::string IP;
	int port;
	//RTK
	std::string server_ip1;
	int server_port1;
	std::string server_ip2;
	int server_port2;


	FileConfig() : obs_file("Unknown"), nav_file("Unknown"), base_obs_file("Unknown"),
		base_nav_file("Unknown"), rover_obs_file("Unknown"), rover_nav_file("Unknown") ,
		IP("Unknown"), port(0)
	{ }
	~FileConfig() = default;
};

class ParamConfig
{
public:
	double elev_min; // 卫星仰角最小值
	double elev_max; // 卫星仰角最大值
	double sigma0;   // 单位权方差
	double ratio_min; // 定位精度比率最小值
	double CN0_min; // 载噪比最小值
	double runtime_seconds; // 运行时长，单位为秒，0表示无限制


	ParamConfig() : elev_min(0.0), elev_max(90.0), sigma0(1.0), ratio_min(3.0), CN0_min(35.0), runtime_seconds(0.0)
	{
	}
	~ParamConfig() = default;

};



class GNSSConfig
{

	
public:
	SolveConfig solve_config;
	FileConfig file_config;
	ParamConfig param_config;
	void InitConfig();
};

#endif