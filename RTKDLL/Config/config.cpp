#include"config.h"

void  GNSSConfig::InitConfig()
{
	// 这里可以添加一些默认配置的初始化
	//-------------------------SOLVE-----------------------------
	solve_config.type = SolveType::SPP; // 默认算法
	solve_config.system = SatSys::GPS_ONLY;   // 默认系统
	solve_config.freq = FreqType::GPS_L1;    // 默认频率
	//-------------------------FILE-----------------------------
	file_config.obs_file = "Unknown";
	file_config.nav_file = "Unknown";
	file_config.spp_result_file = "Unknown";
	file_config.base_obs_file = "Unknown";
	file_config.base_nav_file = "Unknown";
	file_config.rover_obs_file = "Unknown";
	file_config.rover_nav_file = "Unknown";
	file_config.rtk_result_file = "Unknown";
	file_config.IP = "Unknown";
	file_config.port = 0;
	//-------------------------PARAM-----------------------------
	param_config.elev_min = 0.0;
	param_config.elev_max = 90.0;
	param_config.sigma0 = 1.0;
	param_config.ratio_min = 3.0;
	param_config.CN0_min = 35.0;
}