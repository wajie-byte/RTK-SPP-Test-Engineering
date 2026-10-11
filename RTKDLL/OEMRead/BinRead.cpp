#include"BinRead.h"
#include"Parse.h"
#include"../BASEDLL/Base.h"
#include"../SPP/SPP.h"
#include"../SPP/newSPP.h"
#include"../RealTime/RealTime.h"

// 查找同步字节
bool FindBinSync(std::ifstream& rfile)
{
	uint8_t byte = 0;
	int syncnum = 0;
	while (rfile.read(reinterpret_cast<char*>(&byte), 1))//逐字节读取文件，每次读取一个字节
	{
		if (byte == SYNC_BYTES[syncnum])//检验读取到的字节是否与同步字节匹配
		{
			syncnum++;//匹配成功，则将计数器加一，并将目标指向下一个同步字节
			if (syncnum == 3)//如果三个同步字节都匹配成功，则表示找到了同步字节
			{
				rfile.seekg(-3, std::ios::cur);// 找到同步字节，将文件指针重置到同步字节开始位置
				return true;
			}
		}
		else//如果下一个不匹配，则将计数器重置为0，继续查找
		{
			syncnum = 0;
		}
	}
	return false;
}

// 读取OEM7消息
void OEMRead(GNSSConfig gnssconfig)
{
	std::string filename = gnssconfig.file_config.bin_file;
	ifstream rfile1(filename, std::ios::binary);
	if (!rfile1.is_open())
	{
		throw runtime_error("无法打开文件: " + filename);
	}
	


	// ===================== 声明存储变量 =====================
	// 基准站的星历数据，map的key为卫星号，value为对应的星历数据
	std::map<std::string, std::vector<BDSEPHEMERIS>> bds_eph_map1;
	std::map<std::string, std::vector<GPSEPHEM>> gps_eph_map1;

	// 基准站的位置
	Point base_pos;
	vector<OBSS> base_obss_vec;
	vector<SPPResult> base_lsresult_vec;
	//赋值近似数据
	base_pos.x = -2267700.0; base_pos.y = 5009400.0; base_pos.z = 3221000.0;
	int message_count1 = 0;
	int valid_message_count1 = 0;

	IONUTC ionutc1;
	
	


	//-------------------------------------首先解算基准站数据--------------------------------------
	//消息包数量和有效消息包数量

	while (FindBinSync(rfile1))
	{
		BinMessage message;
		message_count1++;
		// 读取头部
		rfile1.read(reinterpret_cast<char*>(&message.header), sizeof(BinHeader));
		if (rfile1.gcount() != sizeof(BinHeader))
		{
			std::cerr << "警告：读取头部失败，长度错误，跳过此消息" << std::endl;
			continue; // 只使用continue，跳过这个错误
		}

		// 读取消息体
		message.body.resize(message.header.message_length);
		rfile1.read(reinterpret_cast<char*>(message.body.data()), message.header.message_length);
		if (rfile1.gcount() != message.header.message_length)
		{
			std::cerr << "警告：读取消息体失败，长度错误，跳过此消息" << std::endl;
			continue; // 跳过错误
		}

		// 读取CRC
		rfile1.read(reinterpret_cast<char*>(&message.crc), sizeof(uint32_t));
		if (rfile1.gcount() != sizeof(uint32_t))
		{
			std::cerr << "警告：读取CRC失败，长度错误，跳过此消息" << std::endl;
			continue; // 跳过错误
		}

		// 验证CRC
		message.crc_valid = verifyCRC(message);



		//--------------------------------------解码消息---------------------------------


		if (message.crc_valid)
		{

			OBSS obss;
			OBSS valid_obss;
			SPPResult lsresult;
			bool is_eph = false, is_obs = false;
			Decode(message, bds_eph_map1, gps_eph_map1, obss, ionutc1, is_eph, is_obs);

			if (is_obs && Single_POS(obss, bds_eph_map1, gps_eph_map1, valid_obss,ionutc1, base_pos, lsresult, gnssconfig))
			{

				base_obss_vec.push_back(valid_obss);
				base_lsresult_vec.push_back(lsresult);
				//输出基准站定位结果
				disp_spp(lsresult);
			}
			valid_message_count1++;

		}

	}
	rfile1.close();
	std::cout << "总消息包数量: " << message_count1 << endl;
	std::cout << "有效消息包数量: " << valid_message_count1 << endl;


	return;

}

// 生成CRC32查找表，返回指向数组的指针
uint32_t* generateCRC32Table()
{
	static uint32_t table[256];
	uint32_t polynomial = 0xEDB88320;

	for (uint32_t i = 0; i < 256; i++)
	{
		uint32_t crc = i;
		for (int j = 0; j < 8; j++)
		{
			if (crc & 1)
				crc = (crc >> 1) ^ polynomial;
			else
				crc = crc >> 1;
		}
		table[i] = crc;
	}
	return table;
}
// 获取CRC32表的函数
uint32_t* getCRC32Table() 
{
	static uint32_t* table = nullptr;  // 静态指针，确保只生成一次
	if (table == nullptr) //确认表是否已经生成
	{
		table = generateCRC32Table();//若未生成，则调用生成函数
	}
	return table;
}
// 计算数据的CRC32值
uint32_t calculateCRC32(const unsigned char* data, int length)
{
	uint32_t* table = getCRC32Table();  // 获取CRC32表
	uint32_t crc = 0;
	for (int i = 0; i < length; ++i) 
	{
		unsigned char byte = data[i];
		// 查表计算
		uint32_t tableIndex = (crc ^ byte) & 0xFF;
		crc = (crc >> 8) ^ table[tableIndex];
	}
	return crc;  // 最终异或操作
}
//CRC校验
bool verifyCRC(const BinMessage& msg)
{
	if (msg.body.empty())
	{
		return false;
	}
	try
	{
		// 将BinHeader结构体序列化为字节流
		std::vector<uint8_t> header_bytes;
		header_bytes.resize(sizeof(BinHeader)); // 分配固定大小
		// 将结构体内容拷贝到vector中
		const uint8_t* header_ptr = reinterpret_cast<const uint8_t*>(&msg.header);
		header_bytes.assign(header_ptr, header_ptr + sizeof(BinHeader));

		// 合并头部和消息体
		std::vector<uint8_t> data;
		data.reserve(header_bytes.size() + msg.body.size());
		data.insert(data.end(), header_bytes.begin(), header_bytes.end());
		data.insert(data.end(), msg.body.begin(), msg.body.end());
		
		// 计算整个数据的CRC
		uint32_t calculated_crc = calculateCRC32(data.data(), data.size());

		
		// 输出详细比较信息
		/*
		std::cout << "=== CRC验证结果 ===" << std::endl;
		std::cout << "消息体长度: " << msg.body.size() << " 字节" << std::endl;
		std::cout << "Message_ID: " << msg.header.message_id << std::endl;
		std::cout << "Message_Type: " << static_cast<int>(msg.header.message_type) << std::endl;
		std::cout << "Header_length: " << static_cast<int>(msg.header.header_length) << std::endl;

		//std::cout << "计算CRC: 0x" << std::hex << std::setw(8) << std::setfill('0') << calculated_crc << std::endl;
		///std::cout << "原始CRC: 0x" << std::hex << std::setw(8) << std::setfill('0') << msg.crc << std::endl;
		std::cout << "验证结果: " << (calculated_crc == msg.crc ? "通过" : "失败") << std::endl;
		std::cout << "===================" << std::dec << std::endl;
		*/
		
		
		return calculated_crc == msg.crc;
	}
	catch (const std::exception& e)
	{
		std::cerr << "CRC验证失败: " << e.what() << std::endl;
		return false;
	}
}

bool DecodeBDSEphEMERIS(const BinMessage& message,
	map<string, vector<BDSEPHEMERIS>>& eph_map,double exptime)
{
	if (message.header.message_id != BDSEPHEMERIS_ID) return false;

	bool is_parse = false;
	BDSEPHEMERIS BDSEph = parseBDSEPHEMERIS(message.body, is_parse);

	if (!is_parse || BDSEph.health1 != 0) return false;

	// 生成PRN键
	char prn_key[10];
	snprintf(prn_key, sizeof(prn_key), "C%02d", BDSEph.ID);

	// 检查是否已存在
	bool exists = false;
	for (const auto& eph : eph_map[prn_key]) 
	{
		if (BDSEph.Week == eph.Week && BDSEph.toe == eph.toe) 
		{
			exists = true;
			break;
		}
		
		
	}

	if (!exists)
	{
		//添加星历
		BDSTime cur_BDT;
		GPSTime cur_GPST;
		cur_GPST.week = message.header.gps_week;
		cur_GPST.sow = message.header.gps_seconds / ms2s;
		cur_BDT = gpsToBds(cur_GPST);
		

		// 删除过期星历
		auto& eph_list = eph_map[prn_key];
		auto it = eph_list.begin();
		while (it != eph_list.end())
		{
			double eph_time_diff = (cur_BDT.week - it->Week) * SecInWeek + (cur_BDT.sow - it->toe);
			// 如果时间差超过3600秒（1小时），则认为星历过期
			if (fabs(eph_time_diff) > exptime)
			{
				std::cout << "删除卫星 " << prn_key << " 的过期星历"
					<< ", BDT:" << it->Week << "  Toe:" << it->toe
					<< ", 时间差:" << eph_time_diff << "秒" << endl;
				it = eph_list.erase(it);
			}
			else
			{
				++it;
			}
		}
		double cur_eph_diff = (cur_BDT.week - BDSEph.Week) * SecInWeek + (cur_BDT.sow - BDSEph.toe);
		if (fabs(cur_eph_diff) <= exptime)
		{
			eph_map[prn_key].push_back(BDSEph);
			std::cout << "已添加卫星 " << prn_key << " 的星历数据" << ", Week:" << BDSEph.Week << "  Toe:" << BDSEph.toe << endl;
			return true;
		}
		
		
	}

	//检查前面的星历是否已经过期，若过期则删除
	

	return false;
}

bool DecodeGPSEphEM(const BinMessage& message,
	map<string, vector<GPSEPHEM>>& eph_map,double exptime)//expdate星历保质期
{
	if (message.header.message_id != GPSEPHEM_ID) return false;

	bool is_parse = false;
	GPSEPHEM GPSEph = parseGPSEPHEM(message.body, is_parse);

	if (!is_parse || GPSEph.health != 0) return false;

	// 生成PRN键
	char prn_key[10];
	snprintf(prn_key, sizeof(prn_key), "G%02d", GPSEph.PRN);

	// 检查是否已存在
	bool exists = false;
	for (const auto& eph : eph_map[prn_key])
	{
		if (GPSEph.week == eph.week && GPSEph.toe == eph.toe)
		{
			exists = true;
			break;
		}


	}

	if (!exists)
	{
		
		GPSTime cur_GPST;
		cur_GPST.week = message.header.gps_week;
		cur_GPST.sow = message.header.gps_seconds / ms2s;


		// 删除过期星历
		auto& eph_list = eph_map[prn_key];
		auto it = eph_list.begin();
		while (it != eph_list.end())
		{
			double eph_time_diff = (cur_GPST.week - it->week) * SecInWeek + (cur_GPST.sow - it->toe);
			// 如果时间差超过7200秒（2小时），则认为星历过期
			if (fabs(eph_time_diff) > exptime)
			{
				std::cout << "删除卫星 " << prn_key << " 的过期星历"
					<< ", GPSTWeek:" << it->week << "  Toe:" << it->toe
					<< ", 时间差:" << eph_time_diff << "秒" << endl;
				it = eph_list.erase(it);
			}
			else
			{
				++it;
			}
		}
		double cur_eph_diff = (cur_GPST.week - GPSEph.week) * SecInWeek + (cur_GPST.sow - GPSEph.toe);
		if (fabs(cur_eph_diff) <= exptime)
		{
			eph_map[prn_key].push_back(GPSEph);
			std::cout << "已添加卫星 " << prn_key << " 的星历数据" << ", Week:" << GPSEph.week << "  Toe:" << GPSEph.toe << endl;
			return true;
		}


	}

	//检查前面的星历是否已经过期，若过期则删除


	return false;
}

bool DecodeObservations(const BinMessage& message, OBSS& obss)
{
	if (message.header.message_id != RANGE_ID) return false;

	bool is_parse = false;
	OBSS new_obss = parseRANGE(message.body, is_parse);

	if (!is_parse) return false;

	
	new_obss.GPStime.week = message.header.gps_week;
	new_obss.GPStime.sow = message.header.gps_seconds / ms2s;
	new_obss.BDStime = gpsToBds(new_obss.GPStime);

	
	obss = new_obss;

	return true;
}

bool DecodeIONUTC(const BinMessage& message, IONUTC& ionutc)
{
	if (message.header.message_id != IONUTC_ID) return false;
	bool is_parse = false;
	IONUTC new_ionutc = parseIONUTC(message.body, is_parse);
	if (!is_parse) return false;
	ionutc = new_ionutc;
	return true;
}


void Decode(const BinMessage& message, map<string, vector<BDSEPHEMERIS>>& bds_eph_map, map<string, vector<GPSEPHEM>>& gps_eph_map,
	OBSS& obss, IONUTC& ionutc, bool& is_eph, bool& is_obs)
{
	switch (message.header.message_id)
	{
	case BDSEPHEMERIS_ID:
	{
		if (DecodeBDSEphEMERIS(message, bds_eph_map,3600.0))
		{
			is_eph = true;
		}
		
		break;
	}
	case GPSEPHEM_ID:
	{
		if (DecodeGPSEphEM(message, gps_eph_map,7200.0))
		{
			is_eph = true;
		}

		break;
	}

	case RANGE_ID:
	{
		if (DecodeObservations(message, obss))
		{
			is_obs = true;
		}
		
		break;
	}
	case IONUTC_ID:
	{
		DecodeIONUTC(message, ionutc);
		break;
	}

	default: break;
	}
}



//void Decode(const BinMessage& message, std::vector<SATE>& sates, OBSS& obss, IONUTC& ionutc)
//{
//	std::map<string, vector<BDSEPHEMERIS>> bds_eph_map;
//	std::map<string, vector<GPSEPHEM>> gps_eph_map;
//	switch (message.header.message_id)
//	{
//	case BDSEPHEMERIS_ID:
//	{
//		DecodeBDSEphEMERIS(message, bds_eph_map, 3600.0);
//		break;
//	}
//	case GPSEPHEM_ID:
//	{
//		DecodeGPSEphEM(message, gps_eph_map, 7200.0);
//		break;
//	}
//
//	case RANGE_ID:
//	{
//		DecodeObservations(message, obss);
//		break;
//	}
//	case IONUTC_ID:
//	{
//		DecodeIONUTC(message, ionutc);
//
//	}
//
//	default: break;
//	}
//}


bool Single_POS(const OBSS& obss, map<string, vector<BDSEPHEMERIS>>& bds_eph_map, map<string, vector<GPSEPHEM>>& gps_eph_map
	, OBSS& valid_obss,IONUTC ionutc, Point& recpos, SPPResult& lsresult, GNSSConfig gnssconfig)
{
	vector<SATE> Sates;
	SolveConfig solveconfig = gnssconfig.solve_config;
	ParamConfig param_config = gnssconfig.param_config;

	if (solveconfig.system == SatSys::BDS_ONLY &&solveconfig.freq==FreqType::BDS_B1B2)
	{
		if (Match_BDS_OBS_AND_EPH(obss, bds_eph_map, Sates, valid_obss,param_config))
		{

			SPP_SYS(valid_obss, Sates, ionutc, recpos, lsresult, 1, solveconfig);
			if (lsresult.converged)
			{
				SPV_LS(valid_obss, Sates, recpos, lsresult,1);
				return true;
			}
			else return false;
		}
		else
		{
			cout << "星历数据不足" << endl;
			return false;
		}
	}
	else if (solveconfig.system == SatSys::GPS_ONLY && solveconfig.freq == FreqType::GPS_L1L2)
	{
		if (Match_GPS_OBS_AND_EPH(obss,gps_eph_map,Sates,valid_obss))
		{
			SPP_SYS(valid_obss, Sates, ionutc, recpos, lsresult, 1, solveconfig);
			if (lsresult.converged)
			{
				SPV_LS(valid_obss, Sates, recpos, lsresult,1);
				return true;
			}
			else return false;
		}

		else
		{
			cout << "星历数据不足" << endl;
			return false;
		}
	}
	else if (solveconfig.system == SatSys::GPS_BDS && solveconfig.freq == FreqType::GPS_L1L2_BDS_B1B2)
	{
		int n_sys = Match_BDS_GPS_OBS_AND_EPH(obss, bds_eph_map, gps_eph_map, Sates, valid_obss);
		if (n_sys>0)
		{
			SPP_SYS(valid_obss, Sates, ionutc, recpos, lsresult, n_sys, solveconfig);
			if (lsresult.converged)
			{
				SPV_LS(valid_obss, Sates, recpos, lsresult, n_sys);
				return true;
			}
			else return false;
		}
		else return false;
	}
	
}







void RTK_FILE_OEM7(GNSSConfig gnssconfig)
{
	std::string base_file = gnssconfig.file_config.base_obs_file;
	std::string rover_file = gnssconfig.file_config.rover_obs_file;
	std::string rtk_file = gnssconfig.file_config.rtk_result_file;
	

	ifstream rfile1(base_file, std::ios::binary);
	if (!rfile1.is_open())
	{
		throw runtime_error("无法打开文件: " + base_file);
	}
	ifstream rfile2(rover_file, std::ios::binary);
	if (!rfile2.is_open())
	{
		throw runtime_error("无法打开文件: " + rover_file);
	}


	// ===================== 声明两套存储变量 =====================
	// 基准站的星历数据，map的key为卫星号，value为对应的星历数据
	std::map<std::string, std::vector<BDSEPHEMERIS>> bds_eph_map1;
	std::map<std::string, std::vector<BDSEPHEMERIS>> bds_eph_map2;
	std::map<std::string, std::vector<GPSEPHEM>> gps_eph_map1;
	std::map<std::string, std::vector<GPSEPHEM>> gps_eph_map2;
	// 基准站的位置
	Point base_pos;
	vector<OBSS> base_obss_vec;
	vector<SPPResult> base_lsresult_vec;
	//赋值近似数据
	base_pos.x = -2267700.0; base_pos.y = 5009400.0; base_pos.z = 3221000.0;
	int message_count1 = 0;
	int valid_message_count1 = 0;

	//流动站也要解算位置
	Point rover_pos;
	vector<OBSS> rover_obss_vec;
	vector<SPPResult> rover_lsresult_vec;
	//赋值近似数据
	rover_pos.x = -2267700.0; rover_pos.y = 5009400.0; rover_pos.z = 3221000.0;
	int message_count2 = 0;
	int valid_message_count2 = 0;

	IONUTC ionutc1;
	IONUTC ionutc2;

	//获取基准站数据索引
	


	//-------------------------------------首先解算基准站数据--------------------------------------
	//消息包数量和有效消息包数量
	
	while (FindBinSync(rfile1))
	{
		BinMessage message;
		message_count1++;
		// 读取头部
		rfile1.read(reinterpret_cast<char*>(&message.header), sizeof(BinHeader));
		if (rfile1.gcount() != sizeof(BinHeader))
		{
			std::cerr << "警告：读取头部失败，长度错误，跳过此消息" << std::endl;
			continue; // 只使用continue，跳过这个错误
		}

		// 读取消息体
		message.body.resize(message.header.message_length);
		rfile1.read(reinterpret_cast<char*>(message.body.data()), message.header.message_length);
		if (rfile1.gcount() != message.header.message_length)
		{
			std::cerr << "警告：读取消息体失败，长度错误，跳过此消息" << std::endl;
			continue; // 跳过错误
		}

		// 读取CRC
		rfile1.read(reinterpret_cast<char*>(&message.crc), sizeof(uint32_t));
		if (rfile1.gcount() != sizeof(uint32_t))
		{
			std::cerr << "警告：读取CRC失败，长度错误，跳过此消息" << std::endl;
			continue; // 跳过错误
		}

		// 验证CRC
		message.crc_valid = verifyCRC(message);



		//--------------------------------------解码消息---------------------------------


		if (message.crc_valid)
		{

			OBSS obss;
			OBSS valid_obss;
			SPPResult lsresult;
			bool is_eph = false, is_obs = false;
			Decode(message, bds_eph_map1,gps_eph_map1, obss,ionutc1, is_eph, is_obs);

			if (is_obs && Single_POS(obss, bds_eph_map1,gps_eph_map1, valid_obss,ionutc1, base_pos, lsresult, gnssconfig))
			{
				
				base_obss_vec.push_back(valid_obss);
				base_lsresult_vec.push_back(lsresult);
				//输出基准站定位结果
				disp_spp(lsresult);
			}
			valid_message_count1++;

		}



	}
	rfile1.close();
	std::cout << "总消息包数量: " << message_count1 << endl;
	std::cout << "有效消息包数量: " << valid_message_count1 << endl;


	//-------------------------------------解算流动站站数据--------------------------------------
	//消息包数量和有效消息包数量

	while (FindBinSync(rfile2))
	{
		BinMessage message;
		message_count2++;
		// 读取头部
		// 读取头部
		rfile2.read(reinterpret_cast<char*>(&message.header), sizeof(BinHeader));
		if (rfile2.gcount() != sizeof(BinHeader))
		{
			std::cerr << "警告：读取头部失败，长度错误，跳过此消息" << std::endl;
			continue; // 只使用continue，跳过这个错误
		}

		// 读取消息体
		message.body.resize(message.header.message_length);
		rfile2.read(reinterpret_cast<char*>(message.body.data()), message.header.message_length);
		if (rfile2.gcount() != message.header.message_length)
		{
			std::cerr << "警告：读取消息体失败，长度错误，跳过此消息" << std::endl;
			continue; // 跳过错误
		}

		// 读取CRC
		rfile2.read(reinterpret_cast<char*>(&message.crc), sizeof(uint32_t));
		if (rfile2.gcount() != sizeof(uint32_t))
		{
			std::cerr << "警告：读取CRC失败，长度错误，跳过此消息" << std::endl;
			continue; // 跳过错误
		}

		// 验证CRC
		message.crc_valid = verifyCRC(message);



		//--------------------------------------解码消息---------------------------------


		if (message.crc_valid)
		{

			OBSS obss;
			OBSS valid_obss;
			SPPResult lsresult;
			bool is_eph = false, is_obs = false;
			Decode(message, bds_eph_map2,gps_eph_map2, obss,ionutc2, is_eph, is_obs);

			if (is_obs && Single_POS(obss, bds_eph_map2,gps_eph_map2, valid_obss,ionutc2, rover_pos, lsresult, gnssconfig))
			{

				rover_obss_vec.push_back(valid_obss);
				rover_lsresult_vec.push_back(lsresult);
				//输出基准站定位结果
				disp_spp(lsresult);
			}
			valid_message_count2++;

		}

	}
	rfile2.close();
	std::cout << "总消息包数量: " << message_count2 << endl;
	std::cout << "有效消息包数量: " << valid_message_count2 << endl;


	//-------------------------------RTK解算----------------------------------------
	std::ofstream fout(rtk_file);
	if (!fout) throw std::runtime_error(std::string("无法创建输出文件: ") + rtk_file);
	if (!rover_lsresult_vec.empty() && !base_lsresult_vec.empty())//如果有新的流动站定位结果且已有基准站定位结果
	{
		//将流动站数据和基准站数据进行时间匹配
		//匹配成后进行RTK解算
		ProcessRTK(base_obss_vec, rover_obss_vec, base_lsresult_vec, rover_lsresult_vec, fout);
	}
	fout.close();
	return;
}
