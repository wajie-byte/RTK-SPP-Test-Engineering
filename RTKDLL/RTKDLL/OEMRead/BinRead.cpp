#include"BinRead.h"
#include"Parse.h"
#include"../BASEDLL/Base.h"
#include"../SPP/SPP.h"
#include"../SPP/newSPP.h"
// 查找同步字节
bool FindBinSync(std::ifstream& rfile)
{
	uint8_t byte;
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
void OEMRead(const std::string& filename,int pos_mode)
{
	ifstream rfile(filename, std::ios::binary);
	if (!rfile.is_open())
	{
		throw runtime_error("无法打开文件: " + filename);
	}

	//存储解码得到的数据
	vector<BDSEPHEMERIS> BDSEPHS;
	vector<RANGE> RANGES;
	vector<OBSS> OBSEpoch;
	map<string, vector<BDSEPHEMERIS>> eph_map;
	//存储接受机位置，每一次使用上一次的结果作为初值
	Point recpos;
	//-------------------------------------获取消息包--------------------------------------
	//消息包数量和有效消息包数量
	int message_count = 0;
	int valid_message_count = 0;
	while (FindBinSync(rfile))
	{
		BinMessage message;
		message_count++;
		// 读取头部
		rfile.read(reinterpret_cast<char*>(&message.header), sizeof(BinHeader));
		if (rfile.gcount() != sizeof(BinHeader))
		{
			throw runtime_error("读取头部失败，长度错误");
			continue; // 读取失败,继续下一次查找
		}

		// 读取消息体
		message.body.resize(message.header.message_length);
		rfile.read(reinterpret_cast<char*>(message.body.data()), message.header.message_length);
		if (rfile.gcount() != message.header.message_length)
		{
			throw runtime_error("读取消息体失败,长度错误");
			continue; // 读取失败
		}

		// 读取CRC
		rfile.read(reinterpret_cast<char*>(&message.crc), sizeof(uint32_t));
		if (rfile.gcount() != sizeof(uint32_t))
		{
			throw runtime_error("读取CRC失败，长度错误");
			continue; // 读取失败
		}

		// 验证CRC
		message.crc_valid= verifyCRC(message);



		//--------------------------------------解码消息---------------------------------

		
		if (message.crc_valid)
		{
			//解码头部

			
			Decode_And_SPP_test(message, eph_map,recpos,pos_mode);
			valid_message_count++;



		}
		
	}
	rfile.close();
	std::cout << "总消息包数量: " << message_count << endl;
	std::cout << "有效消息包数量: " << valid_message_count << endl;
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
	map<string, vector<BDSEPHEMERIS>>& eph_map)
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
		eph_map[prn_key].push_back(BDSEph);
		std::cout << "已添加卫星 " << prn_key << " 的星历数据" << ", BDT:" << BDSEph.Week << "  Toe:" << BDSEph.toe << endl;
		return true;
	}

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

void Decode(const BinMessage& message, map<string, vector<BDSEPHEMERIS>>& eph_map,OBSS& obss,bool& is_eph,bool& is_obs)
{
	switch (message.header.message_id)
	{
	case BDSEPHEMERIS_ID:
	{
		if (DecodeBDSEphEMERIS(message, eph_map))
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
	default: break;
	}
}


bool Single_POS(const OBSS& obss, map<string, vector<BDSEPHEMERIS>>& eph_map, OBSS& valid_obss, Point& recpos, LSResult& lsresult, int mode)
{
	vector<SATE> Sates;

	if (Match_OBS_AND_EPH(obss, eph_map, Sates, valid_obss))
	{
		
		SPP_BDS_SYS(valid_obss, Sates, recpos, lsresult, mode);
		if (lsresult.converged)
		{
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



void Decode_And_SPP_test(const BinMessage message, map<string, vector<BDSEPHEMERIS>>& eph_map, Point recpos,int mode)
{
	switch (message.header.message_id)
	{
	case BDSEPHEMERIS_ID:
	{
		bool is_parse = false;
		BDSEPHEMERIS BDSEph = parseBDSEPHEMERIS(message.body, is_parse);
		if (is_parse && BDSEph.health1 == 0)
		{

			std::cout << "添加BDS星历，PRN: " << BDSEph.ID << std::endl;
			std::string prn_key;
			if (BDSEph.ID < 10)
			{
				prn_key = "C0" + std::to_string(BDSEph.ID);
			}
			else
			{
				prn_key = "C" + std::to_string(BDSEph.ID);
			}


			eph_map[prn_key].push_back(BDSEph);
			cout << "已添加卫星 " << prn_key << " 的星历数据" << endl;
			cout << "EPHBDSWEEK=" << BDSEph.Week << " " << "EPHBDSTOE=" << BDSEph.toe << endl;
			/*
			std::cout << bdseph.ephpara.a0 << " " << bdseph.ephpara.a1 << " " << bdseph.ephpara.a2 << endl;
			for (int j = 0; j < 7; j++)
			{

				switch (j)
				{
				case 0: // 第2行：AODE, Crs, deltan, M0
					std::cout << bdseph.ephpara.AODE << " " << bdseph.ephpara.Crs << " " << bdseph.ephpara.deltan << " " << bdseph.ephpara.M0 << endl;
					break;
				case 1: // 第3行：Cuc, e, Cus, sqrtA
					std::cout << bdseph.ephpara.Cuc << " " << bdseph.ephpara.e << " " << bdseph.ephpara.Cus << " " << bdseph.ephpara.sqrtA << endl;
					break;
				case 2: // 第4行：Toe, Cic, OMEGA0, Cis
					std::cout << bdseph.ephpara.Toe << " " << bdseph.ephpara.Cic << " " << bdseph.ephpara.OMEGA0 << " " << bdseph.ephpara.Cis << endl;
					break;
				case 3: // 第5行：i0, Crc, omega, OMEGADOT
					std::cout << bdseph.ephpara.i0 << " " << bdseph.ephpara.Crc << " " << bdseph.ephpara.omega << " " << bdseph.ephpara.OMEGADOT << endl;
					break;
				case 4: // 第6行：IDOT, spare1, BDT_week, spare2
					std::cout << bdseph.ephpara.IDOT << " " << bdseph.ephpara.spare1 << " " << bdseph.ephpara.BDT_week << " " << bdseph.ephpara.spare2 << endl;
					break;
				case 5: // 第7行：accuracy, health, TGD1, TGD2
					std::cout << bdseph.ephpara.accuracy << " " << bdseph.ephpara.health << " " << bdseph.ephpara.TGD1 << " " << bdseph.ephpara.TGD2 << endl;
					break;
				case 6: // 第8行：Toc, spare3
					std::cout << bdseph.ephpara.Toc << " " << bdseph.ephpara.spare3 << endl;
					break;
				}
			}


			*/


		}
		break;
	}
	case RANGE_ID:
	{
		bool is_parse = false;
		OBSS obss = parseRANGE(message.body, is_parse);
		if (is_parse)
		{
			std::cout << "添加RANGE数据" << std::endl;
			obss.GPStime.week = message.header.gps_week;
			obss.GPStime.sow = message.header.gps_seconds / ms2s;
			obss.BDStime = gpsToBds(obss.GPStime);


			/*
			cout << obss.GPStime.week << " " << obss.GPStime.sow << " " << obss.n_obs << endl;
			cout << obss.BDStime.week << " " << obss.BDStime.sow << " " << obss.n_obs << endl;
			for (int i = 0; i < obss.n_obs; i++)
			{
				cout << obss.obs[i].system << " " << obss.obs[i].signal_type << " " <<
					obss.obs[i].PRN << " " << obss.obs[i].psr << " " << obss.obs[i].adr << endl;

			}
			*/

			//将OBS数据与星历匹配
			vector<SATE> Sates;
			OBSS valid_obss;
			if (Match_OBS_AND_EPH(obss, eph_map, Sates, valid_obss))
			{
				LSResult lsresult;
				SPP_BDS_SYS(valid_obss, Sates, recpos, lsresult,mode);
			}
			

		}


		break;
	}

	}
}

