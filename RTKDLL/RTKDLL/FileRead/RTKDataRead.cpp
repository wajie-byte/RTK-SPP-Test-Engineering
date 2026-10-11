#include "RTKDataRead.h"

//跳过文件头
bool FileHeadSkip(ifstream& file, const string& keyword)
{
	string line;
	while (getline(file, line)) { // 逐行读取
		if (line.find(keyword) != string::npos)
		{
			// 检查是否包含关键词
			return true; // 找到关键词，返回成功
		}
	}
	return false; // 未找到关键词
}

// 修正科学计数法字符串（补0 + D→E）
std::string fixScientificNotation(const std::string& s)
{
	std::string result = s;

	// 补前导0
	if (!result.empty())
	{
		if (result[0] == '.')
		{
			result = "0" + result;
		}
		else if (result.size() > 1 && result[0] == '-' && result[1] == '.')
		{
			result.replace(0, 1, "-0");
		}
	}

	// 替换D→E（不区分大小写）
	size_t d_pos = result.find('D');
	if (d_pos != std::string::npos)
	{
		result[d_pos] = 'E';
	}

	return result;
}

// 处理单行数据，返回修正后的字符串
std::string processLine(const std::string& line)
{
	std::istringstream iss(line);
	std::ostringstream oss;
	std::string token;
	bool is_first_token = true;

	while (iss >> token)
	{
		if (!is_first_token)
		{
			oss << " "; // 用空格分隔字段
		}
		oss << fixScientificNotation(token);
		is_first_token = false;
	}

	return oss.str();
}

// 判断是否为RINEX文件头行
bool isRinexHeaderLine(const std::string& line)
{
	// 空行
	if (line.empty())
	{
		return true;
	}

	// 检查RINEX特定标记
	const std::vector<std::string> RINEX_MARKERS =
	{
		"RINEX VERSION / TYPE",
		"PGM / RUN BY / DATE",
		"COMMENT",
		"END OF HEADER"
	};

	for (const auto& marker : RINEX_MARKERS)
	{
		if (line.find(marker) != std::string::npos)
		{
			return true;
		}
	}

	// 检查行尾标记（RINEX头行通常右对齐）
	if (line.length() > 60 && line.substr(60).find_first_not_of(' ') == std::string::npos)
	{
		return true;
	}

	return false;
}

// 处理文件并保留文件头
void processRinexFile(const std::string& input_filename, const std::string& output_filename)
{
	std::ifstream input_file(input_filename);
	std::ofstream output_file(output_filename);

	if (!input_file) throw std::runtime_error("无法打开输入文件: " + input_filename);
	if (!output_file) throw std::runtime_error("无法创建输出文件: " + output_filename);

	std::string line;
	bool header_ended = false;

	// 处理文件头和数据行
	while (std::getline(input_file, line))
	{
		if (!header_ended)
		{
			// 写入原始文件头
			output_file << line << std::endl;

			// 检查是否到达文件头结束标记
			if (line.find("END OF HEADER") != std::string::npos)
			{
				header_ended = true;
			}
		}
		else
		{
			// 处理数据行
			if (!line.empty())
			{
				output_file << processLine(line) << std::endl;
			}
			else
			{
				// 保留空行
				output_file << std::endl;
			}
		}
	}
}

// 使用示例
void fix_File(const std::string& input_filename, const std::string& output_filename)
{
	try
	{
		processRinexFile(input_filename, output_filename);
		std::cout << input_filename << "文件处理完成，保存为" << output_filename << std::endl;
	}
	catch (const std::exception& e)
	{
		std::cerr << "处理错误: " << e.what() << std::endl;
	}
}
//读取GPS广播星历文件
void GPSEphDataRead(const string& filename, vector<GPSEphData>& EphData)
{
	ifstream infile;
	infile.open(filename, ios::in);
	if (!infile)
	{
		cerr << "open error" << endl;
		exit(1);
	}
	cout << "开始读取GPS广播星历文件：" << filename << endl;
	//跳过头文件
	string keyword = "END OF HEADER"; // 目标关键词
	if (FileHeadSkip(infile, keyword))
	{
		cout << "找到关键词 \"" << keyword << "\"，从该行开始处理：" << endl;
	}
	else
	{
		cout << "未找到关键词 \"" << keyword << "\"" << endl;
	}
	//储存每行的数据
	string line;
	while (getline(infile, line))
	{
		if (line.find('G') != std::string::npos) //找到包含GPS卫星PRN号的行
		{
			GPSEphData eph;
			istringstream iss(line);
			// 第1行：PRN, year, month, day, hour, min, sec, a0, a1, a2
			iss >> eph.PRN >> eph.year >> eph.month >> eph.day
				>> eph.hour >> eph.min >> eph.sec
				>> eph.ephpara.a0 >> eph.ephpara.a1 >> eph.ephpara.a2;
			// 读取后续7行并填充字段
			for (int i = 0; i < 7; ++i)
			{
				if (!std::getline(infile, line)) break; // 确保文件未结束

				iss.clear();  // 清除错误状态
				iss.str(line); // 更新为当前行

				switch (i) {
				case 0: // 第2行：IODE, Crs, deltan, M0
					iss >> eph.ephpara.IODE >> eph.ephpara.Crs >> eph.ephpara.deltan >> eph.ephpara.M0;
					break;
				case 1: // 第3行：Cuc, e, Cus, sqrtA
					iss >> eph.ephpara.Cuc >> eph.ephpara.e >> eph.ephpara.Cus >> eph.ephpara.sqrtA;
					break;
				case 2: // 第4行：Toe, Cic, OMEGA, Cis
					iss >> eph.ephpara.Toe >> eph.ephpara.Cic >> eph.ephpara.OMEGA >> eph.ephpara.Cis;
					break;
				case 3: // 第5行：i0, Crc, omega, OMEGADOT
					iss >> eph.ephpara.i0 >> eph.ephpara.Crc >> eph.ephpara.omega >> eph.ephpara.OMEGADOT;
					break;
				case 4: // 第6行：IDOT, L2, Toe_week, L2P
					iss >> eph.ephpara.IDOT >> eph.ephpara.L2 >> eph.ephpara.Toe_week >> eph.ephpara.L2P;
					break;
				case 5: // 第7行：precision, health, TGD, IDOC
					iss >> eph.ephpara.precision >> eph.ephpara.health >> eph.ephpara.TGD >> eph.ephpara.IDOC;
					break;
				case 6: // 第8行：Toc
					iss >> eph.ephpara.Toc;
					break;
				}
			}
			// 存储解析后的数据
			EphData.push_back(eph);
		}
	}
	infile.clear();
	infile.close();
	cout << "GPS星历文件读取完成！" << endl;
	cout << filename << "共读取 " << EphData.size() << " 条广播星历数据。" << endl;
}
//读取GPS观测文件
void GPSObsDataRead(const string& filename, vector<GPSObsData>& ObsData)
{
	ifstream infile;
	infile.open(filename, ios::in);
	if (!infile)
	{
		cerr << "open error" << endl;
		exit(1);
	}
	cout << "开始读取GPS观测文件：" << filename << endl;
	//跳过头文件
	string keyword = "END OF HEADER"; // 目标关键词
	if (FileHeadSkip(infile, keyword))
	{
		cout << "找到关键词 \"" << keyword << "\"，从该行开始处理：" << endl;
	}
	else
	{
		cout << "未找到关键词 \"" << keyword << "\"" << endl;
	}
	//储存每行的数据
	string line;
	//储存时间信息
	int year = 0; int month = 0; int day = 0;
	int hour = 0; int min = 0; double sec = 0.0;
	string PRN; double C1C = 0.0; double L1C = 0.0;
	string dummy;
	GPSObsData currentObs; // 当前时间点的观测数据
	while (getline(infile, line))
	{
		if (line[0] == '>') //每次找到包含>号的行，更新一次时间
		{
			// 如果是新的历元开始，先保存上一个历元
			if (!currentObs.PRN.empty())
			{
				ObsData.push_back(currentObs);
				currentObs = GPSObsData();  // 重置为新的空对象
			}
			istringstream iss(line);
			// 解析时间信息
			iss >> dummy >> year >> month >> day >> hour >> min >> sec >> dummy >> dummy;
			// 重置当前观测数据的时间并清空之前的数据
			currentObs.ResetTime(year, month, day, hour, min, sec);
		}
		else if (line[0] == 'G')
		{
			// 解析并添加卫星数据
			istringstream iss(line);
			iss >> PRN >> C1C >> L1C;
			currentObs.AddObsData(PRN, C1C, L1C);
		}
	}
	// 最后一个历元也要返回
	if (!currentObs.PRN.empty())
	{
		ObsData.push_back(currentObs);
	}
	infile.clear();
	infile.close();
	cout << "GPS观测文件读取完成！" << endl;
	cout << filename << "共读取 " << ObsData.size() << " 个历元的观测数据。" << endl;
}

//读取BDS广播星历文件
void BDSEphDataRead(const string& filename, vector<BDSEphData>& EphData)
{
	ifstream infile;
	infile.open(filename, ios::in);
	if (!infile)
	{
		cerr << "open error" << endl;
		exit(1);
	}
	cout << "开始读取BDS广播星历文件：" << filename << endl;
	//跳过头文件
	string keyword = "END OF HEADER"; // 目标关键词
	if (FileHeadSkip(infile, keyword))
	{
		cout << "找到关键词 \"" << keyword << "\"，从该行开始处理：" << endl;
	}
	else
	{
		cout << "未找到关键词 \"" << keyword << "\"" << endl;
	}
	//储存每行的数据
	string line;
	while (getline(infile, line))
	{
		if (line.find('C') != std::string::npos) //找到包含BDS卫星PRN号的行
		{
			BDSEphData eph;
			istringstream iss(line);
			// 第1行：PRN, year, month, day, hour, min, sec, a0, a1, a2
			iss >> eph.PRN >> eph.year >> eph.month >> eph.day
				>> eph.hour >> eph.min >> eph.sec
				>> eph.ephpara.a0 >> eph.ephpara.a1 >> eph.ephpara.a2;
			// 读取后续7行并填充字段
			for (int i = 0; i < 7; ++i)
			{
				if (!std::getline(infile, line)) break; // 确保文件未结束

				iss.clear();  // 清除错误状态
				iss.str(line); // 更新为当前行

				switch (i)
				{
					case 0: // 第2行：AODE, Crs, deltan, M0
						iss >> eph.ephpara.AODE >> eph.ephpara.Crs >> eph.ephpara.deltan >> eph.ephpara.M0;
						break;
					case 1: // 第3行：Cuc, e, Cus, sqrtA
						iss >> eph.ephpara.Cuc >> eph.ephpara.e >> eph.ephpara.Cus >> eph.ephpara.sqrtA;
						break;
					case 2: // 第4行：Toe, Cic, OMEGA0, Cis
						iss >> eph.ephpara.Toe >> eph.ephpara.Cic >> eph.ephpara.OMEGA0 >> eph.ephpara.Cis;
						break;
					case 3: // 第5行：i0, Crc, omega, OMEGADOT
						iss >> eph.ephpara.i0 >> eph.ephpara.Crc >> eph.ephpara.omega >> eph.ephpara.OMEGADOT;
						break;
					case 4: // 第6行：IDOT, spare1, BDT_week, spare2
						iss >> eph.ephpara.IDOT >> eph.ephpara.spare1 >> eph.ephpara.BDT_week >> eph.ephpara.spare2;
						break;
					case 5: // 第7行：accuracy, health, TGD1, TGD2
						iss >> eph.ephpara.accuracy >> eph.ephpara.health >> eph.ephpara.TGD1 >> eph.ephpara.TGD2;
						break;
					case 6: // 第8行：Toc, spare3
						iss >> eph.ephpara.Toc >> eph.ephpara.spare3;
						break;
				}
			}
			// 存储解析后的数据
			EphData.push_back(eph);
		}
	}
	infile.clear();
	infile.close();
	cout << "BDS星历文件读取完成！" << endl;
	cout << filename << "共读取 " << EphData.size() << " 条广播星历数据。" << endl;
}

void BDSObsDataRead(const string& filename, vector<BDSObsData>& ObsData)
{
	ifstream infile;
	infile.open(filename, ios::in);
	if (!infile)
	{
		cerr << "open error" << endl;
		exit(1);
	}
	cout << "开始读取BDS观测文件：" << filename << endl;
	//跳过头文件
	string keyword = "END OF HEADER"; // 目标关键词
	if (FileHeadSkip(infile, keyword))
	{
		cout << "找到关键词 \"" << keyword << "\"，从该行开始处理：" << endl;
	}
	else
	{
		cout << "未找到关键词 \"" << keyword << "\"" << endl;
	}
	//储存每行的数据
	string line;
	//储存时间信息
	int year = 0; int month = 0; int day = 0;
	int hour = 0; int min = 0; double sec = 0.0;
	string PRN; double C2I = 0.0; double L2I = 0.0;
	string dummy;
	BDSObsData currentObs; // 当前时间点的观测数据
	while (getline(infile, line))
	{
		if (line[0] == '>') //每次找到包含>号的行，更新一次时间
		{
			// 如果是新的历元开始，先保存上一个历元
			if (!currentObs.PRN.empty())
			{
				ObsData.push_back(currentObs);
				currentObs = BDSObsData();  // 重置为新的空对象
			}
			istringstream iss(line);
			// 解析时间信息
			iss >> dummy >> year >> month >> day >> hour >> min >> sec >> dummy >> dummy;
			// 重置当前观测数据的时间并清空之前的数据
			currentObs.ResetTime(year, month, day, hour, min, sec);
		}
		else if (line[0] == 'C')
		{
			// 解析并添加卫星数据
			istringstream iss(line);
			iss >> PRN >> C2I >> L2I;
			currentObs.AddObsData(PRN, C2I, L2I);
		}
	}
	// 最后一个历元也要返回
	if (!currentObs.PRN.empty())
	{
		ObsData.push_back(currentObs);
	}
	infile.clear();
	infile.close();
	cout << "BDS观测文件读取完成！" << endl;
	cout << filename << "共读取 " << ObsData.size() << " 个历元的观测数据。" << endl;
}



