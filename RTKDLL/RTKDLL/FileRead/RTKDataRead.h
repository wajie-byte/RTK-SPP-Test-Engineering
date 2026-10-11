#ifndef RDR_h
#define RDR_h
#include<iostream>
using namespace std;
#include<fstream>
#include<string>
#include<iomanip>
#include<vector>
#include<sstream>
#include"../RTKData/DefData.h"
#include"../OEMRead/Parse.h"
bool FileHeadSkip(ifstream& file, const string& keyword);//跳过文件头
void GPSEphDataRead(const string& filename, vector<GPSEphData>& EphData);//读取GPS广播星历文件
void GPSObsDataRead(const string& filename, vector<GPSObsData>& ObsData);//读取GPS观测值文件
std::string fixScientificNotation(const std::string& s);// 修正科学计数法字符串（补0 + D→E）
std::string processLine(const std::string& line);// 处理行，修正科学计数法
bool isRinexHeaderLine(const std::string& line);// 判断是否为RINEX头文件行
void processRinexFile(const std::string& input_filename, const std::string& output_filename);// 处理RINEX文件，修正科学计数法并保留头文件
void fix_File(const std::string& input_filename, const std::string& output_filename);

void BDSEphDataRead(const string& filename, vector<BDSEphData>& EphData);//读取BDS广播星历文件
void BDSObsDataRead(const string& filename, vector<BDSObsData>& ObsData);//读取BDS观测值文件
#endif // RDR_h