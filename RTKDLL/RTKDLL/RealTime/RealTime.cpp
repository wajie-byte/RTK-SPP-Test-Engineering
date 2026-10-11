#include"../OEMRead/BinRead.h"
#include"../OEMRead/Parse.h"
#include"RealTime.h"
#include"sockets.h"
#include"../SPP/SPP.h"
#include"../RTKCAL//newRTK.h"
#include <thread>
#include <atomic>
#include <mutex>        // 如果使用多线程
#include <chrono>       // 如果需要时间处理
#include <cmath>        // 数学函数
#include <cstring>      // memcpy等
#include <stdexcept>    // 异常处理


// 函数实现
bool ProcessReceiverData(unsigned char* decBuff, int& lenD,
    std::map<std::string, std::vector<BDSEPHEMERIS>>& eph_map,
    Point& recpos,OBSS& valid_obss, int& message_count, int& valid_message_count,LSResult& lsresult,int pos_mode)
{
    const int BUFFER_SIZE = 2 * MAXRAWLEN;
    size_t processed = 0;

    while (processed < static_cast<size_t>(lenD))
    {
        size_t remaining = lenD - processed;
        bool found_sync = false;
        size_t sync_pos = 0;

        // 查找同步头
        for (size_t i = processed; i <= lenD - sizeof(SYNC_BYTES); i++)
        {
            if (decBuff[i] == SYNC_BYTES[0] &&
                decBuff[i + 1] == SYNC_BYTES[1] &&
                decBuff[i + 2] == SYNC_BYTES[2])
            {
                sync_pos = i;
                found_sync = true;
                break;
            }
        }

        if (!found_sync)
        {
            if (processed > 0)
            {
                memmove(decBuff, decBuff + processed, lenD - processed);
                lenD -= processed;
            }
            
            return true; // 正常处理
        }

        // 跳过同步头之前的数据
        if (sync_pos > processed)
        {
            processed = sync_pos;
            remaining = lenD - processed;
        }

        // 检查是否有足够数据读取头部
        if (remaining < sizeof(BinHeader))
        {
            break;
        }

        // 读取头部
        BinMessage message;
        memcpy(&message.header, decBuff + processed, sizeof(BinHeader));
        processed += sizeof(BinHeader);
        message_count++;

        // 检查是否有足够数据读取消息体
        if (remaining - sizeof(BinHeader) < message.header.message_length)
        {
            processed -= sizeof(BinHeader); // 回退
            break;
        }

        // 读取消息体
        message.body.resize(message.header.message_length);
        memcpy(message.body.data(), decBuff + processed, message.header.message_length);
        processed += message.header.message_length;

        // 检查是否有足够数据读取CRC
        if (remaining - sizeof(BinHeader) - message.header.message_length < sizeof(uint32_t))
        {
            processed -= (sizeof(BinHeader) + message.header.message_length); // 回退
            break;
        }

        // 读取CRC
        memcpy(&message.crc, decBuff + processed, sizeof(uint32_t));
        processed += sizeof(uint32_t);

        // 验证CRC
        message.crc_valid = verifyCRC(message);

        // 开始单点定位
        if (message.crc_valid)
        {
            valid_message_count++;
            OBSS obss;
            bool is_eph = false; bool is_obs = false;
            Decode(message, eph_map, obss, is_eph, is_obs);
            if (is_obs)
            {
                Single_POS(obss, eph_map, valid_obss, recpos,lsresult, pos_mode);
            }
            if (is_eph)
            {
                continue;
            }

            
        }
    }

    // 更新缓冲区长度
    if (processed > 0 && processed <= static_cast<size_t>(lenD))
    {
        size_t remaining = lenD - processed;
        if (remaining > 0 && remaining <= static_cast<size_t>(BUFFER_SIZE))
        {
            memmove(decBuff, decBuff + processed, remaining);
            lenD = static_cast<int>(remaining);
        }
        else
        {
            lenD = 0;
        }
    }
    else if (processed > static_cast<size_t>(lenD))
    {
        std::cerr << "错误：processed > lenD" << std::endl;
        lenD = 0;
        return false; // 错误情况
    }

    return true;
}

bool ProcessDataAndSPP(unsigned char* decBuff, int& lenD,
    std::map<std::string, std::vector<BDSEPHEMERIS>>& eph_map,
    Point& recpos, vector<OBSS>& Valid_obss, int& message_count, int& valid_message_count, vector<LSResult>& lsresults, int pos_mode)
{
    int i = 0;  // 已处理的位置
    bool got_position = false;

    // 处理所有完整消息
    while (i < lenD)
    {
        // 1. 查找同步头
        int sync_pos = -1;
        for (; i <= lenD - 3; i++) 
        {
            if (decBuff[i] == SYNC_BYTES[0] &&
                decBuff[i + 1] == SYNC_BYTES[1] &&
                decBuff[i + 2] == SYNC_BYTES[2]) 
            {
                sync_pos = i;
                break;
            }
        }

        if (sync_pos == -1) break;  // 没找到同步头

        i = sync_pos;  // 跳到同步头位置

        //检查消息完整性
        if (lenD - i < sizeof(BinHeader)) break;

        BinMessage message;
        memcpy(&message.header, decBuff + i, sizeof(BinHeader));

        //检查消息体是否完整
        int total_len = sizeof(BinHeader) + message.header.message_length + sizeof(uint32_t);
        if (lenD - i < total_len) break;

        //读取完整消息
        message.body.resize(message.header.message_length);
        memcpy(message.body.data(), decBuff + i + sizeof(BinHeader), message.header.message_length);
        memcpy(&message.crc, decBuff + i + sizeof(BinHeader) + message.header.message_length, sizeof(uint32_t));

        //处理消息
        message.crc_valid = verifyCRC(message);
        if (message.crc_valid) 
        {
            valid_message_count++;
            OBSS obss;
			OBSS valid_obss;
			LSResult lsresult;
            bool is_eph = false, is_obs = false;
            Decode(message, eph_map, obss, is_eph, is_obs);

            if (is_obs && Single_POS(obss, eph_map, valid_obss, recpos, lsresult, pos_mode))
            {
                got_position = true;
				Valid_obss.push_back(valid_obss);
				lsresults.push_back(lsresult);
                if (Valid_obss.size() > 5)
                {
					Valid_obss.erase(Valid_obss.begin());
					lsresults.erase(lsresults.begin());
                }
            }
        }

        //更新处理位置
        i += total_len;
        message_count++;
    }

    // 更新缓冲区
    int remaining = lenD - i;
    if (remaining > 0) 
    {
        memmove(decBuff, decBuff + i, remaining);
    }
    lenD = remaining;  // 直接更新缓冲区长度

    return got_position;
}


void RealTime_SPP(const std::string& server_ip, int server_port, int pos_mode)
{
    SOCKET NetGps;
    //存储每一次接收机定位的结果，下一次的初值用上一次的结果
    Point recpos;

    recpos.x = -2267700.0; recpos.y = 5009400.0; recpos.z = 3221000.0;
    // 使用提供的OpenSocket函数
    if (OpenSocket(NetGps, server_ip.c_str(), server_port) == false)
    {
        throw std::runtime_error("无法连接网络: " + server_ip + ":" + std::to_string(server_port));
    }

    
	//将星历数据存储在map中，key为卫星号，value为对应的星历数据
    std::map<std::string, std::vector<BDSEPHEMERIS>> eph_map;
	//存储每一次接收机定位的结果，设定最大存储数量，超出后删除最早的数据
    int max_size = 5;
    

    // 网络数据缓冲区
    unsigned char curbuff[MAXRAWLEN];
    const int BUFFER_SIZE = 2 * MAXRAWLEN;
    unsigned char decBuff[BUFFER_SIZE];
    int lenR = 0;
    int lenD = 0;
    vector<OBSS> obss_vec;
    vector<LSResult> lsresult_vec;
    int message_count = 0;
    int valid_message_count = 0;

    std::cout << "开始接收网络数据..." << std::endl;

    try 
    {
        while (true)
        {
			Sleep(980);//设定循环间隔
            
            

            if ((lenR = recv(NetGps, (char*)curbuff, MAXRAWLEN, 0)) > 0)
            {
                std::cout << "接收到 " << lenR << " 字节数据" << std::endl;

                // 缓存拼接
                if ((lenD + lenR) > 2 * MAXRAWLEN)lenD = 0;
                memcpy(decBuff + lenD, curbuff, lenR);
                lenD += lenR;

                // 调用处理函数
                ProcessDataAndSPP(decBuff, lenD, eph_map, recpos, obss_vec, message_count, valid_message_count, lsresult_vec, pos_mode);
            }
            else if (lenR == 0)
            {
                std::cout << "网络连接关闭" << std::endl;
                break;
            }
            else
            {
                int error = WSAGetLastError();
                if (error != WSAEWOULDBLOCK)
                {
                    std::cerr << "网络接收错误: " << error << std::endl;
                    break;
                }
            }
			
        }
    }
    catch (const std::exception& e)
    {
        std::cerr << "网络读取异常: " << e.what() << std::endl;
    }

    // 清理资源
    CloseSocket(NetGps);

    std::cout << "网络接收结束" << std::endl;
    std::cout << "总消息包数量: " << message_count << std::endl;
    std::cout << "有效消息包数量: " << valid_message_count << std::endl;

}




// 函数实现
bool GetOBS(unsigned char* decBuff, int& lenD,
    vector<OBSS>& Valid_obss, int& message_count, int& valid_message_count)
{
    int i = 0;  // 已处理位置

    while (i < lenD)
    {
        // 1. 查找同步头
        int sync_pos = -1;
        for (; i <= lenD - 3; i++) 
        {
            if (decBuff[i] == SYNC_BYTES[0] &&
                decBuff[i + 1] == SYNC_BYTES[1] &&
                decBuff[i + 2] == SYNC_BYTES[2]) 
            {
                sync_pos = i;
                break;
            }
        }

        if (sync_pos == -1) break;  // 没找到同步头

        i = sync_pos;

        // 2. 检查消息完整性
        if (lenD - i < sizeof(BinHeader)) break;

        BinMessage message;
        memcpy(&message.header, decBuff + i, sizeof(BinHeader));

        // 检查消息体是否完整
        int total_len = sizeof(BinHeader) + message.header.message_length + sizeof(uint32_t);
        if (lenD - i < total_len) break;

        // 3. 读取完整消息
        message.body.resize(message.header.message_length);
        memcpy(message.body.data(), decBuff + i + sizeof(BinHeader), message.header.message_length);
        memcpy(&message.crc, decBuff + i + sizeof(BinHeader) + message.header.message_length, sizeof(uint32_t));

        // 4. 处理消息
        message.crc_valid = verifyCRC(message);
        message_count++;

        if (message.crc_valid)
        {
            valid_message_count++;

            if (message.header.message_id == RANGE_ID)
            {
                bool is_obs = false;
                OBSS obss;
                obss = parseRANGE(message.body, is_obs);

                if (is_obs)
                {
                    obss.GPStime.week = message.header.gps_week;
                    obss.GPStime.sow = message.header.gps_seconds / ms2s;

                    
                }
            }
        }

        i += total_len;  // 处理下一个消息
    }

    // 更新缓冲区（如果没有找到观测数据）
    int remaining = lenD - i;
    if (remaining > 0) {
        memmove(decBuff, decBuff + i, remaining);
    }
    lenD = remaining;

    return false;  // 没有找到观测数据
}


bool GetVecOBS(unsigned char* decBuff, int& lenD,
    vector<OBSS>& Vecobss, int& message_count, int& valid_message_count)
{
    int i = 0;  // 已处理位置
	bool get_obs = false;
    
    // 处理所有完整消息，但有数量限制
    while (i < lenD )
    {
        // 1. 查找同步头
        int sync_pos = -1;
        for (; i <= lenD - 3; i++)
        {
            if (decBuff[i] == SYNC_BYTES[0] &&
                decBuff[i + 1] == SYNC_BYTES[1] &&
                decBuff[i + 2] == SYNC_BYTES[2])
            {
                sync_pos = i;
                break;
            }
        }

        if (sync_pos == -1) break;  // 没找到同步头

        i = sync_pos;  // 跳到同步头位置

        // 2. 检查消息完整性
        if (lenD - i < sizeof(BinHeader)) break;

        BinMessage message;
        memcpy(&message.header, decBuff + i, sizeof(BinHeader));

        // 检查消息体是否完整
        int total_len = sizeof(BinHeader) + message.header.message_length + sizeof(uint32_t);
        if (lenD - i < total_len) break;

        // 3. 读取完整消息
        message.body.resize(message.header.message_length);
        memcpy(message.body.data(), decBuff + i + sizeof(BinHeader), message.header.message_length);
        memcpy(&message.crc, decBuff + i + sizeof(BinHeader) + message.header.message_length, sizeof(uint32_t));

        // 4. 处理消息
        message.crc_valid = verifyCRC(message);
        message_count++;
        

        if (message.crc_valid)
        {
            valid_message_count++;

            if (message.header.message_id == RANGE_ID)
            {
                OBSS obss;
                bool is_obs = false;
                obss = parseRANGE(message.body, is_obs);

                if (is_obs)
                {
                    obss.GPStime.week = message.header.gps_week;
                    obss.GPStime.sow = message.header.gps_seconds / ms2s;
					Vecobss.push_back(obss);
                    get_obs = true;
                    if (Vecobss.size() > 5) 
                    {
                        Vecobss.erase(Vecobss.begin()); // 删除最旧的一个
                    }
                    // 继续处理，不立即返回，以处理更多数据
                }
            }
        }

        //更新处理位置
        i += total_len;
    }

    //更新缓冲区
    int remaining = lenD - i;
    if (remaining > 0) 
    {
        memmove(decBuff, decBuff + i, remaining);
    }
    lenD = remaining;

    return get_obs; 
}




void OEMRead_Universal(const std::string& source, int pos_mode,bool is_network,int port)
{
    if (is_network)
    {
        // 使用新的网络读取函数
        RealTime_SPP(source, port,pos_mode);
    }
    else 
    {
        // 使用你原有的文件读取函数
        OEMRead(source,pos_mode);
    }
}



void RTK_RealTime(const std::string& server_ip1, int server_port1,
	const std::string& server_ip2, int server_port2, int pos_mode, const char* filename,int run_time_seconds)//将结果保存到文件中
{

    // ===================== 添加运行时长控制 =====================
    time_t start_time = time(nullptr);
    bool time_limit_enabled = (run_time_seconds > 0);


    // ===================== 创建两个Socket连接 =====================
    SOCKET NetGps1, NetGps2;
    if (OpenSocket(NetGps1, server_ip1.c_str(), server_port1) == false)
    {
        throw std::runtime_error("无法连接接收机1: " + server_ip1 + ":" + std::to_string(server_port1));
    }
    if (OpenSocket(NetGps2, server_ip2.c_str(), server_port2) == false)
    {
        CloseSocket(NetGps1);
        throw std::runtime_error("无法连接接收机2: " + server_ip2 + ":" + std::to_string(server_port2));
    }

    // ===================== 声明两套存储变量 =====================
    // 基准站的星历数据，map的key为卫星号，value为对应的星历数据
    std::map<std::string, std::vector<BDSEPHEMERIS>> eph_map1;
    std::map<std::string, std::vector<BDSEPHEMERIS>> eph_map2;

    // 基准站的位置
    Point base_pos;
    vector<OBSS> base_obss_vec;
    vector<LSResult> base_lsresult_vec;
    //赋值近似数据
    base_pos.x = -2267700.0; base_pos.y = 5009400.0; base_pos.z = 3221000.0;
    int message_count1 = 0;
    int valid_message_count1 = 0;

    //流动站也要解算位置
    Point rover_pos;
    vector<OBSS> rover_obss_vec;
    vector<LSResult> rover_lsresult_vec;
    //赋值近似数据
    rover_pos.x = -2267700.0; rover_pos.y = 5009400.0; rover_pos.z = 3221000.0;
    int message_count2 = 0;
    int valid_message_count2 = 0;
	

    // =====================  两套独立的缓冲区 =====================
    // 接收机1缓冲区
    unsigned char curbuff1[MAXRAWLEN];
    unsigned char decBuff1[2 * MAXRAWLEN];
    int lenR1 = 0, lenD1 = 0;

    // 接收机2缓冲区
    unsigned char curbuff2[MAXRAWLEN];
    unsigned char decBuff2[2 * MAXRAWLEN];
    int lenR2 = 0, lenD2 = 0;

    // =====================  使用select同时监听两个Socket =====================
    fd_set readfds;
    struct timeval timeout;

	//-------------------- 主循环 --------------------
    //开始将数据输出到文件中
    
    std::ofstream fout(filename);
    if (!fout) throw std::runtime_error(std::string("无法创建输出文件: ") + filename);
    while (true)
    {
        // 检查是否超时
        if (time_limit_enabled) 
        {
            time_t current_time = time(nullptr);
            if (difftime(current_time, start_time) >= run_time_seconds) 
            {
                std::cout << "达到运行时长 " << run_time_seconds << " 秒，程序退出" << std::endl;
                break;
            }
        }
        Sleep(980);  // 控制循环频率
        // 基准站：静态变量保存最新数据
        //设置布尔类型，判断是否有新的基准站定位数据或者流动站RANGE数据
        bool is_base_pos = false;
        bool is_rover_pos = false;

        // 设置select
        FD_ZERO(&readfds);
        FD_SET(NetGps1, &readfds);
        FD_SET(NetGps2, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;  // 100ms超时

        int max_fd = (NetGps1 > NetGps2) ? NetGps1 : NetGps2;
        int activity = select(max_fd + 1, &readfds, NULL, NULL, &timeout);

        if (activity > 0)
        {
            // ===================== 处理接收机1数据 =====================
            if (FD_ISSET(NetGps1, &readfds))
            {
                if ((lenR1 = recv(NetGps1, (char*)curbuff1, MAXRAWLEN, 0)) > 0)
                {
                    std::cout << "接收机1: 接收到 " << lenR1 << " 字节数据" << std::endl;

                    // 缓存拼接
                    if ((lenD1 + lenR1) > 2 * MAXRAWLEN)
                        lenD1 = 0;
                    memcpy(decBuff1 + lenD1, curbuff1, lenR1);
                    lenD1 += lenR1;

                    //一次性处理所有的数据
                    is_base_pos = ProcessDataAndSPP(decBuff1, lenD1, eph_map1, base_pos, base_obss_vec, message_count1, valid_message_count1, base_lsresult_vec, pos_mode);
                    //如果有新的基准站定位结果，与现有的流动站数据进行时间匹配
                    if (is_base_pos && !rover_lsresult_vec.empty())
                    {
                        //将流动站数据和基准站数据进行时间匹配
                        //匹配成后进行RTK解算
                        ProcessRTK(base_obss_vec, rover_obss_vec, base_lsresult_vec, rover_lsresult_vec, fout);
                    }
                }


                else if (lenR1 == 0 || WSAGetLastError() != WSAEWOULDBLOCK)
                {
                    std::cout << "接收机1: 连接关闭或错误" << std::endl;
                    break;
                }
            }



            // ===================== 处理接收机2数据 =====================
            if (FD_ISSET(NetGps2, &readfds))
            {
                if ((lenR2 = recv(NetGps2, (char*)curbuff2, MAXRAWLEN, 0)) > 0)
                {
                    std::cout << "接收机2: 接收到 " << lenR2 << " 字节数据" << std::endl;

                    // 缓存拼接
                    if ((lenD2 + lenR2) > 2 * MAXRAWLEN)
                        lenD2 = 0;
                    memcpy(decBuff2 + lenD2, curbuff2, lenR2);
                    lenD2 += lenR2;

                    is_rover_pos = ProcessDataAndSPP(decBuff2, lenD2, eph_map2, rover_pos, rover_obss_vec, message_count2, valid_message_count2, rover_lsresult_vec, pos_mode);
                    if (is_rover_pos && !base_lsresult_vec.empty())//如果有新的流动站定位结果且已有基准站定位结果
                    {
                        //将流动站数据和基准站数据进行时间匹配
                        //匹配成后进行RTK解算
                        ProcessRTK(base_obss_vec, rover_obss_vec, base_lsresult_vec, rover_lsresult_vec, fout);

                    }
                }
            }
        }
        else if (lenR2 == 0 || WSAGetLastError() != WSAEWOULDBLOCK)
        {
            std::cout << "接收机2: 连接关闭或错误" << std::endl;
            break;
        }

    }
    if (!rover_lsresult_vec.empty() && !base_lsresult_vec.empty())//如果有新的流动站定位结果且已有基准站定位结果
    {
		//将流动站数据和基准站数据进行时间匹配
		//匹配成后进行RTK解算
		ProcessRTK(base_obss_vec, rover_obss_vec, base_lsresult_vec, rover_lsresult_vec, fout);
    }


    // 超时或无活动，继续循环

    // ===================== 5. 清理两个Socket =====================
    CloseSocket(NetGps1);
    CloseSocket(NetGps2);
    
	// 关闭文件
	fout.close();

    std::cout << "RTK接收结束" << std::endl;
    std::cout << "接收机1 - 总消息: " << message_count1
        << ", 有效消息: " << valid_message_count1 << std::endl;
    std::cout << "接收机2 - 总消息: " << message_count2
        << ", 有效消息: " << valid_message_count2 << std::endl;

}



void ProcessRTK(vector<OBSS>& base_obss_vec, vector<OBSS>& rover_obss_vec,
    vector<LSResult>& base_lsresult_vec, vector<LSResult>& rover_lsresult_vec,ofstream& fout)
{
	//在流动站数据中查找与基准站数据时间匹配的观测数据进行RTK解算
	//每次匹配成功后删除该流动站数据，直到所有流动站数据处理完毕，没有被处理的数据留待下一次处理
    int i = 0;
    while (i < rover_lsresult_vec.size())
    {
        bool matched = false;
        for (int j = 0; j < base_lsresult_vec.size(); j++)
        {
            double time_diff = (rover_lsresult_vec[i].GPStime.week - base_lsresult_vec[j].GPStime.week) * SecInWeek +
                (rover_lsresult_vec[i].GPStime.sow - base_lsresult_vec[j].GPStime.sow);
            if (fabs(time_diff) < 1e-3)
            {
                RTKResult rtk_result;//临时存储RTK结果
                Solve_RTK(base_obss_vec[j], rover_obss_vec[i], base_lsresult_vec[j], rover_lsresult_vec[i], rtk_result);
                //每解算一个RTK结果就写入文件
                RTKResultWrite(rtk_result, fout);
                // 删除匹配的流动站数据
                rover_obss_vec.erase(rover_obss_vec.begin() + i);
                rover_lsresult_vec.erase(rover_lsresult_vec.begin() + i);
                matched = true;
                break;  // 跳出内层循环
            }
        }
        if (!matched)
        {
            i++;  // 没有匹配，检查下一个
        }
        // 如果matched=true，i不增加（因为删除了当前元素，下一个rover的索引由i+1变为i）
    }
}