#include"DataProcess.h"
#include"../BASEDLL/Base.h"
// 将格里高利历转换为GPS周内秒
double GregorianToGPSSOW(int year, int month, int day, int hour, int min, double sec)
{
    // GPS历元开始时间：1980-01-06 00:00:00
    static const int gps_epoch_year = 1980;
    static const int gps_epoch_month = 1;
    static const int gps_epoch_day = 6;

    // 计算从GPS历元到指定日期的总天数
    auto daysSinceGPSEpoch = [](int year, int month, int day)
        {
            // 月份天数表（非闰年）
            static const int month_days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

            int total_days = 0;

            // 计算年份差（从1980年到year-1年）
            for (int y = gps_epoch_year; y < year; y++)
            {
                total_days += ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) ? 366 : 365;
            }

            // 计算当前年份中之前月份的天数
            for (int m = 1; m < month; m++)
            {
                total_days += month_days[m - 1];
                // 闰年二月调整
                if (m == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
                    total_days += 1;
                }
            }

            // 修正：加上当前月份的天数，减去GPS历元开始前的5天（1月1日到1月5日）
            total_days += day - 1;  // 从1日开始算
            total_days -= 5;        // GPS历元从1月6日开始，所以要减去前5天

            return total_days;
        };

    int days = daysSinceGPSEpoch(year, month, day);
    double seconds = days * SECONDS_PER_DAY + hour * 3600.0 + min * 60.0 + sec;

    return seconds;
}
// 更简洁的版本（直接计算总秒数差）
double CalculateTimeDiffGPSSimple(int year1, int month1, int day1, int hour1, int min1, double sec1,
    int toe_week, double toe) 
{

    // 观测时间的总GPS秒数（从1980-01-06开始）
    double obs_total_seconds = GregorianToGPSSOW(year1, month1, day1, hour1, min1, sec1);

    // 星历时间的总GPS秒数
    double eph_total_seconds = toe_week * SECONDS_PER_WEEK + toe;

    return obs_total_seconds - eph_total_seconds;
}

// 处理时间卷绕的版本（更健壮）
double CalculateTimeDiffGPSRobust(int year1, int month1, int day1, int hour1, int min1, double sec1,
    int toe_week, double toe) 
{

    double time_diff = CalculateTimeDiffGPSSimple(year1, month1, day1, hour1, min1, sec1, toe_week, toe);

    // 处理时间卷绕：如果时间差超过半周，调整到最近的时间
    if (time_diff > SECONDS_PER_WEEK / 2)
    {
        time_diff -= SECONDS_PER_WEEK;
    }
    else if (time_diff < -SECONDS_PER_WEEK / 2)
    {
        time_diff += SECONDS_PER_WEEK;
    }

    return time_diff;
}


// 直接平铺所有星历数据
map<string, vector<GPSEphData>> BuildEphMap(const vector<GPSEphData>& all_ephemeries) 
{
    map<string, vector<GPSEphData>> eph_map;

    for (const auto& eph_data : all_ephemeries) 
    {
        if (!eph_data.PRN.empty()) 
        {
            eph_map[eph_data.PRN].push_back(eph_data);
        }
    }

    cout << "星历映射构建完成，包含 " << eph_map.size() << " 颗卫星的数据" << endl;
    for (const auto& entry : eph_map)
    {
        cout << "  卫星 " << entry.first << ": " << entry.second.size() << " 个星历" << endl;
    }

    return eph_map;
}

void MatchSatePosPara(const GPSObsData obsdata, const map<string,vector<GPSEphData>> eph_map,SatePosPara& satepos)
{
	const double MAX_TIME_DIFF = 2.0 * 3600.0; // 2小时最大时间差
    // 设置输出时间
    satepos.year = obsdata.year;
    satepos.month = obsdata.month;
    satepos.day = obsdata.day;
    satepos.hour = obsdata.hour;
    satepos.min = obsdata.min;
    satepos.sec = obsdata.sec;

    cout << "开始匹配历元: " << obsdata.year << " " << obsdata.month << " " << obsdata.day
        << " " << obsdata.hour << " " << obsdata.min << " " << obsdata.sec << endl;
    cout << "观测卫星数量: " << obsdata.PRN.size() << endl;

    int matched_count = 0;

    // 遍历观测数据中的每颗卫星
    for (size_t i = 0; i < obsdata.PRN.size(); i++)
    {
        const string& prn = obsdata.PRN[i];
        double c1c = obsdata.C1C[i];
        double l1c = obsdata.L1C[i];

        // 1. 按PRN查找对应的星历组
        auto it = eph_map.find(prn);
        if (it == eph_map.end())
        {
            cout << "  卫星 " << prn << ": 在星历映射中未找到" << endl;
            continue;
        }

        const vector<GPSEphData>& eph_list = it->second;
        
        if (eph_list.empty())
        {
            
            continue;
        }

        // 2. 寻找时间差最小且有效的星历
        const GPSEphData* best_eph = nullptr;
        double min_time_diff = 3600.0;

        for (size_t j=0;j<eph_list.size();j++)
        {
            const auto& eph = eph_list[j];
            double time_diff = CalculateTimeDiffGPSRobust (
                obsdata.year, obsdata.month, obsdata.day, obsdata.hour, obsdata.min, obsdata.sec,
                eph.ephpara.Toe_week,eph.ephpara.Toe);

            double abs_time_diff = fabs(time_diff);
      
            // 检查时间有效性并寻找最小时间差
            if (abs_time_diff <= MAX_TIME_DIFF && abs_time_diff < min_time_diff)
            {
                best_eph = &eph;
                min_time_diff = abs_time_diff;
            }
        }

        // 3. 如果找到匹配，添加到结果中
        if (best_eph != nullptr)
        {
            satepos.commonPRN.push_back(prn);
            satepos.ephpara.push_back(best_eph->ephpara);
            satepos.C1C.push_back(c1c);
            satepos.L1C.push_back(l1c);
            matched_count++;

            cout << "  匹配成功: 卫星 " << prn
                << ", 时间差: " << min_time_diff << " 秒" << endl;
        }
        else 
        {
            //cout << "  警告: 卫星 " << prn
                //<< " 未找到有效星历" << endl;
        }
    }

    cout << "匹配完成: " << matched_count << "/" << obsdata.PRN.size() << " 颗卫星匹配成功" << endl;
}

void SateParaMatch(const vector<GPSEphData>& EphData, const vector<GPSObsData>& ObsData, vector<SatePosPara>& Satepospara)
{
    map<string, vector<GPSEphData>>eph_map;
    eph_map = BuildEphMap(EphData);
    for (size_t i = 0; i < ObsData.size();i++)
    {
        SatePosPara satepos;
        MatchSatePosPara(ObsData[i], eph_map, satepos);
        gregorianToGPSTime(satepos.year, satepos.month, satepos.day, satepos.hour, satepos.min, satepos.sec, satepos.GPStime);
        Satepospara.push_back(satepos);
    }
    
}

// 将格里高利历转换为BDS周内秒
double GregorianToBDSOW(int year, int month, int day, int hour, int min, double sec)
{
    // BDS历元开始时间：2006-01-01 00:00:00
    static const int bds_epoch_year = 2006;
    static const int bds_epoch_month = 1;
    static const int bds_epoch_day = 1;

    // 计算从BDS历元到指定日期的总天数
    auto daysSinceBDSEpoch = [](int year, int month, int day)
        {
            // 月份天数表（非闰年）
            static const int month_days[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

            int total_days = 0;

            // 计算年份差（从2006年到year-1年）
            for (int y = bds_epoch_year; y < year; y++)
            {
                total_days += ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) ? 366 : 365;
            }

            // 计算当前年份中之前月份的天数
            for (int m = 1; m < month; m++)
            {
                total_days += month_days[m - 1];
                // 闰年二月调整
                if (m == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
                    total_days += 1;
                }
            }

            // 加上当前月份的天数（BDS历元从1月1日开始，不需要减去额外天数）
            total_days += day - 1;

            return total_days;
        };

    int days = daysSinceBDSEpoch(year, month, day);
    double seconds = days * SECONDS_PER_DAY + hour * 3600.0 + min * 60.0 + sec;

    return seconds;
}

// 计算BDS时间差（简单版本）
double CalculateTimeDiffBDSSimple(int year1, int month1, int day1, int hour1, int min1, double sec1,
    int toe_week, double toe)
{
    // 观测时间的总BDS秒数（从2006-01-01开始）
    double obs_total_seconds = GregorianToBDSOW(year1, month1, day1, hour1, min1, sec1);

    // 星历时间的总BDS秒数
    double eph_total_seconds = toe_week * SECONDS_PER_WEEK + toe;

    return obs_total_seconds - eph_total_seconds;
}

// 处理BDS时间卷绕的版本（更健壮）
double CalculateTimeDiffBDSRobust(int year1, int month1, int day1, int hour1, int min1, double sec1,
    int toe_week, double toe)
{
    double time_diff = CalculateTimeDiffBDSSimple(year1, month1, day1, hour1, min1, sec1, toe_week, toe);

    // 处理时间卷绕：如果时间差超过半周，调整到最近的时间
    if (time_diff > SECONDS_PER_WEEK / 2)
    {
        time_diff -= SECONDS_PER_WEEK;
    }
    else if (time_diff < -SECONDS_PER_WEEK / 2)
    {
        time_diff += SECONDS_PER_WEEK;
    }

    return time_diff;
}

void BuildBDSEphMap(map<string, vector<BDSEphData>>& eph_map, const BDSEphData& eph_data) 
{
    if (!eph_data.PRN.empty()) 
    {
        eph_map[eph_data.PRN].push_back(eph_data);
        cout << "已添加卫星 " << eph_data.PRN << " 的星历数据" << endl;
    }
}

map<string, vector<BDSEphData>> BuildBDSEphMap(const vector<BDSEphData>& all_ephemeries)
{
    map<string, vector<BDSEphData>> eph_map;

    for (const auto& eph_data : all_ephemeries)
    {
        if (!eph_data.PRN.empty())
        {
            eph_map[eph_data.PRN].push_back(eph_data);
        }
    }

    cout << "星历映射构建完成，包含 " << eph_map.size() << " 颗卫星的数据" << endl;
    for (const auto& entry : eph_map)
    {
        cout << "  卫星 " << entry.first << ": " << entry.second.size() << " 个星历" << endl;
    }

    return eph_map;
}

void MatchBDSSatePosPara(const BDSObsData obsdata, const map<string, vector<BDSEphData>> eph_map, BDSSatePosPara& satepos)
{
    const double MAX_TIME_DIFF = 2.0 * 3600.0; // 2小时最大时间差
    // 设置输出时间
    satepos.year = obsdata.year;
    satepos.month = obsdata.month;
    satepos.day = obsdata.day;
    satepos.hour = obsdata.hour;
    satepos.min = obsdata.min;
    satepos.sec = obsdata.sec;

    cout << "开始匹配历元: " << obsdata.year << " " << obsdata.month << " " << obsdata.day
        << " " << obsdata.hour << " " << obsdata.min << " " << obsdata.sec << endl;
    cout << "观测卫星数量: " << obsdata.PRN.size() << endl;

    int matched_count = 0;

    // 遍历观测数据中的每颗卫星
    for (size_t i = 0; i < obsdata.PRN.size(); i++)
    {
        const string& prn = obsdata.PRN[i];
		double c2i = obsdata.C2I[i];
		double l2i = obsdata.L2I[i];
       

        // 1. 按PRN查找对应的星历组
        auto it = eph_map.find(prn);
        if (it == eph_map.end())
        {
            cout << "  卫星 " << prn << ": 在星历映射中未找到" << endl;
            continue;
        }

        const vector<BDSEphData>& eph_list = it->second;

        if (eph_list.empty())
        {

            continue;
        }

        // 2. 寻找时间差最小且有效的星历
        const BDSEphData* best_eph = nullptr;
        double min_time_diff = 3600.0;

        for (size_t j = 0; j < eph_list.size(); j++)
        {
            const auto& eph = eph_list[j];
            double time_diff = CalculateTimeDiffBDSRobust(
                obsdata.year, obsdata.month, obsdata.day, obsdata.hour, obsdata.min, obsdata.sec,
                eph.ephpara.BDT_week, eph.ephpara.Toe);

            double abs_time_diff = fabs(time_diff);

            // 检查时间有效性并寻找最小时间差
            if (abs_time_diff <= MAX_TIME_DIFF && abs_time_diff < min_time_diff)
            {
                best_eph = &eph;
                min_time_diff = abs_time_diff;
            }
        }

        // 3. 如果找到匹配，添加到结果中
        if (best_eph != nullptr)
        {
            satepos.commonPRN.push_back(prn);
            satepos.ephpara.push_back(best_eph->ephpara);
			satepos.C2I.push_back(c2i);
			satepos.L2I.push_back(l2i);
           
            matched_count++;

            cout << "  匹配成功: 卫星 " << prn
                << ", 时间差: " << min_time_diff << " 秒" << endl;
        }
        else
        {
            //cout << "  警告: 卫星 " << prn
                //<< " 未找到有效星历" << endl;
        }
    }

    cout << "匹配完成: " << matched_count << "/" << obsdata.PRN.size() << " 颗卫星匹配成功" << endl;
}

void BDSSateParaMatch(const vector<BDSEphData>& EphData, const vector<BDSObsData>& ObsData, vector<BDSSatePosPara>& Satepospara)
{
    map<string, vector<BDSEphData>>eph_map;
    eph_map = BuildBDSEphMap(EphData);
    for (size_t i = 0; i < ObsData.size(); i++)
    {
        BDSSatePosPara satepos;
        MatchBDSSatePosPara(ObsData[i], eph_map, satepos);
        gregorianToBDSTime(satepos.year, satepos.month, satepos.day, satepos.hour, satepos.min, satepos.sec, satepos.BDStime);
        Satepospara.push_back(satepos);
    }

}

















void TestTimeConversion()
{
    // 测试GPS历元开始时间：1980-01-06 00:00:00
    // 这应该对应 GPS周=0, 周内秒=0
    double epoch_seconds = GregorianToGPSSOW(1980, 1, 6, 0, 0, 0.0);
    int epoch_week = static_cast<int>(epoch_seconds / SECONDS_PER_WEEK);
    double epoch_sow = fmod(epoch_seconds, SECONDS_PER_WEEK);

    cout << "GPS历元时间验证:" << endl;
    cout << "  1980-01-06 00:00:00 -> GPS总秒数: " << epoch_seconds << endl;
    cout << "  对应GPS周: " << epoch_week << " (应该是0)" << endl;
    cout << "  对应周内秒: " << epoch_sow << " (应该是0)" << endl;

    // 测试你的观测时间：2024-03-01 02:53:46
    double obs_seconds = GregorianToGPSSOW(2024, 3, 1, 2, 53, 46.0);
    int obs_week = static_cast<int>(obs_seconds / SECONDS_PER_WEEK);
    double obs_sow = fmod(obs_seconds, SECONDS_PER_WEEK);

    cout << "观测时间验证:" << endl;
    cout << "  2024-03-01 02:53:46 -> GPS总秒数: " << obs_seconds << endl;
    cout << "  对应GPS周: " << obs_week << " (应该在2303左右)" << endl;
    cout << "  对应周内秒: " << obs_sow << endl;

    // 测试星历时间：周=2303, 秒=44640
    double eph_seconds = 2303 * SECONDS_PER_WEEK + 446400;
    cout << "星历时间:" << endl;
    cout << "  周=2303, 秒=44640 -> GPS总秒数: " << eph_seconds << endl;

    // 计算时间差
    double time_diff = obs_seconds - eph_seconds;
    cout << "时间差: " << time_diff << " 秒 (" << time_diff / 3600.0 << " 小时)" << endl;
}