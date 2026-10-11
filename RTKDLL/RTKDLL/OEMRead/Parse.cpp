#include"Parse.h"
#include"BinRead.h"
#include"../BASEDLL/Base.h"
#include<sstream>

BDSEPHEMERIS parseBDSEPHEMERIS(const vector<uint8_t>& body,bool& is_parse)
{
    BDSEPHEMERIS BDSEph;

    if (body.empty())
    {
        std::cerr << "错误: 数据体为空" << std::endl;
        return BDSEph;
    }

    if (body.size() == sizeof(BDSEPHEMERIS))
    {
        memcpy(&BDSEph, body.data(), sizeof(BDSEPHEMERIS));
        is_parse = true;
        return BDSEph;
        
    }
    else
    {
        std::cerr << "错误: 数据长度不匹配，bodysize = " << body.size() << std::endl;
        is_parse = false;
        return BDSEph;
    }
    
    
}

OBSS parseRANGE(const vector<uint8_t>& body, bool& is_parse)
{
    OBSS obss;
    RANGE range;
    
    if (body.empty()) 
    {
        std::cerr << "错误: 数据体为空" << std::endl;
        return obss;
    }

    size_t offset = 0;
    memcpy(&range.n_obs, body.data() + offset, sizeof(range.n_obs));
    offset += sizeof(range.n_obs);
    obss.n_obs = range.n_obs;

    size_t rangesize = sizeof(range.n_obs) + range.n_obs * sizeof(Range);

    if (body.size() == rangesize)
    {
        range.obs.resize(range.n_obs);
        for (int i = 0; i < range.n_obs; i++)
        {
            OBS obs;
            memcpy(&range.obs[i], body.data() + offset, sizeof(Range));
            int sys = -1;
            int signtyp = -1;
            decode_ch_tr_status(range.obs[i].ch_tr_status, sys, signtyp);
            obs = decodeRANGE(range.obs[i], sys, signtyp);
            obss.obs.push_back(obs);

            offset += sizeof(Range);
        }
        is_parse = true;
        return obss;
    }
    else
    {
        std::cerr << "错误: 数据长度不匹配，bodysize = " << body.size() << std::endl;
        is_parse = false;
        return obss;
    }
    
}

bool decode_ch_tr_status(uint32_t ch_tr_status, int& system, int& signal_type)
{
    system = (ch_tr_status >> 16) & 7;
    signal_type = (ch_tr_status >> 21) & 0x1F;
    return true;
}

OBS decodeRANGE(const Range range, int system, int signal_type)
{
    OBS obs;
    obs.system = system;
    obs.signal_type = signal_type;
    obs.PRN = range.PRN_slot;
    
    obs.psr = range.psr;
    obs.psr_sigma = range.psr_sigma;
    obs.adr = -range.adr;
    obs.adr_sigma = range.adr_sigma;
    obs.C_NO = range.C_No;
    return obs;
}






void ProcessBin(const BinMessage message)
{
    
}

BDSEphData Trans_BDSEPHEMERIS(const BDSEPHEMERIS BDSeph)
{
    BDSEphData eph;
    eph.BDStime.sow = BDSeph.toe;
    eph.BDStime.week = BDSeph.Week;
    eph.ephpara.BDT_week = BDSeph.Week;
    eph.ephpara.health = BDSeph.health1;
    eph.ephpara.TGD1 = BDSeph.tgd1;
    eph.ephpara.Toc = BDSeph.toc;
    eph.ephpara.a0 = BDSeph.a0;
    eph.ephpara.a1 = BDSeph.a1;
    eph.ephpara.a2 = BDSeph.a2;
    eph.ephpara.Toe = BDSeph.toe;
    eph.ephpara.sqrtA = BDSeph.RootA;
    eph.ephpara.e = BDSeph.ecc;
    eph.ephpara.omega = BDSeph.omega;
    eph.ephpara.deltan = BDSeph.deltaN;
    eph.ephpara.M0 = BDSeph.M0;
    eph.ephpara.OMEGA0 = BDSeph.Omega0;
    eph.ephpara.OMEGADOT = BDSeph.OmegaDot;
    eph.ephpara.i0 = BDSeph.i0;
    eph.ephpara.IDOT = BDSeph.IDOT;
    eph.ephpara.Cuc = BDSeph.Cuc;
    eph.ephpara.Cus = BDSeph.Cus;
    eph.ephpara.Crc = BDSeph.Crc;
    eph.ephpara.Crs = BDSeph.Crs;
    eph.ephpara.Cic = BDSeph.Cic;
    eph.ephpara.Cis = BDSeph.Cis;
    cout << endl;
    return eph;
}

BDSSatePosPara Match_BDSOBS_AND_BDSEPH(const OBSS obss, const map<string, vector<BDSEphData>> eph_map)
{
    BDSSatePosPara satepos;
    const double MAX_TIME_DIFF =  3600.0; // 2小时最大时间差
    YMDHMS utc = BDT2UTC(obss.BDStime);
    std::cout << "开始处理历元，UTC时间" << utc.year << " " << utc.month << " " << utc.day << " "
        << utc.hour << " " << utc.min << " " << utc.sec << " "
        << "BDT时间: " << obss.BDStime.week << " " << obss.BDStime.sow << endl;
    int matched_count = 0;

    // 遍历观测数据中的每颗卫星
    for (size_t i = 0; i < obss.n_obs; i++)
    {
        if (obss.obs[i].system == 4 && obss.obs[i].signal_type == 0)
        {
            std::string prn;
            if (obss.obs[i].PRN < 10)
            {
                prn = "C0" + std::to_string(obss.obs[i].PRN);
            }
            else
            {
                prn = "C" + std::to_string(obss.obs[i].PRN);
            }

            double psr = obss.obs[i].psr;
            double adr = obss.obs[i].adr;

            // 1. 按PRN查找对应的星历组
            auto it = eph_map.find(prn);
            if (it == eph_map.end())
            {
                std::cout << "  卫星 " << prn << ": 在星历映射中未找到" << endl;
                continue;
            }

            const vector<BDSEphData>& eph_list = it->second;

            if (eph_list.empty())
            {

                continue;
            }

            // 2. 寻找时间差最小且有效的星历
            const BDSEphData* best_eph = nullptr;
            double min_time_diff = 3600;

            for (size_t j = 0; j < eph_list.size(); j++)
            {
                const auto& eph = eph_list[j];
                double time_diff = (obss.BDStime.week - eph.BDStime.week) * SecInWeek + (obss.BDStime.sow - eph.BDStime.sow);

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
                satepos.BDStime = obss.BDStime;
                satepos.commonPRN.push_back(prn);
                satepos.ephpara.push_back(best_eph->ephpara);
                satepos.C2I.push_back(psr);
                satepos.L2I.push_back(adr);
                matched_count++;

                std::cout << "  匹配成功: 卫星 " << prn
                    << ", 时间差: " << min_time_diff << " 秒" << endl;
            }
            else
            {
                //cout << "  警告: 卫星 " << prn
                    //<< " 未找到有效星历" << endl;
            }
        }
        else continue;
        
    }

    return satepos;
}


vector<BDSSatePosPara> BDSMatch(const vector<OBSS> OBSEpoch, const map<string, vector<BDSEphData>> eph_map)
{
    vector<BDSSatePosPara> BDSSatePos;
    for (int i = 0; i < OBSEpoch.size(); i++)
    {
        BDSSatePosPara satepos = Match_BDSOBS_AND_BDSEPH(OBSEpoch[i], eph_map);
        BDSSatePos.push_back(satepos);
    }
    return BDSSatePos;
}


bool Match_OBS_AND_EPH(const OBSS& obss,const map<string, vector<BDSEPHEMERIS>>& eph_map, vector<SATE>& Sates,OBSS& valid_obss)
{
    valid_obss.obs.clear();
    valid_obss.BDStime = obss.BDStime;
    valid_obss.GPStime = obss.GPStime;
    const double BDS_MAX_TIME_DIFF = 3600.0; // 1小时最大时间差
    YMDHMS utc = BDT2UTC(obss.BDStime);
    std::cout << "开始处理历元，UTC时间" << utc.year << " " << utc.month << " " << utc.day << " "
        << utc.hour << " " << utc.min << " " << utc.sec << " "
        << "GPST时间: " << obss.GPStime.week << " " << obss.GPStime.sow << endl;
    int matched_count = 0;

    // 遍历观测数据中的每颗卫星
    for (size_t i = 0; i < obss.n_obs; i++)
    {
        if (obss.obs[i].system == 4 && (obss.obs[i].signal_type == 0 || obss.obs[i].signal_type == 2) && obss.obs[i].C_NO > 35)
        {
            std::string prn;
            if (obss.obs[i].PRN < 10)
            {
                prn = "C0" + std::to_string(obss.obs[i].PRN);
            }
            else
            {
                prn = "C" + std::to_string(obss.obs[i].PRN);
            }

            double psr = obss.obs[i].psr;
            double adr = obss.obs[i].adr;


            // 1. 按PRN查找对应的星历组
            auto it = eph_map.find(prn);
            if (it == eph_map.end())
            {
                //std::cout << "  卫星 " << prn << ": 在星历映射中未找到" << endl;
                continue;
            }

            const vector<BDSEPHEMERIS>& eph_list = it->second;

            if (eph_list.empty())
            {

                continue;
            }

            // 2. 寻找时间差最小且有效的星历
            const BDSEPHEMERIS* best_eph = nullptr;
            double min_time_diff = 3600.0;

            for (size_t j = 0; j < eph_list.size(); j++)
            {
                const auto& eph = eph_list[j];
                double time_diff = (obss.BDStime.week - eph.Week) * SecInWeek + (obss.BDStime.sow - eph.toe);

                double abs_time_diff = fabs(time_diff);

                // 检查时间有效性并寻找最小时间差
                if (abs_time_diff <= BDS_MAX_TIME_DIFF && abs_time_diff < min_time_diff)
                {
                    best_eph = &eph;
                    min_time_diff = abs_time_diff;
                }
            }

            // 3. 如果找到匹配，添加到结果中
            if (best_eph != nullptr)
            {
                SATE sate;
                sate.sys = BDS_SYS;
                sate.BDStime = obss.BDStime;
                sate.BDSEPH = *best_eph;
                sate.GPStime = obss.GPStime;
                Sates.push_back(sate);
                valid_obss.obs.push_back(obss.obs[i]);

                matched_count++;
                /*
                std::cout << "  匹配成功: 卫星 " << prn
                    << " ,频率" << obss.obs[i].signal_type << " , 时间差: " << min_time_diff << " 秒" << endl;
                */

            }
            else
            {
                //cout << "  警告: 卫星 " << prn
                    //<< " 未找到有效星历" << endl;

            }
        }
        else continue;

    }

    valid_obss.n_obs = matched_count;
    return((valid_obss.n_obs == Sates.size())&& valid_obss.n_obs>=4);
    
}


string GetSatPRN(int system,int PRN)
{
    string system_prefix;
    string sat_prn;
    ostringstream ss;
    switch (system)
    {
    case GPS_SYS:
    {
        system_prefix = "G"; break;
        ss << system_prefix << setw(2) << setfill('0') << PRN;
        sat_prn = ss.str();
        break;
    }
    case BDS_SYS:
    {
        system_prefix = "C";
        ss << system_prefix << setw(2) << setfill('0') << PRN;
        sat_prn = ss.str();
        break;
    }
    default:break;

    }
	return sat_prn;
}


void RTKResultWrite(const RTKResult& rtk_result, ofstream& fout)
{
    //输出解算状态
	fout << rtk_result.status << " ";
	//输出GPS时间
	fout << rtk_result.GPStime.week << " " << fixed << setprecision(3) << rtk_result.GPStime.sow << " ";
    
	//如果有固定解则输出固定解，否则输出浮点解
    if (rtk_result.status == 1)
    {
        fout<< fixed << setprecision(4)
			<< rtk_result.ratio << " "
            << rtk_result.baseline_Fixed[0] << " "
            << rtk_result.baseline_Fixed[1] << " "
            << rtk_result.baseline_Fixed[2] << " ";
		//换行输出卫星PRN号
        fout << endl;
        fout << rtk_result.ref_sat << " ";
        for(int i=0;i<rtk_result.other_prns.size();i++)
        {
            fout << rtk_result.other_prns[i] << " ";
		}
		//换行输出卫星高度角
		fout << endl;
        fout << fixed << setprecision(2)
            << rtk_result.ref_elev << " ";
        for (int i = 0; i < rtk_result.other_elevs.size(); i++)
        {
            fout << fixed << setprecision(2)
                << rtk_result.other_elevs[i] << " ";
        }
        
        //换行输出模糊度固定值
        fout << endl;
        for (int i = 0; i < rtk_result.N_Fixed.size(); i++)
        {
            fout << fixed << setprecision(4)
                << rtk_result.N_Fixed[i] << " ";
		}
        //最后换行
		fout << endl;
    }
	else if (rtk_result.status == 0)
    {
        fout << fixed << setprecision(4)
            << rtk_result.ratio << " "
            << rtk_result.baseline_Float[0] << " "
            << rtk_result.baseline_Float[1] << " "
			<< rtk_result.baseline_Float[2] << " ";
        //换行输出卫星PRN号
        fout << endl;
        fout << rtk_result.ref_sat << " ";
        for (int i = 0; i < rtk_result.other_prns.size(); i++)
        {
            fout << rtk_result.other_prns[i] << " ";
        }
        //换行输出卫星高度角
        fout << endl;
        fout << fixed << setprecision(2)
            << rtk_result.ref_elev << " ";
        for (int i = 0; i < rtk_result.other_elevs.size(); i++)
        {
            fout << fixed << setprecision(2)
                << rtk_result.other_elevs[i] << " ";
        }
        //换行输出模糊度浮点值
        fout << endl;
        for (int i = 0; i < rtk_result.N_Float.size(); i++)
        {
            fout << fixed << setprecision(4)
                << rtk_result.N_Float[i] << " ";
        }
		//最后换行
		fout << endl;
    }
    else// 解算失败
    {
        //换行
		fout << endl;
	}
}