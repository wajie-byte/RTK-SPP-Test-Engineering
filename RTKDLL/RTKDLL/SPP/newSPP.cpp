#include"newSPP.h"
#include"../MATDLL/Mat.h"
#include"../OEMRead/Parse.h"
#include"../SatePos/SatePVT.h"
#include"../SatePos/SatePos.h"
#include<sstream>


void BDS_LS_Line(Mat Recpos,OBSS obss,const vector<SATE>& Sates, vector<string> valid_obs_prn, Mat& B, Mat& L, Mat& W)
{
    
    int n_obs = valid_obs_prn.size();
    for (int i = 0; i < n_obs; i++)
    {
        string current_prn = valid_obs_prn[i];
        int sat_index = -1;
        for (int j = 0; j < Sates.size(); j++)
        {
            
            string sat_prn; 
            switch (Sates[i].sys)
            {
            case BDS_SYS:
                sat_prn = GetSatPRN(Sates[j].sys, Sates[j].BDSEPH.ID);
				break;
            default:
                break;
            }
            
            if (sat_prn == current_prn)
            {
                sat_index = j;
                break;
            }


        }

        if (sat_index == -1)
        {
            // 可以选择跳过这个观测值或抛出异常
            cerr << "Warning: No matching satellite found for PRN: " << current_prn << endl;
            continue;  // 跳过当前观测值
        }
        Point recpos;
        recpos.x = Recpos(0, 0);
        recpos.y = Recpos(1, 0);
        recpos.z = Recpos(2, 0);
        double dX = Sates[sat_index].satepos.x - recpos.x;
        double dY = Sates[sat_index].satepos.y - recpos.y;
        double dZ = Sates[sat_index].satepos.z - recpos.z;
        double range = std::sqrt(dX * dX + dY * dY + dZ * dZ);
        double elev = ComputeElevation(Sates[sat_index].satepos, recpos) * PI / 180.0;
        
        if (obss.obs[sat_index].system == BDS_SYS && obss.obs[sat_index].signal_type == BDS_B1I_D1)
        {
            L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight + Sates[sat_index].BDSEPH.tgd1 * clight;
        }
        else
        {
            L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight;
        }
            // 单频处理
        
        

        // 构建设计矩阵
        B(i, 0) = -dX / range;  // 对X的偏导
        B(i, 1) = -dY / range;  // 对Y的偏导
        B(i, 2) = -dZ / range;  // 对Z的偏导
        if (obss.obs[sat_index].system == BDS_SYS)
        {
            B(i, 3) = 1.0;          // 对接收机钟差的偏导
        }
        //以高度角定权
        double a = 1; double b = 3;
        
        double var = 1 / (sin(elev) * sin(elev));
        W(i, i) = 1 / var;
        //设置为单位阵
        //W(i, i) = 1.0;
    }

    return;

}

void BDS_LS_Line_IF(Mat Recpos, OBSS obss, const vector<SATE>& Sates,set<string> dual_freq_sats, Mat& B, Mat& L, Mat& W)
{
    map<int, DualFreqData> prn_observations = GetPRN_OBS_WithSigma(obss);
    vector<string> dual_freq_vec;
    for (const string& prn_str : dual_freq_sats)
    {
        dual_freq_vec.push_back(prn_str);
    }
    double a = 0.3, b = 0.5;
    int n_obs = dual_freq_vec.size();


    for (int i = 0; i < n_obs; i++)
    {
        string current_prn = dual_freq_vec[i];
        int current_prn_num = stoi(dual_freq_vec[i].substr(1));
        int sat_index = -1;
        for (int j = 0; j < Sates.size(); j++)
        {
            string sat_prn;
            switch (Sates[i].sys)
            {
            case BDS_SYS:
                sat_prn = GetSatPRN(Sates[j].sys, Sates[j].BDSEPH.ID);
                break;
            default:
                break;
            }
           
            if (sat_prn == current_prn)
            {
                sat_index = static_cast<int>(j);
                break;  // 跳出for循环
            }
        }
        if (sat_index == -1)
        {
            cerr << "警告: 未找到卫星 '" << current_prn << "'" << endl;
            continue;
        }
        if (prn_observations.find(current_prn_num) == prn_observations.end())
        {
            cerr << "警告: 无观测值 for PRN '" << current_prn << "'" << endl;
            continue;
        }
        Point recpos;
        recpos.x = Recpos(0, 0);
        recpos.y = Recpos(1, 0);
        recpos.z = Recpos(2, 0);
        double dX = Sates[sat_index].satepos.x - recpos.x;
        double dY = Sates[sat_index].satepos.y - recpos.y;
        double dZ = Sates[sat_index].satepos.z - recpos.z;
        double range = std::sqrt(dX * dX + dY * dY + dZ * dZ);
        double elev = ComputeElevation(Sates[sat_index].satepos, recpos) * PI / 180.0;

        

        
        
        // 双频处理
        double b1i_psr = prn_observations[current_prn_num].psr_b1;
        double b3i_psr = prn_observations[current_prn_num].psr_b3;
        double b1i_psr_sigma = prn_observations[current_prn_num].sigma_b1;
        double b3i_psr_sigma = prn_observations[current_prn_num].sigma_b3;
        double sat_clock_corr = Sates[sat_index].Clock_bias * clight;
        double tgd1_corr = Sates[sat_index].BDSEPH.tgd1 * clight;
        
        //b1i经过钟差改正但未经tgd改正
        b1i_psr += sat_clock_corr;

        b3i_psr += sat_clock_corr;

        double PR = CalPR_B1I_B3I(b1i_psr, b3i_psr, Sates[sat_index].BDSEPH.tgd1);
        L(i, 0) = PR - range - Recpos(3, 0);
        
        /*
        
        const double f1 = BDS_B1_FREQ;  // B1频率
        const double f2 = BDS_B3_FREQ;   // B3频率
        double P_IF = (f1 * f1 * b1i_psr - f2 * f2 * b3i_psr) / (f1 * f1 - f2 * f2);
        double IF_tgd_corr = (f1 * f1 * tgd1_corr) / (f2 * f2 - f1 * f1);
        L(i, 0) = P_IF - range - Recpos(3, 0) + sat_clock_corr + IF_tgd_corr;
        */
       
       

        // 构建设计矩阵
        B(i, 0) = -dX / range;  // 对X的偏导
        B(i, 1) = -dY / range;  // 对Y的偏导
        B(i, 2) = -dZ / range;  // 对Z的偏导
        if (obss.obs[sat_index].system == BDS_SYS)
        {
            B(i, 3) = 1.0;          // 对接收机钟差的偏导
            double var = 1 / (sin(elev) * sin(elev));
            if (elev < PI / 10)
            {
                var = 1 / (sin(PI / 10) * sin(PI / 10));
            }
            // 权矩阵
            
            W(i, i) = 1.0 / var;
        }
        


    }

    return;

}

void SPP_BDS_SYS(const OBSS& obss, vector<SATE>& Sates, Point& Recpos, LSResult& lsresult,int mode)
{
    if (obss.n_obs != Sates.size())
    {
        cout << "OBS与星历数据数量不匹配" << endl;
        return;
    }


    
    
    


    switch (mode)
    {
    case 0://BDS双频无组合观测
    {
       
        LeastSquares(obss, Sates, Recpos, lsresult, mode);
        break;
    }
    case 1://GPS双频消电离层组合
    {
        
        LeastSquares(obss, Sates, Recpos, lsresult, mode);
        break;
    }



    default:
        break;
    }
   
    return;
    



   
}



void LeastSquares(OBSS obss, vector<SATE>& Sates, Point& Recpos, LSResult& lsresult,int mode)
{
    Mat L;
    Mat B;
    Mat W;
    Mat RecMat(4, 1);
    Mat dx(4, 1);
    Mat Dw(4, 4);
    //接收机钟差，初值赋值为0
    double estDt = 0.0;
    int n_obs = 0;

    // 迭代计算
    int maxIterations = 20;// 最大迭代次数
    double convergenceThreshold = 1e-6;// 收敛阈值
    bool converged = false;// 是否收敛

    for (int i = 0; i < maxIterations; i++)
    {

        SatPVT(obss, Sates, Recpos);
        RecMat(0, 0) = Recpos.x;
        RecMat(1, 0) = Recpos.y;
        RecMat(2, 0) = Recpos.z;
        RecMat(3, 0) = estDt;
        // 初始化矩阵
        switch (mode)
        {
        case 0:
        {
            vector<string> valid_sat_prns = Valid_obs_prn(Sates, Recpos);
            n_obs = valid_sat_prns.size();
            if (n_obs < 4)
            {
                cout << "卫星数不足" << endl;
                return;
            }
            //估计参数矩阵
            L = Mat(n_obs, 1);  // 观测向量
            B = Mat(n_obs, 4);  // 设计矩阵
            W = Mat(n_obs, n_obs); // 权阵
            BDS_LS_Line(RecMat, obss, Sates, valid_sat_prns, B, L, W);
            break;
        }
        case 1:
        {
            set<string> dual_freq_prns = GetDualFreqSatellites(obss,Sates,Recpos);
            n_obs = dual_freq_prns.size();
            if (n_obs < 4)
            {
                cout << "卫星数不足" << endl;
                return;
            }
            //估计参数矩阵
            L = Mat(n_obs, 1);  // 观测向量
            B = Mat(n_obs, 4);  // 设计矩阵
            W = Mat(n_obs, n_obs); // 权阵
            BDS_LS_Line_IF(RecMat, obss, Sates, dual_freq_prns, B, L, W);
            break;
        }
        default:
            break;
        }
        

        dx = (B.transpose() * W * B).inverse() * B.transpose() * W * L;
        // 检查收敛性
        if (fabs(dx(0, 0)) < convergenceThreshold &&
            fabs(dx(1, 0)) < convergenceThreshold &&
            fabs(dx(2, 0)) < convergenceThreshold &&
            fabs(dx(3, 0)) < convergenceThreshold)
        {
            converged = true;
            lsresult.numIterations = i + 1;
            break;
        }

        else
        {
            Recpos.x += dx(0, 0);
            Recpos.y += dx(1, 0);
            Recpos.z += dx(2, 0);
            estDt += dx(3, 0);
        }
    }
    // 计算残差和方差
    if (converged)
    {
        
        // 计算残差
        Mat B_final(L.row, 4);
        Mat V(L.row, 1);
        
        for (int i = 0; i < L.row; i++)
        {
            double dX = Sates[i].satepos.x - Recpos.x;
            double dY = Sates[i].satepos.y - Recpos.y;
            double dZ = Sates[i].satepos.z - Recpos.z;
            double range = sqrt(dX * dX + dY * dY + dZ * dZ);
            V(i, 0) = obss.obs[i].psr - range - estDt + Sates[i].Clock_bias * clight;
            
            // 重新构建设计矩阵
            B_final(i, 0) = -dX / range;
            B_final(i, 1) = -dY / range;
            B_final(i, 2) = -dZ / range;
            B_final(i, 3) = 1.0;


        }

       
        
        double sigma2 = (V.transpose() * W * V)(0, 0) / (L.row - 4);

        
        Mat N = B_final.transpose() * W * B_final;
        Mat Q = N.inverse();
        Mat Cx = sigma2 * Q;
        
        


        // 存储结果
        lsresult.GPStime = obss.GPStime;
        lsresult.BDStime = obss.BDStime;
        lsresult.est[0] = Recpos.x;
        lsresult.est[1] = Recpos.y;
        lsresult.est[2] = Recpos.z;
        lsresult.est[3] = estDt;
        for (int i = 0; i < 4; i++)
        {
            lsresult.var[i] = Cx(i, i);
        }
        
        lsresult.converged = true;

        // 计算精度因子
        double GDOP = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2) + Q(3, 3));
        double PDOP = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
        // 存储精度因子到结果
        lsresult.gdop = GDOP;
        lsresult.pdop = PDOP;

        
        switch (mode)
        {
        case 0:
        {
            set<string>sat_elev_check = Sat_elev_check(Sates, Recpos);
            
            for (const string& sat : sat_elev_check)
            {
                GetLSResultSat(Sates, Recpos, sat, lsresult);
            }
            break;
            
        }
        case 1:
        {
            set<string>dual_freq_sats = GetDualFreqSatellites(obss,Sates,Recpos);
            
            for (const string& sat : dual_freq_sats)
            {
                GetLSResultSat(Sates, Recpos, sat, lsresult);
            }
            
            break;
        }
        default:break;


        }

        cout << "定位成功, 使用卫星数: " << lsresult.sat_prns.size() << endl;
        for (int i = 0; i < lsresult.sat_prns.size(); i++)
        {
            cout << lsresult.sat_prns[i] << " ";
        }
        cout << endl;
        cout << "验后单位权中误差 = " << sqrt(sigma2) << " m" << endl;
        cout << "GDOP: " << GDOP << ", PDOP: " << PDOP << std::endl;
        cout << "迭代次数: " << lsresult.numIterations << endl;
        cout << "EST:";
        for (int i = 0; i < 4; i++)
        {
            
            cout << fixed << setprecision(3) << lsresult.est[i] << "m   ";
        }
        cout << endl;
        cout << "RMS:";
        for (int i = 0; i < 4; i++)
        {
            
            cout << sqrt(lsresult.var[i]) << "m   ";
        }
        cout << endl;

    }
    else
    {
        cout << "解算未收敛" << endl;
    }
}



void Getvalid(OBSS obss, const vector<SATE>& Sates, OBSS& valid_obss, vector<SATE>& valid_Sates)
{
    if (obss.n_obs == Sates.size())
    {

    }
}



set<string> GetDualFreqSatellites(const OBSS& obss,const vector<SATE>& SATES,Point Recpos)
{
    set<string> dual_freq_sats;

    // 第一次遍历：记录所有有B1I信号的PRN
    set<int> has_b1i;
    for (int i = 0; i < obss.n_obs; i++)
    {
        double elev = ComputeElevation(SATES[i].satepos, Recpos);
        if (elev > 15)
        {
            if (obss.obs[i].system == BDS_SYS && obss.obs[i].signal_type == BDS_B1I_D1)
            {
                has_b1i.insert(obss.obs[i].PRN);
            }
        }
       
    }

    // 第二次遍历：检查有B3I信号的PRN是否也在B1I集合中
    for (int i = 0; i < obss.n_obs; i++)
    {
        if (obss.obs[i].system == BDS_SYS && obss.obs[i].signal_type == BDS_B3I_D1)
        {
            int prn = obss.obs[i].PRN;
            if (has_b1i.count(prn))
            {
                // 转换为字符串格式 "C01", "C02" 等
                ostringstream ss;
                ss << "C" << setw(2) << setfill('0') << prn;
                dual_freq_sats.insert(ss.str());
            }
        }
    }

    return dual_freq_sats;
}

set<string> Sat_elev_check(const vector<SATE>& Sates,Point Recpos)
{
    set<string> Sat_elev_check;
    for (int i = 0; i < Sates.size(); i++)
    {
        double elev = ComputeElevation(Sates[i].satepos, Recpos);
        if (elev > 15)
        {
            string sat_prn; 
            
            switch (Sates[i].sys)
            {
            case GPS_SYS:
            {
                break;

            }
            case BDS_SYS:
            {
                sat_prn = GetSatPRN(Sates[i].sys, Sates[i].BDSEPH.ID);;
                Sat_elev_check.insert(sat_prn);
                break;
            }
            default:  break;// Unknown
            }
        }
    }
    return Sat_elev_check;
}

vector<string> Valid_obs_prn(const vector<SATE>& Sates, Point Recpos)
{
    vector<string> valid_obs_prn;
    for (int i = 0; i < Sates.size(); i++)
    {
        double elev = ComputeElevation(Sates[i].satepos, Recpos);
        if (elev > 15)
        {
            string system_prefix;
            switch (Sates[i].sys)
            {
            case GPS_SYS:
            {
                system_prefix = "G"; break;

            }
            case BDS_SYS:
            {
                system_prefix = "C";
                ostringstream ss;
                ss << system_prefix << setw(2) << setfill('0') << Sates[i].BDSEPH.ID;
                valid_obs_prn.push_back(ss.str());
                break;
            }
            default: system_prefix = "U"; break;  // Unknown
            }

        }
        else continue;
    }
    return valid_obs_prn;
}



map<int, DualFreqData> GetPRN_OBS_WithSigma(const OBSS& obss)
{
    map<int, DualFreqData> prn_data;

    for (int i = 0; i < obss.n_obs; i++)
    {
        int prn = obss.obs[i].PRN;

        if (obss.obs[i].signal_type == BDS_B1I_D1)
        {
            prn_data[prn].psr_b1 = obss.obs[i].psr;
            prn_data[prn].sigma_b1 = obss.obs[i].psr_sigma;  // 添加测距误差
            prn_data[prn].has_b1 = true;
        }
        else if (obss.obs[i].signal_type == BDS_B3I_D1)
        {
            prn_data[prn].psr_b3 = obss.obs[i].psr;
            prn_data[prn].sigma_b3 = obss.obs[i].psr_sigma;  // 添加测距误差
            prn_data[prn].has_b3 = true;
        }
    }

    return prn_data;
}

double CalPR_B1I_B3I(double psr1,double psr3,double tgd1)
{
    
    const double kf_1_3 = BDS_B1_FREQ * BDS_B1_FREQ / BDS_B3_FREQ / BDS_B3_FREQ;
    double PR = (psr3 - kf_1_3 * psr1) / (1 - kf_1_3) + clight * kf_1_3 * tgd1 / (1 - kf_1_3);
    return PR;
}

bool GetLSResultSat(const vector<SATE>& Sates,const Point& Recpos,string sat,LSResult& lsresult)
{
    
    int sat_index = -1;
    for (int j = 0; j < Sates.size(); j++)
    {
        string system_prefix;
        string sat_prn;
        switch (Sates[j].sys)
        {
        case GPS_SYS:
        {
            system_prefix = "G"; break;
        }
        case BDS_SYS:
        {
            system_prefix = "C";
            ostringstream ss_bds;
            ss_bds << system_prefix << setw(2) << setfill('0') << Sates[j].BDSEPH.ID;
            sat_prn = ss_bds.str();
            break;

        }

        default: continue;
        }
        if (sat_prn == sat)
        {
            sat_index = static_cast<int>(j);
            lsresult.sat_prns.push_back(sat);
            double elev = ComputeElevation(Sates[sat_index].satepos, Recpos);
            lsresult.sat_elevs.push_back(elev);
            lsresult.sat_pos.push_back(Sates[sat_index].satepos);
            return true;
             
        }
    }
    if (sat_index == -1)
    {
        cerr << "警告: 未找到卫星 '" << sat << "'" << endl;
        return false;
    }
    
}