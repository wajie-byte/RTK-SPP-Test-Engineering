#include"newSPP.h"
#include"../MATDLL/Mat.h"
#include"../OEMRead/Parse.h"
#include"../SatePos/SatePVT.h"
#include"../SatePos/SatePos.h"
#include<sstream>


bool LS_Line(Mat Recpos,const OBSS& obss,const vector<SATE>& Sates,const IONUTC&ionutc, vector<string> valid_obs_prn, Mat& B, Mat& L, Mat& W,int n_sys)
{
    //存放卫星索引
    vector<int>valid_index;
    if(!GetValidSatIndex(Sates,valid_obs_prn,valid_index))return false;//有效卫星不足4颗，无法进行定位
    int valid_sat = valid_index.size();
    if (valid_sat < 3 + n_sys)return false;
	//重新定义矩阵大小
    B.resize(valid_sat, 3 + n_sys);
	L.resize(valid_sat, 1);
	W.resize(valid_sat, valid_sat);
    

    for (int i = 0; i < valid_sat; i++)
    {
		int sat_index = valid_index[i];
        //由于前面已经经过Match_OBS_AND_EPH,所以卫星和obs数据是完全对应的
        //Sates和obss中obs数量上是相等的，后面通过sat_index索引查找到的数据是对应的



        Point recpos;
        recpos.x = Recpos(0, 0);
        recpos.y = Recpos(1, 0);
        recpos.z = Recpos(2, 0);
        double dX = Sates[sat_index].satepos.x - recpos.x;
        double dY = Sates[sat_index].satepos.y - recpos.y;
        double dZ = Sates[sat_index].satepos.z - recpos.z;
        double range = std::sqrt(dX * dX + dY * dY + dZ * dZ);
        double elev = ComputeElevation(Sates[sat_index].satepos, recpos) * PI / 180.0;
		double iono_delay_gps_m = 0.0;
		double topcon_delay_m = 0.0;
        if (ionutc.a0 != 0 || ionutc.a1 != 0 || ionutc.a2 != 0 || ionutc.a3 != 0 || ionutc.b0 != 0 || ionutc.b1 != 0 || ionutc.b2 != 0 || ionutc.b3 != 0)
        {
            double az = ComputeAzimuth(Sates[sat_index].satepos, recpos) * PI / 180.0;
            double alpha[4] = { ionutc.a0,ionutc.a1,ionutc.a2,ionutc.a3 };
            double beta[4] = { ionutc.b0,ionutc.b1,ionutc.b2,ionutc.b3 };
            Geodetic rec_geo = XYZ2BLH(recpos.x, recpos.y, recpos.z, WGS84);
            iono_delay_gps_m = KlobucharModel_GPS(rec_geo.lat * PI / 180.0, rec_geo.lon * PI / 180.0, elev, az, alpha, beta, Sates[sat_index].GPStime.sow);
			topcon_delay_m = HopfieldTroposphere(rec_geo.lat * PI / 180.0, rec_geo.h, elev);
        }
		
        if (n_sys == 1)
        {
            // 构建设计矩阵
            B(i, 0) = -dX / range;  // 对X的偏导
            B(i, 1) = -dY / range;  // 对Y的偏导
            B(i, 2) = -dZ / range;  // 对Z的偏导
            B(i, 3) = 1.0;
            if (obss.obs[sat_index].system == BDS_SYS)
            {
                if (obss.obs[sat_index].signal_type == BDS_B1I_D1)
                {
                    L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight + Sates[sat_index].BDSEPH.tgd1 * clight;
                }
                else
                {
                    L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight;
                }
            }
            else if (obss.obs[sat_index].system == GPS_SYS)
            {
                if (obss.obs[sat_index].signal_type == GPS_L1C)
                {
                    L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight + Sates[sat_index].GPSEPH.tgd * clight
						- iono_delay_gps_m - topcon_delay_m;
                }
                else
                {
					L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight - iono_delay_gps_m - topcon_delay_m;
                }
            }
        }

        else if(n_sys==2)
        {
            // 构建设计矩阵
            B(i, 0) = -dX / range;  // 对X的偏导
            B(i, 1) = -dY / range;  // 对Y的偏导
            B(i, 2) = -dZ / range;  // 对Z的偏导
            if (obss.obs[sat_index].system == BDS_SYS)
            {
                B(i, 3) = 1.0;
                if (obss.obs[sat_index].signal_type == BDS_B1I_D1)
                {
                    L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight + Sates[sat_index].BDSEPH.tgd1 * clight;
                    
                }
                else
                {
                    L(i, 0) = obss.obs[sat_index].psr - range - Recpos(3, 0) + Sates[sat_index].Clock_bias * clight;
                    
                }
            }
            else if (obss.obs[sat_index].system == GPS_SYS)
            {
                B(i, 4) = 1.0;
                if (obss.obs[sat_index].signal_type == GPS_L1C)
                {
                    L(i, 0) = obss.obs[sat_index].psr - range - Recpos(4, 0) + Sates[sat_index].Clock_bias * clight + Sates[sat_index].GPSEPH.tgd * clight
						- iono_delay_gps_m - topcon_delay_m;
                }
                else
                {
					L(i, 0) = obss.obs[sat_index].psr - range - Recpos(4, 0) + Sates[sat_index].Clock_bias * clight - iono_delay_gps_m - topcon_delay_m;
                }
            }
        }
        
        
        // 单频处理
        
        

        
        

       
        //以高度角定权
        double a = 1.0; double b = 0.5;
        double var = a * a + b * b / (sin(elev) * sin(elev));
        W(i, i) = 1 / var;
       
        
        
    }
    
    
	return true;

}


bool GAB_LS_Line(Mat Recpos, const OBSS& obss, const vector<SATE>& Sates, vector<string> valid_obs_prn, Mat& B, Mat& L, Mat& W)
{
    return true;
}

bool BDS_LS_Line_IF(Mat Recpos, const OBSS& obss, const vector<SATE>& Sates, vector<string> dual_freq_vec, Mat& B, Mat& L, Mat& W)
{
	map<int, DualFreqData> prn_observations = GetPRN_OBS_WithSigma(obss);//获取双频观测值和对应的观测误差
    double a = 0.3, b = 0.5;
    //存放有效卫星索引
    vector<int>valid_index;
    if (!GetValidSatIndex(Sates,dual_freq_vec,valid_index))
    {
        return false;//有效卫星不足4颗，无法进行定位
    }
    int valid_sat = valid_index.size();
	//重新定义矩阵大小
	B.resize(valid_sat, 4);
	L.resize(valid_sat, 1);
	W.resize(valid_sat, valid_sat);

    for (int i = 0; i < valid_sat; i++)
    {
		int sat_index = valid_index[i];
        //由于前面已经经过Match_OBS_AND_EPH,所以卫星和obs数据是完全对应的
        //Sates和obss中obs数量上是相等的，后面通过sat_index索引查找到的数据是对应的
        Point recpos;
        recpos.x = Recpos(0, 0);
        recpos.y = Recpos(1, 0);
        recpos.z = Recpos(2, 0);
        double dX = Sates[sat_index].satepos.x - recpos.x;
        double dY = Sates[sat_index].satepos.y - recpos.y;
        double dZ = Sates[sat_index].satepos.z - recpos.z;
        double range = std::sqrt(dX * dX + dY * dY + dZ * dZ);
        double elev = ComputeElevation(Sates[sat_index].satepos, recpos) * PI / 180.0;

      
        int current_prn_num = Sates[sat_index].BDSEPH.ID;
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
            double var = a * a + b * b / (sin(elev) * sin(elev));
            // 权矩阵
            W(i, i) = 1.0 / var;
            //W(i, i) = 1.0;
        }
        


    }

    return true;

}



void SPP_BDS_SYS(const OBSS& obss, vector<SATE>& Sates, IONUTC& ionutc, Point& Recpos, SPPResult& lsresult, SolveConfig solve_config)
{
    if (obss.n_obs != Sates.size())
    {
        cout << "OBS与星历数据数量不匹配" << endl;
        return;
    }

    if (solve_config.freq != FreqType::BDS_B1B2)
    {
        cout << "频率类型不匹配" << endl;
        return;
    }

    int mode = -1;
    if (solve_config.iono == IonoHandle::NONE && solve_config.freq == FreqType::BDS_B1B2)
    {
        mode = 0;
    }
    else if (solve_config.iono == IonoHandle::ELIMINATE && solve_config.freq == FreqType::BDS_B1B2)
    {
        mode = 1;
    }


    switch (mode)
    {
    case 0://BDS双频无组合观测
    {

        LeastSquares(obss, Sates, ionutc, Recpos, lsresult, 1, solve_config);
        break;
    case 1://BDS双频消电离层组合
    {

        LeastSquares(obss, Sates, ionutc, Recpos, lsresult, 1, solve_config);
        break;
    }



    default:
        break;
    }

    return;


    }
}

void SPP_GPS_SYS(const OBSS& obss, vector<SATE>& Sates,IONUTC& ionutc, Point& Recpos, SPPResult& lsresult, SolveConfig solve_config)
{
    if (obss.n_obs != Sates.size())
    {
        cout << "OBS与星历数据数量不匹配" << endl;
        return;
    }
	int mode = -1;
    if(solve_config.freq==FreqType::GPS_L1L2&&solve_config.iono==IonoHandle::NONE)
    {
        mode = 2;
    }

    switch (mode)
    {
    
    case 2://GPS双频无组合观测
    {

        LeastSquares(obss, Sates,ionutc, Recpos, lsresult, 1, solve_config);
        break;
    }
    default:
        break;
    }

    return;


}

void SPP_SYS(const OBSS& obss, vector<SATE>& Sates, IONUTC& ionutc, Point& Recpos, SPPResult& lsresult, int n_sys,SolveConfig solve_config)
{
    if (obss.n_obs != Sates.size())
    {
        cout << "OBS与星历数据数量不匹配" << endl;
        return;
    }

    LeastSquares(obss, Sates, ionutc,Recpos, lsresult, n_sys, solve_config);
    
   

    return;
}


void LeastSquares(OBSS obss, vector<SATE>& Sates, IONUTC& ionutc, Point& Recpos, SPPResult& lsresult,int n_sys,SolveConfig solve_config)
{
    Mat L;
    Mat B;
    Mat W;
    Mat RecMat(3 + n_sys, 1);
    Mat dx(3+n_sys, 1);
    Mat Dw(3 + n_sys, 3 + n_sys);
    Point recpos = Recpos;
    //接收机钟差，初值赋值为0
    double estDt = 0.0;

    double estDt1 = 0.0;
    int n_obs = 0;

    vector<string> valid_sat_prns;

    // 迭代计算
    int maxIterations = 20;// 最大迭代次数
    double convergenceThreshold = 1e-6;// 收敛阈值
    bool converged = false;// 是否收敛

    for (int i = 0; i < maxIterations; i++)
    {

        SatPVT(obss, Sates, recpos);
        RecMat(0, 0) = recpos.x;
        RecMat(1, 0) = recpos.y;
        RecMat(2, 0) = recpos.z;
        RecMat(3, 0) = estDt;
        if (n_sys == 2)RecMat(4, 0) = estDt1;

		if (solve_config.iono == IonoHandle::NONE)
        {
            valid_sat_prns = Valid_obs_prn(Sates, recpos);
            n_obs = valid_sat_prns.size();
            if (n_obs < 3 + n_sys)
            {
                cout << "卫星数不足" << endl;
                return;
            }

            if (!LS_Line(RecMat, obss,  Sates, ionutc, valid_sat_prns, B, L, W, n_sys))
            {
                cout << "有效观测值不足" << endl;
                return;
            }
        }
        else if (solve_config.system == SatSys::BDS_ONLY&&solve_config.freq == FreqType::BDS_B1B2&&solve_config.iono==IonoHandle::ELIMINATE)
        {
            set<string> dual_freq_prns = GetDualFreqSatellites(obss, Sates, recpos);
            
            for (const string& prn_str : dual_freq_prns)
            {
                valid_sat_prns.push_back(prn_str);
            }
            n_obs = dual_freq_prns.size();
            if (n_obs < 4)
            {
                cout << "卫星数不足" << endl;
                return;
            }


            if (!BDS_LS_Line_IF(RecMat, obss, Sates, valid_sat_prns, B, L, W))
            {
                cout << "有效观测值不足" << endl;
                return;
            }
        }
        

        dx = (B.transpose() * W * B).inverse() * B.transpose() * W * L;
        double delta = 0.0;
        for (int i = 0; i < 3 + n_sys; i++)
        {
            delta += dx(i, 0) * dx(i, 0);
        }
        delta = sqrt(delta);
        // 检查收敛性
        if (delta < convergenceThreshold)
        {
            converged = true;
            lsresult.numIterations = i + 1;
            break;
        }

        else
        {
            recpos.x += dx(0, 0);
            recpos.y += dx(1, 0);
            recpos.z += dx(2, 0);
            estDt += dx(3, 0);
            if (n_sys == 2)estDt1 += dx(4, 0);
        }
    }
    // 计算残差和方差
    if (converged)
    {
        Recpos = recpos;
		lsresult.status = 0;
        vector<int> valid_index;
        // 计算残差
        GetValidSatIndex(Sates, valid_sat_prns, valid_index);
        Mat B_final(L.row, 3 + n_sys);
        Mat V(L.row, 1);
        
        if (n_sys == 1)
        {
            for (int i = 0; i < L.row; i++)
            {

                int sat_index = valid_index[i];
                double dX = Sates[sat_index].satepos.x - Recpos.x;
                double dY = Sates[sat_index].satepos.y - Recpos.y;
                double dZ = Sates[sat_index].satepos.z - Recpos.z;
                double range = sqrt(dX * dX + dY * dY + dZ * dZ);

                // 重新构建设计矩阵
                B_final(i, 0) = -dX / range;
                B_final(i, 1) = -dY / range;
                B_final(i, 2) = -dZ / range;
                B_final(i, 3) = 1.0;
                V(i, 0) = obss.obs[sat_index].psr - range - estDt + Sates[sat_index].Clock_bias * clight;

            }
        }
        else if (n_sys == 2)
        {
            for (int i = 0; i < L.row; i++)
            {

                int sat_index = valid_index[i];
                double dX = Sates[sat_index].satepos.x - Recpos.x;
                double dY = Sates[sat_index].satepos.y - Recpos.y;
                double dZ = Sates[sat_index].satepos.z - Recpos.z;
                double range = sqrt(dX * dX + dY * dY + dZ * dZ);



                // 重新构建设计矩阵
                B_final(i, 0) = -dX / range;
                B_final(i, 1) = -dY / range;
                B_final(i, 2) = -dZ / range;
                if (Sates[sat_index].sys == BDS_SYS)
                {
                    B_final(i, 3) = 1.0;
                    B_final(i, 4) = 0.0;
                    V(i, 0) = obss.obs[sat_index].psr - range - estDt + Sates[sat_index].Clock_bias * clight;
                }
                else if (Sates[sat_index].sys == GPS_SYS)
                {
                    B_final(i, 3) = 0.0;
                    B_final(i, 4) = 1.0;
                    V(i, 0) = obss.obs[sat_index].psr - range - estDt1 + Sates[sat_index].Clock_bias * clight;
                }

            }
        }
       

       
        
        double sigma2 = (V.transpose() * W * V)(0, 0) / (L.row - (n_sys+3));

        
        Mat N = B_final.transpose() * W * B_final;
        Mat Q = N.inverse();
        Mat Cx = sigma2 * Q;
        double GDOP = 0.0;
        double PDOP = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
        for (int i = 0; i < 3 + n_sys; i++)
        {
            GDOP += Q(i, i);
        }
        GDOP = sqrt(GDOP);

        // 存储结果
        lsresult.GPStime = obss.GPStime;
        lsresult.BDStime = obss.BDStime;
        lsresult.est[0] = Recpos.x;
        lsresult.est[1] = Recpos.y;
        lsresult.est[2] = Recpos.z;
        lsresult.est[3] = estDt;
        if (n_sys == 2)lsresult.est[4] = estDt1;
        for (int i = 0; i < 3 + n_sys; i++)
        {
            lsresult.std[i] = sqrt(Cx(i, i));
        }
        // 存储精度因子到结果
        lsresult.gdop = GDOP;
        lsresult.pdop = PDOP;
        
        lsresult.converged = true;
        lsresult.n_sys = n_sys;

        if (solve_config.iono == IonoHandle::NONE)
        {
            set<string>sat_elev_check = Sat_elev_check(Sates, Recpos);

            for (const string& sat : sat_elev_check)
            {
                GetSPPResultSat(Sates, Recpos, sat, lsresult);
            }
        }
        else if (solve_config.system == SatSys::BDS_ONLY && solve_config.freq == FreqType::BDS_B1B2 && solve_config.iono == IonoHandle::ELIMINATE)
        {
            set<string>dual_freq_sats = GetDualFreqSatellites(obss, Sates, Recpos);

            for (const string& sat : dual_freq_sats)
            {
                GetSPPResultSat(Sates, Recpos, sat, lsresult);
            }
        }

    }
    else
    {
        return;
    }
}


bool SPVLine(double* Recpos, const OBSS& obss, const vector<SATE>& Sates, vector<string> valid_obs_prn, Mat& B, Mat& L, Mat& W,int n_sys)
{
    
    vector<int>valid_index;
    if (!GetValidSatIndex(Sates, valid_obs_prn, valid_index))return false;
    int valid_sat = valid_index.size();
    if(valid_sat<3+n_sys)
    {
        return false;//有效卫星不足4颗，无法进行单点测速
	}
    //重新定义矩阵大小
    B.resize(valid_sat, 3+n_sys);
    L.resize(valid_sat, 1);
    W.resize(valid_sat, valid_sat);

    //线性化，构造矩阵
    for(int i=0;i<valid_sat;i++)
    {
        int sat_index = valid_index[i];
        double x = Recpos[0];
        double y = Recpos[1];
        double z = Recpos[2];
        double dX = Sates[sat_index].satepos.x - x;
        double dY = Sates[sat_index].satepos.y - y;
        double dZ = Sates[sat_index].satepos.z - z;
        double range = std::sqrt(dX * dX + dY * dY + dZ * dZ);
        

        if (n_sys == 1)
        {
            B(i, 0) = -dX / range;  // 对X的偏导
            B(i, 1) = -dY / range;  // 对Y的偏导
            B(i, 2) = -dZ / range;  // 对Z的偏导
            B(i, 3) = 1.0;          // 对接收机钟速的偏差（m）
            double radial_vel = (dX * Sates[sat_index].satepos.vx + dY * Sates[sat_index].satepos.vy + dZ * Sates[sat_index].satepos.vz) / range;
            //残差向量
            if (obss.obs[sat_index].system == BDS_SYS)
            {
                if (obss.obs[sat_index].signal_type == BDS_B1I_D1)
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * BDS_B1_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;

                }
                else
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * BDS_B3_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;
                }
            }
            else if (obss.obs[sat_index].system == GPS_SYS)
            {
                if (obss.obs[sat_index].signal_type == GPS_L1C)
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * GPS_L1_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;

                }
                else
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * GPS_L2_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;
                }
            }

        }
        else if (n_sys == 2)
        {
            B(i, 0) = -dX / range;  // 对X的偏导
            B(i, 1) = -dY / range;  // 对Y的偏导
            B(i, 2) = -dZ / range;  // 对Z的偏导
            double radial_vel = (dX * Sates[sat_index].satepos.vx + dY * Sates[sat_index].satepos.vy + dZ * Sates[sat_index].satepos.vz) / range;
            //残差向量
            if (obss.obs[sat_index].system == BDS_SYS)
            {
                B(i, 3) = 1.0;
                if (obss.obs[sat_index].signal_type == BDS_B1I_D1)
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * BDS_B1_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;

                }
                else
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * BDS_B3_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;
                }
            }
            else if (obss.obs[sat_index].system == GPS_SYS)
            {
                B(i, 4) = 1.0;
                if (obss.obs[sat_index].signal_type == GPS_L1C)
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * GPS_L1_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;

                }
                else
                {
                    L(i, 0) = -obss.obs[sat_index].dopp * GPS_L2_WAVELEN
                        - radial_vel
                        + CLIGHT * Sates[sat_index].Clock_vel;
                }
            }
        }
       
        

        
        //权矩阵设置为单位阵
        W(i, i) = 1.0;
	}


    return true;
}

void SPV_LS(const OBSS& obss, const vector<SATE>& Sates, const Point& Recpos, SPPResult& lsresult,int n_sys)
{
    //矩阵定义
    Mat B;
    Mat L;
    Mat W;
    Mat dx(3 + n_sys, 1);
    Mat Dw(3 + n_sys, 3 + n_sys);

    double recpos[3] = { 0.0,0.0,0.0 };
    for (int i = 0; i < 3; i++)recpos[i] = lsresult.est[i];
    vector<string> valid_sat_prns = Valid_obs_prn(Sates, Recpos);
    if (!SPVLine(recpos, obss, Sates, valid_sat_prns, B, L, W, n_sys))
    {
        return;
    }
    //最小二乘解算
    dx = (B.transpose() * W * B).inverse() * B.transpose() * W * L;
    for (int i = 0; i < 3+n_sys; i++)
    {
        lsresult.est_v[i] = dx(i, 0);
    }
    //精度评价
    Mat V = B * dx - L;
    double sigma2 = (V.transpose() * W * V)(0, 0) / (L.row - (3 + n_sys));
    Mat N = B.transpose() * W * B;
    Mat Q = N.inverse();
    Mat Cx = sigma2 * Q;
    
    for (int i = 0; i < 3+n_sys; i++)
    {
        lsresult.std_v[i] = sqrt(Cx(i, i));
    }
    return;
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
                sat_prn = GetSatPRN(Sates[i].sys, Sates[i].GPSEPH.PRN);
                Sat_elev_check.insert(sat_prn);
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
                system_prefix = "G"; 
                ostringstream ss;
                ss << system_prefix << setw(2) << setfill('0') << Sates[i].GPSEPH.PRN;
                valid_obs_prn.push_back(ss.str());
                break;
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

bool GetSPPResultSat(const vector<SATE>& Sates,const Point& Recpos,string sat,SPPResult& lsresult)
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
            system_prefix = "G";
            ostringstream ss_gps;
            ss_gps << system_prefix << setw(2) << setfill('0') << Sates[j].GPSEPH.PRN;
            sat_prn = ss_gps.str();
            break;
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
        //cerr << "警告: 未找到卫星 '" << sat << "'" << endl;
        return false;
    }
    
}

//将SPP解算结果输出到控制台
void disp_spp(const SPPResult& spp_result)
{
    //输出解算状态
    cout << spp_result.converged << " ";
    //输出时间
    cout << spp_result.GPStime.week << " " << fixed << setprecision(3) << spp_result.GPStime.sow << " ";
    if (spp_result.converged)
    {
        //输出卫星数
        cout << spp_result.sat_prns.size() << " ";
        //输出定位精度因子
        cout << spp_result.gdop << " " << spp_result.pdop << " " << endl;

        if (spp_result.n_sys == 1)
        {
            //输出定位结果
            cout << spp_result.est[0] << " " << spp_result.est[1] << " " << spp_result.est[2] << " " << spp_result.est[3] << " "
                << spp_result.std[0] << " " << spp_result.std[1] << " " << spp_result.std[2] << " " << spp_result.std[3] << endl;
            //输出单点测速结果
            {
                cout << fixed << setprecision(4);
                cout << spp_result.est_v[0] << " " << spp_result.est_v[1] << " " << spp_result.est_v[2] << " " << spp_result.est_v[3] << " "
                    << spp_result.std_v[0] << " " << spp_result.std_v[1] << " " << spp_result.std_v[2] << " " << spp_result.std_v[3] << endl;
            }
        }
        else if (spp_result.n_sys == 2)
        {
            //输出定位结果
            cout << spp_result.est[0] << " " << spp_result.est[1] << " " << spp_result.est[2] << " " << spp_result.est[3] << " " << spp_result.est[4] << " "
                << spp_result.std[0] << " " << spp_result.std[1] << " " << spp_result.std[2] << " " << spp_result.std[3] << " " << spp_result.std[4] << endl;
            //输出单点测速结果
            {
                cout << fixed << setprecision(4);
                cout << spp_result.est_v[0] << " " << spp_result.est_v[1] << " " << spp_result.est_v[2] << " " << spp_result.est_v[3] << " " << spp_result.est_v[4] << " "
                    << spp_result.std_v[0] << " " << spp_result.std_v[1] << " " << spp_result.std_v[2] << " " << spp_result.std_v[3] << " " << spp_result.std_v[4] << endl;
            }
        }
        
        //输出卫星PRN
        for (int i = 0; i < spp_result.sat_prns.size(); i++)
        {
            cout << spp_result.sat_prns[i] << " ";
        }
        cout << endl;
        //输出卫星高度角
        for (int i = 0; i < spp_result.sat_elevs.size(); i++)
        {
            cout << fixed << setprecision(2) << spp_result.sat_elevs[i] << " ";
        }
        cout << endl;
    }
    else
    {
        cout << endl;
    }
    
    return;
   
    
}

bool GetValidSatIndex(const vector<SATE>& Sates,vector<string> valid_obs_prn,vector<int>& valid_index)
{
    int n_obs = valid_obs_prn.size();
    int valid_sat = 0;
    for (int i = 0; i < n_obs; i++)
    {
        string current_prn = valid_obs_prn[i];
        int sat_index = -1;
        for (int j = 0; j < Sates.size(); j++)
        {

            string sat_prn;
            switch (Sates[j].sys)
            {
            case BDS_SYS:
                sat_prn = GetSatPRN(Sates[j].sys, Sates[j].BDSEPH.ID);
                break;
            case GPS_SYS:
                sat_prn = GetSatPRN(Sates[j].sys, Sates[j].GPSEPH.PRN);
                break;
            default:
                break;
            }

            if (sat_prn == current_prn)
            {
                sat_index = j;

                //防止读取到相同的索引
                bool same_index = false;
                for (int k = 0; k < valid_index.size(); k++)
                {
                    if (sat_index == valid_index[k])same_index = true;
                }
                if (same_index)continue;

                if (Sates[sat_index].satepos.x == 0 || Sates[sat_index].satepos.y == 0 || Sates[sat_index].satepos.z == 0)
                {
                    continue;//跳过数据异常的卫星
                }
                else
                {
                    valid_index.push_back(sat_index);
                    valid_sat++;
                }
                break;
            }


        }

        if (sat_index == -1)
        {
            // 可以选择跳过这个观测值或抛出异常
            cerr << "Warning: No matching satellite found for PRN: " << current_prn << endl;
            continue;  // 跳过当前观测值
        }
    }

    return(valid_sat == valid_index.size());
}