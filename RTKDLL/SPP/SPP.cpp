#include"SPP.h"

void SatPVTsignal(SatePosPara& para, Point Recpos)
{
    para.Pos.clear();
    para.Clock_bias.clear();
    para.Clock_vel.clear();

    for (int i = 0; i < para.commonPRN.size(); i++)
    {
        GPSTime ttr; // 卫星信号发射时刻
        ttr.week = para.GPStime.week;
        ttr.sow = para.GPStime.sow - para.C1C[i] / clight;

        // 周内秒调整
        if (ttr.sow < 0)
        {
            ttr.week--;
            ttr.sow += 604800;
        }

        double Clock_bias = 0.0;
        double Clock_vel = 0.0;
        Point Satpos;

        // 迭代计算卫星位置和钟差
        for (int j = 0; j < 2; j++)  // 通常2次迭代足够
        {
            // 使用当前的发射时间计算钟差和位置
            Clock_Cal(para.year, para.month, para.day, para.hour, para.min, para.sec,
                para.ephpara[i], ttr, Clock_bias, Clock_vel);
            GPSSatPVT(para.ephpara[i], ttr, Satpos, Clock_bias, Clock_vel);

            // 修正发射时间：考虑卫星钟差的影响
            ttr.sow = para.GPStime.sow - para.C1C[i] / clight - Clock_bias;

            // 周内秒调整
            if (ttr.sow < 0)
            {
                ttr.week--;
                ttr.sow += 604800;
            }
        }

        // 最终计算
        Clock_Cal(para.year, para.month, para.day, para.hour, para.min, para.sec,
            para.ephpara[i], ttr, Clock_bias, Clock_vel);
        GPSSatPVT(para.ephpara[i], ttr, Satpos, Clock_bias, Clock_vel);
        EarRotCor(Satpos, Recpos);
        Clock_bias -= para.ephpara[i].TGD;
        para.Pos.push_back(Satpos);
        para.Clock_bias.push_back(Clock_bias);
        para.Clock_vel.push_back(Clock_vel);
    }
}

void SPP(SatePosPara& para, SPPResult& lsresult,Point& Recpos)
{
    SatePosPara valid;
    GetValidSat(para, valid);
    SatPVTsignal(valid, Recpos);
    LeastSquares(valid, lsresult, Recpos);
    
}

void SPPLine(Mat Recpos,double num, SatePosPara& valid, Mat& B, Mat& L, Mat& W)
{
    for (int i = 0; i < num; i++)
    {
        double dX = valid.Pos[i].x - Recpos(0, 0);
        double dY = valid.Pos[i].y - Recpos(1, 0);
        double dZ = valid.Pos[i].z - Recpos(2, 0);
        double range = std::sqrt(dX * dX + dY * dY + dZ * dZ);
        // 构建观测向量：修正伪距 - 几何距离
        L(i, 0) = valid.C1C[i] - range - Recpos(3, 0) + valid.Clock_bias[i] * clight;

        // 构建设计矩阵
        B(i, 0) = -dX / range;  // 对X的偏导
        B(i, 1) = -dY / range;  // 对Y的偏导  
        B(i, 2) = -dZ / range;  // 对Z的偏导
        B(i, 3) = 1.0;          // 对接收机钟差的偏导
    }
    
    return;

}
void GPS_SPP(vector<SatePosPara>& satePospara, vector<SPPResult>& LSresult)
{
    Point Recpos;
    Recpos.x = -2267711.081; Recpos.y = 5009156.939; Recpos.z = 3221939.042; 
    for (int i = 0; i < satePospara.size(); i++)
    {
        SPPResult lsresult;
        SPP(satePospara[i], lsresult, Recpos);
        LSresult.push_back(lsresult);
       
    }
    cout << "单点定位计算完成！" << "共生成单点定位结果：" << LSresult.size() << "条" << endl;
    return;
}

void GetValidSat(SatePosPara para,SatePosPara& valid)
{
    cout << "开始处理历元" << para.year << " " << para.month << " " << para.day << " " << para.hour << " " << para.min << " " << para.sec << " " << para.GPStime.week << " " << para.GPStime.sow << endl;
    if (para.commonPRN.size() < 4)
    {
        cout << "卫星数小于4，无法解算" << endl;
        return;
    }
    valid.year = para.year;
    valid.month = para.month;
    valid.day = para.day;
    valid.hour = para.hour;
    valid.min = para.min;
    valid.sec = para.sec;
    valid.GPStime = para.GPStime;
    for (int i = 0; i < para.commonPRN.size(); i++)
    {
        // 检查伪距是否有效
        if (para.C1C[i] <= 0.0 || para.C1C[i] <= minPseudorange || para.C1C[i] >= maxPseudorange)
        {
            cout << "警告: 忽略无效伪距的卫星 " << para.commonPRN[i] << ", 伪距值: " << para.C1C[i] << std::endl;
            continue;
        }
        // 如果伪距都有效，则添加到有效卫星列表中

        valid.commonPRN.push_back(para.commonPRN[i]);
        valid.C1C.push_back(para.C1C[i]);
        valid.ephpara.push_back(para.ephpara[i]);
    }

    // 跳过卫星数量不足的历元
    if (valid.commonPRN.size() < 4)
    {
        cout << "有效卫星数小于4，无法解算" << endl;
        return;
    }
}

void LeastSquares(SatePosPara& valid, SPPResult& lsresult,Point& Recpos)
{
    int numSatellites = valid.commonPRN.size();
    //估计参数矩阵
    Mat RecMat(4, 1);
    Mat dx(4, 1);
    Mat Dw(4, 4);
    Mat L = Mat(numSatellites, 1);  // 观测向量
    Mat B = Mat(numSatellites, 4);  // 设计矩阵
    Mat W = Mat(numSatellites,numSatellites); // 权阵

    double estDt = 0.0;

    // 迭代计算
    int maxIterations = 20;// 最大迭代次数
    double convergenceThreshold = 1e-6;// 收敛阈值
    bool converged = false;// 是否收敛
    for (int i = 0; i < maxIterations; i++)
    {
        SatPVTsignal(valid, Recpos);
        RecMat(0, 0) = Recpos.x;
        RecMat(1, 0) = Recpos.y;
        RecMat(2, 0) = Recpos.z;
        RecMat(3, 0) = estDt;
        // 初始化矩阵

        SPPLine(RecMat, numSatellites, valid, B, L, W);
        W = computeElevationWeight(valid.Pos, Recpos);
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
        cout << "=== 残差分析 ===" << endl;
        // 计算残差
        Mat B_final(numSatellites, 4);
        Mat V(numSatellites, 1);
        double sum_vv = 0.0;
        for (int i = 0; i < numSatellites; i++)
        {
            double dX = valid.Pos[i].x - Recpos.x;
            double dY = valid.Pos[i].y - Recpos.y;
            double dZ = valid.Pos[i].z - Recpos.z;
            double range = sqrt(dX * dX + dY * dY + dZ * dZ);
            V(i, 0) = valid.C1C[i] - range - estDt + valid.Clock_bias[i] * clight;
            sum_vv += V(i, 0) * V(i, 0);
            // 重新构建设计矩阵
            B_final(i, 0) = -dX / range;
            B_final(i, 1) = -dY / range;
            B_final(i, 2) = -dZ / range;
            B_final(i, 3) = 1.0;
            
            /*
            
            Point satPos = valid.Pos[i];
            cout << "卫星" << valid.commonPRN[i]
                << ": 伪距=" << valid.C1C[i]
                << " 修正伪距=" << valid.C1C[i] + valid.Clock_bias[i] * clight
                << fixed << setprecision(3) << "  卫星位置: X=" << satPos.x << " Y=" << satPos.y << " Z=" << satPos.z << endl
                << "  卫星地心距: " << sqrt(satPos.x * satPos.x + satPos.y * satPos.y + satPos.z * satPos.z) << " 米"
                << " 卫星钟差=" << valid.Clock_bias[i] * clight
                << " 几何距离=" << range
                << " 接收机钟差=" << estDt
                << " 残差=" << V(i, 0) << "米" << endl;
            */

        }

        // 计算后验方差因子
        double sigma2 = sum_vv / (numSatellites - 4);
        cout << "单位权方差因子 = " << sigma2 << endl;
        // 计算参数方差-协方差矩阵
        Mat N = B_final.transpose() * W * B_final;
        Mat invN = N.inverse();
        Mat Cx = sigma2 * invN;

        // 存储结果
        lsresult.GPStime = valid.GPStime;
        
        lsresult.est[0] = Recpos.x;
        lsresult.est[1] = Recpos.y;
        lsresult.est[2] = Recpos.z;
        lsresult.est[3] = estDt;
        for (int i = 0; i < 4; i++)
        {
            lsresult.std[i] = sqrt(Cx(i, i));
        }

        lsresult.converged = true;

        // 计算精度因子
        Mat Q = invN;
        double GDOP = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2) + Q(3, 3));
        double PDOP = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
        // 存储精度因子到结果
        lsresult.gdop = GDOP;
        lsresult.pdop = PDOP;

        cout << " 定位成功, 使用卫星数: " << numSatellites << ", GDOP: " << GDOP << ", PDOP: " << PDOP << std::endl;
        for (int i = 0; i < 4; i++)
        {
            cout << "EST:";
            cout << lsresult.est[i] << " ";
        }
        cout << endl;
        for (int i = 0; i < 4; i++)
        {
            cout << "RMS:";
            cout << sqrt(lsresult.std[i]) << " ";
        }
        cout << endl;
        cout << " 迭代次数: " << lsresult.numIterations << endl;
        cout << " GDOP: " << lsresult.gdop << " " << "PDOP:" << lsresult.pdop << endl;
    }
}

// 基于高度角的权矩阵
Mat computeElevationWeight(const vector<Point>& satPositions, const Point& recPos) 
{
    int n = satPositions.size();
    Mat W = Mat::Identity(n);
    double a = 0.03;
    double b = 0.04;
    for (int i = 0; i < n; i++)
    {
        double elevation = ComputeElevation(satPositions[i], recPos);
        double elevation_rad = elevation * PI / 180.0;

        // 基于高度角的连续权重函数
        // 伪距精度通常与 1/sin^2(elevation) 相关
        double weight = sin(elevation_rad) * sin(elevation_rad);

        
    }
    return W;
}

double computeHopfieldTropoCorrection(double elevation, double latitude, double height,
    double P = 1013.25, double T = 15.0, double RH = 50.0)
{
    const double R = 6371000.0;  // 地球半径（米）

    // 转换为开尔文
    double T_k = T + 273.15;

    // 计算水汽压
    double e = 6.108 * exp((17.15 * T - 4684.0) / (T - 38.45)) * (RH / 100.0);

    // 干分量和湿分量有效高度
    double h_d = 40136.0 + 148.72 * (T_k - 273.16);
    double h_w = 11000.0;

    // 天顶延迟
    double K_d = 155.2e-7 * (P / T_k) * h_d;
    double K_w = 155.2e-7 * (4810.0 / (T_k * T_k)) * e * h_w;

    // 高度角转换为弧度
    double elev_rad = elevation * PI / 180.0;
    double sinE = sin(elev_rad);

    // 映射函数
    double m_d = 1.0 / sqrt(sinE * sinE + 2.0 * h_d / R);
    double m_w = 1.0 / sqrt(sinE * sinE + 2.0 * h_w / R);

    return K_d * m_d + K_w * m_w;
}


bool GetBDSValidSat(BDSSatePosPara para, BDSSatePosPara& valid)
{
    cout << "开始处理历元" << para.BDStime.week << " " << para.BDStime.sow << endl;
    if (para.commonPRN.size() < 4)
    {
        cout << "卫星数小于4，无法解算" << endl;
        return false;
    }
  
    valid.BDStime = para.BDStime;
    for (int i = 0; i < para.commonPRN.size(); i++)
    {
        // 检查伪距是否有效
        if (para.C2I[i] <= 0.0)
        {
            cout << "警告: 忽略无效伪距的卫星 " << para.commonPRN[i] << ", 伪距值: " << para.C2I[i] << std::endl;
            continue;
        }
        // 如果伪距都有效，则添加到有效卫星列表中

        valid.commonPRN.push_back(para.commonPRN[i]);
        valid.C2I.push_back(para.C2I[i]);
        valid.L2I.push_back(para.L2I[i]);
        valid.ephpara.push_back(para.ephpara[i]);
    }

    // 跳过卫星数量不足的历元
    if (valid.commonPRN.size() < 4)
    {
        cout << "有效卫星数小于4，无法解算" << endl;
        return false;
    }
    return true;
}

void BDSSatPVTsignal(BDSSatePosPara& para, Point Recpos)
{
    para.Pos.clear();
    para.Clock_bias.clear();
    para.Clock_vel.clear();

    for (int i = 0; i < para.commonPRN.size(); i++)
    {
        BDSTime ttr; // 卫星信号发射时刻
        ttr.week = para.BDStime.week;
        ttr.sow = para.BDStime.sow - para.C2I[i] / clight;

        // 周内秒调整
        if (ttr.sow < 0)
        {
            ttr.week--;
            ttr.sow += 604800;
        }

        double Clock_bias = 0.0;
        double Clock_vel = 0.0;
        Point Satpos;

        // 迭代计算卫星位置和钟差
        for (int j = 0; j < 2; j++)  // 通常2次迭代足够
        {
            // 使用当前的发射时间计算钟差和位置
            BDSClock_Cal(para.ephpara[i], ttr, Clock_bias, Clock_vel);
            BDSSatPVT(para.commonPRN[i], para.ephpara[i], ttr, Satpos, Clock_bias, Clock_vel);

            // 修正发射时间：考虑卫星钟差的影响
            ttr.sow = para.BDStime.sow - (para.C2I[i] - Clock_bias * clight) / clight;

            // 周内秒调整
            if (ttr.sow < 0)
            {
                ttr.week--;
                ttr.sow += 604800;
            }
        }

        // 最终计算
        BDSClock_Cal(para.ephpara[i], ttr, Clock_bias, Clock_vel);
        BDSSatPVT(para.commonPRN[i], para.ephpara[i], ttr, Satpos, Clock_bias, Clock_vel);
        BDSEarRotCor(Satpos, Recpos);
        Clock_bias -= para.ephpara[i].TGD1;
        para.Pos.push_back(Satpos);
        para.Clock_bias.push_back(Clock_bias);
        para.Clock_vel.push_back(Clock_vel);
    }

   
}


void BDS_SPPLine(Mat Recpos, double num, BDSSatePosPara& valid, Mat& B, Mat& L, Mat& W)
{
    for (int i = 0; i < num; i++)
    {
        double dX = valid.Pos[i].x - Recpos(0, 0);
        double dY = valid.Pos[i].y - Recpos(1, 0);
        double dZ = valid.Pos[i].z - Recpos(2, 0);
        double range = std::sqrt(dX * dX + dY * dY + dZ * dZ);
        // 构建观测向量：修正伪距 - 几何距离
        L(i, 0) = valid.C2I[i] - range - Recpos(3, 0) + valid.Clock_bias[i] * clight;

        // 构建设计矩阵
        B(i, 0) = -dX / range;  // 对X的偏导
        B(i, 1) = -dY / range;  // 对Y的偏导  
        B(i, 2) = -dZ / range;  // 对Z的偏导
        B(i, 3) = 1.0;          // 对接收机钟差的偏导
    }

    return;

}



void BDS_LeastSquares(BDSSatePosPara& valid, SPPResult& lsresult, Point& Recpos)
{
    int numSatellites = valid.commonPRN.size();
    //估计参数矩阵
    Mat RecMat(4, 1);
    Mat dx(4, 1);
    Mat Dw(4, 4);
    Mat L = Mat(numSatellites, 1);  // 观测向量
    Mat B = Mat(numSatellites, 4);  // 设计矩阵
    Mat W = Mat(numSatellites, numSatellites); // 权阵

    double estDt = 0.0;

    // 迭代计算
    int maxIterations = 20;// 最大迭代次数
    double convergenceThreshold = 1e-6;// 收敛阈值
    bool converged = false;// 是否收敛
    for (int i = 0; i < maxIterations; i++)
    {
        BDSSatPVTsignal(valid, Recpos);
        RecMat(0, 0) = Recpos.x;
        RecMat(1, 0) = Recpos.y;
        RecMat(2, 0) = Recpos.z;
        RecMat(3, 0) = estDt;
        // 初始化矩阵

        BDS_SPPLine(RecMat, numSatellites, valid, B, L, W);
		W = computeElevationWeight(valid.Pos, Recpos);
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
        cout << "=== 残差分析 ===" << endl;
        // 计算残差
        Mat B_final(numSatellites, 4);
        Mat V(numSatellites, 1);
        double sum_vv = 0.0;
        for (int i = 0; i < numSatellites; i++)
        {
            double dX = valid.Pos[i].x - Recpos.x;
            double dY = valid.Pos[i].y - Recpos.y;
            double dZ = valid.Pos[i].z - Recpos.z;
            double range = sqrt(dX * dX + dY * dY + dZ * dZ);
            V(i, 0) = valid.C2I[i] - range - estDt + valid.Clock_bias[i] * clight;
            sum_vv += V(i, 0) * V(i, 0);
            // 重新构建设计矩阵
            B_final(i, 0) = -dX / range;
            B_final(i, 1) = -dY / range;
            B_final(i, 2) = -dZ / range;
            B_final(i, 3) = 1.0;

           /*
          
            Point satPos = valid.Pos[i];
            cout << "卫星" << valid.commonPRN[i]
                << ": 伪距=" << valid.C2I[i]
                << " 修正伪距=" << valid.C2I[i] + valid.Clock_bias[i] * clight
                << fixed << setprecision(3) << "  卫星位置: X=" << satPos.x << " Y=" << satPos.y << " Z=" << satPos.z << endl
                << "  卫星地心距: " << sqrt(satPos.x * satPos.x + satPos.y * satPos.y + satPos.z * satPos.z) << " 米"
                << " 卫星钟差=" << valid.Clock_bias[i] * clight
                << " 几何距离=" << range
                << " 接收机钟差=" << estDt
                << " 残差=" << V(i, 0) << "米" << endl;
            
             */
        }

        // 计算后验方差因子
        double sigma2 = sum_vv / (numSatellites - 4);
        cout << "单位权方差因子 = " << sigma2 << endl;
        // 计算参数方差-协方差矩阵
        Mat N = B_final.transpose() * W * B_final;
        Mat invN = N.inverse();
        Mat Cx = sigma2 * invN;
         
        
        // 存储结果
        lsresult.BDStime = valid.BDStime;

        lsresult.est[0] = Recpos.x;
        lsresult.est[1] = Recpos.y;
        lsresult.est[2] = Recpos.z;
        lsresult.est[3] = estDt;
        for (int i = 0; i < 4; i++)
        {
            lsresult.std[i] = sqrt(Cx(i, i));
        }
        lsresult.converged = true;

        // 计算精度因子
        Mat Q = invN;
        double GDOP = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2) + Q(3, 3));
        double PDOP = std::sqrt(Q(0, 0) + Q(1, 1) + Q(2, 2));
        // 存储精度因子到结果
        lsresult.gdop = GDOP;
        lsresult.pdop = PDOP;

        cout << " 定位成功, 使用卫星数: " << numSatellites << ", GDOP: " << GDOP << ", PDOP: " << PDOP << std::endl;
        cout << " 迭代次数: " << lsresult.numIterations << endl;
        for (int i = 0; i < 4; i++)
        {
            cout << "EST:";
            cout << lsresult.est[i] << " ";
        }
        cout << endl;
        for (int i = 0; i < 4; i++)
        {
            cout << "RMS:";
            cout << sqrt(lsresult.std[i]) << " ";
        }
        cout << endl;
        
    }
    else
    {
        cout << "解算未收敛" << endl;
    }
}




void bds_SPP(BDSSatePosPara& para, SPPResult& lsresult, Point& Recpos,BDSSatePosPara& valid)
{
    if (GetBDSValidSat(para, valid))
    {
        BDSSatPVTsignal(valid, Recpos);
        BDS_LeastSquares(valid, lsresult, Recpos);
    }
    else return;
    

}


void BDS_SPP(vector<BDSSatePosPara>& satePospara, vector<SPPResult>& LSresult)
{
    Point Recpos;
    Recpos.x = 3678300; Recpos.y = 3678300; Recpos.z = 3678300;
    for (int i = 0; i < satePospara.size(); i++)
    {
        SPPResult lsresult;
        BDSSatePosPara valid;
        bds_SPP(satePospara[i], lsresult, Recpos, valid);
        LSresult.push_back(lsresult);
    }
    cout << "单点定位计算完成！" << "共生成单点定位结果：" << LSresult.size() << "条" << endl;
    return;
}