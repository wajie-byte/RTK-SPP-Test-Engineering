#include"RTKSoution.h"
#include<algorithm>
#include"../SPP/SPP.h"
#include"../RTKData/DataProcess.h"
#include"../LAMBDA/lambda.h"
#include"../BASEDLL/Base.h"
//双差观测值构建
void GPSBDS_DDObsBuilder(vector<BDSSatePosPara>& Basedata, vector<BDSObsData>& Roverdata, vector<BDS_DDObs>& BDSDD)
{
    

}


// 按时间排序观测数据
void SortObsDataByTime(vector<GPSObsData>& obs_data)
{
    if (obs_data.empty())
    {
		cout << "观测数据为空，无法排序" << endl;
        return;
    }
    
    sort(obs_data.begin(), obs_data.end(),
        [](const GPSObsData& a, const GPSObsData& b)
        {
            // 可以添加时间有效性验证
            if (a.year != b.year) return a.year < b.year;
            if (a.month != b.month) return a.month < b.month;
            if (a.day != b.day) return a.day < b.day;
            if (a.hour != b.hour) return a.hour < b.hour;
            if (a.min != b.min) return a.min < b.min;
            return a.sec < b.sec;
        });

    //验证排序结果
    cout << "排序完成，共处理 " << obs_data.size() << " 个观测历元" << endl;
}

// 排序后使用双指针匹配
void MatchObservations(vector<BDSSatePosPara>& epocheph,const vector<BDSObsData>& obs,vector<BDS_DDObs> GPSDD)
{
	//存储基准站单点定位结果
	Point base_recpos;
    base_recpos.x = base_recpos.y = base_recpos.z = 3678300.0;
    int i = 0, j = 0; // 双指针
    
    cout << "开始时间匹配..." << endl;
    cout << "基准站数据量: " << epocheph.size() << endl;
    cout << "流动站数据量: " << obs.size() << endl;
    while (i < epocheph.size() && j < obs.size())
    {
        if (IsTimeMatched(epocheph[i], obs[j]))
        {
            // 找到匹配，处理双差
            BDS_DDObs ddobs;
            ProcessMatch(epocheph[i], obs[j], base_recpos, ddobs);
			GPSDD.push_back(ddobs);
            RTKSolve(ddobs);
            i++; j++;
        }
        else if (IsBaseEarlier(epocheph[i], obs[j])) 
        {
            i++;  // 基准站时间较早，移动基准站指针
        }
        else
        {
            j++;  // 流动站时间较早，移动流动站指针
        }
    }
    cout << "时间匹配完成，共生成双差观测数据量 " << GPSDD.size() << " 个" << endl;
	
}




//  时间匹配判断函数
bool IsTimeMatched(const BDSSatePosPara& base, const BDSObsData& rover)
{
    // 检查年月日是否相同
    if (base.year != rover.year || base.month != rover.month || base.day != rover.day) 
        return false;
    // 计算时间差（秒）
    double base_seconds = base.hour * 3600.0 + base.min * 60.0 + base.sec;
    double rover_seconds = rover.hour * 3600.0 + rover.min * 60.0 + rover.sec;
    double time_diff = fabs(base_seconds - rover_seconds);
	//判断时间差是否在容差范围内
    return time_diff <= Time_Tolerance;
}

//  判断基准站时间是否较早
bool IsBaseEarlier(const BDSSatePosPara& base, const BDSObsData& rover)
{
    // 先比较年月日
    if (base.year != rover.year) return base.year < rover.year;
    if (base.month != rover.month) return base.month < rover.month;
    if (base.day != rover.day) return base.day < rover.day;

    // 比较时分秒
    if (base.hour != rover.hour) return base.hour < rover.hour;
    if (base.min != rover.min) return base.min < rover.min;
    return base.sec < rover.sec;
}

// 寻找参考卫星（返回索引）
int FindReferenceSatellite(const vector<double>& elevations)
{
    if (elevations.empty()) 
    {
        return -1;  // 没有卫星
    }

    int ref_index = 0;
    double max_elevation = elevations[0];

    for (int i = 1; i < elevations.size(); i++) 
    {
        if (elevations[i] > max_elevation) 
        {
            max_elevation = elevations[i];
            ref_index = i;
        }
    }

    return ref_index;
}
//  处理匹配的观测数据
void ProcessMatch(BDSSatePosPara& base, const BDSObsData& rover_obs, Point& base_recpos, BDS_DDObs& ddobs)
{
    //找到匹配的数据，首先计算基准站的单点定位结果
    SPPResult base_lsresult;
    BDSSatePosPara valid;
    bds_SPP(base, base_lsresult, base_recpos, valid);
    
    //单点定位成功，计算高度角
    if (base_lsresult.converged)
    {
        //存储基准站坐标
        ddobs.BasePos.x = base_lsresult.est[0];
        ddobs.BasePos.y = base_lsresult.est[1];
        ddobs.BasePos.z = base_lsresult.est[2];
        //寻找基准站和流动站的共视卫星
        vector<string>commonPRN = FindCommonSatellites(valid, rover_obs);
        // 根据共视卫星PRN过滤基准站数据
        BDSSatePosPara base_common;
        FilterBaseDataByPRN(valid, commonPRN, base_common);
		//根据共视卫星PRN过滤流动站数据
		BDSObsData rover_common;
		FilterRoverDataByPRN(rover_obs, commonPRN, rover_common);
        //存储共视卫星坐标
        ddobs.SatPos = base_common.Pos;

        //计算基准站高度角
        CalElevation(base_common.Pos, base_recpos, base_common.elevation);
        ddobs.elevation = base_common.elevation;
        //寻找参考卫星
        int ref_index = FindReferenceSatellite(base_common.elevation);
        ddobs.ref_index = ref_index;
        //根据高度角确定参考星和其他卫星
        // 检查参考卫星是否有效
        if (ref_index != -1 && ref_index < base_common.commonPRN.size())
        {
            ddobs.refPRN = base_common.commonPRN[ref_index];  
			//将其他卫星加入otherPRN
            for(int i=0; i < base_common.commonPRN.size(); i++)
            {
                if (i != ref_index)
                {
                    ddobs.otherPRN.push_back(base_common.commonPRN[i]);
                }
            }
			//计算双差观测值
            CalDoubleDif(base_common, rover_common, ddobs, ref_index);
        }
        else
        {
            cout << "错误：未找到有效的参考卫星" << endl;
            return;
        }
       
    }
    else
    {
        cout << "单点定位结果未收敛" << endl;
        return;
    }
}

// 根据PRN号过滤基准站数据（保持commonPRN顺序）
void FilterBaseDataByPRN(const BDSSatePosPara& base, const vector<string>& commonPRN, BDSSatePosPara& base_common)
{
    // 复制基本信息
    base_common.year = base.year;
    base_common.month = base.month;
    base_common.day = base.day;
    base_common.hour = base.hour;
    base_common.min = base.min;
    base_common.sec = base.sec;
    base_common.BDStime = base.BDStime;

    // 按照 commonPRN 的顺序遍历
    for (const auto& target_prn : commonPRN)
    {
        // 在基准站中查找这个PRN
        for (int i = 0; i < base.commonPRN.size(); i++)
        {
            if (base.commonPRN[i] == target_prn)
            {
                base_common.commonPRN.push_back(base.commonPRN[i]);
                if (i < base.Pos.size())base_common.Pos.push_back(base.Pos[i]);
                if (i < base.C2I.size()) base_common.C2I.push_back(base.C2I[i]);
                if (i < base.L2I.size()) base_common.L2I.push_back(base.L2I[i]);
                if (i < base.elevation.size()) base_common.elevation.push_back(base.elevation[i]);
                break;  // 找到后就跳出内层循环
            }
        }
    }
}
//根据PRN号过滤流动站数据
void FilterRoverDataByPRN(const BDSObsData& rover, const vector<string>& commonPRN, BDSObsData& rover_common)
{
    // 复制基本信息
    rover_common.year = rover.year;
    rover_common.month = rover.month;
    rover_common.day = rover.day;
    rover_common.hour = rover.hour;
    rover_common.min = rover.min;
    rover_common.sec = rover.sec;

    // 按照 commonPRN 的顺序遍历
    for (const auto& target_prn : commonPRN)
    {
        // 在流动站中查找这个PRN
        for (int i = 0; i < rover.PRN.size(); i++)
        {
            if (rover.PRN[i] == target_prn)
            {
                rover_common.PRN.push_back(rover.PRN[i]);
                if (i < rover.C2I.size()) rover_common.C2I.push_back(rover.C2I[i]);
                if (i < rover.L2I.size()) rover_common.L2I.push_back(rover.L2I[i]);
                break;  // 找到后就跳出内层循环
            }
        }
    }
}


//  查找共视卫星
vector<string> FindCommonSatellites(const BDSSatePosPara& base, const BDSObsData& rover)
{
    vector<string> common_sats;
    for (const auto& base_prn : base.commonPRN)
    {
        // 在流动站中查找相同的PRN
        for (const auto& rover_prn : rover.PRN) 
        {
            if (base_prn == rover_prn) 
            {
                common_sats.push_back(base_prn);
                break;
            }
        }
    }
    return common_sats;
}
//  查找共视卫星
vector<string> FindCommonSatellites(const EpochEph& epocheph, const GPSObsData& obs)
{
    vector<string> common_sats;

    // 遍历星历数据中的所有卫星
    for (const auto& eph_sat : epocheph.eph)
    {
        const string& eph_prn = eph_sat.PRN;

        // 在观测数据中查找相同的PRN
        for (const auto& obs_prn : obs.PRN)
        {
            if (eph_prn == obs_prn)
            {
                common_sats.push_back(eph_prn);
                break;  // 找到就跳出内层循环
            }
        }
    }
    return common_sats;
}
// 计算双差观测值
void CalDoubleDif(const BDSSatePosPara& base_obs, const BDSObsData& rover_obs, BDS_DDObs& ddobs,double ref_index)
{
	//计算双差观测值,卫星的PRN号顺序处理为相同了

    for (int i = 0; i < base_obs.commonPRN.size(); i++)
    {
        if (i != ref_index)
        {
            //计算星间单差
            double b_SC2I = base_obs.C2I[i] - base_obs.C2I[ref_index];
            double r_SC2I = rover_obs.C2I[i] - rover_obs.C2I[ref_index];
            double b_SL2I = base_obs.L2I[i] - base_obs.L2I[ref_index];
            double r_SL2I = rover_obs.L2I[i] - rover_obs.L2I[ref_index];
            //计算双差
            double DDC2I = r_SC2I - b_SC2I;
            double DDL2I = r_SL2I - b_SL2I;
            double DDL2I_m = DDL2I * clight / Freq_C2I;
            ddobs.DDC2I.push_back(DDC2I);
            ddobs.DDL2I.push_back(DDL2I);
            ddobs.DDL2I_m.push_back(DDL2I_m);
        }
    }
    
}

// 根据时间分类导航电文数据到不同历元
void ClassifyEphDataByTime(const vector<GPSEphData>& ephDataList, vector<EpochEph>& epochList)
{
    // 清空输出向量
    epochList.clear();

    // 遍历所有星历数据
    for (const auto& eph : ephDataList)
    {
        bool foundEpoch = false;

        // 在现有历元中查找相同时间的历元
        for (auto& epoch : epochList)
        {
            // 检查时间是否完全相同
            if (epoch.year == eph.year &&
                epoch.month == eph.month &&
                epoch.day == eph.day &&
                epoch.hour == eph.hour &&
                epoch.min == eph.min &&
                epoch.sec == eph.sec)
            {
                // 找到相同时间的历元，将星历数据加入
                epoch.eph.push_back(eph);
                foundEpoch = true;
                break;
            }
        }

        // 如果没有找到相同时间的历元，创建新的历元
        if (!foundEpoch)
        {
            EpochEph newEpoch;
            newEpoch.year = eph.year;
            newEpoch.month = eph.month;
            newEpoch.day = eph.day;
            newEpoch.hour = eph.hour;
            newEpoch.min = eph.min;
            newEpoch.sec = eph.sec;
            newEpoch.eph.push_back(eph);
            epochList.push_back(newEpoch);
        }
    }
}


void ClassifyBDSEphDataByTime(const vector<BDSEphData>& ephDataList, vector<BDSEpochEph>& epochList)
{
    // 清空输出向量
    epochList.clear();
    // 遍历所有星历数据
    for (const auto& eph : ephDataList)
    {
        bool foundEpoch = false;
        // 在现有历元中查找相同时间的历元
        for (auto& epoch : epochList)
        {
            // 检查时间是否完全相同
            if (epoch.year == eph.year &&
                epoch.month == eph.month &&
                epoch.day == eph.day &&
                epoch.hour == eph.hour &&
                epoch.min == eph.min &&
                epoch.sec == eph.sec)
            {
                // 找到相同时间的历元，将星历数据加入
                epoch.eph.push_back(eph);
                foundEpoch = true;
                break;
            }
        }
        // 如果没有找到相同时间的历元，创建新的历元
        if (!foundEpoch)
        {
            BDSEpochEph newEpoch;
            newEpoch.year = eph.year;
            newEpoch.month = eph.month;
            newEpoch.day = eph.day;
            newEpoch.hour = eph.hour;
            newEpoch.min = eph.min;
            newEpoch.sec = eph.sec;
            newEpoch.eph.push_back(eph);
            epochList.push_back(newEpoch);
        }
    }
}




// 时间比较函数（检查eph时间是否早于obs时间）
bool TimeEarlier(const EpochEph& eph, const GPSObsData& obs)
{
    if (eph.year != obs.year) return eph.year < obs.year;
    if (eph.month != obs.month) return eph.month < obs.month;
    if (eph.day != obs.day) return eph.day < obs.day;
    if (eph.hour != obs.hour) return eph.hour < obs.hour;
    if (eph.min != obs.min) return eph.min < obs.min;
    return eph.sec < obs.sec;
}
// 处理匹配的历元数据
void ProcessPara(const EpochEph& ephEpoch, const GPSObsData& obsData, SatePosPara& para)
{
    // 设置时间
    para.year = ephEpoch.year; para.month = ephEpoch.month; para.day = ephEpoch.day;
    para.hour = ephEpoch.hour; para.min = ephEpoch.min; para.sec = ephEpoch.sec;
    para.commonPRN = FindCommonSatellites(ephEpoch, obsData);
    // 对otherPRN按PRN号排序，确保顺序规律
    sort(para.commonPRN.begin(), para.commonPRN.end());
    cout << "共同卫星数量: " << para.commonPRN.size() << endl;
    for (int i = 0; i < para.commonPRN.size(); i++)
        cout << para.commonPRN[i] << " ";
    cout << endl;
    // 为每个共视卫星查找对应的索引并存储数据
    for (const string& prn : para.commonPRN)
    {
        // 在星历数据中查找该PRN的索引
        int eph_index = -1;
        for (int i = 0; i < ephEpoch.eph.size(); i++)
        {
            if (ephEpoch.eph[i].PRN == prn)
            {
                eph_index = i;
                break;
            }
        }

        // 在观测数据中查找该PRN的索引
        int obs_index = -1;
        for (int i = 0; i < obsData.PRN.size(); i++)
        {
            if (obsData.PRN[i] == prn)
            {
                obs_index = i;
                break;
            }
        }

        // 如果两个索引都找到，存储数据
        if (eph_index != -1 && obs_index != -1)
        {
            const GPSEphData& eph = ephEpoch.eph[eph_index];

            para.ephpara.push_back(eph.ephpara);
            // 存储观测数据
            para.C1C.push_back(obsData.C1C[obs_index]);
            para.L1C.push_back(obsData.L1C[obs_index]);
        }
    }
}
// 构建设计矩阵和观测值矩阵
void DDMatBuilder(const BDS_DDObs& ddobs, Mat& A, Mat& L, Mat& P)
{
    int n_other = ddobs.otherPRN.size();  // 非参考卫星数量
    int n_obs = 2 * n_other;              // 每个卫星有伪距和载波2个观测值
    int n_params = 3 + n_other;           // 3个位置参数 + n个模糊度参数
    double wavelength = clight / Freq_C2I;
    int ref_index = ddobs.ref_index;
    if (ref_index == -1)
    {
        cout << "错误：未找到参考卫星 " << ddobs.refPRN << endl;
        return;
    }
    Point base = ddobs.BasePos;
    Point ref;
    double ref_elev = 0.0;
    vector<Point> other;
    vector<double> other_elev;
    for (int i = 0; i < n_other + 1; i++)
    {
        if (i == ref_index)
        {
            ref = ddobs.SatPos[i];
            ref_elev = ddobs.elevation[i];
        }
        else
        {
            other.push_back(ddobs.SatPos[i]);
            other_elev.push_back(ddobs.elevation[i]);
        }
    }



    for (int i = 0; i < n_other; i++)
    {
        double dx_ref = ref.x - base.x;
        double dy_ref = ref.y - base.y;
        double dz_ref = ref.z - base.z;
        double d_ref = sqrt(dx_ref * dx_ref + dy_ref * dy_ref + dz_ref * dz_ref);
        double dx_other = other[i].x - base.x;
        double dy_other = other[i].y - base.y;
        double dz_other = other[i].z - base.z;
        double d_other = sqrt(dx_other * dx_other + dy_other * dy_other + dz_other * dz_other);
        //伪距
        A(i, 0) = -(dx_other / d_other - dx_ref / d_ref);
        A(i, 1) = -(dy_other / d_other - dy_ref / d_ref);
        A(i, 2) = -(dz_other / d_other - dz_ref / d_ref);
        
        L(i, 0) = ddobs.DDC2I[i];

        double elev_avg = (ref_elev + other_elev[i]) / 2.0;
        double elev_factor = sin(elev_avg * PI / 180.0);
        elev_factor = max(elev_factor, 0.1);
        P(i, i) = (elev_factor * elev_factor) / (1.0 * 1.0);

        //载波相位
        A(i + n_other, 0) = -(dx_other / d_other - dx_ref / d_ref);
        A(i + n_other, 1) = -(dy_other / d_other - dy_ref / d_ref);
        A(i + n_other, 2) = -(dz_other / d_other - dz_ref / d_ref);
        A(i + n_other, 3 + i) = wavelength;
        L(i + n_other, 0) = ddobs.DDL2I[i] * wavelength;
        P(i + n_other, i + n_other) = elev_factor * elev_factor / (0.01 * 0.01);
    }

}

void RTKSolve(const BDS_DDObs& ddobs)
{
    int n_other = ddobs.otherPRN.size();
    int n_obs = 2 * n_other;
    int n_params = 3 + n_other;
    Mat A(n_obs, n_params);
    Mat L(n_obs, 1);
    Mat P(n_obs, n_obs);
    DDMatBuilder(ddobs, A, L, P);
    Mat N = A.transpose() * P * A;
    Mat U = A.transpose() * P * L;
    if (fabs(N.determinant()) < 1e-12)
    {
        cout << "矩阵奇异" << endl;
    }
    Mat N_inv = N.inverse();
    Mat X = N_inv * U;
    //DeBug
    cout << "浮点解" << endl;
    cout << fixed << setprecision(6);
    cout << "dx=" << X(0, 0) << endl;
    cout << "dy=" << X(1, 0) << endl;
    cout << "dz=" << X(2, 0) << endl;
    cout << "模糊度" << endl;
    for (int i = 3; i < n_params; i++)
    {
        
        cout << X(i, 0) << " ";
        
    }
    cout << endl;

    //后面进行lambda算法，将模糊度固定为整数
    // ==================== Lambda算法固定模糊度 ====================

    int n_ambi = n_other;  // 模糊度个数
    int m = 2;             // 固定解数量（最佳解和次佳解）

    // 1. 提取模糊度浮点解
    double* a_float = new double[n_ambi];
    for (int i = 0; i < n_ambi; i++) {
        a_float[i] = X(i + 3, 0);  // 从第4个参数开始是模糊度
    }

    // 2. 提取模糊度协方差矩阵
    double* Q_ambi = new double[n_ambi * n_ambi];
    for (int i = 0; i < n_ambi; i++) {
        for (int j = 0; j < n_ambi; j++) {
            Q_ambi[i + j * n_ambi] = N_inv(i + 3, j + 3);  // 注意：协方差矩阵在N_inv中
        }
    }

    // 3. 准备输出数组
    double* F_fixed = new double[n_ambi * m];  // 固定解 (n_ambi x m)
    double* s_residual = new double[m];        // 残差平方和 (1 x m)

    // 4. 调用Lambda算法
    cout << "=== 开始Lambda模糊度固定 ===" << endl;
    int lambda_status = lambda(n_ambi, m, a_float, Q_ambi, F_fixed, s_residual);

    if (lambda_status == 0) {
        cout << "Lambda算法成功!" << endl;

        // 5. 输出固定结果
        cout << "最佳固定解模糊度: ";
        for (int i = 0; i < n_ambi; i++) {
            cout << F_fixed[i] << " ";  // 第一列是最佳解
        }
        cout << endl;

        cout << "次佳固定解模糊度: ";
        for (int i = 0; i < n_ambi; i++) {
            cout << F_fixed[i + n_ambi] << " ";  // 第二列是次佳解
        }
        cout << endl;

        cout << "残差平方和: 最佳=" << s_residual[0] << ", 次佳=" << s_residual[1] << endl;

        // 6. 计算Ratio值
        double ratio = (s_residual[1] > 1e-10) ? s_residual[1] / s_residual[0] : 999.0;
        cout << "Ratio值: " << ratio << endl;

        // 7. Ratio检验
        if (ratio > 2.0) 
        {
            cout << "Ratio检验通过! 使用固定解..." << endl;

            // 8. 用固定解更新位置
            UpdatePositionWithFixedAmbiguity(X, N_inv, F_fixed, n_ambi);
        }
        else {
            cout << "Ratio检验未通过! 使用浮点解..." << endl;
        }

    }
    else {
        cout << "Lambda算法失败! 使用浮点解..." << endl;
    }

    // 释放内存
    delete[] a_float;
    delete[] Q_ambi;
    delete[] F_fixed;
    delete[] s_residual;


}

void UpdatePositionWithFixedAmbiguity(Mat& X, const Mat& N_inv, const double* F_fixed, int n_ambi)
{
    // 固定解更新公式: 
    // X_fixed = X_float - Q_ab * Q_bb⁻¹ * (b_float - b_fixed)

    // 提取位置-模糊度协方差 Q_ab (3 x n_ambi)
    Mat Q_ab(3, n_ambi);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < n_ambi; j++) {
            Q_ab(i, j) = N_inv(i, j + 3);
        }
    }

    // 提取模糊度协方差 Q_bb (n_ambi x n_ambi)
    Mat Q_bb(n_ambi, n_ambi);
    for (int i = 0; i < n_ambi; i++) {
        for (int j = 0; j < n_ambi; j++) {
            Q_bb(i, j) = N_inv(i + 3, j + 3);
        }
    }

    // 计算浮点解与固定解的差异 (b_float - b_fixed)
    Mat diff_ambi(n_ambi, 1);
    for (int i = 0; i < n_ambi; i++) {
        diff_ambi(i, 0) = X(i + 3, 0) - F_fixed[i];  // 使用最佳解（第一列）
    }

    // 计算位置修正量
    Mat correction = Q_ab * Q_bb.inverse() * diff_ambi;

    // 更新位置解
    Mat X_fixed = X;
    for (int i = 0; i < 3; i++) {
        X_fixed(i, 0) -= correction(i, 0);
    }

    // 用固定解替换模糊度
    for (int i = 0; i < n_ambi; i++) {
        X_fixed(i + 3, 0) = F_fixed[i];
    }

    cout << "RTK固定解:";
    cout << fixed << setprecision(6);
    cout << "dx=" << X_fixed(0, 0) << " m" << ",   dy_fixed=" << X_fixed(1, 0) << " m" << ",   dz_fixed=" << X_fixed(2, 0) << " m" << endl;
    cout << "固定模糊度: ";
    for (int i = 3; i < 3 + n_ambi; i++)
    {
        cout << X_fixed(i, 0) << " ";
    }
    cout << endl;

    // 精度改善分析
    cout << fixed << setprecision(6);
    cout << "精度改善: ddx=" << (X(0, 0) - X_fixed(0, 0))
        << "m,   ddy=" << (X(1, 0) - X_fixed(1, 0))
        << "m,   ddz=" << (X(2, 0) - X_fixed(2, 0)) << " m" << endl;
}

//RTK解算
/*
In:
    vector<BDSEphData> BDSbaseEph
	vector<BDSObsData> BDSbaseObs
    vector<BDSObsData> BDSroverObs

*/
void RTKSolution(const vector<BDSEphData>& BDSbaseEph,const vector<BDSObsData>& BDSbaseObs,const vector<BDSObsData>& BDSroverObs)
{
    //将星历数据与观测值数据匹配起来
    //建立一个以卫星PRN号和相应星历数据的map
    map<string, vector<BDSEphData>>eph_map;
    eph_map = BuildBDSEphMap(BDSbaseEph);
    //建立流动站数据索引
    int rover_index = -1;
    //存储基准站单点定位结果
    Point base_recpos;
    base_recpos.x = base_recpos.y = base_recpos.z = 3678300.0;
    for (size_t i = 0; i < BDSbaseObs.size(); i++)
    {
        //存储BDS单点定位所需的星历和观测值数据
        BDSSatePosPara satepos;
        //将BDS观测值数据与星历数据匹配，匹配后将所要用到的数据存储到satepos中
        MatchBDSSatePosPara(BDSbaseObs[i], eph_map, satepos);
        //获取BDS时间
        gregorianToBDSTime(satepos.year, satepos.month, satepos.day, satepos.hour, satepos.min, satepos.sec, satepos.BDStime);

        //得到用于单点定位的satepos数据后，进行基准站数据与流动站数据的匹配
        //新建存储进行RTK的存储双差数据及卫星位置，高度角等数据
        BDS_DDObs BDS_ddobs;

        
        //匹配基准站与流动站数据，匹配后将流动站数据索引更新，下次寻找从上一次索引开始
        if (Match_Base_Rover(satepos, BDSroverObs, rover_index))
        {
            //匹配成功，则建立双差观测值数据
            BDSObsData rover_obs = BDSroverObs[rover_index];
            ProcessMatch(satepos, rover_obs, base_recpos, BDS_ddobs);
            //进行RTK解算
            RTKSolve(BDS_ddobs);
        }

        //没有匹配成功则进入下一个基准站数据与流动站数据匹配

        
    }
}

bool Match_Base_Rover(const BDSSatePosPara satepos, const vector<BDSObsData>& BDSroverObs, int& Rover_index)
{
    // 输入参数检查
    if (BDSroverObs.empty()) 
    {
        cout << "错误：流动站观测数据为空" << endl;
        return false;
    }

    // 索引边界检查和处理
    if (Rover_index < -1 || Rover_index >= BDSroverObs.size()) 
    {
        cout << "警告：流动站索引越界，重置为起始位置" << endl;
        Rover_index = -1;
    }

    // 如果已到末尾，重置索引
    if (Rover_index >= BDSroverObs.size() - 1) 
    {
        Rover_index = -1;
    }
    int start_index = Rover_index + 1;
    bool found_match = false;
    int match_index = -1;
    for (int i = start_index; i < BDSroverObs.size(); i++)
    {
        if (IsTimeMatched(satepos, BDSroverObs[i]))
        {
            match_index = i;
            found_match = true;
            break;
        }
    }
    

    if (found_match)
    {
        Rover_index = match_index;

        return true;
    }
    // 如果从中间开始没找到，尝试从头搜索
    if (start_index > 0)
    {
        cout << "从当前位置未找到匹配，尝试从头搜索..." << endl;
        Rover_index = -1;
        return Match_Base_Rover(satepos, BDSroverObs, Rover_index);
    }
    cout << "未找到时间 " << satepos.year << " " << satepos.month << " " << satepos.day << " "
        << satepos.hour << " " << satepos.min << " " << satepos.sec << " " << " 的流动站匹配数据" << endl;
    return false;
   
}

/*
void LambdaSolve(const Mat& float_sol, const Mat& Q, int n_ambi)
{
    // 提取模糊度部分
    Mat a_float(n_ambi, 1); // 模糊度浮点解
    Mat Q_aa(n_ambi, n_ambi); // 模糊度协方差矩阵
    for (int i = 0; i < n_ambi; i++)
    {
        a_float(i, 0) = float_sol(i + 3, 0);
        for (int j = 0; j < n_ambi; j++)
        {
            Q_aa(i, j) = Q(i + 3, j + 3);
        }
    }



    cout << "=== Lambda算法开始 ===" << endl;
    cout << "模糊度数量: " << n_ambi << endl;
    // 输出提取的模糊度用于验证
    cout << "提取的模糊度浮点解: ";
    for (int i = 0; i < n_ambi; i++) {
        cout << a_float(i, 0) << " ";
    }
    cout << endl;
    // 1. LDL分解降相关
    Mat L(n_ambi, n_ambi);
    Mat D(n_ambi, n_ambi);
    LDLTDecomposition(Q_aa, L, D);

    // 2. Z变换（整数高斯变换）
    Mat Z(n_ambi, n_ambi);
    Mat a_z(n_ambi, 1);
    ZTransform(L, D, a_float, Z, a_z);

    // 3. 整数搜索
    vector<double> fixed_amb;
    double ratio = 0;
    bool success = IntegerSearch(a_z, D, Z, fixed_amb, ratio);

    if (success)
    {
        cout << "模糊度固定成功! Ratio值: " << ratio << endl;
        cout << "固定模糊度: ";
        for (size_t i = 0; i < fixed_amb.size(); i++)
        {
            cout << fixed_amb[i] << " ";
        }
        cout << endl;

        // 4. 更新位置解
        UpdateFixedSolution(float_sol, Q, fixed_amb, n_ambi);
    }
    else
    {
        cout << "模糊度固定失败! Ratio值: " << ratio << " (建议 > 2.0)" << endl;
        cout << "使用浮点解继续..." << endl;
    }
}



void LDLTDecomposition(const Mat& Q, Mat& L, Mat& D)
{
    int n = Q.row;
    L = Mat::Identity(n);
    D = Mat(n, n);

    for (int i = 0; i < n; i++)
    {
        D(i, i) = Q(i, i);
        for (int k = 0; k < i; k++)
        {
            D(i, i) -= L(i, k) * L(i, k) * D(k, k);
        }

        for (int j = i + 1; j < n; j++)
        {
            L(j, i) = Q(j, i);
            for (int k = 0; k < i; k++)
            {
                L(j, i) -= L(j, k) * L(i, k) * D(k, k);
            }
            L(j, i) /= D(i, i);
        }
    }
}

void ZTransform(const Mat& L, const Mat& D, const Mat& a_float, Mat& Z, Mat& a_z)
{
    int n = L.row;
    Z = L.inverse();  // Z变换矩阵
    a_z = Z * a_float; // 变换后的模糊度

    cout << "Z变换后的模糊度: ";
    for (int i = 0; i < n; i++)
    {
        cout << a_z(i, 0) << " ";
    }
    cout << endl;
}

*/