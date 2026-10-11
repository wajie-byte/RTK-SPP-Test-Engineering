#include"../BASEDLL/Base.h"
#include"../OEMRead/BinRead.h"
#include"newRTK.h"
#include<sstream>
#include<iomanip>
#include"../SPP/newSPP.h"
#include"../LAMBDA/lambda.h"
#include"RTKSoution.h"



OBSS Match_RoverObs_BaseLS(const OBSS& rover_obs, const LSResult& base_lsresult)
{
	OBSS cmsat_rover_obs;//基于卫星系统与PRN比较得到的一组观测值数据，还未加入频率比较
	cmsat_rover_obs.GPStime = rover_obs.GPStime;
	cmsat_rover_obs.BDStime = rover_obs.BDStime;
	//根据卫星匹配观测值
	int base_sat_num = base_lsresult.sat_prns.size();//基准站可视卫星
	for (int i = 0; i < base_sat_num; i++)
	{
		//比较基准站与流动站卫星PRN进行匹配
		string base_sat_prn = base_lsresult.sat_prns[i];
		//记录流动站观测值索引

		for (int j = 0; j < rover_obs.n_obs; j++)
		{
			string rover_sat_prn = GetSatPRN(rover_obs.obs[j].system, rover_obs.obs[j].PRN);

			//匹配成功
			if (rover_sat_prn == base_sat_prn)
			{

				//将信号记录下来，接下来还要与基准站信号比较频率是否匹配
				cmsat_rover_obs.obs.push_back(rover_obs.obs[j]);
				//不需要break，因为要找出所有数据
			}
			else continue;
		}


	}
	cmsat_rover_obs.n_obs = cmsat_rover_obs.obs.size();
	return cmsat_rover_obs;
}

void Match_RoverObs_BaseObs(const OBSS& rover_obs, const OBSS& base_obs,OBSS& rover,OBSS& base)
{
	base.GPStime = base_obs.GPStime; base.BDStime = base_obs.BDStime;
	rover.GPStime = rover_obs.GPStime; rover.BDStime = rover_obs.BDStime;
	for (int i = 0; i < base_obs.n_obs; i++)
	{
		for (int j = 0; j < rover_obs.n_obs; j++)
		{
			if ((base_obs.obs[i].system == rover_obs.obs[j].system) &&
				(base_obs.obs[i].signal_type == rover_obs.obs[j].signal_type) &&
				(base_obs.obs[i].PRN == rover_obs.obs[j].PRN))
			{
				//如果系统，频率，PRN号都匹配，则添加到base和rover中
				base.obs.push_back(base_obs.obs[i]);
				rover.obs.push_back(rover_obs.obs[j]);//此时base和rover之间的系统和频率，卫星是对应的
			}
			else continue;
		}
	}
	base.n_obs = base.obs.size();
	rover.n_obs = rover.obs.size();
}


DDOBSS Match_BaseObs_BaseLS(const OBSS&base,const OBSS&rover,const LSResult& base_lsresult, const LSResult& rover_lsresult)
{
	DDOBSS dd_obss;
	dd_obss.base = base;
	dd_obss.rover = rover;
	dd_obss.Base_pos.x = base_lsresult.est[0];
	dd_obss.Base_pos.y = base_lsresult.est[1];
	dd_obss.Base_pos.z = base_lsresult.est[2];
	dd_obss.Rover_pos.x = rover_lsresult.est[0];
	dd_obss.Rover_pos.y = rover_lsresult.est[1];
	dd_obss.Rover_pos.z = rover_lsresult.est[2];

	//从lsresult中获取卫星位置，高度角,基准站位置
	int n_obs = base.n_obs;
	//创建储存可以进行双差矩阵构建的观测值
	for (int i = 0; i < n_obs; i++)
	{
		string sat_prn = GetSatPRN(base.obs[i].system, base.obs[i].PRN);


		for (int j = 0; j < base_lsresult.sat_elevs.size(); j++)
		{
			string base_sat_prn = base_lsresult.sat_prns[j];
			//匹配成功
			if (sat_prn == base_sat_prn)
			{
				dd_obss.sat_prns.push_back(sat_prn);
				dd_obss.elev.push_back(base_lsresult.sat_elevs[j]);
				dd_obss.SatPos.push_back(base_lsresult.sat_pos[j]);
				//找到卫星就break，找下一个观测值对应的卫星

			}
			else continue;
		}
	}
	return dd_obss;
}

DualOBS Match_Process_DD(const DDOBSS& dd_obss)
{
	DualOBS dd;
	int dd_obs = dd_obss.base.n_obs;
	
	dd.GPStime = dd_obss.base.GPStime;
	dd.BDStime = dd_obss.base.BDStime;
	dd.Base_pos = dd_obss.Base_pos;
	dd.Rover_pos = dd_obss.Rover_pos;
	for (int i = 0; i < dd_obs - 1; i++)
	{
		for (int j = i + 1; j < dd_obs; j++)
		{
			if (dd_obss.base.obs[i].system == dd_obss.base.obs[j].system &&
				dd_obss.base.obs[i].PRN == dd_obss.base.obs[j].PRN)
			{
				dd.sat_prns.push_back(dd_obss.sat_prns[i]);
				dd.elev.push_back(dd_obss.elev[i]);
				dd.SatPos.push_back(dd_obss.SatPos[i]);
				PSR_ADR base_b1i;
				PSR_ADR base_b3i;
				PSR_ADR rover_b1i;
				PSR_ADR rover_b3i;
				if (dd_obss.base.obs[i].signal_type == BDS_B1I_D1)
				{
					base_b1i.psr = dd_obss.base.obs[i].psr;
					base_b3i.psr = dd_obss.base.obs[j].psr;
					base_b1i.adr = dd_obss.base.obs[i].adr;
					base_b3i.adr = dd_obss.base.obs[j].adr;
					rover_b1i.psr = dd_obss.rover.obs[i].psr;
					rover_b3i.psr = dd_obss.rover.obs[j].psr;
					rover_b1i.adr = dd_obss.rover.obs[i].adr;
					rover_b3i.adr = dd_obss.rover.obs[j].adr;
					dd.Base_B1I.push_back(base_b1i);
					dd.Base_B3I.push_back(base_b3i);
					dd.Rover_B1I.push_back(rover_b1i);
					dd.Rover_B3I.push_back(rover_b3i);
				}
				else
				{
					base_b1i.psr = dd_obss.base.obs[j].psr;
					base_b3i.psr = dd_obss.base.obs[i].psr;
					base_b1i.adr = dd_obss.base.obs[j].adr;
					base_b3i.adr = dd_obss.base.obs[i].adr;
					rover_b1i.psr = dd_obss.base.obs[j].psr;
					rover_b3i.psr = dd_obss.base.obs[i].psr;
					rover_b1i.adr = dd_obss.base.obs[j].adr;
					rover_b3i.adr = dd_obss.base.obs[i].adr;
					dd.Base_B1I.push_back(base_b1i);
					dd.Base_B3I.push_back(base_b3i);
					dd.Rover_B1I.push_back(rover_b1i);
					dd.Rover_B3I.push_back(rover_b3i);
				}
			}
		}
	}
	return dd;
}


bool DoubleDifMatch(const OBSS& base_obs, const OBSS& rover_obs, const LSResult& base_lsresult,const LSResult& rover_lsresult, DualOBS& dd)
{
	double time_tolerance = 1e-3;
	if (base_lsresult.sat_prns.size() < 4|| rover_lsresult.sat_prns.size() < 4)
	{
		cout << "卫星数量不足" << endl;
		return false;
	}
	if (base_obs.n_obs < 4 || rover_obs.n_obs < 4)
	{
		cout << "观测值数量不足" << endl;
		return false;
	}

	//检测时间是否同步
	double time_diff = (base_obs.GPStime.week - rover_obs.GPStime.week) * SecInWeek + (base_obs.GPStime.sow - rover_obs.GPStime.sow);
	if (fabs(time_diff) < time_tolerance)
	{
		
		//基于卫星系统与PRN比较，还未加入频率比较，将流动站观测值中与基准站相同卫星的观测值提取出来
		OBSS cmsat_rover_obs = Match_RoverObs_BaseLS(rover_obs, base_lsresult);
		
		//经过上面的步骤，现在已经得到了流动站中与基准站观测相同卫星来源的观测值数据



		//接下来的步骤是将base_obs与cmsat_rover_obs进行比较,主要目的是将cmsat_rover_obs中与base_obs相同类型的数据提取出来
		//因为rover的观测值数据已经经过卫星检验了，所以接下来可以直接与base_obs比较，不用担心会匹配到没有使用到的卫星

		//接下来的步骤是将base_obs与rover_obs进行比较,主要目的是将rover_obs中与base_obs相同类型的数据提取出来
		//主要比较的是信号对应的系统，频率，PRN号
		
		OBSS base;//记录最后构建双差要用的base观测值
		OBSS rover;//记录最后构建双差要用的rover观测值
		Match_RoverObs_BaseObs(cmsat_rover_obs, base_obs, rover, base);


		//到这里，base和rover中的观测值已经是对应的了，可以进行双差观测值的构建了
		//在构建过程中要注意base_obs的观测值卫星与base_lsresult中的卫星可能并不完全对应，所以还需要进行匹配
		//同时获取卫星位置和高度角信息
		DDOBSS dd_obss = Match_BaseObs_BaseLS(base, rover, base_lsresult, rover_lsresult);
		
		//将基准站和流动站数据按频率进行分组
		dd = Match_Process_DD(dd_obss);

		//到这里就完成了双差观测值的匹配
		return true;

	}
	else
	{
		cout << "时间不匹配" << endl;
		return false;
	}


}



void BuildDDMat(const DualOBS& dd, Mat& A, Mat& L, Mat& P,RTKResult& rtk_result)
{
	rtk_result.other_prns.clear();
	rtk_result.other_elevs.clear();

	int ref_index = -1;//获取参考星的索引
	int n_dd_sat = dd.elev.size();
	double max_elev = 0;
	for (int i = 0; i < n_dd_sat; i++)
	{
		if (dd.elev[i] > max_elev)
		{
			max_elev = dd.elev[i];
			ref_index = i;
		}
	}
	if (ref_index == -1)
	{
		cout << "错误：未找到参考卫星 " << endl;
		return;
	}

	//得到参考星的索引后就可以开始构建矩阵
	int n_other = n_dd_sat - 1;
	int n_obs = n_other * 2 * 2;//每颗卫星有双频，有伪距和载波相位观测值
	int n_params = 3 + n_other * 2;//3个基线向量的分量和2*n_other个模糊度参数

	Point base = dd.Base_pos;
	Point rover = dd.Rover_pos;
	Point ref;
	double ref_elev = 0.0;
	vector<Point> other;
	vector<double> other_elev;

	for (int i = 0; i < n_other + 1; i++)
	{
		if (i == ref_index)
		{
			ref = dd.SatPos[i];
			ref_elev = dd.elev[i];
			rtk_result.ref_sat = dd.sat_prns[i];
			rtk_result.ref_elev = ref_elev;
		}
		else
		{
			other.push_back(dd.SatPos[i]);
			other_elev.push_back(dd.elev[i]);
			rtk_result.other_prns.push_back(dd.sat_prns[i]);
			rtk_result.other_elevs.push_back(dd.elev[i]);
		}
	}
	vector<double>DD_B1I_C;
	vector<double>DD_B1I_L;
	vector<double>DD_B3I_C;
	vector<double>DD_B3I_L;

	for (int i = 0; i < n_other + 1; i++)
	{
		if (i != ref_index)
		{
			//计算星间单差
			double b_SCB1I = dd.Base_B1I[i].psr - dd.Base_B1I[ref_index].psr;
			double r_SCB1I = dd.Rover_B1I[i].psr - dd.Rover_B1I[ref_index].psr;
			double b_SLB1I = dd.Base_B1I[i].adr - dd.Base_B1I[ref_index].adr;
			double r_SLB1I = dd.Rover_B1I[i].adr - dd.Rover_B1I[ref_index].adr;
			double b_SCB3I = dd.Base_B3I[i].psr - dd.Base_B3I[ref_index].psr;
			double r_SCB3I = dd.Rover_B3I[i].psr - dd.Rover_B3I[ref_index].psr;
			double b_SLB3I = dd.Base_B3I[i].adr - dd.Base_B3I[ref_index].adr;
			double r_SLB3I = dd.Rover_B3I[i].adr - dd.Rover_B3I[ref_index].adr;
			//计算双差
			double DDCB1I = r_SCB1I - b_SCB1I;
			double DDLB1I = r_SLB1I - b_SLB1I;

			double DDCB3I = r_SCB3I - b_SCB3I;
			double DDLB3I = r_SLB3I - b_SLB3I;

			DD_B1I_C.push_back(DDCB1I);
			DD_B1I_L.push_back(DDLB1I);
			DD_B3I_C.push_back(DDCB3I);
			DD_B3I_L.push_back(DDLB3I);
		}

	}



	for (int i = 0; i < n_other; i++)
	{
		//基站到参考星矢量
		double dx_b_ref = ref.x - base.x;
		double dy_b_ref = ref.y - base.y;
		double dz_b_ref = ref.z - base.z;
		double d_b_ref = sqrt(dx_b_ref * dx_b_ref + dy_b_ref * dy_b_ref + dz_b_ref * dz_b_ref);
		//基站到其他星矢量
		double dx_b_other = other[i].x - base.x;
		double dy_b_other = other[i].y - base.y;
		double dz_b_other = other[i].z - base.z;
		double d_b_other = sqrt(dx_b_other * dx_b_other + dy_b_other * dy_b_other + dz_b_other * dz_b_other);
		//流动站到参考星矢量
		double dx_r_ref = ref.x - rover.x;
		double dy_r_ref = ref.y - rover.y;
		double dz_r_ref = ref.z - rover.z;
		double d_r_ref = sqrt(dx_r_ref * dx_r_ref + dy_r_ref * dy_r_ref + dz_r_ref * dz_r_ref);
		//流动站到其他星矢量
		double dx_r_other = other[i].x - rover.x;
		double dy_r_other = other[i].y - rover.y;
		double dz_r_other = other[i].z - rover.z;
		double d_r_other = sqrt(dx_r_other * dx_r_other + dy_r_other * dy_r_other + dz_r_other * dz_r_other);
		//双差计算值
		
		double dd_Cal = (d_r_other - d_r_ref) - (d_b_other - d_b_ref);
		//计算高度角因子
		double elev_avg = (ref_elev + other_elev[i]) / 2.0;
		double elev_factor = sin(elev_avg * PI / 180.0);
		elev_factor = max(elev_factor, 0.1);
		//B1I载波相位
		A(i, 0) = -(dx_r_other / d_r_other - dx_r_ref / d_r_ref);
		A(i, 1) = -(dy_r_other / d_r_other - dy_r_ref / d_r_ref);
		A(i, 2) = -(dz_r_other / d_r_other - dz_r_ref / d_r_ref);
		A(i, 3 + i) = BDS_B1_WAVELEN;
		L(i, 0) = DD_B1I_L[i] * BDS_B1_WAVELEN - dd_Cal;
		P(i, i) = (elev_factor * elev_factor) / (0.01 * 0.01);

		//B3I载波相位
		A(i + n_other, 0) = -(dx_r_other / d_r_other - dx_r_ref / d_r_ref);
		A(i + n_other, 1) = -(dy_r_other / d_r_other - dy_r_ref / d_r_ref);
		A(i + n_other, 2) = -(dz_r_other / d_r_other - dz_r_ref / d_r_ref);
		A(i + n_other, 3 + i + n_other) = BDS_B3_WAVELEN;
		L(i + n_other, 0) = DD_B3I_L[i] * BDS_B3_WAVELEN - dd_Cal;
		P(i + n_other, i + n_other) = elev_factor * elev_factor / (0.01 * 0.01);

		//B1I伪距
		A(i + 2 * n_other, 0) = -(dx_r_other / d_r_other - dx_r_ref / d_r_ref);
		A(i + 2 * n_other, 1) = -(dy_r_other / d_r_other - dy_r_ref / d_r_ref);
		A(i + 2 * n_other, 2) = -(dz_r_other / d_r_other - dz_r_ref / d_r_ref);

		L(i + 2 * n_other, 0) = DD_B1I_C[i] - dd_Cal;
		P(i + 2 * n_other, i + 2 * n_other) = (elev_factor * elev_factor) / (1.0 * 1.0);

		//B3I伪距
		A(i + 3 * n_other, 0) = -(dx_r_other / d_r_other - dx_r_ref / d_r_ref);
		A(i + 3 * n_other, 1) = -(dy_r_other / d_r_other - dy_r_ref / d_r_ref);
		A(i + 3 * n_other, 2) = -(dz_r_other / d_r_other - dz_r_ref / d_r_ref);

		L(i + 3 * n_other, 0) = DD_B3I_C[i] - dd_Cal;
		P(i + 3 * n_other, i + 3 * n_other) = (elev_factor * elev_factor) / (1.0 * 1.0);
	}

}




void Solve_RTK(const OBSS& base_obs, const OBSS& rover_obs, const LSResult& base_lsresult, const LSResult& rover_lsresult,RTKResult& rtk_result)
{
	DualOBS dd;

	rtk_result.GPStime = base_obs.GPStime;

	Mat A;
	Mat L;
	Mat P;
	Mat N;
	Mat U;
	Mat X;
	Mat N_inv;
	int n_dd_sat = 0;
	int n_other = 0;
	int n_obs = 0;//每颗卫星有双频，有伪距和载波相位观测值
	int n_params = 0;//3个基线向量的分量和2*n_other个模糊度参数
	if (DoubleDifMatch(base_obs,rover_obs,base_lsresult,rover_lsresult,dd))
	{
		n_dd_sat = dd.elev.size();
		n_other = n_dd_sat - 1;
		n_obs = n_other * 2 * 2;//每颗卫星有双频，有伪距和载波相位观测值
		n_params = 3 + n_other * 2;//3个基线向量的分量和2*n_other个模糊度参数
		//定义矩阵大小
		A = Mat(n_obs, n_params);
		L = Mat(n_obs, 1);
		P = Mat(n_obs, n_obs);

		
	}
	else
	{
		return;
	}

	
	//计算基线向量初值
	double dx = dd.Rover_pos.x - dd.Base_pos.x;
	double dy = dd.Rover_pos.y - dd.Base_pos.y;
	double dz = dd.Rover_pos.z - dd.Base_pos.z;
	double BaseLine_Vec[3] = { dx,dy,dz };

	//开始进行最小二乘求解
	for (int iter = 0; iter < 5; iter++)
	{
		BuildDDMat(dd, A, L, P, rtk_result);
		//计算法方程
		N = A.transpose() * P * A;
		U = A.transpose() * P * L;
		if (fabs(N.determinant()) < 1e-12)
		{
			cout << "矩阵奇异" << endl;
			return;
		}
		N_inv = N.inverse();
		X = N_inv * U;
		//收敛条件
		if (fabs(X(0, 0)) < 1e-4 && fabs(X(1, 0)) < 1e-4 && fabs(X(2, 0)) < 1e-4)
		{
			//收敛则说明rtk浮点解确定
			rtk_result.status = 0; // 浮点解

			break;
		}
		//最后一次的结果不更新基线向量，因为要进行模糊度固定
		dd.Rover_pos.x += X(0, 0);
		dd.Rover_pos.y += X(1, 0);
		dd.Rover_pos.z += X(2, 0);
		
		BaseLine_Vec[0] += X(0, 0);
		BaseLine_Vec[1] += X(1, 0);
		BaseLine_Vec[2] += X(2, 0);
		
		
		

	}


	if(rtk_result.status == -1)
	{
		//如果没有收敛，则说明浮点解未确定，直接返回
		cout << "浮点解未收敛，RTK解算失败" << endl;
		return;
	}
	else
	{
		//DeBug
		cout << "GPSTime:" << dd.GPStime.week << " " << dd.GPStime.sow << endl;
		cout << "浮点解：";
		cout << fixed << setprecision(6);
		for (int i = 0; i < 3; i++)
		{
			double Float = BaseLine_Vec[i] + X(i, 0);
			cout << Float << " m ";
			//保存浮点解结果
			rtk_result.baseline_Float[i] = Float;
		}
		cout << endl;

		cout << "浮点解模糊度：";
		for (int i = 3; i < n_params; i++)
		{
			cout << X(i, 0) << " ";
			//保存浮点模糊度结果
			//从线性化函数可以得知，前面是B1I模糊度，后面是B3I模糊度
			rtk_result.N_Float.push_back(X(i, 0));
		}
		cout << endl;

		//后面进行lambda算法，将模糊度固定为整数
		// ==================== Lambda算法固定模糊度 ====================

		int n_ambi = 2 * n_other;  // 模糊度个数
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
		//cout << "=== 开始Lambda模糊度固定 ===" << endl;
		int lambda_status = lambda(n_ambi, m, a_float, Q_ambi, F_fixed, s_residual);

		if (lambda_status == 0)
		{
			//更新RTK解算状态
			

			/*
			cout << "Lambda算法成功!" << endl;
			cout << "最佳固定解: ";
			// 5. 输出固定结果

			for (int i = 0; i < n_ambi; i++)
			{
				cout << F_fixed[i] << " ";  // 第一列是最佳解
			}
			cout << endl;
			*/

			/*
			cout << "次佳固定解模糊度: ";
			for (int i = 0; i < n_ambi; i++) {
				cout << F_fixed[i + n_ambi] << " ";  // 第二列是次佳解
			}
			cout << endl;
			*/
			//cout << "残差平方和: 最佳=" << s_residual[0] << ", 次佳=" << s_residual[1] << endl;

			// 6. 计算Ratio值
			double ratio = (s_residual[1] > 1e-10) ? s_residual[1] / s_residual[0] : 999.0;
			cout << "Ratio值: " << ratio << endl;
			rtk_result.ratio = ratio;
			// 7. Ratio检验
			if (ratio > 2.0)
			{
				cout << "Ratio检验通过! 使用固定解" << endl;
				rtk_result.status = 1; // 固定解成功
				// 8. 用固定解更新位置
				UpdatePositionWithFixedAmbiguity_r(X, N_inv, F_fixed, n_ambi, rtk_result);
			}
			else 
			{
				cout << "Ratio检验未通过! 使用浮点解" << endl;
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
	


	return;
}

void UpdatePositionWithFixedAmbiguity_r(Mat& X, const Mat& N_inv, const double* F_fixed, int n_ambi,RTKResult& rtk_result)
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
	for(int i=0;i<3;i++)
	{
		double Fixed = rtk_result.baseline_Float[i] + X_fixed(i, 0);
		cout << Fixed << " m ";
		//保存固定解结果
		rtk_result.baseline_Fixed[i] = Fixed;
	}
	cout << endl;
	cout << "固定模糊度: ";
	for (int i = 3; i < 3 + n_ambi; i++)
	{
		cout << X_fixed(i, 0) << " ";
		//保存固定模糊度结果
		rtk_result.N_Fixed.push_back(X_fixed(i, 0));
	}
	cout << endl;

	// 精度改善分析
	cout << fixed << setprecision(6);
	cout << "精度改善: ddx=" << (X(0, 0) - X_fixed(0, 0))
		<< "m,   ddy=" << (X(1, 0) - X_fixed(1, 0))
		<< "m,   ddz=" << (X(2, 0) - X_fixed(2, 0)) << " m" << endl;
}