#include"SatePos.h"
#include"../MATDLL/Mat.h"
#include"../BASEDLL/Base.h"
// 添加：计算BDS时间差
double CalculateTimeDiffBDS(const BDSTime& obs_time, const BDSTime& eph_time)
{
	double time_diff = (obs_time.week - eph_time.week) * 604800.0 + (obs_time.sow - eph_time.sow);

	// 处理时间卷绕
	if (time_diff > 302400.0)  // 604800 / 2 = 302400
	{
		time_diff -= 604800.0;
	}
	else if (time_diff < -302400.0)
	{
		time_diff += 604800.0;
	}

	return time_diff;
}


// 相对论效应改正计算
double calculateRelativisticCorrection(const EphPara& eph, double Ek)
{
	/**
	 * 计算周期性相对论效应改正
	 * 公式: dtr = -2 * sqrt(GM * a) * e * sin(Ek) / c^2
	 */
	const double GM_GPS = 3.986005e14;
	const double C = 299792458.0;

	double a = eph.sqrtA * eph.sqrtA;  // 轨道长半轴
	double dtr = -2.0 * sqrt(GM_GPS * a) * eph.e * sin(Ek) / (C * C);

	return dtr;
}

double calculateApproximateRelativisticCorrection(const EphPara& eph, double dt_orbit)
{
	double n0 = sqrt(GM_GPS) / pow(eph.sqrtA, 3.0);
	double n = n0 + eph.deltan;
	double Mk = eph.M0 + n * dt_orbit;

	// 近似计算偏近点角（单次迭代）
	double Ek = Mk + eph.e * sin(Mk);

	// 相对论效应改正量
	return -4.442807633e-10 * eph.e * eph.sqrtA * sin(Ek);
}

double calculateTransmitTime(double receiverTime, double pseudorange, const EphPara& eph)
{
	double transmitTime = receiverTime - pseudorange / clight;

	for (int iter = 0; iter < 6; iter++)
	{
		// 钟差时间差计算
		double dt_clock = transmitTime - (eph.Toc + 604800 * eph.Toe_week);
		if (dt_clock > 302400.0) dt_clock -= 604800.0;
		if (dt_clock < -302400.0) dt_clock += 604800.0;

		// 轨道时间差计算（用于相对论效应）
		double dt_orbit = transmitTime - (eph.Toe + 604800 * eph.Toe_week);
		if (dt_orbit > 302400.0) dt_orbit -= 604800.0;
		if (dt_orbit < -302400.0) dt_orbit += 604800.0;

		// 基本卫星钟差
		double satClockBias = eph.a0 + eph.a1 * dt_clock + eph.a2 * dt_clock * dt_clock;

		// 计算相对论效应改正
		double relativisticCorrection = calculateApproximateRelativisticCorrection(eph, dt_orbit);

		// TGD改正（在时间域）
		double tgdCorrection = eph.TGD;

		// 总钟差
		double totalClockCorrection = satClockBias + relativisticCorrection - tgdCorrection;

		// 重新计算发射时间
		double correctedPseudorange = pseudorange - totalClockCorrection * clight;
		double newTransmitTime = receiverTime - correctedPseudorange / clight - totalClockCorrection;

		// 收敛检查
		if (fabs(newTransmitTime - transmitTime) < 1e-10)
		{
			break;
		}

		transmitTime = newTransmitTime;
	}

	return transmitTime;
}

// 修改：去掉相对论效应参数，避免重复计算
double computePreciseSatelliteClock(double transmitTime, const EphPara& eph)
{
	double totalToc = eph.Toc + 604800 * eph.Toe_week;
	double dt_clock = transmitTime - totalToc;

	// 时间跨周处理
	if (dt_clock > 302400.0) dt_clock -= 604800.0;
	if (dt_clock < -302400.0) dt_clock += 604800.0;

	// 基本钟差计算（不含相对论效应）
	double basicBias = eph.a0 + eph.a1 * dt_clock + eph.a2 * dt_clock * dt_clock;

	// TGD改正（减去）
	double tgdCorrection = eph.TGD;

	return basicBias - tgdCorrection;
}




bool Clock_Cal(int year, int month, int day, int hour, int min, double sec, EphPara eph,GPSTime GPStime,double& Clock_bias,double& Clock_vel)
{
	double limittime = 3600.0;
	GPSTime ct;
	ct = GPStime;
	double traveltime = (ct.week - eph.Toe_week) * 604800.0 + (ct.sow - eph.Toe);
	if (traveltime > limittime)return false;
	double dt_clock = (ct.week - eph.Toe_week) * 604800.0 + (ct.sow - eph.Toc);
	Clock_bias = eph.a0 + eph.a1 * dt_clock + eph.a2 * dt_clock * dt_clock;
	Clock_vel = eph.a1 + 2 * eph.a2 * dt_clock;
	return true;
}




bool GPSSatPVT(EphPara eph, GPSTime GPStime,Point& Satpos,double &Clock_bias,double& Clock_vel)
{
	double A, n, tk, Mk, Ek, vk, FAIk;
	double deltauk, deltark, deltaik, uk, rk, ik, xk, yk;
	double OMEGAeDOT = 7.2921151467e-5;
	double OMEGAk, rotation_angle;
	double Ekdot, FAIkdot, ukdot, rkdot, Ikdot, xksdot, yksdot;

	//卫星位置矢量，速度旋转参数向量
	Mat Position(3, 1);
	Mat DotS(4, 1);
	//旋转矩阵
	Mat Rdot(3, 4);
	A = eph.sqrtA * eph.sqrtA;
	tk = (GPStime.week - eph.Toe_week) * 604800 + GPStime.sow - eph.Toe;
	n = sqrt(GM_GPS / (A * A * A)) + eph.deltan;
	Mk = eph.M0 + n * tk;
	double Ek0 = Mk;//偏近点角赋初值
	Ek = Mk + eph.e * sin(Ek0);
	for (int i = 0; abs(Ek - Ek0) > 1e-12; i++)//迭代计算偏近点角
	{
		Ek0 = Ek;
		Ek = Mk + eph.e * sin(Ek0);
	}
	vk = atan2(sqrt(1 - eph.e * eph.e) * sin(Ek),
		cos(Ek) - eph.e);
	FAIk = vk + eph.omega;
	deltauk = eph.Cus * sin(2 * FAIk) + eph.Cuc * cos(2 * FAIk);
	deltark = eph.Crs * sin(2 * FAIk) + eph.Crc * cos(2 * FAIk);
	deltaik = eph.Cis * sin(2 * FAIk) + eph.Cic * cos(2 * FAIk);
	uk = FAIk + deltauk;
	rk = A * (1 - eph.e * cos(Ek)) + deltark;
	ik = eph.i0 + deltaik + eph.IDOT * tk;
	xk = rk * cos(uk);
	yk = rk * sin(uk);
	OMEGAk = eph.OMEGA + (eph.OMEGADOT - OMEGAeDOT) * tk - OMEGAeDOT * eph.Toe;
	Position(0, 0) = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
	Position(1, 0) = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
	Position(2, 0) = yk * sin(ik);
	//获取卫星位置
	Satpos.x = Position(0, 0);
	Satpos.y = Position(1, 0);
	Satpos.z = Position(2, 0);
	//对卫星钟差相对论效应改正
	Clock_bias += Ftr_GPS * eph.e * eph.sqrtA * sin(Ek);//对卫星钟差进行相对论效应改正
	Ekdot = n / (1 - eph.e * cos(Ek));
	FAIkdot = sqrt(1 - eph.e * eph.e) / (1 - eph.e * cos(Ek)) * Ekdot;
	ukdot = 2 * (eph.Cus * cos(2 * FAIk) - eph.Cuc * sin(2 * FAIk)) * FAIkdot + FAIkdot;
	rkdot = A * eph.e * sin(Ek) * Ekdot + 2 * (eph.Crs * cos(2 * FAIk) - eph.Crc * sin(2 * FAIk)) * FAIkdot;
	Ikdot = eph.IDOT + 2 * (eph.Cis * cos(2 * FAIk) - eph.Cic * sin(2 * FAIk)) * FAIkdot;
	xksdot = rkdot * cos(uk) - rk * ukdot * sin(uk);
	yksdot = rkdot * sin(uk) + rk * ukdot * cos(uk);
	Rdot(0, 0) = cos(OMEGAk);
	Rdot(0, 1) = -sin(OMEGAk) * cos(ik);
	Rdot(0, 2) = -xk * sin(OMEGAk) - yk * cos(ik) * cos(OMEGAk);
	Rdot(0, 3) = yk * sin(ik) * sin(OMEGAk);
	Rdot(1, 0) = sin(OMEGAk);
	Rdot(1, 1) = cos(OMEGAk) * cos(ik);
	Rdot(1, 2) = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
	Rdot(1, 3) = -yk * sin(ik) * cos(OMEGAk);
	Rdot(2, 0) = 0;
	Rdot(2, 1) = sin(ik);
	Rdot(2, 2) = 0;
	Rdot(2, 3) = yk * cos(ik);
	DotS(0, 0) = xksdot;
	DotS(1, 0) = yksdot;
	DotS(2, 0) = eph.OMEGADOT - OMEGAeDOT;
	DotS(3, 0) = Ikdot;
	Mat Vel = Rdot * DotS;
	Satpos.vx = Vel(0, 0);
	Satpos.vy = Vel(1, 0);
	Satpos.vz = Vel(2, 0);
	Clock_vel += Ftr_GPS * eph.e * eph.sqrtA * cos(Ek) * Ekdot;//对卫星钟速进行相对论效应改正
	return true;
}


void EarRotCor(Point& Satpos,Point Recpos)//卫星位置和速度自转改正函数
{
	double S, omegae;//卫星与接收机间几何距离，地球自转角速度
	double dx = Satpos.x - Recpos.x;
	double dy = Satpos.y - Recpos.y;
	double dz = Satpos.z - Recpos.z;
	S = sqrt(dx * dx + dy * dy + dz * dz);
	double rotation_angle = OMEGAeDOT_GPS * (S / clight);
	Point sat = Satpos;
	Satpos.x = sat.x * cos(rotation_angle) + sat.y * sin(rotation_angle);
	Satpos.y = -sat.x * sin(rotation_angle) + sat.y * cos(rotation_angle);
	Satpos.z = sat.z;
	Satpos.vx = sat.vx * cos(rotation_angle) + sat.vy * sin(rotation_angle);
	Satpos.vy = -sat.vx * sin(rotation_angle) + sat.vy * cos(rotation_angle);
	Satpos.vz = sat.vz;
}

void BDSEarRotCor(Point& Satpos, Point Recpos)//卫星位置和速度自转改正函数
{
	double S, omegae;//卫星与接收机间几何距离，地球自转角速度
	double dx = Satpos.x - Recpos.x;
	double dy = Satpos.y - Recpos.y;
	double dz = Satpos.z - Recpos.z;
	S = sqrt(dx * dx + dy * dy + dz * dz);
	double rotation_angle = OMEGAeDOT_BDS * (S / clight);
	Point sat = Satpos;
	Satpos.x = sat.x * cos(rotation_angle) + sat.y * sin(rotation_angle);
	Satpos.y = -sat.x * sin(rotation_angle) + sat.y * cos(rotation_angle);
	Satpos.z = sat.z;
	Satpos.vx = sat.vx * cos(rotation_angle) + sat.vy * sin(rotation_angle);
	Satpos.vy = -sat.vx * sin(rotation_angle) + sat.vy * cos(rotation_angle);
	Satpos.vz = sat.vz;
}


double ComputeElevation(const Point & Satpos, const Point & Recpos)
{
	// 计算卫星与接收机的相对位置矢量
	double dx = Satpos.x - Recpos.x;
	double dy = Satpos.y - Recpos.y;
	double dz = Satpos.z - Recpos.z;
	// 计算距离
	double distance = sqrt(dx * dx + dy * dy + dz * dz);
	// 计算接收机位置的地理坐标（纬度、经度、高度）
	double r = sqrt(Recpos.x * Recpos.x + Recpos.y * Recpos.y);
	double lat = atan2(Recpos.z, r); // 纬度
	double lon = atan2(Recpos.y, Recpos.x); // 经度
	// 计算局部东北天坐标系下的卫星位置
	double sinLat = sin(lat);
	double cosLat = cos(lat);
	double sinLon = sin(lon);
	double cosLon = cos(lon);
	// 转换到局部东北天坐标系
	double x_local = -sinLon * dx + cosLon * dy;
	double y_local = -cosLon * sinLat * dx - sinLon * sinLat * dy + cosLat * dz;
	double z_local = cosLon * cosLat * dx + sinLon * cosLat * dy + sinLat * dz;
	// 计算仰角
	double elevation = asin(z_local / distance);
	// 转换为度数
	elevation = elevation * (180.0 / PI);
	return elevation;
}

void CalElevation(vector<Point>& Satpos, Point Recpos,vector<double>&elevation)
{
	elevation.clear();
	for (int i = 0; i < Satpos.size(); i++)
	{
		double elev = ComputeElevation(Satpos[i], Recpos);
		elevation.push_back(elev);
	}
	return;
}

/**
 * @brief 计算卫星方位角 (从北顺时针)
 *
 * @param SatPos 卫星 ECEF 坐标 (m)
 * @param RecPos 接收机 ECEF 坐标 (m)
 * @return double 方位角 (度, 0~360)
 */
double ComputeAzimuth(const Point& Satpos, const Point& Recpos)
{
	// 1. 计算卫星与接收机的相对位置矢量
	double dx = Satpos.x - Recpos.x;
	double dy = Satpos.y - Recpos.y;
	double dz = Satpos.z - Recpos.z;

	// 2. 计算接收机位置的地理坐标（纬度、经度）
	double r = sqrt(Recpos.x * Recpos.x + Recpos.y * Recpos.y);
	double lat = atan2(Recpos.z, r); // 纬度 (弧度)
	double lon = atan2(Recpos.y, Recpos.x); // 经度 (弧度)

	// 3. 计算局部东北天 (ENU) 坐标系下的卫星位置
	double sinLat = sin(lat);
	double cosLat = cos(lat);
	double sinLon = sin(lon);
	double cosLon = cos(lon);

	// ENU 坐标转换矩阵 (从 ECEF 到 ENU)
	// [ E ]   [ -sinLon         cosLon         0      ] [ dx ]
	// [ N ] = [ -cosLon*sinLat  -sinLon*sinLat  cosLat ] [ dy ]
	// [ U ]   [ cosLon*cosLat   sinLon*cosLat   sinLat ] [ dz ]

	double E_local = -sinLon * dx + cosLon * dy;         // 东向分量
	double N_local = -cosLon * sinLat * dx - sinLon * sinLat * dy + cosLat * dz; // 北向分量
	double U_local = cosLon * cosLat * dx + sinLon * cosLat * dy + sinLat * dz;   // 天向分量 (你的 elevation 函数中用的 z_local)

	// 4. 计算方位角
	// 方位角 = atan2(E, N)，即从北(N)顺时针到东(E)
	double azimuth = atan2(E_local, N_local);

	// 5. 转换为度数，并确保在 [0, 360) 范围内
	azimuth = azimuth * (180.0 / PI);

	if (azimuth < 0) 
	{
		azimuth += 360.0;
	}

	return azimuth;
}





void CriticalDiagnosis(vector<SatePosPara> &para)
{
	cout << "=== 关键诊断：卫星位置与几何距离 ===" << endl;

	// 使用一个近似接收机位置（之前计算的结果）
	double approxRX = -2177984.316;
	double approxRY = 4973805.686;
	double approxRZ = 3216250.921;

	for (int n = 0; n < para.size(); n++)
	{
		cout << "历元时间: " << para[n].year << " " << para[n].month << " " << para[n].day << " "
			<< para[n].hour << " " << para[n].min << " " << para[n].sec << endl;
		for (int i = 0; i < para[n].commonPRN.size(); i++) {
			// 计算几何距离
			double dX = para[n].Pos[i].x - approxRX;
			double dY = para[n].Pos[i].y - approxRY;
			double dZ = para[n].Pos[i].z - approxRZ;
			double geometric_range = sqrt(dX * dX + dY * dY + dZ * dZ);

			cout << "\n卫星" << para[n].commonPRN[i] << ":" << endl;
			cout << "  卫星位置: X=" << para[n].Pos[i].x << " Y=" << para[n].Pos[i].y << " Z=" << para[n].Pos[i].z << endl;
			cout << "  接收机位置: X=" << approxRX << " Y=" << approxRY << " Z=" << approxRZ << endl;
			cout << "  几何距离: " << geometric_range << " 米" << endl;
			cout << "  修正后伪距: " << para[n].C1C[i] << " 米" << endl;
			cout << "  差值(伪距-几何距离): " << para[n].C1C[i] - geometric_range << " 米" << endl;

			// 检查卫星位置的合理性
			double earth_radius = sqrt(para[n].Pos[i].x * para[n].Pos[i].x +
				para[n].Pos[i].y * para[n].Pos[i].y +
				para[n].Pos[i].z * para[n].Pos[i].z);
			cout << "  卫星地心距: " << earth_radius << " 米" << endl;

			if (earth_radius < 2.5e7 || earth_radius > 2.7e7) {
				cout << " 卫星位置异常！" << endl;
			}
		}
	}
	
}


bool BDSClock_Cal(BDSEphPara eph, BDSTime BDStime, double& Clock_bias, double& Clock_vel)
{
	double limittime = 3600.0;
	BDSTime ct;
	ct = BDStime;
	double traveltime = (ct.week - eph.BDT_week) * 604800.0 + (ct.sow - eph.Toe);
	if (traveltime > limittime)return false;
	double dt_clock = (ct.week - eph.BDT_week) * 604800.0 + (ct.sow - eph.Toc);
	
	Clock_bias = eph.a0 + eph.a1 * dt_clock + eph.a2 * dt_clock * dt_clock;
	Clock_vel = eph.a1 + 2 * eph.a2 * dt_clock;
	return true;
}


bool BDSSatPVT(string BDSPRN,BDSEphPara eph, BDSTime BDStime, Point& Satpos, double& Clock_bias, double& Clock_vel)
{
	double A, n, tk, Mk, Ek, vk, FAIk;
	double deltauk, deltark, deltaik, uk, rk, ik, xk, yk;
	double OMEGAeDOT = 7.2921150e-5;
	double OMEGAk, rotation_angle;
	double Ekdot, FAIkdot, ukdot, rkdot, Ikdot, xksdot, yksdot;

	//卫星位置矢量，速度旋转参数向量
	Mat Position(3, 1);
	Mat DotS(4, 1);
	//旋转矩阵
	Mat Rdot(3, 4);
	Mat RZdot(3, 3);
	A = eph.sqrtA * eph.sqrtA;
	tk = (BDStime.week - eph.BDT_week) * 604800 + BDStime.sow - eph.Toe;
	n = sqrt(GM_BDS / (A * A * A)) + eph.deltan;
	Mk = eph.M0 + n * tk;
	double Ek0 = Mk;//偏近点角赋初值
	Ek = Mk + eph.e * sin(Ek0);
	for (int i = 0; abs(Ek - Ek0) > 1e-12; i++)//迭代计算偏近点角
	{
		Ek0 = Ek;
		Ek = Mk + eph.e * sin(Ek0);
	}
	vk = atan2(sqrt(1 - eph.e * eph.e) * sin(Ek),
		cos(Ek) - eph.e);
	FAIk = vk + eph.omega;
	deltauk = eph.Cus * sin(2 * FAIk) + eph.Cuc * cos(2 * FAIk);
	deltark = eph.Crs * sin(2 * FAIk) + eph.Crc * cos(2 * FAIk);
	deltaik = eph.Cis * sin(2 * FAIk) + eph.Cic * cos(2 * FAIk);
	uk = FAIk + deltauk;
	rk = A * (1 - eph.e * cos(Ek)) + deltark;
	ik = eph.i0 + deltaik + eph.IDOT * tk;
	xk = rk * cos(uk);
	yk = rk * sin(uk);
	int prn_number = std::stoi(BDSPRN.substr(1));  // 从第1个字符开始提取

	
	if ((prn_number >= 1 && prn_number <= 5) ||
		(prn_number >= 59 && prn_number <= 63))
	{
		OMEGAk = eph.OMEGA0 + eph.OMEGADOT * tk - OMEGAeDOT * eph.Toe;
		Position(0, 0) = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
		Position(1, 0) = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
		Position(2, 0) = yk * sin(ik);
		
		double rotate_z_angle = OMEGAeDOT * tk;
		Mat rotateZ = RotateZ(rotate_z_angle);
		double rotate_x_angle = -5.0 * PI / 180.0;
		Mat rotateX = RotateX(rotate_x_angle);
		Position = rotateZ * rotateX * Position;
		Satpos.x = Position(0, 0);
		Satpos.y = Position(1, 0);
		Satpos.z = Position(2, 0);
	}
	else
	{
		OMEGAk = eph.OMEGA0 + (eph.OMEGADOT - OMEGAeDOT) * tk - OMEGAeDOT * eph.Toe;
		Position(0, 0) = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
		Position(1, 0) = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
		Position(2, 0) = yk * sin(ik);
		Satpos.x = Position(0, 0);
		Satpos.y = Position(1, 0);
		Satpos.z = Position(2, 0);
	}

	

	
	//获取卫星位置
	
	//对卫星钟差相对论效应改正
	Clock_bias += Ftr_BDS * eph.e * eph.sqrtA * sin(Ek);//对卫星钟差进行相对论效应改正
	Ekdot = n / (1 - eph.e * cos(Ek));
	FAIkdot = sqrt(1 - eph.e * eph.e) / (1 - eph.e * cos(Ek)) * Ekdot;
	ukdot = 2 * (eph.Cus * cos(2 * FAIk) - eph.Cuc * sin(2 * FAIk)) * FAIkdot + FAIkdot;
	rkdot = A * eph.e * sin(Ek) * Ekdot + 2 * (eph.Crs * cos(2 * FAIk) - eph.Crc * sin(2 * FAIk)) * FAIkdot;
	Ikdot = eph.IDOT + 2 * (eph.Cis * cos(2 * FAIk) - eph.Cic * sin(2 * FAIk)) * FAIkdot;
	xksdot = rkdot * cos(uk) - rk * ukdot * sin(uk);
	yksdot = rkdot * sin(uk) + rk * ukdot * cos(uk);
	Rdot(0, 0) = cos(OMEGAk);
	Rdot(0, 1) = -sin(OMEGAk) * cos(ik);
	Rdot(0, 2) = -xk * sin(OMEGAk) - yk * cos(ik) * cos(OMEGAk);
	Rdot(0, 3) = yk * sin(ik) * sin(OMEGAk);
	Rdot(1, 0) = sin(OMEGAk);
	Rdot(1, 1) = cos(OMEGAk) * cos(ik);
	Rdot(1, 2) = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
	Rdot(1, 3) = -yk * sin(ik) * cos(OMEGAk);
	Rdot(2, 0) = 0;
	Rdot(2, 1) = sin(ik);
	Rdot(2, 2) = 0;
	Rdot(2, 3) = yk * cos(ik);
	DotS(0, 0) = xksdot;
	DotS(1, 0) = yksdot;
	DotS(2, 0) = eph.OMEGADOT - OMEGAeDOT;
	DotS(3, 0) = Ikdot;
	if ((prn_number >= 1 && prn_number <= 5) ||
		(prn_number >= 59 && prn_number <= 63))
	{
		DotS(2, 0) = eph.OMEGADOT;
		RZdot(0, 0) = -OMEGAeDOT * sin(OMEGAeDOT * tk);
		RZdot(0, 1) = OMEGAeDOT * cos(OMEGAeDOT * tk);
		RZdot(0, 2) = 0;
		RZdot(1, 0) = -OMEGAeDOT * cos(OMEGAeDOT * tk);
		RZdot(1, 1) = -OMEGAeDOT * sin(OMEGAeDOT * tk);
		RZdot(1, 2) = 0;
		RZdot(2, 0) = 0;
		RZdot(2, 1) = 0;
		RZdot(2, 2) = 0;
		double rotate_z_angle = OMEGAeDOT * tk;
		Mat rotateZ = RotateZ(rotate_z_angle);
		double rotate_x_angle = -5.0 * PI / 180.0;
		Mat rotateX = RotateX(rotate_x_angle);
		Mat Vel = rotateZ * rotateX * Rdot * DotS + RZdot * rotateX * Position;
		Satpos.vx = Vel(0, 0);
		Satpos.vy = Vel(1, 0);
		Satpos.vz = Vel(2, 0);

	}
	else
	{
		Mat Vel = Rdot * DotS;
		Satpos.vx = Vel(0, 0);
		Satpos.vy = Vel(1, 0);
		Satpos.vz = Vel(2, 0);
	}
	
	Clock_vel += Ftr_BDS * eph.e * eph.sqrtA * cos(Ek) * Ekdot;//对卫星钟速进行相对论效应改正
	
	return true;
}
