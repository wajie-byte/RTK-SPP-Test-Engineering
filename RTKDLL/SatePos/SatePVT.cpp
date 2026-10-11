#include"SatePVT.h"
#include"../MATDLL/Mat.h"
#include"../BASEDLL/Base.h"
#include"SatePos.h"



bool BDSClock_Cal(BDSEPHEMERIS eph, BDSTime BDStime, double& Clock_bias, double& Clock_vel)
{
    double limittime = 3600.0;
    BDSTime ct;
    ct = BDStime;
    double traveltime = (ct.week - eph.Week) * 604800.0 + (ct.sow - eph.toe);
    if (traveltime > limittime)return false;
    double dt_clock = (ct.week - eph.Week) * 604800.0 + (ct.sow - eph.toc);

    Clock_bias = eph.a0 + eph.a1 * dt_clock + eph.a2 * dt_clock * dt_clock;
    Clock_vel = eph.a1 + 2 * eph.a2 * dt_clock;
    return true;
}

bool GPSClock_Cal(GPSEPHEM eph, GPSTime GPStime, double& Clock_bias, double& Clock_vel)
{
	double limittime = 7200.0;
	GPSTime ct;
	ct = GPStime;
	double traveltime = (ct.week - eph.week) * 604800.0 + (ct.sow - eph.toe);
	if (traveltime > limittime)return false;
	double dt_clock = (ct.week - eph.week) * 604800.0 + (ct.sow - eph.toc);

	Clock_bias = eph.af0 + eph.af1 * dt_clock + eph.af2 * dt_clock * dt_clock;
	Clock_vel = eph.af1 + 2 * eph.af2 * dt_clock;
	return true;
}


bool BDSSatPVT(BDSEPHEMERIS eph, BDSTime BDStime, Point& Satpos, double& Clock_bias, double& Clock_vel)
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
	A = eph.RootA * eph.RootA;
	tk = (BDStime.week - eph.Week) * 604800 + BDStime.sow - eph.toe;
	n = sqrt(GM_BDS / (A * A * A)) + eph.deltaN;
	Mk = eph.M0 + n * tk;
	double Ek0 = Mk;//偏近点角赋初值
	Ek = Mk + eph.ecc * sin(Ek0);
	for (int i = 0; abs(Ek - Ek0) > 1e-12; i++)//迭代计算偏近点角
	{
		Ek0 = Ek;
		Ek = Mk + eph.ecc * sin(Ek0);
	}
	vk = atan2(sqrt(1 - eph.ecc * eph.ecc) * sin(Ek),
		cos(Ek) - eph.ecc);
	FAIk = vk + eph.omega;
	deltauk = eph.Cus * sin(2 * FAIk) + eph.Cuc * cos(2 * FAIk);
	deltark = eph.Crs * sin(2 * FAIk) + eph.Crc * cos(2 * FAIk);
	deltaik = eph.Cis * sin(2 * FAIk) + eph.Cic * cos(2 * FAIk);
	uk = FAIk + deltauk;
	rk = A * (1 - eph.ecc * cos(Ek)) + deltark;
	ik = eph.i0 + deltaik + eph.IDOT * tk;
	xk = rk * cos(uk);
	yk = rk * sin(uk);
	


	if ((eph.ID >= 1 && eph.ID <= 5) ||
		(eph.ID >= 59 && eph.ID <= 63))
	{
		OMEGAk = eph.Omega0 + eph.OmegaDot * tk - OMEGAeDOT * eph.toe;
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
		OMEGAk = eph.Omega0 + (eph.OmegaDot - OMEGAeDOT) * tk - OMEGAeDOT * eph.toe;
		Position(0, 0) = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
		Position(1, 0) = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
		Position(2, 0) = yk * sin(ik);
		Satpos.x = Position(0, 0);
		Satpos.y = Position(1, 0);
		Satpos.z = Position(2, 0);
	}




	//获取卫星位置

	//对卫星钟差相对论效应改正
	Clock_bias += Ftr_BDS * eph.ecc * eph.RootA * sin(Ek);//对卫星钟差进行相对论效应改正


	Ekdot = n / (1 - eph.ecc * cos(Ek));
	FAIkdot = sqrt(1 - eph.ecc * eph.ecc) / (1 - eph.ecc * cos(Ek)) * Ekdot;
	ukdot = 2 * (eph.Cus * cos(2 * FAIk) - eph.Cuc * sin(2 * FAIk)) * FAIkdot + FAIkdot;
	rkdot = A * eph.ecc * sin(Ek) * Ekdot + 2 * (eph.Crs * cos(2 * FAIk) - eph.Crc * sin(2 * FAIk)) * FAIkdot;
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
	DotS(2, 0) = eph.OmegaDot - OMEGAeDOT;
	DotS(3, 0) = Ikdot;
	if ((eph.ID >= 1 && eph.ID <= 5) ||
		(eph.ID >= 59 && eph.ID <= 63))
	{
		DotS(2, 0) = eph.OmegaDot;
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

	Clock_vel += Ftr_BDS * eph.ecc * eph.RootA * cos(Ek) * Ekdot;//对卫星钟速进行相对论效应改正

	return true;
}


bool GPSSatPVT(GPSEPHEM eph, GPSTime GPStime, Point& Satpos, double& Clock_bias, double& Clock_vel)
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
	A = eph.A;
	tk = (GPStime.week - eph.week) * 604800 + GPStime.sow - eph.toe;
	n = sqrt(GM_GPS / (A * A * A)) + eph.deltaN;
	Mk = eph.M0 + n * tk;
	double Ek0 = Mk;//偏近点角赋初值
	Ek = Mk + eph.ecc * sin(Ek0);
	for (int i = 0; abs(Ek - Ek0) > 1e-12; i++)//迭代计算偏近点角
	{
		Ek0 = Ek;
		Ek = Mk + eph.ecc * sin(Ek0);
	}
	vk = atan2(sqrt(1 - eph.ecc * eph.ecc) * sin(Ek),
		cos(Ek) - eph.ecc);
	FAIk = vk + eph.omega;
	deltauk = eph.cus * sin(2 * FAIk) + eph.cuc * cos(2 * FAIk);
	deltark = eph.crs * sin(2 * FAIk) + eph.crc * cos(2 * FAIk);
	deltaik = eph.cis * sin(2 * FAIk) + eph.cic * cos(2 * FAIk);
	uk = FAIk + deltauk;
	rk = A * (1 - eph.ecc * cos(Ek)) + deltark;
	ik = eph.I0 + deltaik + eph.IDot * tk;
	xk = rk * cos(uk);
	yk = rk * sin(uk);
	OMEGAk = eph.omega0 + (eph.omegaDot - OMEGAeDOT) * tk - OMEGAeDOT * eph.toe;
	Position(0, 0) = xk * cos(OMEGAk) - yk * cos(ik) * sin(OMEGAk);
	Position(1, 0) = xk * sin(OMEGAk) + yk * cos(ik) * cos(OMEGAk);
	Position(2, 0) = yk * sin(ik);
	//获取卫星位置
	Satpos.x = Position(0, 0);
	Satpos.y = Position(1, 0);
	Satpos.z = Position(2, 0);
	//对卫星钟差相对论效应改正
	Clock_bias += Ftr_GPS * eph.ecc * sqrt(eph.A) * sin(Ek);//对卫星钟差进行相对论效应改正
	Ekdot = n / (1 - eph.ecc * cos(Ek));
	FAIkdot = sqrt(1 - eph.ecc * eph.ecc) / (1 - eph.ecc * cos(Ek)) * Ekdot;
	ukdot = 2 * (eph.cus * cos(2 * FAIk) - eph.cuc * sin(2 * FAIk)) * FAIkdot + FAIkdot;
	rkdot = A * eph.ecc * sin(Ek) * Ekdot + 2 * (eph.crs * cos(2 * FAIk) - eph.crc * sin(2 * FAIk)) * FAIkdot;
	Ikdot = eph.IDot + 2 * (eph.cis * cos(2 * FAIk) - eph.cic * sin(2 * FAIk)) * FAIkdot;
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
	DotS(2, 0) = eph.omegaDot - OMEGAeDOT;
	DotS(3, 0) = Ikdot;
	Mat Vel = Rdot * DotS;
	Satpos.vx = Vel(0, 0);
	Satpos.vy = Vel(1, 0);
	Satpos.vz = Vel(2, 0);
	Clock_vel += Ftr_GPS * eph.ecc * sqrt(eph.A) * cos(Ek) * Ekdot;//对卫星钟速进行相对论效应改正
	return true;
}

void SatPVT(OBSS obss,vector<SATE>& sates,Point Recpos)
{
    int n_sats = sates.size();

	for (int i = 0; i < n_sats; i++)
	{
		//BDS系统
		if (sates[i].sys == BDS_SYS)
		{
			BDSTime ttr; // 卫星信号发射时刻
			ttr.week = sates[i].BDStime.week;
			ttr.sow = sates[i].BDStime.sow - obss.obs[i].psr / clight;

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
				BDSClock_Cal(sates[i].BDSEPH, ttr, Clock_bias, Clock_vel);
				BDSSatPVT(sates[i].BDSEPH, ttr, Satpos, Clock_bias, Clock_vel);
				switch (obss.obs[i].signal_type)
				{
				case BDS_B1I_D1:
				{
					Clock_bias -= sates[i].BDSEPH.tgd1;
					break;
				}

				case BDS_B3I_D1:
				{
					break;
				}
				default:
					break;
				}
				// 修正发射时间：考虑卫星钟差的影响
				ttr.sow = sates[i].BDStime.sow - obss.obs[i].psr / clight - Clock_bias;

				// 周内秒调整
				if (ttr.sow < 0)
				{
					ttr.week--;
					ttr.sow += 604800;
				}
			}

			// 最终计算
			BDSClock_Cal(sates[i].BDSEPH, ttr, Clock_bias, Clock_vel);
			BDSSatPVT(sates[i].BDSEPH, ttr, Satpos, Clock_bias, Clock_vel);
			BDSEarRotCor(Satpos, Recpos);
			
			/*
			
			*/
			


			sates[i].satepos = Satpos;
			sates[i].Clock_bias = Clock_bias;
			sates[i].Clock_vel = Clock_vel;
		}

		else if (sates[i].sys == GPS_SYS)
		{
			GPSTime ttr; // 卫星信号发射时刻
			ttr.week = sates[i].GPStime.week;
			ttr.sow = sates[i].GPStime.sow - obss.obs[i].psr / clight;
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
				GPSClock_Cal(sates[i].GPSEPH, ttr, Clock_bias, Clock_vel);
				GPSSatPVT(sates[i].GPSEPH, ttr, Satpos, Clock_bias, Clock_vel);
				switch (obss.obs[i].signal_type)
				{
				case GPS_L1C:
				{
					Clock_bias -= sates[i].GPSEPH.tgd;
					break;
				}

				case GPS_L2P_Y:
				{
					break;
				}
				default:
					break;
				}
				// 修正发射时间：考虑卫星钟差的影响
				ttr.sow = sates[i].GPStime.sow - obss.obs[i].psr / clight - Clock_bias;

				// 周内秒调整
				if (ttr.sow < 0)
				{
					ttr.week--;
					ttr.sow += 604800;
				}
			}
			// 最终计算
			GPSClock_Cal(sates[i].GPSEPH, ttr, Clock_bias, Clock_vel);
			GPSSatPVT(sates[i].GPSEPH, ttr, Satpos, Clock_bias, Clock_vel);
			EarRotCor(Satpos, Recpos);

			/*

			*/



			sates[i].satepos = Satpos;
			sates[i].Clock_bias = Clock_bias;
			sates[i].Clock_vel = Clock_vel;
		}
	}
	
	
       


}

