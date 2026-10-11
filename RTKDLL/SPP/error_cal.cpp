#include "newSPP.h"


// 常量定义
const double DEG_TO_RAD = PI / 180.0;
const double RAD_TO_SEMI = 1.0 / PI; // 弧度转半圆
const double SEMI_TO_RAD = PI;       // 半圆转弧度
const double SPEED_OF_LIGHT = 299792458.0; // 光速 (m/s)


// 辅助函数：将角度从半圆(semi - circles)转换为弧度(radians)
inline double semiToRad(double semi) {
    return semi * SEMI_TO_RAD;
}

// 辅助函数：将角度从弧度(radians)转换为半圆(semi-circles)
inline double radToSemi(double rad) {
    return rad * RAD_TO_SEMI;
}

/**
 * @brief 计算Klobuchar电离层延迟 (基于GPS ICD-GPS-200D)
 *
 * @param user_lat_rad    测站纬度 (弧度)
 * @param user_lon_rad    测站经度 (弧度)
 * @param el_rad          卫星高度角 (弧度)
 * @param az_rad          卫星方位角 (弧度)
 * @param alpha           alpha系数数组 [4]
 * @param beta            beta系数数组 [4]
 * @param gps_week_sec    GPS周内秒 (s)
 * @return double         电离层延迟 (米)
 */
double KlobucharModel_GPS(double user_lat_rad, double user_lon_rad,
    double el_rad, double az_rad,
    const double alpha[4], const double beta[4],
    double gps_week_sec) {

    // ---------------- 步骤 1: 计算地球中心角 Psi (半圆) ----------------
    // 公式: psi = 0.0137 / (E + 0.11) - 0.022
    double E_semi = radToSemi(el_rad); // 高度角转换为半圆
    double psi = 0.0137 / (E_semi + 0.11) - 0.022;
    if (psi < 0.0) psi = 0.0; // 防止异常

    // ---------------- 步骤 2: 计算穿刺点(IPP)纬度 Phi_I (半圆) ----------------
    double phi_u_semi = radToSemi(user_lat_rad);
    double A_semi = radToSemi(az_rad);
    double phi_I = phi_u_semi + psi * cos(semiToRad(A_semi)); // 注意 cos 输入需为弧度

    // 限制纬度在 [-0.416, 0.416] 半圆之间
    if (phi_I > 0.416) phi_I = 0.416;
    if (phi_I < -0.416) phi_I = -0.416;

    // ---------------- 步骤 3: 计算穿刺点(IPP)经度 Lambda_I (半圆) ----------------
    double lambda_u_semi = radToSemi(user_lon_rad);
    double lambda_I = lambda_u_semi + (psi * sin(semiToRad(A_semi))) / cos(semiToRad(phi_I));

    // ---------------- 步骤 4: 计算地磁纬度 Phi_m (半圆) ----------------
    // 公式: Phi_m = Phi_I + 0.064 * cos(Lambda_I - 1.617)
    // 注意: Lambda_I 和 1.617 都是半圆单位，但 cos 需要弧度
    double phi_m = phi_I + 0.064 * cos(semiToRad(lambda_I - 1.617));

    // ---------------- 步骤 5: 计算穿刺点地方时 t (秒) ----------------
    // 公式: t = 43200 * Lambda_I + GPS_Time
    double t = 43200.0 * lambda_I + gps_week_sec;
    // 归一化到 [0, 86400)
    if (t >= 86400.0) t -= 86400.0;
    if (t < 0.0) t += 86400.0;

    // ---------------- 步骤 6: 计算幅度 A_I (秒) ----------------
    double A_I = 0.0;
    for (int i = 0; i < 4; ++i) {
        A_I += alpha[i] * pow(phi_m, i);
    }
    if (A_I < 0.0) A_I = 0.0;

    // ---------------- 步骤 7: 计算周期 P_I (秒) ----------------
    double P_I = 0.0;
    for (int i = 0; i < 4; ++i) {
        P_I += beta[i] * pow(phi_m, i);
    }
    if (P_I < 72000.0) P_I = 72000.0;

    // ---------------- 步骤 8: 计算相位 X_I (弧度) ----------------
    double X_I = 2.0 * PI * (t - 50400.0) / P_I;

    // ---------------- 步骤 9: 计算倾斜因子 F (无量纲) ----------------
    // 公式: F = 1.0 + 16.0 * (0.53 - E)^3
    double F = 1.0 + 16.0 * pow(0.53 - E_semi, 3);

    // ---------------- 步骤 10: 计算电离层延迟 (秒) ----------------
    double iono_delay_sec = 0.0;
    double abs_X = fabs(X_I);

    if (abs_X <= 1.57) {
        // 白天情况 (余弦曲线)
        double term = 1.0 - (X_I * X_I) / 2.0 + (X_I * X_I * X_I * X_I) / 24.0;
        // PPT公式是: 5e-9 + A_I * (1 - X^2/2 + X^4/24)
        // 注意：这里的 A_I 就是 amplitude, 5e-9 是夜间常数 DC
        iono_delay_sec = (5.0e-9 + A_I * term) * F;
    }
    else {
        // 夜间情况 (常数)
        iono_delay_sec = 5.0e-9 * F;
    }

    // ---------------- 转换为米 ----------------
    return iono_delay_sec * SPEED_OF_LIGHT;
}






/**
 * @brief 将 ECEF 坐标 (X, Y, Z) 转换为大地坐标 (B, L, H)
 *        
 *
 * @param X ECEF X (m)
 * @param Y ECEF Y (m)
 * @param Z ECEF Z (m)
 * @param ellip 椭球参数结构体 (包含 a 和 e2)
 * @return Geodetic 结构体 (lat: 度, lon: 度, h: 米)
 */
Geodetic XYZ2BLH(double X, double Y, double Z, const EllipsoidParams& ellip)
{
    Geodetic geo = { 0.0, 0.0, 0.0 };

    // 1. 计算 XY 平面距离 S
    double S = std::sqrt(X * X + Y * Y);

    // 2. 初始纬度 B0 (使用球面纬度作为初值)
    double B0 = std::atan2(Z, S);

    // 3. 经度 L (直接计算，与 MATLAB 一致)
    geo.lon = std::atan2(Y, X);

    // 4. 迭代计算纬度 B 和高度 H
    double B = 0.0; // 当前纬度 (弧度)
    double N = 0.0; // 卯酉圈曲率半径

    // 第一次迭代 (在循环外执行，与 MATLAB 逻辑一致)
    N = ellip.a / std::sqrt(1.0 - ellip.e2 * std::sin(B0) * std::sin(B0));
    B = std::atan2(Z + N * ellip.e2 * std::sin(B0), S);

    // 循环迭代，直到收敛 (MATLAB 为 while abs(B-B0) > 1e-12)
    const double TOL = 1e-12;
    int max_iter = 100; // 防止死循环
    int iter = 0;

    while (std::abs(B - B0) > TOL && iter < max_iter) {
        B0 = B; // 更新上一轮的值
        N = ellip.a / std::sqrt(1.0 - ellip.e2 * std::sin(B0) * std::sin(B0));
        B = std::atan2(Z + N * ellip.e2 * std::sin(B0), S);
        iter++;
    }

    // 5. 计算最终高度 H
    // MATLAB: H = S/cos(B) - N;
    // 注意：当 B 接近 +/- 90 度时，cos(B) 接近 0，可能导致数值不稳定。
    // 在实际情况中，北极或南极的 S 会非常小，此时可以改用 Z/sin(B) - N*(1-e2)
    // 这里保持与 MATLAB 一致
    double H = S / std::cos(B) - N;

    // 6. 将纬度和经度转换为度
    const double RAD_TO_DEG = 180.0 / PI;
    geo.lat = B * RAD_TO_DEG;
    geo.lon = geo.lon * RAD_TO_DEG;
    geo.h = H;

    return geo;
}



/**
 * @brief 计算对流层延迟 (Hopfield模型)
 *        
 *
 * @param lat_rad      测站纬度 (弧度)
 * @param h_m          测站高度 (m)
 * @param el_rad       卫星高度角 (弧度)
 * @param T0           参考面温度 (K) (默认海平面: 273.16+15 = 288.16K)
 * @param P0           参考面气压 (mbar) (默认海平面: 1013.25 mbar)
 * @param RH0          参考面相对湿度 (%) (默认海平面: 50%)
 * @param h0_m         参考面高度 (m) (默认海平面: 0m)
 * @return double      对流层延迟 (m)
 */
double HopfieldTroposphere(double lat_rad, double h_m, double el_rad,
    double T0 , double P0 ,
    double RH0 , double h0_m )
{
    // ---------- 1. 计算测站处的气象参数 (PPT 第 10 页) ----------

    // 温度: T = T0 - 0.0065 * (H - H0)
    double T = T0 - 0.0065 * (h_m - h0_m);

    // 气压: p = p0 * (1 - 0.0000226 * (H - H0))^5.225
    double P = P0 * pow(1.0 - 0.0000226 * (h_m - h0_m), 5.225);

    // 相对湿度: RH = RH0 * exp(-0.0006396 * (H - H0))
    double RH = RH0 * exp(-0.0006396 * (h_m - h0_m));

    // 水汽压: e = RH * exp(-37.2465 + 0.213166*T - 0.000256908*T^2)
    double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);

    // ---------- 2. 计算干顶距 Kd 和 湿顶距 Kv ----------
    // 干对流层顶高: hd = 40136 + 148.72 * (T0 - 273.16)
    double hd = 40136.0 + 148.72 * (T0 - 273.16);

    // 湿对流层顶高: hv = 11000 m
    double hv = 11000.0;

    // Kd = 155.2e-7 * (P/T) * (hd - H)
    double Kd = 155.2e-7 * (P / T) * (hd - h_m);

    // Kv = 155.2e-7 * (4810/T^2) * e * (hv - H)
    double Kv = 155.2e-7 * (4810.0 / (T * T)) * e * (hv - h_m);

    // ---------- 3. 计算倾斜因子 ----------
    double E_deg = el_rad * 180.0 / PI; // 高度角 (度)
    double sinE = sin(el_rad);

    // 干延迟倾斜因子: sin(sqrt(E^2 + 6.25))
    // 注意：PPT公式中 E 单位为度，需先转换为度再计算平方
    double factor_dry = 1.0 / sqrt(sinE * sinE + 0.0019); // sin^2(E) + 6.25度的平方
    // 实际上：sin(sqrt(E^2 + 6.25)) 中 E 是度
    // 常用简化写法：sqrt(sin^2(E) + 0.0019) 对应弧度
    // 但为严格遵循PPT，需做如下转换：
    double E_sq_deg = E_deg * E_deg;
    double sin_sqrt = sin(sqrt(E_sq_deg + 6.25) * PI / 180.0);
    double factor_dry_ppt = 1.0 / sin_sqrt;

    // 湿延迟倾斜因子: sin(sqrt(E^2 + 2.25))
    double sin_sqrt_wet = sin(sqrt(E_sq_deg + 2.25) * PI / 180.0);
    double factor_wet_ppt = 1.0 / sin_sqrt_wet;

    // ---------- 4. 计算总对流层延迟 ----------
    // Delta_Trop = Kd / sin(sqrt(E^2+6.25)) + Kv / sin(sqrt(E^2+2.25))
    double delay_dry = Kd * factor_dry_ppt;
    double delay_wet = Kv * factor_wet_ppt;
    double tropo_delay = delay_dry + delay_wet;

    return tropo_delay;
}

/**
 * @brief 简化版 Hopfield 模型 (使用标准气象参数)
 *        适合没有实测气象数据的情况
 *
 * @param lat_rad      测站纬度 (弧度)
 * @param h_m          测站高度 (m)
 * @param el_rad       卫星高度角 (弧度)
 * @return double      对流层延迟 (m)
 */
double HopfieldSimple(double lat_rad, double h_m, double el_rad)
{
    // 使用标准气象元素 (PPT 第 11 页)
    double T0 = 288.16;    // 15°C + 273.16
    double P0 = 1013.25;   // 标准大气压
    double RH0 = 50.0;     // 相对湿度 50%
    double h0 = 0.0;       // 海平面

    return HopfieldTroposphere(lat_rad, h_m, el_rad, T0, P0, RH0, h0);
}
