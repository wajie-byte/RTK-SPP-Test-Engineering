"""SPP 单点定位算法模块

Python 等价实现，对应 C++ 项目中的 SPP/newSPP.h
"""
import math
import numpy as np
from src.matrix import Matrix

# 常量
CLIGHT = 299792458.0
PI = 3.141592653589793
BDS_B1_FREQ = 1561.098e6
BDS_B3_FREQ = 1268.52e6
DEG_TO_RAD = PI / 180.0


class LSResult:
    """最小二乘定位结果（对应 C++ 的 LSResult 结构体）"""

    def __init__(self, **kwargs):
        self.converged = kwargs.get('converged', False)
        self.position = kwargs.get('position', (0.0, 0.0, 0.0))
        self.clock_bias = kwargs.get('clock_bias', 0.0)
        self.gdop = kwargs.get('gdop', 0.0)
        self.pdop = kwargs.get('pdop', 0.0)
        self.iterations = kwargs.get('iterations', 0)
        self.rms = kwargs.get('rms', 0.0)
        self.n_satellites = kwargs.get('n_satellites', 0)
        self.error = kwargs.get('error', '')

    def __repr__(self):
        if not self.converged:
            return f"LSResult(not converged, error={self.error})"
        return (f"LSResult(x={self.position[0]:.4f}, y={self.position[1]:.4f}, "
                f"z={self.position[2]:.4f}, rms={self.rms:.3f}m, "
                f"gdop={self.gdop:.2f}, sats={self.n_satellites})")


def compute_elevation(sat_pos, rec_pos):
    """
    计算卫星高度角（度）
    sat_pos: (x, y, z) ECEF
    rec_pos: (x, y, z) ECEF
    """
    dx = sat_pos[0] - rec_pos[0]
    dy = sat_pos[1] - rec_pos[1]
    dz = sat_pos[2] - rec_pos[2]
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    # 卫星到接收机的单位向量
    e = (dx / distance, dy / distance, dz / distance)

    # 将 ECEF 近似转到当地水平面
    lat = math.atan2(rec_pos[2], math.sqrt(rec_pos[0]**2 + rec_pos[1]**2))
    lon = math.atan2(rec_pos[1], rec_pos[0])

    # ENU 单位向量：北方向
    n = (-math.sin(lat) * math.cos(lon),
         -math.sin(lat) * math.sin(lon),
         math.cos(lat))
    # 上方向
    u = (math.cos(lat) * math.cos(lon),
         math.cos(lat) * math.sin(lon),
         math.sin(lat))

    elev_enu = (e[0] * u[0] + e[1] * u[1] + e[2] * u[2])
    elev_enu = max(-1.0, min(1.0, elev_enu))
    return math.asin(elev_enu) / DEG_TO_RAD


def generate_mock_satellites(rec_pos, num_sats=6):
    """生成模拟卫星数据用于测试 SPP 算法

    卫星围绕接收机分布，伪距 = 几何距离 + 钟差 + 小噪声
    接收机参考位置为 rec_pos
    """
    # 模拟 GEO/IGSO/MEO 轨道
    orbit_radius = 42164000.0  # GEO 轨道高度约 42164 km
    earth_radius = 6371000.0

    satellites = []
    for i in range(num_sats):
        # 均匀分布在天顶方向附近
        az = (i * 360.0 / num_sats + 10) * DEG_TO_RAD
        elev = (30 + i * 8) * DEG_TO_RAD  # 30°-70° 高度角

        # 卫星位置（简化：接收机上方球面）
        sat_dist = orbit_radius
        sx = rec_pos[0] + sat_dist * math.cos(elev) * math.sin(az)
        sy = rec_pos[1] + sat_dist * math.cos(elev) * math.cos(az)
        sz = rec_pos[2] + sat_dist * math.sin(elev)

        # 几何距离
        dx = sx - rec_pos[0]
        dy = sy - rec_pos[1]
        dz = sz - rec_pos[2]
        geom_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        # 伪距 = 几何距离 + 100m 钟差等效 + 小噪声
        clock_offset = 100.0  # 钟差（米）
        noise = (i - num_sats / 2) * 0.5  # 小噪声
        pseudorange = geom_dist + clock_offset + noise

        satellites.append({
            'pos': (sx, sy, sz),
            'pseudorange': pseudorange,
            'elevation': elev / DEG_TO_RAD,
            'clock_bias': 0.0,
        })

    return satellites


def least_squares_spp(satellites, init_pos=None, max_iter=20, tol=1e-6):
    """
    最小二乘单点定位

    参数:
        satellites: 卫星数据列表，每个包含 pos, pseudorange, clock_bias
        init_pos: 初始位置 (x, y, z)，默认用地球中心
        max_iter: 最大迭代次数
        tol: 收敛阈值

    返回:
        LSResult
    """
    if len(satellites) < 4:
        return LSResult(converged=False, error="卫星数量不足，至少需要4颗")

    if init_pos is None:
        rec_pos = [0.0, 0.0, 0.0]
    else:
        rec_pos = list(init_pos)

    est_dt = 0.0
    n_sats = len(satellites)

    for iteration in range(max_iter):
        B = np.zeros((n_sats, 4))
        L = np.zeros((n_sats, 1))
        W = np.zeros((n_sats, n_sats))

        for i, sat in enumerate(satellites):
            dx = sat['pos'][0] - rec_pos[0]
            dy = sat['pos'][1] - rec_pos[1]
            dz = sat['pos'][2] - rec_pos[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            # 设计矩阵
            B[i, 0] = -dx / dist
            B[i, 1] = -dy / dist
            B[i, 2] = -dz / dist
            B[i, 3] = 1.0

            # 观测残差
            L[i, 0] = sat['pseudorange'] - dist - est_dt + sat['clock_bias'] * CLIGHT

            # 高度角定权
            elev_rad = sat['elevation'] * DEG_TO_RAD
            var = 1.0 / max(math.sin(elev_rad)**2, 0.01)
            W[i, i] = 1.0 / var

        # 法方程 N = B^T W B, U = B^T W L
        N = B.T @ W @ B
        U = B.T @ W @ L

        # 检查奇异
        det = np.linalg.det(N)
        if abs(det) < 1e-15:
            return LSResult(converged=False, error="法方程奇异")

        dx = np.linalg.solve(N, U)

        # 收敛判断
        if all(abs(dx[i, 0]) < tol for i in range(4)):
            # 收敛，计算精度指标
            rec_pos[0] += dx[0, 0]
            rec_pos[1] += dx[1, 0]
            rec_pos[2] += dx[2, 0]
            est_dt += dx[3, 0]

            # 后验残差
            V = np.zeros((n_sats, 1))
            for i, sat in enumerate(satellites):
                dx_s = sat['pos'][0] - rec_pos[0]
                dy_s = sat['pos'][1] - rec_pos[1]
                dz_s = sat['pos'][2] - rec_pos[2]
                dist = math.sqrt(dx_s**2 + dy_s**2 + dz_s**2)
                V[i, 0] = sat['pseudorange'] - dist - est_dt

            sigma2 = float((V.T @ W @ V)[0, 0]) / (n_sats - 4)
            Q = np.linalg.inv(N)

            gdop = math.sqrt(float(Q[0, 0] + Q[1, 1] + Q[2, 2] + Q[3, 3]))
            pdop = math.sqrt(float(Q[0, 0] + Q[1, 1] + Q[2, 2]))

            return LSResult(
                converged=True,
                position=(rec_pos[0], rec_pos[1], rec_pos[2]),
                clock_bias=est_dt,
                gdop=gdop, pdop=pdop,
                iterations=iteration + 1,
                rms=math.sqrt(max(sigma2, 0)),
                n_satellites=n_sats,
            )

        # 更新位置和钟差
        rec_pos[0] += dx[0, 0]
        rec_pos[1] += dx[1, 0]
        rec_pos[2] += dx[2, 0]
        est_dt += dx[3, 0]

    return LSResult(converged=False, error=f"未在 {max_iter} 次迭代内收敛")