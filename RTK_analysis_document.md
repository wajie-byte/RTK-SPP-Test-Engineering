# GNSS RTK 定位引擎 -- 面试备战级项目分析文档

---

## 一、项目一句话概述

这是一个自己实现的双系统（GPS/BDS）双频 RTK 定位引擎，包含从二进制协议解析、卫星位置计算、SPP 单点定位、LAMBDA 模糊度固定到基线解算的完整 GNSS 处理管线。支持实时（TCP 流式接收 OEM7 协议）和后处理（二进制/文本文件）两种模式。零基线内符合精度 GPS < 0.5mm、BDS < 2mm；实测 ~1590m 基线全部历元成功固定（Ratio 20~144，4 小时连续跟踪 13-14 颗 BDS 卫星）。

---

## 二、目录与模块架构

### 2.1 目录树与各模块职责

```
RTKDLL/                          ← 源代码根目录
├── main.cpp                     ← 主入口 / 测试驱动（配置路径、启动模式）
├── BASEDLL/ (Base.h/cpp)        ← 时间系统转换+坐标变换（WGS84, Gregorian↔GPST/BDT, XYZ→BLH→ENU）
├── Config/ (config.h/cpp)       ← 全局配置（SolveType, SatSys, FreqType, IonoHandle, ParamConfig）
├── FileRead/ (RTKDataRead.h/cpp)← RINEX 文本格式 I/O（文件头跳过、D值修复、结果格式转换）
├── LAMBDA/ (lambda.h/cpp)       ← 整数最小二乘模糊度搜索（自研 LAMBDA 实现）
├── MATDLL/ (Mat.h/cpp)          ← 自研矩阵运算库（逆、行列式、转置、乘法、旋转矩阵、Gauss-Jordan）
├── OEMRead/                     ← NovAtel OEM7 二进制协议模块
│   ├── Parse.h/cpp              ←   消息体解析（BDSEPHEMERIS, GPSEPHEM, Range, IONUTC）+ 数据匹配
│   └── BinRead.h/cpp            ←   文件 I/O（Sync 检测, CRC32, Decode, Single_POS, RTK_FILE_OEM7）
├── RealTime/ (RealTime.h/cpp)   ← TCP 实时采集+处理（select 多路复用、Buffer 管理、基-流动站同步）
├── RTKCAL/                      ← RTK 解算引擎
│   ├── newRTK.h/cpp             ←   双频 RTK（B1I+B3I DD, 迭代 LSQ, LAMBDA 固定, Ratio 检验）
│   └── RTKSoution.h/Solution.cpp←   单频 BDS RTK（RINEX 数据驱动, 非迭代 LSQ, 兼容旧格式）
├── RTKData/                     ← 全局数据结构
│   ├── DefData.h/cpp            ←   GPSTime, SPPResult, EphPara, BDSEphPara, BDS_DDObs 等结构体
│   └── DataProcess.h/cpp        ←   时间转换, 星历建图, 观测值-星历匹配
├── SatePos/                     ← 卫星位置/速度/钟差
│   ├── SatePos.h/cpp            ←   RINEX 星历版本 (EphPara/BDSEphPara)
│   └── SatePVT.h/cpp            ←   OEM7 二进制星历版本 (GPSEPHEM/BDSEPHEMERIS) + SatPVT 主控
└── SPP/                         ← 单点定位
    ├── newSPP.h/cpp             ←   OEM7 版本 SPP（多系统、双频、DOP 计算）
    ├── SPP.h/cpp                ←   RINEX 版本 SPP（单系统、legacy）
    └── error_cal.cpp            ←   Klobuchar 电离层 + Hopfield 对流层 + XYZ2BLH
```

### 2.2 数据流图

```
═══════════════════════════════════════════════════════════════════════════
                        GNSS RTK 整体数据流
═══════════════════════════════════════════════════════════════════════════

  [TCP Socket / 二进制文件]                           [配置文件]
          │                                              │
          ▼                                              ▼
  ┌─────────────────┐                          ┌──────────────────┐
  │ OEMRead/        │  Sync 0xAA 0x44 0x12     │ Config/config.h  │
  │ BinRead.cpp     │→ CRC32 校验               │  System, Freq,   │
  │ Parse.cpp       │→ 消息分发 (by ID)          │  IonoHandle,     │
  │                 │  ├─ 1696: BDS Ephemeris   │  ElevMin=15,     │
  │                 │  ├─    7: GPS Ephemeris   │  CN0Min=35,      │
  └──────┬──────────┘  ├─   43: Range Obs       │  RatioMin=3.0    │
         │             └─    8: ION/UTC         └──────────────────┘
         │
    ┌────┴────────────┐
    ▼                 ▼
┌──────────┐    ┌─────────────┐
│ SatePos/ │    │ SPP/        │
│SatPVT.cpp│    │ newSPP.cpp  │
│          │    │             │
│卫星 ECEF │    │迭代 LSQ     │
│位置+速度 │    │航向向量 =   │
│钟差+REL  │    │[dX/r,dY/r,  │
│Earth Rot │    │ dZ/r, 1.0]  │
│Elev+Az   │    │Iono+Tropo   │
│BDS GEO─Rz│    │  GPS only   │
│        Rx│    │收敛<1e-6m   │
└────┬─────┘    └──────┬──────┘
     │                 │
     │  SATE struct    │  SPPResult
     │  (pos+clock)    │  (reciver_xyz+std)
     │                 │
     └────────┬────────┘
              ▼
     ┌─────────────────────────────────┐
     │        RTKCAL/newRTK.cpp        │
     │                                 │
     │ DoubleDifMatch (时间同步 1ms)    │
     │   ├─ Match_RoverObs_BaseLS      │
     │   ├─ Match_RoverObs_BaseObs     │
     │   ├─ Match_BaseObs_BaseLS       │
     │   └─ Match_Process_DD (B1I+B3I) │
     │                                 │
     │ BuildDDMat                       │
     │   ├─ 参考星选择 (最高高度角)     │
     │   ├─ DD 观测方程 (4维/非参考星)  │
     │   ├─ A(4n×3+2n), L(4n×1)        │
     │   └─ BuildP_Matrix (高度角加权)   │
     │                                 │
     │ Solve_RTK (迭代 5 次→收敛 1e-4) │
     │   ├─ N=A'PA, U=A'PL, X=N⁻¹U     │
     │   ├─ Sigma0², Q_xx, D_xx        │
     │   ├─ lambda(n, 2, â, Q_aa, F, s)│
     │   ├─ Ratio = s[1]/s[0] > 2.0    │
     │   └─ 固定解位置更新              │
     │       b̂_fix = b̂_float - Q_ab·Q_bb⁻¹·(â-ǎ)│
     │                                 │
     └──────────────┬──────────────────┘
                    ▼
     ┌─────────────────────────────────┐
     │          输出与精度评估          │
     │                                 │
     │ RTKResult → txt 文件 (每历元4行) │
     │ 状态/Ratio/dE,dN,dU/卫星/DD残差  │
     │                                 │
     │ 图表 (PNG)                      │
     │ baseline_components, errors,    │
     │ enu_errors, satellites&status,  │
     │ ratio_analysis                  │
     └─────────────────────────────────┘
```

### 2.3 关键调用链

```
main.cpp
  └─ GNSSConfig::InitConfig()  [config.cpp:3]
      ├─ [文件模式] RTK_FILE_OEM7()  [BinRead.cpp:550]
      │     ├─ 读取基站文件 → Single_POS() → base_obss_vec
      │     ├─ 读取流动站文件 → Single_POS() → rover_obss_vec
      │     └─ ProcessRTK() → Solve_RTK() [newRTK.cpp:431]
      │
      └─ [实时模式] RTK_RealTime()  [RealTime.cpp:374]
            ├─ TCP Socket ×2 (基站/流动站)
            ├─ select() 多路复用
            ├─ ProcessDataAndSPP() → Single_POS()
            └─ ProcessRTK() → Solve_RTK()
```

---

## 三、核心算法实现原理

### 3.1 数据读取与观测值组织 (OEMRead/)

**输入**: 二进制 OEM7 数据流（文件或 TCP 网络流）
**输出**: 按历元组织的 OBS（观测值）+ SATE（卫星位置/钟）+ Ephemeris Map

**协议解析流程** (`BinRead.cpp:9-132`):
1. 同步字节检测：3 字节 FSM `{0xAA, 0x44, 0x12}`，遇不匹配字节即重置（`BinRead.cpp:17-30`）
2. BinHeader（28 字节 `#pragma pack(1)`）包含 message_id、message_length、GPS week/seconds
3. CRC32 校验（多项式 `0xEDB88320`），计算 header+body（`BinRead.cpp:165-177`）
4. 消息分发（`Decode`，`BinRead.cpp:403-444`）：

| Message ID | 类型 | 解析函数 |
|-----------|------|---------|
| 1696 | BDS Ephemeris | `parseBDSEPHEMERIS` → `DecodeBDSEphEMERIS` |
| 7 | GPS Ephemeris | `parseGPSEPHEM` → `DecodeGPSEphEM` |
| 43 | Range Obs | `parseRANGE` → `DecodeObservations` |
| 8 | ION/UTC | `parseIONUTC` → `DecodeIONUTC` |

5. `parseRANGE` 逐卫星遍历 `Range` 记录（每卫星 44 字节 packed），调用 `decode_ch_tr_status` 从 `ch_tr_status` 提取 system（bits 16-18）和 signal_type（bits 21-25）。（`Parse.cpp:61-104`）

6. `decodeRANGE` 将活跃度 `adr = -range.adr` 取反（NovAtel 采用符号计数累积多普勒，`Parse.cpp:144`）

**数据匹配** (`Parse.cpp:295-502`):
- `Match_BDS_OBS_AND_EPH`: 过滤 `system==BDS_SYS && (B1I 或 B3I) && C_NO>35`，时间差 ≤ 3600s（`MAX_TIME_DIFF`）
- `Match_GPS_OBS_AND_EPH`: 过滤 `system==GPS_SYS && (L1C 或 L2P_Y) && C_NO>35`，时间差 ≤ 7200s

### 3.2 卫星位置计算与钟差改正 (SatePos/)

**输入**: 广播星历（RINEX 或 OEM7）、观测时刻（GPSTime/BDSTime）
**输出**: 卫星 ECEF 位置、速度、钟差、钟速、高度角、方位角

**GPS 卫星 PVT** (`SatePVT.cpp:173-247`, 使用 GPSEPHEM 结构):

Kepler 方程求解（固定点迭代，容差 1e-12 弧度，无迭代上限）:
```
n₀ = sqrt(GM_GPS / A³)               GM_GPS = 3.986005e14
n = n₀ + Δn
Mₖ = M₀ + n·tₖ
Eₖ^(i+1) = Mₖ + e·sin(Eₖ^(i))       初值 Eₖ^(0) = Mₖ + e·sin(Mₖ)
```

真近点角:
```
νₖ = atan2(√(1-e²)·sin(Eₖ), cos(Eₖ)-e)
```

扰动改正:
```
δuₖ = C_us·sin(2Φₖ) + C_uc·cos(2Φₖ)
δrₖ = C_rs·sin(2Φₖ) + C_rc·cos(2Φₖ)
δiₖ = C_is·sin(2Φₖ) + C_ic·cos(2Φₖ)
```

ECEF 坐标转换:
```
Ωₖ = Ω₀ + (Ω̇ - ωₑ)·tₖ - ωₑ·tₒₑ     (GPS/IGSO/MEO)
x = rₖ·cos(uₖ)·cos(Ωₖ) - rₖ·sin(uₖ)·cos(iₖ)·sin(Ωₖ)
y = rₖ·cos(uₖ)·sin(Ωₖ) + rₖ·sin(uₖ)·cos(iₖ)·cos(Ωₖ)
z = rₖ·sin(uₖ)·sin(iₖ)
```

**BDS GEO 特殊处理** (`SatePos.cpp:431-460`):
识别依据：PRN 1-5 或 59-63。
```
Ωₖ_GEO = Ω₀ + Ω̇·tₖ - ωₑ·tₒₑ       (无 ωₑ·tₖ 项)
再经 Rz(ωₑ·tₖ)·Rx(-5°) → CGCS2000
```

**钟差**:
多项式: `dt_sv = a₀ + a₁·(t-tₒc) + a₂·(t-tₒc)²`
相对论: `dt_r = F·e·√A·sin(Eₖ)`，其中 `F = -2√(GM)/c²`
GPS: `Ftr_GPS = -4.442807633e-10`，BDS: `Ftr_BDS = -4.442807309e-10`
钟速: `Δṫ = a₁ + 2·a₂·(t-tₒc) + F·e·√A·cos(Eₖ)·Ėₖ`

**地球自转改正** (`EarRotCor`, `SatePos.cpp:211-226`):
τ = |r_sat - r_rcv| / c，旋转角 = ωₑ·τ，对卫星位置进行 Rz 旋转。

**高度角/方位角** (`ComputeElevation`, `SatePos.cpp:246-332`):
通过 ECEF→ENU 变换计算。**存在精度问题**：使用了地心纬度（`atan2(z, √(x²+y²))`）而非大地纬度，差最大约 0.19°。

### 3.3 SPP 单点定位 (SPP/newSPP.cpp)

**输入**: OBSS（历元观测值）、SATE 向量（卫星位置/钟）、IONUTC（电离层参数）、近似接收机坐标
**输出**: SPPResult（接收机 ECEF + 钟差 + DOP + 卫星高度角）

**观测方程** (GPS L1C 为例，`LS_Line`，`newSPP.cpp:9-139`):
```
Pⁱ = ρⁱ + c·dt_r - c·dt_sⁱ - c·TGDⁱ + Iⁱ + Tⁱ
Lⁱ = Pⁱ - ρⁱ - c·dt_r + c·dt_sⁱ + c·TGDⁱ - Iⁱ - Tⁱ
```

设计矩阵行向量（每颗卫星一行）:
```
Bᵢ = [-dX/ρ, -dY/ρ, -dZ/ρ, 1.0]       单系统
Bᵢ = [-dX/ρ, -dY/ρ, -dZ/ρ, (1.0|0), (0|1.0)]   双系统（GPS+BDS 各一时钟）
```

**BDS 无电离层/对流层改正** (`newSPP.cpp:62-66`):
BDS 观测方程不含 `Iⁱ` 和 `Tⁱ` 项。

**迭代策略** (`LeastSquares`, `newSPP.cpp:328-548`):
```
初始化: Recpos = 近似坐标, estDt = 0, maxIter=20, 容差=1e-6
FOR iter = 1..20:
    SatPVT(obss, Sates, recpos)      ← 更新卫星位置
    LS_Line(...) → B, L, W           ← 构建设计矩阵
    dx = (BᵀWB)⁻¹·BᵀWL               ← 最小二乘求解
    IF ∥dx∥ < 1e-6: converged=true; BREAK
    recpos += dx(0:2), estDt += dx(3)
IF converged:
    V = B·dx - L
    σ² = VᵀWV / (n - p)              ← 单位权方差
    Q = N⁻¹, Cx = σ²·Q               ← 协方差
    PDOP = √(Q₀₀+Q₁₁+Q₂₂), GDOP = √(tr(Q))
```

**权重模型** (每颗卫星):
```
var = a² + b²/sin²(e)
W(i,i) = 1/var
a=1.0, b=0.5     (BDS/GPS 单频)
a=0.3, b=0.5     (BDS 电离层无关组合)
```

**并发策略差异**: 旧版 SPP (`SPP.cpp:140-272`) 使用**各分量独立**收敛判断 `|dx(k)| < 1e-6 ∀k`，新版用倒数的 2-范数。

### 3.4 误差改正模型 (error_cal.cpp)

**Klobuchar 电离层模型** (`error_cal.cpp:33-108`):

完整实现了 ICD-GPS-200D Section 20.3.3.5.2.5 的八参数 Klobuchar：
```
ψ = 0.0137/(E+0.11) - 0.022        地球中心角(半周)
φ_I = φ_u + ψ·cos(A)               IPP 地心大纬度
λ_I = λ_u + ψ·sin(A)/cos(φ_I)      IPP 经度
φ_m = φ_I + 0.064·cos(λ_I-1.617)   地磁纬度
t = 43200·λ_I + GPS_周内秒 (mod 86400)
A_I = Σ αₙ·φ_mⁿ   (n=0..3), A_I ≥ 0
P_I = Σ βₙ·φ_mⁿ   (n=0..3), P_I ≥ 72000
X_I = 2π·(t-50400)/P_I
F = 1+16·(0.53-E)³                  映射函数
IF |X_I| ≤ 1.57: delay = (5e-9 + A_I·cosTaylor(X_I))·F
ELSE:            delay = 5e-9·F      (夜间常数)
delay_meters = delay·c
```

**注意**: 常数 `5.0e-9` 秒夜间偏移正确。Taylor 展开 `1-x²/2+x⁴/24` 近似 cos(x) 正确。`0.064` 半周 ≈ 11.5° 纬偏匹配 ICD。

**Hopfield 对流层模型** (`error_cal.cpp:189-245`):

```
T = T₀ - 0.0065·(h-h₀)                       温度 (K)
P = P₀·(1 - 0.0000226·(h-h₀))^5.225          气压 (mbar)
e = RH·exp(-37.2465+0.213166·T-0.000256908·T²) 水汽分压 (mbar)

h_d = 40136 + 148.72·(T₀-273.16)              干顶高度
h_w = 11000                                    湿顶高度
K_d = 155.2e-7·(P/T)·(h_d - h)                天顶干延迟 (m)
K_w = 155.2e-7·(4810/T²)·e·(h_w - h)          天顶湿延迟 (m)

m_d(E) = 1/sin(√(E²+6.25)·π/180)              干映射函数
m_w(E) = 1/sin(√(E²+2.25)·π/180)              湿映射函数
T_dry = K_d·m_d, T_wet = K_w·m_w
```

**注意**: `lat_rad` 参数被接受但从未使用。水汽分压公式 `exp(-37.2465+...)` 为非标准近似（不同于 Magnus 公式）。

### 3.5 RTK 双差解算 (RTKCAL/newRTK.cpp)

这是项目的核心 RTK 引擎。

#### 3.5.1 观测值匹配与 DD 构建

**DoubleDifMatch** (`newRTK.cpp:184-240`):
```
时间同步: |GPSTime_diff| < 1e-3 s (1ms)
卫星过滤: base_lsresult.sat_prns.size() ≥ 4 AND rover ≥ 4
流程: Match_RoverObs_BaseLS → Match_RoverObs_BaseObs → Match_BaseObs_BaseLS → Match_Process_DD
```

**Match_Process_DD** (`newRTK.cpp:108-181`):
将匹配的观测值按频率分组：每组连续的两个 entries（同卫星、不同信号类型）表示为 B1I 和 B3I。
输入结构 `DDOBSS` → 输出结构 `DualOBS`。

**已知 BUG** (`newRTK.cpp:164-171`): 在 else 分支中错误地使用 `dd_obss.base.obs[j]` 赋值给了流动站伪距：
```cpp
rover_b1i.psr = dd_obss.base.obs[j].psr;   // 应为 dd_obss.rover.obs[j].psr
rover_b3i.psr = dd_obss.base.obs[i].psr;   // 应为 dd_obss.rover.obs[i].psr
```
但因为在良好条件下基站和流动站的伪距差异很小（公里级基线），此 bug 的影响被稀释在最小二乘的残差中。

#### 3.5.2 设计矩阵构建

**BuildDDMat** (`newRTK.cpp:244-426`):

**参考星选择**: 选择高度角最高的卫星以最小化噪声放大。

**维度计算**:
```
n_other = n_sat - 1                                 非参考星数
n_obs = 4·n_other                                   (B1I_C, B3I_C, B1I_L, B3I_L)×每颗
n_params = 3 + 2·n_other                            基线 (dx,dy,dz) + B1I+B3I 模糊度
```

**双差观测方程**（对第 i 颗非参考星）:
```
DD_B1I_pseudo[i]  = (R_B1I_pseudo[i] - R_B1I_pseudo[ref]) - (B_B1I_pseudo[i] - B_B1I_pseudo[ref])
DD_B3I_pseudo[i]  = (R_B3I_pseudo[i] - R_B3I_pseudo[ref]) - (B_B3I_pseudo[i] - B_B3I_pseudo[ref])
DD_B1I_phase[i]   = (R_B1I_phase[i] - R_B1I_phase[ref]) - (B_B1I_phase[i] - B_B1I_phase[ref])
DD_B3I_phase[i]   = (R_B3I_phase[i] - R_B3I_phase[ref]) - (B_B3I_phase[i] - B_B3I_phase[ref])
```

**设计矩阵 A** (4·n_other 行 × (3+2·n_other) 列):

对第 i 颗非参考星，第 i 行（B1I 载波相位）:
```
A(i, 0:2) = -(∇_other - ∇_ref)          三个位置的偏导数
A(i, 3+i) = λ_B1                        (≈ 0.192m)
L(i, 0)   = DD_B1I_phase[i]·λ_B1 - dd_Cal
```

其中 `dd_Cal = (d_rover_other - d_rover_ref) - (d_base_other - d_base_ref)`

伪距行无模糊度项（含三个位置偏导数，λ 列填 0）:
```
L(i+2·n_other, 0) = DD_B1I_pseudo[i] - dd_Cal    (米)
```

**高度角因子**:
```
elev_avg = (ref_elev + other_elev[i]) / 2.0
elev_factor = sin(elev_avg · π/180)
elev_factor = max(elev_factor, 0.1)     低高度角保护
```

#### 3.5.3 浮点解迭代

**Solve_RTK** (`newRTK.cpp:431-639`):

```
BaseLine_Vec = SPP 基线 = rover_pos - base_pos

FOR iter = 0..4 (最多 5 次迭代):
    BuildDDMat → A, L, P
    N = Aᵀ·P·A, U = Aᵀ·P·L
    IF |det(N)| < 1e-12: RETURN (奇异)
    N_inv = N⁻¹
    X = N_inv·U    (n_params × 1)

    IF |X(0,0)|<1e-4 AND |X(1,0)|<1e-4 AND |X(2,0)|<1e-4:
        status = 0 (浮点解确认)
        BREAK

    dd.Rover_pos += X(0..2,0)
    BaseLine_Vec += X(0..2,0)

V = A·X - L
σ₀² = VᵀP·V / (n_obs - n_params)
D_xx = N⁻¹·σ₀²

浮点基线 = BaseLine_Vec + X(0..2,0)    ← 累积 + 最终增量
浮点标准差 = √(D_xx(0..2,0..2))
浮点模糊度 = X(3..n_params-1,0)
```

**权重矩阵** (`BuildP_Matrix`, `newRTK.cpp:903-945`):
```
P_block(i,i) = [sin(elev_avg)]² / (2·σ²)      对角权重（每颗卫星独立）
σ = sigma_phase (0.01 周) 或 sigma_code (1.0 m)
```

**存在单位不一致问题** (`newRTK.cpp:929`): `BuildP_Matrix` 的注释写 `// 注意：elev_avg 已经是弧度`，而 `BuildDDMat:391` 实际使用了 `sin(elev_avg*PI/180)`（度转弧度）。**两个函数的 elev 来源相同（`dd.elev`），单位也相同（度）。BuildP_Matrix 缺少 `*PI/180` 转换是错误的。**

#### 3.5.4 LAMBDA 模糊度固定

**lambda()** (`lambda.cpp:321-344`):

```
1. LD 分解: Q_ââ = L'·diag(D)·L
2. Z 变换 (降相关): reduction(L, D, Z) → L_z, D_z, Z
   └─ 整数 Gauss 消去 + 列置换 (SRC1 算法)
   └─ 目标: 使 D 对角线元素尽可能递减
3. Z 变换模糊度: ž = Zᵀ·â
4. 整数搜索: search(n, m, L_z, D_z, ž, E, s)
   └─ 深度优先树搜索（Branch-and-Bound）
   └─ m=2: 最佳+次佳候选
   └─ 距离: d² = Σ(yₖ²/Dₖ)
   └─ LOOPMAX=5000 步保护
5. 逆变换: F = (Z⁻¹)ᵀ·E → 原始模糊度空间
```

**Ratio 检验** (`newRTK.cpp:603-613`):
```
ratio = s[1] / s[0]
如果 ratio > 2.0 → 接受固定解
否则 → 保留浮点解
如果 s[0] < 1e-10 → ratio = 999.0 (安全回退)
```

这是**单历元** Ratio 检验（不是多历元累积或统计算法如 FF-RT）。

#### 3.5.5 固定解位置更新

**UpdatePositionWithFixedAmbiguity_r** (`newRTK.cpp:641-722`):

```
Q_ab = N⁻¹(0..2, 3..n_params-1)         位置-模糊度协方差
Q_bb = N⁻¹(3..n_params-1, 3..n_params-1) 模糊度协方差

b̂_fixed = b̂_float - Q_ab·Q_bb⁻¹·(â_float - ǎ_fixed)

Q_bb_fixed = Q_bb_float - Q_ab·Q_bb⁻¹·Q_abᵀ
D_fixed = Q_bb_fixed · σ₀²
固定标准差 = √(D_fixed(i,i)), i=0,1,2
```

数学上这是模糊度条件最小二乘的标准公式。固定后基线分量存储在 `rtk_result.baseline_Fixed[3]`。

### 3.6 结果输出与精度评估

**RTKResultWrite** (`Parse.cpp:729-809`):
每历元输出 4 行:
```
行1: status GPSTime ratio dE dN dU          (或 X Y Z)
行2: refPRN otherPRN₁ otherPRN₂ ...
行3: ref_elev other_elev₁ other_elev₂ ...
行4: DD残差1 DD残差2 ... (单位: 0.0001 周)
```

**disp_spp** (`newSPP.cpp:905-964`):
输出收敛标志、卫星数、GDOP/PDOP、位置/钟差估计值与标准差。

---

## 四、项目的优势（面试可主动讲的亮点）

### 4.1 完整的 GNSS 处理管线（全栈实现）
从底层二进制协议解析到高层 LAMBDA 模糊度固定，所有环节均为自主实现，无第三方定位库依赖（仅在 Mat 类的矩阵运算中使用了自研方法）。

- 证据: `OEMRead/` 解析 NovAtel OEM7 协议（CRC32、Sync 检测、消息解码）
- 证据: `SatePos/` 完整实现了 GPS 和 BDS 的广播星历计算，包括 BDS GEO 的 CGCS2000 转换
- 证据: `LAMBDA/` 是自研的 LAMBDA 实现（LD分解+Z变换+整数搜索）

### 4.2 零基线验证通过（精度测试可信）
零基线（Zero-baseline）测试是检验 RTK 引擎精度的黄金标准。本项目的零基线结果证明算法实现正确：
- **GPS 零基线**（`rtk_result.txt`）: 所有 2139 历元固定，dE/dN/dU < 0.7mm
- **BDS 零基线**（`test1.txt`）: 所有 667 历元固定，dE/dN/dU < 2mm
- Ratio 值: GPS 最高 1608，BDS 最高 17648（极强的模糊度置信度）

### 4.3 双系统双频支持
同时支持 GPS (L1+L2) 和 BDS (B1I+B3I)，并通过 `n_sys` 参数实现多系统联合解算，每个系统独立估计接收机钟差（`newSPP.cpp:83-116`）

### 4.4 BDS GEO 的正确处理
对 BDS GEO 卫星（PRN 1-5, 59-63）使用了不同于 IGSO/MEO 的轨道计算：
- Ωₖ 公式不含 `-ωₑ·tₖ` 项（`SatePos.cpp:435`）
- CGCS2000 坐标系转换 Rz(ωₑ·tₖ)·Rx(-5°)
- 速度计算包含旋转矩阵的时间导数

### 4.5 实际的工程应用验证
`CeHuiXueYuan_to_GongxueBu.txt` 展示了真实场景下的表现：~1589m 基线，4 小时连续跟踪全部历元固定，Ratio 范围 20-144，证明了代码在实际测量中的稳定性。

---

## 五、项目的短板与风险点（面试官可能攻击的地方）

按严重程度从高到低排列。

### 【严重】BUG-1: BuildP_Matrix 高度角单位不一致
- **位置**: `newRTK.cpp:929` vs `newRTK.cpp:391`
- **问题**: `BuildDDMat` 计算 `sin(elev_avg*PI/180)` 说明 elev 是度；但 `BuildP_Matrix:929` 使用 `sin(elev_avg)` 假设 elev 是弧度。缺少 `*PI/180` 转换导致权重计算偏高（对于中等高度角，sin 值量值不对）。
- **后果**: 权重矩阵 P 偏离正确值→影响模糊度协方差 Q_ââ→影响 Ratio 检验和固定解。
- **修复**: 统一为 `sin(elev_avg*PI/180)`。

### 【严重】BUG-2: Match_Process_DD 数据复制错误
- **位置**: `newRTK.cpp:164-171`
- **问题**: else 分支中使用 `dd_obss.base.obs[j]` 给 `rover_b1i.psr` 赋值（应为 `dd_obss.rover.obs[j]`）。同理 `rover_b3i.psr`。
- **后果**: B1I/B3I 信号类型的 else 分支中，流动站伪距被基站伪距替代→导致此分支下的 DD 伪距残差异常偏大。
- **触发条件**: 取决于观测值在 `obs` 向量中的排列顺序——当 B3I_D1 (signal_type=2) 排在 B1I_D1 (signal_type=0) 之后时触发。

### 【中等】BUG-3: 旧版 RTK (RTKSolution.cpp) L 向量不使用 O-C
- **位置**: `RTKSolution.cpp:514`
- **问题**: `L(i,0) = ddobs.DDC2I[i]` 直接使用原始双差伪距，而非观测值减计算值 (O-C)。
- **后果**: 单次求解无迭代补偿→如果 SPP 初值偏差较大，线性化误差直接进入解算结果。

### 【中等】BUG-4: BDS 观测方程未应用大气改正
- **位置**: `newSPP.cpp:62-66`
- **问题**: `LS_Line` 仅对 GPS 进行 Klobuchar 电离层和 Hopfield 对流层改正。BDS 的 SPP 观测方程完全省略了大气改正。
- **后果**: BDS SPP 定位精度下降（尤其在低高度角卫星），影响 RTK 初值的准确性。

### 【中等】BUG-5: 实时模式 ProcessRTK 存在三重调用
- **位置**: `RealTime.cpp:500-504, 545-549, 569-574`
- **问题**: `ProcessRTK` 在同一循环迭代中被调用了 3 次——每次 socket 可读后一次，再加一次无条件调用。
- **后果**: 同一历元对可能被写入输出文件最多 3 次→结果文件重复。
- **修复**: 合并为一次调用，或增加去重判断。

### 【中等】BUG-6: 缓冲溢出导致静默数据丢失
- **位置**: `RealTime.cpp:148, 487-488, 534-535`
- **问题**: 当 `lenD + lenR > 2*MAXRAWLEN` 时 `lenD=0`，**无警告地丢弃所有已缓冲数据**。丢失消息中可能包含跨缓冲边界的部分消息。
- **后果**: 高数据速率下丢失观测值→RTK 解算缺失历元。

### 【中等】BUG-7: computeElevationWeight 总是返回单位阵
- **位置**: `SPP.cpp:275-293`
- **问题**: `sin²(elevation)` 被正确计算但从未赋值给 `W` 的任意元素。`W` 初始化为单位阵后未做任何修改。最终返回的就是 Identity。
- **后果**: 旧版 SPP 中所有卫星等权重。
- **修复**: 在第 288 行后添加 `W(i,i)=weight;`

### 【较轻】BUG-8: 地心纬度偏差
- **位置**: `SatePos.cpp:256`
- **问题**: `lat = atan2(z, sqrt(x²+y²))` 使用的是地心纬度而非大地纬度（差约 0.19°）。
- **后果**: ENU 与真 ENU 略倾斜→高度角/方位角偏 0.1° 量级→转换为高度角加权误差。

### 【较轻】BUG-9: Kepler 方程无最大迭代次数保护
- **位置**: `SatePos.cpp:153, 415` + `SatePVT.cpp:57, 192`
- **问题**: 固定点迭代无最大迭代次数限制。若 e>1（损坏星历）可能无限循环。
- **修复**: 增加 `while (iter++ < 30)` 保护。

### 【较轻】BUG-10: LAMBDA 中 LDLT 函数的内层循环错误
- **位置**: `lambda.cpp:188`
- **问题**: `for (i = 1; i < n; i++)` 应为 `for (i = j+1; i < n; i++)`。
- **后果**: 但当前代码**使用 LD 分解**(line 327)而非 LDLT，因此此 bug 不影响实际运行。

### 【信息】Config 中 ratio_min 默认值 vs 实际使用值不一致
- **位置**: `config.cpp:17` (ratio_min=3.0) vs `newRTK.cpp:610` (硬编码 2.0)
- Config 中定义了 `ratio_min = 3.0` 但在实际 Ratio 检验中使用的是硬编码 `2.0`。

### 【信息】大量死代码
以下函数从未被调用：`CalculateTimeDiffBDS`、`calculateRelativisticCorrection`、`calculateTransmitTime`、`computePreciseSatelliteClock`、`BuildCov_DD`、`GAB_LS_Line`、`computeHopfieldTropoCorrection`、`HopfieldSimple`、`GetOBS`。面试时可坦诚承认开发过程中的实验代码未清理。

---

## 六、面试官最可能追问的 15 个问题及参考回答

### Q1: 你的双差观测方程是在哪个坐标系下构建的？为什么？

**答**: ECEF (Earth-Centered Earth-Fixed) 坐标系。原因：(1) 卫星位置从广播星历计算出来天然是 ECEF；(2) 无需考虑地球自转对基线向量的影响（ENU 局部坐标系会随时间变化）；(3) 二重差可以消去接收机钟差和卫星钟差，在 ECEF 下做 LSQ 直接得到基线向量。但代价是基线结果不够直观，需要在 `disp_rtk` 输出前或后处理中转为 ENU。

### Q2: 你为什么选择最高高度角卫星作为参考星？有其他选择策略吗？

**答**: 选择最高高度角卫星作为参考星是经典策略（代码在 `BuildDDMat:262`）。理由：DD 观测值的噪声 = 非参考星噪声 + 参考星噪声，而高度角越高信噪比越高、多路径越小。选择最高高度角卫星能最小化参考星对全体 DD 观测值的"污染"。替代策略：(1) 综合权重最优（所有 DD 噪声和最小）；(2) 几何参考星（GDOP 贡献最大）。但最高高度角简单可靠，线性化的设计矩阵不变。

### Q3: 你的 Ratio 检验为什么用 2.0 作为阈值？这个值怎么选的？

**答**: Ratio = s_best² / s_second² 是 LAMBDA 的标准输出指标（代码在 `newRTK.cpp:603-613`）。2.0 是工程经验值，偏保守（避免误固定）。当前实现是**单历元** Ratio 检验，没有利用历元间的平滑信息。如果面试官追问：可以改进为多历元累积 Ratio 或使用 FF-RT (Fixed Failure-Rate Ratio Test)，基于 Chi² 分布的性质选取与模糊度维度 n 相关的动态阈值。

### Q4: 你的设计矩阵 A 为什么只有 3 个位置参数，而不含对流层、电离层残差参数？

**答**: 对短基线（<15km），电离层和对流层在基站和流动站之间的空间相关性很高，经过双差后残差很小，可以忽略。对于中长基线，需要对流层残余进行参数估计或使用映射函数。代码对 BDS 是通过 B1I+B3I 双频组合提供了一部分电离层消除（在 SPP 的电离层无关路径中），但 RTK 的 DD 构建没有显式的电离层估计参数。这是一个明确的简化（代码在 `BuildDDMat:267`，`n_params = 3+2n_other`）。

### Q5: 你的 RTK 收敛判断标准是什么？为什么选 0.1mm？

**答**: 在 `Solve_RTK` 中，收敛条件为 `|dx| < 1e-4, |dy| < 1e-4, |dz| < 1e-4`（`newRTK.cpp:488-489`）。选择 0.1mm 是因为：(1) 载波相位精度优于 1cm，模糊度固定后精度毫米级；(2) 迭代 5 次已基本收敛到浮点解精度极限；(3) 更小的收敛阈值没有物理意义（受整周模糊度浮点解抖动影响）。

### Q6: 你的代码如何保证基站和流动站的历元同步？

**答**: `DoubleDifMatch` 要求基站和流动站的 GPS 时间差 < 1ms（`newRTK.cpp:199-200`）。在实时模式下 `ProcessRTK` 中，通过双指针法匹配 GPS week + SOW，匹配后删除已处理的流动站数据，基站数据由调用方维护最多 5 个历元的滑动窗口。这是标准的 receiver-to-receiver 同步方法。

### Q7: 你在 BDS GEO 卫星处理中做了什么特别处理？为什么？

**答**: BDS GEO（PRN 1-5, 59-63）与 IGSO/MEO 有两个关键区别（`SatePos.cpp:431-460`）:
1. **Ωₖ 公式不同**: GEO 使用 `Ωₖ = Ω₀ + Ω̇·tₖ - ωₑ·tₒₑ`（不含 `-ωₑ·tₖ` 项），因为 GEO 卫星的升交点对地球观测者是固定的。
2. **CGCS2000 坐标系转换**: GEO 的计算坐标基于惯性系，需要通过 `Rz(ωₑ·tₖ)·Rx(-5°)` 转换到 CGCS2000 固定坐标系。

### Q8: 你的权重矩阵 P 是怎么构造的？为什么选择对角阵？

**答**: 在 `BuildP_Matrix` 中（`newRTK.cpp:903-945`），权重公式为 `w = sin²(E_avg) / (2·σ²)`。选择对角阵简化是因为：(1) 各卫星 DD 观测值虽然理论上相关（共享参考星导致非对角协方差），但对角近似是卫星导航中的常见做法；(2) 代码中确实有过相关协方差矩阵的实现（`BuildCov_DD`，`newRTK.cpp:874-891`），构建了 `Cov(i,j) = σ² * [2,1,1...; 1,2,1...; ...]` 的全协方差矩阵，但从未被调用。
如果需要被追问——可以承认这是一个已知简化，对短基线精度影响小（毫米级），对长基线可通过相关权矩阵改进 5-15%。

### Q9: 你的 LAMBDA 是自己写的吗？和 RTKLIB 的 LAMBDA 有什么区别？

**答**: 完全自研。实现了完整的 LD 分解（`lambda.cpp:158`）、SRC1 Z 降相关（`lambda.cpp:229`）、深度优先整数搜索（`lambda.cpp:247`）。和 RTKLIB 的区别：(1) RTKLIB 使用 QR 分解做浮点解，我用的是自研 `Mat` 类的 Gauss-Jordan 逆；(2) RTKLIB 有分部 Ratio 检验，我只实现了简单的比值检验；(3) 我的 Z 变换使用了 FMFAC5 风格的 LD 分解而不是 LDL' 分解。但核心算法流程是一致的。

### Q10: LAMBDA 搜索中距离怎么计算的？

**答**: 在搜索函数中（`lambda.cpp:247-308`），距离计算基于 Z 变换后的条件方差：
```
newdist = dist[k] + y²/D[k]
```
其中 `y = ž[k] - z[k]` 是条件估计与实际整数值的残差，`D[k]` 是 LDL^T 分解中对角阵的第 k 个元素。这个二次型等价于 `(â-ǎ)ᵀ·Q⁻¹·(â-ǎ)` 在 Z 变换空间中的分解形式。搜索策略是深度优先 + zig-zag 展开（先从最近整数向外交替搜索：+1, -2, +3, -4...）。

### Q11: 你的载波相位和伪距的方差比例是多少？为什么要这样设？

**答**: 在 `BuildP_Matrix` 中，`sigma_phase = 0.01`（周）对应 B1I 约 2mm，`sigma_code = 1.0m`（`newRTK.cpp:921`）。相位/伪距方差比 ≈ 2500。选择 0.01 周的经验值是因为：(1) 商用接收机 1Hz 相位噪声通常 <0.01 周；(2) 伪距 1m sigma 代表了广播星历 GPS C/A 的典型精度。RTK 的浮点解主要依赖载波相位（方差小），伪距提供的是无模糊度的距离约束来约束基线长度和消除粗差。

### Q12: 你的 SPP 为什么迭代 20 次上限？实际一般几次收敛？

**答**: 20 次是安全上限（`newSPP.cpp:344`）。实际在近似坐标偏差<100km 时（如 hardcoded `-2267700, 5009400, 3221000` 位于中国），通常 3-5 次就收敛到 1e-6m。如果 20 次还未收敛，则说明观测值存在问题（如粗差、PDOP 过高、星历错误等）。值得一提的是，旧版 `SPP.cpp` 使用的是**分量独立**收敛（`|dx[k]| < 1e-6`），新版由**2-范数**（`∥dx∥ < 1e-6`）。

### Q13: 你为什么要给 BDS 和 GPS 分别估计接收机钟差？

**答**: 在双系统联合解算中（`n_sys = 2`，`newSPP.cpp:83-116`），BDS 和 GPS 各自有独立的接收机钟差。这是因为：(1) GPS 和 BDS 的时间系统不同（GPS Time vs BDT），两个系统时间差约 14 秒；(2) 接收机硬件对不同 GNSS 系统存在 Inter-System Bias (ISB)。如果不独立估计，则等效于把 ISB 吸收进伪距残差中。

### Q14: 你的零基线测试中为什么 BDS 的残差比 GPS 大？

**答**: 从 `test1.txt` 和 `rtk_result.txt` 可以看出：BDS 零基线 dE/dN/dU < 2mm，GPS < 0.5mm。可能原因：(1) BDS B1I/B3I 信号的载波相位观测噪声普遍比 GPS L1/L2 高约 3-5 倍；(2) BDS 广播星历精度低于 GPS（BDS-2 轨道精度 10m vs GPS ~2m）；(3) 但 BDS 卫星数更多（10 vs 7），在参数估计中带来了更多观测冗余。

### Q15: 你的代码有什么测试覆盖或自动化测试吗？

**答**: 没有单元测试框架。但有两个实证验证：
1. **零基线测试**（GPS + BDS 两个数据集）：直接测量代码内部精度，是最严格的 RTK 验证手段。
2. **LAMBDA 单元测试数据**（`amb2g_L1L2.txt` + `result.txt`）：可以对比 LAMBDA 输出与已知正确结果。
3. `TestTimeConversion()` 函数（`DataProcess.cpp` 末尾）对时间转换做了硬编码测试。

---

## 七、需要重点记忆的数字与事实

### 7.1 关键精度指标

| 指标 | 值 | 来源 |
|------|-----|------|
| GPS 零基线 ENU 精度 | < 0.5mm RMS | `rtk_result.txt` (2303周, 36min, 2139 历元全部固定) |
| BDS 零基线 ENU 精度 | < 2mm RMS | `test1.txt` (2399周, 11min, 667 历元全部固定) |
| 实测基线 (~1590m) 固定率 | 100% (4h内全部固定) | `CeHuiXueYuan_to_GongxueBu.txt` (2396周) |

### 7.2 实测基线参数

| 参数 | 值 |
|------|-----|
| 基线名称 | 测绘学院 → 工学部 (CeHuiXueYuan → GongxueBu) |
| 近似基线长度 | ~1590 m (ECEF 分量: X≈474m, Y≈-675m, Z≈1358m) |
| 观测时长 | 约 4 小时 (GPS Week 2396, SOW 278425-292816) |
| 历元数 | 约 14297 历元 (全部固定) |
| Ratio 范围 | 20 ~ 144 |
| 卫星数范围 | 11 ~ 14 颗 BDS |

### 7.3 零基线参数

| 参数 | GPS (rtk_result) | BDS (test1) |
|------|-------------------|--------------|
| GPS 周 | 2303 | 2399 |
| 时段 | SOW 442430-444601 (约36min) | SOW 188944-189610 (约11min) |
| 历元数 | 2139 | 667 |
| 固定率 | 100% | 100% |
| 卫星数 | 6-8 (GPS) | 10 (BDS) |
| dE/dN/dU 范围 | ±0.7mm / ±0.7mm / ±0.5mm | ±0.8mm / ±1.4mm / ±1.0mm |
| Ratio 范围 | 25 ~ 1608 | 2500 ~ 17648 |

### 7.4 代码中的关键阈值与常量

| 常量 | 值 | 位置 | 含义 |
|------|-----|------|------|
| 时间同步容差 | 1e-3 (1ms) | `newRTK.cpp:200`, `RTKSoution.h:8` | 基站-流动站时间匹配容差 |
| 高度角截止 | 15° | `newSPP.cpp:723,781` | SPP 有效卫星最低高度角 |
| C/N0 门限 | 35 dB-Hz | `Parse.cpp:322,431` | 观测值最低信噪比 |
| SPP 迭代上限 | 20 次 | `newSPP.cpp:344` | SPP LSQ 最大迭代次数 |
| SPP 收敛阈值 | 1e-6 m | `newSPP.cpp:345` | SPP 位置修正的范数收敛标准 |
| RTK 迭代上限 | 5 次 | `newRTK.cpp:474` | RTK LSQ 最大迭代次数 |
| RTK 收敛阈值 | 1e-4 m (0.1mm) | `newRTK.cpp:489` | RTK 位置分量修正的收敛标准 |
| 矩阵奇异阈值 | 1e-12 | `newRTK.cpp:479` | 法矩阵行列式下限 |
| Ratio 检验阈值 | 2.0 | `newRTK.cpp:610` | 模糊度固定接受阈值 |
| Ratio 零保护 | 1e-10 | `newRTK.cpp:603` | s[0]趋于零时的安全回退 |
| Ratio 最大值 | 999.0 | `newRTK.cpp:603` | 零残差时的默认 ratio |
| 载波相位 sigma | 0.01 周 (≈2mm B1I) | `newRTK.cpp:921` | 权重矩阵相位标准差 |
| 伪距 sigma | 1.0 m | `newRTK.cpp:921` | 权重矩阵伪距标准差 |
| 高度角权重下限 | 0.1 (=sin(5.7°)) | `newRTK.cpp:393,939` | 低高度角权重保护 |
| 最小 PSR (GPS) | 10,000 km | `SPP.h:4` | 有效伪距下限（排除粗差） |
| 最大 PSR (GPS) | 30,000 km | `SPP.h:5` | 有效伪距上限 |
| GPS 星历时限 | 7200 s (2h) | `SatePVT.cpp:27`, `BinRead.cpp:361` | 最大星历有效时间差 |
| BDS 星历时限 | 3600 s (1h) | `SatePos.cpp:384`, `BinRead.cpp:289` | 最大星历有效时间差 |
| Kepler 收敛容差 | 1e-12 rad | `SatePos.cpp:153` | Kepler 方程迭代收敛标准 |
| 最大缓冲区大小 | 40960×2 = 81920 B | `RealTime.cpp:102` | TCP 数据接收缓冲区 |
| 滑动窗口大小 | 5 个历元 | `RealTime.cpp:166-170` | SPP 结果缓冲区大小 |

### 7.5 关键物理常数

| 常数 | 值 | 来源 |
|------|-----|------|
| GPS GM | 3.986005e14 m³/s² | `SatePos.h:7` |
| BDS GM | 3.986004418e14 m³/s² | `SatePos.h:8` |
| 光速 c | 299792458.0 m/s | SatePos.h:5, Parse.h:24 等多处 |
| GPS F (相对论) | -4.442807633e-10 | `SatePos.h:9` |
| BDS F (相对论) | -4.442807309e-10 | `SatePos.h:10` |
| WGS-84 a | 6378137.0 m | `Base.h:10` |
| WGS-84 1/f | 298.257223563 | `Base.h:11` |
| GPS L1 频率 | 1575.42 MHz | `Parse.h:18` |
| GPS L2 频率 | 1227.60 MHz | `Parse.h:19` |
| BDS B1I 频率 | 1561.098 MHz | `Parse.h:21` |
| BDS B3I 频率 | 1268.52 MHz | `Parse.h:22` |
| GPS L1 波长 | ~0.1903 m | `Parse.h:28` |
| GPS L2 波长 | ~0.2442 m | `Parse.h:29` |
| BDS B1I 波长 | ~0.1920 m | `Parse.h:26` |
| BDS B3I 波长 | ~0.2364 m | `Parse.h:27` |
| 地球自转速率 | 7.2921151467e-5 rad/s (GPS) / 7.2921150e-5 (BDS) | `SatePos.h:6` |

### 7.6 模糊度维度示例

假设 13 颗 BDS 共视卫星（参考 CToG 场景）:
```
n_sat = 13, n_other = 12
n_obs = 4 × 12 = 48            (每次 DD: B1I_C, B3I_C, B1I_L, B3I_L)
n_params = 3 + 2×12 = 27       (3 位置 + 24 模糊度)
n_ambi = 24                     (送入 LAMBDA 的模糊度数量)
```

---

## 八、我还未完全想清楚、今晚需要补课的点

### 8.1 Match_Process_DD bug 的实际影响
- `newRTK.cpp:164-171` 的数据复制错误何时触发？是否在实测数据中（B1I/B3I 的观测值排列顺序）确实触发了？
- 如果触发了——实测数据中所有历元都固定且精度良好，是代码表现不正常，还是 bug 的影响被最小二乘的残差吸收？

### 8.2 关于 BuildP_Matrix 的单位不一致
- `newRTK.cpp:391` 写的是 `sin(elev_avg*PI/180)`（说明 elev 为度）
- `newRTK.cpp:929` 写的是 `sin(elev_avg)` 但注释说"已经是弧度"
- 字节实际来源是 `base_lsresult.sat_elevs`（SPP 计算的 `ComputeElevation` 返回的度数）
- 这意味着 `BuildP_Matrix` 缺少转换。但它仍工作，因为：(1) 权重公式中的因子值不影响模糊度；（2）Ratio 检验对权重适度的缩放不敏感。

但面试时如果被问到"你的高度角加权对不对"，我需要能解释清楚这件事。

### 8.3 LAMBDA 搜索的 LOOPMAX=5000 在实际中是否够用
- 24 个模糊度在 Z 变换后深度优先搜索——如果去相关效果不好，搜索树可能爆炸
- 5000 次迭代是否足够？在实测数据中是否发生过溢出（返回 -1 用浮点解）？

### 8.4 什么是"Partial Ambiguity Resolution"以及本项目为什么不支持
- 本项目固定所有非参考星的模糊度，没有 Partial Fix（子集固定）策略
- 如果部分模糊度无法固定，整个历元将降级为浮点解
- 面试时如果被问到"遇到无法全部固定的情况怎么办"，要准备好回答何时需要 PAR

### 8.5 为什么 BDS 零基线 Ratio 比实测基线高出约 100 倍
- BDS 零基线 Ratio 高达 2500-17648，而实测基线只有 20-144
- 理论原因：零基线 DD 残差主要来自观测噪声（小），而实测基线的对流层/电离层残留贡献了更大的 s[0] 和 s[1] 的随机波动，导致次佳/最佳之比下降

### 8.6 SPP 中对 BDS 未使用大气改正的设计理由
- `newSPP.cpp:62-66` 不做 BDS 电离层/对流层改正
- 是因为没有实现 BDS Klobuchar 参数？还是认为 BDS SPP 精度要求不高而刻意简化？
- 如果面试官质疑：可以回应 BDS 也有对应的 Klobuchar 参数，但代码中只实现了 GPS 模型

### 8.7 实时 SPP 预设固定初值的合理性
- `RealTime.cpp:105` 的 `-2267700, 5009400, 3221000` 是中国的某点。（`main.cpp:56` 显示已知基站坐标是 `28.1825754300N, 112.9965413397E, 63.0388m`，这是长沙附近一个测绘学院的位置）
- 如果用于其他地方则 SPP 收敛需要 4-6 次迭代而非 3-4 次