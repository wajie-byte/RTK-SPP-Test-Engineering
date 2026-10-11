"""RINEX 数据读取模块

解析 RINEX 3.x 格式的观测文件和导航文件
对应 C++ 项目中的 FileRead/RTKDataRead.h 和 OEMRead 模块
"""
from dataclasses import dataclass, field
from typing import List, Dict, Optional
import re


# ==================== 数据结构 ====================

@dataclass
class GPSTime:
    week: int = 0
    sow: float = 0.0


@dataclass
class Observation:
    """单颗卫星的一条观测记录"""
    sat: str        # 卫星编号，如 "G04", "C01"
    system: str     # "G" 或 "C"
    prn: int        # PRN 号
    pseudorange: Dict[str, float] = field(default_factory=dict)  # 频点 -> 伪距
    carrier_phase: Dict[str, float] = field(default_factory=dict)  # 频点 -> 载波
    snr: Dict[str, float] = field(default_factory=dict)  # 频点 -> 信噪比


@dataclass
class EpochData:
    """一个历元的所有观测数据"""
    time: GPSTime
    year: int
    month: int
    day: int
    hour: int
    minute: int
    second: float
    satellites: List[Observation] = field(default_factory=list)

    @property
    def num_sats(self) -> int:
        return len(self.satellites)


@dataclass
class RinexObsFile:
    """RINEX 观测文件"""
    filename: str
    approx_position: tuple = (0.0, 0.0, 0.0)  # 近似坐标 (x,y,z)
    gps_obs_types: List[str] = field(default_factory=list)
    bds_obs_types: List[str] = field(default_factory=list)
    epochs: List[EpochData] = field(default_factory=list)

    @property
    def num_epochs(self) -> int:
        return len(self.epochs)


# ==================== 解析函数 ====================

def _parse_float(s: str) -> float:
    """解析 RINEX 的 D 格式科学计数法"""
    s = s.strip()
    if not s:
        return 0.0
    return float(s.replace('D', 'E').replace('d', 'e'))


def read_rinex_obs(filename: str) -> RinexObsFile:
    """读取 RINEX 3.x 观测文件"""
    result = RinexObsFile(filename=filename)

    with open(filename, 'r', encoding='utf-8', errors='ignore') as f:
        lines = f.readlines()

    # 解析头文件
    header_end = 0
    for i, line in enumerate(lines):
        if "END OF HEADER" in line:
            header_end = i + 1
            break
        if "APPROX POSITION XYZ" in line:
            parts = line.split()
            result.approx_position = (
                float(parts[0]), float(parts[1]), float(parts[2])
            )
        if "SYS / # / OBS TYPES" in line:
            if line.startswith("G"):
                n_types = int(line[4:6].strip())
                result.gps_obs_types = line[7:].strip().split()[:n_types]
            elif line.startswith("C"):
                n_types = int(line[4:6].strip())
                result.bds_obs_types = line[7:].strip().split()[:n_types]

        # BDS 频点在 RINEX 3.04 中的映射：C2I=B1I, C6I=B3I

    # 解析观测数据
    i = header_end
    while i < len(lines):
        line = lines[i]
        if line.startswith('>'):
            # 历元头行：> 2024 03 01 02 53 46.0000000  0 29
            parts = line[1:].strip().split()
            year, month, day = int(parts[0]), int(parts[1]), int(parts[2])
            hour, minute = int(parts[3]), int(parts[4])
            second = float(parts[5])
            epoch_flag = int(parts[6])
            num_sats = int(parts[7])

            epoch = EpochData(
                time=GPSTime(),
                year=year, month=month, day=day,
                hour=hour, minute=minute, second=second
            )

            # 读取 num_sats 行观测数据
            i += 1
            for _ in range(num_sats):
                if i >= len(lines):
                    break
                sat_line = lines[i]
                sat_id = sat_line[:3].strip()

                if sat_id.startswith('G'):
                    epoch.satellites.append(
                        _parse_gps_obs(sat_line, result.gps_obs_types, sat_id)
                    )
                elif sat_id.startswith('C'):
                    epoch.satellites.append(
                        _parse_bds_obs(sat_line, result.bds_obs_types, sat_id)
                    )
                i += 1

            result.epochs.append(epoch)
        else:
            i += 1

    return result


def _parse_gps_obs(line: str, obs_types: List[str], sat_id: str) -> Observation:
    """解析 GPS 观测行"""
    sat = sat_id[0]
    prn = int(sat_id[1:])
    obs = Observation(sat=sat_id, system='G', prn=prn)

    # RINEX 3 每 16 个字符一个值
    pos = 3
    for ot in obs_types:
        if pos + 16 > len(line):
            break
        val_str = line[pos:pos+16].strip()
        pos += 16
        if not val_str:
            continue
        val = _parse_float(val_str)
        if ot.startswith('C'):  # 伪距 (C1C, C2W, C5Q)
            obs.pseudorange[ot] = val
        elif ot.startswith('L'):  # 载波相位
            obs.carrier_phase[ot] = val
        elif ot.startswith('S'):
            obs.snr[ot] = val
        # Dxx (多普勒) 暂时跳过

    return obs


def _parse_bds_obs(line: str, obs_types: List[str], sat_id: str) -> Observation:
    """解析 BDS 观测行

    BDS RINEX 3.04 频点映射：
    - C2I → B1I (1561.098 MHz)
    - C7I → B2b
    - C6I → B3I (1268.52 MHz)
    """
    sat = sat_id[0]
    prn = int(sat_id[1:])
    obs = Observation(sat=sat_id, system='C', prn=prn)

    pos = 3
    for ot in obs_types:
        if pos + 16 > len(line):
            break
        val_str = line[pos:pos+16].strip()
        pos += 16
        if not val_str:
            continue
        val = _parse_float(val_str)
        if ot.startswith('C'):
            obs.pseudorange[ot] = val
        elif ot.startswith('L'):
            obs.carrier_phase[ot] = val
        elif ot.startswith('S'):
            obs.snr[ot] = val

    return obs


def get_bds_b1b3_obs(epoch: EpochData) -> List[dict]:
    """从历元中提取北斗 B1I(C2I) 和 B3I(C6I) 双频观测值

    返回: [
        {'sat': 'C01', 'prn': 1, 'C2I_psr': ..., 'C6I_psr': ...},
        ...
    ]
    """
    result = []
    for obs in epoch.satellites:
        if obs.system == 'C':
            has_b1 = 'C2I' in obs.pseudorange
            has_b3 = 'C6I' in obs.pseudorange
            if has_b1 and has_b3:
                result.append({
                    'sat': obs.sat,
                    'prn': obs.prn,
                    'C2I_psr': obs.pseudorange['C2I'],
                    'C6I_psr': obs.pseudorange['C6I'],
                    'C2I_adr': obs.carrier_phase.get('L2I', 0),
                    'C6I_adr': obs.carrier_phase.get('L6I', 0),
                })
    return result