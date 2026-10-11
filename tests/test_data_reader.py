"""TC_READ系列: RINEX 数据读取单元测试

测试 src/data_reader.py，对应 C++ 项目中的 FileRead/RTKDataRead
验证 RINEX 3.04 格式解析的正确性
"""
import pytest
import os
from src.data_reader import (
    read_rinex_obs, get_bds_b1b3_obs,
    RinexObsFile, EpochData, Observation
)

# RINEX 测试数据路径
BASE_OBS = os.path.join(os.path.dirname(__file__), "..", "RINEX", "base.obs")
BASE_NAV = os.path.join(os.path.dirname(__file__), "..", "RINEX", "base.nav")
ROVE_OBS = os.path.join(os.path.dirname(__file__), "..", "RINEX", "rove.obs")


class TestReadObsFile:
    """TC_READ_002: 读取 RINEX 观测文件"""

    @pytest.fixture
    def base_obs(self):
        """加载基准站观测文件"""
        if not os.path.exists(BASE_OBS):
            pytest.skip(f"测试数据文件不存在: {BASE_OBS}")
        return read_rinex_obs(BASE_OBS)

    def test_file_reads_successfully(self, base_obs):
        """文件应能成功读取"""
        assert isinstance(base_obs, RinexObsFile)

    def test_approximate_position(self, base_obs):
        """RINEX 头中的近似坐标应正确解析"""
        x, y, z = base_obs.approx_position
        # 基准站近似坐标（米级）
        assert abs(x + 2267808) < 10  # x ≈ -2267808
        assert abs(y - 5009324) < 10  # y ≈ 5009324
        assert abs(z - 3221016) < 10  # z ≈ 3221016

    def test_multiple_epochs(self, base_obs):
        """应有多个历元的数据"""
        assert base_obs.num_epochs > 10, \
            f"期望 >10 个历元，实际 {base_obs.num_epochs} 个"

    def test_first_epoch_has_satellites(self, base_obs):
        """第一个历元应有卫星观测值"""
        first = base_obs.epochs[0]
        assert first.num_sats > 0

    def test_gps_satellites_present(self, base_obs):
        """应包含 GPS 卫星"""
        first = base_obs.epochs[0]
        gps_sats = [s for s in first.satellites if s.system == 'G']
        assert len(gps_sats) > 0

    def test_bds_satellites_present(self, base_obs):
        """应包含 BDS 卫星"""
        first = base_obs.epochs[0]
        bds_sats = [s for s in first.satellites if s.system == 'C']
        assert len(bds_sats) > 0


class TestObservationFields:
    """TC_READ_004: 验证解析的观测值字段"""

    @pytest.fixture
    def base_obs(self):
        if not os.path.exists(BASE_OBS):
            pytest.skip(f"测试数据文件不存在: {BASE_OBS}")
        return read_rinex_obs(BASE_OBS)

    def test_gps_pseudorange_not_none(self, base_obs):
        """GPS C1C 伪距应有有效值"""
        first = base_obs.epochs[0]
        gps_sats = [s for s in first.satellites if s.system == 'G']
        for sat in gps_sats:
            assert 'C1C' in sat.pseudorange, f"{sat.sat} 缺少 C1C 伪距"
            assert sat.pseudorange['C1C'] > 1e6  # 伪距 > 1000km
            assert sat.pseudorange['C1C'] < 4e7   # 伪距 < 40000km

    def test_bds_pseudorange_not_none(self, base_obs):
        """BDS C2I 伪距应有有效值"""
        first = base_obs.epochs[0]
        bds_sats = [s for s in first.satellites if s.system == 'C']
        for sat in bds_sats:
            assert 'C2I' in sat.pseudorange, f"{sat.sat} 缺少 C2I 伪距"

    def test_satellite_prn_matches_id(self, base_obs):
        """PRN 号和卫星 ID 应匹配"""
        first = base_obs.epochs[0]
        for sat in first.satellites:
            expected = int(sat.sat[1:])
            assert sat.prn == expected, \
                f"PRN 不匹配: sat={sat.sat}, prn={sat.prn}"


class TestBDSDualFreq:
    """TC_READ_005: BDS 双频观测值提取"""

    @pytest.fixture
    def base_obs(self):
        if not os.path.exists(BASE_OBS):
            pytest.skip(f"测试数据文件不存在: {BASE_OBS}")
        return read_rinex_obs(BASE_OBS)

    def test_bds_dual_freq_available(self, base_obs):
        """至少应有部分北斗卫星同时有 B1I(C2I) 和 B3I(C6I)"""
        first = base_obs.epochs[0]
        dual = get_bds_b1b3_obs(first)
        assert len(dual) > 0, "没有 BDS 双频观测卫星"


class TestRoverObsFile:
    """TC_READ_003: 流动站观测文件"""

    @pytest.fixture
    def rover_obs(self):
        if not os.path.exists(ROVE_OBS):
            pytest.skip("流动站观测文件不存在")
        return read_rinex_obs(ROVE_OBS)

    def test_rover_file_reads(self, rover_obs):
        """流动站文件应有数据"""
        assert rover_obs.num_epochs > 0


class TestFileNotFound:
    """TC_READ_006: 异常——文件不存在"""

    def test_nonexistent_file(self):
        """不存在的文件应抛 FileNotFoundError"""
        with pytest.raises(FileNotFoundError):
            read_rinex_obs("nonexistent_file.obs")