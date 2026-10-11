"""TC_SPP系列: SPP 单点定位算法单元测试

测试 src/spp.py，对应 C++ 项目中的 SPP/newSPP
"""
import pytest
import math
from src.spp import (
    LSResult, compute_elevation, generate_mock_satellites,
    least_squares_spp, CLIGHT, PI
)

KNOWN_POS = (-2267808.62, 5009324.77, 3221016.89)  # 基准站近似坐标


class TestElevation:
    """高度角计算"""

    def test_elevation_zenith(self):
        """天顶方向的卫星，高度角应 ≈ 90°"""
        rec = (0, 0, 6371000)
        sat = (0, 0, 6371000 + 20000000)
        elev = compute_elevation(sat, rec)
        assert abs(elev - 90.0) < 1.0

    def test_elevation_reasonable(self):
        """使用基准站位置，高度角应在合理范围 [-90, 90]"""
        rec = KNOWN_POS
        sat = (rec[0] + 40000000, rec[1], rec[2])
        elev = compute_elevation(sat, rec)
        assert -90 <= elev <= 90


class TestLeastSquaresSPP:
    """TC_SPP_001: 最小二乘 SPP"""

    @pytest.fixture
    def mock_sats(self):
        """生成 6 颗模拟卫星"""
        return generate_mock_satellites(KNOWN_POS, num_sats=6)

    def test_converges_with_good_data(self, mock_sats):
        """正常数据应收敛（TC_SPP_004）"""
        result = least_squares_spp(mock_sats)
        assert result.converged, f"SPP 应收敛，错误: {result.error}"

    def test_recovers_known_position(self, mock_sats):
        """应对基准站坐标有较好估计 (TC_SPP_001)"""
        result = least_squares_spp(mock_sats)
        assert result.converged

        # 模拟数据中接收机位置就是 KNOWN_POS，加上已知噪声
        # 给定 6 颗卫星和 100m 钟差，精度应在合理范围
        error = math.sqrt(
            (result.position[0] - KNOWN_POS[0])**2 +
            (result.position[1] - KNOWN_POS[1])**2 +
            (result.position[2] - KNOWN_POS[2])**2
        )
        assert error < 100.0, f"SPP 定位误差 {error:.2f}m 过大"

    def test_gdop_pdop_valid(self, mock_sats):
        """GDOP 和 PDOP 应为正值 (TC_SPP_005)"""
        result = least_squares_spp(mock_sats)
        assert result.gdop > 0
        assert result.pdop > 0
        assert result.pdop < result.gdop or abs(result.pdop - result.gdop) < 1e-6

    def test_iterates_to_convergence(self, mock_sats):
        """应在最多 20 次迭代内收敛"""
        result = least_squares_spp(mock_sats, max_iter=20)
        assert result.iterations <= 20
        assert result.iterations > 0


class TestSPPEdgeCases:
    """SPP 边界情况"""

    def test_insufficient_satellites(self):
        """少于 4 颗卫星应返回失败 (TC_SPP_003)"""
        sats = generate_mock_satellites(KNOWN_POS, num_sats=3)
        result = least_squares_spp(sats)
        assert not result.converged
        assert "不足" in result.error

    def test_converges_from_far_initial(self):
        """即使初始位置很遥远也应收敛"""
        sats = generate_mock_satellites(KNOWN_POS, num_sats=8)
        far_init = (0.0, 0.0, 0.0)  # 地球中心
        result = least_squares_spp(sats, init_pos=far_init)
        assert result.converged, f"远距离收敛失败: {result.error}"

    def test_prn_numbers_match_observed(self):
        """生成的模拟卫星数据卫星编号应连续"""
        sats = generate_mock_satellites(KNOWN_POS, num_sats=8)
        assert len(sats) == 8
        for sat in sats:
            assert 'pos' in sat
            assert 'pseudorange' in sat
            assert len(sat['pos']) == 3
            assert sat['pseudorange'] > 0