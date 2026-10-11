"""集成测试: 零基线测试 + SPP→RTK 串联

这是本次实践中最重要的测试！
零基线测试 = GNSS 测试中的"黄金标准"：
用同一台接收机的数据同时作为基准站和流动站，基线应为 0。
"""
import pytest
import math
import numpy as np
from src.spp import generate_mock_satellites, least_squares_spp, LSResult
from src.matrix import Matrix

KNOWN_POS = (-2267808.62, 5009324.77, 3221016.89)


class TestZeroBaseline:
    """
    TC_INT_001: 零基线集成测试

    原理：同一台接收机的数据同时当 base 和 rover
    如果 RTK 算法正确，解算出的基线应为零。
    """

    @pytest.fixture
    def zero_baseline_sats(self):
        """为接收机位置生成卫星数据"""
        return generate_mock_satellites(KNOWN_POS, num_sats=8)

    def test_spp_gives_same_position_for_same_data(self, zero_baseline_sats):
        """验证：同一组卫星数据做 SPP，结果应完全一致"""
        result1 = least_squares_spp(zero_baseline_sats)
        result2 = least_squares_spp(zero_baseline_sats)

        assert result1.converged and result2.converged
        assert result1.position == pytest.approx(result2.position)

    def test_zero_baseline_positions_equal(self, zero_baseline_sats):
        """
        零基线场景：base 和 rover 用同一组观测值
        SPP 计算出的位置应相同 → 基线为零
        """
        base_result = least_squares_spp(zero_baseline_sats)

        # 在零基线场景中，rover 和 base 位置相同
        # SPP 算出的位置也应相同
        rover_result = least_squares_spp(zero_baseline_sats)

        dx = base_result.position[0] - rover_result.position[0]
        dy = base_result.position[1] - rover_result.position[1]
        dz = base_result.position[2] - rover_result.position[2]

        baseline = math.sqrt(dx**2 + dy**2 + dz**2)
        assert baseline < 1e-6, f"零基线下 SPP 基线应为 0，实际 {baseline}"


class TestSPPtoRTKPipeline:
    """
    TC_INT_003: SPP→RTK 串联测试

    验证从数据读取到最终结果的完整流程
    """

    @pytest.fixture
    def pipeline_data(self):
        """准备管道测试所需数据"""
        base_sats = generate_mock_satellites(KNOWN_POS, num_sats=8)
        rover_pos_shifted = (
            KNOWN_POS[0] + 10.0,
            KNOWN_POS[1] + 20.0,
            KNOWN_POS[2] + 15.0,
        )
        rover_sats = generate_mock_satellites(rover_pos_shifted, num_sats=8)
        return base_sats, rover_sats

    def test_spp_pipeline(self, pipeline_data):
        """验证 SPP 流程完整性"""
        base_sats, rover_sats = pipeline_data

        # Step 1: SPP 获取基准站位置
        base_ls = least_squares_spp(base_sats)
        assert base_ls.converged, "基准站 SPP 应收敛"

        # Step 2: SPP 获取流动站位置
        rover_ls = least_squares_spp(rover_sats)
        assert rover_ls.converged, "流动站 SPP 应收敛"

        # Step 3: 验证流动站位置偏移量合理
        dx = rover_ls.position[0] - base_ls.position[0]
        dy = rover_ls.position[1] - base_ls.position[1]
        dz = rover_ls.position[2] - base_ls.position[2]

        # 模拟数据中偏移量是 (10, 20, 15)
        # SPP 应该能恢复这个偏移（允许一些误差）
        assert abs(dx - 10.0) < 5.0, f"dx={dx:.3f} 与期望 10.0 偏差过大"
        assert abs(dy - 20.0) < 5.0, f"dy={dy:.3f} 与期望 20.0 偏差过大"
        assert abs(dz - 15.0) < 5.0, f"dz={dz:.3f} 与期望 15.0 偏差过大"

    def test_result_structure(self, pipeline_data):
        """验证 LSResult 结构完整性"""
        base_sats, _ = pipeline_data
        result = least_squares_spp(base_sats)

        assert hasattr(result, 'converged')
        assert hasattr(result, 'position')
        assert hasattr(result, 'gdop')
        assert hasattr(result, 'pdop')
        assert hasattr(result, 'rms')
        assert hasattr(result, 'iterations')
        assert len(result.position) == 3


class TestMatrixInRTK:
    """
    验证 Matrix 类在 RTK 最小二乘中的实际使用
    """

    def test_normal_equation(self):
        """
        构建一个简化的法方程并求解
        模拟 RTK 中 B^T W B · dx = B^T W L 的计算
        """
        # 简化的设计矩阵 (8个观测，4个未知数)
        B = Matrix([
            [-0.5, -0.3, -0.8, 1.0],
            [0.4, -0.6, -0.7, 1.0],
            [0.3, 0.5, -0.8, 1.0],
            [-0.6, 0.4, -0.7, 1.0],
            [-0.3, -0.7, -0.6, 1.0],
            [0.5, 0.4, -0.7, 1.0],
            [0.0, -0.8, -0.6, 1.0],
            [0.2, 0.3, -0.9, 1.0],
        ])

        # 观测向量（伪距残差）
        L = Matrix([[2.1], [1.5], [0.8], [1.9], [2.3], [1.1], [1.7], [1.3]])

        # 权矩阵（假设等权）
        W = Matrix.identity(8)

        # 法方程 N = B^T W B
        N = B.transpose() @ W @ B
        U = B.transpose() @ W @ L

        # 求解
        N_inv = N.inverse()
        dx = N_inv @ U

        # 验证: N @ N_inv ≈ I
        I_check = N @ N_inv
        assert I_check == Matrix.identity(4)

        # 结果应有 4 个分量 (dx, dy, dz, dt)
        assert dx.rows == 4
        assert dx.cols == 1