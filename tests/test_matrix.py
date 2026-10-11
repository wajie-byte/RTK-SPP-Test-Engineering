"""TC_MAT系列: 矩阵运算单元测试

测试 src/matrix.py 的 Matrix 类，对应 C++ 项目中的 MATDLL/Mat
"""
import pytest
from src.matrix import Matrix


class TestMatrixTranspose:
    """TC_MAT_001: 矩阵转置"""

    def test_transpose_3x2(self):
        """3行2列 转置后 应为 2行3列"""
        m = Matrix([[1, 2], [3, 4], [5, 6]])
        mt = m.transpose()
        assert mt.rows == 2
        assert mt.cols == 3
        assert mt.data == [[1, 3, 5], [2, 4, 6]]

    def test_transpose_identity(self):
        """单位阵转置后等于自身"""
        m = Matrix.identity(3)
        assert m.transpose() == m

    def test_transpose_twice(self):
        """转置两次等于原矩阵"""
        m = Matrix([[1, 2, 3], [4, 5, 6]])
        assert m.transpose().transpose() == m


class TestMatrixMultiply:
    """TC_MAT_002: 矩阵乘法"""

    def test_multiply_2x3_by_3x2(self):
        """2×3 乘 3×2，结果应为 2×2"""
        a = Matrix([[1, 2, 3], [4, 5, 6]])
        b = Matrix([[7, 8], [9, 10], [11, 12]])
        c = a @ b
        assert c.rows == 2
        assert c.cols == 2
        # 手算验证: c[0][0] = 1*7+2*9+3*11 = 58
        assert c.data[0][0] == pytest.approx(58)
        assert c.data[0][1] == pytest.approx(64)
        assert c.data[1][0] == pytest.approx(139)
        assert c.data[1][1] == pytest.approx(154)

    def test_multiply_by_identity(self):
        """A × I = A，I × A = A"""
        a = Matrix([[3, 1, 4], [1, 5, 9]])
        i2 = Matrix.identity(2)
        i3 = Matrix.identity(3)
        assert i2 @ a == a
        assert a @ i3 == a

    def test_multiply_dimension_mismatch(self):
        """维度不匹配应抛出 ValueError"""
        a = Matrix([[1, 2, 3]])  # 1×3
        b = Matrix([[1], [2]])   # 2×1
        with pytest.raises(ValueError):
            a @ b  # 3 != 2，不能乘


class TestMatrixInverse:
    """TC_MAT_003: 矩阵求逆"""

    def test_inverse_of_2x2(self):
        """验证 A × A⁻¹ = I"""
        a = Matrix([[4, 7], [2, 6]])
        inv = a.inverse()
        assert a @ inv == Matrix.identity(2)
        assert inv @ a == Matrix.identity(2)

    def test_inverse_of_identity(self):
        """单位阵的逆等于自身"""
        i = Matrix.identity(4)
        assert i.inverse() == i

    def test_inverse_singular_raises(self):
        """奇异矩阵不可逆，应抛 ValueError"""
        singular = Matrix([[2, 4], [1, 2]])  # 第二行是第一行的 0.5 倍
        with pytest.raises(ValueError):
            singular.inverse()


class TestMatrixDeterminant:
    """TC_MAT_005: 行列式"""

    def test_determinant_2x2(self):
        """[[a,b],[c,d]] 行列式 = ad - bc"""
        m = Matrix([[3, 8], [4, 6]])
        det = m.determinant()
        # ad - bc = 3×6 - 8×4 = -14
        assert det == pytest.approx(-14.0)

    def test_determinant_identity(self):
        """单位阵行列式为 1"""
        assert Matrix.identity(5).determinant() == pytest.approx(1.0)

    def test_determinant_singular(self):
        """奇异矩阵行列式为 0"""
        m = Matrix([[2, 4], [1, 2]])
        assert m.determinant() == pytest.approx(0.0)


class TestMatrixAddSub:
    """矩阵加减法"""

    def test_add(self):
        a = Matrix([[1, 2], [3, 4]])
        b = Matrix([[5, 6], [7, 8]])
        assert (a + b).data == [[6, 8], [10, 12]]

    def test_sub(self):
        a = Matrix([[5, 6], [7, 8]])
        b = Matrix([[1, 2], [3, 4]])
        assert (a - b).data == [[4, 4], [4, 4]]

    def test_add_dimension_mismatch(self):
        with pytest.raises(ValueError):
            Matrix([[1, 2]]) + Matrix([[1], [2]])


class TestMatrixFactory:
    """TC_MAT_006: 工厂方法"""

    def test_zeros(self):
        z = Matrix.zeros(2, 3)
        assert z.rows == 2 and z.cols == 3
        for row in z.data:
            assert all(v == 0.0 for v in row)

    def test_identity(self):
        i3 = Matrix.identity(3)
        assert i3.data[0] == [1.0, 0.0, 0.0]
        assert i3.data[1] == [0.0, 1.0, 0.0]
        assert i3.data[2] == [0.0, 0.0, 1.0]