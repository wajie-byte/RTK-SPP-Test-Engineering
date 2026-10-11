"""矩阵运算模块 - C++ Mat 类的 Python 等价实现

对应项目中的 MATDLL/Mat.cpp，提供 SPP/RTK 所需的基础线性代数运算：
- 矩阵加减乘
- 转置
- 求逆（高斯-约当消元法）
- 行列式
"""


class Matrix:
    """二维矩阵，内部存储为 [row][col] 嵌套列表"""

    def __init__(self, data):
        """
        data: list of lists, e.g. [[1,2,3],[4,5,6]]
        """
        self.data = data
        self.rows = len(data)
        self.cols = len(data[0]) if data else 0

    # ========== 工厂方法 ==========

    @classmethod
    def zeros(cls, rows, cols):
        """创建全零矩阵"""
        return cls([[0.0] * cols for _ in range(rows)])

    @classmethod
    def identity(cls, n):
        """创建 n×n 单位矩阵"""
        m = cls.zeros(n, n)
        for i in range(n):
            m.data[i][i] = 1.0
        return m

    # ========== 基本运算 ==========

    def transpose(self):
        """矩阵转置 A^T"""
        result = [[self.data[j][i] for j in range(self.rows)]
                  for i in range(self.cols)]
        return Matrix(result)

    def __add__(self, other):
        """矩阵加法 A + B"""
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("矩阵维度不匹配，无法相加")
        result = [[self.data[i][j] + other.data[i][j]
                   for j in range(self.cols)]
                  for i in range(self.rows)]
        return Matrix(result)

    def __sub__(self, other):
        """矩阵减法 A - B"""
        if self.rows != other.rows or self.cols != other.cols:
            raise ValueError("矩阵维度不匹配，无法相减")
        result = [[self.data[i][j] - other.data[i][j]
                   for j in range(self.cols)]
                  for i in range(self.rows)]
        return Matrix(result)

    def __matmul__(self, other):
        """矩阵乘法 A @ B"""
        if self.cols != other.rows:
            raise ValueError(
                f"维度不匹配: ({self.rows},{self.cols}) @ ({other.rows},{other.cols})"
            )
        result = Matrix.zeros(self.rows, other.cols)
        for i in range(self.rows):
            for j in range(other.cols):
                total = 0.0
                for k in range(self.cols):
                    total += self.data[i][k] * other.data[k][j]
                result.data[i][j] = total
        return result

    def __eq__(self, other):
        """判断两个矩阵是否相等（误差容限 1e-12）"""
        if not isinstance(other, Matrix):
            return False
        if self.rows != other.rows or self.cols != other.cols:
            return False
        for i in range(self.rows):
            for j in range(self.cols):
                if abs(self.data[i][j] - other.data[i][j]) > 1e-12:
                    return False
        return True

    # ========== 高级运算 ==========

    def inverse(self):
        """高斯-约当消元法求逆矩阵"""
        if self.rows != self.cols:
            raise ValueError("只有方阵可以求逆")
        n = self.rows

        # 增广矩阵 [A | I]
        aug = [row[:] + [1.0 if i == j else 0.0 for j in range(n)]
               for i, row in enumerate(self.data)]

        for col in range(n):
            # 选主元
            pivot_row = max(range(col, n),
                            key=lambda r: abs(aug[r][col]))
            if abs(aug[pivot_row][col]) < 1e-12:
                raise ValueError("矩阵奇异，无法求逆")
            aug[col], aug[pivot_row] = aug[pivot_row], aug[col]

            # 主元归一化
            pivot = aug[col][col]
            for j in range(2 * n):
                aug[col][j] /= pivot

            # 消元其他行
            for i in range(n):
                if i != col:
                    factor = aug[i][col]
                    for j in range(2 * n):
                        aug[i][j] -= factor * aug[col][j]

        return Matrix([row[n:] for row in aug])

    def determinant(self):
        """LU 分解法求行列式"""
        if self.rows != self.cols:
            raise ValueError("只有方阵可以求行列式")
        n = self.rows
        # 复制一份防止修改原矩阵
        a = [row[:] for row in self.data]
        det = 1.0

        for i in range(n):
            # 选主元
            pivot_row = max(range(i, n), key=lambda r: abs(a[r][i]))
            if abs(a[pivot_row][i]) < 1e-12:
                return 0.0  # 奇异矩阵行列式为 0
            if pivot_row != i:
                a[i], a[pivot_row] = a[pivot_row], a[i]
                det = -det

            det *= a[i][i]
            for j in range(i + 1, n):
                factor = a[j][i] / a[i][i]
                for k in range(i + 1, n):
                    a[j][k] -= factor * a[i][k]

        return det

    def __repr__(self):
        lines = [", ".join(f"{v:10.6f}" for v in row) for row in self.data]
        return "[\n " + "\n ".join(lines) + "\n]"