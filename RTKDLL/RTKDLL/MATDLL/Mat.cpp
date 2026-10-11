#include"Mat.h"

// 单位矩阵
Mat Mat::Identity(size_t size) 
{
    if (size == 0) 
    {
        throw std::runtime_error("Identity matrix size must be > 0");
    }
    Mat result(size, size);
    for (size_t i = 0; i < size; ++i)
    {
        result(i, i) = 1.0;
    }
    return result;
}
double& Mat::operator()(size_t i, size_t j)
{
    return mat[i * col + j];
}
const double& Mat::operator()(size_t i, size_t j) const 
{
    return mat[i * col + j];
}
// 矩阵转置
Mat Mat::transpose() const 
{
    Mat result(col, row);
    for (size_t i = 0; i < row; ++i) 
    {
        for (size_t j = 0; j < col; ++j) 
        {
            result(j, i) = (*this)(i, j);
        }
    }
    return result;
}

// 标量乘法
Mat& Mat::operator*=(double scalar)
{
    for (double& val : mat) 
    {
        val *= scalar;
    }
    return *this;
}

Mat Mat::operator*(double scalar) const
{
    Mat result = *this;
    result *= scalar;
    return result;
}

// 矩阵求逆
Mat Mat::inverse() const 
{
    if (row != col)
    {
        throw std::runtime_error("Inverse requires a square matrix");
    }

    size_t n = row;
    Mat aug(n, 2 * n);

    // 构造增广矩阵 [A | I]
    for (size_t i = 0; i < n; ++i) 
    {
        for (size_t j = 0; j < n; ++j) 
        {
            aug(i, j) = (*this)(i, j);
        }
        aug(i, i + n) = 1.0;
    }

    // 高斯-约当消元
    for (size_t k = 0; k < n; ++k) 
    {
        // 部分主元选择
        size_t max_row = k;
        double max_val = fabs(aug(k, k));
        for (size_t i = k + 1; i < n; ++i)
        {
            if (fabs(aug(i, k)) > max_val) 
            {
                max_val = fabs(aug(i, k));
                max_row = i;
            }
        }

        if (max_val < 1e-10) {
            throw std::runtime_error("Matrix is singular (non-invertible)");
        }

        if (max_row != k) {
            for (size_t j = k; j < 2 * n; ++j) 
            {
                std::swap(aug(k, j), aug(max_row, j));
            }
        }

        // 归一化主元行
        double pivot = aug(k, k);
        for (size_t j = k; j < 2 * n; ++j) 
        {
            aug(k, j) /= pivot;
        }

        // 消去其他行
        for (size_t i = 0; i < n; ++i) 
        {
            if (i != k) {
                double factor = aug(i, k);
                for (size_t j = k; j < 2 * n; ++j) 
                {
                    aug(i, j) -= factor * aug(k, j);
                }
            }
        }
    }

    // 提取逆矩阵
    Mat inv(n, n);
    for (size_t i = 0; i < n; ++i) 
    {
        for (size_t j = 0; j < n; ++j) 
        {
            inv(i, j) = aug(i, j + n);
        }
    }

    return inv;
}

// 输出运算符
std::ostream& operator<<(std::ostream& os, const Mat& m) 
{
    for (size_t i = 0; i < m.row; ++i) 
    {
        for (size_t j = 0; j < m.col; ++j) 
        {
            os << std::setw(10) << m(i, j);
        }
        os << std::endl;
    }
    return os;
}

// 矩阵乘法
Mat operator*(const Mat& A, const Mat& B) 
{
    if (A.col != B.row)
    {
        throw std::runtime_error("Matrix dimensions mismatch for multiplication");
    }

    Mat C(A.row, B.col);
    for (size_t i = 0; i < A.row; ++i) 
    {
        for (size_t j = 0; j < B.col; ++j) 
        {
            double sum = 0.0;
            for (size_t k = 0; k < A.col; ++k) 
            {
                sum += A(i, k) * B(k, j);
            }
            C(i, j) = sum;
        }
    }
    return C;
}

// 标量左乘
Mat operator*(double scalar, const Mat& m) 
{
    return m * scalar;
}
// 矩阵加法
Mat Mat::operator+(const Mat& other) const 
{
    if (row != other.row || col != other.col) 
    {
        throw std::runtime_error("Matrix dimensions mismatch for addition");
    }

    Mat result(row, col);
    for (size_t i = 0; i < row * col; ++i) 
    {
        result.mat[i] = mat[i] + other.mat[i];
    }
    return result;
}

// 复合加法赋值
Mat& Mat::operator+=(const Mat& other) 
{
    if (row != other.row || col != other.col) 
    {
        throw std::runtime_error("Matrix dimensions mismatch for addition");
    }

    for (size_t i = 0; i < row * col; ++i) 
    {
        mat[i] += other.mat[i];
    }
    return *this;
}
// 矩阵减法
Mat Mat::operator-(const Mat& other) const 
{
    if (row != other.row || col != other.col)
    {
        throw std::runtime_error("Matrix dimensions mismatch for subtraction");
    }

    Mat result(row, col);
    for (size_t i = 0; i < row * col; ++i) 
    {
        result.mat[i] = mat[i] - other.mat[i];
    }
    return result;
}

// 复合减法赋值
Mat& Mat::operator-=(const Mat& other) 
{
    if (row != other.row || col != other.col)
    {
        throw std::runtime_error("Matrix dimensions mismatch for subtraction");
    }

    for (size_t i = 0; i < row * col; ++i) 
    {
        mat[i] -= other.mat[i];
    }
    return *this;
}

// 矩阵行列式计算 - 使用LU分解
double Mat::determinant() const 
{
    // 只有方阵才有行列式
    if (row != col) 
    {
        throw std::invalid_argument("行列式只能计算方阵");
    }

    // 对于1x1矩阵
    if (row == 1) 
    {
        return mat[0];
    }

    // 对于2x2矩阵
    if (row == 2) {
        return mat[0] * mat[3] - mat[1] * mat[2];
    }

    // 对于更大的矩阵，使用LU分解计算行列式
    // 创建矩阵副本
    Mat lu(*this);
    std::vector<size_t> pivot(row);
    for (size_t i = 0; i < row; i++) 
    {
        pivot[i] = i;
    }

    int sign = 1; // 行列式的符号

    // LU分解
    for (size_t k = 0; k < row; k++) 
    {
        // 寻找主元
        double max_val = 0.0;
        size_t max_index = k;
        for (size_t i = k; i < row; i++)
        {
            if (std::fabs(lu(i, k)) > max_val)
            {
                max_val = std::fabs(lu(i, k));
                max_index = i;
            }
        }

        // 如果主元为零，则行列式为零
        if (max_val < 1e-12) 
        {
            return 0.0;
        }

        // 交换行
        if (max_index != k) 
        {
            for (size_t j = 0; j < row; j++) 
            {
                std::swap(lu(k, j), lu(max_index, j));
            }
            std::swap(pivot[k], pivot[max_index]);
            sign = -sign; // 交换行改变行列式符号
        }

        // 消元
        for (size_t i = k + 1; i < row; i++) 
        {
            lu(i, k) = lu(i, k) / lu(k, k);
            for (size_t j = k + 1; j < row; j++)
            {
                lu(i, j) = lu(i, j) - lu(i, k) * lu(k, j);
            }
        }
    }

    // 计算行列式（对角元素的乘积）
    double det = sign;
    for (size_t i = 0; i < row; i++) 
    {
        det *= lu(i, i);
    }

    return det;
}

Mat RotateZ(double angle)
{
    Mat RotateZ(3, 3);
    
    RotateZ(0, 0) = cos(angle);
    RotateZ(0, 1) = sin(angle);
    RotateZ(0, 2) = 0;
    RotateZ(1, 0) = -sin(angle);
    RotateZ(1, 1) = cos(angle);
    RotateZ(1, 2) = 0;
    RotateZ(2, 0) = 0;
    RotateZ(2, 1) = 0;
    RotateZ(2, 2) = 1;
	return RotateZ;
}
Mat RotateX(double angle)
{
    Mat RotateX(3, 3);
    RotateX(0, 0) = 1;
    RotateX(0, 1) = 0;
    RotateX(0, 2) = 0;
    RotateX(1, 0) = 0;
    RotateX(1, 1) = cos(angle);
    RotateX(1, 2) = sin(angle);
    RotateX(2, 0) = 0;
    RotateX(2, 1) = -sin(angle);
    RotateX(2, 2) = cos(angle);
	return RotateX;
}