#ifndef MAT_H
#define MAT_H
#include <stdexcept>
#include <vector>
#include <iostream>
#include <iomanip>

struct Mat
{
	size_t row;//行
	size_t col;//列
	std::vector<double>mat;
	// 默认构造函数 - 创建空矩阵
	Mat() = default;

	// 带参数的构造函数
	
	Mat(size_t Row, size_t Col) : row(Row), col(Col), mat(Row* Col, 0.0) {}
	// 复制构造函数
	Mat(const Mat& other) = default;

	// 移动构造函数
	Mat(Mat&& other) noexcept = default;

	// 复制赋值运算符
	Mat& operator=(const Mat& other) = default;

	// 移动赋值运算符
	Mat& operator=(Mat&& other) noexcept = default;

	// 析构函数
	~Mat() = default;
	// 重新设置矩阵大小
	void resize(size_t newRow, size_t newCol) 
	{
		row = newRow;
		col = newCol;
		mat.resize(newRow * newCol);
		std::fill(mat.begin(), mat.end(), 0.0);
	}

	// 检查矩阵是否为空
	bool empty() const { return row == 0 || col == 0; }
	
	// 静态成员函数：生成单位矩阵
	static Mat Identity(size_t size);
	// 访问元素（可修改）
	double& operator()(size_t i, size_t j);
	// 访问元素（const 版本，不可修改）
	const double& operator()(size_t i, size_t j) const;
	// 矩阵转置
	Mat transpose() const;
	// 标量乘法
	Mat& operator*=(double scalar);
	Mat operator*(double scalar) const;
	// 矩阵求逆
	Mat inverse() const;
	// 矩阵行列式 
	double determinant() const;
	// 友元函数：输出运算符
	friend std::ostream& operator<<(std::ostream& os, const Mat& m);
	// 矩阵加法
	Mat operator+(const Mat& other) const;
	Mat& operator+=(const Mat& other);
	// 矩阵减法
	Mat operator-(const Mat& other) const;
	Mat& operator-=(const Mat& other);
};
// 矩阵乘法
Mat operator*(const Mat& A, const Mat& B);
// 标量左乘
Mat operator*(double scalar, const Mat& m);
//
Mat RotateZ(double angle);
//
Mat RotateX(double angle);
#endif