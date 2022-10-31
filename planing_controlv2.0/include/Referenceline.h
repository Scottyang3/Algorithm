#pragma once
#include <vector>
#include <iostream>
#include <cmath>
#include <string>
#include <struct_common.h>
#include "eigen3/Eigen/Eigen"
#include "eigen3/Eigen/Dense"
#include "osqp/osqp.h"
#include "OsqpEigen/OsqpEigen.h"

using Matrix = Eigen::MatrixXd;

class ReferenceLine
{
public:
    //计算路径点的heading，accumulated_s，kappa，dkappa
	bool Clac_heading_kappa_info(std::vector<xy_points_info>& xy_points_,
                                 std::vector<double>* headings,
                                 std::vector<double>* accumulated_s,
                                 std::vector<double>* kappas,
                                 std::vector<double>* dkappas);
    //计算投影点的信息
    void Clac_proj_points_info(double &x_set,
                               double &y_set,
                               const std::vector<xy_points_info>& xy_points,
                               std::vector<proj_points_info> *proj_points);
    //在全局路径上提取referenceline的未平滑的初值，从匹配点往后取30个点，往前取150个点即可，一共181个点                             
    void intercept_referenceline(int host_match_point_index,
                                 std::vector<xy_points_info>& global_xy_points,
                                 std::vector<xy_points_info>* xy_points);
    //Fem平滑算法
    bool Fem_smoothing(const std::vector<xy_points_info>& xy_points, std::vector<xy_points_info> *fem_smoothing_line);

private:
    //计算投影点部分参数
    int is_first_run=0; //判断是否是首次运行
    int start_search_index=0;   //寻找匹配点开始的索引
    int pre_match_point_index=0;    //上一个周期的匹配点
    double pre_frenet_path_x;   //上一个周期的x
    double pre_frenet_path_y;   //上一个周期的y
    double pre_frenet_path_heading; //上一个周期的heading
    double pre_frenet_path_kappa;   //上一个周期的kappa
    int match_points_index=0;   //匹配点的索引
    int first_index = 0;
    int increase_count=0;
    double distance = 0.0;
    double match_point_x=0;
    double match_point_y=0;
    double match_point_heading=0;
    double match_point_kappa=0;
    //匹配点的方向向量与法向量
    std::vector<double> vector_match_point;
    std::vector<double> vector_match_point_direction;
    std::vector<double> vector_d;
    //声明待投影点的位矢
    std::vector<double> vector_r;

    //参考线平滑部分参数
    double weight_smoothing = 100;    //平滑性权重
    double weight_similarity = 1;  //几何相似性权重
    double weight_Uniformity = 1;  //均匀性权重
    double x_lb = -0.1;   //下界误差
    double x_ub = 0.1;    //上界误差
    // 二次规划形式：
    // min_x J=0.5 * x'Hx + f'x
    // s.t.  l <= Ax <= u
    // 分配QP问题矩阵和向量
    Eigen::SparseMatrix<double> hessian; //H
    Eigen::VectorXd gradient;  //f
    Eigen::SparseMatrix<double> linearMatrix;  //A
    Eigen::VectorXd lowerBound; //l约束下边界
    Eigen::VectorXd upperBound; //u约束上边界

    Eigen::MatrixXd matrix_A1;
    Eigen::MatrixXd matrix_A2;
    Eigen::MatrixXd matrix_A3;
    Eigen::MatrixXd matrix_H;
    Eigen::MatrixXd matrix_f;
    xy_points_info fem_points;//平滑后的点的存储中间变量
};
