#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <memory>
#include <algorithm>
#include "include/struct_common.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"

using Matrix = Eigen::MatrixXd;

//Riccati方程返回类型
typedef Eigen::Matrix<double, 1, 4> Matrix1x4;

//--------------------------------------------------------------------------------------------------//
// 控制量突变优化
// 问题描述：在道路曲率变化处的控制量存在突变现象
// 原因：规划路径的曲率不连续，路径点本身曲率就存在突变，没有过渡曲率
// 解决方法 1：在曲率突变处进行插值，保证连接处曲率的一阶导数与二阶导数都连续，在实际路况中大多数的公路设计都是连续曲率。
// 解决方法 2：增大 LQR 控制器中 R 的值，即增大ｕ的惩罚权重，使ｕ变化减缓。
//--------------------------------------------------------------------------------------------------//
// 控制量抖动优化:控制量波形受到饱和约束，前轮转角变化范围为 ±1 rad
// 问题描述：控制量抖动过大，即方向盘不平稳
// 原因：在 err 和 曲率计算模块中，投影点的航向角用匹配点的航向角近似代替，导致计算结果反复不定，修正后重新执行。
//--------------------------------------------------------------------------------------------------//

class lqr_control
{
private:
    Eigen::MatrixXd matrix_A;
    Eigen::MatrixXd matrix_B;
    Eigen::MatrixXd matrix_Q;
    Eigen::MatrixXd matrix_R;
    Eigen::MatrixXd matrix_err_state;
    //R矩阵的数值，越大对控制u的惩罚越大，变化越缓慢
    double R = 30;
    // corner stiffness; front
    double cf = -175016;
    // corner stiffness; rear
    double cr = -130634;
    //mass of the vehicle,总质量
    double mass = 2020;
    // wheelbase    
    double wheelbase = 2.947;
    // distance from front wheel center to COM
    double lf = 1.265;
    // distance from rear wheel center to COM
    double lr = wheelbase-lf;
    // moment of inertia
    double iz = 4095.0;

    //Riccati,误差收敛阈值
    double err_tolerance=0.01;
    //迭代终止次数
    uint max_num_iteration=1500;
    //每隔0.01s进行一次控制
    double ts=0.01; 

public:
    void UpdataState(const location_info &VehicleState);
    Matrix1x4 Cal_Riccati(const Matrix &A, const Matrix &B, const Matrix &Q,
                         const Matrix &R, const double err_tolerance,
                         const uint max_num_iteration);  //计算K矩阵
    LateralControlError Cal_Error(double x, double y, double yaw, double vx, double vy, double dyaw, std::vector<planning_trajectory> &planning_path);
    int QueryNearestPointByPosition(double x, double y, std::vector<planning_trajectory> &planning_path);
    double ComputeControlCommand(const location_info &VehicleState, std::vector<planning_trajectory> &planning_path);
};
