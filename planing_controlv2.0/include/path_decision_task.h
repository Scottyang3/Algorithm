#pragma ones
#include <iostream>
#include <cmath>
#include "eigen3/Eigen/Eigen"
#include "struct_common.h"


//-------------------------------------------------------------------------------//
// 障碍物距离代价:
// g_cost(x)=   0                       x>d1
//              kx+b或10/(x-d2)^2       d2<x<d1
//              +inf                    x<d2
// 在d1范围外表示安全，代价为0。
// 在(d2,d1)范围内有一定危险，随着距离越近代价越大。
// 在d2范围内非常危险，表示有碰撞的危险，代价为无穷大。
//-------------------------------------------------------------------------------//
class path_decision_task
{
private:
    double dp_cost_collision = 1e6;
    double dp_cost_dl = 2000;
    double dp_cost_ddl = 2;
    double dp_cost_dddl = 3;
    double dp_cost_ref = 10;
    double dp_row = 11;
    double dp_col = 4;
    double dp_sample_s = 15;
    double dp_sample_l = 1;
public:
    void path_decision_dynamic_planning(std::vector<double> obs_s, 
                                        std::vector<double> obs_l, 
                                        std::vector<double> plan_start_s,
                                        std::vector<double> plan_start_l,
                                        std::vector<double> plan_start_dl,
                                        std::vector<double> plan_start_ddl,
                                        std::vector<double>*dp_path_s, 
                                        std::vector<double>*dp_path_l);
};
