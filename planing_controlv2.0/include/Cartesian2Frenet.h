#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include "struct_common.h"

const double Inf =0x3f3f3f3f;
class Cartesian2Frenet
{
private:
    /* data */
public:
    bool fcn_index2s(std::vector<xy_points_info> path, 
                     xy_points_info origin_path, 
                     int origin_match_point_index,
                     std::vector<double> *index2s);
    bool Calc_SL(double x_set, double y_set, 
                 std::vector<xy_points_info> frenet_path, 
                 std::vector<proj_points_info> proj_points, 
                 std::vector<double> index2s,
                 std::vector<double> *s_set,
                 std::vector<double> *l_set);
    bool Calc_sl_dot(std::vector<double> l_set,
                     double plan_start_vx,
                     double plan_start_vy,
                     double proj_heading,
                     double proj_kappa,
                     std::vector<double> *s_dot_set,
                     std::vector<double> *l_dot_set,
                     std::vector<double> *dl_set);
    bool Calc_sl_dot2(double plan_start_ax,
                      double plan_start_ay,
                      double proj_heading,
                      double proj_kappa,
                      std::vector<double> l_set,
                      std::vector<double> s_dot_set,
                      std::vector<double> dl_set,
                      std::vector<double> *s_dot2_set,
                      std::vector<double> *l_dot2_set,
                      std::vector<double> *ddl_set);
};

