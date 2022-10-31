#pragma ones
#include <iostream>
#include <cmath>
#include "Cartesian2Frenet.h"
#include "Referenceline.h"
#include "eigen3/Eigen/Eigen"
#include "struct_common.h"

using Matrix = Eigen::MatrixXd;

class Obstacle
{
private:
    double NAN_d = Info::quiet_NaN();
public:
    void obs_filter(double host_x, 
                    double host_y, 
                    double host_heading, 
                    std::vector<obstacle_info> per_obstacle,
                    std::vector<obstacle_info> *obstacle_filter);
    void obs_staticORdynamic(std::vector<obstacle_info> per_obstacle_filter, 
                             std::vector<obstacle_info> *static_obstacle, 
                             std::vector<obstacle_info> *dynamic_obstacle);
    void Calc_static_sl(std::vector<xy_points_info> planning_path, 
                        std::vector<obstacle_info> static_obstacle,
                        std::vector<double> *static_s,
                        std::vector<double> *static_l);
};

