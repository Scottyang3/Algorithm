#pragma once
#include <iostream>
#include <cmath>
#include <vector>
#include "struct_common.h"

class calc_plan_trajectory
{
private:
    int is_first_run = 0;
    double x_cur=0.0;
    double y_cur=0.0;
    double heading_cur=0.0;
    double kappa_cur=0.0;
    double vx_cur=0.0;
    double vy_cur=0.0;
    double ax_cur=0.0;
    double ay_cur=0.0;
    // 规划周期
    double dt=0.1;
public:
    bool calc_plan_stitchtrajectory(std::vector<planning_trajectory> &pre_planning_trajectory_info, 
                                    location_info &ego_info, 
                                    double current_time,
                                    std::vector<stitch_trajectory>* stitch_points,
                                    plan_start_info *plan_start_points);
};
