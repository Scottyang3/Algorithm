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


void Obstacle::obs_filter(double host_x, 
                          double host_y, 
                          double host_heading, 
                          std::vector<obstacle_info> per_obstacle,
                          std::vector<obstacle_info> *obstacle_filter){
    //------------------------------------------------------------------------------------------//
    // 该函数将筛选障碍物，纵向[-10,200] 横向[-20,20]的障碍物才会被考虑
    // 该函数只是一种单车道的临时办法，考虑到多车道情况，即使障碍物距离较远也应该考虑
    // EM Planner完全体是多车道并行计算的，每个车道都生成参考线然后并行计算出多条轨迹，再选择最优的轨迹作为输出
    // 传感器最多可以识别32个障碍物
    //------------------------------------------------------------------------------------------//
    int n = 32;
    // 输出初始化
    obstacle_filter->resize(n);
    for (size_t i = 0; i < n; i++)
    {
        obstacle_filter->at(i).obs_x = NAN_d;
        obstacle_filter->at(i).obs_y = NAN_d;
        obstacle_filter->at(i).obs_heading = NAN_d;
        obstacle_filter->at(i).obs_velocity = NAN_d;
    }
    int count = 0;
    for (size_t i = 0; i < per_obstacle.size(); i++)
    {
        if (std::isnan(per_obstacle[i].obs_x))
        {
            break;
        }
        // 自车的heading的方向向量与法向量,距离向量
        Eigen::MatrixXd tor,nor,vector_obs;
        tor = Matrix::Zero(1,2);
        nor = Matrix::Zero(1,2);
        vector_obs = Matrix::Zero(1,2);
        tor(0,0) = cos(host_heading);
        tor(0,1) = sin(host_heading);
        nor(0,0) = -sin(host_heading);
        nor(0,1) = cos(host_heading);
        vector_obs(0,0) = per_obstacle[i].obs_x - host_x;
        vector_obs(0,1) = per_obstacle[i].obs_y - host_y;
        // 障碍物纵向距离
        double lon_distance = (vector_obs.transpose()* tor)(0,0);
        // 障碍物横向距离
        double lat_distance = (vector_obs.transpose()* nor)(0,0);

        if ((lon_distance < 200) && (lon_distance > -10) && (lat_distance > -20) && (lat_distance < 20))
        {
            obstacle_filter->at(count).obs_x = per_obstacle[i].obs_x;
            obstacle_filter->at(count).obs_y = per_obstacle[i].obs_y;
            obstacle_filter->at(count).obs_velocity = per_obstacle[i].obs_velocity;
            obstacle_filter->at(count).obs_heading = per_obstacle[i].obs_heading;
            count += 1;
        }
    }
}

void Obstacle::obs_staticORdynamic(std::vector<obstacle_info> per_obstacle_filter, 
                                   std::vector<obstacle_info> *static_obstacle, 
                                   std::vector<obstacle_info> *dynamic_obstacle){
    //------------------------------------------//
    // 该函数将分类静态障碍物和动态障碍物
    //------------------------------------------//
    int n = 32;
    // 输出初始化
    static_obstacle->resize(n);
    dynamic_obstacle->resize(n);
    for (size_t i = 0; i < n; i++)
    {
        static_obstacle->at(i).obs_x = NAN_d;
        static_obstacle->at(i).obs_y = NAN_d;
        static_obstacle->at(i).obs_heading = NAN_d;
        static_obstacle->at(i).obs_velocity = NAN_d;
        dynamic_obstacle->at(i).obs_x = NAN_d;
        dynamic_obstacle->at(i).obs_y = NAN_d;
        dynamic_obstacle->at(i).obs_heading = NAN_d;
        dynamic_obstacle->at(i).obs_velocity = NAN_d;
    }
    int count_static = 0;
    int count_dynamic = 0;
    for (size_t i = 0; i < per_obstacle_filter.size(); i++)
    {
        if (std::isnan(per_obstacle_filter[i].obs_x))
        {
            break;
        }
        if (std::abs(per_obstacle_filter[i].obs_velocity)<0.1)
        {
            static_obstacle->at(i).obs_x = per_obstacle_filter[i].obs_x;
            static_obstacle->at(i).obs_y = per_obstacle_filter[i].obs_x;
            static_obstacle->at(i).obs_heading = per_obstacle_filter[i].obs_heading;
            static_obstacle->at(i).obs_velocity = per_obstacle_filter[i].obs_velocity;
            count_static += 1;
        }
        else{
            dynamic_obstacle->at(i).obs_x = per_obstacle_filter[i].obs_x;
            dynamic_obstacle->at(i).obs_y = per_obstacle_filter[i].obs_x;
            dynamic_obstacle->at(i).obs_heading = per_obstacle_filter[i].obs_heading;
            dynamic_obstacle->at(i).obs_velocity = per_obstacle_filter[i].obs_velocity;
            count_dynamic += 1;
        }
    }
}

void Obstacle::Calc_static_sl(std::vector<xy_points_info> planning_path, 
                              std::vector<obstacle_info> static_obstacle,
                              std::vector<double> *static_s,
                              std::vector<double> *static_l){
    std::vector<proj_points_info> proj_obs_point;
    ReferenceLine obs_calc_proj;
    Cartesian2Frenet obs_cartofre;
    xy_points_info origin_obs_path;
    std::vector<double> index2s;
    for (size_t i = 0; i < static_obstacle.size(); i++)
    {
        obs_calc_proj.Clac_proj_points_info(static_obstacle[i].obs_x, static_obstacle[i].obs_y, planning_path, &proj_obs_point);
        for (size_t i = 0; i < proj_obs_point.size(); i++)
        {
            origin_obs_path.x=proj_obs_point[i].proj_point_x;
            origin_obs_path.y=proj_obs_point[i].proj_point_y;
        }
        obs_cartofre.fcn_index2s(planning_path, origin_obs_path, proj_obs_point[i].proj_match_index, &index2s);
        obs_cartofre.Calc_SL(static_obstacle[i].obs_x, static_obstacle[i].obs_y,planning_path,proj_obs_point, index2s, static_s, static_l);
    }
}