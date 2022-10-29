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
    // �ú�����ɸѡ�ϰ������[-10,200] ����[-20,20]���ϰ���Żᱻ����
    // �ú���ֻ��һ�ֵ���������ʱ�취�����ǵ��೵���������ʹ�ϰ�������ԶҲӦ�ÿ���
    // EM Planner��ȫ���Ƕ೵�����м���ģ�ÿ�����������ɲο���Ȼ���м���������켣����ѡ�����ŵĹ켣��Ϊ���
    // ������������ʶ��32���ϰ���
    //------------------------------------------------------------------------------------------//
    int n = 32;
    // �����ʼ��
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
        // �Գ���heading�ķ��������뷨����,��������
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
        // �ϰ����������
        double lon_distance = (vector_obs.transpose()* tor)(0,0);
        // �ϰ���������
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
    // �ú��������ྲ̬�ϰ���Ͷ�̬�ϰ���
    //------------------------------------------//
    int n = 32;
    // �����ʼ��
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