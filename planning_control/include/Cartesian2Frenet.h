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


bool Cartesian2Frenet::fcn_index2s(std::vector<xy_points_info> path, 
                                   xy_points_info origin_path, 
                                   int origin_match_point_index,
                                   std::vector<double> *index2s){
    // 该函数将输出index与s的转换关系，index2s表示当path_x,path_y的离散点的编号为i时，对应的弧长为index2s(i)
    // 输入 path_x path_y 待转化的离散点的集合
    //      origin_x,y frenet原点在世界坐标系下的坐标
    //      origin_match_point_index 原点的匹配点的编号
    // path点的个数
    // 输出初始化
    int n=180;
    index2s->resize(n);
    // 首先计算以path起点为坐标原点的index2s
    for (size_t i = 1; i < n; i++)
    {
        index2s->at(i)=std::sqrt(std::pow((path[i].x-path[i-1].x),2)+std::pow((path[i].y-path[i-1].y),2)+index2s->at(i-1));
    }
    // 再计算以轨迹起点到frenet_path的坐标原点的弧长，记为s0，再用index2s - s0 就是最终的结果
    // 计算s0
    // s_temp frenet原点的匹配点的弧长
    double s_temp;
    s_temp=index2s->at(origin_match_point_index);
    // 判断原点在匹配点的前面还是后面
    // 两个向量match_point_to_origin 
    //        match_point_to_match_point_next
    std::vector<double>vector_match_2_origin,vector_match_2_match_next;
    //origin_path投影点只有一个，所以只用[0]
    vector_match_2_origin[0]=origin_path.x-path[origin_match_point_index].x;
    vector_match_2_origin[1]=origin_path.y-path[origin_match_point_index].y;
    vector_match_2_match_next[0]=path[origin_match_point_index+1].x-path[origin_match_point_index].x;
    vector_match_2_match_next[1]=path[origin_match_point_index+1].y-path[origin_match_point_index].y;

    double s0;
    if (vector_match_2_origin[0]*vector_match_2_match_next[0]+vector_match_2_origin[1]*vector_match_2_match_next[1]>0)
    {
        //坐标原点在匹配点的前面
        s0=s_temp + std::sqrt(vector_match_2_origin[0]*vector_match_2_origin[0]+vector_match_2_origin[1]*vector_match_2_origin[1]);
    }
    else{
        s0=s_temp - std::sqrt(vector_match_2_origin[0]*vector_match_2_origin[0]+vector_match_2_origin[1]*vector_match_2_origin[1]);
    }
    for (size_t i = 0; i < n; i++)
    {
        index2s->at(i)=index2s->at(i)-s0;
    }
}



double CalcSFromIndex2S(std::vector<double> index2s, std::vector<xy_points_info> frenet_path, double proj_x, double proj_y,int proj_match_point_index){
    //该函数将计算当指定index2s的映射关系后，计算点proj_x,proj_y的弧长
    std::vector<double> vector_1(2);
    vector_1[0]=proj_x-frenet_path[proj_match_point_index].x;
    vector_1[1]=proj_y-frenet_path[proj_match_point_index].y;
    std::vector<double> vector_2(2);
    if (proj_match_point_index < frenet_path.size())
    {
        vector_2[0]=frenet_path[proj_match_point_index+1].x-frenet_path[proj_match_point_index].x;
        vector_2[1]=frenet_path[proj_match_point_index+1].y-frenet_path[proj_match_point_index].y;
    }
    else{
        vector_2[0]=frenet_path[proj_match_point_index].x-frenet_path[proj_match_point_index].x;
        vector_2[1]=frenet_path[proj_match_point_index].x-frenet_path[proj_match_point_index].x;
    }
    double s;
    if (vector_1[0]*vector_1[0]+vector_1[1]*vector_1[1] > 0)
    {
        s = index2s[proj_match_point_index] + std::sqrt(vector_1[0]*vector_1[0]+vector_1[1]*vector_1[1]);
    }
    else{
        s = index2s[proj_match_point_index] + std::sqrt(vector_1[0]*vector_1[0]+vector_1[1]*vector_1[1]);
    }
    return s;
}

bool Cartesian2Frenet::Calc_SL(double x_set, double y_set, 
                               std::vector<xy_points_info> frenet_path, 
                               std::vector<proj_points_info> proj_points, 
                               std::vector<double> index2s,
                               std::vector<double> *s_set,
                               std::vector<double> *l_set){
    // 该函数将计算世界坐标系下的x_set，y_set上的点在frenet_path下的坐标s l
    // 输入 x_set,y_set 待坐标转换的点
    //      frenet_path_x,frenet_path_y   frenet坐标轴
    //      proj_x,y,heading,kappa,proj_match_point_index 待坐标转换的点的投影点的信息
    //      index2s   frenet_path的index与s的转换表
    // 由于不知道有多少个点需要做坐标转换，所以需要做缓冲
    int n=128;//最多处理的128个点
    // std::vector<double> s_set(n,Inf);
    // std::vector<double> l_set(n,Inf);
    s_set->resize(n,Inf);
    l_set->resize(n,Inf);
    // for (size_t i = 0; i < s_set->size(); i++)
    // {
    //     std::cout<<"s_set"<<s_set->at(i)<<std::endl;
    // }
    for (size_t i = 0; i < s_set->size(); i++)
    {
        if (s_set->at(i)==Inf)
        {
            break;
        }
        //计算s
        s_set->at(i) = CalcSFromIndex2S(index2s, frenet_path, proj_points[i].proj_point_x, proj_points[i].proj_point_y, proj_points[i].proj_match_index);
        l_set->at(i) = -(x_set-proj_points[i].proj_point_x)*sin(proj_points[i].proj_heading)+(y_set-proj_points[i].proj_point_y)*cos(proj_points[i].proj_heading);
    }    
}

bool Cartesian2Frenet::Calc_sl_dot(std::vector<double> l_set,
                                   double plan_start_vx,
                                   double plan_start_vy,
                                   double proj_heading,
                                   double proj_kappa,
                                   std::vector<double> *s_dot_set,
                                   std::vector<double> *l_dot_set,
                                   std::vector<double> *dl_set){
    //该函数将计算frenet坐标系下的s_dot, l_dot, dl/ds
    int n=128;
    //输出初始化
    s_dot_set->resize(n,Inf);
    l_dot_set->resize(n,Inf);
    dl_set->resize(n,Inf);
    for (size_t i = 0; i < l_set.size(); i++)
    {
        if (l_set[i]==Inf)
        {
            break;
        }
        l_dot_set->at(i)=-plan_start_vx*sin(proj_heading)+plan_start_vy*cos(proj_heading);
        s_dot_set->at(i)=plan_start_vx*cos(proj_heading)+plan_start_vy*sin(proj_heading);
        // 向量法做cartesian与frenet的转换要更简单，但是也有缺点，向量法必须依赖速度加速度
        // l' = l_dot/s_dot 但是如果s_dot = 0 此方法就失效了
        if (std::abs(s_dot_set->at(i))<1e-6)
        {
            dl_set->at(i)=0;
        }
        else{
            dl_set->at(i)=l_dot_set->at(i)/s_dot_set->at(i);
        }
    }
}

bool Cartesian2Frenet::Calc_sl_dot2(double plan_start_ax,
                                    double plan_start_ay,
                                    double proj_heading,
                                    double proj_kappa,
                                    std::vector<double> l_set,
                                    std::vector<double> s_dot_set,
                                    std::vector<double> dl_set,
                                    std::vector<double> *s_dot2_set,
                                    std::vector<double> *l_dot2_set,
                                    std::vector<double> *ddl_set){
    // 由于不知道有多少个点需要做坐标转换，设一个最大值做缓冲
    int n=128;
    // 输出初始化
    s_dot2_set->resize(n,Inf);
    l_dot2_set->resize(n,Inf);
    ddl_set->resize(n,Inf);
    for (size_t i = 0; i < l_set.size(); i++)
    {
        if (l_set[i]==Inf)
        {
            break;
        }
        l_dot2_set->at(i)=-plan_start_ax*sin(proj_heading)+plan_start_ay*cos(proj_heading)-proj_kappa*(1-proj_kappa*l_set[i]*s_dot_set[i]*s_dot_set[i]);
        s_dot2_set->at(i)=(1/(1-proj_kappa*l_set[i]))*(plan_start_ax*cos(proj_heading)+plan_start_ay*sin(proj_heading)+2*proj_kappa*dl_set[i]*s_dot_set[i]*s_dot_set[i]);
        // 向量法做cartesian与frenet的转换要更简单，但是也有缺点，向量法必须依赖速度加速度
        // l' = l_dot/s_dot 但是如果s_dot = 0 此方法就失效了
        if (std::abs(s_dot_set[i])<1e-6)
        {
            ddl_set->at(i)=0;
        }
        else{
            ddl_set->at(i)=(l_dot2_set->at(i)-dl_set[i]*s_dot2_set->at(i))/std::pow(s_dot_set[i],2);
        }
    }
}