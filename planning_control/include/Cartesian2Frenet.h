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
    // �ú��������index��s��ת����ϵ��index2s��ʾ��path_x,path_y����ɢ��ı��Ϊiʱ����Ӧ�Ļ���Ϊindex2s(i)
    // ���� path_x path_y ��ת������ɢ��ļ���
    //      origin_x,y frenetԭ������������ϵ�µ�����
    //      origin_match_point_index ԭ���ƥ���ı��
    // path��ĸ���
    // �����ʼ��
    int n=180;
    index2s->resize(n);
    // ���ȼ�����path���Ϊ����ԭ���index2s
    for (size_t i = 1; i < n; i++)
    {
        index2s->at(i)=std::sqrt(std::pow((path[i].x-path[i-1].x),2)+std::pow((path[i].y-path[i-1].y),2)+index2s->at(i-1));
    }
    // �ټ����Թ켣��㵽frenet_path������ԭ��Ļ�������Ϊs0������index2s - s0 �������յĽ��
    // ����s0
    // s_temp frenetԭ���ƥ���Ļ���
    double s_temp;
    s_temp=index2s->at(origin_match_point_index);
    // �ж�ԭ����ƥ����ǰ�滹�Ǻ���
    // ��������match_point_to_origin 
    //        match_point_to_match_point_next
    std::vector<double>vector_match_2_origin,vector_match_2_match_next;
    //origin_pathͶӰ��ֻ��һ��������ֻ��[0]
    vector_match_2_origin[0]=origin_path.x-path[origin_match_point_index].x;
    vector_match_2_origin[1]=origin_path.y-path[origin_match_point_index].y;
    vector_match_2_match_next[0]=path[origin_match_point_index+1].x-path[origin_match_point_index].x;
    vector_match_2_match_next[1]=path[origin_match_point_index+1].y-path[origin_match_point_index].y;

    double s0;
    if (vector_match_2_origin[0]*vector_match_2_match_next[0]+vector_match_2_origin[1]*vector_match_2_match_next[1]>0)
    {
        //����ԭ����ƥ����ǰ��
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
    //�ú��������㵱ָ��index2s��ӳ���ϵ�󣬼����proj_x,proj_y�Ļ���
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
    // �ú�����������������ϵ�µ�x_set��y_set�ϵĵ���frenet_path�µ�����s l
    // ���� x_set,y_set ������ת���ĵ�
    //      frenet_path_x,frenet_path_y   frenet������
    //      proj_x,y,heading,kappa,proj_match_point_index ������ת���ĵ��ͶӰ�����Ϣ
    //      index2s   frenet_path��index��s��ת����
    // ���ڲ�֪���ж��ٸ�����Ҫ������ת����������Ҫ������
    int n=128;//��ദ���128����
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
        //����s
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
    //�ú���������frenet����ϵ�µ�s_dot, l_dot, dl/ds
    int n=128;
    //�����ʼ��
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
        // ��������cartesian��frenet��ת��Ҫ���򵥣�����Ҳ��ȱ�㣬���������������ٶȼ��ٶ�
        // l' = l_dot/s_dot �������s_dot = 0 �˷�����ʧЧ��
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
    // ���ڲ�֪���ж��ٸ�����Ҫ������ת������һ�����ֵ������
    int n=128;
    // �����ʼ��
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
        // ��������cartesian��frenet��ת��Ҫ���򵥣�����Ҳ��ȱ�㣬���������������ٶȼ��ٶ�
        // l' = l_dot/s_dot �������s_dot = 0 �˷�����ʧЧ��
        if (std::abs(s_dot_set[i])<1e-6)
        {
            ddl_set->at(i)=0;
        }
        else{
            ddl_set->at(i)=(l_dot2_set->at(i)-dl_set[i]*s_dot2_set->at(i))/std::pow(s_dot_set[i],2);
        }
    }
}