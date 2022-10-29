#pragma ones
#include <iostream>
#include <cmath>
#include "eigen3/Eigen/Eigen"
#include "struct_common.h"

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

void path_decision_task::path_decision_dynamic_planning(std::vector<double> obs_s, 
                                                        std::vector<double> obs_l, 
                                                        std::vector<double> plan_start_s,
                                                        std::vector<double> plan_start_l,
                                                        std::vector<double> plan_start_dl,
                                                        std::vector<double> plan_start_ddl,
                                                        std::vector<double>*dp_path_s, 
                                                        std::vector<double>*dp_path_l){
    // �ú���Ϊ·�������㷨��̬�滮��������
    // ���룺obs s l set ɸѡ����ϰ���sl��Ϣ
    //      plan_start s l dl ddl �滮�����Ϣ
    //      w_cost_obs,w_cost_smooth_dl,w_cost_smooth_ddl,w_cost_smooth_dddl,w_cost_ref��̬�滮����Ȩ��
    //      row col ��̬�滮�����������������(row������������col������6)
    //      sample_s,sample_l ������s l ����
    // �����dp_path_l,dp_path_s ��̬�滮��·��sl,��·���������滮���  
}