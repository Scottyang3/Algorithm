#include "path_decision_task.h"

void path_decision_task::path_decision_dynamic_planning(std::vector<double> obs_s, 
                                                        std::vector<double> obs_l, 
                                                        std::vector<double> plan_start_s,
                                                        std::vector<double> plan_start_l,
                                                        std::vector<double> plan_start_dl,
                                                        std::vector<double> plan_start_ddl,
                                                        std::vector<double>*dp_path_s, 
                                                        std::vector<double>*dp_path_l){
    // 该函数为路径决策算法动态规划的主函数
    // 输入：obs s l set 筛选后的障碍物sl信息
    //      plan_start s l dl ddl 规划起点信息
    //      w_cost_obs,w_cost_smooth_dl,w_cost_smooth_ddl,w_cost_smooth_dddl,w_cost_ref动态规划代价权重
    //      row col 动态规划采样点的行数和列数(row必须是奇数，col必须是6)
    //      sample_s,sample_l 采样的s l 长度
    // 输出：dp_path_l,dp_path_s 动态规划的路径sl,此路径不包含规划起点  
}