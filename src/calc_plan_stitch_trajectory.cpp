#include "calc_plan_stitch_trajectory.h"

bool calc_plan_trajectory::calc_plan_stitchtrajectory(std::vector<planning_trajectory> &pre_planning_trajectory_info, 
                                                                      location_info &ego_info, 
                                                                      double current_time,
                                                                      std::vector<stitch_trajectory> *stitch_points,
                                                                      plan_start_info *plan_start_points){
    // 该函数将计算规划的起点以及拼接轨迹的信息
    // 输入 上一周期的规划结果信息pre_trajectory x ,y,heading,kappa,velocity,accel,time
    //      本周期的绝对时间current_time
    //      定位信息host x,y,heading,kappa,vx,vy,ax,ay
    // 输出 规划起点信息plan_start x,y,heading,kappa,vx,vy,ax,ay(vx,vy,ax,ay)是世界坐标系的
    //      待拼接的轨迹信息，一般在上一周期的轨迹中的current_time+100ms，往前取20个点。
    //      当规划完成后，本周期的规划结果和stitch_trajectory一起拼好发给控制
    stitch_points->resize(20);
    //首次运行
    if (is_first_run==0)
    {
        is_first_run=1;
        plan_start_points->plan_start_x=ego_info.host_x;
        plan_start_points->plan_start_y=ego_info.host_y;
        plan_start_points->plan_start_heading=ego_info.host_heading_xy;
        plan_start_points->plan_start_kappa=0;
        plan_start_points->plan_start_vx=0;
        plan_start_points->plan_start_vy=0;
        plan_start_points->plan_start_ax=0;
        plan_start_points->plan_start_ay=0;
        plan_start_points->plan_start_time=current_time+0.1;//规划的起点时间应该是当前周期时间+100ms(100ms为一周期的时间)
    }
    else{
        x_cur=ego_info.host_x;
        y_cur=ego_info.host_y;
        heading_cur=ego_info.host_heading_xy;
        kappa_cur=0;
        vx_cur=ego_info.host_vx*cos(heading_cur)-ego_info.host_vy*sin(heading_cur);
        vy_cur=ego_info.host_vx*sin(heading_cur)+ego_info.host_vy*cos(heading_cur);
        ax_cur=ego_info.host_ax*cos(heading_cur)-ego_info.host_ay*sin(heading_cur);
        ay_cur=ego_info.host_ax*sin(heading_cur)+ego_info.host_ay*cos(heading_cur);
        //由于不是首次运行了，有上一时刻的轨迹了，找到上一周期轨迹规划的本周期车辆应该在的位置
        int index=0;
        for (size_t i = 0; i < pre_planning_trajectory_info.size()-1; i++)
        {
            if (pre_planning_trajectory_info[i].trajectory_time<=current_time+0.1 && pre_planning_trajectory_info[i+1].trajectory_time>current_time+0.1)
            {
                index=i;
                break;
            }
        }
        double pre_x_desire=pre_planning_trajectory_info[index].trajectory_x;
        double pre_y_desire=pre_planning_trajectory_info[index].trajectory_y;
        double pre_heading_desire=pre_planning_trajectory_info[index].trajectory_heading;
        //计算横纵向误差
        double dx_error=ego_info.host_x-pre_x_desire;
        double dy_error=ego_info.host_y-pre_y_desire;
        //横向误差
        double lat_error=std::abs(-dx_error*sin(pre_heading_desire)+dy_error*cos(pre_heading_desire));
        //纵向误差
        double lon_error=std::abs(dx_error*cos(pre_heading_desire)+dy_error*sin(pre_heading_desire));
        //纵向误差大于2.5 横向误差大于0.5 认为控制没跟上
        if (lat_error>2.5 || lon_error>0.5)
        {
            //此分支处理控制未跟上的情况，规划起点通过运动学递推
            plan_start_points->plan_start_x=x_cur+vx_cur*dt+0.5*ax_cur*dt*dt;
            plan_start_points->plan_start_y=y_cur+vy_cur*dt+0.5*ay_cur*dt*dt;
            plan_start_points->plan_start_vx= vx_cur + ax_cur * dt;;
            plan_start_points->plan_start_vy=vy_cur + ay_cur * dt;;
            plan_start_points->plan_start_heading=std::atan2(plan_start_points->plan_start_vy,plan_start_points->plan_start_vx);
            plan_start_points->plan_start_kappa=kappa_cur;
            plan_start_points->plan_start_ax=ax_cur;
            plan_start_points->plan_start_ay=ay_cur;
            plan_start_points->plan_start_time=current_time+0.1;
            return 1;
        }
        else{
            //此分支表示控制能跟上规划开始拼接
            int index_2=0;
            for (size_t i = 0; i < pre_planning_trajectory_info.size()-1; i++)
            {
                if (pre_planning_trajectory_info[i].trajectory_time<=current_time+0.1 && pre_planning_trajectory_info[i+1].trajectory_time>current_time+0.1)
                {
                    index_2=i;
                    break;
                }
            }
            //此算法需要能运行的轨迹点友很高的密度
            plan_start_points->plan_start_x=pre_planning_trajectory_info[index_2].trajectory_x;
            plan_start_points->plan_start_y=pre_planning_trajectory_info[index_2].trajectory_y;
            plan_start_points->plan_start_heading=pre_planning_trajectory_info[index_2].trajectory_heading;  
            plan_start_points->plan_start_kappa=pre_planning_trajectory_info[index_2].trajectory_kappa;
            //这里的pre_trajectory的速度,加速度是指轨迹的切向速度，切向加速度
            plan_start_points->plan_start_vx=pre_planning_trajectory_info[index_2].trajectory_speed*cos(plan_start_points->plan_start_heading);
            plan_start_points->plan_start_vy=pre_planning_trajectory_info[index_2].trajectory_speed*sin(plan_start_points->plan_start_heading);
            //pre_trajectory的加速度是切向加速度，要想计算ax,ay还要计算法向加速度
            //计算轨迹在current_time + 100ms 这个点的切向量法向量
            plan_start_points->plan_start_ax=pre_planning_trajectory_info[index_2].trajectory_accel*cos(plan_start_points->plan_start_heading)
                                             -pre_planning_trajectory_info[index_2].trajectory_accel*sin(plan_start_points->plan_start_heading);
            plan_start_points->plan_start_ay=pre_planning_trajectory_info[index_2].trajectory_accel*sin(plan_start_points->plan_start_heading)
                                             +pre_planning_trajectory_info[index_2].trajectory_accel*cos(plan_start_points->plan_start_heading);
            plan_start_points->plan_start_time=pre_planning_trajectory_info[index_2].trajectory_time;
            // 计算拼接轨迹
            // j 表示 current_time + 0.1 的时间点在 pre_trajectory_time的编号
            // 拼接是往从j开始，往前拼接20个，也就是pre_trajectory(j-1),j-2,j-3,....j-19
            // 分两种情况，如果j小于20，意味着前面的轨迹点不够20个
            // 如果j >= 20,意味着前面的点多于20个
            // 还有一个细节需要考虑，pre_trajectory x(j) y(j) ....是规划的起点，那么在轨迹拼接时
            // 需不需要在stitch_trajectory中把pre_trajectory x(j) y(j) ....包含进去
            // 答案是否定的，不应该包进去，因为规划的起点会在规划模块计算完成后的结果中包含，如果在拼接的时候
            // 还要包含，那就等于规划起点包含了两次
            // 除非本周期计算的轨迹不包含规划起点，那么stitch_trajectory可以包含规划起点。
            // 如果本周期计算的轨迹包含规划起点，那么stitch_trajectory就不可以包含规划起点。
            // 我们选择规划包含起点，拼接不包含起点的做法

            //把起点去掉
            index_2=index_2-1;
            stitch_trajectory stitch_path;
            if (index_2>=19)
            {
                int stitch_index_1=0;
                for (size_t i = index_2-19; i <=index_2; i++)
                {
                    stitch_path.stitch_x=pre_planning_trajectory_info[i].trajectory_x;
                    stitch_path.stitch_y=pre_planning_trajectory_info[i].trajectory_y;
                    stitch_path.stitch_heading=pre_planning_trajectory_info[i].trajectory_heading;
                    stitch_path.stitch_kappa=pre_planning_trajectory_info[i].trajectory_kappa;
                    stitch_path.stitch_speed=pre_planning_trajectory_info[i].trajectory_speed;
                    stitch_path.stitch_accel=pre_planning_trajectory_info[i].trajectory_accel;
                    stitch_path.stitch_time=pre_planning_trajectory_info[i].trajectory_time;
                    stitch_points->at(stitch_index_1) = stitch_path;
                    stitch_index_1 +=1;
                }
            }
            else{
                int stitch_index_2=19-index_2+1;
                for (size_t i = 0; i <= index_2; i++)
                {
                    stitch_path.stitch_x=pre_planning_trajectory_info[i].trajectory_x;
                    stitch_path.stitch_y=pre_planning_trajectory_info[i].trajectory_y;
                    stitch_path.stitch_heading=pre_planning_trajectory_info[i].trajectory_heading;
                    stitch_path.stitch_kappa=pre_planning_trajectory_info[i].trajectory_kappa;
                    stitch_path.stitch_speed=pre_planning_trajectory_info[i].trajectory_speed;
                    stitch_path.stitch_accel=pre_planning_trajectory_info[i].trajectory_accel;
                    stitch_path.stitch_time=pre_planning_trajectory_info[i].trajectory_time;
                    stitch_points->at(stitch_index_2) = stitch_path;
                    stitch_index_2 +=1;
                }
            } 
        }
    }
}