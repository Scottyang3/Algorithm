#include "calc_plan_stitch_trajectory.h"

bool calc_plan_trajectory::calc_plan_stitchtrajectory(std::vector<planning_trajectory> &pre_planning_trajectory_info, 
                                                                      location_info &ego_info, 
                                                                      double current_time,
                                                                      std::vector<stitch_trajectory> *stitch_points,
                                                                      plan_start_info *plan_start_points){
    // �ú���������滮������Լ�ƴ�ӹ켣����Ϣ
    // ���� ��һ���ڵĹ滮�����Ϣpre_trajectory x ,y,heading,kappa,velocity,accel,time
    //      �����ڵľ���ʱ��current_time
    //      ��λ��Ϣhost x,y,heading,kappa,vx,vy,ax,ay
    // ��� �滮�����Ϣplan_start x,y,heading,kappa,vx,vy,ax,ay(vx,vy,ax,ay)����������ϵ��
    //      ��ƴ�ӵĹ켣��Ϣ��һ������һ���ڵĹ켣�е�current_time+100ms����ǰȡ20���㡣
    //      ���滮��ɺ󣬱����ڵĹ滮�����stitch_trajectoryһ��ƴ�÷�������
    stitch_points->resize(20);
    //�״�����
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
        plan_start_points->plan_start_time=current_time+0.1;//�滮�����ʱ��Ӧ���ǵ�ǰ����ʱ��+100ms(100msΪһ���ڵ�ʱ��)
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
        //���ڲ����״������ˣ�����һʱ�̵Ĺ켣�ˣ��ҵ���һ���ڹ켣�滮�ı����ڳ���Ӧ���ڵ�λ��
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
        //������������
        double dx_error=ego_info.host_x-pre_x_desire;
        double dy_error=ego_info.host_y-pre_y_desire;
        //�������
        double lat_error=std::abs(-dx_error*sin(pre_heading_desire)+dy_error*cos(pre_heading_desire));
        //�������
        double lon_error=std::abs(dx_error*cos(pre_heading_desire)+dy_error*sin(pre_heading_desire));
        //����������2.5 ����������0.5 ��Ϊ����û����
        if (lat_error>2.5 || lon_error>0.5)
        {
            //�˷�֧�������δ���ϵ�������滮���ͨ���˶�ѧ����
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
            //�˷�֧��ʾ�����ܸ��Ϲ滮��ʼƴ��
            int index_2=0;
            for (size_t i = 0; i < pre_planning_trajectory_info.size()-1; i++)
            {
                if (pre_planning_trajectory_info[i].trajectory_time<=current_time+0.1 && pre_planning_trajectory_info[i+1].trajectory_time>current_time+0.1)
                {
                    index_2=i;
                    break;
                }
            }
            //���㷨��Ҫ�����еĹ켣���Ѻܸߵ��ܶ�
            plan_start_points->plan_start_x=pre_planning_trajectory_info[index_2].trajectory_x;
            plan_start_points->plan_start_y=pre_planning_trajectory_info[index_2].trajectory_y;
            plan_start_points->plan_start_heading=pre_planning_trajectory_info[index_2].trajectory_heading;  
            plan_start_points->plan_start_kappa=pre_planning_trajectory_info[index_2].trajectory_kappa;
            //�����pre_trajectory���ٶ�,���ٶ���ָ�켣�������ٶȣ�������ٶ�
            plan_start_points->plan_start_vx=pre_planning_trajectory_info[index_2].trajectory_speed*cos(plan_start_points->plan_start_heading);
            plan_start_points->plan_start_vy=pre_planning_trajectory_info[index_2].trajectory_speed*sin(plan_start_points->plan_start_heading);
            //pre_trajectory�ļ��ٶ���������ٶȣ�Ҫ�����ax,ay��Ҫ���㷨����ٶ�
            //����켣��current_time + 100ms ������������������
            plan_start_points->plan_start_ax=pre_planning_trajectory_info[index_2].trajectory_accel*cos(plan_start_points->plan_start_heading)
                                             -pre_planning_trajectory_info[index_2].trajectory_accel*sin(plan_start_points->plan_start_heading);
            plan_start_points->plan_start_ay=pre_planning_trajectory_info[index_2].trajectory_accel*sin(plan_start_points->plan_start_heading)
                                             +pre_planning_trajectory_info[index_2].trajectory_accel*cos(plan_start_points->plan_start_heading);
            plan_start_points->plan_start_time=pre_planning_trajectory_info[index_2].trajectory_time;
            // ����ƴ�ӹ켣
            // j ��ʾ current_time + 0.1 ��ʱ����� pre_trajectory_time�ı��
            // ƴ��������j��ʼ����ǰƴ��20����Ҳ����pre_trajectory(j-1),j-2,j-3,....j-19
            // ��������������jС��20����ζ��ǰ��Ĺ켣�㲻��20��
            // ���j >= 20,��ζ��ǰ��ĵ����20��
            // ����һ��ϸ����Ҫ���ǣ�pre_trajectory x(j) y(j) ....�ǹ滮����㣬��ô�ڹ켣ƴ��ʱ
            // �費��Ҫ��stitch_trajectory�а�pre_trajectory x(j) y(j) ....������ȥ
            // ���Ƿ񶨵ģ���Ӧ�ð���ȥ����Ϊ�滮�������ڹ滮ģ�������ɺ�Ľ���а����������ƴ�ӵ�ʱ��
            // ��Ҫ�������Ǿ͵��ڹ滮������������
            // ���Ǳ����ڼ���Ĺ켣�������滮��㣬��ôstitch_trajectory���԰����滮��㡣
            // ��������ڼ���Ĺ켣�����滮��㣬��ôstitch_trajectory�Ͳ����԰����滮��㡣
            // ����ѡ��滮������㣬ƴ�Ӳ�������������

            //�����ȥ��
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