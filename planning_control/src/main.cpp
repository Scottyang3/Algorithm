#include <iostream>
#include <cmath>
#include <vector>
#include <ctime>
#include "struct_common.h"
#include "Referenceline.h"
#include "Load_Roadmap.h"
#include "calc_plan_stitch_trajectory.h"
#include "Cartesian2Frenet.h"
#include "LQR_control.h"
#include "lqr_control.h"
#include "Obstacle.h"
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;

location_info update_state(double steer, location_info car) {
    car.host_x += car.host_speed * cos(car.host_heading_xy)* 0.01;
    car.host_y += car.host_speed * sin(car.host_heading_xy)* 0.01;
    car.host_heading_xy=car.host_heading_xy + car.host_speed/2.947*tan(steer)*0.01;
    car.host_speed=30;
    car.host_vx=0;
    car.host_vy=0;
    car.host_yawrate=0;
    car.host_acceleration=0;
    car.host_ax=0;
    car.host_ay=0;
	return car;
}

int main() {
    plt::figure_size(1200, 800);
    //------------------------------------------------------------//
    //load roadmap
    std::vector<xy_points_info> xy_points_;
    xy_points_info xy_load;
    PathPlanningNode loadpath;
    std::string roadmap_path;
    roadmap_path="/home/yj/le_materials/demo_test/data/path.txt";
    loadpath.loadRoadmap(roadmap_path, &xy_points_);

    //将点存如向量画图观察
    std::vector<double> xx,yy;//画图全局路径
    for (size_t i = 0; i < xy_points_.size(); i++)
    {
        xy_load.x=xy_points_[i].x;
        xy_load.y=xy_points_[i].y;
        xx.push_back(xy_points_[i].x);
        yy.push_back(xy_points_[i].y);
    }
    //------------------------------------------------------------//

    //------------------------------------------------------------//
    //Get the current time
    time_t nowtime;
    double current_time=time(&nowtime);
    // std::cout<<"current_time=  "<<current_time<<std::endl;
    //------------------------------------------------------------//

    //------------------------------------------------------------//
    //Calculate heading accumulated_s kappa dkappas
    ReferenceLine refline;//参考线�?
    std::vector<double> headings, accumulated_s, kappas, dkappas;
    refline.Clac_heading_kappa_info(xy_points_, &headings, &accumulated_s, &kappas, &dkappas);
    std::vector<proj_points_info> proj_points;//声明一个类型为proj_points的变量来存投影点的数�?
    std::vector<xy_points_info> referenceline_xy_points;
    std::vector<xy_points_info> fem_smoothing_line;
    //画图数据记录

    //------------------------------------------------------------//
    // 初始本车信息
    location_info ego_info;
    ego_info.host_x= xy_points_[40].x;
    ego_info.host_y= xy_points_[40].y;
    ego_info.host_heading_xy=headings[40];
    ego_info.host_speed=2;
    ego_info.host_vx=0;
    ego_info.host_vy=0;
    ego_info.host_yawrate=0;
    ego_info.host_acceleration=0;
    ego_info.host_ax=0;
    ego_info.host_ay=0;
    //------------------------------------------------------------//
    std::vector<double>ego_x,ego_y;
    std::vector<double>fem_x,fem_y;


    for (size_t i = 0; i < 800; i++){
        //------------------------------------------------------------//
        //Calculate proj_match_index proj_point_x proj_point_y
        refline.Clac_proj_points_info(ego_info.host_x,ego_info.host_y,xy_points_, &proj_points);
        // std::cout<<"proj_match_index= "<<proj_points.proj_match_index<<"   proj_point_x= "<<proj_points.proj_point_x<<"   proj_point_y= "<<proj_points.proj_point_y<<std::endl;

        //提取referenceline 取出向前150个点 和向前30个点 减少计算量
        refline.intercept_referenceline(proj_points[0].proj_match_index,xy_points_,&referenceline_xy_points);

        //输出查看参考线
        std::vector<double> ref_x,ref_y;
        for (size_t i = 0; i < referenceline_xy_points.size(); i++)
        {
            ref_x.push_back(referenceline_xy_points[i].x);
            ref_y.push_back(referenceline_xy_points[i].y);
        }
        //------------------------------------------------------------//
        //输出查看自车所在位置
        ego_x.push_back(ego_info.host_x);
        ego_y.push_back(ego_info.host_y);


        //------------------------------------------------------------------------------//
        // refline.Fem_smoothing(referenceline_xy_points, &fem_smoothing_line);
        // for (size_t i = 0; i < fem_smoothing_line.size(); i++)
        // {
        //     fem_x.push_back(fem_smoothing_line[i].x);
        //     fem_y.push_back(fem_smoothing_line[i].y);
        // }
        // std::vector<planning_trajectory> planning_path;
        // planning_path.resize(fem_smoothing_line.size());
        // ControlCmd cmd;
        // std::vector<double> pheadings, paccumulated_s, pkappas, pdkappas;
        // refline.Clac_heading_kappa_info(fem_smoothing_line, &pheadings, &paccumulated_s, &pkappas, &pdkappas);
        // for (size_t i = 0; i < fem_smoothing_line.size(); i++)
        // {
        //     planning_path[i].trajectory_x = fem_smoothing_line[i].x;
        //     planning_path[i].trajectory_y = fem_smoothing_line[i].y;
        //     planning_path[i].trajectory_heading=pheadings[i];
        //     planning_path[i].trajectory_kappa=pkappas[i];
        //     planning_path[i].trajectory_accel=0.0;
        //     planning_path[i].trajectory_speed=0.0;
        //     planning_path[i].trajectory_time=1.0;
        // }
        //------------------------------------------------------------------------------//

        //------------------------------------------------------------------------------//
        std::vector<planning_trajectory> planning_path;
        planning_path.resize(referenceline_xy_points.size());
        ControlCmd cmd;
        std::vector<double> pheadings, paccumulated_s, pkappas, pdkappas;
        refline.Clac_heading_kappa_info(referenceline_xy_points, &pheadings, &paccumulated_s, &pkappas, &pdkappas);
        for (size_t i = 0; i < referenceline_xy_points.size(); i++)
        {
            planning_path[i].trajectory_x = referenceline_xy_points[i].x;
            planning_path[i].trajectory_y = referenceline_xy_points[i].y;
            planning_path[i].trajectory_heading=pheadings[i];
            planning_path[i].trajectory_kappa=pkappas[i];
            planning_path[i].trajectory_accel=0.0;
            planning_path[i].trajectory_speed=0.0;
            planning_path[i].trajectory_time=1.0;
        }
        //------------------------------------------------------------------------------//
        Obstacle obs;

        // lqr计算前轮控制转角
        //深蓝
        // LQR_control lqr_sl;
        // double steer_angle_control = -lqr_sl.ComputeControlCommand(ego_info, planning_path, cmd);
        // std::cout<<" sl--------steeer_angle=  "<<steer_angle_control << " rad \n \n"<<std::endl;
        //老王
        lqr_control lqr;
        double steer_angle_control = lqr.ComputeControlCommand(ego_info, planning_path);
        std::cout<<"---------steeer_angle=  "<<steer_angle_control << " rad"<<std::endl;
        //------------------------------------------------------------//

        // //轨迹拼接
        // calc_plan_trajectory stitch_path;
        // plan_start_info plan_start_points;
        // std::vector<stitch_trajectory> stitch_points;
        // std::vector<planning_trajectory> pre_planning_trajectory_info;
        // stitch_path.calc_plan_stitchtrajectory(pre_planning_trajectory_info, ego_info, current_time, &stitch_points, &plan_start_points);
        // // std::cout << "stitch_points大小= " << stitch_points.size() << std::endl;
        // // for (size_t i = 0; i < stitch_points.size(); i++)
        // // {
        // //     std::cout<<"stitch_points[i].stitch_x= "<<stitch_points[i].stitch_x<<std::endl;
        // // }

        // //Cartesian2Frenet
        // Cartesian2Frenet CarFte;
        // //              std::vector<points_conf> frenet_path, 
        // //              std::vector<proj_points> proj_points, 
        // // std::vector<double> index2s;
        // // std::vector<double> *s_set;
        // // std::vector<double> *l_set;
        // // CarFte.Calc_SL(ego_info.host_x,ego_info.host_y, referenceline_xy_points,proj_points,index2s,&s_set, &l_set);
        
        ego_info = update_state(steer_angle_control,ego_info);
        // 画图部分
        // plt::clf();
        // plt::named_plot("map", xx, yy, "r:");   //全局地图
        // plt::title("Road Map");
        // plt::legend();
        // plt::named_plot("referenceline",ref_x, ref_y, "b:");    //提取的参考线
        // plt::named_plot("ego_xy",ego_x, ego_y, "r-*");  //自车所在位�?
        // plt::legend();
        // plt::named_plot("fem",fem_x, fem_y, "g:");  //平滑过后的曲�?
        // plt::legend();
        // plt::pause(0.01);

        fem_smoothing_line.clear();
        referenceline_xy_points.clear();
        planning_path.clear();
        fem_x.clear();
        fem_y.clear();
    }

    // plt::figure_size(1200, 800);

    plt::named_plot("map", xx, yy, "r:");   //全局地图
    plt::title("Road Map");
    plt::legend();
    // plt::named_plot("referenceline",ref_x, ref_y, "b:");    //提取的参考线
    plt::named_plot("ego_xy",ego_x, ego_y, "r-*");  //自车所在位置
    plt::legend();
    plt::named_plot("fem",fem_x, fem_y, "g:");  //平滑过后的曲线
    plt::legend();
    plt::show();
}
