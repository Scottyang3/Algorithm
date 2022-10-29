#pragma once
#include <limits> 
#include <vector>

typedef std::numeric_limits<double> Info;
double NAN_d = Info::quiet_NaN();

// ������Ϣ
 struct location_info {
     double host_x;
     double host_y;
     double host_heading_xy;
     double host_yawrate;
     double host_speed;
     double host_vx;
     double host_vy;
     double host_acceleration;
     double host_ax;
     double host_ay;
 };

 struct xy_points_info{
    double x;
    double y;
 }; 

 // �ϰ�����Ϣ
 struct obstacle_info {
     double obs_x;
     double obs_y;
     double obs_heading;
     double obs_velocity;
 };

 // 
 struct PathProfile_info{
      double headings;
      double accumulated_s;
      double kappas;
      double dkappas;
 };

 // ·�������Ϣ
 struct points_conf {
   double x;
   double y;
   double heading;
   double kappa;
 };

 // �켣�滮�Ľ��
 struct planning_trajectory
 {
  double trajectory_x;
  double trajectory_y;
  double trajectory_heading;
  double trajectory_kappa;
  double trajectory_speed;
  double trajectory_accel;
  double trajectory_time;
 };

//ƴ�ӹ켣�ṹ��
 struct stitch_trajectory
{
    double stitch_x;
    double stitch_y;
    double stitch_heading;
    double stitch_kappa;
    double stitch_speed;
    double stitch_accel;
    double stitch_time;
};
struct plan_start_info
{
    double plan_start_x;
    double plan_start_y;
    double plan_start_vx;
    double plan_start_vy;
    double plan_start_heading;
    double plan_start_ax;
    double plan_start_ay;
    double plan_start_kappa;
    double plan_start_time;
};

//ͶӰ��Ľṹ��
struct proj_points_info
{
    double proj_match_index;
    double proj_point_x;
    double proj_point_y;
    double proj_heading;
    double proj_kappa;
};

struct ControlCmd {
    double steer_target;
    double acc;
};

struct LateralControlError {
    double lateral_error;       // �������
    double heading_error;       // ת�����
    double lateral_error_rate;  // �����������
    double heading_error_rate;  // ת������ٶ�
};