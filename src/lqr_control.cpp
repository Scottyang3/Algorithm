#include "lqr_control.h"

// 将角度(弧度制)归化到[-M_PI, M_PI]之间
double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI); //查找除法的余数
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

// 将角度转换为弧度制
double atan2_to_PI(const double atan2) { 
    return atan2 * M_PI / 180; 
}

void lqr_control::UpdataState(const location_info &VehicleState){
    //-----------------------------------------------------------------------------------------------------------------------//
    // A matrix (Gear Drive)
    // [0.0,                             1.0,                           0.0,                                            0.0;
    //  0.0,          (-(c_f + c_r) / m) / v,               (c_f + c_r) / m,                (l_r * c_r - l_f * c_f) / m / v;
    //  0.0,                             0.0,                           0.0,                                            1.0;
    //  0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-(l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    //-----------------------------------------------------------------------------------------------------------------------//
    double v = VehicleState.host_speed;
    if (v<0.01)
    {
        v = 0.01;//防止速度等于0时，0做分母
    }
    matrix_A = Matrix::Zero(4, 4);
    matrix_A(0,1) = 1.0;
    matrix_A(1,1) = (cf + cr) / (mass*v);
    matrix_A(1,2) = -(cf + cr) / mass;
    matrix_A(1,3) = -(lr*cr - lf*cf) / (mass*v);
    matrix_A(2,3) = 1.0;
    matrix_A(3,1) = -(lr * cr - lf * cf) / (iz*v);
    matrix_A(3,2) = -(lf * cf - lr * cr) / iz;
    matrix_A(3,3) = (lf*lf*cf + lr*lr*cr) / (iz*v);
    Eigen::MatrixXd matrix_a_temp = Eigen::MatrixXd::Identity(matrix_A.rows(), matrix_A.cols());
    matrix_A = matrix_A * ts + matrix_a_temp;
    //-----------------------------------------------------------------------------------------------------------------------//
    // b = [0.0;
    //      c_f / m;
    //      0.0;
    //      l_f * c_f / i_z;]
    //-----------------------------------------------------------------------------------------------------------------------//
    // 初始化B矩阵
    matrix_B = Matrix::Zero(4,1);
    matrix_B(1, 0) = -cf / mass;
    matrix_B(3, 0) = -lf * cf / iz;
    matrix_B=matrix_B*ts;

    // 初始化Q矩阵
    matrix_Q = Matrix::Zero(4, 4);
    matrix_Q(0, 0) = 5;   // lateral_error
    matrix_Q(1, 1) = 1;    // lateral_error_rate
    matrix_Q(2, 2) = 4;    // heading_error
    matrix_Q(3, 3) = 1;    // heading__error_rate

    //// 初始化R矩阵
    matrix_R = Matrix::Identity(1, 1);
    matrix_R(0, 0) = R; //R越大对输入u惩罚权重越大，越大则u变化越缓慢
}

Matrix1x4 lqr_control::Cal_Riccati(const Matrix &A, const Matrix &B, const Matrix &Q,
                                   const Matrix &R, const double err_tolerance,
                                   const uint max_num_iteration){
    Matrix1x4 K;
    // 防止矩阵的维数出错导致后续的运算失败
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
        std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
        return K;
    }
    Eigen::MatrixXd P = Q;
    double Riccati_err;
    Eigen::MatrixXd P_next;
    //--------------------------------------------------------------------------------//
    // std::cout<<"A \n"<<A<<std::endl;
    // std::cout<<"B \n"<<B<<std::endl;
    // std::cout<<"R \n"<<R<<std::endl;
    // std::cout<<"P \n"<<P<<std::endl;
    // std::cout<<"Q \n"<<Q<<std::endl;
    int num_Riccati = 0;
    for (size_t i = 0; i < max_num_iteration; i++) {
        P_next = Q + A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;//迭代公式
        Riccati_err = std::fabs((P_next - P).maxCoeff());
        //std::cout<<"Riccati_err=  "<<Riccati_err<<std::endl;
        P = P_next;
        num_Riccati += 1;
        if (Riccati_err < err_tolerance) {
            break;
        }
    }
    if (num_Riccati>=max_num_iteration)
    {
        std::cout<<"LQR solver cannot converge to a solution. (Riccati不收敛) "<<std::endl;
    }
    // else{
    //     std::cout<<"max consecutive result diff. (最大连续结果差异):  "<< num_Riccati <<std::endl;
    // }
    
    K = (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
    return K;
}

int lqr_control::QueryNearestPointByPosition(double x, double y, std::vector<planning_trajectory> &planning_path){
        int path_length;
    path_length = planning_path.size(); //规划轨迹的长度
    double d_min = 1e6;
    int min_index = 0;
    for (size_t i = 0; i < path_length; i++)
    {
        double d_temp = std::pow((x - planning_path[i].trajectory_x),2) + std::pow((y - planning_path[i].trajectory_y),2);
        if (d_temp < d_min){
            d_min = d_temp;
            min_index = i;
        }
    }
    return min_index;
}

LateralControlError lqr_control::Cal_Error(double x, double y, double yaw, double vx, double vy, double dyaw, std::vector<planning_trajectory> &planning_path){
    int min_index = QueryNearestPointByPosition(x,y,planning_path);
    std::vector<double> tor(2);
    std::vector<double> nor(2);
    std::vector<double> d_err(2);
    // std::cout<<"ego_x=  "<<x<<"  ego_y=  "<<y<<std::endl;
    // std::cout<<"match_points_x=  "<<planning_path[min_index].trajectory_x<<"  match_points_y=  "<<planning_path[min_index].trajectory_y<<std::endl;
    // std::cout<<"match_points_heading=  "<<planning_path[min_index].trajectory_heading<<std::endl;
    tor[0] = cos(planning_path[min_index].trajectory_heading);
    tor[1] = sin(planning_path[min_index].trajectory_heading);
    nor[0] = -sin(planning_path[min_index].trajectory_heading);
    nor[1] = cos(planning_path[min_index].trajectory_heading);
    d_err[0] = x - planning_path[min_index].trajectory_x;
    d_err[1] = y - planning_path[min_index].trajectory_y;
    LateralControlError lqr_error;
    double lateral_error= nor[0]*d_err[0] + nor[1]*d_err[1];    //横向误差
    lqr_error.lateral_error = lateral_error;

    double lon_error = nor[0]*d_err[0] + nor[1]*d_err[1];    //纵向误差
    double projection_point_thetar = planning_path[min_index].trajectory_heading + planning_path[min_index].trajectory_kappa * lon_error;

    double lateral_error_rate = vy * cos(yaw - projection_point_thetar) + vx * sin(yaw - projection_point_thetar);   //横向误差变化率
    lqr_error.lateral_error_rate = lateral_error_rate;

    double heading_error = sin(yaw - projection_point_thetar);   //航行误差
    lqr_error.heading_error = heading_error;

    double ss_dot = vx*cos(yaw-projection_point_thetar)-vy*sin(yaw-projection_point_thetar);
    double s_dot = ss_dot/(1 - planning_path[min_index].trajectory_kappa * lateral_error);
    double heading_error_rate = dyaw - planning_path[min_index].trajectory_kappa * s_dot; ////纵向误差变化率
    lqr_error.heading_error_rate = heading_error_rate;

    matrix_err_state = Matrix::Zero(4, 1);
    matrix_err_state(0, 0)=lateral_error;
    matrix_err_state(1, 0)=lateral_error_rate;
    matrix_err_state(2, 0)=heading_error;
    matrix_err_state(3, 0)=heading_error_rate;
    // std::cout<<"***lateral_error=  "<<lateral_error<<std::endl;
    return lqr_error;
}

double lqr_control::ComputeControlCommand(const location_info &VehicleState, std::vector<planning_trajectory> &planning_path){
    UpdataState(VehicleState);
    Matrix1x4 K=Cal_Riccati(matrix_A, matrix_B, matrix_Q, matrix_R , err_tolerance, max_num_iteration);
    //---------------------------------------------------------//
    //  预测模块
    //  pre_x = x + vx * ts * cos(phi) - vy * ts * sin(phi);
    //  pre_y = y + vy * ts * cos(phi) + vx * ts * sin(phi);
    //  pre_heading = phi + phi_dot * ts;
    //  pre_vx = vx;
    //  pre_vy = vy;
    //  pre_phi_dot = phi_dot;
    double pre_ts=0.01;
    double pre_x = VehicleState.host_x + VehicleState.host_vx*pre_ts*cos(VehicleState.host_heading_xy)-VehicleState.host_vy*pre_ts*sin(VehicleState.host_heading_xy);
    double pre_y = VehicleState.host_y + VehicleState.host_vy*pre_ts*cos(VehicleState.host_heading_xy)-VehicleState.host_vx*pre_ts*sin(VehicleState.host_heading_xy);
    double pre_heading = VehicleState.host_heading_xy + VehicleState.host_yawrate*pre_ts;
    double pre_speed = VehicleState.host_speed;
    double pre_vx = VehicleState.host_vx;
    double pre_vy = VehicleState.host_vy;
    double pre_dheading = VehicleState.host_yawrate;
    if(pre_vx < 0.1){pre_vx = pre_speed;}//如果传入的vx=0说明没有接受到，用v近似代替
    //---------------------------------------------------------//
    //计算误差模块
    LateralControlError control_err;
    control_err = Cal_Error(pre_x, pre_y, pre_heading, pre_vx, pre_vy, pre_dheading, planning_path);

    //------------------------------------------------------------------------------------//
    //前馈控制模块
    double steer_angle_feedforwardterm;
    int min_index = QueryNearestPointByPosition(pre_x, pre_y,planning_path);
    double ref_curvature = planning_path[min_index].trajectory_kappa;
    // 前馈控制方法一:
    steer_angle_feedforwardterm = atan(ref_curvature * (lf + lr));
    // 前馈控制方法二:
    // double v = VehicleState.host_speed;
    // const double kv = lr * mass / 2 / cf / wheelbase - lf * mass / 2 / cr / wheelbase;
    // steer_angle_feedforwardterm = (wheelbase * ref_curvature + kv * v * v * ref_curvature - K(0, 2) * 
    //                                 (lr * ref_curvature - lf * mass * v * v * ref_curvature / 2 / cr / wheelbase));
    // 前馈控制方法三:
    // double v = VehicleState.host_speed;
    // steer_angle_feedforwardterm = ref_curvature*(wheelbase-lr*K(0, 2)-
    //                                    mass*v*v*(lr/cf+lf/cr*K(0, 2)-lf/cr)/wheelbase);//老王
    //------------------------------------------------------------------------------------//

    //------------------------------------------------------------------------------------//
    //控制大小的计算
    double steer_angle_feedback = -(K * matrix_err_state)(0,0);
    double steer_angle = steer_angle_feedback + steer_angle_feedforwardterm;
    //------------------------------------------------------------------------------------//

    //------------------------------------------------------------------------------------//
    //输出观察数值
    // std::cout<<"K =  "<< K <<std::endl;
    // std::cout<<"matrix_err_state=  "<<matrix_err_state.transpose()<<std::endl;
    // std::cout<<"steer_angle_feedback=  "<<steer_angle_feedback<<std::endl;
    // std::cout<<"steer_angle_feedforwardterm=  "<<steer_angle_feedforwardterm<<std::endl;
    // std::cout<<"----------steeer_angle=  "<<steer_angle *180 / 3.1415926<< " deg"<<std::endl;
    //------------------------------------------------------------------------------------//

    // 限制前轮最大转角，这里定义前轮最大转角位于 [-20度～20度].
    if (steer_angle >= atan2_to_PI(20)) {
        steer_angle = atan2_to_PI(20);
    } else if (steer_angle <= -atan2_to_PI(20)) {
        steer_angle = -atan2_to_PI(20);
    }
    return steer_angle;
}