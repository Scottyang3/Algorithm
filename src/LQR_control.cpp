#include "LQR_control.h"

// 两点之间的距离
double PointDistanceSquare(const planning_trajectory &point, const double x, const double y) {
    double dx = point.trajectory_x - x;
    double dy = point.trajectory_y - y;
    return dx * dx + dy * dy;
}

// 将角度(弧度制)归化到[-M_PI, M_PI]之间
double NormalizeAngle_sl(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI); //查找除法的余数
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

// 将角度转换为弧度制
double atan2_to_PI_sl(const double atan2) { 
    return atan2 * M_PI / 180; 
}

// 加载车辆参数
void LQR_control::LoadControlConf(){
    ts_=0.01;   //每隔0.01s进行一次控制
    cf_ = 155494.663;                              // 前轮侧偏刚度,左右轮之和
    cr_ = 155494.663;                              // 后轮侧偏刚度, 左右轮之和
    wheelbase_ = 2.852;                            // 左右轮的距离
    steer_ratio_ = 16;                             // 方向盘的转角到轮胎转动角度之间的比值系数
    steer_single_direction_max_degree_ = 470.0;    // 最大方向转角
    const double mass_fl = 520;                    // 左前悬的质量
    const double mass_fr = 520;                    // 右前悬的质量
    const double mass_rl = 520;                    // 左后悬的质量
    const double mass_rr = 520;                    // 右后悬的质量
    const double mass_front = mass_fl + mass_fr;   // 前悬质量
    const double mass_rear = mass_rl + mass_rr;    // 后悬质量
    mass_ = mass_front + mass_rear;                //总质量
    lf_ = wheelbase_ * (1.0 - mass_front / mass_);    // 汽车前轮到中心点的距离
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);     // 汽车后轮到中心点的距离
    // moment of inertia
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;    // 汽车的转动惯量

    lqr_eps_ = 0.01;              // LQR 迭代求解精度
    lqr_max_iteration_ = 1500;    // LQR的迭代次数 
}

// 初始化状态矩阵
void LQR_control::Init(){
    matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_ad_= Matrix::Zero(basic_state_size_, basic_state_size_);
    //-----------------------------------------------------------------------------------------------------------------------//
    // A matrix (Gear Drive)
    // [0.0,                             1.0,                           0.0,                                            0.0;
    //  0.0,          (-(c_f + c_r) / m) / v,               (c_f + c_r) / m,                (l_r * c_r - l_f * c_f) / m / v;
    //  0.0,                             0.0,                           0.0,                                            1.0;
    //  0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z, (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    //-----------------------------------------------------------------------------------------------------------------------//
    // 初始化A矩阵的常数项
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    // 初始化A矩阵的非常数项
    matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

    //-----------------------------------------------------------------------------------------------------------------------//
    // b = [0.0;
    //      c_f / m;
    //      0.0;
    //      l_f * c_f / i_z;]
    //-----------------------------------------------------------------------------------------------------------------------//
    // 初始化B矩阵
    matrix_b_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    // 状态向量
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    // 反馈矩阵
    matrix_k_ = Matrix::Zero(1, basic_state_size_);
    // lqr cost function中 输入值u的权重
    matrix_r_ = Matrix::Identity(1, 1);
    matrix_r_(0, 0) = 10;
    // lqr cost function中 状态向量x的权重
    matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

    // int q_param_size = 4;
    matrix_q_(0, 0) = 25;   // lateral_error
    matrix_q_(1, 1) = 3;    // lateral_error_rate
    matrix_q_(2, 2) = 10;    // heading_error
    matrix_q_(3, 3) = 4;    // heading__error_rate
}

// 计算匹配点的信息
planning_trajectory LQR_control::QueryNearestPointByPosition(const double x, const double y) {
    double d_min = 1e6; //最小距离初始化
    size_t index_min = 0;
    for (size_t i = 0; i < planning_path.size(); i++) {
        double d_temp = PointDistanceSquare(planning_path[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    ref_curv_ = planning_path[index_min].trajectory_kappa;    // 对应的最近的轨迹点上的曲率

    double front_index = index_min + 5; //前视5个点
    if (front_index > planning_path.size()){
        ref_curv_front_ = planning_path[index_min].trajectory_kappa;
    }
    else{
        ref_curv_front_ = planning_path[front_index].trajectory_kappa;

    }
    return planning_path[index_min];    //此时返回的是当前最近的一个点，如要前视将index_min改为front_index
}

// 计算误差
void LQR_control::ComputeErrors(const double x,
                                const double y,
                                const double theta,
                                const double linear_v,
                                const double angular_v,
                                const double linear_a,
                                LateralControlErrorPtr &lat_con_error){
    double pre_x = x+linear_v*0.1*std::cos(theta); //预测部分
    double pre_y = y+linear_v*0.1*std::sin(theta);
    planning_trajectory match_points=this->QueryNearestPointByPosition(pre_x, pre_y);
    // 车辆坐标系：X轴沿着车辆纵向，向前为正，Y沿着车辆横向，向左为正（从车头往前看的视角），在车辆坐标系下，距离车辆最近的路径点位于车辆左侧，车辆应该左转以跟踪参考路径
    // 计算横向误差的时候，需要将路径上距离车辆最近的点从世界坐标系变换到车辆坐标系下，路径点在车辆坐标系下的横坐标就是横向位置误差，根据路径点在车辆坐标系下的横坐标的正负决定前轮转角的方向
    // double closest_point_x_in_vehicle_coordinate = (current_closest_point.x - x) * cos(theta) + (current_closest_point.y - y) * sin(theta);
    double e_y = -(match_points.trajectory_x - x) * sin(match_points.trajectory_heading) + (match_points.trajectory_y - y) * cos(match_points.trajectory_heading);
    // 纵向误差
    double e_x = (match_points.trajectory_x - x) * cos(match_points.trajectory_heading) + (match_points.trajectory_y- y) * sin(match_points.trajectory_heading);
    double e_theta = match_points.trajectory_heading+match_points.trajectory_kappa *e_x - theta;
    // double e_theta = current_closest_point.heading - theta;    // 路径上距离车辆最近的点的参考航向角，大于车辆的当前航向角的话，车辆应左转以跟踪航向
    // 限制前轮转角的取值区间
    if (e_theta > M_PI) {
        e_theta = e_theta - M_PI * 2;
    }
    if (e_theta < -M_PI) {
        e_theta = e_theta + M_PI * 2;
    }

    double e_y_dot = linear_v * sin(e_theta);
    // double e_theta_dot = angular_v - current_closest_point.kappa * current_closest_point.v;
    double e_theta_dot = angular_v - match_points.trajectory_kappa * (match_points.trajectory_speed *cos(e_theta)/(1-match_points.trajectory_kappa *e_y));
    // std::cout<<"\n -----横向偏差为:"<< e_y<<"\n -----航行偏差为:"<< e_theta<<std::endl;
    lat_con_error->lateral_error = e_y;
    lat_con_error->lateral_error_rate = e_y_dot;
    lat_con_error->heading_error = e_theta;
    lat_con_error->heading_error_rate = e_theta_dot;
}

// 更新状态
void LQR_control::UpdataState(const location_info &VehicleState){
    // LateralControlError lat_con_error;  // 将其更改为智能指针
    std::shared_ptr<LateralControlError> lat_con_error = std::make_shared<LateralControlError>();
    // 计算横向误差
    ComputeErrors(VehicleState.host_x, 
                  VehicleState.host_y, 
                  VehicleState.host_heading_xy, 
                  VehicleState.host_speed, 
                  VehicleState.host_yawrate, 
                  VehicleState.host_acceleration, 
                  lat_con_error);
    // State matrix update;
    matrix_state_(0, 0) = lat_con_error->lateral_error;
    matrix_state_(1, 0) = lat_con_error->lateral_error_rate;
    matrix_state_(2, 0) = lat_con_error->heading_error;
    matrix_state_(3, 0) = lat_con_error->heading_error_rate;
    // 更新状态矩阵A并将状态矩阵A离散化
    Eigen::MatrixXd matrix_a_temp = Eigen::MatrixXd::Identity(matrix_a_.rows(), matrix_a_.cols());
    matrix_ad_ = matrix_a_ * ts_ + matrix_a_temp;
}

// 求解LQR方程
void LQR_control::SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, const double tolerance, const uint max_num_iteration, Matrix *ptr_K) {
    // 防止矩阵的维数出错导致后续的运算失败
    if (A.rows() != A.cols() || B.rows() != A.rows() || Q.rows() != Q.cols() || Q.rows() != A.rows() || R.rows() != R.cols() || R.rows() != B.cols()) {
        std::cout << "LQR solver: one or more matrices have incompatible dimensions." << std::endl;
        return;
    }
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd AT = A.transpose();
    Eigen::MatrixXd BT = B.transpose();
    Eigen::MatrixXd Rinv = R.inverse();
    double Riccati_err;
    Eigen::MatrixXd P_next;
    //--------------------------------------------------------------------------------//
    // std::cout<<"A"<<A<<std::endl;
    // std::cout<<"B"<<B<<std::endl;
    // std::cout<<"R"<<R<<std::endl;
    // std::cout<<"P"<<P<<std::endl;
    // std::cout<<"Q"<<Q<<std::endl;
    for (size_t i = 0; i < max_num_iteration-1470; i++) {
        P_next = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
        Riccati_err = std::fabs((P_next - P).maxCoeff());
        P = P_next;
        if (Riccati_err < tolerance) {
            break;
        }
    }
    *ptr_K = (R + BT * P * B).inverse() * BT * P * A;
}

// 计算前馈控制
double LQR_control::ComputeFeedForward(const location_info &VehicleState,
                                       double ref_curvature){
    if (std::isnan(ref_curvature))
    {
        ref_curvature = 0;
    }
    double steer_angle_feedforwardterm;
    steer_angle_feedforwardterm = atan(ref_curvature * (lf_ + lr_));

    //const double v = localization.velocity;
    // const double kv = lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;
    // steer_angle_feedforwardterm = (wheelbase_ * ref_curvature + kv * v * v * ref_curvature - matrix_k_(0, 2) * 
    //                                 (lr_ * ref_curvature - lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_));
    // steer_angle_feedforwardterm = ref_curvature*(wheelbase_-lr_*matrix_k_(0, 2)-
    //                                    mass_*v*v*(lr_/cf_+lf_/cr_*matrix_k_(0, 2)-lf_/cr_)/wheelbase_);//老王
    return steer_angle_feedforwardterm;
} 

// 计算控制
double LQR_control::ComputeControlCommand(const location_info &VehicleState,
                                 const std::vector<planning_trajectory> &planning_published_trajectory,
                                 struct ControlCmd &cmd){
    LoadControlConf();
    Init();
    planning_path=planning_published_trajectory;
    //----------------------------------------------------------------------------------------------------------------------------//
    // A matrix (Gear Drive)
    // [0.0,                               1.0,                            0.0,                                               0.0;
    //  0.0,            (-(c_f + c_r) / m) / v,                (c_f + c_r) / m,                   (l_r * c_r - l_f * c_f) / m / v;
    //  0.0,                               0.0,                            0.0,                                               1.0;
    //  0.0,   ((lr * cr - lf * cf) / i_z) / v,   (l_f * c_f - l_r * c_r) / i_z,   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    //----------------------------------------------------------------------------------------------------------------------------//
    // 配置状态矩阵A
    double v_x = VehicleState.host_speed;
    double v_warning;
    if(v_x< 0.1){
        v_warning=0.1;
    }
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / (v_x + v_warning);    // 避免速度为0的时候，0做分母导致计算失败
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / (v_x + v_warning);
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / (v_x + v_warning);
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / (v_x + v_warning);

    matrix_bd_=matrix_bd_;
    // std::cout<<"matrix_a_ \n"<<matrix_a_<<std::endl;
    // std::cout<<"matrix_bd_ \n"<<matrix_bd_<<std::endl;

    // 计算横向误差并且更新状态向量x
    UpdataState(VehicleState);

    // Solve Lqr Problem
    SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_, lqr_max_iteration_, &matrix_k_);

    //计算feedback, 根据反馈对计算状态变量（误差状态）的时候的符号的理解：K里面的值实际运算中全部为正值，steer = -K *state，
    //按照代码中采用的横向误差的计算方式，横向误差为正值的时候（state中的第一项为正），
    //参考点位于车辆左侧，车辆应该向左转以减小误差，而根据试验，仿真器中，给正值的时候，车辆向右转，给负值的时候，车辆向左转。
    double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0,0);

    // 计算前馈控制，计算横向转角的反馈量
    double steer_angle_feedforward = 0.0;
    steer_angle_feedforward = ComputeFeedForward(VehicleState, ref_curv_);
    double steer_angle = steer_angle_feedback + steer_angle_feedforward;     //LGSVL仿真器是相反的。

    // std::cout<<"matrix_k_ =  "<< matrix_k_ <<std::endl;
    // std::cout<<"matrix_state_ =  "<< matrix_state_.transpose() <<std::endl;
    // std::cout<<"steer_angle_feedback= "<<steer_angle_feedback<<std::endl;
    // std::cout<<"steer_angle_feedforward= "<<steer_angle_feedforward<<std::endl;
    //std::cout<<" sl---------steeer_angle=  "<<steer_angle *180 / 3.1415926<< " deg"<<std::endl;
    // 限制前轮最大转角，这里定义前轮最大转角位于 [-20度～20度].
    if (steer_angle >= atan2_to_PI_sl(20.0)) {
        steer_angle = atan2_to_PI_sl(20.0);
    } else if (steer_angle <= -atan2_to_PI_sl(20.0)) {
        steer_angle = -atan2_to_PI_sl(20.0);
    }
    // Set the steer commands
    cmd.steer_target = steer_angle;
    return steer_angle;
}