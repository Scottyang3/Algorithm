#include "LQR_control.h"

// ����֮��ľ���
double PointDistanceSquare(const planning_trajectory &point, const double x, const double y) {
    double dx = point.trajectory_x - x;
    double dy = point.trajectory_y - y;
    return dx * dx + dy * dy;
}

// ���Ƕ�(������)�黯��[-M_PI, M_PI]֮��
double NormalizeAngle_sl(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI); //���ҳ���������
    if (a < 0.0) {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

// ���Ƕ�ת��Ϊ������
double atan2_to_PI_sl(const double atan2) { 
    return atan2 * M_PI / 180; 
}

// ���س�������
void LQR_control::LoadControlConf(){
    ts_=0.01;   //ÿ��0.01s����һ�ο���
    cf_ = 155494.663;                              // ǰ�ֲ�ƫ�ն�,������֮��
    cr_ = 155494.663;                              // ���ֲ�ƫ�ն�, ������֮��
    wheelbase_ = 2.852;                            // �����ֵľ���
    steer_ratio_ = 16;                             // �����̵�ת�ǵ���̥ת���Ƕ�֮��ı�ֵϵ��
    steer_single_direction_max_degree_ = 470.0;    // �����ת��
    const double mass_fl = 520;                    // ��ǰ��������
    const double mass_fr = 520;                    // ��ǰ��������
    const double mass_rl = 520;                    // �����������
    const double mass_rr = 520;                    // �Һ���������
    const double mass_front = mass_fl + mass_fr;   // ǰ������
    const double mass_rear = mass_rl + mass_rr;    // ��������
    mass_ = mass_front + mass_rear;                //������
    lf_ = wheelbase_ * (1.0 - mass_front / mass_);    // ����ǰ�ֵ����ĵ�ľ���
    lr_ = wheelbase_ * (1.0 - mass_rear / mass_);     // �������ֵ����ĵ�ľ���
    // moment of inertia
    iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;    // ������ת������

    lqr_eps_ = 0.01;              // LQR ������⾫��
    lqr_max_iteration_ = 1500;    // LQR�ĵ������� 
}

// ��ʼ��״̬����
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
    // ��ʼ��A����ĳ�����
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    // ��ʼ��A����ķǳ�����
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
    // ��ʼ��B����
    matrix_b_ = Matrix::Zero(basic_state_size_, 1);
    matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_bd_ = matrix_b_ * ts_;

    // ״̬����
    matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    // ��������
    matrix_k_ = Matrix::Zero(1, basic_state_size_);
    // lqr cost function�� ����ֵu��Ȩ��
    matrix_r_ = Matrix::Identity(1, 1);
    matrix_r_(0, 0) = 10;
    // lqr cost function�� ״̬����x��Ȩ��
    matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);

    // int q_param_size = 4;
    matrix_q_(0, 0) = 25;   // lateral_error
    matrix_q_(1, 1) = 3;    // lateral_error_rate
    matrix_q_(2, 2) = 10;    // heading_error
    matrix_q_(3, 3) = 4;    // heading__error_rate
}

// ����ƥ������Ϣ
planning_trajectory LQR_control::QueryNearestPointByPosition(const double x, const double y) {
    double d_min = 1e6; //��С�����ʼ��
    size_t index_min = 0;
    for (size_t i = 0; i < planning_path.size(); i++) {
        double d_temp = PointDistanceSquare(planning_path[i], x, y);
        if (d_temp < d_min) {
            d_min = d_temp;
            index_min = i;
        }
    }
    ref_curv_ = planning_path[index_min].trajectory_kappa;    // ��Ӧ������Ĺ켣���ϵ�����

    double front_index = index_min + 5; //ǰ��5����
    if (front_index > planning_path.size()){
        ref_curv_front_ = planning_path[index_min].trajectory_kappa;
    }
    else{
        ref_curv_front_ = planning_path[front_index].trajectory_kappa;

    }
    return planning_path[index_min];    //��ʱ���ص��ǵ�ǰ�����һ���㣬��Ҫǰ�ӽ�index_min��Ϊfront_index
}

// �������
void LQR_control::ComputeErrors(const double x,
                                const double y,
                                const double theta,
                                const double linear_v,
                                const double angular_v,
                                const double linear_a,
                                LateralControlErrorPtr &lat_con_error){
    double pre_x = x+linear_v*0.1*std::cos(theta); //Ԥ�ⲿ��
    double pre_y = y+linear_v*0.1*std::sin(theta);
    planning_trajectory match_points=this->QueryNearestPointByPosition(pre_x, pre_y);
    // ��������ϵ��X�����ų���������ǰΪ����Y���ų�����������Ϊ�����ӳ�ͷ��ǰ�����ӽǣ����ڳ�������ϵ�£����복�������·����λ�ڳ�����࣬����Ӧ����ת�Ը��ٲο�·��
    // �����������ʱ����Ҫ��·���Ͼ��복������ĵ����������ϵ�任����������ϵ�£�·�����ڳ�������ϵ�µĺ�������Ǻ���λ��������·�����ڳ�������ϵ�µĺ��������������ǰ��ת�ǵķ���
    // double closest_point_x_in_vehicle_coordinate = (current_closest_point.x - x) * cos(theta) + (current_closest_point.y - y) * sin(theta);
    double e_y = -(match_points.trajectory_x - x) * sin(match_points.trajectory_heading) + (match_points.trajectory_y - y) * cos(match_points.trajectory_heading);
    // �������
    double e_x = (match_points.trajectory_x - x) * cos(match_points.trajectory_heading) + (match_points.trajectory_y- y) * sin(match_points.trajectory_heading);
    double e_theta = match_points.trajectory_heading+match_points.trajectory_kappa *e_x - theta;
    // double e_theta = current_closest_point.heading - theta;    // ·���Ͼ��복������ĵ�Ĳο�����ǣ����ڳ����ĵ�ǰ����ǵĻ�������Ӧ��ת�Ը��ٺ���
    // ����ǰ��ת�ǵ�ȡֵ����
    if (e_theta > M_PI) {
        e_theta = e_theta - M_PI * 2;
    }
    if (e_theta < -M_PI) {
        e_theta = e_theta + M_PI * 2;
    }

    double e_y_dot = linear_v * sin(e_theta);
    // double e_theta_dot = angular_v - current_closest_point.kappa * current_closest_point.v;
    double e_theta_dot = angular_v - match_points.trajectory_kappa * (match_points.trajectory_speed *cos(e_theta)/(1-match_points.trajectory_kappa *e_y));
    // std::cout<<"\n -----����ƫ��Ϊ:"<< e_y<<"\n -----����ƫ��Ϊ:"<< e_theta<<std::endl;
    lat_con_error->lateral_error = e_y;
    lat_con_error->lateral_error_rate = e_y_dot;
    lat_con_error->heading_error = e_theta;
    lat_con_error->heading_error_rate = e_theta_dot;
}

// ����״̬
void LQR_control::UpdataState(const location_info &VehicleState){
    // LateralControlError lat_con_error;  // �������Ϊ����ָ��
    std::shared_ptr<LateralControlError> lat_con_error = std::make_shared<LateralControlError>();
    // ����������
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
    // ����״̬����A����״̬����A��ɢ��
    Eigen::MatrixXd matrix_a_temp = Eigen::MatrixXd::Identity(matrix_a_.rows(), matrix_a_.cols());
    matrix_ad_ = matrix_a_ * ts_ + matrix_a_temp;
}

// ���LQR����
void LQR_control::SolveLQRProblem(const Matrix &A, const Matrix &B, const Matrix &Q, const Matrix &R, const double tolerance, const uint max_num_iteration, Matrix *ptr_K) {
    // ��ֹ�����ά�������º���������ʧ��
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

// ����ǰ������
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
    //                                    mass_*v*v*(lr_/cf_+lf_/cr_*matrix_k_(0, 2)-lf_/cr_)/wheelbase_);//����
    return steer_angle_feedforwardterm;
} 

// �������
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
    // ����״̬����A
    double v_x = VehicleState.host_speed;
    double v_warning;
    if(v_x< 0.1){
        v_warning=0.1;
    }
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / (v_x + v_warning);    // �����ٶ�Ϊ0��ʱ��0����ĸ���¼���ʧ��
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / (v_x + v_warning);
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / (v_x + v_warning);
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / (v_x + v_warning);

    matrix_bd_=matrix_bd_;
    // std::cout<<"matrix_a_ \n"<<matrix_a_<<std::endl;
    // std::cout<<"matrix_bd_ \n"<<matrix_bd_<<std::endl;

    // ����������Ҹ���״̬����x
    UpdataState(VehicleState);

    // Solve Lqr Problem
    SolveLQRProblem(matrix_ad_, matrix_bd_, matrix_q_, matrix_r_, lqr_eps_, lqr_max_iteration_, &matrix_k_);

    //����feedback, ���ݷ����Լ���״̬���������״̬����ʱ��ķ��ŵ���⣺K�����ֵʵ��������ȫ��Ϊ��ֵ��steer = -K *state��
    //���մ����в��õĺ������ļ��㷽ʽ���������Ϊ��ֵ��ʱ��state�еĵ�һ��Ϊ������
    //�ο���λ�ڳ�����࣬����Ӧ������ת�Լ�С�����������飬�������У�����ֵ��ʱ�򣬳�������ת������ֵ��ʱ�򣬳�������ת��
    double steer_angle_feedback = -(matrix_k_ * matrix_state_)(0,0);

    // ����ǰ�����ƣ��������ת�ǵķ�����
    double steer_angle_feedforward = 0.0;
    steer_angle_feedforward = ComputeFeedForward(VehicleState, ref_curv_);
    double steer_angle = steer_angle_feedback + steer_angle_feedforward;     //LGSVL���������෴�ġ�

    // std::cout<<"matrix_k_ =  "<< matrix_k_ <<std::endl;
    // std::cout<<"matrix_state_ =  "<< matrix_state_.transpose() <<std::endl;
    // std::cout<<"steer_angle_feedback= "<<steer_angle_feedback<<std::endl;
    // std::cout<<"steer_angle_feedforward= "<<steer_angle_feedforward<<std::endl;
    //std::cout<<" sl---------steeer_angle=  "<<steer_angle *180 / 3.1415926<< " deg"<<std::endl;
    // ����ǰ�����ת�ǣ����ﶨ��ǰ�����ת��λ�� [-20�ȡ�20��].
    if (steer_angle >= atan2_to_PI_sl(20.0)) {
        steer_angle = atan2_to_PI_sl(20.0);
    } else if (steer_angle <= -atan2_to_PI_sl(20.0)) {
        steer_angle = -atan2_to_PI_sl(20.0);
    }
    // Set the steer commands
    cmd.steer_target = steer_angle;
    return steer_angle;
}