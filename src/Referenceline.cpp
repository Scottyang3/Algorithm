#include "Referenceline.h"

bool ReferenceLine::Clac_heading_kappa_info(std::vector<xy_points_info>& xy_points_,
                                            std::vector<double>* headings, 
                                            std::vector<double>* accumulated_s, 
                                            std::vector<double>* kappas, 
                                            std::vector<double>* dkappas) {
    //-----------------------------------------------//
    // 该函数将计算path的切线方向与x轴的夹角和曲率
    // 输入：path_x,y路径坐标
    // 输出：path heading kappa 路径的heading和曲率
    // 原理 heading = arctan(dy/dx);
    //      kappa = dheading/ds;
    //      ds = (dx^2 + dy^2)^0.5
    //-----------------------------------------------//
    
    headings->clear();
    accumulated_s->clear();
    kappas->clear();
    dkappas->clear();
    if (xy_points_.size() < 2) {
        return false;
    }
    std::vector<double> dxs;
    std::vector<double> dys;
    std::vector<double> y_over_s_first_derivatives;
    std::vector<double> x_over_s_first_derivatives;
    std::vector<double> y_over_s_second_derivatives;
    std::vector<double> x_over_s_second_derivatives;

    //Calculate dx dy heading kappa
    int points_size = xy_points_.size();
    for (size_t i = 0; i < points_size; i++)
    {
        double x_delta;
        double y_delta;
        if (i == 0) {
            x_delta = (xy_points_[i + 1].x - xy_points_[i].x);
            y_delta = (xy_points_[i + 1].y - xy_points_[i].y);
        } else if (i == points_size - 1) {
            x_delta = (xy_points_[i].x - xy_points_[i - 1].x);
            y_delta = (xy_points_[i].y - xy_points_[i - 1].y);
        } else {
            x_delta = 0.5 * (xy_points_[i + 1].x - xy_points_[i - 1].x);
            y_delta = 0.5 * (xy_points_[i + 1].y - xy_points_[i - 1].y);
        }
        dxs.push_back(x_delta);
        dys.push_back(y_delta);
    }
    // Heading calculation
    for (size_t i = 0; i < points_size; i++)
    {
        headings->push_back(std::atan2(dys[i],dxs[i]));
    }

    // accumulated_s calculation
    double distance=0.0;
    accumulated_s->push_back(distance);
    double fx = xy_points_[0].x;
    double fy = xy_points_[0].y;
    double nx = 0.0;
    double ny = 0.0;
    for (size_t i = 1; i < points_size; i++)
    {
        nx = xy_points_[i].x;
        ny = xy_points_[i].y;
        double end_segment_s = std::sqrt((nx-fx)*(nx-fx)+(ny-fy)*(ny-fy));
        accumulated_s->push_back(end_segment_s+distance);
        distance +=end_segment_s;
        fx = nx;
        fy = ny;
    }

    // x y respective to s derivative calculation
    for (std::size_t i = 0; i < points_size; i++) {
        double xds = 0.0;
        double yds = 0.0;
        if (i == 0) {
            xds = (xy_points_[i + 1].x - xy_points_[i].x) / (accumulated_s->at(i + 1) - accumulated_s->at(i)+1e-6);
            yds = (xy_points_[i + 1].y - xy_points_[i].y) / (accumulated_s->at(i + 1) - accumulated_s->at(i)+1e-6);
        } else if (i == points_size - 1) {
            xds = (xy_points_[i].x - xy_points_[i - 1].x) / (accumulated_s->at(i) - accumulated_s->at(i - 1)+1e-6);
            yds = (xy_points_[i].y - xy_points_[i - 1].y) / (accumulated_s->at(i) - accumulated_s->at(i - 1)+1e-6);
        } else {
            xds = (xy_points_[i + 1].x - xy_points_[i - 1].x) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1)+1e-6);
            yds = (xy_points_[i + 1].y - xy_points_[i - 1].y) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1)+1e-6);
        }
        x_over_s_first_derivatives.push_back(xds);
        y_over_s_first_derivatives.push_back(yds);
    }
    for (std::size_t i = 0; i < points_size; i++) {
        double xdds = 0.0;
        double ydds = 0.0;
        if (i == 0) {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) / (accumulated_s->at(i + 1) - accumulated_s->at(i)+1e-6);
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) / (accumulated_s->at(i + 1) - accumulated_s->at(i)+1e-6);
        } else if (i == points_size - 1) {
            xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i) - accumulated_s->at(i - 1)+1e-6);
            ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i) - accumulated_s->at(i - 1)+1e-6);
        } else {
            xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1)+1e-6);
            ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1)+1e-6);
        }
        x_over_s_second_derivatives.push_back(xdds);
        y_over_s_second_derivatives.push_back(ydds);
    }

    // kappa calculation
    for (std::size_t i = 0; i < points_size; i++) {
        double xds = x_over_s_first_derivatives[i];
        double yds = y_over_s_first_derivatives[i];
        double xdds = x_over_s_second_derivatives[i];
        double ydds = y_over_s_second_derivatives[i];
        double kappa = (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
        // std::cout<<"kappa= "<<std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds)<<std::endl;
        // std::cout<<"xds= "<<xds<<std::endl;
        // std::cout<<"yds= "<<yds<<std::endl;
        kappas->push_back(kappa);
    }

    // dkappa calculation
    for (std::size_t i = 0; i < points_size; i++) {
        double dkappa = 0.0;
        if (i == 0) {
            dkappa = (kappas->at(i + 1) - kappas->at(i)) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
        } else if (i == points_size - 1) {
            dkappa = (kappas->at(i) - kappas->at(i - 1)) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
        } else {
            dkappa = (kappas->at(i + 1) - kappas->at(i - 1)) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
        }
        dkappas->push_back(dkappa);
    }
    return true;
}


void ReferenceLine::Clac_proj_points_info(double &x_set,
                                          double &y_set,
                                          const std::vector<xy_points_info>& xy_points,
                                          std::vector<proj_points_info> *proj_points){
    //-----------------------------------------------------------------------------------//
    // 该函数将批量计算x_set，y_set中的xy，在frenet_path下的投影的信息
    // 输入 x_set,y_set，待投影的点的集合
    // x_set,y_set,frenet_path_x,frenet_path_y,frenet_path_heading,frenet_path_kappa
    // 曲线在直角坐标系下的x，y，heading，kappa
    // 输出：match_point_index_set 匹配点在frenet_path下的编号的集合(即从全局路径第一个点开始数，第几个点是匹配点)
    //      proj_x y heading kappa 投影的x,y,heading,kappa的集合
    //------------------------------------------------------------------------------------//
    proj_points->clear();
    vector_match_point.resize(2);
    vector_match_point_direction.resize(2);
    vector_r.resize(2);
    vector_d.resize(2);
    if(is_first_run==0){
        //首次运行
        //寻找匹配点
        double min_distance = 1e6;
        int length_frenet_path=xy_points.size();
        for (size_t i = start_search_index; i < length_frenet_path; i++)
        {
            distance = std::sqrt((x_set-xy_points[i].x)*(x_set-xy_points[i].x)+(y_set-xy_points[i].y)*(y_set-xy_points[i].y));
            if (distance<min_distance)
            {
                min_distance = distance;
                match_points_index = i;
                increase_count = 0;
            }else{
                increase_count +=1;
            }
            // if (increase_count>50)//50个太少了，找出的可能不是离自己最近的点
            // {
            //     break;
            // }
        }
        match_point_x=xy_points[match_points_index].x;
        match_point_y=xy_points[match_points_index].y;
        //计算匹配点的方向向量与法向量
        vector_match_point[0]=match_point_x;    //方向向量
        vector_match_point[1]=match_point_y;
        vector_match_point_direction[0]=std::cos(match_point_heading);  //法向量
        vector_match_point_direction[1]=std::sin(match_point_heading);
        //计算待投影点的位置矢量
        vector_r[0]=x_set;
        vector_r[1]=y_set;
        //通过匹配点计算投影点
        vector_d[0]=vector_r[0]-vector_match_point[0];
        vector_d[1]=vector_r[1]-vector_match_point[1];
        double ds=vector_d[0]*vector_match_point_direction[0]+vector_d[1]*vector_match_point_direction[1];
        //匹配点的计算结果保存，供下一个周期使用
        pre_match_point_index = match_points_index;
        //投影点的x y heading kappa为:
        proj_points_info proj_p;
        proj_p.proj_match_index=match_points_index;
        proj_p.proj_point_x=vector_match_point[0]+ds*vector_match_point_direction[0];
        proj_p.proj_point_y=vector_match_point[1]+ds*vector_match_point_direction[1];
        proj_p.proj_heading=match_point_heading+match_point_kappa*ds;
        proj_p.proj_kappa=match_point_kappa;
        proj_points->push_back(proj_p);
    }
    else{
        //不是首次运行
        //上一周期的数据
        start_search_index=pre_match_point_index;   //将开始寻找索引设置为上一周期的索引
        pre_frenet_path_x=xy_points[start_search_index].x;
        pre_frenet_path_y=xy_points[start_search_index].y;
        //判断遍历的方向
        double flag = (x_set-pre_frenet_path_x)*cos(pre_frenet_path_heading)+(y_set-pre_frenet_path_y)*sin(pre_frenet_path_heading);
        double min_distance = 1e6;
        int length_frenet_path=xy_points.size();
        if(flag>0.001){
            for (size_t i = start_search_index; i < length_frenet_path; i++)
            {
                distance = std::sqrt((x_set-xy_points[i].x)*(x_set-xy_points[i].x)+(y_set-xy_points[i].y)*(y_set-xy_points[i].y));
                if (distance<min_distance)
                {
                    min_distance = distance;
                    match_points_index = i;
                    increase_count = 0;
                }else{
                    increase_count +=1;
                }
                if (increase_count>50)
                {
                    break;
                }
            }
        }
        else if(flag<-0.001){
            for (size_t i = start_search_index; i >= 0; i--)
            {
                distance = std::sqrt((x_set-xy_points[i].x)*(x_set-xy_points[i].x)+(y_set-xy_points[i].y)*(y_set-xy_points[i].y));
                if (distance<min_distance)
                {
                    min_distance = distance;
                    match_points_index = i;
                    increase_count = 0;
                }else{
                    increase_count +=1;
                }
                if (increase_count>50)
                {
                    break;
                }
            }
        }
        else{
            match_points_index = start_search_index;
        }
        match_point_x=xy_points[match_points_index].x;
        match_point_y=xy_points[match_points_index].y;
        //计算匹配点的方向向量与法向量
        vector_match_point[0]=match_point_x;    //方向向量
        vector_match_point[1]=match_point_y;
        vector_match_point_direction[0]=std::cos(match_point_heading);  //法向量
        vector_match_point_direction[1]=std::sin(match_point_heading);
        //计算待投影点的位置矢量
        vector_r[0]=x_set;
        vector_r[1]=y_set;
        //通过匹配点计算投影点
        vector_d[0]=vector_r[0]-vector_match_point[0];
        vector_d[1]=vector_r[1]-vector_match_point[1];
        double ds=vector_d[0]*vector_match_point_direction[0]+vector_d[1]*vector_match_point_direction[1];
        //匹配点的计算结果保存，供下一个周期使用
        pre_match_point_index = match_points_index;
        //投影点的x y heading kappa为:
        proj_points_info proj_p;
        proj_p.proj_match_index=match_points_index;
        proj_p.proj_point_x=vector_match_point[0]+ds*vector_match_point_direction[0];
        proj_p.proj_point_y=vector_match_point[1]+ds*vector_match_point_direction[1];
        proj_p.proj_heading=match_point_heading+match_point_kappa*ds;
        proj_p.proj_kappa=match_point_kappa;
        proj_points->push_back(proj_p);
    }
};

void ReferenceLine::intercept_referenceline(int host_match_point_index,
                                          std::vector<xy_points_info>& global_xy_points,
                                          std::vector<xy_points_info>* xy_points){
    //-------------------------------------------------------------------------------------//
    // 该函数将在全局路径上提取reference line的未平滑的初值
    // 输入： host_match_point_index 自车的位置在全局路径的匹配点的编号
    //        global_path_x,global_path_y 全局路径数据
    // 输出：reference line_x_init,reference line_y_init 未平滑的参考线的xy坐标
    // 由于global path 是每1m取一个点，所以从匹配点往后取30个点，往前取150个点即可，一共181个点
    // 后面的点不够的话就用前面的点补，前面的点不够的话就用后面的点补，保证总数是181个
    //--------------------------------------------------------------------------------------//
    xy_points->clear();
    //索引初始化
    int start_index=-1;
    int global_path_length=global_xy_points.size();
    //判断后面前面的点是否足够多
    if (host_match_point_index-30<1)
    {
        //匹配点后面的点太少了，不够30个
        start_index = 1;
    }
    else if(host_match_point_index + 150 > global_path_length){
        //匹配点前面的点太少了，不够150个
        start_index = global_path_length-180;
    }
    else{
        //匹配点数量正常
        start_index=host_match_point_index - 30;
    }
    //提取referenceline
    int index_count=0;
    if(global_path_length<start_index+180){
        index_count=global_path_length;
    }else{
        index_count=start_index+180;
    }
    //输出部分
    xy_points_info fre_output;
    for (size_t i = start_index; i < index_count; i++)
    {
        fre_output.x=global_xy_points[i].x;
        fre_output.y=global_xy_points[i].y;
        xy_points->push_back(fre_output);
    }
}

bool ReferenceLine::Fem_smoothing(const std::vector<xy_points_info>& xy_points, std::vector<xy_points_info> *fem_smoothing_line) {
    fem_smoothing_line->clear();
    // int xy_points_length = xy_points.size();
    int xy_points_length = xy_points.size();
    if (xy_points_length < 2) {
        return false;
    }
    //平滑求解形式为:
    //J = j1+j2+j3=
    //    w1*x'*A1'*A1*x+w2*x'*A2'*A2*x+w3*x'*x-2*h'*x=
    //    x'*(w1*A1'*A1+w2*A2'*A2+w3*E)*x-2*h'*x
    //H = 2*(w1*A1'*A1+w2*A2'*A2+w3*E)
    //f = -2h
    //约束条件为:
    //lb<=|x-xr|<=ub
    //即为:lb+xr <= x <= ub+xr
    // TODO 初始化
    matrix_A1 = Matrix::Zero(2 * xy_points_length - 4, 2 * xy_points_length);
    matrix_A2 = Matrix::Zero(2 * xy_points_length - 2, 2 * xy_points_length);
    matrix_A3 = Matrix::Zero(2 * xy_points_length, 2 * xy_points_length);
    matrix_f = Matrix::Zero(2 * xy_points_length, 1);
    lowerBound.resize(2 * xy_points_length);
    upperBound.resize(2 * xy_points_length);

    for (int i = 0; i < 2 * xy_points_length - 5; i++)
    {
        matrix_A1(i, i) = 1;
        matrix_A1(i, i + 2) = -2;
        matrix_A1(i, i + 4) = 1;
        matrix_A1(i + 1, i + 1) = 1;
        matrix_A1(i + 1, i + 3) = -2;
        matrix_A1(i + 1, i + 5) = 1;
    }
    for (int i = 0; i < 2 * xy_points_length - 3; i++)
    {
        matrix_A2(i, i) = 1;
        matrix_A2(i, i + 2) = -1;
        matrix_A2(i + 1, i + 1) = 1;
        matrix_A2(i + 1, i + 3) = -1;
    }
    for (size_t i = 0; i < 2 * xy_points_length; i++)
    {
        matrix_A3(i, i) = 1;
    }

    matrix_H = 2 * (weight_smoothing * (matrix_A1.transpose() * matrix_A1)) + weight_similarity * (matrix_A2.transpose() * matrix_A2) + weight_Uniformity * matrix_A3;
    // 矩阵H设置
    hessian.resize(matrix_H.rows(), matrix_H.cols());
    for (size_t i = 0; i < matrix_H.rows(); i++)
    {
        for (size_t j = 0; j < matrix_H.cols(); j++)
        {
            hessian.insert(i, j) = matrix_H(i, j);
        }
    }
    // 矩阵f设置
    gradient.resize(2 * xy_points_length);
    for (size_t i = 0; i < xy_points_length; i++)
    {
        gradient(2 * i) = -2 * weight_Uniformity * xy_points[i].x;
        gradient(2 * i + 1) = -2 * weight_Uniformity * xy_points[i].y;
    }
    // 约束矩阵A设置
    linearMatrix.resize(2 * xy_points_length, 2 * xy_points_length);
    for (size_t i = 0; i < 2 * xy_points_length; i++)
    {
        linearMatrix.insert(i, i) = 1;
    }
    // std::cout << "matrix_H=  \n" << matrix_H << std::endl;
    // std::cout << "hessian=  \n" << hessian << std::endl;
    // std::cout << "gradient=  \n" << gradient << std::endl;
    // std::cout << "linearMatrix=  \n" << linearMatrix << std::endl;

    // 上下边界的设置
    for (size_t i = 0; i < xy_points_length; i++)
    {
        matrix_f(2 * i, 0) = xy_points[i].x;
        matrix_f(2 * i + 1, 0) = xy_points[i].y;
        lowerBound(2 * i + 1) = matrix_f(2 * i + 1, 0) + x_lb;
        upperBound(2 * i + 1) = matrix_f(2 * i + 1, 0) + x_ub;
        lowerBound(2 * i) = matrix_f(2 * i, 0) + x_lb;
        upperBound(2 * i) = matrix_f(2 * i, 0) + x_ub;
    }

    int NumberOfVariables = 2 * xy_points_length; //A矩阵的列数
    int NumberOfConstraints = 2 * xy_points_length; //A矩阵的行数
    // 实例化解算器
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    // 设置QP解算器的初始数据
    //矩阵A为m*n矩阵
    solver.data()->setNumberOfVariables(NumberOfVariables); //设置A矩阵的列数，即n
    solver.data()->setNumberOfConstraints(NumberOfConstraints); //设置A矩阵的行数，即m
    if (!solver.data()->setHessianMatrix(hessian)) return 1;//设置H矩阵
    if (!solver.data()->setGradient(gradient)) return 1; //设置f矩阵。当没有时设置为全0向量
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//设置线性约束的A矩阵
    if (!solver.data()->setLowerBound(lowerBound)) return 1;//设置下边界
    if (!solver.data()->setUpperBound(upperBound)) return 1;//设置上边界
    // 实例化解算器
    if (!solver.initSolver()) return 1;
    Eigen::VectorXd QPSolution;
    // 解决QP问题
    if (!solver.solve()) return 1;
    // 获取控制器输入
    QPSolution = solver.getSolution();
    //平滑后的点的存储
    for (size_t i = 0; i < xy_points_length; i++)
    {
        fem_points.x=QPSolution(2*i);
        fem_points.y=QPSolution(2*i+1);
        fem_smoothing_line->push_back(fem_points);
    }
    // std::cout << "QPSolution:" << std::endl << QPSolution(2*i)<< std::endl;
    return 1;
}
