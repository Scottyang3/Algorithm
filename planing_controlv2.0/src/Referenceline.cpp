#include "Referenceline.h"

bool ReferenceLine::Clac_heading_kappa_info(std::vector<xy_points_info>& xy_points_,
                                            std::vector<double>* headings, 
                                            std::vector<double>* accumulated_s, 
                                            std::vector<double>* kappas, 
                                            std::vector<double>* dkappas) {
    //-----------------------------------------------//
    // �ú���������path�����߷�����x��ļнǺ�����
    // ���룺path_x,y·������
    // �����path heading kappa ·����heading������
    // ԭ�� heading = arctan(dy/dx);
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
    // �ú�������������x_set��y_set�е�xy����frenet_path�µ�ͶӰ����Ϣ
    // ���� x_set,y_set����ͶӰ�ĵ�ļ���
    // x_set,y_set,frenet_path_x,frenet_path_y,frenet_path_heading,frenet_path_kappa
    // ������ֱ������ϵ�µ�x��y��heading��kappa
    // �����match_point_index_set ƥ�����frenet_path�µı�ŵļ���(����ȫ��·����һ���㿪ʼ�����ڼ�������ƥ���)
    //      proj_x y heading kappa ͶӰ��x,y,heading,kappa�ļ���
    //------------------------------------------------------------------------------------//
    proj_points->clear();
    vector_match_point.resize(2);
    vector_match_point_direction.resize(2);
    vector_r.resize(2);
    vector_d.resize(2);
    if(is_first_run==0){
        //�״�����
        //Ѱ��ƥ���
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
            // if (increase_count>50)//50��̫���ˣ��ҳ��Ŀ��ܲ������Լ�����ĵ�
            // {
            //     break;
            // }
        }
        match_point_x=xy_points[match_points_index].x;
        match_point_y=xy_points[match_points_index].y;
        //����ƥ���ķ��������뷨����
        vector_match_point[0]=match_point_x;    //��������
        vector_match_point[1]=match_point_y;
        vector_match_point_direction[0]=std::cos(match_point_heading);  //������
        vector_match_point_direction[1]=std::sin(match_point_heading);
        //�����ͶӰ���λ��ʸ��
        vector_r[0]=x_set;
        vector_r[1]=y_set;
        //ͨ��ƥ������ͶӰ��
        vector_d[0]=vector_r[0]-vector_match_point[0];
        vector_d[1]=vector_r[1]-vector_match_point[1];
        double ds=vector_d[0]*vector_match_point_direction[0]+vector_d[1]*vector_match_point_direction[1];
        //ƥ���ļ��������棬����һ������ʹ��
        pre_match_point_index = match_points_index;
        //ͶӰ���x y heading kappaΪ:
        proj_points_info proj_p;
        proj_p.proj_match_index=match_points_index;
        proj_p.proj_point_x=vector_match_point[0]+ds*vector_match_point_direction[0];
        proj_p.proj_point_y=vector_match_point[1]+ds*vector_match_point_direction[1];
        proj_p.proj_heading=match_point_heading+match_point_kappa*ds;
        proj_p.proj_kappa=match_point_kappa;
        proj_points->push_back(proj_p);
    }
    else{
        //�����״�����
        //��һ���ڵ�����
        start_search_index=pre_match_point_index;   //����ʼѰ����������Ϊ��һ���ڵ�����
        pre_frenet_path_x=xy_points[start_search_index].x;
        pre_frenet_path_y=xy_points[start_search_index].y;
        //�жϱ����ķ���
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
        //����ƥ���ķ��������뷨����
        vector_match_point[0]=match_point_x;    //��������
        vector_match_point[1]=match_point_y;
        vector_match_point_direction[0]=std::cos(match_point_heading);  //������
        vector_match_point_direction[1]=std::sin(match_point_heading);
        //�����ͶӰ���λ��ʸ��
        vector_r[0]=x_set;
        vector_r[1]=y_set;
        //ͨ��ƥ������ͶӰ��
        vector_d[0]=vector_r[0]-vector_match_point[0];
        vector_d[1]=vector_r[1]-vector_match_point[1];
        double ds=vector_d[0]*vector_match_point_direction[0]+vector_d[1]*vector_match_point_direction[1];
        //ƥ���ļ��������棬����һ������ʹ��
        pre_match_point_index = match_points_index;
        //ͶӰ���x y heading kappaΪ:
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
    // �ú�������ȫ��·������ȡreference line��δƽ���ĳ�ֵ
    // ���룺 host_match_point_index �Գ���λ����ȫ��·����ƥ���ı��
    //        global_path_x,global_path_y ȫ��·������
    // �����reference line_x_init,reference line_y_init δƽ���Ĳο��ߵ�xy����
    // ����global path ��ÿ1mȡһ���㣬���Դ�ƥ�������ȡ30���㣬��ǰȡ150���㼴�ɣ�һ��181����
    // ����ĵ㲻���Ļ�����ǰ��ĵ㲹��ǰ��ĵ㲻���Ļ����ú���ĵ㲹����֤������181��
    //--------------------------------------------------------------------------------------//
    xy_points->clear();
    //������ʼ��
    int start_index=-1;
    int global_path_length=global_xy_points.size();
    //�жϺ���ǰ��ĵ��Ƿ��㹻��
    if (host_match_point_index-30<1)
    {
        //ƥ������ĵ�̫���ˣ�����30��
        start_index = 1;
    }
    else if(host_match_point_index + 150 > global_path_length){
        //ƥ���ǰ��ĵ�̫���ˣ�����150��
        start_index = global_path_length-180;
    }
    else{
        //ƥ�����������
        start_index=host_match_point_index - 30;
    }
    //��ȡreferenceline
    int index_count=0;
    if(global_path_length<start_index+180){
        index_count=global_path_length;
    }else{
        index_count=start_index+180;
    }
    //�������
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
    //ƽ�������ʽΪ:
    //J = j1+j2+j3=
    //    w1*x'*A1'*A1*x+w2*x'*A2'*A2*x+w3*x'*x-2*h'*x=
    //    x'*(w1*A1'*A1+w2*A2'*A2+w3*E)*x-2*h'*x
    //H = 2*(w1*A1'*A1+w2*A2'*A2+w3*E)
    //f = -2h
    //Լ������Ϊ:
    //lb<=|x-xr|<=ub
    //��Ϊ:lb+xr <= x <= ub+xr
    // TODO ��ʼ��
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
    // ����H����
    hessian.resize(matrix_H.rows(), matrix_H.cols());
    for (size_t i = 0; i < matrix_H.rows(); i++)
    {
        for (size_t j = 0; j < matrix_H.cols(); j++)
        {
            hessian.insert(i, j) = matrix_H(i, j);
        }
    }
    // ����f����
    gradient.resize(2 * xy_points_length);
    for (size_t i = 0; i < xy_points_length; i++)
    {
        gradient(2 * i) = -2 * weight_Uniformity * xy_points[i].x;
        gradient(2 * i + 1) = -2 * weight_Uniformity * xy_points[i].y;
    }
    // Լ������A����
    linearMatrix.resize(2 * xy_points_length, 2 * xy_points_length);
    for (size_t i = 0; i < 2 * xy_points_length; i++)
    {
        linearMatrix.insert(i, i) = 1;
    }
    // std::cout << "matrix_H=  \n" << matrix_H << std::endl;
    // std::cout << "hessian=  \n" << hessian << std::endl;
    // std::cout << "gradient=  \n" << gradient << std::endl;
    // std::cout << "linearMatrix=  \n" << linearMatrix << std::endl;

    // ���±߽������
    for (size_t i = 0; i < xy_points_length; i++)
    {
        matrix_f(2 * i, 0) = xy_points[i].x;
        matrix_f(2 * i + 1, 0) = xy_points[i].y;
        lowerBound(2 * i + 1) = matrix_f(2 * i + 1, 0) + x_lb;
        upperBound(2 * i + 1) = matrix_f(2 * i + 1, 0) + x_ub;
        lowerBound(2 * i) = matrix_f(2 * i, 0) + x_lb;
        upperBound(2 * i) = matrix_f(2 * i, 0) + x_ub;
    }

    int NumberOfVariables = 2 * xy_points_length; //A���������
    int NumberOfConstraints = 2 * xy_points_length; //A���������
    // ʵ����������
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    // ����QP�������ĳ�ʼ����
    //����AΪm*n����
    solver.data()->setNumberOfVariables(NumberOfVariables); //����A�������������n
    solver.data()->setNumberOfConstraints(NumberOfConstraints); //����A�������������m
    if (!solver.data()->setHessianMatrix(hessian)) return 1;//����H����
    if (!solver.data()->setGradient(gradient)) return 1; //����f���󡣵�û��ʱ����Ϊȫ0����
    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;//��������Լ����A����
    if (!solver.data()->setLowerBound(lowerBound)) return 1;//�����±߽�
    if (!solver.data()->setUpperBound(upperBound)) return 1;//�����ϱ߽�
    // ʵ����������
    if (!solver.initSolver()) return 1;
    Eigen::VectorXd QPSolution;
    // ���QP����
    if (!solver.solve()) return 1;
    // ��ȡ����������
    QPSolution = solver.getSolution();
    //ƽ����ĵ�Ĵ洢
    for (size_t i = 0; i < xy_points_length; i++)
    {
        fem_points.x=QPSolution(2*i);
        fem_points.y=QPSolution(2*i+1);
        fem_smoothing_line->push_back(fem_points);
    }
    // std::cout << "QPSolution:" << std::endl << QPSolution(2*i)<< std::endl;
    return 1;
}
