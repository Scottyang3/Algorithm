#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdio>
#include<cassert>

class PathPlanningNode
{
private:
    xy_points_info xy_load;//·����
public:
    bool loadRoadmap(const std::string &roadmap_path, std::vector<xy_points_info> *xy_points);
};

//����·����ͼ
bool PathPlanningNode::loadRoadmap(const std::string &roadmap_path, std::vector<xy_points_info> *xy_points){
    // ��ȡ�ο���·��
    std::ifstream infile;
    infile.open(roadmap_path);    //���ļ����������ļ���������
    assert(infile.is_open());   //����Ƿ���ļ��ɹ�������������Բ��ã�Ƶ�����ûἫ��Ӱ���������ܣ�
    std::string s, x, y;
    while (getline(infile, s)) {
        std::stringstream word(s);
        word >> x;
        word >> y;
        double pt_x = std::atof(x.c_str());
        double pt_y = std::atof(y.c_str());
        xy_load.x=pt_x;
        xy_load.y=pt_y;
        xy_points->push_back(xy_load);
    }
    infile.close();
}

