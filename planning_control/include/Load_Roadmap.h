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
    xy_points_info xy_load;//路网点
public:
    bool loadRoadmap(const std::string &roadmap_path, std::vector<xy_points_info> *xy_points);
};

//加载路网地图
bool PathPlanningNode::loadRoadmap(const std::string &roadmap_path, std::vector<xy_points_info> *xy_points){
    // 读取参考线路径
    std::ifstream infile;
    infile.open(roadmap_path);    //将文件流对象与文件连接起来
    assert(infile.is_open());   //检测是否打开文件成功（如果正常可以不用，频繁调用会极大影响程序的性能）
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

