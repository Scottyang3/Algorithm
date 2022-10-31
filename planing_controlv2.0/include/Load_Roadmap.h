#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cassert>
#include "struct_common.h"

class PathPlanningNode
{
private:
    xy_points_info xy_load;//路网点
public:
    bool loadRoadmap(const std::string &roadmap_path, std::vector<xy_points_info> *xy_points);
};
