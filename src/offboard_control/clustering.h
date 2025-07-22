#pragma once
#include <vector>
#include "OffboardControl.h"

struct Points 
{
    Vector3d point;
    int cluster_id;
    double diameters; // 直径属性
};

// 声明全局变量
extern std::vector<Points> Target_Samples;

double distance(Vector3d a, Vector3d b);
std::vector<Vector3d> calculate_center(std::vector<Points> samples);
std::vector<Points> Initialize_Clustering(std::vector<Points> samples);
std::vector<Points> Clustering(std::vector<Points> samples);