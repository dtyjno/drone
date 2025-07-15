#pragma once
#include <vector>
#include "OffboardControl.h"

struct Points 
{
    Vector3d point;
    int cluster_id;
};

// 声明全局变量
extern std::vector<Points> Target_Samples;

double distance(Vector3d a, Vector3d b);
std::vector<Vector3d> calculate_center(std::vector<Points> samples);
std::vector<Vector3d> Initialize_Clustering(std::vector<Points> samples);
std::vector<Vector3d> Clustering(std::vector<Points> samples);