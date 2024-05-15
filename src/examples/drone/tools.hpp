#ifndef TOOLS_H
#define TOOLS_H
double quaternion_to_yaw(double x, double y, double z, double w);
void yaw_to_quaternion(double yaw, double* x, double* y, double* z, double* w);
#endif