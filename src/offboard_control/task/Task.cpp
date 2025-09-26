#include "Task.h"
// #include "../drone/AbstractDrone.h"
// #include "../ROS2drone/ROS2Drone.h"
// #include "../APMROS2drone/APMROS2Drone.h"

// Include the specific task classes for template instantiation
#include "BlankTask.h"
#include "WaitTask.h"
#include "../drone/task/PrintInfoTask.h"
#include "../drone/task/WayPointTask.h"
#include "../drone/task/SetPointTask.h"
#include "../drone/task/RTLLandTask.h"
#include "../APMROS2drone/task/AppochTargetTask.h"
#include "../APMROS2drone/task/DoShotTask.h"
#include "../APMROS2drone/task/DoLandTask.h"

void reset_all_tasks() {
    for (auto& pair : BlankTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : WaitTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : PrintInfoTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : SetPointTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : WayPointTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : RTLLandTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : AppochTargetTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : DoShotTask::TASKS) {
        pair.second->reset();
    }
    for (auto& pair : DoLandTask::TASKS) {
        pair.second->reset();
    }
}

// Explicit template instantiation for common task types
// This ensures the template methods are compiled with the concrete classes
template class Task<BlankTask>;
template class Task<WaitTask>;
template class Task<PrintInfoTask>;
template class Task<WayPointTask>;
template class Task<SetPointTask>;
template class Task<RTLLandTask>;
template class Task<AppochTargetTask>;
template class Task<DoShotTask>;
template class Task<DoLandTask>;