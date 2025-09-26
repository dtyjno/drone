#pragma once

#include <vector>
#include "TaskBase.h"

template<typename T>
class TaskManager {
public:
    TaskManager(std::shared_ptr<T> drone) : drone(drone) {
        tasks.clear();
    }

    TaskManager* addTask(std::shared_ptr<TaskBase> task){
        tasks.push_back(task);
        return this;
    }
    TaskManager* execute(){
        for(auto task : tasks) {
            !task->final_task()->visit(drone);
            if (task->is_execute_finished()) {
                // std::cout << task->get_string() << " finished." << std::endl;
                continue;
            } else {
                break;
            }
        }
        return this;
    }
    TaskManager* clean(){
        tasks.clear();
        return this;
    }

private:
    std::vector<std::shared_ptr<TaskBase>> tasks;
    std::shared_ptr<T> drone;
};