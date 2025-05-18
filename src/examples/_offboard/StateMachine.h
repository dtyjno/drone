#pragma once
#include <memory>
#include "FlyState.h"

class OffboardControl;

class StateMachine {
public:
    explicit StateMachine(OffboardControl& controller) 
        : controller_(controller), current_state_(nullptr) {}

    void initialize(std::unique_ptr<FlyState> initialState) {
        current_state_ = std::move(initialState);
        current_state_->enter(controller_);
    }

    void transitionTo(std::unique_ptr<FlyState> newState) {
        if (current_state_) {
            current_state_->exit(controller_);
        }
        current_state_ = std::move(newState);
        current_state_->enter(controller_);
    }

    void executeCurrentState() {
        if (current_state_) {
            current_state_->execute(controller_);
        }
    }

private:
    OffboardControl& controller_;
    std::unique_ptr<FlyState> current_state_;
};