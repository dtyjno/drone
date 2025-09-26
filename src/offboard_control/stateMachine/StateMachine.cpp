#include "StateMachine.h"
#include "State.h"
// 如果需要在源文件中定义模板特化或静态成员，可以在这里添加
// 但由于这是模板类，大部分实现都在头文件中

// 示例：如果需要为特定类型特化StateMachine
// template<>
// StateMachine<YourSpecificType>& StateMachine<YourSpecificType>::getInstance() {
//     static StateMachine<YourSpecificType> instance;
//     return instance;
// }

template<typename T>
void StateMachine<T>::registerState(const std::string& name, State<T>& state) {
    all_states_map_[name] = &state;
    // 只在owner存在时设置状态的owner
    if (owner_) {
        state.setOwner(owner_);
    }
    RCLCPP_DEBUG(rclcpp::get_logger("StateMachine"), 
                "Registered state: %s", name.c_str());
    // 注释掉自动设置当前状态的逻辑，让调用者明确控制
    // if (current_state_name_.empty()) {
    //     setCurrentState(state);
    // }
}

// Forward declaration for APMROS2Drone
class APMROS2Drone;

// Explicit template instantiation for APMROS2Drone
template class StateMachine<APMROS2Drone>;