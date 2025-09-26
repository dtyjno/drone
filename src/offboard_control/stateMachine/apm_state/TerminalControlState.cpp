#include "TerminalControlState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"
#include "EndState.h"

void TerminalControlState::executeImpl()  {
    RCLCPP_INFO_ONCE(rclcpp::get_logger("TerminalControlState"), "执行终端控制状态");
    // 终端控制逻辑可以在这里实现  char key = 0;
    static std::string input = "";

    if (_kbhit()) // 检查是否有按键输入
    {
        char key = _getch(); // 获取按键输入
        auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
        // 特殊状态处理（起飞解锁前）
        auto apm_sta_ctl = std::dynamic_pointer_cast<APMROS2StatusController>(owner_->sta_ctl);
        if (apm_sta_ctl && apm_sta_ctl->state_ == APMROS2StatusController::TakeoffState::wait_for_takeoff_command){
            if (key == '\n') // 检查是否按下回车键
            {
                apm_sta_ctl->takeoff_command = true; // 设置起飞命令
            }
            else if (key == 'q') // 检查是否按下q键
            {
                RCLCPP_INFO(owner_->get_node()->get_logger(), "退出程序");
                stateMachine.transitionTo(EndState::getInstance()); // 切换到结束状态
            }
            else if (key != 0)
            {
                RCLCPP_INFO(owner_->get_node()->get_logger(), "无效输入，请按回车键解锁无人机或按q键退出程序");
            }
            return;
        }
        // 处理多字符输入
        if (key == '\n' || key == '\r') { // 按下回车，尝试解析命令
        std::string upperInput = input;
        std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
        const auto& allStates = stateMachine.getAllStates();
        auto it = allStates.find(upperInput);
        if (it != allStates.end()) {
            stateMachine.transitionTo(it->second->getName());
            RCLCPP_INFO(owner_->get_node()->get_logger(), "切换到状态: %s", upperInput.c_str());
        } else {
            RCLCPP_INFO(owner_->get_node()->get_logger(), "无效指令: %s", input.c_str());
        }
        input.clear();
        } else if (key == '\b' && !input.empty()) { // 处理退格
        input.pop_back();
            } else if (key == '\t') { // Tab键自动补全
                std::string upperInput = input;
                std::transform(upperInput.begin(), upperInput.end(), upperInput.begin(), ::toupper);
                std::vector<std::string> candidates;
                auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
                const auto& allStates = stateMachine.getAllStates();
                for (const auto& kv : allStates) {
                        if (kv.first.find(upperInput) == 0) { // 前缀匹配
                                candidates.push_back(kv.first);
                        }
                }
                if (candidates.size() == 1) {
                        input = candidates[0];
                        RCLCPP_INFO(owner_->get_node()->get_logger(), "自动补全: %s", input.c_str());
                } else if (candidates.size() > 1) {
                        std::string msg = "可选项: ";
                        for (const auto& s : candidates) msg += s + " ";
                        RCLCPP_INFO(owner_->get_node()->get_logger(), "%s", msg.c_str());
                }
        } else if (key != 0) {
        input += key; // 将按键添加到输入字符串中
        }
    }
}
