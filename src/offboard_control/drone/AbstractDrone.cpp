#include "AbstractDrone.h"
#include "../task/TaskBase.h"
#include <cstdarg>
#include <cstdio>

// 定义静态成员变量
Vector4f AbstractDrone::start{0.0f, 0.0f, 0.0f, 0.0f};

// 实现 accept 方法

void AbstractDrone::accept(std::shared_ptr<TaskBase> visitor) {
	visitor->visit(std::shared_ptr<AbstractDrone>(this, [](AbstractDrone*){}));
}

bool AbstractDrone::is_equal_start_target_xy(float x, float y, double accuracy) {
    rotate_global2stand(x, y, x, y);
    return (std::abs(get_x_pos() - x) < accuracy) &&
           (std::abs(get_y_pos() - y) < accuracy);
}

bool AbstractDrone::is_equal_local_target_xy(float x, float y, double accuracy) {
    rotate_world2local(x, y, x, y);
    return (std::abs(get_x_pos() - x) < accuracy) &&
           (std::abs(get_y_pos() - y) < accuracy);
}

void AbstractDrone::send_start_setpoint_command(float x, float y, float z, float yaw){
    // std::cout << "send_start_setpoint_command: x=" << x << ", y=" << y << ", z=" << z << ", yaw=" << yaw << std::endl;
    rotate_global2stand(x, y, x, y);
    // std::cout << "After rotate_global2stand: x=" << x << ", y=" << y << ", z=" << z << ", yaw=" << yaw << std::endl;
    get_position_controller()->pos_publisher->send_local_setpoint_command(x + get_x_home_pos(), y + get_y_home_pos(), z, default_yaw - yaw);
}

void AbstractDrone::send_local_setpoint_command(float x, float y, float z, float yaw){
    rotate_world2local(x, y, x, y);
    std::cout << "send_local_setpoint_command: x=" << x << ", y=" << y << ", z=" << z << ", yaw=" << yaw << std::endl;
    std::cout << "send_local_setpoint_command: get_x_pos=" << get_x_pos() << ", y=" << get_y_pos() << ", z=" << get_z_pos() << ", yaw=" << get_yaw() << std::endl;
    get_position_controller()->pos_publisher->send_local_setpoint_command(x + get_x_pos(), y + get_y_pos(), z + get_z_pos(), default_yaw - yaw + get_yaw());
}

void AbstractDrone::send_world_setpoint_command(float x, float y, float z, float yaw){
    get_position_controller()->pos_publisher->send_local_setpoint_command(x, y, z, default_yaw - yaw);
}

bool AbstractDrone::local_setpoint_command(float x, float y, float z, float yaw, double accuracy){
    return get_position_controller()->pos_publisher->local_setpoint_command(
        Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
        Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
        accuracy);
}

bool AbstractDrone::trajectory_setpoint(float x, float y, float z, float yaw, double accuracy)
{
    // RCLCPP_INFO_ONCE(node->get_logger(), "trajectory_setpoint转换后目标位置：%f %f", x, y);
    return get_position_controller()->trajectory_setpoint(
            Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
            Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
            accuracy);
}
// bool AbstractDrone::trajectory_setpoint_world(float x, float y, float z, float yaw, PID::Defaults defaults, double accuracy)
// {
//     return get_position_controller()->trajectory_setpoint_world(
//             Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
//             Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
//             defaults,
//             accuracy);
// }
bool AbstractDrone::trajectory_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
    return get_position_controller()->trajectory_setpoint_world(
            Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
            Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
            accuracy);
}

bool AbstractDrone::publish_setpoint_world(float x, float y, float z, float yaw, double accuracy)
{
    return get_position_controller()->publish_setpoint_world(
            Vector4f{get_x_pos(), get_y_pos(), get_z_pos(), get_yaw()},
            Vector4f{x, y, z, static_cast<float>(yaw + default_yaw)},
            accuracy);
}

void AbstractDrone::send_velocity_command(float x, float y, float z, float yaw)
{
    return get_position_controller()->pos_publisher->send_velocity_command(
            Vector4f{x, y, z, yaw});
}

bool AbstractDrone::send_velocity_command_with_time(float x, float y, float z, float yaw, double time)
{
    return get_position_controller()->pos_publisher->send_velocity_command_with_time(
            Vector4f{x, y, z, yaw},
            time);
}
bool AbstractDrone::trajectory_circle(float a, float b, float height, float dt, float yaw)
{
    return get_position_controller()->trajectory_circle(
            a,
            b,
            (height - get_z_pos()),
            dt,
            yaw + default_yaw,
            default_yaw);
}
bool AbstractDrone::trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal)
{
    return get_position_controller()->trajectory_generator_world(
            speed_factor,
            q_goal);
}
// bool AbstractDrone::trajectory_generator(double speed_factor, std::array<double, 3> q_goal){
//     rotate_xy(q_goal[0], q_goal[1], default_yaw);
//     return get_position_controller()->trajectory_generator_world(
//         speed_factor,
//         {q_goal[0]+get_x_pos(), q_goal[1]+get_y_pos(),q_goal[2]+get_z_pos()}
//     );
// }
bool AbstractDrone::trajectory_generator_world_points(double speed_factor, const std::vector<std::array<double, 3>> &data, int data_length, Vector3f max_speed_xy, Vector3f max_accel_xy, float tar_yaw)
{
    static bool first = true;
    static uint16_t data_length_;
    static uint16_t current_waypoint_index = 0;
    static bool sequence_completed = false;
    
    // 只在第一次调用或者序列完成后需要重新开始时初始化
    if (first || (sequence_completed))
    {
        data_length_ = data_length;
        current_waypoint_index = 0;
        sequence_completed = false;
        first = false;
        std::cout << "Initializing waypoint sequence: data_length=" << data_length_ << std::endl;
    }
    
    std::cout << "data.size(): " << data.size() << std::endl;
    std::cout << "data_length: " << data_length_ << std::endl;
    std::cout << "current_waypoint_index: " << current_waypoint_index << std::endl;
    
    // 检查数据有效性，防止越界访问
    if (data.empty() || current_waypoint_index >= data.size() || current_waypoint_index >= data_length_) {
        std::cout << "数据无效或已完成，返回true" << std::endl;
        sequence_completed = true;
        return true;
    }

    std::array<double, 3> q_goal = data[current_waypoint_index];
    double global_x, global_y;
    rotate_global2stand(q_goal[0], q_goal[1], global_x, global_y);
    std::cout << "waypoint " << current_waypoint_index << " q_goal: " << global_x << " " << global_y << " " << q_goal[2] << std::endl;

    if (get_position_controller()->trajectory_generator_world(
            speed_factor,
            {global_x, global_y, q_goal[2]},
            max_speed_xy,
            max_accel_xy,
            static_cast<float>(tar_yaw + default_yaw)
            )
        )
    {
        current_waypoint_index++;
        std::cout << "Waypoint " << (current_waypoint_index - 1) << " completed! Moving to waypoint " << current_waypoint_index << std::endl;
        
        // 检查是否完成所有waypoint
        if (current_waypoint_index >= data.size() || current_waypoint_index >= data_length_)
        {
            std::cout << "All waypoints completed!" << std::endl;
            sequence_completed = true;
            return true;
        }
    }
    
    return false;
}
