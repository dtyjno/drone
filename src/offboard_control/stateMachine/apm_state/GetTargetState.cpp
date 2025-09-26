#include "GetTargetState.h"
#include "../APMROS2Drone.h"
#include "../../stateMachine/StateMachine.h"

// 计算目标位置
void GetTargetState::executeImpl()  {
    if (!owner_) {
        RCLCPP_ERROR(rclcpp::get_logger("GetTargetState"), "Owner is null");
        return;
    }
    
    RCLCPP_INFO_ONCE(rclcpp::get_logger("GetTargetState"), "执行GetTargetState状态");

	std::vector<vision_msgs::msg::BoundingBox2D> raw_circles = owner_->get_yolo_detector()->get_raw_targets(YOLO_TARGET_TYPE::CIRCLE);
	for (const auto& circle : raw_circles) 
	{
		// std::cout << "检测到的像素坐标: (" << circle.center.position.x << ", " << circle.center.position.y << ")" << std::endl;
		
		auto target1 = owner_->get_camera_gimbal()->pixelToWorldPosition(
			Vector2d(circle.center.position.x, circle.center.position.y), 
			owner_->bucket_height // 桶顶高度
		);
		double avg_size = (circle.size_x + circle.size_y) / 2.0;
		double distance_to_bucket = owner_->get_camera_gimbal()->get_position().z() - owner_->bucket_height;
		double diameter = 0.0;
		if (distance_to_bucket > 0.01) { // 防止除零
			diameter = owner_->get_camera_gimbal()->calculateRealDiameter(avg_size, distance_to_bucket);
		}
		if (target1.has_value()) {
			RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 1000, 
                "(T 1s) Example 1 - Target position: %f, %f, %f. Diameter: %f",
				target1->x(), target1->y(), target1->z(), diameter);
            Circles circle_info;
            circle_info.point = *target1;
            circle_info.cluster_id = 0; // 初始聚类ID为0
            circle_info.original_index = 0; // 这里没有原始索引，可以设置为0
            circle_info.diameters = diameter;
			Target_Samples.push_back(circle_info);
		}
		else {
			RCLCPP_WARN(owner_->get_node()->get_logger(), "Example 1 - 无效的目标位置");
		}
	}
    // Fix the state comparisons - use StateMachine getCurrentState method
    auto& stateMachine = StateMachine<APMROS2Drone>::getInstance();
    if(!Target_Samples.empty() && (
        stateMachine.getCurrentState().getName() == "InitState" || 
        stateMachine.getCurrentState().getName() == "DoshotState" || 
        stateMachine.getCurrentState().getName() == "GotoShotPointState"))
    {
		owner_->cal_center = Clustering(Target_Samples);
		if (!owner_->cal_center.empty()) {
			std::ostringstream ss;
			ss << "(T 2s) \n";
			for (size_t i = 0; i < owner_->cal_center.size(); ++i) {
				ss << "侦查点坐标 " << i << ": x: " << owner_->cal_center[i].point.x() 
					<< ", y: " << owner_->cal_center[i].point.y() 
					<< ", d: " << owner_->cal_center[i].diameters << "\n";
			}
			RCLCPP_INFO_THROTTLE(owner_->get_node()->get_logger(), *owner_->get_node()->get_clock(), 2000, 
                "%s", ss.str().c_str());
		}
		uint8_t shot_count = 0;
		for(size_t i = 0; i < owner_->cal_center.size(); ++i)
		{
			double tx, ty;
			owner_->rotate_stand2global(owner_->cal_center[i].point.x() - owner_->get_x_home_pos(), 
                                       owner_->cal_center[i].point.y() - owner_->get_y_home_pos(), tx, ty);
			if (tx < owner_->dx_shot - owner_->shot_length_max / 2 - 1.5 || tx > owner_->dx_shot + owner_->shot_length_max / 2 + 1.5 ||
				ty < owner_->dy_shot - 1.5 || ty > owner_->dy_shot + owner_->shot_width_max + 1.5) {
				RCLCPP_WARN(owner_->get_node()->get_logger(), "侦查点坐标异常，跳过: %zu, x: %f, y: %f, tx: %f, ty: %f", 
                           i, owner_->cal_center[i].point.x(), owner_->cal_center[i].point.y(), tx, ty);
				owner_->cal_center.erase(owner_->cal_center.begin() + i);
				i--; // 调整索引以适应删除后的数组
				continue;
			}
			// 从大到小排列
			sort(owner_->cal_center.begin(), owner_->cal_center.end(), [this](const Circles& a, const Circles& b) {
				return owner_->shot_big_target ? a.diameters > b.diameters : a.diameters < b.diameters;
			});

			owner_->surround_shot_points[shot_count] = Vector2f((tx - owner_->dx_shot) / owner_->shot_length_max, 
                                                               (ty - owner_->dy_shot) / owner_->shot_width_max);
			shot_count++;
		}
	}
}