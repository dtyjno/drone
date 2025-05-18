#include "OffboardControl_Base.h"
#include "Vector4.h"

Vector4f OffboardControl_Base::start;

// void OffboardControl_Base::set_pose(){}
// void OffboardControl_Base::set_gps(){}
// void OffboardControl_Base::set_velocity(){}
// void OffboardControl_Base::set_altitude(){}
// void OffboardControl_Base::set_state(){}
// void OffboardControl_Base::set_home_position(){}

// void OffboardControl_Base::set_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
// 	// local_frame.x = msg->pose.position.x;
// 	// local_frame.y = msg->pose.position.y;
// 	// local_frame.z = msg->pose.position.z;
// 	// quaternion.w() = msg->pose.orientation.w;
// 	// quaternion.x() = msg->pose.orientation.x;
// 	// quaternion.y() = msg->pose.orientation.y;
// 	// quaternion.z() = msg->pose.orientation.z;
//     (void)msg;
// }
// void OffboardControl_Base::set_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg){
// 	// global_frame.lat = msg->latitude;
// 	// global_frame.lon = msg->longitude;
// 	// global_frame.alt = msg->altitude;
//     (void)msg;
// }
// void OffboardControl_Base::set_velocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg){
// 	// velocity.x = msg->twist.linear.x;
// 	// velocity.y = msg->twist.linear.y;
// 	// velocity.z = msg->twist.linear.z;
//     (void)msg;
// }
// void OffboardControl_Base::set_altitude(const mavros_msgs::msg::Altitude::SharedPtr msg){
// 	// altitude = msg->monotonic;
// 	(void)msg;
// }
// void OffboardControl_Base::set_state(const mavros_msgs::msg::State::SharedPtr msg){
//     // drone_state_ = *msg;
//     (void)msg;
// }
// void OffboardControl_Base::set_home_position(const mavros_msgs::msg::HomePosition::SharedPtr msg){
//     // home_position_ = *msg;
//     (void)msg;
// }