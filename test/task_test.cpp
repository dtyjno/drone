// Copyright [year] <Copyright Owner>
#include <gtest/gtest.h>
#include "../src/offboard_control/task/Task.h"
#include "../src/offboard_control/task/BlankTask.h"
#include "../src/offboard_control/task/SetPointTask.h"
#include "../src/offboard_control/task/WayPointTask.h"
#include "../src/offboard_control/task/DoShotTask.h"
#include "../src/offboard_control/task/DoLandTask.h"
#include "../src/offboard_control/task/RTLLandTask.h"
#include "../src/offboard_control/drone/AbstractDrone.h"

TEST(TaskTest, CreateAndName) {
  auto task = BlankTask::createTask("TestBlank");
  EXPECT_EQ(task->get_name(), "TestBlank");
  EXPECT_EQ(task->task_state_, Task<BlankTask>::TaskState::init);
}

TEST(TaskTest, StateSwitch) {
  auto device = std::make_shared<AbstractDrone>();
  auto task1 = BlankTask::createTask("TestBlank");
  auto task2 = BlankTask::createTask("TestBlank");
  task1->next_task(task2);
  EXPECT_TRUE(!task1->is_execute_finished());
  EXPECT_TRUE(!task1->get_task_result());
  device->accept(task1);
  device->accept(task1);
  device->accept(task1);
  EXPECT_TRUE(task1->is_execute_finished());
  EXPECT_TRUE(task1->get_task_result());
  // 任务执行后应进入end或init状态
  EXPECT_TRUE(
    task1->task_state_ == Task<BlankTask>::TaskState::end ||
    task1->task_state_ == Task<BlankTask>::TaskState::init
  );
  EXPECT_TRUE(task2->is_execute_finished());
  EXPECT_TRUE(task2->get_task_result());
  EXPECT_TRUE(
    task2->task_state_ == Task<BlankTask>::TaskState::end ||
    task2->task_state_ == Task<BlankTask>::TaskState::init
  );
}

// 测试SetPointTask通过LOCAL和START发布航点功能，测试任务执行和状态转换，测试send_start_setpoint_command和send_local_setpoint_command位置转换是否准确, 验证计时器
TEST(TaskTest, SetPointTask) {
  auto device = std::make_shared<AbstractDrone>();
  device->read_default_yaw_configs("test.yaml");  // 默认旋转180度
  ASSERT_TRUE(device->get_position_controller() != nullptr);
  ASSERT_TRUE(device->get_position_controller()->pos_data != nullptr);
  std::cout << "Initial Position: (" 
            << device->get_position_controller()->pos_data->get_position().x() << ", "
            << device->get_position_controller()->pos_data->get_position().y() << ", "
            << device->get_position_controller()->pos_data->get_position().z() << ")\n";
  auto task1 = SetPointTask::createTask("TestSetPoint");
  SetPointTask::Parameters params;
  params.target_x = 1.0f;
  params.target_y = 1.0f;
  params.target_z = 1.0f;
  params.target_yaw = 0.0f;
  // params.point_time = 2.0f;
  params.accuracy = 0.1f;
  params.frame = SetPointTask::Parameters::Frame::START;  // 相对起始位置
  task1->set_parameters(params);
  std::cout << "Executing SetPointTask to ("
            << params.target_x << ", "
            << params.target_y << ", "
            << params.target_z << ") over " << params.point_time << " seconds.\n";
  EXPECT_TRUE(!task1->is_execute_finished());
  EXPECT_TRUE(!task1->get_task_result());
  device->accept(task1);
  device->accept(task1); // 检查是否满足accuracy条件

  std::cout << "Final Position: (" 
            << device->get_x_pos() << ", "
            << device->get_y_pos() << ", "
            << device->get_z_pos() << ")\n";
  float temp_x = params.target_x;
  float temp_y = params.target_y;
  device->rotate_global2stand(temp_x, temp_y, temp_x, temp_y);
  EXPECT_TRUE(is_equal(device->get_x_pos(), temp_x, params.accuracy));
  EXPECT_TRUE(is_equal(device->get_y_pos(), temp_y, params.accuracy));
  EXPECT_TRUE(is_equal(device->get_z_pos(), 1.0f, params.accuracy));
  EXPECT_TRUE(task1->is_execute_finished());
  EXPECT_TRUE(task1->get_task_result());
  // 任务执行后应进入end或init状态
  EXPECT_TRUE(
    task1->task_state_ == Task<SetPointTask>::TaskState::end ||
    task1->task_state_ == Task<SetPointTask>::TaskState::init
  );
  std::cout << "Resetting and re-executing SetPointTask"; 
  task1->reset();
  EXPECT_TRUE(!task1->get_task_result());
  params.target_x = 0.0f;
  params.target_y = 0.0f;
  params.target_z = 1.0f;
  params.point_time = 0.05f;
  params.accuracy = 0.0f;
  params.frame = SetPointTask::Parameters::Frame::LOCAL;
  task1->set_parameters(params);
  std::cout << "Executing SetPointTask to ("
            << params.target_x << ", "
            << params.target_y << ", "
            << params.target_z << ") over " << params.point_time << " seconds.\n";
  device->accept(task1);
  EXPECT_TRUE(!task1->is_execute_finished());
  EXPECT_TRUE(!task1->get_task_result());
  task1->get_timer().set_start_time_offset(1.0); // 等待1秒让位置更新
  std::cout << task1->get_timer().elapsed() << " seconds elapsed.\n";
  std::cout << "Position: (" 
            << device->get_x_pos() << ", "
            << device->get_y_pos() << ", "
            << device->get_z_pos() << ")\n";
  device->accept(task1);
  float temp_x_2 = params.target_x;
  float temp_y_2 = params.target_y;
  device->rotate_global2stand(temp_x_2, temp_y_2, temp_x_2, temp_y_2);
  EXPECT_TRUE(is_equal(device->get_x_pos(), temp_x + temp_x_2, 0.1f));
  EXPECT_TRUE(is_equal(device->get_y_pos(), temp_y + temp_y_2, 0.1f));
  EXPECT_TRUE(is_equal(device->get_z_pos(), 2.0f, 0.1f));
  EXPECT_TRUE(task1->is_execute_finished());
  EXPECT_TRUE(task1->get_task_result());
  // 任务执行后应进入end或init状态
  EXPECT_TRUE(
    task1->task_state_ == Task<SetPointTask>::TaskState::end ||
    task1->task_state_ == Task<SetPointTask>::TaskState::init
  );
}

// 测试DoShotTask任务执行和状态转换，测试动态目标坐标回调函数功能,测试AppochTargetTask链接功能
TEST(TaskTest, DoShotTask) {
  auto device = std::make_shared<AbstractDrone>();
  device->read_default_yaw_configs("test.yaml");  // 默认旋转180度
  ASSERT_TRUE(device->get_position_controller() != nullptr);
  ASSERT_TRUE(device->get_position_controller()->pos_data != nullptr);
  std::cout << "Initial Position: (" 
            << device->get_position_controller()->pos_data->get_position().x() << ", "
            << device->get_position_controller()->pos_data->get_position().y() << ", "
            << device->get_position_controller()->pos_data->get_position().z() << ")\n";

  float dx_shot = 5.0f;          // 投弹点X坐标
  float dy_shot = 0.0f;          // 投弹点Y坐标
  // float shot_halt = 3.5f;        // 投弹点悬停高度
  // float shot_halt_low = 2.5f;    // 投弹点悬停最低高度
  float shot_halt_surround = 4.0f; // 投弹点周围悬停高度
  // float shot_width_max = 4.0f;   // 投弹区最大宽度
  float bucket_height = 0.5f;    // 桶的高度
  int shot_counter = 0;

	auto do_shot_waypoint_task = WayPointTask::createTask("Do_shot_waypoint");
	WayPoints do_shot_waypoint = WayPoints(
		"投弹区",
		std::vector<Vector4f>{
			Vector4f(1.0f, 1.0f, shot_halt_surround, 0.0f),
			Vector4f(0.0f, 0.0f, shot_halt_surround, 0.0f),
			Vector4f(-1.0f, -1.0f, shot_halt_surround, 0.0f)
		}
	);
	do_shot_waypoint_task->set_config(
		do_shot_waypoint,
		WayPointTask::Parameters{
			dx_shot,    // center_x
			dy_shot,    // center_y
			1.0f,    // point_time (每个航点停留时间)
			0.3f     // accuracy (航点到达精度)
		}
	);

	auto do_shot_task = DoShotTask::createTask("Do_shot");
	DoShotTask::Parameters doshot_params;
  std::function<AppochTargetTask::PositionTarget()> empty_position_callback = []() -> AppochTargetTask::PositionTarget {
      AppochTargetTask::PositionTarget pos_target;
			pos_target.position = Vector3f::Zero();
			pos_target.radius = 0.0f;
			pos_target.index = -1;
			return pos_target;
  };
  std::function<Vector2f()> empty_image_callback = []() -> Vector2f {
      return Vector2f::Zero();
  };
  std::function<AppochTargetTask::PositionTarget()> target_position_callback = []() -> AppochTargetTask::PositionTarget {
      AppochTargetTask::PositionTarget pos_target;
			pos_target.position = Vector3f{1.0f, 1.0f, 1.0f};
			pos_target.radius = 0.5f;
			pos_target.index = 0;
      return pos_target;
	};
  // (-1.0, -1.0)
  std::function<Vector2f()> target_image_callback = [device, bucket_height]() -> Vector2f {
    // std::cout << "pox_x = " << device->get_x_pos() << " pox_y = " << device->get_y_pos() << " pox_z = " << device->get_z_pos() << std::endl;
    // std::cout << "camera position = " << device->get_camera()->get_position().transpose() << std::endl;
    Vector3d target_world_position = Vector3d{-0.5, -0.5, bucket_height};
    auto output_pixel_opt = device->get_camera()->worldToPixel(target_world_position);
    if (output_pixel_opt.has_value()) {
        Vector2d output_pixel = output_pixel_opt.value();
        std::cout << "worldToPixel: output_pixel: " << output_pixel.transpose() << std::endl;
        return Vector2f(output_pixel.x(), output_pixel.y());
    } else {
        std::cerr << "worldToPixel error" << std::endl;
        return Vector2f::Zero();
    }
  };

	doshot_params.dynamic_target_position_callback = empty_position_callback;
	doshot_params.dynamic_target_image_callback = empty_image_callback;
	doshot_params.device_index = 0;
	doshot_params.target_height = bucket_height;
	doshot_params.shot_duration = 0.5f;
  doshot_params.shot_wait = 0.2f;
  doshot_params.task_type = AppochTargetTask::Type::AUTO;  // 任务类型，AUTO自动选择PID或TARGET

	do_shot_task->setParameters(doshot_params);


  doshot_params.device_index = shot_counter;
  // task_manager.addTask(WaitTask::createTask("Wait_70s")->set_config(70.0, false));
  do_shot_task->set_task_when_no_target(do_shot_waypoint_task);
  auto appoch_ptr = do_shot_task->get_appochtarget_task();
  ASSERT_TRUE(appoch_ptr != nullptr);
  EXPECT_TRUE(is_equal(device->get_x_pos(), 0.0f, 0.5f));
  EXPECT_TRUE(is_equal(device->get_y_pos(), 0.0f, 0.5f));
  EXPECT_TRUE(is_equal(device->get_z_pos(), 2.0f, 0.5f));
  std::cout << "空目标测试(WayPointTask)" << std::endl;
  for (size_t j = 0; j < do_shot_waypoint.size(); ++j){
    device->accept(do_shot_task);
    // 航点j
    float target_x = dx_shot + do_shot_waypoint[j].x();
    float target_y = dy_shot + do_shot_waypoint[j].y();
    device->rotate_global2stand(target_x, target_y, target_x, target_y);
    std::cout << "Moving to WayPoint " << j << ": Target Position: (" 
              << target_x << ", "
              << target_y << ", "
              << shot_halt_surround << ")\n";
    EXPECT_TRUE(is_equal(device->get_x_pos(), target_x, 0.5f));
    EXPECT_TRUE(is_equal(device->get_y_pos(), target_y, 0.5f));
    EXPECT_TRUE(is_equal(device->get_z_pos(), shot_halt_surround, 0.5f));
    for (size_t i = 0; i < 2; ++i) {
      do_shot_waypoint_task->get_timer().set_start_time_offset(0.5); // 等待
        device->accept(do_shot_task);
        std::cout << "Step " << i << ": Position: (" 
                  << device->get_x_pos() << ", "
                  << device->get_y_pos() << ", "
                  << device->get_z_pos() << ")\n";
    }
  }
  // 循环回到第一个航点
  device->accept(do_shot_task);    //end->run
  float target_x = dx_shot + do_shot_waypoint[0].x();
  float target_y = dy_shot + do_shot_waypoint[0].y();
  device->rotate_global2stand(target_x, target_y, target_x, target_y);
  EXPECT_TRUE(is_equal(device->get_x_pos(), target_x, 0.5f));
  EXPECT_TRUE(is_equal(device->get_y_pos(), target_y, 0.5f));
  EXPECT_TRUE(is_equal(device->get_z_pos(), shot_halt_surround, 0.5f));

  std::cout << "TARGET" << std::endl;
  std::cout << "同时有位置和图像目标时优先位置" << std::endl;
  // device->send_world_setpoint_command(0.0f, 0.0f, 4.0f, 0.0f); // 初始位置设为(0,0,4)  
  device->get_position_controller()->get_pos_data()->set_position(Vector3f{0.0, 0.0, 4.0}); // 初始位置设为(0,0,4)
  // 设置相机位置和方向 0, 0, 4 roll=0 pitch=0 yaw=0 垂直向下
	device->get_camera()->set_parent_position(Vector3d(device->get_x_pos(), device->get_y_pos(), device->get_z_pos()));
	device->get_camera()->set_camera_relative_rotation(Vector3d(0, 0, 0)); // 相机相对飞机的旋转，roll=0, pitch=0 (垂直向下), yaw=0
	device->get_camera()->set_parent_rotation(Vector3d(0, 0, device->get_world_yaw()));

  do_shot_task->setDynamicPositionTargetCallback(target_position_callback);
  do_shot_task->setDynamicImageTargetCallback(target_image_callback);
  device->accept(do_shot_task);
  std::cout << "dynamic_target_position_callback" << appoch_ptr->getCurrentPositionTargets().position.transpose() << std::endl;
  std::cout << "dynamic_target_image_callback" << appoch_ptr->getCurrentImageTargets().transpose() << std::endl;
  EXPECT_TRUE(appoch_ptr->getCurrentType() == AppochTargetTask::Type::TARGET);
  std::cout << "只有位置目标" << std::endl;
  do_shot_task->setDynamicPositionTargetCallback(target_position_callback);
  do_shot_task->setDynamicImageTargetCallback(empty_image_callback);
  do_shot_task->get_appochtarget_task()->setTaskType(AppochTargetTask::Type::TARGET);   // 只测试TARGET逻辑
  for (int i = 0; i * device->get_wait_time() < 0.8; ++i) {
      std::cout << "Step " << i << ": Position: (" 
                << device->get_x_pos() << ", "
                << device->get_y_pos() << ", "
                << device->get_z_pos() << ")\n";
      device->accept(do_shot_task);
  }
  appoch_ptr = do_shot_task->get_appochtarget_task();
  ASSERT_TRUE(appoch_ptr != nullptr);
  EXPECT_TRUE(appoch_ptr->getCurrentType() == AppochTargetTask::Type::TARGET);
  EXPECT_TRUE(is_equal(device->get_x_pos(), 1.0f, 0.5f));
  EXPECT_TRUE(is_equal(device->get_y_pos(), 1.0f, 0.5f));
  EXPECT_TRUE(is_equal(device->get_z_pos(), 1.0f, 0.5f));
  EXPECT_TRUE(do_shot_task->is_execute_finished());
  EXPECT_TRUE(do_shot_task->get_task_result());
  std::cout << "PID" << std::endl; 
  device->get_position_controller()->get_pos_data()->set_position(Vector3f{0.0, 0.0, 4.0}); // 初始位置设为(0,0,4)
  // 设置相机位置和方向 0, 0, 4 roll=0 pitch=0 yaw=0 垂直向下
	device->get_camera()->set_parent_position(Vector3d(device->get_x_pos(), device->get_y_pos(), device->get_z_pos()));
	device->get_camera()->set_camera_relative_rotation(Vector3d(0, 0, 0)); // 相机相对飞机的旋转，roll=0, pitch=0 (垂直向下), yaw=0
	device->get_camera()->set_parent_rotation(Vector3d(0, 0, device->get_world_yaw()));
  // do_shot_task->set_parameters(doshot_params);
  // 模拟第二次投弹
  do_shot_task->reset();
  do_shot_task->setDynamicPositionTargetCallback(empty_position_callback);
  do_shot_task->setDynamicImageTargetCallback(target_image_callback);
  do_shot_task->get_appochtarget_task()->setTaskType(AppochTargetTask::Type::PID);   // 只测试PID逻辑
  for (int i = 0; i * device->get_wait_time() < 2.5; ++i) {
	    device->get_camera()->set_parent_position(Vector3d(device->get_x_pos(), device->get_y_pos(), device->get_z_pos()));
      std::cout << "Step " << i << ": Position: (" 
                << device->get_x_pos() << ", "
                << device->get_y_pos() << ", "
                << device->get_z_pos() << ")\n";
      device->accept(do_shot_task);
  }
  EXPECT_TRUE(is_equal(device->get_x_pos(), -0.5f, 0.2f));
  EXPECT_TRUE(is_equal(device->get_y_pos(), -0.5f, 0.2f));
  EXPECT_TRUE(is_equal(device->get_z_pos(), 1.0f, 0.2f));
  appoch_ptr = do_shot_task->get_appochtarget_task();
  ASSERT_TRUE(appoch_ptr != nullptr);
  EXPECT_TRUE(appoch_ptr->getCurrentType() == AppochTargetTask::Type::PID);
  EXPECT_TRUE(do_shot_task->is_execute_finished());
  EXPECT_TRUE(do_shot_task->get_task_result());
}

// 未实现完整的EXPECT，主要是验证任务执行流程和PID控制效果
TEST(TaskTest, DoLandTask) {
  auto device = std::make_shared<AbstractDrone>();
  device->read_default_yaw_configs("test.yaml");  // 默认旋转180度
  ASSERT_TRUE(device->get_position_controller() != nullptr);
  ASSERT_TRUE(device->get_position_controller()->pos_data != nullptr);
  std::shared_ptr<RTLLandTask> rtl_land_task = RTLLandTask::createTask("RTLLand");
  std::shared_ptr<DoLandTask> land_task = DoLandTask::createTask("DoLand");
  rtl_land_task->setLandTask(land_task);

  std::cout << "Initial Position: (" 
            << device->get_position_controller()->pos_data->get_position().x() << ", "
            << device->get_position_controller()->pos_data->get_position().y() << ", "
            << device->get_position_controller()->pos_data->get_position().z() << ")\n";
  device->accept(rtl_land_task);
  rtl_land_task->get_timer().set_start_time_offset(18.0); // 等待
  device->accept(rtl_land_task);
  // 前5个航点
  for (int i = 0; i < 5; ++i) {
      device->accept(rtl_land_task);
      rtl_land_task->get_timer().set_start_time_offset(2.0); // 等待1秒让位置更新
      land_task->get_timer().set_start_time_offset(2.0); // 等待1秒让位置更新
      std::cout << "Step " << i << ": Position: (" 
                << device->get_x_pos() << ", "
                << device->get_y_pos() << ", "
                << device->get_z_pos() << ")\n";
  }
  
  auto land_task_ptr = std::dynamic_pointer_cast<DoLandTask>(rtl_land_task->getLandTask());
  if (land_task_ptr == nullptr) {
      std::cerr << "Failed to cast to DoLandTask" << std::endl;
      EXPECT_TRUE(false);
      return;
  }
  land_task_ptr->setDynamicImageTargetCallback([device]() -> Vector2f {
    // std::cout << "pox_x = " << device->get_x_pos() << " pox_y = " << device->get_y_pos() << " pox_z = " << device->get_z_pos() << std::endl;
    // std::cout << "camera position = " << device->get_camera()->get_position().transpose() << std::endl;
    Vector3d target_world_position = Vector3d{-0.5, -0.5, 0.0};
    auto output_pixel_opt = device->get_camera()->worldToPixel(target_world_position);
    if (output_pixel_opt.has_value()) {
        Vector2d output_pixel = output_pixel_opt.value();
        std::cout << "worldToPixel: output_pixel: " << output_pixel.transpose() << std::endl;
        return Vector2f(output_pixel.x(), output_pixel.y());
    } else {
        std::cerr << "worldToPixel error" << std::endl;
        return Vector2f::Zero();
    }
  });

  // 设置相机位置和方向 0, 0, 0 roll=0 pitch=0 yaw=0 垂直向下
  // device->get_position_controller()->get_pos_data()->set_position(Vector3f{0.0, 0.0, 4.0}); // 初始位置设为(0,0,4)
	device->get_camera()->set_parent_position(Vector3d(device->get_x_pos(), device->get_y_pos(), device->get_z_pos()));
	device->get_camera()->set_camera_relative_rotation(Vector3d(0, 0, 0)); // 相机相对飞机的旋转，roll=0, pitch=0 (垂直向下), yaw=0
	device->get_camera()->set_parent_rotation(Vector3d(0, 0, device->get_world_yaw()));

  // PID
  for (int i = 0; i < 20; ++i) {
      device->get_camera()->set_parent_position(Vector3d(device->get_x_pos(), device->get_y_pos(), device->get_z_pos()));
      device->accept(rtl_land_task);
      std::cout << "Step " << i << ": Position: (" 
                << device->get_x_pos() << ", "
                << device->get_y_pos() << ", "
                << device->get_z_pos() << ")\n";
  }
  float land_x = -0.5f;    // world point
  float land_y = -0.5f;
  EXPECT_TRUE(is_equal(device->get_x_pos(), land_x, 0.1f));
  EXPECT_TRUE(is_equal(device->get_y_pos(), land_y, 0.1f));

  // 后5个航点 (不执行PID)
  for (int i = 0; i < 5; ++i) {
      device->accept(rtl_land_task);
      rtl_land_task->get_timer().set_start_time_offset(2.0); // 等待1秒让位置更新
      land_task->get_timer().set_start_time_offset(2.0); // 等待1秒让位置更新
      std::cout << "Step " << i << ": Position: (" 
                << device->get_x_pos() << ", "
                << device->get_y_pos() << ", "
                << device->get_z_pos() << ")\n";
  }
  EXPECT_TRUE(rtl_land_task->is_execute_finished());
  // EXPECT_TRUE(false);

}

