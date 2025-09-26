#pragma once

#include "../algorithm/pid/PID.h"
#include "../algorithm/pid/FuzzyPID.h"
#include "../algorithm/pid/AutoTune.h"
#include "../waypoint/TrajectoryGenerator.h"

#include "../utils/math.h"
#include "../utils/Readyaml.h"

#include "../drone/PosSubscriber.h"
#include "../drone/PosPublisher.h"


 # define POSCONTROL_Z_P                    0.6f    // vertical velocity controller P gain default
 # define POSCONTROL_Z_I                    0.0f    // vertical velocity controller I gain default
 # define POSCONTROL_Z_D                    0.0f    // vertical velocity controller D gain default
 # define POSCONTROL_Z_IMAX                 1.0f // vertical velocity controller IMAX gain default

 # define POSCONTROL_Z_FILT_P_HZ            0.3f    // vertical velocity controller input filter
 # define POSCONTROL_Z_FILT_D_HZ            0.01f    // vertical velocity controller input filter for D
 
 # define POSCONTROL_XY_P                   0.6f //0.5f  //0.8f  // horizontal velocity controller P gain default 0.5
 # define POSCONTROL_XY_I                   0.1f    // horizontal velocity controller I gain default 0.2
 # define POSCONTROL_XY_D                   0.05f    // horizontal velocity controller D gain default 0.1
 # define POSCONTROL_XY_IMAX                1.0f // horizontal velocity controller IMAX gain default


// 串级PID控制器
//  # define POSCONTROL_POS_Z_P                    1.3f//1.0f    // vertical position controller P gain default
//  # define POSCONTROL_VEL_Z_P                    1.0f//5.0f    // vertical velocity controller P gain default
//  # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
//  # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
//  # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
//  # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
//  # define POSCONTROL_ACC_Z_I                    0.5f//1.0f    // vertical acceleration controller I gain default
//  # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
//  # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
//  # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
//  # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.6f//1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   0.80f//2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.40f//1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.40f//0.5f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
//  # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
//  # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D

 # define POSCONTROL_POS_Z_P                    1.0f//1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    1.0f//5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_VEL_Z_IMAX                 1000.0f // vertical velocity controller IMAX gain default
 # define POSCONTROL_VEL_Z_FILT_HZ              5.0f    // vertical velocity controller input filter
 # define POSCONTROL_VEL_Z_FILT_D_HZ            5.0f    // vertical velocity controller input filter for D
 # define POSCONTROL_ACC_Z_P                    0.05f    // vertical acceleration controller P gain default


 # define POSCONTROL_ACC_XY_MAX                 1.2f    
 # define POSCONTROL_ACC_Z_MAX                  1.2f 

 # define POSCONTROL_VEL_XY_MAX                    2.0f    // horizontal acceleration controller max acceleration default
 # define POSCONTROL_VEL_Z_MAX                     1.0f    // vertical acceleration controller max acceleration default
 # define POSCONTROL_VEL_YAW_MAX                   0.3f   // yaw acceleration controller max acceleration default

 # define DEFAULT_ACCURACY                          0.05f   // default accuracy for position control
 # define DEFAULT_YAW_ACCURACY                      0.1f   // default accuracy for yaw control

 #define DEFAULT_YAW 			 0.0f    // default yaw
 
class PosController {
public:
	PosController() = default;
    PosController(std::shared_ptr<PosSubscriber> pos_data, std::shared_ptr<PosPublisher> pos_publisher) :
		pos_data(pos_data), pos_publisher(pos_publisher)
	{
		std::cout << "PosController: Initializing Position Controller" << std::endl;
		// 默认的模糊规则库
		float rule_base[][qf_default] = {
			// delta kp 规则库
			{PB, PB, FUZZY_PM, FUZZY_PM, PS, ZO, ZO},
			{PB, PB, FUZZY_PM, PS, PS, ZO, NS},
			{FUZZY_PM, FUZZY_PM, FUZZY_PM, PS, ZO, NS, NS},
			{FUZZY_PM, FUZZY_PM, PS, ZO, NS, NM, NM},
			{PS, PS, ZO, NS, NS, NM, NM},
			{PS, ZO, NS, NM, NM, NM, NB},
			{ZO, ZO, NM, NM, NM, NB, NB},
			// delta ki 规则库
			{NB, NB, NM, NM, NS, ZO, ZO},
			{NB, NB, NM, NS, NS, ZO, ZO},
			{NB, NM, NS, NS, ZO, PS, PS},
			{NM, NM, NS, ZO, PS, FUZZY_PM, FUZZY_PM},
			{NM, NS, ZO, PS, PS, FUZZY_PM, PB},
			{ZO, ZO, PS, PS, FUZZY_PM, PB, PB},
			{ZO, ZO, PS, FUZZY_PM, FUZZY_PM, PB, PB},
			// delta kd 规则库
			{PS, NS, NB, NB, NB, NM, PS},
			{PS, NS, NB, NM, NM, NS, ZO},
			{ZO, NS, NM, NM, NS, NS, ZO},
			{ZO, NS, NS, NS, NS, NS, ZO},
			{ZO, ZO, ZO, ZO, ZO, ZO, ZO},
			{PB, PS, PS, PS, PS, PS, PB},
			{PB, FUZZY_PM, FUZZY_PM, FUZZY_PM, PS, PS, PB}};
		// 默认的模糊函数参数（membership function parameters）
		float mf_params[4 * qf_default] = {-3, -3, -2, 0,
										-3, -2, -1, 0,
										-2, -1, 0, 0,
										-1, 0, 1, 0,
										0, 1, 2, 0,
										1, 2, 3, 0,
										2, 3, 3, 0};
		struct FuzzyPID::Fuzzy_params fuzzy_params[8] = {
			{4, 1, 0, mf_params, rule_base, POSCONTROL_VEL_XY_MAX, POSCONTROL_VEL_XY_MAX, 8},
			{4, 1, 0, mf_params, rule_base, POSCONTROL_VEL_XY_MAX, POSCONTROL_VEL_XY_MAX, 8},
			{4, 1, 0, mf_params, rule_base, POSCONTROL_VEL_Z_MAX, POSCONTROL_VEL_Z_MAX, 8},
			{4, 1, 0, mf_params, rule_base, POSCONTROL_VEL_YAW_MAX, POSCONTROL_VEL_YAW_MAX, 8},
			{4, 1, 0, mf_params, rule_base, POSCONTROL_VEL_XY_MAX, POSCONTROL_ACC_XY_MAX, 8},
			{4, 1, 0, mf_params, rule_base, POSCONTROL_VEL_XY_MAX, POSCONTROL_ACC_XY_MAX, 8},
			{4, 1, 0, mf_params, rule_base, POSCONTROL_VEL_Z_MAX, POSCONTROL_ACC_Z_MAX, 8},
			{4, 1, 0, mf_params, rule_base, 100, 100, 8}
		};

		fuzzy_pid = FuzzyPID(fuzzy_params);
		pid_x = PID(
			"x_pos",
			POSCONTROL_XY_P,
			POSCONTROL_XY_I,
			POSCONTROL_XY_D,
			0,
			0,
			POSCONTROL_XY_IMAX,
			0
		);
		pid_y = PID(
			"y_pos",
			POSCONTROL_XY_P,
			POSCONTROL_XY_I,
			POSCONTROL_XY_D,
			0,
			0,
			POSCONTROL_XY_IMAX,
			0
		);
		pid_z = PID(
			"z_pos",
			POSCONTROL_Z_P,
			POSCONTROL_Z_I,
			POSCONTROL_Z_D,
			0,
			0,
			POSCONTROL_Z_IMAX,
			0
		);
		pid_yaw = PID(
			"yaw_pos",
			POSCONTROL_Z_FILT_P_HZ,
			0,
			POSCONTROL_Z_FILT_D_HZ,
			0,
			0,
			0,
			0
		);
		pid_px = PID(
			POSCONTROL_POS_XY_P,
			0,
			0
		);
		pid_py = PID(
			POSCONTROL_POS_XY_P,
			0,
			0
		);
		pid_pz = PID(
			POSCONTROL_POS_Z_P,
			0,
			0
		);
		pid_vx = PID(
			POSCONTROL_VEL_XY_P,
			POSCONTROL_VEL_XY_I,
			POSCONTROL_VEL_XY_D,
			0,
			0,
			POSCONTROL_VEL_XY_IMAX,
			0
		);
		pid_vy = PID(
			POSCONTROL_VEL_XY_P,
			POSCONTROL_VEL_XY_I,
			POSCONTROL_VEL_XY_D,
			0,
			0,
			POSCONTROL_VEL_XY_IMAX,
			0
		);
		pid_vz = PID(
			POSCONTROL_VEL_Z_P,
			0,
			0
		);
		pid_x_defaults=PID::readPIDParameters("pos_config.yaml","pos_x");
		pid_y_defaults=PID::readPIDParameters("pos_config.yaml","pos_y");
		pid_z_defaults=PID::readPIDParameters("pos_config.yaml","pos_z");
		pid_yaw_defaults=PID::readPIDParameters("pos_config.yaml","pos_yaw");
		pid_px_defaults=PID::readPIDParameters("pos_config.yaml","pos_px");
		pid_py_defaults=PID::readPIDParameters("pos_config.yaml","pos_py");
		pid_pz_defaults=PID::readPIDParameters("pos_config.yaml","pos_pz");
		pid_vx_defaults=PID::readPIDParameters("pos_config.yaml","pos_vx");
		pid_vy_defaults=PID::readPIDParameters("pos_config.yaml","pos_vy");
		pid_vz_defaults=PID::readPIDParameters("pos_config.yaml","pos_vz");
		limit_defaults=readLimits("pos_config.yaml","limits");
		reset_pid();
		reset_limits();
    }

    bool publish_setpoint_world(Vector4f now, Vector4f target, double accuracy = DEFAULT_ACCURACY, double yaw_accuracy = DEFAULT_YAW_ACCURACY);
    bool trajectory_setpoint(Vector4f pos_now, Vector4f pos_target, double accuracy = DEFAULT_ACCURACY, double yaw_accuracy = DEFAULT_YAW_ACCURACY);
    bool trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, double accuracy = DEFAULT_ACCURACY, double yaw_accuracy = DEFAULT_YAW_ACCURACY);
    bool trajectory_circle(float a, float b, float height, float dt, float default_yaw = DEFAULT_YAW, float yaw = DEFAULT_YAW);
    bool trajectory_generator_world(double speed_factor, std::array<double, 3> q_goal, Vector3f max_speed = {100,100,100}, Vector3f max_accel = {100,100,100}, float tar_yaw = DEFAULT_YAW);
	bool trajectory_setpoint_world(Vector4f pos_now, Vector4f pos_target, PID::Defaults defaults, double accuracy, double yaw_accuracy, bool calculate_or_get_vel = false, float vel_x = DEFAULT_VELOCITY, float vel_y = DEFAULT_VELOCITY);
	float get_speed_max();
	float get_turn_rate_speed_max();
	float get_accel_max();
	float get_decel_max();
	float get_jerk_max();
	// virtual float get_time(void) {
		// return (node->get_clock()->now().nanoseconds() / 1000)/1000000.0;
	// }
	struct Limits_t{
		float speed_max_xy = POSCONTROL_VEL_XY_MAX;
		float speed_max_z = POSCONTROL_VEL_Z_MAX;
		float speed_max_yaw = POSCONTROL_VEL_YAW_MAX; 
		float accel_max_xy = POSCONTROL_ACC_XY_MAX;
		float accel_max_z = POSCONTROL_ACC_Z_MAX;
		float accel_max_yaw = 0;
	};
	struct Limits_t readLimits(const std::string& filename, const std::string& section);
	Limits_t get_limits_defaults(){
		return limit_defaults;
	}
	void set_limits(struct Limits_t limits);
	void reset_limits();
	void set_pid(PID& pid, PID::Defaults defaults);
	void reset_pid();
	void reset_pid_config();
	void set_dt(float dt){
		this->dt = dt;
	}
	bool auto_tune(Vector4f pos_now, Vector4f pos_target, uint32_t delayMsec, bool tune_x=true, bool tune_y=true, bool tune_z=true, bool tune_yaw=true);

	PID::Defaults
		pid_x_defaults,
		pid_y_defaults,
		pid_z_defaults,
		pid_yaw_defaults,
		pid_px_defaults,
		pid_py_defaults,
		pid_pz_defaults,
		pid_vx_defaults,
		pid_vy_defaults,
		pid_vz_defaults;
	Limits_t limit_defaults;

	std::shared_ptr<PosSubscriber> pos_data;
	std::shared_ptr<PosPublisher> pos_publisher;
protected:
// private:
	//
	FuzzyPID fuzzy_pid;
	PID pid_x;
	PID pid_y;
	PID pid_z;
	PID pid_yaw;
	// 串级PID控制器
	PID pid_px;
	PID pid_py;
	PID pid_pz;
	PID pid_vx;
	PID pid_vy;
	PID pid_vz;

	std::unique_ptr<TrajectoryGenerator> _trajectory_generator;

	float max_speed_xy = POSCONTROL_VEL_XY_MAX;
	float max_speed_z = POSCONTROL_VEL_Z_MAX;
	float max_speed_yaw = POSCONTROL_VEL_YAW_MAX;
	float max_accel_xy = POSCONTROL_ACC_XY_MAX;
	float max_accel_z = POSCONTROL_ACC_Z_MAX;
	float max_dccel_xy = POSCONTROL_ACC_XY_MAX;
	float max_jerk = 0.5;
	// float max_speed_yaw = POSCONTROL_VEL_YAW_MAX;


	float default_accuracy = DEFAULT_ACCURACY;
	float default_yaw_accuracy = DEFAULT_YAW_ACCURACY;
	float default_yaw = DEFAULT_YAW;
	float dt = 0.1;
	float dt_pid_p_v = 1;

	Vector3f input_pos_xyz(Vector3f now, Vector3f target, bool fuzzy = false);
	Vector4f input_pos_xyz_yaw(Vector4f now, Vector4f target, bool fuzzy = false, bool calculate_or_get_vel = false, float vel_x = DEFAULT_VELOCITY, float vel_y = DEFAULT_VELOCITY);
	Vector4f input_pos_xyz_yaw_without_vel(Vector4f now, Vector4f target);
	Vector4f input_pos_vel_1_xyz_yaw(Vector4f now, Vector4f target);
	Vector4f input_pos_vel_xyz_yaw(Vector4f now, Vector4f target);


	Vector4f _pos_target;
	Vector4f _pos_desired;
	TUNE_ID_t get_autotuneID();
	float autotuneWORKCycle(float feedbackVal, TUNE_ID_t tune_id, bool& result, uint32_t delayMsec);
};