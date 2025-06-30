#ifndef FUZZY_PID_H
#define FUZZY_PID_H

#include <string>
#include <cmath>

using namespace std;

// Fuzzy quantity fields
enum quantity_fields
{
    qf_small = 5,
    qf_middle = 7,
    qf_large = 8
};

#define qf_default qf_middle

// dof: degree of freedom
#define DOF 6

#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3

// #define pid_debug_print
// #define pid_dead_zone
// #define pid_integral_limit

// #define fuzzy_pid_debug_print
#define fuzzy_pid_dead_zone
#define fuzzy_pid_integral_limit
///// #define fuzzy_pid_rule_base_deep_copy

// #define pid_params_count 7

#define torque_mode 1
#define position_mode 2
#define control_mode position_mode

#if control_mode == position_mode
// #define max_error 100.0f
// #define max_delta_error 100.0f
#else
#define max_error 12.0f
#define max_delta_error 12.0f
#endif

// dof: degree of freedom
#define DOF 6
class FuzzyPID
{
public:
    // mf_type:模糊隶属度函数种类
    // 0: gaussmf
    // 1: gbellmf
    // 2: sigmf
    // 3: trapmf
    // 5: zmf
    // else: trimf
    // fo_type:模糊运算符种类
    // 1-2: and
    // 3-5: or
    // else: equilibrium
    // df_type:去模糊化种类
    // 0: moc
    struct Fuzzy_params
    {

        unsigned int mf_type;
        unsigned int fo_type;
        unsigned int df_type;
        float *mf_params;
        float (*rule_base)[qf_default];
        float max_error;
        float max_delta_error;
        int control_id_count;
    };
    FuzzyPID();

    // delta_k: the ratio of kp, ki, kd to delta kp, delta ki, delta kd
    FuzzyPID(struct FuzzyPID::Fuzzy_params *fuzzy_params);

    ~FuzzyPID() {};

    void Init_FuzzyPID(struct FuzzyPID::Fuzzy_params *fuzzy_params);

    void fuzzy_control(float e, float de);

    // real: 实际值(初始为大值)
    // idea: 目标值(初始为小值)
    // kp: 比例系数 输出kp = kp + delta_kp
    void fuzzy_pid_control(float real, float idea, int control_id, float &kp, float &ki, float &kd, float delta_k = 2.0f);

    void showMf(int type, float *mf_paras);
    void showFuzzyPIDInfo(int control_id = 0);
    void set_rule_base(int i, int j, float value);

private:
    int control_id_count;
    // int dof = DOF;

    float target; // 系统的控制目标
    float actual; // 采样获得的实际值
    // 模糊控制器
    float Ke;  // 误差输出
    float Kde; // 误差的变化率输出

    unsigned int fuzzy_input_num;  // 模糊输入数量
    unsigned int fuzzy_output_num; // 模糊输出数量
    unsigned int fo_type;          // 模糊运算符种类
    unsigned int *mf_type;         // 模糊隶属度函数种类
    float *mf_params;              // 模糊隶属度函数参数
    unsigned int df_type;          // 去模糊化种类
    float *rule_base;              // 模糊规则表
    float *fuzzy_output;           // 模糊输出

    // PID 控制器
    float kp; // 比例系数
    float ki; // 积分系数
    float kd; // 微分系数

    float delta_kp_max; // delta_kp输出的上限 = kp/delta_k
    float delta_ki_max; // delta_ki输出上限 = ki/delta_k
    float delta_kd_max; // delta_kd输出上限 = kd/delta_k

    float delta_kp; // kp输出增量
    float delta_ki; // ki输出增量
    float delta_kd; // kd输出增量

    float *last_error;      // 上一次的误差
    float current_error;    // 当前的误差
    float *error_max;       // 误差基本论域上限/误差限幅
    float *delta_error_max; // 误差变化率基本论域的上限/误差变化限幅

    // float linear_adaptive_kp; // 线性自适应比例系数

    void moc(const float *joint_membership, const unsigned int *index, const unsigned int *count);
    void df(const float *joint_membership, const unsigned int *output, const unsigned int *count, int df_type);
};

#endif // FUZZY_PID_H
