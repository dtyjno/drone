#ifndef PID_H
#define PID_H
#include <cfloat>
#define DEFAULT_VELOCITY FLT_MAX
#include "Readyaml.h"

// #define pid_debug_print
// #define fuzzy_pid_dead_zone

class PID
{
public:
    struct Defaults
    {
        float p = 0;
        float i = 0;
        float d = 0;
        float ff = 0;
        float dff = 0;
        float imax = 10;
        // bool _use_vel = true;
        float filt_T_hz = 0;
        float filt_E_hz = 0;
        float filt_D_hz = 0;
        float srmax = 0;
        float srtau = 0;
    };
#ifdef READYAML_H
    static Defaults readPIDParameters(const std::string &filename, const std::string &pid_name)
    {
        YAML::Node config = Readyaml::readYAML(filename);
        Defaults pidDefaults;
        pidDefaults.p = config[pid_name]["p"].as<double>();
        pidDefaults.i = config[pid_name]["i"].as<double>();
        pidDefaults.d = config[pid_name]["d"].as<double>();
        pidDefaults.ff = config[pid_name]["ff"].as<double>();
        pidDefaults.dff = config[pid_name]["dff"].as<double>();
        pidDefaults.imax = config[pid_name]["imax"].as<double>();
        std::cout << "读取PID参数: " << pid_name << " p: " << pidDefaults.p
                  << ", i: " << pidDefaults.i
                  << ", d: " << pidDefaults.d
                  << ", ff: " << pidDefaults.ff
                  << ", dff: " << pidDefaults.dff
                  << ", imax: " << pidDefaults.imax
                  << std::endl;
        return pidDefaults;
    };
#endif
    PID(float kp, float ki, float kd, float kff = 0, float kdff = 0, float kimax = 2, float srmax = 0);
    PID(const PID::Defaults &defaults);
    PID() {};

    void set_pid_info();
    void set_gains(const PID::Defaults &defaults);
    void set_gains(float kp, float ki, float kd);
    void set_pid(float kp, float ki, float kd);
    void set_pid(const PID::Defaults &defaults);
    void get_pid(float &kp, float &ki, float &kd);
    float update_all(float measurement, float target, float dt, float limit, float velocity = DEFAULT_VELOCITY);
    void update_i(float dt, float limit);
    void print_update_info();

    float smooth_data(float current_value, float alpha);

    // internal variables
    float _integrator; // integrator value
    float _target;     // target value to enable filtering
    float _error;      // error value to enable filtering
    float _derivative; // derivative value to enable filtering

    struct PIDInfo
    {
        float target;
        float actual;
        float error;
        float _kP;
        float _kI;
        float _kD;
        float P;
        float I;
        float D;
        float FF;
        float DFF;
        float output;
        float output_max;
        float Dmod;
        float slew_rate;
        bool limit;
        bool PD_limit;
        bool reset;
        bool I_term_set;
    };
    PIDInfo _pid_info;

private:
    float _kp, _ki, _kd;
    float _kff;
    float _kimax;
    float filt_T_hz;
    float filt_E_hz;
    float filt_D_hz;
    float srmax;
    float srtau;
    float _kdff;
};

#endif // PID_H