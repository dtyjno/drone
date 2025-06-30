// 来自 https://www.cnblogs.com/21203-iHome/p/10505369.html

#include <array>
#include <Eigen/Core>
// #include "Vector3.h"
 
struct RobotState 
{
    std::array<double, 3> q_d{};
};
    
    
class TrajectoryGenerator 
{

public:
  
    TrajectoryGenerator(){};
    // Creates a new TrajectoryGenerator instance for a target q.
    // @param[in] speed_factor: General speed factor in range [0, 1].
    // @param[in] q_goal: Target joint positions.
    TrajectoryGenerator(double speed_factor, const std::array<double, 3> q_goal);
    void set_dq_max(const std::array<double, 3> d_q_max);
    void set_dqq_max(const std::array<double, 3> d_q_max);

    // Calculate joint positions for use inside a control loop.
    bool operator()(const RobotState& robot_state, double time);
    
    using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;

    Vector3d delta_q_d;

private:
    // using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::ColMajor>;
    
    using Vector3i = Eigen::Matrix<int, 3, 1, Eigen::ColMajor>;

    bool calculateDesiredValues(double t, Vector3d* delta_q_d); // generate joint trajectory 
    
    void calculateSynchronizedValues();

    static constexpr double DeltaQMotionFinished = 1e-6;
    
    const Vector3d q_goal_;

    Vector3d q_start_;  // initial joint position
    Vector3d delta_q_;  // the delta angle between start and goal

    Vector3d dq_max_sync_;
    Vector3d t_1_sync_;    
    Vector3d t_2_sync_;
    Vector3d t_f_sync_;    
    Vector3d q_1_;  // q_1_ = q(\tau) - q_start_     

    double time_ = 0.0;

    Vector3d dq_max_ = (Vector3d() << 2.0, 2.0, 2.0).finished();   

    Vector3d ddq_max_ = (Vector3d() << 0.6, 0.6, 0.6).finished(); 

    Vector3d dq;
};

