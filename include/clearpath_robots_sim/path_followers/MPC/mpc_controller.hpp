#ifndef _CLEARPATH_ROBOTS_SIM__PATH_FOLLOWER__MPC__MPC_CONTROLLER_
#define _CLEARPATH_ROBOTS_SIM__PATH_FOLLOWER__MPC__MPC_CONTROLLER_

// External Libraries
// Eigen
#include <Eigen/Dense>

// Local
#include "clearpath_robots_sim/path_followers/path_follower_controller.hpp"
#include "clearpath_robots_sim/path_followers/MPC/trajectory_generator.hpp"

// System
#include <memory>
#include <mutex>
#include <thread>

// ROS2
#include "rclcpp/rclcpp.hpp"
// ROS Messages
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>


namespace clearpath_robots_sim
{

struct RobotState
{
    double x;
    double y;
    double yaw;
    double velocity;
    double yaw_rate;

    rclcpp::Time stamp;

    bool init = false;
}; // RobotState


class MPCController : public PathFollowerController, public rclcpp::Node
{
public:
    MPCController(double dt, int max_iterations, std::shared_ptr<DynamicModel> dynamic_model,
                  int horizon);

    ~MPCController();

    void solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference) override;

private:

    void buildSolver();
    void warmStartState(casadi::DMDict &res);
    void trajectoryFollowerThread(const std::vector<TrajectoryPoint> &reference_traj);
    void publishControlInput();
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // User-defined attributes
    int horizon_;

    Eigen::VectorXd control_input_to_publish_;

    std::mutex control_input_mutex_;
    std::mutex current_robot_state_pose_mutex_;
    std::mutex current_robot_state_imu_mutex_;
    RobotState current_robot_state_;
    std::thread trajectory_follower_thread_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr control_input_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::TimerBase::SharedPtr publish_control_timer_;

    casadi::Function solver_;
    casadi::DM lbx_, ubx_, lbg_, ubg_;
    bool solver_built_ = false;
    casadi::DM x_init_;
    bool first_solve_ = true;

    TrajectoryGenerator trajectory_generator_;

}; // class MPCController

}; // namespace clearpath_robots_sim

#endif // _CLEARPATH_ROBOTS_SIM__PATH_FOLLOWER__MPC__MPC_CONTROLLER_
