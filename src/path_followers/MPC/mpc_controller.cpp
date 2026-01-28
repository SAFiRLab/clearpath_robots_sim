#include "clearpath_robots_sim/path_followers/MPC/mpc_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace clearpath_robots_sim
{

MPCController::MPCController(double dt, unsigned int max_iterations, std::shared_ptr<DynamicModel> dynamic_model,
                  unsigned int horizon)
: PathFollowerController(dt, max_iterations, dynamic_model), Node("mpc_path_follower"), horizon_(horizon)
{
    A_ = Eigen::MatrixXd::Zero(state_size_, state_size_);
    B_ = Eigen::MatrixXd::Zero(state_size_, control_size_);
    C_ = Eigen::MatrixXd::Zero(output_size_, state_size_);

    Q_ = Eigen::MatrixXd::Identity(horizon_ * output_size_, horizon_ * output_size_);

    alpha_ = Eigen::VectorXd::Zero(horizon_ * state_size_);
    R_ = Eigen::MatrixXd::Zero(horizon_ * state_size_, horizon_ * control_size_);

    P_ = Eigen::MatrixXd::Zero(horizon_ * control_size_, horizon_ * control_size_);
    q_ = Eigen::VectorXd::Zero(horizon_ * control_size_);

    control_input_to_publish_ = Eigen::VectorXd::Zero(control_size_);

    // ROS2
    // Publisher
    control_input_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/platform_velocity_controller/cmd_vel", 10);

    // Subscriptions
    path_subscription_ = this->create_subscription<nav_msgs::msg::Path>(
        "/reference_trajectory", 10, std::bind(&MPCController::pathCallback, this, std::placeholders::_1));
    current_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/groundTruth/poseStamped", 10, std::bind(&MPCController::currentPoseCallback, this, std::placeholders::_1));
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, std::bind(&MPCController::imuCallback, this, std::placeholders::_1));

    // Timer
    auto publish_period = std::chrono::milliseconds(10); // 100 Hz
    this->create_wall_timer(publish_period, std::bind(&MPCController::publishControlInput, this));
}

void MPCController::solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference)
{
    
    // TODO: How to define the control_state for linearization? Might be better to take the current control input
    dynamic_model_->linearize(state, Eigen::VectorXd::Zero(control_size_), dt_, A_, B_, C_);

    R_.setZero();
    Eigen::MatrixXd A_k = A_;
    for (unsigned int k = 0; k < horizon_; k++)
    {
        alpha_.segment(k * state_size_, state_size_) = A_k * state;

        for (unsigned int j = 0; j <= k; ++j)
        {
            Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(state_size_, state_size_);
            for (unsigned int p = 0; p < k - j; ++p)
                A_power = A_ * A_power;

            R_.block(k * state_size_, j * control_size_, state_size_, control_size_) = A_power * B_;
        }

        A_k = A_ * A_k;
    }

    Eigen::MatrixXd Cy = Eigen::MatrixXd::Zero(horizon_ * output_size_, horizon_ * state_size_);
    for (unsigned int k = 0; k < horizon_; ++k)
        Cy.block(k * output_size_, k * state_size_, output_size_, state_size_) = C_;

    Eigen::VectorXd alpha_y = Cy * alpha_;
    Eigen::MatrixXd R_y = Cy * R_;

    Eigen::VectorXd reference_vector(horizon_ * output_size_);
    for (unsigned int k = 0; k < horizon_; ++k)
        reference_vector.segment(k * output_size_, output_size_) = reference.col(k);

    Eigen::VectorXd f = alpha_y - reference_vector;
    P_ = R_y.transpose() * Q_ * R_y;
    q_ = R_y.transpose() * Q_ * f;

    Eigen::VectorXd u_opt = solveQP();

    // U_opt = [accel, yaw_accel]
    // We control the robot in velocity and angular velocity
    control_input_mutex_.lock();
    control_input_to_publish_(0) = state(3) * u_opt(0);
    control_input_to_publish_(1) = state(4) * u_opt(1);
    control_input_mutex_.unlock();
}

Eigen::VectorXd MPCController::solveQP()
{
    return -P_.ldlt().solve(q_);
}

// This function publishes the control input to a ROS2 topic at a spcecified rate
void MPCController::publishControlInput()
{
    control_input_mutex_.lock();
    geometry_msgs::msg::TwistStamped control_msg;
    control_msg.header.stamp = rclcpp::Clock().now();
    control_msg.twist.linear.x = control_input_to_publish_(0);
    control_msg.twist.angular.z = control_input_to_publish_(1);
    control_input_mutex_.unlock();

    control_input_publisher_->publish(control_msg);
}

void MPCController::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    // Start the trajectory follower thread if not already running
    if (!trajectory_follower_thread_.joinable())
    {
        Eigen::MatrixXd reference( output_size_, msg->poses.size() );
        for (size_t i = 0; i < msg->poses.size(); i++)
        {
            reference(0, i) = msg->poses[i].pose.position.x;
            reference(1, i) = msg->poses[i].pose.position.y;
        }
        trajectory_follower_thread_ = std::thread(&MPCController::trajectoryFollowerThread, this, reference);
    }
}

void MPCController::trajectoryFollowerThread(const Eigen::MatrixXd &reference)
{
    unsigned int reference_size = reference.cols();
    unsigned int current_ref_idx = 0;

    while (rclcpp::ok() && current_ref_idx < reference_size)
    {
        // Get current robot state
        current_robot_state_pose_mutex_.lock();
        current_robot_state_imu_mutex_.lock();
        Eigen::VectorXd state(state_size_);
        state(0) = current_robot_state_.x;
        state(1) = current_robot_state_.y;
        state(2) = current_robot_state_.yaw;
        state(3) = current_robot_state_.velocity;
        state(4) = current_robot_state_.yaw_rate;

        // Set the current reference index to the closest point on the path
        for (size_t i = 0; i < reference_size; i++)
        {
            double dist_current = std::hypot(reference(0, current_ref_idx) - state(0),
                                             reference(1, current_ref_idx) - state(1));
            double dist_i = std::hypot(reference(0, i) - state(0),
                                       reference(1, i) - state(1));

            if (dist_i < dist_current)
                current_ref_idx = i;
        }
        current_robot_state_pose_mutex_.unlock();
        current_robot_state_imu_mutex_.unlock();

        // Prepare the trajectory reference for the horizon since the current reference is a geometric path and 
        // not a time-parameterized trajectory
        Eigen::MatrixXd mpc_reference(output_size_, horizon_);
        for (unsigned int k = 0; k < horizon_; k++)
        {
            unsigned int ref_idx = current_ref_idx + k;
            if (ref_idx >= reference_size)
                ref_idx = reference_size - 1;
            
            mpc_reference(0, k) = reference(0, ref_idx);
            mpc_reference(1, k) = reference(1, ref_idx);
        }

        // Solve MPC
        solve(state, mpc_reference);
    }
}

void MPCController::currentPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    current_robot_state_pose_mutex_.lock();
    current_robot_state_.velocity = (msg->pose.position.x - current_robot_state_.x) / 
                                    ((msg->header.stamp.nanosec - current_robot_state_.stamp) / 1.0e9);
    current_robot_state_.stamp = msg->header.stamp.nanosec;
    current_robot_state_.x = msg->pose.position.x;
    current_robot_state_.y = msg->pose.position.y;
    current_robot_state_.yaw = yaw;
    current_robot_state_pose_mutex_.unlock();
}

void MPCController::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    current_robot_state_imu_mutex_.lock();
    current_robot_state_.yaw_rate = msg->angular_velocity.z;
    current_robot_state_imu_mutex_.unlock();
}

}; // namespace clearpath_robots_sim
