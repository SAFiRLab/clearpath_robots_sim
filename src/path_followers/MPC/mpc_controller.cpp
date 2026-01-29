#include "clearpath_robots_sim/path_followers/MPC/mpc_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>


namespace clearpath_robots_sim
{

MPCController::MPCController(double dt, unsigned int max_iterations, std::shared_ptr<DynamicModel> dynamic_model,
                  unsigned int horizon)
: PathFollowerController(dt, max_iterations, dynamic_model), Node("mpc_path_follower"), horizon_(horizon), 
trajectory_generator_(std::dynamic_pointer_cast<DiffDriveModel>(dynamic_model_), dt_)
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
    publish_control_timer_ = this->create_wall_timer(publish_period, std::bind(&MPCController::publishControlInput, this));

    current_robot_state_.stamp = this->get_clock()->now();
}

void MPCController::solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference)
{
    //std::cout << "MPCController::solve started" << std::endl;
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
    control_input_to_publish_(0) = u_opt(0) * dt_;
    control_input_to_publish_(1) = u_opt(1) * dt_;
    control_input_mutex_.unlock();
}

Eigen::VectorXd MPCController::solveQP()
{
    //std::cout << "Solving QP..." << std::endl;
    return -P_.ldlt().solve(q_);
}

// This function publishes the control input to a ROS2 topic at a spcecified rate
void MPCController::publishControlInput()
{
    control_input_mutex_.lock();
    geometry_msgs::msg::TwistStamped control_msg;
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
        std::vector<Eigen::Vector2d> reference_path;
        for (size_t i = 0; i < msg->poses.size(); i++)
        {
            Eigen::Vector2d point;
            point(0) = msg->poses[i].pose.position.z;
            point(1) = msg->poses[i].pose.position.x;
            reference_path.push_back(point);
        }
        std::vector<TrajectoryPoint> traj = trajectory_generator_.generateTrajectory(reference_path);
        trajectory_follower_thread_ = std::thread(&MPCController::trajectoryFollowerThread, this, traj);
    }
}

void MPCController::trajectoryFollowerThread(const std::vector<TrajectoryPoint> &reference_traj)
{
    std::cout << "Starting trajectory follower thread..." << std::endl;
    unsigned int reference_size = reference_traj.size();
    unsigned int current_ref_idx = 0;
    std::vector<unsigned int> reached_indices;

    // Print reference
    Eigen::IOFormat fmt(3, 0, ", ", "\n", "[", "]");
    //std::cout << "A:\n" << reference.format(fmt) << std::endl;

    rclcpp::Rate rate(1.0 / dt_);

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
        current_robot_state_pose_mutex_.unlock();
        current_robot_state_imu_mutex_.unlock();

        double min_dist = std::numeric_limits<double>::max();
        // Set the current reference index to the closest point on the path
        for (size_t i = 0; i < reference_size; i++)
        {
            if (std::find(reached_indices.begin(), reached_indices.end(), i) != reached_indices.end())
                continue;

            double dist = std::sqrt(std::pow(state(0) - reference_traj.at(i).position(0), 2) + std::pow(state(1) - reference_traj.at(i).position(1), 2));
            if (dist < 0.1)
            {
                reached_indices.push_back(i);
                current_ref_idx = i;
            }
            else if (dist < min_dist)
            {
                min_dist = dist;
                current_ref_idx = i;
            }
        }
        std::cout << "Current reference index: " << current_ref_idx << std::endl;

        if (current_ref_idx >= reference_size - 1)
        {
            std::cout << "Reached the end of the reference path" << std::endl;
            control_input_mutex_.lock();
            control_input_to_publish_(0) = 0.0;
            control_input_to_publish_(1) = 0.0;
            control_input_mutex_.unlock();
            break;
        }

        Eigen::MatrixXd mpc_reference(output_size_, horizon_);
        for (unsigned int k = 0; k < horizon_; k++)
        {
            if (current_ref_idx + k >= reference_size)
            {
                mpc_reference(0, k) = reference_traj.back().position(0);
                mpc_reference(1, k) = reference_traj.back().position(1);
                mpc_reference(2, k) = reference_traj.back().yaw;
            }
            else
            {
                mpc_reference(0, k) = reference_traj.at(current_ref_idx + k).position(0);
                mpc_reference(1, k) = reference_traj.at(current_ref_idx + k).position(1);
                mpc_reference(2, k) = reference_traj.at(current_ref_idx + k).yaw;
            }
        }

        std::cout << "B:\n" << mpc_reference.format(fmt) << std::endl;

        // Solve MPC
        solve(state, mpc_reference);

        rate.sleep();
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

    rclcpp::Time now = msg->header.stamp;
    rclcpp::Time last(current_robot_state_.stamp);
    double dt = (now - last).seconds();

    current_robot_state_pose_mutex_.lock();
    current_robot_state_.velocity = (msg->pose.position.x - current_robot_state_.x) / dt;
    current_robot_state_.stamp = msg->header.stamp;
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
