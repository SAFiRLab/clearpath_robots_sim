#include "clearpath_robots_sim/path_followers/MPC/mpc_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <casadi/casadi.hpp>


namespace clearpath_robots_sim
{

MPCController::MPCController(double dt, unsigned int max_iterations, std::shared_ptr<DynamicModel> dynamic_model,
                  unsigned int horizon)
: PathFollowerController(dt, max_iterations, dynamic_model), Node("mpc_path_follower"), horizon_(horizon), 
trajectory_generator_(std::dynamic_pointer_cast<DiffDriveModel>(dynamic_model_), dt_)
{
    control_input_to_publish_ = Eigen::VectorXd::Zero(control_size_);

    // ROS2
    // Publisher
    control_input_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/platform_velocity_controller/cmd_vel", 10);
    traj_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/traj_from_ref", 10);

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

void MPCController::buildSolver()
{
    int nx = state_size_;
    int nu = control_size_;
    int N  = horizon_;

    // Decision variables
    casadi::MX X = casadi::MX::sym("X", nx, N + 1);
    casadi::MX U = casadi::MX::sym("U", nu, N);

    // Parameters
    casadi::MX x0   = casadi::MX::sym("x0", nx);
    casadi::MX Xref = casadi::MX::sym("Xref", nx, N);

    // Cost
    casadi::MX cost = 0;
    casadi::DM Q = casadi::DM::eye(nx);
    casadi::DM R = casadi::DM::eye(nu) * 0.1;

    for (int k = 0; k < N; ++k)
    {
        casadi::MX e = X(casadi::Slice(), k) - Xref(casadi::Slice(), k);
        cost += mtimes(e.T(), mtimes(Q, e));
        cost += mtimes(U(casadi::Slice(), k).T(), mtimes(R, U(casadi::Slice(), k)));
    }

    // Constraints
    std::vector<casadi::MX> g;

    // Initial condition
    g.push_back(X(casadi::Slice(), 0) - x0);

    // Dynamics
    for (int k = 0; k < N; ++k)
    {
        casadi::MX x_next = dynamic_model_->f(X(casadi::Slice(), k), U(casadi::Slice(), k), dt_);
        g.push_back(X(casadi::Slice(), k + 1) - x_next);
    }

    casadi::MX G = vertcat(g);

    // NLP
    casadi::MXDict nlp;
    nlp["x"] = casadi::MX::vertcat({reshape(X, nx * (N + 1), 1),
                        reshape(U, nu * N, 1)});
    nlp["f"] = cost;
    nlp["g"] = G;
    nlp["p"] = casadi::MX::vertcat({x0, reshape(Xref, nx * N, 1)});

    solver_ = nlpsol("solver", "ipopt", nlp);

    // ---------- Bounds ----------
    std::vector<double> x_lb, x_ub, u_lb, u_ub;
    dynamic_model_->getStateBounds(x_lb, x_ub);
    dynamic_model_->getControlBounds(u_lb, u_ub);

    std::vector<double> lbx, ubx;

    // State bounds
    for (int k = 0; k <= N; ++k)
    {
        lbx.insert(lbx.end(), x_lb.begin(), x_lb.end());
        ubx.insert(ubx.end(), x_ub.begin(), x_ub.end());
    }

    // Control bounds
    for (int k = 0; k < N; ++k)
    {
        lbx.insert(lbx.end(), u_lb.begin(), u_lb.end());
        ubx.insert(ubx.end(), u_ub.begin(), u_ub.end());
    }

    lbx_ = casadi::DM(lbx);
    ubx_ = casadi::DM(ubx);

    lbg_ = casadi::DM::zeros(G.size1());
    ubg_ = casadi::DM::zeros(G.size1());

    solver_built_ = true;
}

void MPCController::solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference)
{
    if (!solver_built_)
        buildSolver();

    using namespace casadi;

    // Pack parameters
    std::vector<double> p;

    // x0
    for (int i = 0; i < state_size_; ++i)
        p.push_back(state(i));

    // Xref
    for (int k = 0; k < horizon_; ++k)
        for (int i = 0; i < state_size_; ++i)
            p.push_back(reference(i, k));

    std::map<std::string, DM> args;
    args["lbx"] = lbx_;
    args["ubx"] = ubx_;
    args["lbg"] = lbg_;
    args["ubg"] = ubg_;
    args["p"]   = DM(p);

    auto res = solver_(args);
    DM sol = res.at("x");

    // Extract first control
    int offset = state_size_ * (horizon_ + 1);
    double a   = static_cast<double>(sol(offset));
    double yaw_acc = static_cast<double>(sol(offset + 1));

    control_input_mutex_.lock();
    control_input_to_publish_(0) = a * dt_;
    control_input_to_publish_(1) = yaw_acc * dt_;
    control_input_mutex_.unlock();
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

        // Publish traj
        /*nav_msgs::msg::Path traj_msg;
        for (size_t i = 0; i < traj.size(); i++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = traj.at(i).position(1);
            pose.pose.position.z = traj.at(i).position(0);
            pose.pose.position.y = 1.0;
            traj_msg.poses.push_back(pose);
        }
        traj_publisher_->publish(traj_msg);*/

        trajectory_follower_thread_ = std::thread(&MPCController::trajectoryFollowerThread, this, traj);
    }
}

void MPCController::trajectoryFollowerThread(const std::vector<TrajectoryPoint> &reference_traj)
{
    unsigned int reference_size = reference_traj.size();
    unsigned int current_ref_idx = 0;
    std::vector<unsigned int> reached_indices;

    // Print reference
    Eigen::IOFormat fmt(3, 0, ", ", "\n", "[", "]");
    //std::cout << "A:\n" << reference.format(fmt) << std::endl;

    rclcpp::Rate rate(1.0 / 0.01);

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
            if (dist < 0.01)
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
                mpc_reference(3, k) = reference_traj.back().velocity;
                mpc_reference(4, k) = reference_traj.back().yaw_rate;
            }
            else
            {
                mpc_reference(0, k) = reference_traj.at(current_ref_idx + k).position(0);
                mpc_reference(1, k) = reference_traj.at(current_ref_idx + k).position(1);
                mpc_reference(2, k) = reference_traj.at(current_ref_idx + k).yaw;
                mpc_reference(3, k) = reference_traj.at(current_ref_idx + k).velocity;
                mpc_reference(4, k) = reference_traj.at(current_ref_idx + k).yaw_rate;
            }
        }

        //std::cout << "B:\n" << mpc_reference.format(fmt) << std::endl;

        nav_msgs::msg::Path traj_msg;
        for (size_t i = 0; i < horizon_; i++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = mpc_reference(1, i);
            pose.pose.position.z = mpc_reference(0, i);
            pose.pose.position.y = 1.0;
            traj_msg.poses.push_back(pose);
        }
        traj_publisher_->publish(traj_msg);

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

MPCController::~MPCController()
{}

}; // namespace clearpath_robots_sim
