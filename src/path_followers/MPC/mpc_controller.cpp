#include "clearpath_robots_sim/path_followers/MPC/mpc_controller.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>
#include <casadi/casadi.hpp>


namespace clearpath_robots_sim
{

MPCController::MPCController(double dt, int max_iterations, std::shared_ptr<DynamicModel> dynamic_model,
                  int horizon)
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
    int no = output_size_;
    int N  = horizon_;

    // Decision variables
    casadi::MX X = casadi::MX::sym("X", nx, N + 1);
    casadi::MX U = casadi::MX::sym("U", nu, N);

    // Parameters
    casadi::MX x0   = casadi::MX::sym("x0", nx);
    casadi::MX Xref = casadi::MX::sym("Xref", no, N);

    // Cost
    casadi::MX cost = 0;
    casadi::DM Q = casadi::DM::eye(no);
    /*Q(0, 0) = 1.0;
    Q(1, 1) = 1.0;
    Q(2, 2) = 20.0;*/
    casadi::DM R = casadi::DM::eye(nu) * 0.1;

    for (int k = 0; k < N; ++k)
    {
        casadi::MX e = X(casadi::Slice(), k) - Xref(casadi::Slice(), k);
        cost += mtimes(e.T(), mtimes(Q, e));
        cost += mtimes(U(casadi::Slice(), k).T(), mtimes(R, U(casadi::Slice(), k)));
    }

    casadi::MX eN = X(casadi::Slice(), N) - Xref(casadi::Slice(), N-1);
    cost += mtimes(eN.T(), mtimes(Q, eN));

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
    nlp["x"] = casadi::MX::vertcat({reshape(X, nx * (N + 1), 1), reshape(U, nu * N, 1)});
    nlp["f"] = cost;
    nlp["g"] = G;
    nlp["p"] = casadi::MX::vertcat({x0, reshape(Xref, no * N, 1)});

    casadi::Dict opts;
    opts["ipopt.print_level"] = 5;
    opts["ipopt.warm_start_init_point"] = "yes";

    solver_ = nlpsol("solver", "ipopt", nlp, opts);

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

    if (first_solve_)
    {
        x_init_ = DM::zeros(state_size_ * (horizon_ + 1) + control_size_ * horizon_);
        first_solve_ = false;
    }

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
    args["x0"] = x_init_;
    args["lbx"] = lbx_;
    args["ubx"] = ubx_;
    args["lbg"] = lbg_;
    args["ubg"] = ubg_;
    args["p"]   = DM(p);

    auto res = solver_(args);
    DM sol = res.at("x");
    warmStartState(res);

    int offset = state_size_ * (horizon_ + 1);
    double vel   = static_cast<double>(sol(offset));
    double yaw_rate = static_cast<double>(sol(offset + 1));

    control_input_mutex_.lock();
    control_input_to_publish_(0) = vel;
    control_input_to_publish_(1) = yaw_rate;
    control_input_mutex_.unlock();
}

void MPCController::warmStartState(casadi::DMDict &res)
{
    casadi::DM sol = res.at("x");

    // Shift states
    int nx = state_size_;
    int nu = control_size_;

    casadi::DM Xsol = sol(casadi::Slice(0, int(nx*(horizon_+1))));
    casadi::DM Usol = sol(casadi::Slice(int(nx*(horizon_+1)), sol.size1()));

    // Build new initial guess
    casadi::DM x_next = casadi::DM::zeros(sol.size1());

    // Shift X
    for (int k = 0; k < horizon_; ++k)
    {
        x_next(casadi::Slice(k*nx, (k+1)*nx)) =
            Xsol(casadi::Slice((k+1)*nx, (k+2)*nx));
    }

    // Keep last state
    x_next(casadi::Slice(int(horizon_*nx), int((horizon_+1)*nx))) = Xsol(casadi::Slice(int(horizon_*nx), int((horizon_+1)*nx)));

    // Shift U
    int u_offset = nx*(horizon_+1);
    for (int k = 0; k < horizon_-1; ++k)
    {
        x_next(casadi::Slice(u_offset + k*nu, u_offset + (k+1)*nu)) =
            Usol(casadi::Slice((k+1)*nu, (k+2)*nu));
    }

    // Repeat last control
    x_next(casadi::Slice(int(u_offset + (horizon_-1)*nu), int(u_offset + horizon_*nu))) = Usol(casadi::Slice(int((horizon_-1)*nu), int(horizon_*nu)));

    x_init_ = x_next;
};

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
        current_robot_state_pose_mutex_.unlock();
        current_robot_state_imu_mutex_.unlock();

        size_t best_idx = current_ref_idx;
        double best_s   = -std::numeric_limits<double>::infinity();

        for (size_t i = current_ref_idx; i + 1 < reference_size && i < current_ref_idx + horizon_; ++i)
        {
            Eigen::Vector2d Pi = reference_traj[i].position;
            Eigen::Vector2d Pj = reference_traj[i + 1].position;
            Eigen::Vector2d robot_pos = Eigen::Vector2d(state(0), state(1));

            Eigen::Vector2d t = Pj - Pi;
            double len = t.norm();
            if (len < 1e-6) continue;

            Eigen::Vector2d t_hat = t / len;
            Eigen::Vector2d d = robot_pos - Pi;

            double s = d.dot(t_hat);
            Eigen::Vector2d e = d - s * t_hat;
            double e_ct = e.norm();

           bool accept_projection = false;

            // Allow "entry" to the path
            if (current_ref_idx == 0)
            {
                accept_projection = (e_ct < 0.5);
            }
            else
            {
                accept_projection = (s >= 0.0 && e_ct < 0.5);
            }

            if (accept_projection)
            {
                if (s > best_s)
                {
                    best_s   = s;
                    best_idx = i;
                }
            }
        }

        current_ref_idx = best_idx;
        std::cout << "Current reference index: " << current_ref_idx << std::endl;

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

    std::cout << "Reached the end of the reference path" << std::endl;
    control_input_mutex_.lock();
    control_input_to_publish_(0) = 0.0;
    control_input_to_publish_(1) = 0.0;
    control_input_mutex_.unlock();
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

    double dx = msg->pose.position.x - current_robot_state_.x;
    double dy = msg->pose.position.y - current_robot_state_.y;

    current_robot_state_pose_mutex_.lock();
    current_robot_state_.velocity = cos(yaw) * dx / dt + sin(yaw) * dy / dt;
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
