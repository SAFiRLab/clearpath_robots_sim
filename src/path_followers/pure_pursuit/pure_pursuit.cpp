#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>


class SimpleTrajectoryFollower : public rclcpp::Node
{
public:
    SimpleTrajectoryFollower()
    : Node("simple_trajectory_follower"),
      current_idx_(0),
      has_path_(false)
    {
        k_v_ = declare_parameter("k_v", 0.8);
        k_w_ = declare_parameter("k_w", 2.0);
        goal_tol_ = declare_parameter("goal_tolerance", 0.25);
        angle_threshold_ = declare_parameter("angle_threshold", 0.25);
        max_v_ = declare_parameter("max_linear_vel", 1.0);
        max_w_ = declare_parameter("max_angular_vel", 1.0);

        cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/platform_velocity_controller/cmd_vel", 10);

        path_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
            "/husky_test_node/arena_path", 10,
            std::bind(&SimpleTrajectoryFollower::pathCallback, this, std::placeholders::_1));

        odom_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/groundTruth/poseStamped", 10,
            std::bind(&SimpleTrajectoryFollower::odomCallback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SimpleTrajectoryFollower::controlLoop, this));
    }

private:

    void pathCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        if (msg->markers.empty())
        {
            RCLCPP_WARN(get_logger(), "Received empty path.");
            return;
        }

        path_.clear();

        // There are multiple markers, but we only want to extract points form ns: "arena_path"
        for (const auto & marker : msg->markers)
        {
            if (marker.ns == "arena_path")
            {
                for (const auto & point : marker.points)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = marker.header;
                    pose.pose.position = point;
                    pose.pose.orientation.w = 1.0; // No orientation, just a point
                    path_.push_back(pose);
                }
            }
        }
        current_idx_ = 0;
        has_path_ = true;
        RCLCPP_INFO(get_logger(), "Received new path with %zu waypoints.", path_.size());
    }

    void odomCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        x_ = msg->pose.position.x;
        y_ = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        theta_ = tf2::getYaw(q);
    }

    void controlLoop()
    {
        if (!has_path_ || current_idx_ >= path_.size())
            return;

        auto & target = path_[current_idx_].pose.position;

        double dx = target.x - x_;
        double dy = target.y - y_;
        double rho = std::hypot(dx, dy);

        if (rho < goal_tol_)
        {
            current_idx_++;
            if (current_idx_ >= path_.size())
            {
                stopRobot();
                RCLCPP_INFO(get_logger(), "Trajectory completed.");
            }
            return;
        }

        double theta_d = std::atan2(dy, dx);
        double e_theta = normalizeAngle(theta_d - theta_);

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->get_clock()->now();

        if (std::fabs(e_theta) > angle_threshold_)
        {
            // Rotate in place
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = std::clamp(k_w_ * e_theta, -max_w_, max_w_);
        }
        else
        {
            // Move forward while correcting heading
            cmd.twist.linear.x = std::clamp(k_v_ * rho, 0.0, max_v_);
            cmd.twist.angular.z = std::clamp(k_w_ * e_theta, -max_w_, max_w_);
        }

        cmd_pub_->publish(cmd);
    }

    void stopRobot()
    {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->get_clock()->now();
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_idx_;
    bool has_path_;

    double x_, y_, theta_;
    double k_v_, k_w_;
    double goal_tol_;
    double angle_threshold_;
    double max_v_, max_w_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTrajectoryFollower>());
    rclcpp::shutdown();
    return 0;
}