/**
 * BSD 3-Clause License
 * 
 * Copyright (c) 2025, David-Alexandre Poissant, Université de Sherbrooke
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <cmath>
#include <limits>
#include <algorithm>
 
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.hpp>
 
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>
 
 
class PurePursuit : public rclcpp::Node
{
public:
 
    //! Constructor
    PurePursuit();
 
    void odomCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void pathCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void controlLoop();
 
private:
 
    geometry_msgs::msg::Transform transformToBaseLink(const geometry_msgs::msg::Pose & pose_map,
                                                                   const geometry_msgs::msg::Transform & tf_map_to_base);
 
    double normalizeAngle(double a)
    {
        while (a > M_PI)  a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }
 
    size_t findClosestWaypointIndex()
    {
        if (path_.empty())
            return 0;
 
        double min_dist = std::numeric_limits<double>::max();
        size_t best_idx = 0;
 
        for (size_t i = 0; i < path_.size(); ++i)
        {
            const auto & p = path_[i].pose.position;
 
            double dx = p.x - x_;
            double dy = p.y - y_;
            double dist = dx * dx + dy * dy;   // squared distance
 
            if (dist < min_dist)
            {
                min_dist = dist;
                best_idx = i;
            }
        }
 
        return best_idx;
    }
 
    template<typename T1, typename T2>
    double distance(T1 pt1, T2 pt2)
    {
        return sqrt(pow(pt1.x - pt2.x,2) + pow(pt1.y - pt2.y,2) + pow(pt1.z - pt2.z,2));
    }
 
    // Algorithm variables
    // Position tolerance is measured along the x-axis of the robot!
    double pos_tol_;
    double lookahead_dist_;                            // base lookahead distance [m]
    double min_lookahead_dist_, max_lookahead_dist_;    // adaptive lookahead clamp [m]
 
    // Heading-error handling (fixes the ~180 deg "drives away" failure mode)
    double heading_error_threshold_;   // |bearing| beyond which we rotate in place instead of driving [rad]
    double align_gain_;                // proportional gain used while rotating in place
 
    // Generic control variables
    double v_max_, v_, w_max_;
 
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr path_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
 
    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t current_idx_;
    bool goal_reached_;
    bool has_path_;
 
    double x_, y_, theta_;
 
    // Ros infrastructure
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    geometry_msgs::msg::TransformStamped lookahead_;
    std::string map_frame_id_, robot_frame_id_, lookahead_frame_id_;
};
 
PurePursuit::PurePursuit()
: Node("simple_trajectory_follower"),
  pos_tol_(0.25),
  lookahead_dist_(1.0), min_lookahead_dist_(0.5), max_lookahead_dist_(3.0),
  heading_error_threshold_(M_PI / 2.0), align_gain_(1.5),
  v_max_(1.0), v_(0.0), w_max_(1.0),
  current_idx_(0), goal_reached_(true), has_path_(false),
  x_(0.0), y_(0.0), theta_(0.0),
  tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_), tf_broadcaster_(this),
  map_frame_id_("world"), robot_frame_id_("base_link"), lookahead_frame_id_("lookahead")
{
    v_max_ = declare_parameter("max_linear_vel", 1.0);
    w_max_ = declare_parameter("max_angular_vel", 1.0);
    lookahead_dist_ = declare_parameter("lookahead_distance", 1.0);
    min_lookahead_dist_ = declare_parameter("min_lookahead_distance", 0.5);
    max_lookahead_dist_ = declare_parameter("max_lookahead_distance", 3.0);
    heading_error_threshold_ = declare_parameter("heading_error_threshold", M_PI / 2.0);
    align_gain_ = declare_parameter("align_gain", 1.5);
 
    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/platform_velocity_controller/cmd_vel", 10);
 
    path_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>("/arena_path", 10,
        std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));
 
    odom_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/groundTruth/poseStamped", 10,
        std::bind(&PurePursuit::odomCallback, this, std::placeholders::_1));
 
    timer_ = create_wall_timer( std::chrono::milliseconds(50), std::bind(&PurePursuit::controlLoop, this));
 
    // Populate messages with static data
    lookahead_.header.frame_id = robot_frame_id_;
    lookahead_.child_frame_id = lookahead_frame_id_;
    lookahead_.transform.rotation.w = 1.0; // valid identity quaternion before the first control cycle
}
 
void PurePursuit::odomCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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
 
void PurePursuit::pathCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
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
 
    // Start tracking from whichever waypoint is currently closest to the robot,
    // rather than always index 0. This matters if a new path is published while
    // the robot is already partway along a similar route.
    current_idx_ = findClosestWaypointIndex();
 
    goal_reached_ = false;
    has_path_ = true;
    RCLCPP_INFO(get_logger(), "Received new path with %zu waypoints.", path_.size());
}
 
geometry_msgs::msg::Transform PurePursuit::transformToBaseLink(const geometry_msgs::msg::Pose & pose_map,
                                                               const geometry_msgs::msg::Transform & tf_map_to_base)
{
    // Convert geometry msgs to tf2 types
    tf2::Transform T_map_pose;
    tf2::fromMsg(pose_map, T_map_pose);
 
    tf2::Transform T_map_base;
    tf2::fromMsg(tf_map_to_base, T_map_base);
 
    // Compute pose in base_link frame
    tf2::Transform T_base_pose = T_map_base.inverse() * T_map_pose;
 
    return tf2::toMsg(T_base_pose);
}
 
void PurePursuit::controlLoop()
{
    if (!has_path_ || path_.empty())
        return;
 
    geometry_msgs::msg::TransformStamped tf;
 
    try
    {
        tf = tf_buffer_.lookupTransform(map_frame_id_, robot_frame_id_, rclcpp::Time(0));
 
        // ===============================
        // ADAPTIVE LOOKAHEAD DISTANCE
        // Grows with current speed so faster motion looks further ahead.
        // v_ is fed back below from the last commanded speed.
        // ===============================
        double ld = std::clamp(lookahead_dist_ + 1.5 * v_, min_lookahead_dist_, max_lookahead_dist_);
 
        // ===============================
        // FIND LOOKAHEAD POINT
        // ===============================
 
        for (; current_idx_ < path_.size(); current_idx_++)
        {
            if (distance(path_[current_idx_].pose.position, tf.transform.translation) > ld)
            {
                geometry_msgs::msg::Transform pose_bl = transformToBaseLink(path_[current_idx_].pose, tf.transform);
 
                lookahead_.transform.translation.x = pose_bl.translation.x;
                lookahead_.transform.translation.y = pose_bl.translation.y;
                lookahead_.transform.translation.z = pose_bl.translation.z;
 
                lookahead_.transform.rotation = pose_bl.rotation;
 
                break;
            }
        }
 
        // ===============================
        // GOAL HANDLING
        // ===============================
 
        if (current_idx_ >= path_.size())
        {
            geometry_msgs::msg::Transform pose_bl = transformToBaseLink(path_.back().pose, tf.transform);
 
            if (std::fabs(pose_bl.translation.x) <= pos_tol_)
            {
                goal_reached_ = true;
                has_path_ = false;
                path_.clear();
            }
        }
 
        // ===============================
        // CONTROL LAW (heading-aware)
        // ===============================
 
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->get_clock()->now();
 
        if (!goal_reached_)
        {
            double xt = lookahead_.transform.translation.x;
            double yt = lookahead_.transform.translation.y;
 
            double alpha = std::atan2(yt, xt);                // bearing to lookahead point
            double L = std::max(std::hypot(xt, yt), 1e-3);    // true distance to lookahead point
 
            double v, w;
 
            if (std::fabs(alpha) > heading_error_threshold_)
            {
                // Lookahead point is behind the robot's forward half-plane --
                // this is exactly the regime where y (and hence curvature)
                // collapses toward zero even though the heading error is large.
                // Stop translating and rotate in place toward the point instead
                // of trusting curvature here.
                v = 0.0;
                w = std::clamp(align_gain_ * alpha, -w_max_, w_max_);
            }
            else
            {
                double kappa = 2.0 * std::sin(alpha) / L;
 
                // Adaptive lookahead
                v = v_max_ * std::max(0.0, std::cos(alpha)); // taper speed with bearing error
 
                // Curvature feasibility
                if (std::abs(kappa) > 1e-6)
                {
                    v = std::min(v, 0.9 * w_max_ / std::abs(kappa));
                }
 
                // Lateral error slowdown
                double lateral_error = std::abs(yt);
                v *= 1.0 / (1.0 + 2.0 * lateral_error);
 
                w = std::clamp(v_max_ * kappa, -w_max_, w_max_);
            }
 
            cmd.twist.linear.x = v;
            cmd.twist.angular.z = w;
 
            v_ = v; // feed back into the adaptive lookahead distance next cycle
        }
        else
        {
            lookahead_.transform = geometry_msgs::msg::Transform();
            lookahead_.transform.rotation.w = 1.0;
 
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = 0.0;
            v_ = 0.0;
        }
 
        // ===============================
        // PUBLISH
        // ===============================
 
        lookahead_.header.stamp = this->get_clock()->now();
        tf_broadcaster_.sendTransform(lookahead_);
 
        cmd_pub_->publish(cmd);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
}
 
 
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
