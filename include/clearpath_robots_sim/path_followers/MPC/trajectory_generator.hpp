#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <memory>

#include "clearpath_robots_sim/path_followers/models/diff_drive_model.hpp"


namespace clearpath_robots_sim
{

struct TrajectoryPoint
{
    Eigen::Vector2d position; // X, Y
    double yaw;               // rad
    double velocity;          // m/s
    double yaw_rate;          // rad/s
    double time;              // s
};

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(const std::shared_ptr<DiffDriveModel> &model, double dt)
        : model_(model), dt_(dt)
    {}

    std::vector<TrajectoryPoint> generateTrajectory(const std::vector<Eigen::Vector2d> &path)
    {
        std::vector<TrajectoryPoint> trajectory;
        if (path.size() < 2)
            return trajectory;

        const DiffDriveConstraints &c = model_->constraints_;

        double prev_yaw = std::atan2(path[1].y() - path[0].y(), path[1].x() - path[0].x());
        double prev_vel = 0.0;
        double prev_time = 0.0;

        trajectory.push_back({path[0], prev_yaw, prev_vel, 0.0, prev_time});

        for (size_t i = 1; i < path.size(); ++i)
        {
            Eigen::Vector2d delta = path[i] - path[i-1];
            double dist = delta.norm();
            if (dist < 1e-6) continue;

            double target_yaw = std::atan2(delta.y(), delta.x());
            double yaw_diff = normalizeAngle(target_yaw - prev_yaw);

            // Compute max allowed velocities based on yaw rate limit
            double max_vel_due_to_yaw = std::abs(c.max_raw_rate / (yaw_diff / dt_));

            // Trapezoidal velocity profile respecting acceleration limits
            double max_vel = std::min(c.max_velocity, max_vel_due_to_yaw);
            double accel = (max_vel - prev_vel) > 0 ? std::min(c.max_acceleration, max_vel - prev_vel) : std::min(c.max_deceleration, prev_vel - max_vel);

            // Compute the time needed for this segment
            double segment_time = (2 * dist) / (prev_vel + max_vel); // trapezoidal approx
            segment_time = std::max(segment_time, dt_);

            // Number of integration steps for this segment
            int steps = std::ceil(segment_time / dt_);
            for (int s = 1; s <= steps; ++s)
            {
                double alpha = static_cast<double>(s) / steps;

                // Linear interpolation for position
                Eigen::Vector2d pos = path[i-1] + alpha * delta;

                // Interpolate velocity with simple acceleration limit
                double vel = prev_vel + alpha * accel * dt_;
                vel = std::clamp(vel, c.min_velocity, c.max_velocity);

                // Interpolate yaw and compute yaw rate
                double yaw = normalizeAngle(prev_yaw + alpha * yaw_diff);
                double yaw_rate = yaw_diff / segment_time;
                yaw_rate = std::clamp(yaw_rate, c.min_yaw_rate, c.max_raw_rate);

                double time = prev_time + alpha * segment_time;

                trajectory.push_back({pos, yaw, vel, yaw_rate, time});
            }

            prev_yaw = target_yaw;
            prev_vel = max_vel;
            prev_time += segment_time;
        }

        return trajectory;
    }

    // Normalize angle to [-pi, pi]
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    }

private:
    std::shared_ptr<DiffDriveModel> model_;
    double dt_;

};

} // namespace clearpath_robots_sim
