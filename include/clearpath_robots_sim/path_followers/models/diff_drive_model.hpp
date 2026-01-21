#pragma once

// External Libraries
// Eigen
#include <Eigen/Dense>

// Local
#include "clearpath_robots_sim/path_followers/models/dynamic_model.hpp"


namespace clearpath_robots_sim
{

struct DiffDriveConstraints
{
    double max_velocity; // m/s
    double min_velocity; // m/s
    bool min_max_velocity_set = false;
    
    double max_raw_rate; // rad/s
    double min_yaw_rate; // rad/s
    bool min_max_yaw_rate_set = false;

    double max_acceleration; // m/s^2
    double max_deceleration; // m/s^2
    bool min_max_acceleration_set = false;

    double max_yaw_acceleration; // rad/s^2
    double max_yaw_deceleration; // rad/s^2
    bool min_max_yaw_acceleration_set = false;
}; // struct DiffDriveConstraints


class DiffDriveModel : public DynamicModel
{
public:
    DiffDriveModel(const DiffDriveConstraints &constraints, const float track_width)
    : DynamicModel(5, 2), constraints_(constraints), track_width_(track_width)
    {}

    Eigen::VectorXd fx(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input, const double dt) override
    {
        // x <- [X, Y, yaw, velocity, yaw rate]
        // control_input <- [acceleration, yaw acceleration]

        Eigen::VectorXd next_state(state.size());
        next_state(0) = state(0) + dt * state(3) * std::cos(state(2));
        next_state(1) = state(1) + dt * state(3) * std::sin(state(2));
        next_state(2) = state(2) + dt * state(4);
        next_state(3) = state(3) + dt * control_input(0);
        next_state(4) = state(4) + dt * control_input(1);

        return next_state;
    }

private:

    // User-defined attributes
    float track_width_; // Distance between left and right wheels
    DiffDriveConstraints constraints_;

}; // class DiffDriveModel

}; // namespace clearpath_robots_sim
