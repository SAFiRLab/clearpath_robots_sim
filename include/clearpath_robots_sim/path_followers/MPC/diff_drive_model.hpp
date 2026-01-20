#pragma once

// External Libraries
// Eigen
#include <Eigen/Dense>

// Local
#include "clearpath_robots_sim/path_followers/MPC/dynamic_model.hpp"


namespace clearpath_robots_sim
{

class DiffDriveModel : public DynamicModel
{
public:
    DiffDriveModel(double dt)
    : DynamicModel(dt)
    {}

    Eigen::VectorXd fx(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input) override
    {
        // x <- [X, Y, yaw, velocity, yaw rate]
        // control_input <- [acceleration, yaw acceleration]
        Eigen::VectorXd next_state(state.size());
        next_state(0) = state(0) + dt_ * state(3) * std::cos(state(2));
        next_state(1) = state(1) + dt_ * state(3) * std::sin(state(2));
        next_state(2) = state(2) + dt_ * state(4);
        next_state(3) = state(3) + dt_ * control_input(0);
        next_state(4) = state(4) + dt_ * control_input(1);
        return next_state;
    }

private:

    // User-defined attributes
    float track_width_; // Distance between left and right wheels

}; // class DiffDriveModel

}; // namespace clearpath_robots_sim
