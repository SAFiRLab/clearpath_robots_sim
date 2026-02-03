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
    : DynamicModel(5, 2, 5), constraints_(constraints), track_width_(track_width)
    {}

    casadi::MX f(const casadi::MX &x, const casadi::MX &u, const casadi::MX &dt) override
    {
        casadi::MX x_next = casadi::MX::zeros(state_size_, 1);

        x_next(0) = x(0) + dt * x(3) * cos(x(2));
        x_next(1) = x(1) + dt * x(3) * sin(x(2));
        x_next(2) = x(2) + dt * x(4);
        x_next(3) = x(3) + dt * u(0);
        x_next(4) = x(4) + dt * u(1);

        return x_next;
    }

    void getStateBounds(std::vector<double> &lb, std::vector<double> &ub) const
    {
        lb.assign(state_size_, -casadi::inf);
        ub.assign(state_size_,  casadi::inf);

        if (constraints_.min_max_velocity_set)
        {
            lb[3] = constraints_.min_velocity;
            ub[3] = constraints_.max_velocity;
        }

        if (constraints_.min_max_yaw_rate_set)
        {
            lb[4] = constraints_.min_yaw_rate;
            ub[4] = constraints_.max_raw_rate;
        }
    }

    void getControlBounds(std::vector<double> &lb,  std::vector<double> &ub) const override
    {
        lb.assign(control_size_, -casadi::inf);
        ub.assign(control_size_,  casadi::inf);

        if (constraints_.min_max_acceleration_set)
        {
            lb[0] = -constraints_.max_deceleration;
            ub[0] =  constraints_.max_acceleration;
        }

        if (constraints_.min_max_yaw_acceleration_set)
        {
            lb[1] = -constraints_.max_yaw_deceleration;
            ub[1] =  constraints_.max_yaw_acceleration;
        }
    }

    DiffDriveConstraints constraints_;

private:

    // User-defined attributes
    float track_width_; // Distance between left and right wheels

}; // class DiffDriveModel

}; // namespace clearpath_robots_sim
