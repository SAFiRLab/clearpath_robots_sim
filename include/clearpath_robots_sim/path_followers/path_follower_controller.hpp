#pragma once

// External Libraries
// Eigen
#include <Eigen/Dense>

// System
#include <memory>

// Local
#include "clearpath_robots_sim/path_followers/models/dynamic_model.hpp"


namespace clearpath_robots_sim
{

class PathFollowerController
{
public:
    PathFollowerController(double dt, unsigned int max_iterations, std::shared_ptr<DynamicModel> dynamic_model)
    : dt_(dt), max_iterations_(max_iterations), dynamic_model_(dynamic_model)
    {
        if (dynamic_model_)
        {
            state_size_ = dynamic_model_->getStateSize();
            control_size_ = dynamic_model_->getControlSize();
            output_size_ = dynamic_model_->getOutputSize();
        }
    }

    virtual void solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference) = 0;


protected:

    // User-defined attributes
    double dt_;
    unsigned int max_iterations_;
    unsigned int state_size_;
    unsigned int control_size_;
    unsigned int output_size_;

    std::shared_ptr<DynamicModel> dynamic_model_;

}; // class PathFollowerController

}; // namespace clearpath_robots_sim
