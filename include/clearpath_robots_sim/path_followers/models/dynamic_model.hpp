#pragma once


#include <Eigen/Dense>


namespace clearpath_robots_sim
{

class DynamicModel
{
public:

    DynamicModel(unsigned int state_size, unsigned int control_size)
    : state_size_(state_size), control_size_(control_size)
    {}

    virtual Eigen::VectorXd fx(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input, const double dt) = 0;

    unsigned int getStateSize() const { return state_size_; };
    unsigned int getControlSize() const { return control_size_; };

protected:

    // User-defined attributes
    unsigned int state_size_;
    unsigned int control_size_;

}; // class DynamicModel

}; // namespace clearpath_robots_sim
