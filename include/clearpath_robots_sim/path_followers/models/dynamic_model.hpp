#pragma once


#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <vector>


namespace clearpath_robots_sim
{

class DynamicModel
{
public:

    DynamicModel(int state_size, int control_size, int output_size)
    : state_size_(state_size), control_size_(control_size), output_size_(output_size)
    {}

    virtual casadi::MX f(const casadi::MX &x, const casadi::MX &u, const casadi::MX &dt) = 0;

    int getStateSize() const { return state_size_; };
    int getControlSize() const { return control_size_; };
    int getOutputSize() const { return output_size_; };
    virtual void getStateBounds(std::vector<double> &lb, std::vector<double> &ub) const = 0;
    virtual void getControlBounds(std::vector<double> &lb, std::vector<double> &ub) const = 0;

protected:

    // User-defined attributes
    int state_size_;
    int control_size_;
    int output_size_;

}; // class DynamicModel

}; // namespace clearpath_robots_sim
