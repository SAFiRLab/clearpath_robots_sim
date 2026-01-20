#pragma once


#include <Eigen/Dense>


namespace clearpath_robots_sim
{

class DynamicModel
{
public:

    DynamicModel(double dt)
    : dt_(dt)
    {}

    virtual Eigen::VectorXd fx(const Eigen::VectorXd &state, const Eigen::VectorXd &control_input) = 0;
    
protected:
    // User-defined attributes
    double dt_; // Time step

}; // class DynamicModel

}; // namespace clearpath_robots_sim
