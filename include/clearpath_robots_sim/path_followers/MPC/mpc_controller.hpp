#ifndef _CLEARPATH_ROBOTS_SIM__PATH_FOLLOWER__MPC__MPC_CONTROLLER_
#define _CLEARPATH_ROBOTS_SIM__PATH_FOLLOWER__MPC__MPC_CONTROLLER_

// External Libraries
// Eigen
#include <Eigen/Dense>

// Local
#include "clearpath_robots_sim/path_followers/path_follower_controller.hpp"


namespace clearpath_robots_sim
{

class MPCController : public PathFollowerController
{
public:
    MPCController(double dt, unsigned int max_iterations, std::shared_ptr<DynamicModel> dynamic_model, unsigned int horizon, unsigned int step_size)
    : PathFollowerController(dt, max_iterations, dynamic_model), horizon_(horizon), step_size_(step_size)
    {
        // Define cost matrices Q and R
        unsigned int state_size = dynamic_model_->getStateSize();
        unsigned int control_size = dynamic_model_->getControlSize();

        Q_ = Eigen::MatrixXd::Zero(state_size, state_size);
        Q_.diagonal() << 10.0, 10.0, 5.0, 1.0, 1.0;

        R_ = Eigen::MatrixXd::Zero(control_size, control_size);
        R_.diagonal() << 0.5, 0.2;
    }

    Eigen::MatrixXd solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference) override;


private:

    // User-defined attributes
    unsigned int horizon_;
    unsigned int step_size_;

    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

}; // class MPCController

}; // namespace clearpath_robots_sim

#endif // _CLEARPATH_ROBOTS_SIM__PATH_FOLLOWER__MPC__MPC_CONTROLLER_
