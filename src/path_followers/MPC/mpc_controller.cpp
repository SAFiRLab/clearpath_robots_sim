#include "clearpath_robots_sim/path_followers/MPC/mpc_controller.hpp"


namespace clearpath_robots_sim
{

Eigen::MatrixXd MPCController::solve(const Eigen::VectorXd &state, const Eigen::MatrixXd &reference)
{
    const int nu = 2;

    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(horizon_, nu);

    for (unsigned int iter = 0; iter < max_iterations_; ++iter)
    {
        // Rollout states
        std::vector<Eigen::VectorXd> X(horizon_ + 1);
        X[0] = state;

        for (unsigned int k = 0; k < horizon_; ++k)
        {
            X[k + 1] = dynamic_model_->fx(X[k], U.row(k), dt_);
        }

        // Gradient descent update
        for (unsigned int k = 0; k < horizon_; ++k)
        {
            // Position error only
            Eigen::Vector2d p_err;
            //p_err(0) = X - reference(k, 0);
            //p_err(1) = X - reference(k, 1);

            Eigen::VectorXd grad_u = R_ * U.row(k).transpose();

            // Heuristic sensitivity: position depends on velocity
            grad_u(0) += dt_ * (p_err.norm());
            grad_u(1) += 0.1 * dt_ * (p_err.norm());

            U.row(k) -= step_size_ * grad_u.transpose();
        }
    }

    return U;
}

}; // namespace clearpath_robots_sim
