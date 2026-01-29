#include <rclcpp/rclcpp.hpp>
#include "clearpath_robots_sim/path_followers/MPC/mpc_controller.hpp"
#include "clearpath_robots_sim/path_followers/models/diff_drive_model.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  double dt = 0.5;
  unsigned int max_iterations = 1000;
  unsigned int horizon_ = 5;
  std::shared_ptr<clearpath_robots_sim::DiffDriveModel> dynamic_model = std::make_shared<clearpath_robots_sim::DiffDriveModel>(
    clearpath_robots_sim::DiffDriveConstraints{1.0, -1.0, true, 1.0, -1.0, true, 1.0, -1.0, true, 1.0, -1.0, true}, 0.5f);
  clearpath_robots_sim::MPCController controller(dt, max_iterations, dynamic_model, horizon_);

  rclcpp::spin(controller.get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
