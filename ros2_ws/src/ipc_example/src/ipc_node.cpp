#include "ipc_example/ipc.hpp"

using namespace ipc;

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  auto ipc_node = std::make_shared<IpcExample>();

  rclcpp::spin(ipc_node);

  rclcpp::shutdown();

  return 0;
}
