#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <string>
namespace ipc
{
    class IpcRosSim : public rclcpp::Node
    {
    public:
        IpcRosSim() : Node("ipc_example")
        {

            pub_ = this->create_publisher<std_msgs::msg::UInt8>("/ipc_data", 1);

            using std::placeholders::_1;
            sub_ = this->create_subscription<std_msgs::msg::String>(
                "/icp_msg", 10, std::bind(&IpcRosSim::sub_callback, this, _1));

            i = 0;

            pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                                 std::bind(&IpcRosSim::pub_timer_callback, this));
        }

    private:
        unsigned int i = 0;

        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        void pub_timer_callback()
        {
            std_msgs::msg::UInt8 pub_msg;
            pub_msg.data = i;
            i++;
            pub_->publish(pub_msg);
        }

        void sub_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "%s", std::string(msg->data).c_str());
        }
    };
} // namespace ipc

using namespace ipc;

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto ipc_ros_sim = std::make_shared<IpcRosSim>();

    rclcpp::spin(ipc_ros_sim);

    rclcpp::shutdown();

    return 0;
}
