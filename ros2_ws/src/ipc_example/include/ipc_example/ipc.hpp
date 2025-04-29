#ifndef ipc_example__ipc_HPP_
#define ipc_example__ipc_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// Socket includes
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define PORT 8080
#define SERVER_IP "143.106.207.65"

typedef struct socket_msg
{
    char msg[100];
    int code;
} socket_msg_t;

namespace ipc
{
    class IpcExample : public rclcpp::Node
    {
    public:
        IpcExample();
        ~IpcExample();

    private:
        int server_fd, rx_socket;
        struct sockaddr_in address;
        int addrlen;
        socket_msg_t buffer;

        int i;

        rclcpp::CallbackGroup::SharedPtr timers_callback_group_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;
        rclcpp::TimerBase::SharedPtr sub_timer_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        void pub_timer_callback();
        void sub_timer_callback();
    };
} // namespace ipc
#endif // ipc_example__ipc_HPP_
