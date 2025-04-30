#ifndef ipc_example__ipc_HPP_
#define ipc_example__ipc_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"


#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <iostream>

// Socket includes
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <thread>

#define SOCKET_PATH_SUB "/tmp/socket_obu2ros"
#define SOCKET_PATH_PUB "/tmp/socket_ros2obu"

#define N_TRY_CONNECT_PUB 300

typedef struct socket_msg
{
    char msg[100];
    int code;
} socket_msg_t;

typedef struct sockaddr_un sockaddr_un_t;

// Creating struct with args for thread
typedef struct{
    int sock_fd;
} subscriber_args_t;

namespace ipc
{
    class IpcExample : public rclcpp::Node
    {
    public:
        IpcExample();
        ~IpcExample();

        void configure_socket();

    private:
        int socket_server_fd;
        int socket_pub_fd;
        int socket_sub_fd;

        sockaddr_un_t socket_pub_addr;
        sockaddr_un_t socket_sub_addr;

        std::thread socket_sub_thread_handler;
        subscriber_args_t socket_sub_thread_args;

        int i;

        rclcpp::CallbackGroup::SharedPtr timers_callback_group_;

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_pub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;

        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_;

        void pub_timer_callback();
        void sub_callback(const std_msgs::msg::UInt8::SharedPtr msg);

        int create_unix_socket_sub(int &socket_fd, sockaddr_un_t &socket_addr, char *socket_path);
        int create_unix_socket_pub(int &socket_server_fd, int &socket_pub_fd, sockaddr_un_t &socket_addr, char *socket_path);
        
        void socket_sub_thread(int socket_fd, sockaddr_un_t socket_addr);

        void publish_socket_pub(socket_msg_t *msg, int socket_pub_fd);

    };
} // namespace ipc
#endif // ipc_example__ipc_HPP_
