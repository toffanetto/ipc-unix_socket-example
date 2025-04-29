#include "ipc_example/ipc.hpp"

namespace ipc
{
    IpcExample::IpcExample() : Node("ipc_example")
    {

        timers_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        msg_pub_ = this->create_publisher<std_msgs::msg::String>("/icp_msg", 1);

        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                             std::bind(&IpcExample::pub_timer_callback, this),
                                             timers_callback_group_);

        sub_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                             std::bind(&IpcExample::sub_timer_callback, this),
                                             timers_callback_group_);

        addrlen = sizeof(address);

        // Create socket
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Bind the socket
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY; // Bind to any interface
        address.sin_port = htons(PORT);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
        {
            perror("bind failed");
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        // Listen for a connection
        if (listen(server_fd, 3) < 0)
        {
            perror("listen failed");
            close(server_fd);
            exit(EXIT_FAILURE);
        }

        // Accept a connection
        if ((rx_socket = accept(server_fd, (struct sockaddr *)&address,
                                (socklen_t *)&addrlen)) < 0)
        {
            perror("accept failed");
            exit(EXIT_FAILURE);
        }

        i = 0;
    }

    IpcExample::~IpcExample()
    {
    }

    void IpcExample::sub_timer_callback()
    {
        // Receiving
        read(rx_socket, &buffer, sizeof(buffer));
        printf("|-Socket received\n");
        printf("|--Client msg : %s\n", buffer.msg);
        printf("|--Client code: %d\n\n", buffer.code);

        std_msgs::msg::String msg;
        msg.data = std::string(buffer.msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing IPC message | %d", buffer.code);
        // RCLCPP_INFO(this->get_logger(), buffer.msg);
        msg_pub_->publish(msg);
    }

    void IpcExample::pub_timer_callback()
    {
        // Sending
        if (i < 15)
        {
            socket_msg_t msg = {"Hello from server", i++};
            send(rx_socket, &msg, sizeof(msg), 0);
            printf("|-Socket sent\n\n");
        }
    }

} // namespace ipc
