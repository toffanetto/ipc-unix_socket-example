#include "ipc_example/ipc.hpp"

namespace ipc
{
    IpcExample::IpcExample() : Node("ipc_example")
    {

        timers_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        msg_pub_ = this->create_publisher<std_msgs::msg::String>("/icp_msg", 1);

        i = 0;

        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                             std::bind(&IpcExample::pub_timer_callback, this),
                                             timers_callback_group_);
    }

    IpcExample::~IpcExample()
    {
        if (socket_sub_thread_handler.joinable())
        {
            socket_sub_thread_handler.join(); // Ensure thread cleanup
        }
        close(socket_server_fd);
        close(socket_pub_fd);
        close(socket_sub_fd);
        unlink(SOCKET_PATH_PUB);
    }

    void IpcExample::configure_socket()
    {
        create_unix_socket_sub(socket_sub_fd, socket_sub_addr, SOCKET_PATH_SUB);

        create_unix_socket_pub(socket_server_fd, socket_pub_fd, socket_pub_addr, SOCKET_PATH_PUB);
    }

    void IpcExample::pub_timer_callback()
    {
        socket_msg_t msg = {.msg = "Hello ROS to OBU", .code = i};
        i++;
        publish_socket_pub(&msg, socket_pub_fd);
    }

    int IpcExample::create_unix_socket_pub(int &socket_server_fd, int &socket_pub_fd, sockaddr_un_t &socket_addr, char *socket_path)
    {
        // Create socket
        if ((socket_server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        // Initializing socket structure with zeros
        memset(&socket_addr, 0, sizeof(socket_addr));

        // Selection UNIX domain socket type
        socket_addr.sun_family = AF_UNIX;

        // Setting socket address path
        strncpy(socket_addr.sun_path, socket_path, strlen(socket_path));

        // Remove any socket file before start
        unlink(socket_path);

        // Bind the socket file descriptor with the socket address
        if (bind(socket_server_fd, (struct sockaddr *)&socket_addr, sizeof(socket_addr)) == -1)
        {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }

        // Listen for a connection
        if (listen(socket_server_fd, 1) == -1)
        {
            perror("listen failed");
            exit(EXIT_FAILURE);
        }

        // Waiting subscriber connection
        printf("[Publisher] Waiting for subscriber to connect...\n");
        if ((socket_pub_fd = accept(socket_server_fd, NULL, NULL)) == -1)
        {
            perror("accept failed");
            exit(EXIT_FAILURE);
        }

        return 1;
    }

    int IpcExample::create_unix_socket_sub(int &socket_fd, sockaddr_un_t &socket_addr, char *socket_path)
    {
        // Create socket
        if ((socket_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
        {
            perror("Socket creation error");
            return -1;
        }

        // Initializing socket structure with zeros
        memset(&socket_addr, 0, sizeof(socket_addr));

        // Selection UNIX domain socket type
        socket_addr.sun_family = AF_UNIX;

        // Setting socket address path
        strncpy(socket_addr.sun_path, socket_path, strlen(socket_path));

        printf("[Subscriber] Calling thread...\n");

        // Calling socket subscriber thread
        socket_sub_thread_handler = std::thread(&IpcExample::socket_sub_thread, this, socket_fd, socket_addr);

        return 1;
    }

    int IpcExample::socket_sub_thread(int socket_fd, sockaddr_un_t socket_addr)
    {
        printf("[Subscriber] Into in the thread...\n");
        
        // Connect to the socket server (publisher)
        while (connect(socket_fd, (struct sockaddr *)&socket_addr, sizeof(socket_addr)) == -1)
        {
            if (i < N_TRY_CONNECT_PUB)
            {
                printf("Trying to reconnect...\n");
                i++;
                sleep(1);
            }
            else
            {
                perror("Connection failed");
                return -1;
            }
        }

        i = 0;

        printf("[Subscriber] Publisher connected. Waiting messages...\n");

        socket_msg_t buffer;

        // Read server sockets when they arrive
        while (read(socket_fd, &buffer, sizeof(buffer)) > 0)
        {
            printf("\n[Subscriber] New message from publisher\n");
            printf("Server msg : %s\n", buffer.msg);
            printf("Server code: %d\n", buffer.code);

            std_msgs::msg::String msg;
            sprintf(buffer.msg, "%s  | Code: %d",buffer.msg, buffer.code);
            msg.data = std::string(buffer.msg);
            // RCLCPP_INFO(this->get_logger(), "Publishing IPC message | %d", buffer.code);
            // RCLCPP_INFO(this->get_logger(), buffer.msg);
            msg_pub_->publish(msg);
        }

        return 1;
    }

    void IpcExample::publish_socket_pub(socket_msg_t *msg, int socket_pub_fd)
    {
        printf("\n[Publisher] Sending message from publisher\n");
        printf("Server msg : %s\n", msg->msg);
        printf("Server code: %d\n", msg->code);
        write(socket_pub_fd, msg, sizeof(*msg));
    }

} // namespace ipc
