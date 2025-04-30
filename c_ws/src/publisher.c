/*

Publisher

UNIX-Socket server

*/

#include "ipc_socket.h"

int configure_publisher_socket(int *socket_server_fd, int *socket_fub_fd, sockaddr_un_t *socket_addr)
{
    // Create socket
    if ((*socket_server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }

    // Initializing socket structure with zeros
    memset(socket_addr, 0, sizeof(*socket_addr));

    // Selection UNIX domain socket type
    socket_addr->sun_family = AF_UNIX;

    // Setting socket address path
    strncpy(socket_addr->sun_path, SOCKET_PATH_PUB, strlen(SOCKET_PATH_PUB));

    // Remove any socket file before start
    unlink(SOCKET_PATH_PUB);

    // Bind the socket file descriptor with the socket address
    if (bind(*socket_server_fd, (struct sockaddr *)socket_addr, sizeof(*socket_addr)) == -1)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Listen for a connection
    if (listen(*socket_server_fd, 1) == -1)
    {
        perror("listen failed");
        exit(EXIT_FAILURE);
    }

    // Waiting subscriber connection
    printf("[Publisher] Waiting for subscriber to connect...\n");
    if ((*socket_fub_fd = accept(*socket_server_fd, NULL, NULL)) == -1)
    {
        perror("accept failed");
        exit(EXIT_FAILURE);
    }
    printf("[Publisher] Subscriber connected. Ready for send messages...\n");

    return 0;
}

void publish_socket(int socket_fd, socket_msg_t *msg)
{
    write(socket_fd, msg, sizeof(*msg));
}

int main()
{
    int server_socket_fd;
    int publisher_socket;
    sockaddr_un_t sock_addr;

    socket_msg_t msg = {"Hello OBU to ROS", 0};

    if (configure_publisher_socket(&server_socket_fd, &publisher_socket, &sock_addr) == -1)
    {
        return -1;
    }

    // Publishing
    for (int i = 0; i < 255; i++)
    {
        msg.code = i;

        printf("\nSending message from publisher\n");
        printf("Server msg : %s\n", msg.msg);
        printf("Server code: %d\n", msg.code);
        publish_socket(publisher_socket, &msg);

        sleep(2);
    }

    close(server_socket_fd);
    close(publisher_socket);
    unlink(SOCKET_PATH_PUB);
    return 0;
}