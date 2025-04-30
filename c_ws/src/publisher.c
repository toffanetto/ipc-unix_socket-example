/*

Publisher

UNIX-Socket server

*/

#include "ipc_socket.h"

int main()
{
    int server_fd;
    int rx_socket;
    struct sockaddr_un sock_addr;
    int sock_addrlen = sizeof(sock_addr);

    socket_msg_t msg = {"Hello OBU to ROS", 0};

    // Create socket
    if ((server_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }


    // Initializing socket structure with zeros
    memset(&sock_addr, 0, sock_addrlen);

    // Selection UNIX domain socket type
    sock_addr.sun_family = AF_UNIX;

    // Setting socket address path
    strncpy(sock_addr.sun_path, SOCKET_PATH_PUB, strlen(SOCKET_PATH_PUB));

    // Remove any socket file before start
    unlink(SOCKET_PATH_PUB);

    // Bind the socket file descriptor with the socket address
    if (bind(server_fd, (struct sockaddr *)&sock_addr, sock_addrlen) == -1)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }

    // Listen for a connection
    if (listen(server_fd, 1) == -1)
    {
        perror("listen failed");
        exit(EXIT_FAILURE);
    }

    // Waiting subscriber connection
    printf("[Publisher] Waiting for subscriber to connect...\n");
    if ((rx_socket = accept(server_fd, NULL, NULL)) == -1)
    {
        perror("accept failed");
        exit(EXIT_FAILURE);
    }

    // Publishing
    printf("[Publisher] Subscriber connected. Sending messages...\n");
    for (int i = 0; i < 255; i++)
    {
        msg.code = i;
        printf("\nSending message from publisher\n");
        printf("Server msg : %s\n", msg.msg);
        printf("Server code: %d\n", msg.code);
        write(rx_socket, &msg, sizeof(msg));
        sleep(2);
    }

    sleep(30);

    close(rx_socket);
    close(server_fd);
    unlink(SOCKET_PATH_PUB);
    return 0;
}