/*

Subscriber

UNIX-Socket client

*/

#include "ipc_socket.h"

int main()
{
    int sock = 0;
    struct sockaddr_un sock_addr;
    int sock_addrlen = sizeof(sock_addr);

    int i = 0;

    socket_msg_t buffer;

    // Create socket
    if ((sock = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
        perror("Socket creation error");
        return -1;
    }

    // Initializing socket structure with zeros
    memset(&sock_addr, 0, sock_addrlen);

    // Selection UNIX domain socket type
    sock_addr.sun_family = AF_UNIX;

    // Setting socket address path
    strncpy(sock_addr.sun_path, SOCKET_PATH, strlen(SOCKET_PATH));

    // Connect to the socket server (publisher)
    while (connect(sock, (struct sockaddr *)&sock_addr, sizeof(sock_addr)) == -1)
    {
        if (i < 30)
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

    // Read server sockets when they arrive
    printf("[Subscriber] Publisher connected. Waiting messages...\n");

    while (read(sock, &buffer, sizeof(buffer)) > 0)
    {
        // Receiving from server
        printf("\nNew message from publisher\n");
        printf("Server msg : %s\n", buffer.msg);
        printf("Server code: %d\n", buffer.code);
    }

    close(sock);
    return 0;
}