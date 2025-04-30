/*

Subscriber

UNIX-Socket client

*/

#include "ipc_socket.h"

// Creating typedef for function pointer
typedef void (*subscriber_callback_t)(const socket_msg_t *msg);

// Creating struct with args for thread
typedef struct
{
    int sock_fd;
    subscriber_callback_t callback;
} subscriber_args_t;

int configure_subscriber_socket(int *socket_fd, sockaddr_un_t *socket_addr)
{
    // Create socket
    if ((*socket_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1)
    {
        perror("Socket creation error");
        return -1;
    }

    // Initializing socket structure with zeros
    memset(socket_addr, 0, sizeof(*socket_addr));

    // Selection UNIX domain socket type
    socket_addr->sun_family = AF_UNIX;

    // Setting socket address path
    strncpy(socket_addr->sun_path, SOCKET_PATH_SUB, strlen(SOCKET_PATH_SUB));

    printf("%s\n", SOCKET_PATH_SUB);

    int i = 0;

    // Connect to the socket server (publisher)
    while (connect(*socket_fd, (struct sockaddr *)socket_addr, sizeof(*socket_addr)) == -1)
    {
        if (i < 300)
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
    printf("[Subscriber] Publisher connected. Waiting messages...\n");
}

// Defining thread
void *subscriber_thread(void *arg)
{
    subscriber_args_t *args = (subscriber_args_t *)arg;
    socket_msg_t buffer;

    // Read server sockets when they arrive
    while (read(args->sock_fd, &buffer, sizeof(buffer)) > 0)
    {
        if (args->callback)
        {
            args->callback(&buffer);
        }
    }
}


// Defining callback
void subscriber_callback(const socket_msg_t *msg)
{
    printf("\nNew message from publisher\n");
    printf("Server msg : %s\n", msg->msg);
    printf("Server code: %d\n", msg->code);
}



int main()
{
    int sock_fd = 0;
    sockaddr_un_t sock_addr;

    if(configure_subscriber_socket(&sock_fd, &sock_addr) == -1){
        return -1;
    }

    // Creating thread handler
    pthread_t subscriber_thread_handler;

    // Creating thread args
    subscriber_args_t subscriber_thread_args = {.sock_fd = sock_fd, .callback = subscriber_callback};

    // Creating and starting thread with args
    if (pthread_create(&subscriber_thread_handler, NULL, subscriber_thread, &subscriber_thread_args) != 0)
    {
        perror("pthread_create");
        exit(EXIT_FAILURE);
    }

    // Keep going on main...
    for (int i = 0; i < 20; i++)
    {
        printf("[Main] doing some stuff\n");
        sleep(1);
    }

    // Need to wait the thread ends before close the socket.
    pthread_join(subscriber_thread_handler, NULL);

    close(sock_fd);
    return 0;
}