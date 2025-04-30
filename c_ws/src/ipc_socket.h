#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <pthread.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define SOCKET_PATH "/tmp/unix_socket"
#define SOCKET_PATH_PUB "/tmp/socket_obu2ros"
#define SOCKET_PATH_SUB "/tmp/socket_ros2obu"

typedef struct socket_msg
{
    char msg[100];
    int code;
} socket_msg_t;

typedef struct sockaddr_un sockaddr_un_t;

int configure_subscriber_socket(int *, sockaddr_un_t *);
int configure_publisher_socket(int *, int *, sockaddr_un_t *);
void publish_socket(int , socket_msg_t *);

