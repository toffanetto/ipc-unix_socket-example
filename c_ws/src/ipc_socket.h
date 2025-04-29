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







