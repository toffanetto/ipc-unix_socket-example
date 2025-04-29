#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>


#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define SOCKET_PATH "/tmp/unix_socket"

typedef struct socket_msg
{
    char msg[100];
    int code;
} socket_msg_t;




