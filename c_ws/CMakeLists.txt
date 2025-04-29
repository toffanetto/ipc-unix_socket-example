cmake_minimum_required(VERSION 3.0)
project(ipc_socket_example)

find_package(Threads REQUIRED)

add_executable(publisher src/publisher.c)
target_link_libraries(publisher PRIVATE Threads::Threads -lrt)
target_compile_options(publisher PRIVATE -g)

add_executable(subscriber src/subscriber.c)
target_link_libraries(subscriber PRIVATE Threads::Threads -lrt)
target_compile_options(subscriber PRIVATE -g)
