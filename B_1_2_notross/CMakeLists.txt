cmake_minimum_required(VERSION 3.15)
project(B_1_2_notross)

set(CMAKE_CXX_STANDARD 14)

include_directories(include)

add_executable(B_1_2_notross
        src/main.cc
        src/master.cc include/master.h src/publisher.cc include/publisher.h src/subscriber.cc include/subscriber.h)
