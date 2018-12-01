cmake_minimum_required(VERSION 3.10)
project(advanced_1)

set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
set(CMAKE_CXX_STANDARD 17)

set(SOURCE_FILES main.cpp main.c)
add_executable(advanced_1 ${SOURCE_FILES})
