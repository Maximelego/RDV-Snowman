cmake_minimum_required(VERSION 3.10)
project(Snowman)

find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)

add_executable(Snowman src/main.cpp)
target_link_libraries(Snowman glfw GLEW::GLEW)