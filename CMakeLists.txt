cmake_minimum_required(VERSION 3.12)
project(Snowman)

# Check if GLFW3 is installed
find_package(glfw3 QUIET)
if(NOT GLFW3_FOUND)
    message(STATUS "[INFO] - GLFW3 not found. Installing using apt...")
    execute_process(COMMAND sudo apt update)
    execute_process(COMMAND sudo apt install -y libglfw3-dev)
    find_package(glfw3 REQUIRED)
endif()

find_package(OpenGL REQUIRED)

# Now that GLFW3 and GLEW are installed, proceed with your project setup
add_executable(Snowman src/main.cpp)

# Link against GLFW3, GLEW and OPENGL
target_link_libraries(Snowman glfw ${OPENGL_LIBRARIES})