cmake_minimum_required(VERSION 3.13)  # CMake version check

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(InverseKinematicsTest)               # Create project "simple_example"
set(CMAKE_CXX_STANDARD 20)            # Enable c++20 standard

set(SOURCE_FOLDER src/)

# Add main.cpp file of the project root directory as a source file
set(SOURCE_FILES main.cpp DisplayWindow.cpp WindowShapes.cpp Chain.cpp)

list(TRANSFORM SOURCE_FILES PREPEND ${SOURCE_FOLDER})

add_compile_options(--debug)

# Add executable target with source files listed in SOURCE_FILES variable
add_executable(InverseKinematicsTest ${SOURCE_FILES})
LINK_DIRECTORIES(/usr/local/lib/)
target_include_directories(InverseKinematicsTest PRIVATE /usr/local/include/ /usr/local/include/eigen3 include/)
target_link_directories(InverseKinematicsTest PRIVATE /usr/local/lib/)
target_link_libraries(InverseKinematicsTest -lSDL3 -leigen_blas)
