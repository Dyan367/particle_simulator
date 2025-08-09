cmake_minimum_required(VERSION 3.10)
project(particle_simulator)

set(CMAKE_CXX_STANDARD 17)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(particle_simulator main.cpp src/particle.cpp)
target_link_libraries(particle_simulator sfml-graphics sfml-window sfml-system)
target_include_directories(particle_simulator PRIVATE include)