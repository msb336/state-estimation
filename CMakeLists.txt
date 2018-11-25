project(KFtest)
cmake_minimum_required(VERSION 3.9.0)
set (CMAKE_CXX_STANDARD 11)
include_directories(/usr/include/eigen3 include)
add_library(SystemDynamics SHARED 
    src/Robot.cpp
    src/SystemDynamics.cpp   
    src/KalmanFilter.cpp
)
add_executable(KFtest main.cpp)
target_link_libraries(KFtest SystemDynamics)
