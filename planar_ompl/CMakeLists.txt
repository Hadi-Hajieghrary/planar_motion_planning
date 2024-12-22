cmake_minimum_required(VERSION 3.0)
project(OMPL_2D_Planner)

# Find the OMPL package
find_package(ompl REQUIRED)

# Add include directories
include_directories(${OMPL_INCLUDE_DIRS})

# Add the executable
add_executable(planner main.cpp)

# Link OMPL to the executable
target_link_libraries(planner ${OMPL_LIBRARIES})
