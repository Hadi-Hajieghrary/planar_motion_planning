cmake_minimum_required(VERSION 3.10)

project(planar_rrt VERSION 1.0
		     DESCRIPTION "The 2D Motion Planning"
		     LANGUAGES CXX)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

include_directories(include)

add_library(libutility src/utility.cpp )
target_link_libraries(libutility sfml-graphics sfml-window sfml-system)


add_library(librobot src/robot.cpp )
target_link_libraries(librobot libutility sfml-graphics sfml-window sfml-system)

add_library(libworkspace src/workspace.cpp )
target_link_libraries(libworkspace libutility librrt librobot  sfml-graphics sfml-window sfml-system)

add_library(librrt src/rrt.cpp)
target_link_libraries(librrt libutility librobot libworkspace sfml-graphics sfml-window sfml-system)


add_executable(main main.cpp)
target_link_libraries(main libutility librobot librrt sfml-graphics sfml-window sfml-system)

 

# enable_testing()
# add_subdirectory(tests)

