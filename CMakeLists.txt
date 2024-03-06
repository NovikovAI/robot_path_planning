cmake_minimum_required(VERSION 3.8)
project(robot_path_planning)

# set(Boost_DEBUG ON)
# set(Boost_USE_STATIC_LIBS ON)
# set(Boost_USE_STATIC ON)

# find_package(Boost)
find_package(PCL CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(a-star AStar.cpp)

add_executable(main main.cpp)

install(TARGETS main DESTINATION lib/${PROJECT_NAME})

target_link_libraries(main PRIVATE ${PCL_LIBRARIES})
target_link_libraries(main PRIVATE ${OpenCV_LIBS})
target_link_libraries(main PRIVATE a-star)