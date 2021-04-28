cmake_minimum_required(VERSION 3.0.2)
project(RoadWidth)

set(CMAKE_CXX_STANDARD 11)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
add_definitions(-Doffline_ros)


find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp nav_msgs tf  geometry_msgs visualization_msgs
        )
find_package(OpenCV)
FIND_PACKAGE( Eigen3 )
catkin_package( )
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS} include ${EIGEN3_INCLUDE_DIRS} common/configure common/data common receive ${catkin_INCLUDE_DIRS})

add_executable(roadWidth_node
        main.cpp
        src/road_width4.cpp
	common/configure/config.cpp )
add_dependencies(roadWidth_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(roadWidth_node ${OpenCV_LIBS} ${catkin_LIBRARIES})
