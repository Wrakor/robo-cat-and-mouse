cmake_minimum_required(VERSION 2.8.3)
project(simple_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
roscpp 
geometry_msgs 
turtlesim
sensor_msgs
nav_msgs
stdr_msgs
)

# This is a catkin package
catkin_package() 

add_executable(cat_exe cat.cpp) #nome do executavel - source files
add_executable(cat2_exe cat2.cpp) #nome do executavel - source files
add_executable(mouse mouse.cpp) #nome do executavel - source files
add_executable(mouse2 mouse2.cpp) #nome do executavel - source files
target_link_libraries(cat_exe ${catkin_LIBRARIES})
target_link_libraries(cat2_exe ${catkin_LIBRARIES})
target_link_libraries(mouse ${catkin_LIBRARIES})    
target_link_libraries(mouse2 ${catkin_LIBRARIES}) 



