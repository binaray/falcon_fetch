add_library(wit_ros wit_ros.cpp)
target_link_libraries(wit_ros wit_driver ${catkin_LIBRARIES} )

add_executable(rpy_check rpy_check.cpp)
target_link_libraries(rpy_check ${catkin_LIBRARIES})
