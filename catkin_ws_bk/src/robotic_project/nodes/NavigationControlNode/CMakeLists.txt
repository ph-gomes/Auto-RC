cmake_minimum_required(VERSION 2.8.3)



##################################################################################################
# Image Trajectory Generator Node
##################################################################################################
SET(NODE_NAME navigationControl)

SET( NODE_SRC_CODE
	src/navigationControl_node.cpp	
	src/navigationControl.cpp
)


## Declare a C++ executable
 add_executable(${NODE_NAME} ${NODE_SRC_CODE})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rosmorse_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${NODE_NAME}
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
 )

