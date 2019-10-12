cmake_minimum_required(VERSION 2.8.3)



##################################################################################################
#  LIFI_receiver Node
##################################################################################################
SET(NODE_NAME LIFI_receiver)

SET( NODE_SRC_CODE
	src/LIFI_receiver_node.cpp	
	src/LIFI_receiver.cpp
)


## Declare a C++ executable
 add_executable(${NODE_NAME} ${NODE_SRC_CODE})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rosmorse_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(${NODE_NAME}
   ${catkin_LIBRARIES}
 )

