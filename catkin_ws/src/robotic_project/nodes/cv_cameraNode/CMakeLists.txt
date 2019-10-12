
cmake_minimum_required(VERSION 2.8.3)



##################################################################################################
# cv_cameraNode
##################################################################################################
SET(NODE_NAME cv_cameraNode)


## Declare a cpp library
add_library(cv_camera src/capture.cpp src/driver.cpp)
add_library(cv_camera_nodelet src/cv_camera_nodelet.cpp)
  
SET( NODE_SRC_CODE
	src/cv_camera_node.cpp	
)


## Declare a C++ executable
 add_executable(${NODE_NAME} ${NODE_SRC_CODE})

add_dependencies(${NODE_NAME} cv_camera)
add_dependencies(cv_camera_nodelet cv_camera)



## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rosmorse_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${NODE_NAME}
#   ${catkin_LIBRARIES}
# )







## Declare a cpp executable
#add_executable(cv_camera_node src/cv_camera_node.cpp)
#add_dependencies(cv_camera_node cv_camera)
#add_dependencies(cv_camera_nodelet cv_camera)

## Specify libraries to link a library or executable target against
target_link_libraries(cv_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )

target_link_libraries(cv_camera_nodelet
  ${catkin_LIBRARIES}
  cv_camera
  )

target_link_libraries(${NODE_NAME}
  ${catkin_LIBRARIES}
  cv_camera
  )



