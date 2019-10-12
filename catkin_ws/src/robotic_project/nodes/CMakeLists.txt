#=============================================
#  Nodes directories 
#=============================================
# fPIC for our swig shared library
#add_definitions(-fPIC -DDLL_EXPORTS)


#add_subdirectory(cv_cameraNode)
#add_subdirectory(VirtualGPSNode)
add_subdirectory(ImageTrajectoryGeneratorNode)
add_subdirectory(NavigationControlNode)
add_subdirectory(PWMCommand_)
#add_subdirectory(LIFI_receiverNode)
#add_subdirectory(LIFI_emitterNode)
add_subdirectory(ManualOperation_)
