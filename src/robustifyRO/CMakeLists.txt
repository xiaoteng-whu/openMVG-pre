

#
add_executable(openMVG_main_ComputeMatchesBlackList main_ComputeMatchesBlackList.cpp)
target_link_libraries(openMVG_main_ComputeMatchesBlackList
  PRIVATE
    openMVG_graph
    openMVG_features
    openMVG_matching_image_collection
    openMVG_multiview
    openMVG_sfm
    openMVG_system
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_ComputeMatchesBlackList PROPERTY FOLDER OpenMVG/robustifyRO)
install(TARGETS openMVG_main_ComputeMatchesBlackList DESTINATION bin/)
#
add_executable(openMVG_main_ComputeMatchesWhiteList main_ComputeMatchesWhiteList.cpp)
target_link_libraries(openMVG_main_ComputeMatchesWhiteList
  PRIVATE
    openMVG_graph
    openMVG_features
    openMVG_matching_image_collection
    openMVG_multiview
    openMVG_sfm
    openMVG_system
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_ComputeMatchesWhiteList PROPERTY FOLDER OpenMVG/robustifyRO)
install(TARGETS openMVG_main_ComputeMatchesWhiteList DESTINATION bin/)