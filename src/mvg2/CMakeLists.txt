

#
add_executable(openMVG_main_SfMInit_ImageListing_Unknown main_SfMInit_ImageListing_Unknown.cpp)
target_link_libraries(openMVG_main_SfMInit_ImageListing_Unknown
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    openMVG_exif
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_SfMInit_ImageListing_Unknown PROPERTY FOLDER OpenMVG/mvg2)
install(TARGETS openMVG_main_SfMInit_ImageListing_Unknown DESTINATION bin/)
#
add_executable(openMVG_main_GlobalSfM_IO main_GlobalSfM_IO.cpp)
target_link_libraries(openMVG_main_GlobalSfM_IO
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
	openMVG_graph
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_GlobalSfM_IO PROPERTY FOLDER OpenMVG/mvg2)
install(TARGETS openMVG_main_GlobalSfM_IO DESTINATION bin/)
#
add_executable(openMVG_main_MatchesToRelativeInfo_ReS main_MatchesToRelativeInfo_ReS.cpp)
target_link_libraries(openMVG_main_MatchesToRelativeInfo_ReS
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_MatchesToRelativeInfo_ReS PROPERTY FOLDER OpenMVG/mvg2)
install(TARGETS openMVG_main_MatchesToRelativeInfo_ReS DESTINATION bin/)
#
add_executable(openMVG_main_MatchesToRelativeInfo_V3D main_MatchesToRelativeInfo_V3D.cpp)
target_link_libraries(openMVG_main_MatchesToRelativeInfo_V3D
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_MatchesToRelativeInfo_V3D PROPERTY FOLDER OpenMVG/mvg2)
install(TARGETS openMVG_main_MatchesToRelativeInfo_V3D DESTINATION bin/)
#
add_executable(openMVG_main_MatchesToRelativeInfo_essential main_MatchesToRelativeInfo_essential.cpp)
target_link_libraries(openMVG_main_MatchesToRelativeInfo_essential
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_MatchesToRelativeInfo_essential PROPERTY FOLDER OpenMVG/mvg2)
install(TARGETS openMVG_main_MatchesToRelativeInfo_essential DESTINATION bin/)
#
add_executable(openMVG_main_exportTracksTxt main_exportTracksTxt.cpp)
target_link_libraries(openMVG_main_exportTracksTxt
  openMVG_system
  openMVG_features
  openMVG_sfm
  ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_exportTracksTxt PROPERTY FOLDER OpenMVG/mvg2)
install(TARGETS openMVG_main_exportTracksTxt DESTINATION bin/)