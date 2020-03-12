
#
add_executable(openMVG_main_exportTracksTxt_Normalized main_exportTracksTxt_Normalized.cpp)
target_link_libraries(openMVG_main_exportTracksTxt_Normalized
  openMVG_system
  openMVG_features
  openMVG_sfm
  ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_exportTracksTxt_Normalized PROPERTY FOLDER OpenMVG/gesfm)
install(TARGETS openMVG_main_exportTracksTxt_Normalized DESTINATION bin/)
#
add_executable(openMVG_main_GlobalSfM_gesfm main_GlobalSfM_gesfm.cpp)
target_link_libraries(openMVG_main_GlobalSfM_gesfm
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_GlobalSfM_gesfm PROPERTY FOLDER OpenMVG/gesfm)
install(TARGETS openMVG_main_GlobalSfM_gesfm DESTINATION bin/)
#
add_executable(openMVG_main_ConvertSfM_DataFormat_gesfm main_ConvertSfM_DataFormat_gesfm.cpp)
target_link_libraries(openMVG_main_ConvertSfM_DataFormat_gesfm
  PRIVATE
    openMVG_system
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_ConvertSfM_DataFormat_gesfm PROPERTY FOLDER OpenMVG/gesfm)
install(TARGETS openMVG_main_ConvertSfM_DataFormat_gesfm DESTINATION bin/)