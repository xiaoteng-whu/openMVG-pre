

add_executable(openMVG_main_GlobalSfM_POS main_GlobalSfM_POS.cpp)
target_link_libraries(openMVG_main_GlobalSfM_POS
  PRIVATE
    openMVG_system
    openMVG_image
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_GlobalSfM_POS PROPERTY FOLDER OpenMVG/POS)
install(TARGETS openMVG_main_GlobalSfM_POS DESTINATION bin/)
#
add_executable(openMVG_main_ConvertSfM_DataFormat_POS main_ConvertSfM_DataFormat_POS.cpp)
target_link_libraries(openMVG_main_ConvertSfM_DataFormat_POS
  PRIVATE
    openMVG_system
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_ConvertSfM_DataFormat_POS PROPERTY FOLDER OpenMVG/POS)
install(TARGETS openMVG_main_ConvertSfM_DataFormat_POS DESTINATION bin/)