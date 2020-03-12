

#
add_executable(openMVG_main_ConvertSfM_DataFormat_bal main_ConvertSfM_DataFormat_bal.cpp)
target_link_libraries(openMVG_main_ConvertSfM_DataFormat_bal
  PRIVATE
    openMVG_system
    openMVG_features
    openMVG_sfm
    ${STLPLUS_LIBRARY}
)
set_property(TARGET openMVG_main_ConvertSfM_DataFormat_bal PROPERTY FOLDER OpenMVG/rpba)
install(TARGETS openMVG_main_ConvertSfM_DataFormat_bal DESTINATION bin/)