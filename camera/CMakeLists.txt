add_executable(test_0_camera test_0.cpp)
target_link_libraries(test_0_camera PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control realsense2)


find_package(OpenGL REQUIRED)
add_executable(test_1_camera test_1.cpp)
target_link_libraries(test_1_camera PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control realsense2 glfw ${OPENGL_LIBRARIES})