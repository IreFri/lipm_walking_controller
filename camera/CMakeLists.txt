add_executable(test_0_camera test_0.cpp)
target_link_libraries(test_0_camera PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control realsense2)


find_package(OpenGL REQUIRED)
add_executable(test_1_camera test_1.cpp)
target_link_libraries(test_1_camera PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control realsense2 glfw ${OPENGL_LIBRARIES})

add_executable(re-export_log re_export_log.cpp)
target_link_libraries(re-export_log PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control mc_mujoco::devices)

# find_package(matplotlib_cpp REQUIRED)
# find_package(libpointmatcher REQUIRED)
find_package(Open3D REQUIRED)
# find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
# add_executable(estimate_pitch_z estimate_pitch_z_error.cpp)
# target_link_libraries(estimate_pitch_z PRIVATE mc_rtc::mc_rtc_gui mc_rtc::mc_control mc_mujoco::devices ceres Python3::Python Python3::Module ${libpointmatcher_LIBRARIES} Open3D::Open3D)

# add_executable(test_2_camera test_2.cpp)
# target_link_libraries(test_2_camera PRIVATE mc_rtc::mc_control mc_mujoco::devices Open3D::Open3D realsense2 ceres ${libpointmatcher_LIBRARIES})

add_executable(ground_estimation ground_estimation.cpp)
target_link_libraries(ground_estimation PRIVATE mc_rtc::mc_control mc_mujoco::devices Open3D::Open3D realsense2 ceres)