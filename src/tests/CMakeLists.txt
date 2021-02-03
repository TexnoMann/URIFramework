cmake_minimum_required(VERSION 3.1)

project(tests_program CXX)

# Test for json writer
add_executable(test_json_writer parser_writer/test_json_writer.cpp)
target_link_libraries(test_json_writer jsoncpp)

# Test for identification in static
add_executable(test_static_ident identification/static_identification.cpp)
target_link_libraries(test_static_ident spdlog ur_rtde::rtde stdc++fs)

# Test for read points in teach mode
add_executable(test_teach_pose_read teach_mode/read_trj.cpp)
target_link_libraries(test_teach_pose_read spdlog ur_rtde::rtde stdc++fs)
