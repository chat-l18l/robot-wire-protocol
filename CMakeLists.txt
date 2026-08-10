cmake_minimum_required(VERSION 3.16)
project(robot_wire_protocol LANGUAGES C)

add_library(robot_wire_protocol STATIC src/robot_wire_protocol.c)
target_compile_features(robot_wire_protocol PUBLIC c_std_11)
set_target_properties(robot_wire_protocol PROPERTIES C_EXTENSIONS OFF POSITION_INDEPENDENT_CODE ON)
target_include_directories(robot_wire_protocol PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(robot_wire_protocol PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(test_robot_wire_protocol tests/test_robot_wire_protocol.c)
  target_link_libraries(test_robot_wire_protocol PRIVATE robot_wire_protocol)
  add_test(NAME test_robot_wire_protocol COMMAND test_robot_wire_protocol)
endif()

install(TARGETS robot_wire_protocol ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
