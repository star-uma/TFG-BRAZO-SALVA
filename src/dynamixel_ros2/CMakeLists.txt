cmake_minimum_required(VERSION 3.5)
project(dynamixel_ros2)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Install and export headers
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(DIRECTORY include/ DESTINATION include)
  ament_export_include_directories(include)
endif()

# Library implementing the wrapper
add_library(dynamixel_ros2_lib SHARED src/dynamixel_ros2.cpp)
target_include_directories(dynamixel_ros2_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(dynamixel_ros2_lib rclcpp std_msgs)

# Link Dynamixel SDK to the library (propagates to executables that link the library)
if(TARGET dynamixel_sdk::dynamixel_sdk)
  target_link_libraries(dynamixel_ros2_lib dynamixel_sdk::dynamixel_sdk)
elseif(DEFINED dynamixel_sdk_LIBRARIES)
  target_link_libraries(dynamixel_ros2_lib ${dynamixel_sdk_LIBRARIES})
endif()

# Helper macro to add optional executables if their source files exist
macro(add_optional_executable name src)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${src}")
    add_executable(${name} ${src})
    ament_target_dependencies(${name} rclcpp std_msgs)
    target_link_libraries(${name} dynamixel_ros2_lib)
    install(TARGETS ${name} RUNTIME DESTINATION lib/${PROJECT_NAME})
  endif()
endmacro()

# Optional example/test executables
add_optional_executable(testCommunication src/testCommunication.cpp)
add_optional_executable(blinkingLED src/blinkingLED.cpp)
add_optional_executable(testPositionControl src/testPositionControl.cpp)
add_optional_executable(testPWMControl src/testPWMControl.cpp)
add_optional_executable(dmxlParamsMonitor src/dmxlParamsMonitor.cpp)
add_optional_executable(pinzaTest src/realTest.cpp)

# Install library and export targets (install into prefix/lib so -ldynamixel_ros2_lib finds it)
install(TARGETS dynamixel_ros2_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

# make the library visible to dependents that use ament's plain linking
ament_export_libraries(dynamixel_ros2_lib)

# Install msg directory if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/msg")
  install(DIRECTORY msg DESTINATION share/${PROJECT_NAME})
endif()

ament_package()
