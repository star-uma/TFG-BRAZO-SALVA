cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# ============================================================================
# DEPENDENCIAS ROS2
# ============================================================================

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)  # Hardware interface necesita lifecycle
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(lifecycle_msgs REQUIRED)    # Para lifecycle

find_package(dynamixel_ros2 REQUIRED)
find_package(dynamixel_sdk REQUIRED)


# ============================================================================
# LIBRERÍA SERVO API (LOCAL)
# ============================================================================

set(SERVO_API_DIR ${CMAKE_CURRENT_SOURCE_DIR}/api_folder)

# Buscar libservo_api.so en api_folder/build
find_library(SERVO_API_LIB 
  NAMES servo_api
  PATHS ${SERVO_API_DIR}/build
  NO_DEFAULT_PATH
  REQUIRED
)

message(STATUS "Found servo_api library: ${SERVO_API_LIB}")

# Headers de la API
set(SERVO_API_INCLUDE_DIR ${SERVO_API_DIR}/include)

if(NOT EXISTS ${SERVO_API_INCLUDE_DIR}/api.h)
  message(FATAL_ERROR "api.h not found in ${SERVO_API_INCLUDE_DIR}")
endif()


# ============================================================================
# COMPILAR LIBRERÍA DEL HARDWARE INTERFACE
# ============================================================================

# Incluir directorios
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/api_folder/include
)


# Crear la biblioteca para incluir el Plugin del hardware_interface
add_library(${PROJECT_NAME} SHARED src/hw_interface.cpp)

# Biblioteca del controller Plugin q hemos creado
#add_library(r6bot_controller SHARED src/robot_controller.cpp)

# Directorios de inclusión
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${SERVO_API_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# Vincular dependencias
target_link_libraries(${PROJECT_NAME}
  ${SERVO_API_LIB}
)

# Dependencias de ament
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  lifecycle_msgs
  dynamixel_ros2
  dynamixel_sdk
)

# ============================================================================
# EXPORTAR PLUGIN
# ============================================================================

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)


# ============================================================================
# INSTALACIÓN
# ============================================================================

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================================
# EXPORTAR DEPENDENCIAS
# ============================================================================

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  
  rclcpp 
  rclcpp_lifecycle 
  hardware_interface 
  pluginlib 
  lifecycle_msgs
  dynamixel_ros2
  dynamixel_sdk
)

# ============================================================================
# TESTING
# ============================================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()