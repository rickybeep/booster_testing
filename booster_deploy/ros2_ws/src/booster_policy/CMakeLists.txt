cmake_minimum_required(VERSION 3.15)
project(booster_policy LANGUAGES C CXX)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(booster_interface REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(onnxruntime CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# conda-forge installs the C++ API headers below core/session rather than
# directly in the exported include directory.
get_target_property(ONNXRUNTIME_INCLUDE_DIRS onnxruntime::onnxruntime INTERFACE_INCLUDE_DIRECTORIES)
find_path(ONNXRUNTIME_CXX_INCLUDE_DIR onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_INCLUDE_DIRS}
  PATH_SUFFIXES core/session
)
if(NOT ONNXRUNTIME_CXX_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find onnxruntime_cxx_api.h in ${ONNXRUNTIME_INCLUDE_DIRS}")
endif()
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 CONFIG REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolicyCommand.msg"
  "msg/PolicyStatus.msg"
  "srv/StartPolicy.srv"
  DEPENDENCIES builtin_interfaces
)

# ROS-independent inference core, shared by the node and the Python bindings.
add_library(policy_core STATIC src/policy.cpp)
target_include_directories(policy_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${ONNXRUNTIME_CXX_INCLUDE_DIR}
)
target_link_libraries(policy_core
  PUBLIC onnxruntime::onnxruntime
  PRIVATE nlohmann_json::nlohmann_json
)
set_target_properties(policy_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(policy_core PRIVATE -Wall -Wextra)

add_executable(policy_node src/policy_node.cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Link the C++ typesupport targets directly: ament_target_dependencies would
# also link the message packages' Python extensions into the executable.
target_link_libraries(policy_node
  policy_core
  rclcpp::rclcpp
  booster_interface::booster_interface__rosidl_typesupport_cpp
  std_srvs::std_srvs__rosidl_typesupport_cpp
  "${cpp_typesupport_target}"
)
target_compile_options(policy_node PRIVATE -Wall -Wextra)
if(APPLE)
  # RoboStack's rclcpp exports interface Python extensions, which need Python
  # symbols at load time on macOS. The node uses none of them.
  target_link_options(policy_node PRIVATE -Wl,-dead_strip_dylibs)
endif()
set_target_properties(policy_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

pybind11_add_module(policy_core_python src/bindings.cpp)
target_link_libraries(policy_core_python PRIVATE policy_core)
set_target_properties(policy_core_python PROPERTIES
  OUTPUT_NAME booster_policy_core
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

install(TARGETS policy_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS policy_core_python DESTINATION "${PYTHON_INSTALL_DIR}")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
