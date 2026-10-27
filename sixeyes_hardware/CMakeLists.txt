cmake_minimum_required(VERSION 3.16)
project(sixeyes_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(Threads REQUIRED)

# src/host/*.cpp and include/sixeyes/* are copies of the sixeyes-host C++ core. Do not edit them here:
# change sixeyes-host and run `python scripts/sync-host-core.py` from the repository root.
add_library(sixeyes_hardware SHARED
  src/sixeyes_system.cpp
  src/host/codec.cpp
  src/host/serial.cpp
  src/host/session.cpp
)
target_include_directories(sixeyes_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/sixeyes_hardware>
)
target_link_libraries(sixeyes_hardware PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  Threads::Threads
)

pluginlib_export_plugin_description_file(hardware_interface sixeyes_hardware.xml)

install(TARGETS sixeyes_hardware
  EXPORT export_sixeyes_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/sixeyes_hardware)

ament_export_targets(export_sixeyes_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
