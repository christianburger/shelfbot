cmake_minimum_required(VERSION 3.8)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(msgpack REQUIRED)

# Create the hardware library
add_library(shelfbot_hardware
)

# Specify include directories
target_include_directories(shelfbot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link dependencies
target_link_libraries(shelfbot_hardware
  CURL::libcurl
  nlohmann_json::nlohmann_json
  msgpack::msgpack
  ${rclcpp_LIBRARIES}
)

# Set compile options
target_compile_features(shelfbot_hardware PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS shelfbot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Export targets
ament_export_targets(shelfbot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  CURL
  nlohmann_json
  msgpack
)

ament_package()
