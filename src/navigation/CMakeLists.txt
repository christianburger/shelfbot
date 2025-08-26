add_library(four_wheel_drive_odometry SHARED
  four_wheel_drive_odometry.cpp
)

target_include_directories(four_wheel_drive_odometry PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

ament_target_dependencies(four_wheel_drive_odometry
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# --- Behavior Tree Nodes ---
add_library(shelfbot_bt_nodes SHARED
  behavior_tree_nodes/find_tag_action.cpp
  behavior_tree_nodes/spin_action.cpp
  behavior_tree_nodes/move_action.cpp
)

# Add include directories for behaviortree_cpp headers
target_include_directories(shelfbot_bt_nodes PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  ${behaviortree_cpp_INCLUDE_DIRS}
)

ament_target_dependencies(shelfbot_bt_nodes
  rclcpp
  behaviortree_cpp
  tf2_ros
  geometry_msgs
  nav2_bt_navigator
  nav2_common
  rtabmap_ros
)

# --- Mission Control Node ---
add_executable(mission_control_node path_planner.cpp)

# Add include directories for behaviortree_cpp headers
target_include_directories(mission_control_node PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  ${behaviortree_cpp_INCLUDE_DIRS}
)

ament_target_dependencies(mission_control_node
  rclcpp
  behaviortree_cpp
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  ament_index_cpp
  nav2_common
  rtabmap_ros
)

target_link_libraries(mission_control_node
  shelfbot_bt_nodes
)

# --- Installation & RPATH ---
set_target_properties(mission_control_node PROPERTIES
  INSTALL_RPATH "$ORIGIN/../"
)

install(TARGETS
  shelfbot_bt_nodes
  four_wheel_drive_odometry
  DESTINATION lib
)

install(TARGETS
  mission_control_node
  DESTINATION lib/${PROJECT_NAME}
)