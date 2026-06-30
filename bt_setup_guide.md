# Shelfbot Exploration — Build, Setup & Usage Guide

This guide covers everything needed to compile, wire up, launch, and operate
the modular exploration mission: `frontier_discovery`, `frontier_queue`,
`tag_registry`, and the `exploration_bt` mission node.

**This mission is no longer a separate launch file.** It now ships as
**Tier 8** of `launch/real_robot_nav2.launch.py`, gated behind a single
`run_exploration_mission` argument, so one command brings up the whole robot
— hardware, SLAM, Nav2, *and* the autonomous exploration mission.

---

## 1. Architecture Recap

Four independent nodes, each replaceable/restartable on its own:

```
/map ──► frontier_discovery ──► /frontiers ──► frontier_queue ──► (services)
                                                                        │
/tag_detections ──► tag_registry ──► (service: check_found)            │
                                                                        ▼
                                                              exploration_bt
                                                          (BehaviorTree.CPP v3)
                                                                        │
                                                                        ▼
                                                          /navigate_to_pose (Nav2)
                                                          /cmd_vel (spin scan)
```

No node holds a reference to another node's internals — they only talk
through topics and services. You can kill and restart `tag_registry` without
affecting `frontier_queue`, and vice versa. Because of this, the whole tier
is also wired with a single `run_exploration_mission` launch argument so it
can be switched off without touching anything else in
`real_robot_nav2.launch.py` (see §7).

---

## 2. Directory Layout

This mission lives **inside** the existing `shelfbot` package, using its
existing flat layout — there is no separate `shelfbot_exploration` package
and no `include/shelfbot/exploration/` or `src/exploration/` subfolder.
Match the structure your other nodes already use:

```
shelfbot/
├── CMakeLists.txt
├── package.xml
├── srv/
│   ├── GetFrontiers.srv
│   ├── GetNextFrontier.srv
│   ├── UpdateFrontierStatus.srv
│   ├── GetQueueSummary.srv
│   └── CheckTagsFound.srv
├── include/shelfbot/
│   ├── types.hpp
│   ├── bt_context.hpp
│   └── bt_behaviors.hpp
├── src/nodes/
│   ├── frontier_discovery_node.cpp
│   ├── frontier_queue_node.cpp
│   ├── tag_registry_node.cpp
│   ├── bt_behaviors.cpp
│   ├── exploration_bt_node.cpp        ← created in §3 below
│   └── CMakeLists.txt
├── config/
│   └── exploration_tree.xml           ← created in §4 below
└── launch/
    └── real_robot_nav2.launch.py      ← Tier 8 added in §7 below
```

> **Path fix:** earlier drafts of `bt_behaviors.hpp`/`bt_behaviors.cpp`
> included `"shelfbot/exploration/bt_context.hpp"` /
> `"shelfbot/exploration/bt_behaviors.hpp"`, which doesn't exist anywhere in
> this package — `bt_context.hpp` and `bt_behaviors.hpp` live directly under
> `include/shelfbot/`, alongside every other header in the project. Both
> includes must read `"shelfbot/bt_context.hpp"` and
> `"shelfbot/bt_behaviors.hpp"` respectively or the package will not compile.
> The `shelfbot::exploration` C++ namespace is unaffected — only the file
> paths change.

---

## 3. Missing Piece: `exploration_bt_node.cpp`

This is the executable that owns the `BtContext`, builds the
`BehaviorTreeFactory`, loads the XML tree, and ticks it. Create this file:

```cpp
// src/nodes/exploration_bt_node.cpp
// ─────────────────────────────────────────────────────────────────────────────
// exploration_bt_node.cpp
//
// Owns the BtContext, builds the BT tree from config/exploration_tree.xml,
// and ticks it at a fixed rate on the main thread while the shared
// rclcpp::Node spins on a background thread.
// ─────────────────────────────────────────────────────────────────────────────
#include <thread>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>   // optional Groot2 live view

#include "shelfbot/bt_context.hpp"
#include "shelfbot/bt_behaviors.hpp"

using namespace shelfbot::exploration;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("exploration_bt_node");

    // ── declare mission parameters ──────────────────────────────────────────
    node->declare_parameter("target_tag_ids", std::vector<int64_t>{});
    node->declare_parameter("spin_angular_vel", 0.6);     // rad/s
    node->declare_parameter("spin_duration_s", 10.5);     // ≈ 2π / 0.6
    node->declare_parameter("tick_period_ms", 200);
    node->declare_parameter("bt_xml_path", std::string(""));  // empty = use default
    node->declare_parameter("enable_groot_monitoring", false);

    // ── build BtContext ──────────────────────────────────────────────────────
    auto ctx = std::make_shared<BtContext>();
    ctx->node = node;

    auto tag_ids_param = node->get_parameter("target_tag_ids").as_integer_array();
    ctx->target_tag_ids.assign(tag_ids_param.begin(), tag_ids_param.end());
    ctx->spin_angular_vel = node->get_parameter("spin_angular_vel").as_double();
    ctx->spin_duration_s  = node->get_parameter("spin_duration_s").as_double();

    ctx->get_next      = node->create_client<shelfbot::srv::GetNextFrontier>(
        "/frontier_queue/get_next");
    ctx->update_status = node->create_client<shelfbot::srv::UpdateFrontierStatus>(
        "/frontier_queue/update_status");
    ctx->get_summary   = node->create_client<shelfbot::srv::GetQueueSummary>(
        "/frontier_queue/get_summary");
    ctx->check_tags    = node->create_client<shelfbot::srv::CheckTagsFound>(
        "/tag_registry/check_found");

    ctx->navigate = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        node, "navigate_to_pose");

    ctx->cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    RCLCPP_INFO(node->get_logger(), "Waiting for frontier_queue / tag_registry services...");
    if (!ctx->wait_for_services(std::chrono::seconds(30))) {
        RCLCPP_FATAL(node->get_logger(), "Required services never appeared. Exiting.");
        rclcpp::shutdown();
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "All services available.");

    if (!ctx->navigate->wait_for_action_server(std::chrono::seconds(30))) {
        RCLCPP_FATAL(node->get_logger(), "navigate_to_pose action server never appeared.");
        rclcpp::shutdown();
        return 1;
    }

    // ── spin the shared node on a background thread ────────────────────────
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    // ── build the tree ───────────────────────────────────────────────────────
    BT::BehaviorTreeFactory factory;
    register_exploration_nodes(factory);

    auto blackboard = BT::Blackboard::create();
    blackboard->set<BtContextPtr>("ctx", ctx);

    std::string xml_path = node->get_parameter("bt_xml_path").as_string();
    if (xml_path.empty()) {
        xml_path = ament_index_cpp::get_package_share_directory("shelfbot")
                 + "/config/exploration_tree.xml";
    }
    RCLCPP_INFO(node->get_logger(), "Loading BT XML: %s", xml_path.c_str());

    BT::Tree tree = factory.createTreeFromFile(xml_path, blackboard);

    // Optional: live Groot2 monitoring (publishes on default ZMQ ports)
    std::unique_ptr<BT::PublisherZMQ> zmq_publisher;
    if (node->get_parameter("enable_groot_monitoring").as_bool()) {
        zmq_publisher = std::make_unique<BT::PublisherZMQ>(tree);
        RCLCPP_INFO(node->get_logger(), "Groot2 monitoring enabled on default ports.");
    }

    // ── tick loop on the MAIN thread ────────────────────────────────────────
    const int period_ms = node->get_parameter("tick_period_ms").as_int();
    rclcpp::WallRate rate(std::chrono::milliseconds(period_ms));

    RCLCPP_INFO(node->get_logger(), "Exploration BT started. Ticking every %d ms.", period_ms);

    while (rclcpp::ok()) {
        BT::NodeStatus status = tree.tickRoot();

        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(node->get_logger(),
                "Exploration mission complete (SUCCESS). Halting tree.");
            tree.haltTree();
            break;
        }
        if (status == BT::NodeStatus::FAILURE) {
            RCLCPP_WARN(node->get_logger(),
                "Exploration tree returned FAILURE at root — this should not "
                "happen with the documented tree structure (root Selector "
                "always has a fallback). Check XML wiring.");
            // Keep ticking rather than exiting — a transient service hiccup
            // should not kill the mission node.
        }

        rate.sleep();
    }

    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
```

---

## 4. Missing Piece: `exploration_tree.xml`

This XML drives the tree described earlier and lives at
`config/exploration_tree.xml`, alongside every other runtime config the
package already ships (`nav2_params.yaml`, `slam_toolbox_params.yaml`,
etc.) — not in a separate `bt_xml/` folder. BehaviorTree.CPP v3 XML syntax:

```xml
<?xml version="1.0"?>
<!--
  exploration_tree.xml
  ──────────────────────────────────────────────────────────────────────────
  Root: Selector (no memory) — re-evaluated every tick so termination
  conditions are always checked first, even mid-navigation.

  1. AllTargetTagsFound      → SUCCESS as soon as every target tag is in the registry
  2. Inverter(HasPendingFrontiers) → SUCCESS when the queue has nothing left
  3. FrontierCycle (Sequence with memory) → does the actual work

  Inside FrontierCycle:
    GetNextFrontier  pops one PENDING frontier, writes frontier_id/target_x/target_y
    NavWithRecovery  Selector: try Navigate+Scan, else MarkFrontierBlocked
    UpdateFrontierStatus  always runs last — commits SUCCEEDED or BLOCKED
  ──────────────────────────────────────────────────────────────────────────
-->
<root BTCPP_format="3" main_tree_to_execute="MissionRoot">
  <BehaviorTree ID="MissionRoot">
    <Selector name="MissionRoot">

      <!-- Termination gate #1: mission complete -->
      <AllTargetTagsFound name="check_tags_found"/>

      <!-- Termination gate #2: nothing left to explore -->
      <Inverter name="invert_pending_check">
        <HasPendingFrontiers name="check_pending"/>
      </Inverter>

      <!-- Main work cycle: resumes mid-sequence across ticks (memory node) -->
      <Sequence name="FrontierCycle">

        <GetNextFrontier name="pop_frontier"
                         frontier_id="{frontier_id}"
                         target_x="{target_x}"
                         target_y="{target_y}"/>

        <Selector name="NavWithRecovery">

          <Sequence name="NavigateAndScan">
            <NavigateToGoal name="drive_to_frontier"
                            target_x="{target_x}"
                            target_y="{target_y}"
                            nav_succeeded="{nav_succeeded}"/>
            <SpinAndScan name="scan_360"/>
          </Sequence>

          <MarkFrontierBlocked name="mark_blocked"
                               frontier_id="{frontier_id}"
                               nav_succeeded="{nav_succeeded}"/>

        </Selector>

        <UpdateFrontierStatus name="commit_status"
                              frontier_id="{frontier_id}"
                              nav_succeeded="{nav_succeeded}"/>

      </Sequence>

    </Selector>
  </BehaviorTree>
</root>
```

**Why this composition works:**

- `MissionRoot` is a plain `Selector` (no memory) — every tick it re-checks
  "are we done?" before continuing the frontier cycle. This means if all
  target tags get found *during* a `NavigateAndScan` run, the very next tick
  the Selector short-circuits to `AllTargetTagsFound` = SUCCESS, but note
  `NavigateAndScan` itself is still `RUNNING` from the prior tick — the
  Selector calls `halt()` on it before moving to the higher-priority child,
  which triggers `NavigateToGoalAction::onHalted()` and `SpinAndScanAction::onHalted()`,
  cleanly cancelling the Nav2 goal and stopping cmd_vel.
- `FrontierCycle` is a `Sequence` *with* memory (the default in BT.CPP v3) —
  once `GetNextFrontier` succeeds, the Sequence remembers it doesn't need to
  re-tick that child; it resumes at whichever child is `RUNNING`.
- `NavWithRecovery`'s `Selector` means: try `NavigateAndScan`; if that whole
  branch fails (Nav2 aborts/rejects), instead run `MarkFrontierBlocked`,
  which always returns `SUCCESS` — this is the BT-native equivalent of a
  try/except that guarantees `UpdateFrontierStatus` always runs.

---

## 5. `CMakeLists.txt` Additions

Add these blocks to `src/nodes/CMakeLists.txt` — the same file that already
builds `camera_publisher`, `apriltag_detector_node`, `shelfbot_slam_orb3_node`,
and `lidar_relay_node`. Mission nodes are first-class citizens of that
directory, not a separate `src/exploration/` subdirectory:

```cmake
# ── Exploration / BT dependencies ──────────────────────────────────────────────
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Exploration service interfaces ─────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetFrontiers.srv"
  "srv/GetNextFrontier.srv"
  "srv/UpdateFrontierStatus.srv"
  "srv/GetQueueSummary.srv"
  "srv/CheckTagsFound.srv"
  DEPENDENCIES geometry_msgs
)
# This generates shelfbot/srv/get_frontiers.hpp etc., consumed by the
# exploration .cpp/.hpp files via #include <shelfbot/srv/get_frontiers.hpp>
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ── frontier_discovery ────────────────────────────────────────────────────────
add_executable(frontier_discovery_node
  frontier_discovery_node.cpp
)
target_link_libraries(frontier_discovery_node "${cpp_typesupport_target}")
ament_target_dependencies(frontier_discovery_node
  rclcpp nav_msgs geometry_msgs visualization_msgs
)

# ── frontier_queue ────────────────────────────────────────────────────────────
add_executable(frontier_queue_node
  frontier_queue_node.cpp
)
target_link_libraries(frontier_queue_node "${cpp_typesupport_target}")
ament_target_dependencies(frontier_queue_node
  rclcpp geometry_msgs visualization_msgs
)

# ── tag_registry ──────────────────────────────────────────────────────────────
add_executable(tag_registry_node
  tag_registry_node.cpp
)
target_link_libraries(tag_registry_node "${cpp_typesupport_target}")
ament_target_dependencies(tag_registry_node
  rclcpp tf2_ros tf2_geometry_msgs visualization_msgs geometry_msgs
)

# ── exploration_bt (BT behaviors lib + main node) ─────────────────────────────
add_library(exploration_bt_behaviors SHARED
  bt_behaviors.cpp
)
target_link_libraries(exploration_bt_behaviors "${cpp_typesupport_target}")
ament_target_dependencies(exploration_bt_behaviors
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_msgs geometry_msgs
)
target_include_directories(exploration_bt_behaviors PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)

add_executable(exploration_bt_node
  exploration_bt_node.cpp
)
target_link_libraries(exploration_bt_node
  exploration_bt_behaviors
  "${cpp_typesupport_target}"
)
ament_target_dependencies(exploration_bt_node
  rclcpp rclcpp_action behaviortree_cpp_v3 nav2_msgs
  geometry_msgs ament_index_cpp
)

# ── install all executables ───────────────────────────────────────────────────
install(TARGETS
  camera_publisher
  apriltag_detector_node
  shelfbot_slam_orb3_node
  lidar_relay_node
  frontier_discovery_node
  frontier_queue_node
  tag_registry_node
  exploration_bt_node
  exploration_bt_behaviors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
```

`config/exploration_tree.xml` needs no separate `install(DIRECTORY ...)`
line — the top-level `CMakeLists.txt` already does
`install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`, which picks
it up automatically along with every other YAML/XML file already in that
folder.

> **Note:** the `frontier_discovery_node` / `frontier_queue_node` /
> `tag_registry_node` find_package calls intentionally do **not** include
> `pluginlib` or `hardware_interface` — those belong to the
> `four_wheel_drive_hardware_interface` block elsewhere in the top-level
> `CMakeLists.txt` and have nothing to do with the mission tier.

---

## 6. `package.xml` Additions

The real `package.xml` already declares `geometry_msgs`, `visualization_msgs`,
`tf2_ros`, `tf2_geometry_msgs`, `apriltag_msgs`, and `nav2_bringup` (plus the
rest of the Nav2 package set) for the existing hardware/perception/Nav2
nodes — those did **not** need to be added again. `nav2_msgs` was missing
from the otherwise-exhaustive Nav2 dependency list and has been added there.

What was actually missing, and has now been added in a new "Exploration
mission" block inside the existing `<package>` element:

```xml
<!-- ── Exploration mission (BehaviorTree.CPP custom node layer) ───────── -->
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>

<depend>behaviortree_cpp_v3</depend>
<depend>rclcpp_action</depend>
<depend>ament_index_cpp</depend>
```

The `rosidl_interface_packages` group membership and the
`rosidl_default_generators` / `rosidl_default_runtime` depends are required
specifically because this package both generates its own `.srv` interfaces
*and* uses them in the same package — a common ROS2 pattern, but easy to
miss if you've only ever consumed someone else's interfaces before.

The five `.srv` files referenced throughout this guide
(`srv/GetFrontiers.srv`, `srv/GetNextFrontier.srv`,
`srv/UpdateFrontierStatus.srv`, `srv/GetQueueSummary.srv`,
`srv/CheckTagsFound.srv`) already exist in the repo and their fields line up
exactly with what `bt_context.hpp`, `bt_behaviors.cpp`, and the three
service-server nodes expect (`frontier_id`/`x`/`y`/`success`/`reason` on
`GetNextFrontier`, `pending`/`active`/`succeeded`/`blocked`/`total`/
`has_pending` on `GetQueueSummary`, etc.) — no changes needed there, only
the `rosidl_generate_interfaces()` call in §5 to actually build them.

---

## 7. Mission Tasks — Tier 8 of `real_robot_nav2.launch.py`

The exploration mission is **no longer a standalone `exploration_launch.py`**.
It's folded directly into `launch/real_robot_nav2.launch.py` as **Tier 8**,
firing 4 s after RViz (t=26 s) — by which point `bt_navigator`'s lifecycle
has had time to activate and `/navigate_to_pose` is already serving goals.
Everything from Tier 1 through Tier 7 (hardware, camera, `lidar_relay_node`
with `reverse_scan_order: True`, SLAM, Nav2, RViz) is untouched.

Four new launch arguments control the mission tasks:

| Argument | Default | Effect |
|---|---|---|
| `run_exploration_mission` | `true` | Set to `false` to bring up only Tiers 1–7 (base navigation), exactly as before this mission tier existed. |
| `target_tag_ids` | `[1, 2, 3]` | AprilTag IDs that, once all found, end the mission successfully. |
| `bt_xml_path` | `config/exploration_tree.xml` | Override to point at a different mission tree without rebuilding. |
| `enable_groot_monitoring` | `false` | Publish BT state over ZMQ for live Groot2 inspection (§11). |

```python
# Excerpt — Tier 8 of real_robot_nav2.launch.py
frontier_discovery = Node(
    package='shelfbot',
    executable='frontier_discovery_node',
    name='frontier_discovery',
    output='screen',
    parameters=[{
        'min_frontier_size': 5,
        'publish_hz': 2.0,
    }],
    condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
)

frontier_queue = Node(
    package='shelfbot',
    executable='frontier_queue_node',
    name='frontier_queue',
    output='screen',
    parameters=[{
        'merge_radius':    0.40,
        'max_attempts':    3,
        'requeue_delay_s': 30.0,
        'publish_hz':      2.0,
    }],
    condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
)

tag_registry = Node(
    package='shelfbot',
    executable='tag_registry_node',
    name='tag_registry',
    output='screen',
    parameters=[{
        'map_frame':       'map',
        'camera_frame':    'camera_link_optical_frame',
        'pose_avg_alpha':  0.10,
        'publish_hz':      2.0,
    }],
    condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
)

exploration_bt = Node(
    package='shelfbot',
    executable='exploration_bt_node',
    name='exploration_bt',
    output='screen',
    parameters=[{
        'target_tag_ids':          LaunchConfiguration('target_tag_ids'),
        'spin_angular_vel':        0.6,
        'spin_duration_s':         10.5,
        'tick_period_ms':          200,
        'bt_xml_path':             LaunchConfiguration('bt_xml_path'),
        'enable_groot_monitoring': LaunchConfiguration('enable_groot_monitoring'),
    }],
    condition=IfCondition(LaunchConfiguration('run_exploration_mission')),
)

delay_mission = TimerAction(
    period=26.0,
    actions=[frontier_discovery, frontier_queue, tag_registry, exploration_bt],
)
```

All four mission nodes share a single `IfCondition(LaunchConfiguration(
'run_exploration_mission'))`, so the entire tier turns on or off together —
there's no way to end up with, say, `exploration_bt_node` running without
`frontier_queue_node` underneath it.

This replaces the old `scripts/mission_starter.py` workflow, which drove
Nav2's *own* `bt_navigator` via a `behavior_tree` field on a
`NavigateToPose` goal pointed at a `mission.xml` plugin tree. The new
`exploration_bt_node` instead runs its own independent BT.CPP v3 tree that
calls `/navigate_to_pose` as a regular action *client* — `bt_navigator`
still owns single-goal navigation and recovery exactly as it always has;
`exploration_bt_node` just decides *which* goal to send next and *when* the
mission is over. `mission_starter.py` and `mission.xml` (used by
`nav2_dynamic.launch.py`) are unrelated to this tier and unaffected by it.

---

## 8. Build Steps

```bash
# from your workspace root, e.g. ~/shelfbot_ws
cd ~/shelfbot_ws

# Install the one dependency that's not in a standard ROS2 desktop install
sudo apt install ros-humble-behaviortree-cpp-v3

colcon build --packages-select shelfbot --symlink-install
source install/setup.bash
```

If the build fails on `shelfbot/srv/get_frontiers.hpp: No such file or
directory`, it almost always means one of:

- `rosidl_generate_interfaces` wasn't called before the `add_executable`
  blocks that depend on it (ordering matters in CMake)
- The `target_link_libraries(... "${cpp_typesupport_target}")` line is
  missing on that specific executable/library
- A stale build cache — run `rm -rf build install log && colcon build`

If the build fails with `fatal error: shelfbot/exploration/bt_context.hpp:
No such file or directory`, see the path-fix note in §2 — that header lives
at `shelfbot/bt_context.hpp`, not under an `exploration/` subfolder.

---

## 9. Sanity-Check Each Node Independently

Because the nodes are independent, verify each one in isolation before
relying on the full Tier 8 bring-up — this is the actual point of the
modular design. You can run any of these manually even while
`run_exploration_mission:=false` keeps Tier 8 out of the main launch.

### 9.1 `frontier_discovery_node`

```bash
ros2 run shelfbot frontier_discovery_node --ros-args --log-level debug
```

In a second terminal, with SLAM running and `/map` being published:

```bash
ros2 topic echo /frontiers --once
ros2 service call /frontier_discovery/get_frontiers shelfbot/srv/GetFrontiers "{min_size: 5}"
```

In RViz2, add a `MarkerArray` display on `/frontiers/markers` — you should
see cyan discs along the boundary between explored (white/grey) and unknown
(dark grey) regions of the costmap.

**If you see zero frontiers** with a partially-explored map: check your
`min_frontier_size` parameter isn't filtering everything out, and confirm
`/map` is using `-1` for unknown cells (not some other sentinel) by
echoing `ros2 topic echo /map --once | head -50` and inspecting `data`.

### 9.2 `frontier_queue_node`

```bash
ros2 run shelfbot frontier_queue_node --ros-args --log-level debug
```

With `frontier_discovery_node` also running and feeding it:

```bash
ros2 service call /frontier_queue/get_summary shelfbot/srv/GetQueueSummary "{}"
ros2 service call /frontier_queue/get_next shelfbot/srv/GetNextFrontier "{}"
# Note the returned frontier_id, then:
ros2 service call /frontier_queue/update_status shelfbot/srv/UpdateFrontierStatus \
  "{frontier_id: 0, status: 'SUCCEEDED'}"
ros2 service call /frontier_queue/get_summary shelfbot/srv/GetQueueSummary "{}"
# succeeded count should now be 1
```

Add a `MarkerArray` display on `/frontier_queue/markers` in RViz2 — spheres
colour-coded cyan (PENDING) / yellow (ACTIVE) / green (SUCCEEDED) / red
(BLOCKED), confirming the merge-radius deduplication is working (you should
NOT see a dense cluster of near-identical markers at the same spot).

### 9.3 `tag_registry_node`

This one needs the camera pipeline and `apriltag_detector_node` running
(both already part of Tiers 1 and 5 of `real_robot_nav2.launch.py`), plus a
live TF tree.

```bash
ros2 run shelfbot tag_registry_node --ros-args --log-level debug
```

Place a printed AprilTag in view of the camera, then:

```bash
ros2 service call /tag_registry/check_found shelfbot/srv/CheckTagsFound "{target_ids: [1]}"
```

`all_found` should flip to `true` once the registry node logs `Tag 1 first
seen at map (...)`. If it never registers the tag, check `tag_registry`'s
warnings — most commonly the TF chain `map → camera_link_optical_frame` is
not yet complete (SLAM hasn't published `map→odom` yet), in which case you
will see repeating `TF for tag N: ...` warnings.

### 9.4 `exploration_bt_node`

Only run this once 9.1–9.3 are independently confirmed working, plus Nav2
is up. Start it last:

```bash
ros2 run shelfbot exploration_bt_node --ros-args \
  -p target_tag_ids:="[1, 2, 3]" \
  -p enable_groot_monitoring:=true \
  --log-level info
```

You should see, in order:

```
[exploration_bt_node]: Waiting for frontier_queue / tag_registry services...
[exploration_bt_node]: All services available.
[exploration_bt_node]: Loading BT XML: .../share/shelfbot/config/exploration_tree.xml
[exploration_bt_node]: Exploration BT started. Ticking every 200 ms.
[BT] GetNextFrontier: frontier 0 at (1.20, 0.40).
[BT] NavigateToGoal: sending goal to (1.20, 0.40).
[BT] NavigateToGoal: result received (code=2).      # 2 = SUCCEEDED
[BT] SpinAndScan: starting 10.5 s spin at 0.60 rad/s.
[BT] SpinAndScan: complete.
[BT] UpdateFrontierStatus: frontier 0 → SUCCEEDED.
```

---

## 10. Full System Launch

Everything — base robot, Nav2, SLAM, *and* the exploration mission — now
comes up from a single launch file:

```bash
ros2 launch shelfbot real_robot_nav2.launch.py target_tag_ids:="[1, 2, 3]"
```

To bring up only the base robot/Nav2/SLAM stack and skip the mission tier
entirely (e.g. for manual teleop or debugging Nav2 in isolation):

```bash
ros2 launch shelfbot real_robot_nav2.launch.py run_exploration_mission:=false
```

Watch in RViz2 simultaneously:
- `/frontiers/markers` (cyan discs — raw discovery output)
- `/frontier_queue/markers` (status-coloured spheres — what the BT consumes)
- `/tag_registry/markers` (orange cubes — found tags)
- Nav2's standard global/local costmap and plan displays
- `/scan`, sourced from `lidar_relay_node` with `reverse_scan_order: True`
  unchanged from before this mission tier was added

---

## 11. Live Tree Inspection with Groot2 (Optional but Recommended)

```bash
sudo apt install ros-humble-behaviortree-cpp-v3   # already installed from §8
# Download Groot2 (free Community Edition) from:
#   https://www.behaviortree.dev/groot
```

Launch with `enable_groot_monitoring:=true`:

```bash
ros2 launch shelfbot real_robot_nav2.launch.py enable_groot_monitoring:=true
```

open Groot2, choose "Monitor" mode, connect to `localhost` on the default
BT.CPP v3 ZMQ ports (`1666`/`1667`). You'll see the tree light up green/red
live as it ticks — this is the fastest way to debug exactly which branch is
firing when navigation behaves unexpectedly (e.g. confirming whether
`MarkFrontierBlocked` or `NavigateAndScan` fired on a given cycle).

---

## 12. Tuning Reference

| Parameter | Node | Default | Effect |
|---|---|---|---|
| `min_frontier_size` | frontier_discovery | 5 | Higher = fewer, larger frontier regions; eliminates noise specks near walls |
| `merge_radius` | frontier_queue | 0.40 m | Higher = fewer duplicate goals near the same physical spot |
| `max_attempts` | frontier_queue | 3 | How many times a BLOCKED frontier is retried before being abandoned permanently |
| `requeue_delay_s` | frontier_queue | 30 s | Cooldown before retrying a BLOCKED frontier (gives time for transient obstacles, e.g. another moving object, to clear) |
| `pose_avg_alpha` | tag_registry | 0.10 | Lower = smoother/slower-converging tag pose estimate; raise for faster lock-in with a static, well-lit tag |
| `spin_angular_vel` / `spin_duration_s` | exploration_bt | 0.6 rad/s / 10.5 s | Together should satisfy `angular_vel × duration ≈ 2π` for one full rotation per frontier |
| `tick_period_ms` | exploration_bt | 200 ms | BT tick rate — 200 ms is generally fine since all the real work (nav, services) is async and non-blocking |
| `run_exploration_mission` | launch (Tier 8) | `true` | Set `false` to launch Tiers 1–7 only — base navigation with no autonomous mission layer |

---

## 13. Common Failure Modes

**BT root keeps returning FAILURE every tick, robot never moves.**
Check `/frontier_queue/get_summary` — if `total: 0`, `frontier_discovery_node`
isn't publishing anything, almost always because `/map` hasn't been received
yet (QoS mismatch, or SLAM not running). Verify with
`ros2 topic info /map --verbose` that durability is `TRANSIENT_LOCAL` on
both ends.

**Nav2 goal accepted but `NavigateToGoal` never returns.**
Check the Nav2 BT's own recovery behaviors aren't stuck in an infinite spin/
backup loop — `ros2 action info /navigate_to_pose` and look at Nav2's own
logs. The exploration BT correctly waits indefinitely (by design — Nav2 owns
retry logic for a single goal); if Nav2 itself never gives up, fix the
underlying Nav2 config rather than adding a timeout here, or you'll mask the
real issue.

**`SpinAndScan` looks like it never starts.**
Confirm `/cmd_vel` isn't being remapped or intercepted elsewhere. Tier 1 of
`real_robot_nav2.launch.py` remaps `four_wheel_drive_controller/cmd_vel` to
`/cmd_vel`, and Nav2's `velocity_smoother` also publishes to `/cmd_vel`; if
Nav2's lifecycle hasn't fully released control after the navigate action
completes, your spin command can get raced/overwritten — add a short
`rclcpp::sleep_for` before spin start if you observe this in logs.

**Tags found but mission never terminates.**
`target_tag_ids` param must exactly match the IDs your AprilTag detector
reports — check with `ros2 topic echo /tag_detections` and compare `det.id`
field values against what you passed via `target_tag_ids:="[...]"`.

**Mission tier never starts at all, but Tiers 1–7 look fine.**
Check you didn't launch with `run_exploration_mission:=false` (the new Tier
8 condition gate). If it's `true` and still nothing appears at t=26 s, check
for a missed executable in `install(TARGETS ...)` (§5) — `ros2 pkg
executables shelfbot | grep exploration` and `... | grep -E
"frontier|tag_registry"` should each list their respective node.
