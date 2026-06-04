#!/usr/bin/env python3
"""
shelfbot_integration_test.py – fixed version with node discovery retry.
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Int32, Float32, Bool, Float32MultiArray
from sensor_msgs.msg import LaserScan

# -----------------------------------------------------------------------------
# QoS profiles (must match firmware publishers)
# -----------------------------------------------------------------------------
BEST_EFFORT_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

RELIABLE_QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
RECEIVE_TIMEOUT_S    = 8.0
HEARTBEAT_WINDOW_S   = 3.0
EPOCH_MIN            = 1_700_000_000
MOTOR_COUNT_EXPECTED = 5
SENSOR_COUNT_EXPECTED = 6
LASER_POINTS_EXPECTED = 12
NODE_DISCOVERY_TIMEOUT = 5.0   # seconds to wait for shelfbot_firmware node

NS = "shelfbot_firmware"

# -----------------------------------------------------------------------------
# Test runner helpers
# -----------------------------------------------------------------------------
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
RESET  = "\033[0m"
BOLD   = "\033[1m"

results = []

def record(name: str, passed: bool, detail: str = "") -> bool:
    mark = f"{GREEN}PASS{RESET}" if passed else f"{RED}FAIL{RESET}"
    print(f"  [{mark}] {name}" + (f" — {detail}" if detail else ""))
    results.append((name, passed, detail))
    return passed

def section(title: str) -> None:
    print(f"\n{BOLD}{title}{RESET}")
    print("─" * 60)

# -----------------------------------------------------------------------------
# Collector node
# -----------------------------------------------------------------------------
class Collector(Node):
    def __init__(self):
        super().__init__("shelfbot_test_collector")
        self.lock = threading.Lock()
        self._msgs = {
            "heartbeat":        [],
            "motor_positions":  [],
            "distance_sensors": [],
            "led_state":        [],
            "tof_distance":     [],
            "laser_scan":       [],
        }
        self.create_subscription(Int32,            f"{NS}/heartbeat",
            lambda m: self._store("heartbeat", m),        BEST_EFFORT_QOS)
        self.create_subscription(Float32MultiArray, f"{NS}/motor_positions",
            lambda m: self._store("motor_positions", m),  BEST_EFFORT_QOS)
        self.create_subscription(Float32MultiArray, f"{NS}/distance_sensors",
            lambda m: self._store("distance_sensors", m), BEST_EFFORT_QOS)
        self.create_subscription(Bool,              f"{NS}/led_state",
            lambda m: self._store("led_state", m),        BEST_EFFORT_QOS)
        self.create_subscription(Float32,           f"{NS}/tof_distance",
            lambda m: self._store("tof_distance", m),     BEST_EFFORT_QOS)
        self.create_subscription(LaserScan,         f"{NS}/laser_scan",
            lambda m: self._store("laser_scan", m),       RELIABLE_QOS)
        self._led_pub = self.create_publisher(Bool, f"{NS}/led", RELIABLE_QOS)

    def _store(self, key: str, msg) -> None:
        with self.lock:
            self._msgs[key].append(msg)

    def get(self, key: str) -> list:
        with self.lock:
            return list(self._msgs[key])

    def publish_led(self, state: bool) -> None:
        msg = Bool()
        msg.data = state
        self._led_pub.publish(msg)

    def wait_for(self, key: str, count: int = 1, timeout: float = RECEIVE_TIMEOUT_S) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if len(self.get(key)) >= count:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

# -----------------------------------------------------------------------------
# Checks
# -----------------------------------------------------------------------------
def check_node_visible(node: Node) -> bool:
    section("1. Node discovery")
    deadline = time.monotonic() + NODE_DISCOVERY_TIMEOUT
    visible = False
    while time.monotonic() < deadline:
        names = node.get_node_names()
        if "shelfbot_firmware" in names:
            visible = True
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    record("shelfbot_firmware node is visible", visible,
           f"nodes found after timeout: {names}" if not visible else "")
    return visible

def check_qos_profiles(node: Node) -> None:
    section("2. QoS profile verification")
    expected = {
        f"{NS}/heartbeat":        "BEST_EFFORT",
        f"{NS}/motor_positions":  "BEST_EFFORT",
        f"{NS}/distance_sensors": "BEST_EFFORT",
        f"{NS}/led_state":        "BEST_EFFORT",
        f"{NS}/tof_distance":     "BEST_EFFORT",
        f"{NS}/laser_scan":       "RELIABLE",
    }
    for topic, want in expected.items():
        infos = node.get_publishers_info_by_topic(topic)
        if not infos:
            record(f"QoS {topic.split('/')[-1]}", False, "no publisher found")
            continue
        got = infos[0].qos_profile.reliability.name
        record(f"QoS {topic.split('/')[-1]} == {want}", got == want, f"got {got}" if got != want else "")

def check_topic_receipt(col: Collector) -> None:
    section("3. Message receipt (all topics must publish within timeout)")
    for key in ["heartbeat", "motor_positions", "distance_sensors", "tof_distance", "laser_scan"]:
        arrived = col.wait_for(key, count=1)
        record(f"{key} receives messages", arrived,
               f"no message in {RECEIVE_TIMEOUT_S} s" if not arrived else f"{len(col.get(key))} message(s)")

def check_heartbeat_increments(col: Collector) -> None:
    section("4. Heartbeat counter increments monotonically")
    col.wait_for("heartbeat", count=1)
    deadline = time.monotonic() + HEARTBEAT_WINDOW_S
    while time.monotonic() < deadline:
        rclpy.spin_once(col, timeout_sec=0.1)
    msgs = col.get("heartbeat")
    if len(msgs) < 2:
        record("Heartbeat increments", False, f"only {len(msgs)} message(s), need ≥ 2")
        return
    values = [m.data for m in msgs]
    diffs = [values[i+1] - values[i] for i in range(len(values)-1)]
    mono = all(d >= 1 for d in diffs)
    record("Heartbeat counter increments", mono,
           f"values={values[:5]}{'…' if len(values)>5 else ''} diffs={diffs[:5]}")

def check_motor_positions_payload(col: Collector) -> None:
    section("5. motor_positions payload")
    msgs = col.get("motor_positions")
    if not msgs:
        record("motor_positions has data", False, "no messages"); return
    m = msgs[-1]
    cnt = len(m.data)
    record("motor_positions has 5 elements", cnt == MOTOR_COUNT_EXPECTED, f"got {cnt}")
    record("motor_positions values are finite", all(abs(v) < 1e6 for v in m.data), f"values={list(m.data)}")

def check_distance_sensors_payload(col: Collector) -> None:
    section("6. distance_sensors payload")
    msgs = col.get("distance_sensors")
    if not msgs:
        record("distance_sensors has data", False, "no messages"); return
    m = msgs[-1]
    cnt = len(m.data)
    record(f"distance_sensors has {SENSOR_COUNT_EXPECTED} elements", cnt == SENSOR_COUNT_EXPECTED, f"got {cnt}")
    valid_values = all(v == -1.0 or 0.0 <= v <= 5000.0 for v in m.data)
    record("distance_sensors values in range or -1.0", valid_values, f"values={[round(v,1) for v in m.data]}")

def check_tof_distance_payload(col: Collector) -> None:
    section("7. tof_distance payload")
    msgs = col.get("tof_distance")
    if not msgs:
        record("tof_distance has data", False, "no messages"); return
    v = msgs[-1].data
    record("tof_distance is -1.0 (no sensor) or in 0–12 m range", v == -1.0 or 0.0 <= v <= 12.0, f"value={v:.4f} m")

def check_laser_scan_payload(col: Collector) -> None:
    section("8. laser_scan payload and timestamp")
    msgs = col.get("laser_scan")
    if not msgs:
        record("laser_scan has data", False, "no messages"); return
    m = msgs[-1]
    host_epoch = time.time()
    record(f"laser_scan has {LASER_POINTS_EXPECTED} range points", len(m.ranges) == LASER_POINTS_EXPECTED, f"got {len(m.ranges)}")
    record("laser_scan frame_id == 'lidar_frame'", m.header.frame_id == "lidar_frame", f"got '{m.header.frame_id}'")
    record("laser_scan range_min == 0.02 m", abs(m.range_min - 0.02) < 1e-4, f"got {m.range_min}")
    record("laser_scan range_max == 12.0 m", abs(m.range_max - 12.0) < 1e-4, f"got {m.range_max}")
    stamp_sec = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
    record("laser_scan stamp is wall-clock epoch", stamp_sec > EPOCH_MIN, f"stamp={stamp_sec:.3f} (below {EPOCH_MIN}=boot-relative)" if stamp_sec <= EPOCH_MIN else f"stamp={stamp_sec:.3f}")
    record("laser_scan stamp within 5 s of host clock", abs(stamp_sec - host_epoch) < 5.0, f"|stamp - host| = {abs(stamp_sec - host_epoch):.3f} s")
    sentinel = m.range_max + 1.0
    valid_ranges = all(0.0 <= r <= sentinel for r in m.ranges)
    record("laser_scan range values are within bounds", valid_ranges, f"ranges={[round(r,3) for r in m.ranges]}")

def check_led_round_trip(col: Collector) -> None:
    section("9. LED command round-trip")
    col.publish_led(True)
    deadline = time.monotonic() + RECEIVE_TIMEOUT_S
    received_on = False
    while time.monotonic() < deadline:
        rclpy.spin_once(col, timeout_sec=0.1)
        msgs = col.get("led_state")
        if msgs and msgs[-1].data is True:
            received_on = True
            break
    record("LED ON command reflected in led_state", received_on)
    time.sleep(0.5)
    col.publish_led(False)
    deadline = time.monotonic() + RECEIVE_TIMEOUT_S
    received_off = False
    while time.monotonic() < deadline:
        rclpy.spin_once(col, timeout_sec=0.1)
        msgs = col.get("led_state")
        if msgs and msgs[-1].data is False:
            received_off = True
            break
    record("LED OFF command reflected in led_state", received_off)

def check_no_qos_incompatibility(node: Node) -> None:
    section("10. QoS compatibility")
    pairs = {
        f"{NS}/motor_positions": ("BEST_EFFORT", "BEST_EFFORT"),
        f"{NS}/laser_scan":      ("RELIABLE",    "RELIABLE"),
    }
    for topic, (want_pub, want_sub) in pairs.items():
        pub_infos = node.get_publishers_info_by_topic(topic)
        sub_infos = node.get_subscriptions_info_by_topic(topic)
        if not pub_infos:
            record(f"{topic.split('/')[-1]} QoS pairing", False, "no publisher"); continue
        pub_qos = pub_infos[0].qos_profile.reliability.name
        remote_subs = [s for s in sub_infos if s.node_name != "shelfbot_test_collector"]
        if not remote_subs:
            record(f"{topic.split('/')[-1]} publisher QoS == {want_pub}", pub_qos == want_pub, f"got {pub_qos}")
            continue
        sub_qos = remote_subs[0].qos_profile.reliability.name
        compatible = not (pub_qos == "BEST_EFFORT" and sub_qos == "RELIABLE")
        record(f"{topic.split('/')[-1]} pub={pub_qos} sub={sub_qos} compatible", compatible,
               "BEST_EFFORT pub + RELIABLE sub = incompatible!" if not compatible else "")

def main() -> int:
    rclpy.init()
    col = Collector()
    print(f"\n{BOLD}╔══════════════════════════════════════════════════╗{RESET}")
    print(f"{BOLD}║        Shelfbot Firmware Integration Test        ║{RESET}")
    print(f"{BOLD}╚══════════════════════════════════════════════════╝{RESET}")
    node_ok = check_node_visible(col)
    if not node_ok:
        print(f"\n{RED}Node not visible — is the firmware connected and the micro-ROS agent running?{RESET}")
        rclpy.shutdown()
        return 1
    check_qos_profiles(col)
    print(f"\n  Collecting messages for up to {RECEIVE_TIMEOUT_S} s…")
    for key in ["heartbeat", "motor_positions", "distance_sensors", "tof_distance", "laser_scan"]:
        col.wait_for(key, count=3)
    check_topic_receipt(col)
    check_heartbeat_increments(col)
    check_motor_positions_payload(col)
    check_distance_sensors_payload(col)
    check_tof_distance_payload(col)
    check_laser_scan_payload(col)
    check_led_round_trip(col)
    check_no_qos_incompatibility(col)
    total = len(results)
    passed = sum(1 for _, ok, _ in results if ok)
    failed = total - passed
    colour = GREEN if failed == 0 else RED
    print(f"\n{BOLD}{'─'*60}{RESET}")
    print(f"{BOLD}Result: {colour}{passed}/{total} passed{RESET}", end="")
    if failed:
        print(f"  ({RED}{failed} failed{RESET})")
        print(f"\n{BOLD}Failed checks:{RESET}")
        for name, ok, detail in results:
            if not ok:
                print(f"  {RED}✗{RESET} {name}" + (f" — {detail}" if detail else ""))
    else:
        print(f"  {GREEN}✓ All checks passed{RESET}")
    print()
    rclpy.shutdown()
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())