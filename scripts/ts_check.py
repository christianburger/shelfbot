#!/usr/bin/env python3
"""
ts_check.py — Shelfbot navigation stack timestamp & TF diagnostic.

Collects data from live topics for WINDOW_S seconds, then prints a
colour-coded report covering:

  TOPICS     — stamp age vs wall clock, clock-domain flag
  SCAN       — stamp offset, finite-range fraction, frame_id
  /map       — dimensions, populated %, per-value breakdown
  COSTMAPS   — unknown %, frame_id, stamp age
  TF CHAIN   — each hop: translation, TF entry age
  SCAN→TF    — looks up TF at scan.stamp (exactly what slam_toolbox
               and the global costmap obstacle_layer do internally)

USAGE
  ros2 run shelfbot ts_check
  ros2 run shelfbot ts_check --ros-args -p window:=20

ADD TO CMakeLists.txt
  install(
    PROGRAMS scripts/mission_starter.py scripts/topic_logger.py scripts/ts_check.py
    DESTINATION lib/${PROJECT_NAME}
  )
"""

import math
import sys
import time
import threading

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
import tf2_ros

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

# ── Thresholds ────────────────────────────────────────────────────────────────
DEFAULT_WINDOW_S  = 12.0   # collect data for this many seconds
WARN_AGE_S        = 0.8    # stamp age WARN threshold (s)
FAIL_AGE_S        = 2.0    # stamp age FAIL threshold (s)
TF_TIMEOUT_S      = 0.5    # TF lookup timeout (s)
EPOCH_MIN         = 1_700_000_000   # < this ⟹ boot-relative clock (not wall)

# Expected stamp offset for /scan (lidar_relay re-stamps to now − STAMP_OFFSET)
SCAN_OFFSET_LO_S  = 0.30
SCAN_OFFSET_HI_S  = 1.50
SCAN_OFFSET_TGT_S = 0.60   # target: 600 ms

# ── Colours ───────────────────────────────────────────────────────────────────
GRN  = "\033[92m"
YEL  = "\033[93m"
RED  = "\033[91m"
CYN  = "\033[96m"
BLD  = "\033[1m"
DIM  = "\033[2m"
RST  = "\033[0m"

SEP  = "─" * 65


def _colour(text: str, age: float) -> str:
    """Colour text by age relative to WARN/FAIL thresholds."""
    if age < WARN_AGE_S:
        return f"{GRN}{text}{RST}"
    if age < FAIL_AGE_S:
        return f"{YEL}{text}{RST}"
    return f"{RED}{text}{RST}"


def _pass(detail: str = "") -> str:
    return f"[{GRN}PASS{RST}]" + (f" {detail}" if detail else "")


def _fail(detail: str = "") -> str:
    return f"[{RED}FAIL{RST}]" + (f" {detail}" if detail else "")


def _warn(detail: str = "") -> str:
    return f"[{YEL}WARN{RST}]" + (f" {detail}" if detail else "")


def _result(ok: bool, detail: str = "", warn_only: bool = False) -> str:
    if ok:
        return _pass(detail)
    if warn_only:
        return _warn(detail)
    return _fail(detail)


# ─────────────────────────────────────────────────────────────────────────────
# QoS profiles
# ─────────────────────────────────────────────────────────────────────────────
_QOS_BE = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
_QOS_REL = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)
_QOS_TL = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


# ─────────────────────────────────────────────────────────────────────────────
class ShelfbotTsCheck(Node):
    """Subscribe to key nav topics, collect WINDOW_S seconds, print report."""

    # (topic, msg_type, qos_profile, description)
    TOPIC_SPECS = [
        ('/scan',                     LaserScan,     _QOS_BE,  'lidar relay output'),
        ('/odom',                     Odometry,      _QOS_BE,  'wheel odometry'),
        ('/map',                      OccupancyGrid, _QOS_TL,  'slam_toolbox occupancy grid'),
        ('/local_costmap/costmap',    OccupancyGrid, _QOS_REL, 'Nav2 local costmap'),
        ('/global_costmap/costmap',   OccupancyGrid, _QOS_REL, 'Nav2 global costmap'),
        ('/cmd_vel_nav',              Twist,         _QOS_REL, 'controller → velocity_smoother'),
        ('/cmd_vel',                  Twist,         _QOS_REL, 'velocity_smoother → hardware'),
    ]

    TF_CHAIN = [
        ('map',            'odom',            'slam_toolbox publishes this'),
        ('odom',           'base_footprint',  'hardware interface @ 4 Hz'),
        ('base_footprint', 'base_link',       'URDF static'),
        ('base_link',      'laser_link',      'URDF static'),
    ]

    def __init__(self):
        super().__init__('shelfbot_ts_check')
        self.declare_parameter('window', DEFAULT_WINDOW_S)
        self._window = float(self.get_parameter('window').value)

        self._lock   = threading.Lock()
        self._msgs   = {}
        self._counts = {}

        # TF
        self._tf_buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self._tf_buf, self)

        # Subscriptions
        for topic, mtype, qos, _ in self.TOPIC_SPECS:
            self._msgs[topic]   = None
            self._counts[topic] = 0
            self.create_subscription(
                mtype, topic,
                lambda m, t=topic: self._store(t, m),
                qos,
            )

    # ── Internal helpers ──────────────────────────────────────────────────────
    def _store(self, topic: str, msg) -> None:
        with self._lock:
            self._msgs[topic]    = msg
            self._counts[topic] += 1

    def _get(self, topic):
        with self._lock:
            return self._msgs.get(topic)

    def _cnt(self, topic: int) -> int:
        with self._lock:
            return self._counts.get(topic, 0)

    @staticmethod
    def _stamp_sec(msg) -> float | None:
        if msg is None:
            return None
        s = msg.header.stamp
        return s.sec + s.nanosec * 1e-9

    def _age(self, msg) -> float:
        t = self._stamp_sec(msg)
        if t is None:
            return float('inf')
        return time.time() - t

    # ── Report sections ───────────────────────────────────────────────────────

    def _sec(self, title: str) -> None:
        print(f"\n{BLD}{SEP}{RST}")
        print(f"{BLD}  {title}{RST}")

    # ── 1. Topics ─────────────────────────────────────────────────────────────
    def _report_topics(self) -> None:
        self._sec("1. Topic Stamps")
        wall = time.time()
        hdr  = f"  {'Topic':<38} {'Count':>5}  {'Age s':>6}  {'Stamp epoch':>14}  Notes"
        print(hdr)
        print(f"  {'─'*38}  {'─'*5}  {'─'*6}  {'─'*14}  {'─'*24}")

        for topic, _, _, desc in self.TOPIC_SPECS:
            msg = self._get(topic)
            cnt = self._cnt(topic)
            if msg is None:
                print(f"  {topic:<38} {cnt:>5}  {'—':>6}  {'—':>14}  "
                      f"{_fail('no message')}")
                continue
            stamp = self._stamp_sec(msg)
            age   = wall - stamp
            epoch_ok = stamp > EPOCH_MIN
            age_ok   = age   < FAIL_AGE_S
            note = ("boot-clock!" if not epoch_ok
                    else (f"{DIM}{desc}{RST}" if age_ok else "stale"))
            print(f"  {topic:<38} {cnt:>5}  {_colour(f'{age:6.2f}', age)}"
                  f"  {stamp:>14.1f}  {_result(age_ok and epoch_ok, note)}")

    # ── 2. Scan detail ────────────────────────────────────────────────────────
    def _report_scan(self) -> None:
        self._sec("2. Scan Detail  (/scan from lidar_relay_node)")
        scan: LaserScan = self._get('/scan')
        if scan is None:
            print(f"  {_fail('/scan not received — lidar_relay_node not running?')}")
            return

        wall     = time.time()
        stamp    = self._stamp_sec(scan)
        offset_s = wall - stamp

        total    = len(scan.ranges)
        finite   = sum(1 for r in scan.ranges
                       if not math.isinf(r) and not math.isnan(r))
        pct      = 100.0 * finite / total if total > 0 else 0.0

        epoch_ok  = stamp > EPOCH_MIN
        offset_ok = SCAN_OFFSET_LO_S < offset_s < SCAN_OFFSET_HI_S

        print(f"  frame_id       : {scan.header.frame_id}")
        print(f"  range_min/max  : {scan.range_min:.3f} m / {scan.range_max:.3f} m")
        print(f"  Ranges         : {finite}/{total} finite  ({pct:.1f} %)"
              + (f"  {_warn('< 20 % — sparse scan degrades slam matching')}"
                 if pct < 20 else ""))
        print(f"  Clock domain   : {_result(epoch_ok, 'wall-clock' if epoch_ok else 'BOOT-RELATIVE — check lidar_relay STAMP_OFFSET_NS')}")
        print(f"  Stamp offset   : {_colour(f'{offset_s*1e3:.0f} ms', offset_s if not offset_ok else 0)}"
              f"  target={SCAN_OFFSET_TGT_S*1000:.0f} ms  "
              f"{_result(offset_ok, '' if offset_ok else f'outside {SCAN_OFFSET_LO_S*1000:.0f}–{SCAN_OFFSET_HI_S*1000:.0f} ms window')}")

    # ── 3. Map analysis ───────────────────────────────────────────────────────
    def _report_map(self) -> None:
        self._sec("3. /map Occupancy Grid  (slam_toolbox output)")
        omap: OccupancyGrid = self._get('/map')
        if omap is None:
            print(f"  {_fail('/map not received — slam_toolbox not running?')}")
            return

        data    = omap.data
        total   = len(data)
        unknown = data.count(-1)
        free    = data.count(0)
        occ     = sum(1 for v in data if v > 0)
        pct_unk = 100.0 * unknown / total if total > 0 else 100.0
        pct_pop = 100.0 * (free + occ) / total if total > 0 else 0.0

        info    = omap.info
        w_m     = info.width  * info.resolution
        h_m     = info.height * info.resolution

        map_ok  = pct_pop > 5.0   # at least 5 % of cells must be observed

        print(f"  Dimensions     : {info.width} × {info.height} cells  "
              f"({w_m:.1f} × {h_m:.1f} m @ {info.resolution:.3f} m/cell)")
        print(f"  Origin         : ({info.origin.position.x:.2f}, "
              f"{info.origin.position.y:.2f})")
        print(f"  Unknown  (-1)  : {unknown:>8}  ({pct_unk:5.1f} %)")
        print(f"  Free      (0)  : {free:>8}  ({100.*free/total if total else 0:5.1f} %)")
        print(f"  Occupied (>0)  : {occ:>8}  ({100.*occ/total  if total else 0:5.1f} %)")
        print(f"  Populated      : {_colour(f'{pct_pop:5.1f} %', 0.0 if map_ok else FAIL_AGE_S)}"
              f"  {_result(map_ok, '' if map_ok else 'ALL CELLS UNKNOWN')}")

        if not map_ok:
            print()
            print(f"  {BLD}{YEL}▶ slam_toolbox is tracking pose (map→odom TF exists) but NOT{RST}")
            print(f"  {BLD}{YEL}  inserting scan data into the occupancy grid.{RST}")
            print()
            print(f"  Likely causes:")
            print(f"    • Scan stamp offset too small → TF lookup at scan.stamp fails")
            print(f"      FIX: check 'Scan→TF Gap' section below")
            print(f"    • Scan match score below threshold (sparse environment)")
            print(f"      FIX: reduce loop_match_minimum_response_coarse in slam_toolbox_params.yaml")
            print(f"    • Nav2 started before slam built any map → static_layer gets empty /map")
            print(f"      FIX: increase T_NAV2 in nav_debug.launch.py")

    # ── 4. Costmaps ───────────────────────────────────────────────────────────
    def _report_costmaps(self) -> None:
        self._sec("4. Costmaps")
        for topic, label, dep in [
            ('/local_costmap/costmap',  'local  (frame: odom)', 'obstacle_layer only'),
            ('/global_costmap/costmap', 'global (frame: map) ', 'static_layer + obstacle_layer'),
        ]:
            cm: OccupancyGrid = self._get(topic)
            if cm is None:
                print(f"  {label}  {_fail('not received')}")
                continue

            data    = cm.data
            total   = len(data)
            unknown = sum(1 for v in data if v == -1 or v == 255)
            pct_unk = 100.0 * unknown / total if total > 0 else 100.0
            age     = self._age(cm)
            ok      = pct_unk < 80.0   # >80 % unknown = costmap not useful

            print(f"  {label}")
            print(f"    {cm.info.width}×{cm.info.height} cells  "
                  f"frame={cm.header.frame_id}  "
                  f"stamp_age={_colour(f'{age:.2f}s', age)}  "
                  f"unknown={pct_unk:5.1f}%  "
                  f"({dep})")
            print(f"    {_result(ok, '' if ok else '>80% unknown — see /map section')}")

    # ── 5. TF chain ───────────────────────────────────────────────────────────
    def _report_tf(self) -> None:
        self._sec("5. TF Chain")
        now_rclpy = self.get_clock().now()
        all_ok    = True

        for parent, child, note in self.TF_CHAIN:
            try:
                tf = self._tf_buf.lookup_transform(
                    parent, child,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT_S),
                )
                t  = tf.transform.translation
                stamp_ns = (tf.header.stamp.sec * 1_000_000_000
                            + tf.header.stamp.nanosec)

                # Static transforms (/tf_static) carry stamp=0; avoid showing
                # epoch time as the "age".
                if stamp_ns == 0:
                    age_str = "static"
                    tag     = _pass()
                else:
                    age     = (now_rclpy.nanoseconds - stamp_ns) * 1e-9
                    age_str = _colour(f'{age:.3f}s', age)
                    tag     = _pass()

                print(f"  {parent:<18} → {child:<22} "
                      f"t=[{t.x:6.3f},{t.y:6.3f},{t.z:6.3f}]  "
                      f"age={age_str}  "
                      f"{tag}  {DIM}{note}{RST}")
            except Exception as exc:
                all_ok = False
                print(f"  {parent:<18} → {child:<22} {_fail(str(exc)[:55])}")
                print(f"    {DIM}↳ {note}{RST}")

        if not all_ok:
            print(f"\n  {_warn('Broken TF chain — Nav2 costmaps and slam_toolbox will fail')}")

    # ── 6. Scan → TF gap (the critical diagnostic) ───────────────────────────
    def _report_scan_tf_gap(self) -> None:
        """
        Perform the same TF lookups that slam_toolbox and the global costmap
        obstacle_layer perform internally when processing each scan.
        If any lookup fails here, it fails there too — silently.
        """
        self._sec("6. Scan→TF Gap  (critical for slam_toolbox & global costmap)")

        scan: LaserScan = self._get('/scan')
        if scan is None:
            print(f"  {_fail('/scan not available — cannot run gap test')}")
            return

        wall_now  = time.time()
        stamp_sec = self._stamp_sec(scan)
        offset_ms = (wall_now - stamp_sec) * 1000.0

        scan_rtime = rclpy.time.Time(
            seconds=scan.header.stamp.sec,
            nanoseconds=scan.header.stamp.nanosec,
        )

        offset_ok = SCAN_OFFSET_LO_S * 1000 < offset_ms < SCAN_OFFSET_HI_S * 1000
        print(f"  scan.stamp offset : {_colour(f'{offset_ms:.0f} ms', offset_ms/1000 if not offset_ok else 0)}"
              f"  (lidar_relay target: {SCAN_OFFSET_TGT_S*1000:.0f} ms)")

        # Lookups at scan.stamp — mirrors what slam_toolbox & costmaps do
        checks = [
            ('odom',           'base_footprint',
             'slam_toolbox needs this at scan.stamp',
             'Increase controller update_rate in four_wheel_drive_controller.yaml'),
            ('map',            'odom',
             'global costmap obstacle_layer needs this at scan.stamp',
             'slam_toolbox may not have processed a scan yet; increase T_NAV2'),
            ('base_footprint', 'laser_link',
             'static — should always succeed',
             'Check robot_state_publisher and URDF'),
        ]

        print()
        all_ok = True
        for parent, child, reason, fix in checks:
            try:
                self._tf_buf.lookup_transform(
                    parent, child, scan_rtime,
                    timeout=rclpy.duration.Duration(seconds=TF_TIMEOUT_S),
                )
                print(f"  {parent}→{child:<18} {_pass()}")
                print(f"    {DIM}↳ {reason}{RST}")
            except Exception as exc:
                all_ok = False
                print(f"  {parent}→{child:<18} {_fail(str(exc)[:60])}")
                print(f"    {DIM}↳ {reason}{RST}")
                print(f"    {YEL}↳ FIX: {fix}{RST}")

        if all_ok:
            print(f"\n  {_pass('All TF lookups at scan.stamp succeeded')} — "
                  f"slam_toolbox and global costmap can process this scan.")
        else:
            print(f"\n  {_fail('TF lookup at scan.stamp failed')} — "
                  f"these scans are silently dropped by slam_toolbox/costmap.")
            print(f"  Check: controller update_rate (must be ≥ 4 Hz for 600 ms offset)")
            print(f"         four_wheel_drive_controller.yaml → update_rate: 4")

    # ── Summary ───────────────────────────────────────────────────────────────
    def report(self) -> None:
        print(f"\n{BLD}{'═'*65}{RST}")
        print(f"{BLD}  Shelfbot Navigation Stack — Timestamp & TF Diagnostic{RST}")
        print(f"{BLD}  Wall clock : {time.time():.3f}  |  collected {self._window:.0f} s{RST}")
        print(f"{BLD}{'═'*65}{RST}")

        self._report_topics()
        self._report_scan()
        self._report_map()
        self._report_costmaps()
        self._report_tf()
        self._report_scan_tf_gap()

        print(f"\n{BLD}{SEP}{RST}\n")


# ─────────────────────────────────────────────────────────────────────────────
def main() -> None:
    rclpy.init()
    node   = ShelfbotTsCheck()
    window = node.get_parameter('window').value

    print(f"\n  Collecting data for {window:.0f} s …  (Ctrl+C to abort and print now)\n")

    try:
        deadline = time.monotonic() + window
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()