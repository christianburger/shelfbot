#!/usr/bin/env bash
# =============================================================================
# shelfbot_integration_test.sh
# -----------------------------------------------------------------------------
# End-to-end integration test for the shelfbot firmware micro-ROS node.
# Replaces shelfbot_integration_test.py — no rclpy, no Python required.
#
# Usage:
#   source /opt/ros/humble/setup.bash   # or source install/setup.bash
#   bash shelfbot_integration_test.sh [OPTIONS]
#
# Options:
#   -v TAG[,TAG,...]   Enable verbose output for specific tags.
#                      Tags: node,qos,topics,heartbeat,motor,distance,
#                            tof,laser,led,lifecycle,all
#   -t SECONDS         Per-topic receive timeout (default: 8)
#   -w SECONDS         Heartbeat collection window (default: 3)
#   -h                 Show this help
#
# Exit code: 0 = all passed, 1 = one or more failures.
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Defaults
# ---------------------------------------------------------------------------
RECEIVE_TIMEOUT=8
HEARTBEAT_WINDOW=3
EPOCH_MIN=1700000000          # Nov 2023 — below = boot-relative stamp
MOTOR_COUNT_EXPECTED=5
SENSOR_COUNT_EXPECTED=6
LASER_POINTS_EXPECTED=12
NS="shelfbot_firmware"
VERBOSE_TAGS=""               # comma-separated; "all" enables everything

# ---------------------------------------------------------------------------
# Colour codes
# ---------------------------------------------------------------------------
GREEN="\033[92m"
RED="\033[91m"
YELLOW="\033[93m"
CYAN="\033[96m"
BOLD="\033[1m"
DIM="\033[2m"
RESET="\033[0m"

# ---------------------------------------------------------------------------
# Result accumulator  (uses temp file so subshells can write to it)
# ---------------------------------------------------------------------------
RESULTS_FILE=$(mktemp /tmp/shelfbot_results.XXXXXX)
trap 'rm -f "$RESULTS_FILE" /tmp/shelfbot_echo_*.tmp' EXIT

# ---------------------------------------------------------------------------
# Helpers: arg parsing
# ---------------------------------------------------------------------------
usage() {
    grep '^#' "$0" | sed 's/^# \?//' | head -20
    exit 0
}

parse_args() {
    while getopts "v:t:w:h" opt; do
        case $opt in
            v) VERBOSE_TAGS="$OPTARG" ;;
            t) RECEIVE_TIMEOUT="$OPTARG" ;;
            w) HEARTBEAT_WINDOW="$OPTARG" ;;
            h) usage ;;
            *) usage ;;
        esac
    done
}

# ---------------------------------------------------------------------------
# Helper: check if a verbosity tag is active
# ---------------------------------------------------------------------------
is_verbose() {
    local tag="$1"
    [[ "$VERBOSE_TAGS" == *"all"* ]] && return 0
    [[ ",$VERBOSE_TAGS," == *",$tag,"* ]] && return 0
    return 1
}

# ---------------------------------------------------------------------------
# Helper: print a verbose detail line (only if tag active)
# ---------------------------------------------------------------------------
vlog() {
    local tag="$1"; shift
    is_verbose "$tag" && echo -e "    ${DIM}↳ $*${RESET}" || true
}

# ---------------------------------------------------------------------------
# Core: record a test result
# ---------------------------------------------------------------------------
record() {
    local name="$1"
    local passed="$2"
    local detail="${3:-}"
    if [[ "$passed" == "1" ]]; then
        echo -e "  [${GREEN}PASS${RESET}] ${name}${detail:+  ${DIM}— ${detail}${RESET}}"
        echo "PASS|${name}|${detail}" >> "$RESULTS_FILE"
    else
        echo -e "  [${RED}FAIL${RESET}] ${name}${detail:+  ${DIM}— ${detail}${RESET}}"
        echo "FAIL|${name}|${detail}" >> "$RESULTS_FILE"
    fi
}

# ---------------------------------------------------------------------------
# Core: section header
# ---------------------------------------------------------------------------
section() {
    echo -e "\n${BOLD}${1}${RESET}"
    printf '%.0s─' {1..60}; echo
}

# ---------------------------------------------------------------------------
# Core: collect messages from a topic into a temp file
#   collect_topic TOPIC MIN_MSGS TIMEOUT [extra ros2 args...]
#   Returns temp file path on stdout
# ---------------------------------------------------------------------------
collect_topic() {
    local topic="$1"
    local min_msgs="$2"
    local timeout_s="$3"
    shift 3
    local extra_args=("$@")
    local tmpfile
    tmpfile=$(mktemp /tmp/shelfbot_echo_XXXXXX.tmp)

    timeout "$timeout_s" \
        ros2 topic echo "${extra_args[@]}" "$topic" \
        >> "$tmpfile" 2>/dev/null &
    local pid=$!

    local elapsed=0
    while kill -0 "$pid" 2>/dev/null; do
        local sep_count
        sep_count=$(grep -c "^---$" "$tmpfile" 2>/dev/null || true)
        [[ "$sep_count" -ge "$min_msgs" ]] && { kill "$pid" 2>/dev/null; break; }
        sleep 0.2
        elapsed=$(awk "BEGIN{printf \"%.1f\", $elapsed + 0.2}")
        awk "BEGIN{exit ($elapsed >= $timeout_s) ? 0 : 1}" && \
            { kill "$pid" 2>/dev/null; break; } || true
    done
    wait "$pid" 2>/dev/null || true

    echo "$tmpfile"
}

# ---------------------------------------------------------------------------
# Helper: extract first value of a YAML field (simple version)
# ---------------------------------------------------------------------------
extract_field() {
    local file="$1"
    local field="$2"
    grep "$field" "$file" | head -1 | sed "s/.*${field}[[:space:]]*//"
}

# =============================================================================
# TEST MODULES
# =============================================================================

# ---------------------------------------------------------------------------
# 1. Node discovery
# ---------------------------------------------------------------------------
test_node_discovery() {
    section "1. Node discovery"

    local node_list
    node_list=$(ros2 node list 2>/dev/null)

    vlog "node" "Full node list:\n$(echo "$node_list" | sed 's/^/      /')"

    local found=0
    echo "$node_list" | grep -qx "/${NS}" && found=1

    record "${NS} node is visible" "$found" \
        "$(echo "$node_list" | wc -l | tr -d ' ') nodes on graph"

    if [[ "$found" != "1" ]]; then
        echo -e "\n${RED}Node not visible — is firmware connected and micro-ROS agent running?${RESET}"
        return 1
    fi
}

# ---------------------------------------------------------------------------
# 2. QoS profile verification
# ---------------------------------------------------------------------------
test_qos_profiles() {
    section "2. QoS profile verification"

    declare -A expected=(
        ["heartbeat"]="best_effort"
        ["motor_positions"]="best_effort"
        ["distance_sensors"]="best_effort"
        ["led_state"]="best_effort"
        ["tof_distance"]="best_effort"
        ["laser_scan"]="reliable"
    )

    for short in "${!expected[@]}"; do
        local topic="/${NS}/${short}"
        local want="${expected[$short]}"
        local info
        info=$(ros2 topic info -v "$topic" 2>/dev/null || true)

        vlog "qos" "ros2 topic info -v ${topic}:\n$(echo "$info" | sed 's/^/      /')"

        local got
        got=$(echo "$info" | gawk '
            /Publisher/  { in_pub=1 }
            /Subscriber/ { in_pub=0 }
            in_pub && /Reliability:/ { print tolower($2); exit }
        ')

        if [[ -z "$got" ]]; then
            record "QoS ${short}" 0 "no publisher found"
        elif [[ "$got" == "$want" ]]; then
            record "QoS ${short} == ${want}" 1
        else
            record "QoS ${short} == ${want}" 0 "got: ${got}"
        fi
    done
}

# ---------------------------------------------------------------------------
# 3. Message receipt
# ---------------------------------------------------------------------------
test_topic_receipt() {
    section "3. Message receipt (all topics must publish within ${RECEIVE_TIMEOUT}s)"

    local topics=(heartbeat motor_positions distance_sensors tof_distance laser_scan)

    for short in "${topics[@]}"; do
        local topic="/${NS}/${short}"
        local tmpfile
        tmpfile=$(collect_topic "$topic" 1 "$RECEIVE_TIMEOUT" --no-arr)

        local sep_count
        sep_count=$(grep -c "^---$" "$tmpfile" 2>/dev/null || echo 0)

        vlog "topics" "${short} raw sample:\n$(head -10 "$tmpfile" | sed 's/^/      /')"

        if [[ "$sep_count" -ge 1 ]]; then
            record "${short} receives messages" 1 "${sep_count} message(s) collected"
        else
            record "${short} receives messages" 0 "no message in ${RECEIVE_TIMEOUT}s"
        fi
        rm -f "$tmpfile"
    done
}

# ---------------------------------------------------------------------------
# 4. Heartbeat increments monotonically
# ---------------------------------------------------------------------------
test_heartbeat_increments() {
    section "4. Heartbeat counter increments monotonically"

    local topic="/${NS}/heartbeat"
    local tmpfile
    tmpfile=$(collect_topic "$topic" 5 "$((HEARTBEAT_WINDOW + 2))" --no-arr)

    vlog "heartbeat" "Raw heartbeat output:\n$(cat "$tmpfile" | sed 's/^/      /')"

    local result
    result=$(gawk '
        /^data:/ { vals[n++] = $2+0 }
        END {
            if (n < 2) { print "FAIL|only " n " sample(s), need >= 2"; exit }
            mono=1; diffs=""
            for (i=1; i<n; i++) {
                d = vals[i] - vals[i-1]
                diffs = diffs d " "
                if (d < 1) mono=0
            }
            vals_str=""
            for (i=0; i<(n<6?n:5); i++) vals_str=vals_str vals[i] " "
            status = mono ? "PASS" : "FAIL"
            print status "|values=[" vals_str "] diffs=[" diffs "]"
        }
    ' "$tmpfile")

    local status detail
    status=$(echo "$result" | cut -d'|' -f1)
    detail=$(echo "$result" | cut -d'|' -f2)
    [[ "$status" == "PASS" ]] \
        && record "Heartbeat counter increments" 1 "$detail" \
        || record "Heartbeat counter increments" 0 "$detail"

    rm -f "$tmpfile"
}

# ---------------------------------------------------------------------------
# 5. motor_positions payload
# ---------------------------------------------------------------------------
test_motor_positions() {
    section "5. motor_positions payload"

    local topic="/${NS}/motor_positions"
    local tmpfile
    tmpfile=$(collect_topic "$topic" 1 "$RECEIVE_TIMEOUT")

    vlog "motor" "Raw motor_positions:\n$(cat "$tmpfile" | sed 's/^/      /')"

    if ! grep -q "^---$" "$tmpfile" 2>/dev/null; then
        record "motor_positions has data" 0 "no messages"
        rm -f "$tmpfile"; return
    fi

    local result
    result=$(gawk '
        /^data:/ { in_data=1; next }
        in_data && /^- / { vals[n++] = $2+0 }
        in_data && !/^- / && NF>0 { in_data=0 }
        END {
            if (n==0) { print "0|0|"; exit }
            finite=1
            for (i=0; i<n; i++) if (vals[i]>1e6 || vals[i]<-1e6) finite=0
            vals_str=""
            for (i=0; i<n; i++) vals_str=vals_str vals[i] " "
            print n "|" finite "|" vals_str
        }
    ' "$tmpfile")

    local count finite values
    count=$(echo "$result"  | cut -d'|' -f1)
    finite=$(echo "$result" | cut -d'|' -f2)
    values=$(echo "$result" | cut -d'|' -f3)

    [[ "$count" -eq "$MOTOR_COUNT_EXPECTED" ]] \
        && record "motor_positions has ${MOTOR_COUNT_EXPECTED} elements" 1 "values=[$values]" \
        || record "motor_positions has ${MOTOR_COUNT_EXPECTED} elements" 0 "got ${count}"
    record "motor_positions values are finite" "$finite" "values=[$values]"

    rm -f "$tmpfile"
}

# ---------------------------------------------------------------------------
# 6. distance_sensors payload
# ---------------------------------------------------------------------------
test_distance_sensors() {
    section "6. distance_sensors payload"

    local topic="/${NS}/distance_sensors"
    local tmpfile
    tmpfile=$(collect_topic "$topic" 1 "$RECEIVE_TIMEOUT")

    vlog "distance" "Raw distance_sensors:\n$(cat "$tmpfile" | sed 's/^/      /')"

    if ! grep -q "^---$" "$tmpfile" 2>/dev/null; then
        record "distance_sensors has data" 0 "no messages"
        rm -f "$tmpfile"; return
    fi

    local result
    result=$(gawk '
        /^data:/ { in_data=1; next }
        in_data && /^- / { vals[n++] = $2+0 }
        in_data && !/^- / && NF>0 { in_data=0 }
        END {
            ok=1
            for (i=0; i<n; i++) {
                v=vals[i]
                if (v != -1.0 && (v < 0.0 || v > 5000.0)) ok=0
            }
            vals_str=""
            for (i=0; i<n; i++) vals_str=vals_str vals[i] " "
            print n "|" ok "|" vals_str
        }
    ' "$tmpfile")

    local count ok values
    count=$(echo "$result" | cut -d'|' -f1)
    ok=$(echo "$result"    | cut -d'|' -f2)
    values=$(echo "$result" | cut -d'|' -f3)

    [[ "$count" -eq "$SENSOR_COUNT_EXPECTED" ]] \
        && record "distance_sensors has ${SENSOR_COUNT_EXPECTED} elements" 1 "values=[$values]" \
        || record "distance_sensors has ${SENSOR_COUNT_EXPECTED} elements" 0 "got ${count}"
    record "distance_sensors values in range or -1.0" "$ok" "values=[$values]"

    rm -f "$tmpfile"
}

# ---------------------------------------------------------------------------
# 7. tof_distance payload
# ---------------------------------------------------------------------------
test_tof_distance() {
    section "7. tof_distance payload"

    local topic="/${NS}/tof_distance"
    local tmpfile
    tmpfile=$(collect_topic "$topic" 1 "$RECEIVE_TIMEOUT")

    vlog "tof" "Raw tof_distance:\n$(cat "$tmpfile" | sed 's/^/      /')"

    if ! grep -q "^---$" "$tmpfile" 2>/dev/null; then
        record "tof_distance has data" 0 "no messages"
        rm -f "$tmpfile"; return
    fi

    local val
    val=$(extract_field "$tmpfile" "data:")
    local ok
    ok=$(echo "$val" | gawk '{v=$1+0; print (v==-1.0 || (v>=0.0 && v<=12.0)) ? 1 : 0}')

    record "tof_distance is -1.0 or in 0–12m range" "$ok" "value=${val}"
    rm -f "$tmpfile"
}

# ---------------------------------------------------------------------------
# 8. laser_scan payload and timestamp (FIXED extraction)
# ---------------------------------------------------------------------------
test_laser_scan() {
    section "8. laser_scan payload and timestamp"

    local topic="/${NS}/laser_scan"
    local tmpfile
    tmpfile=$(collect_topic "$topic" 1 "$RECEIVE_TIMEOUT")

    vlog "laser" "Raw laser_scan (first 40 lines):\n$(head -40 "$tmpfile" | sed 's/^/      /')"

    if ! grep -q "^---$" "$tmpfile" 2>/dev/null; then
        record "laser_scan has data" 0 "no messages"
        rm -f "$tmpfile"; return
    fi

    local host_epoch
    host_epoch=$(date +%s)

    # Use gawk with flexible field extraction
    local result
    result=$(gawk -v epoch_min="$EPOCH_MIN" -v host_epoch="$host_epoch" \
                  -v pts_expected="$LASER_POINTS_EXPECTED" '
        function extract_value(line, field) {
            if (match(line, field ":[[:space:]]*([0-9.-]+)", arr))
                return arr[1]
            return ""
        }
        /frame_id:/  { frame_id = $2 }
        /range_min:/ { range_min = $2+0 }
        /range_max:/ { range_max = $2+0 }
        {
            sec_val = extract_value($0, "sec")
            if (sec_val != "" && !stamp_sec_done) { stamp_sec = sec_val+0; stamp_sec_done=1 }
            ns_val = extract_value($0, "nanosec")
            if (ns_val != "" && !stamp_ns_done) { stamp_ns = ns_val+0; stamp_ns_done=1 }
        }
        /^ranges:/   { in_ranges=1; next }
        in_ranges && /^- / { ranges[nr++] = $2+0 }
        in_ranges && !/^- / && NF>0 { in_ranges=0 }
        END {
            stamp = stamp_sec + stamp_ns * 1e-9
            delta = (stamp > host_epoch) ? stamp - host_epoch : host_epoch - stamp
            sentinel = range_max + 1.0
            range_ok=1
            for (i=0; i<nr; i++) if (ranges[i]<0 || ranges[i]>sentinel) range_ok=0
            ranges_str=""
            for (i=0; i<(nr<6?nr:6); i++) ranges_str=ranges_str sprintf("%.3f",ranges[i]) " "
            printf "%s|%.4f|%.4f|%.3f|%.3f|%d|%d|%s\n",
                frame_id, range_min, range_max, stamp, delta, nr, range_ok, ranges_str
        }
    ' "$tmpfile")

    local frame_id range_min range_max stamp delta nr range_ok ranges_str
    IFS='|' read -r frame_id range_min range_max stamp delta nr range_ok ranges_str <<< "$result"

    record "laser_scan has ${LASER_POINTS_EXPECTED} range points" \
        "$([[ "$nr" -eq "$LASER_POINTS_EXPECTED" ]] && echo 1 || echo 0)" "got ${nr}"
    record "laser_scan frame_id == 'lidar_frame'" \
        "$([[ "$frame_id" == "lidar_frame" ]] && echo 1 || echo 0)" "got '${frame_id}'"
    record "laser_scan range_min == 0.02m" \
        "$(awk "BEGIN{v=$range_min+0; print (v>0.0199 && v<0.0201)?1:0}")" "got ${range_min}"
    record "laser_scan range_max == 12.0m" \
        "$(awk "BEGIN{v=$range_max+0; print (v>11.9999 && v<12.0001)?1:0}")" "got ${range_max}"
    record "laser_scan stamp is wall-clock (not boot-relative)" \
        "$(awk "BEGIN{print ($stamp > $EPOCH_MIN)?1:0}")" "stamp=${stamp}"
    record "laser_scan stamp within 5s of host clock" \
        "$(awk "BEGIN{print ($delta < 5.0)?1:0}")" "|stamp-host|=${delta}s"
    record "laser_scan range values within bounds" "$range_ok" "ranges=[${ranges_str}…]"

    rm -f "$tmpfile"
}

# ---------------------------------------------------------------------------
# 9. LED round-trip (FIXED: more reliable collection)
# ---------------------------------------------------------------------------
test_led_round_trip() {
    section "9. LED command round-trip (publish led → receive led_state)"

    local led_topic="/${NS}/led"
    local state_topic="/${NS}/led_state"

    for state in "true" "false"; do
        local label
        [[ "$state" == "true" ]] && label="ON" || label="OFF"

        vlog "led" "Publishing LED ${label} to ${led_topic}"
        ros2 topic pub --once "$led_topic" std_msgs/msg/Bool "{data: ${state}}" >/dev/null 2>&1

        # Wait for the state to appear (poll manually)
        local found=0
        for i in {1..20}; do
            local val
            val=$(timeout 1 ros2 topic echo --once --field data "$state_topic" 2>/dev/null || true)
            if [[ "$val" == "$state" ]]; then
                found=1
                break
            fi
            sleep 0.2
        done
        record "LED ${label} command reflected in led_state" "$found"
        sleep 0.5
    done
}

# ---------------------------------------------------------------------------
# 10. QoS compatibility — no mismatched pub/sub pairs
# ---------------------------------------------------------------------------
test_qos_compatibility() {
    section "10. QoS compatibility — no incompatible pairings"

    declare -A want_pub=(
        ["motor_positions"]="best_effort"
        ["laser_scan"]="reliable"
    )

    for short in "${!want_pub[@]}"; do
        local topic="/${NS}/${short}"
        local info
        info=$(ros2 topic info -v "$topic" 2>/dev/null || true)

        vlog "qos" "Compatibility ${short}:\n$(echo "$info" | sed 's/^/      /')"

        local pub_qos sub_qos
        pub_qos=$(echo "$info" | gawk '
            /Publisher/  { in_pub=1 }
            /Subscriber/ { in_pub=0 }
            in_pub && /Reliability:/ { print tolower($2); exit }
        ')
        sub_qos=$(echo "$info" | gawk '
            /Subscriber/ { in_sub=1; found_sub=1 }
            in_sub && /Reliability:/ { print tolower($2); exit }
        ')

        if [[ -z "$pub_qos" ]]; then
            record "${short} QoS pairing" 0 "no publisher found"; continue
        fi

        local compat=1
        [[ "$pub_qos" == "best_effort" && "$sub_qos" == "reliable" ]] && compat=0

        local detail="pub=${pub_qos}"
        [[ -n "$sub_qos" ]] && detail+=" sub=${sub_qos}" || detail+=" (no external subscriber)"

        [[ "$compat" == "1" ]] \
            && record "${short} QoS pairing compatible" 1 "$detail" \
            || record "${short} QoS pairing compatible" 0 "BEST_EFFORT+RELIABLE mismatch — ${detail}"
    done
}

# ---------------------------------------------------------------------------
# 11. Node lifecycle — full graph (FIXED state checking)
# ---------------------------------------------------------------------------
test_node_lifecycle() {
    section "11. Node lifecycle — full graph"

    local -a expected_nodes=(
        apriltag_detector_node
        behavior_server
        bt_navigator
        camera_publisher
        controller_manager
        controller_server
        four_wheel_drive_controller
        joint_state_broadcaster
        lifecycle_manager_navigation
        lidar_relay_node
        planner_server
        robot_state_publisher
        shelfbot_camera
        shelfbot_firmware
        shelfbot_hardware_interface_microros_node
        shelfbot_odometry_node
        smoother_server
        static_tf_lidar
        velocity_smoother
        waypoint_follower
    )

    local node_list lifecycle_list
    node_list=$(ros2 node list 2>/dev/null)
    lifecycle_list=$(ros2 lifecycle nodes 2>/dev/null || true)

    vlog "lifecycle" "ros2 lifecycle nodes:\n$(echo "$lifecycle_list" | sed 's/^/      /')"

    for node in "${expected_nodes[@]}"; do
        local found=0
        echo "$node_list" | grep -qxF "/${node}" && found=1

        if [[ "$found" == "0" ]]; then
            record "/${node} present" 0 "not in ros2 node list"
            continue
        fi

        record "/${node} present" 1

        # Only check lifecycle state if this node is a managed node
        if echo "$lifecycle_list" | grep -qxF "/${node}"; then
            local state_raw
            state_raw=$(ros2 lifecycle get "/${node}" 2>/dev/null || echo "unknown")
            # state_raw can be "active", "inactive", "unconfigured", or "state = [3]"
            local active=0
            if [[ "$state_raw" =~ active ]] || [[ "$state_raw" =~ \[3\] ]]; then
                active=1
            fi
            vlog "lifecycle" "/${node} state_raw = $state_raw"
            record "  /${node} lifecycle == active" "$active" "raw: $state_raw"
        fi
    done
}

# ---------------------------------------------------------------------------
# 12. Per-node detail dump  (only when -v node or -v all)
# ---------------------------------------------------------------------------
report_node_details() {
    is_verbose "node" || return 0

    section "12. Per-node detail report  [verbose: node]"

    local node_list
    node_list=$(ros2 node list 2>/dev/null)

    while IFS= read -r node; do
        echo -e "\n  ${CYAN}${BOLD}${node}${RESET}"
        ros2 node info "$node" 2>/dev/null | sed 's/^/    /' || true
    done <<< "$node_list"
}

# =============================================================================
# Summary
# =============================================================================
print_summary() {
    local total pass fail
    total=$(wc -l < "$RESULTS_FILE" | tr -d ' ')
    pass=$(grep -c "^PASS|" "$RESULTS_FILE" 2>/dev/null || echo 0)
    fail=$(grep -c "^FAIL|" "$RESULTS_FILE" 2>/dev/null || echo 0)

    echo -e "\n${BOLD}$(printf '%.0s─' {1..60})${RESET}"
    if [[ "$fail" -eq 0 ]]; then
        echo -e "${BOLD}Result: ${GREEN}${pass}/${total} passed  ✓ All checks passed${RESET}"
    else
        echo -e "${BOLD}Result: ${RED}${pass}/${total} passed  (${fail} failed)${RESET}"
        echo -e "\n${BOLD}Failed checks:${RESET}"
        gawk -F'|' '$1=="FAIL" {
            printf "  \033[91m✗\033[0m %s", $2
            if ($3 != "") printf "  \033[2m— %s\033[0m", $3
            print ""
        }' "$RESULTS_FILE"
    fi
    echo
}

# =============================================================================
# Main
# =============================================================================
main() {
    parse_args "$@"

    echo -e "\n${BOLD}╔══════════════════════════════════════════════════╗${RESET}"
    echo -e "${BOLD}║        Shelfbot Firmware Integration Test        ║${RESET}"
    echo -e "${BOLD}╚══════════════════════════════════════════════════╝${RESET}"
    echo -e "  Timeout per topic : ${RECEIVE_TIMEOUT}s"
    echo -e "  Heartbeat window  : ${HEARTBEAT_WINDOW}s"
    echo -e "  Host epoch        : $(date +%s)"
    echo -e "  Epoch lower bound : ${EPOCH_MIN}  (Nov 2023)"
    [[ -n "$VERBOSE_TAGS" ]] && \
        echo -e "  Verbose tags      : ${YELLOW}${VERBOSE_TAGS}${RESET}"

    test_node_discovery || { print_summary; exit 1; }

    test_qos_profiles
    test_topic_receipt
    test_heartbeat_increments
    test_motor_positions
    test_distance_sensors
    test_tof_distance
    test_laser_scan
    test_led_round_trip
    test_qos_compatibility
    test_node_lifecycle
    report_node_details

    print_summary

    grep -q "^FAIL|" "$RESULTS_FILE" && exit 1 || exit 0
}

main "$@"