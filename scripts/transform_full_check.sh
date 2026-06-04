#!/usr/bin/env bash
# =============================================================================
# transform_full_check.sh
# -----------------------------------------------------------------------------
# Check TF transforms for shelfbot navigation stack.
# Uses ros2 run tf2_ros tf2_echo with timeout.
#
# Usage:
#   source /opt/ros/humble/setup.bash
#   ./transform_full_check.sh [OPTIONS]
#
# Options:
#   -t FROM TO          Check a single transform (e.g. -t map odom)
#   -a, --all           Check all default transforms (full stack)
#   -l, --list          List all currently published transforms
#   -v, --verbose       Show raw transform output
#   -h, --help          Show this help
#
# Default transforms checked with --all:
#   map → odom
#   odom → base_link
#   base_link → lidar_frame
#   base_link → camera_link_optical_frame
#   base_link → camera_link
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Colour codes
# ---------------------------------------------------------------------------
GREEN="\033[92m"
RED="\033[91m"
YELLOW="\033[93m"
BOLD="\033[1m"
RESET="\033[0m"

TIMEOUT_SEC=3           # seconds to wait for a transform

# ---------------------------------------------------------------------------
# Helper: check a single transform
# Returns 0 if transform exists and is recent, 1 otherwise
# ---------------------------------------------------------------------------
check_transform() {
    local from="$1"
    local to="$2"
    local verbose="${3:-0}"

    echo -n "  ${from} → ${to} : "

    local output
    output=$(timeout "$TIMEOUT_SEC" ros2 run tf2_ros tf2_echo "$from" "$to" 2>/dev/null || true)

    if [[ -z "$output" ]]; then
        echo -e "${RED}✗ NO TRANSFORM${RESET}"
        return 1
    fi

    # Extract translation (should be non-empty if transform exists)
    local trans
    trans=$(echo "$output" | gawk '/Translation/ {print $0; exit}')
    if [[ -z "$trans" ]]; then
        echo -e "${RED}✗ INVALID (no translation)${RESET}"
        return 1
    fi

    if [[ "$verbose" -eq 1 ]]; then
        echo -e "${GREEN}✓ OK${RESET}"
        echo "      $trans" | head -1
    else
        echo -e "${GREEN}✓ OK${RESET}"
    fi
    return 0
}

# ---------------------------------------------------------------------------
# List all transforms currently published (using tf2_monitor or tf2_echo)
# ---------------------------------------------------------------------------
list_transforms() {
    echo -e "\n${BOLD}Currently published transforms (first 20 lines of /tf):${RESET}"
    timeout 2 ros2 topic echo /tf --once 2>/dev/null | grep "frame_id" | head -20 || \
        echo "  No transforms found or /tf topic not active"
    echo -e "\n${BOLD}Static transforms (from /tf_static):${RESET}"
    timeout 2 ros2 topic echo /tf_static --once 2>/dev/null | grep "frame_id" | head -20 || \
        echo "  No static transforms found"
}

# ---------------------------------------------------------------------------
# Check all default transforms for shelfbot navigation
# ---------------------------------------------------------------------------
check_all() {
    local verbose="$1"
    local failures=0

    echo -e "\n${BOLD}Checking full TF transform tree for navigation...${RESET}"
    echo "────────────────────────────────────────────────────────────"

    # Define required transforms: "from to"
    local transforms=(
        "map odom"
        "odom base_link"
        "base_link lidar_frame"
        "base_link camera_link_optical_frame"
        "base_link camera_link"
    )

    for tf in "${transforms[@]}"; do
        read -r from to <<< "$tf"
        if ! check_transform "$from" "$to" "$verbose"; then
            ((failures++))
        fi
    done

    echo "────────────────────────────────────────────────────────────"
    if [[ $failures -eq 0 ]]; then
        echo -e "${BOLD}Result: ${GREEN}All transforms OK${RESET}"
        return 0
    else
        echo -e "${BOLD}Result: ${RED}${failures} transform(s) missing/invalid${RESET}"
        return 1
    fi
}

# ---------------------------------------------------------------------------
# Usage
# ---------------------------------------------------------------------------
usage() {
    grep '^#' "$0" | sed 's/^# \?//' | head -20
    exit 0
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
main() {
    local mode=""
    local check_from=""
    local check_to=""
    local verbose=0

    if [[ $# -eq 0 ]]; then
        usage
    fi

    while [[ $# -gt 0 ]]; do
        case "$1" in
            -t)
                if [[ $# -lt 3 ]]; then
                    echo "Error: -t requires FROM and TO arguments"
                    usage
                fi
                mode="single"
                check_from="$2"
                check_to="$3"
                shift 3
                ;;
            -a|--all)
                mode="all"
                shift
                ;;
            -l|--list)
                mode="list"
                shift
                ;;
            -v|--verbose)
                verbose=1
                shift
                ;;
            -h|--help)
                usage
                ;;
            *)
                echo "Unknown option: $1"
                usage
                ;;
        esac
    done

    case "$mode" in
        single)
            check_transform "$check_from" "$check_to" "$verbose"
            exit $?
            ;;
        all)
            check_all "$verbose"
            exit $?
            ;;
        list)
            list_transforms
            exit 0
            ;;
        *)
            usage
            ;;
    esac
}

main "$@"