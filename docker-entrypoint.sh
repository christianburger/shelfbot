#!/usr/bin/env bash
# docker-entrypoint.sh
#
# Ensures named-volume mount points are owned by the runtime user, then
# hands off to the container CMD.
#
# ┌─ PREREQUISITE ──────────────────────────────────────────────────────────┐
# │ This script uses sudo.  The Dockerfile MUST include:                    │
# │                                                                         │
# │   RUN echo "Defaults !fqdn" > /etc/sudoers.d/no-fqdn \                 │
# │       && chmod 0440 /etc/sudoers.d/no-fqdn                             │
# │                                                                         │
# │ Without it, Ubuntu's sudo attempts FQDN resolution on every call and   │
# │ prints "sudo: unable to resolve host <hostname>" because the container  │
# │ inherits the host hostname via network_mode: host but /etc/hosts has    │
# │ no matching entry.                                                       │
# │                                                                         │
# │ Patching /etc/hosts from inside the entrypoint cannot fix this:         │
# │   • writing to /etc/hosts requires sudo (root-owned, mode 644)          │
# │   • sudo warns about the hostname before it executes any command        │
# │   • with tty: true, sudo writes to /dev/tty — 2>/dev/null has no       │
# │     effect, making stderr redirection completely useless here            │
# └─────────────────────────────────────────────────────────────────────────┘

set -euo pipefail

EXPECTED_UID=$(id -u)
EXPECTED_GID=$(id -g)

echo "[entrypoint] Starting (UID=${EXPECTED_UID}, GID=${EXPECTED_GID})"

# ── fix_ownership <dir> [recursive=false] ─────────────────────────────────────
#
# 1. Creates <dir> and all parents if they do not exist.
# 2. Checks the directory owner.  If it is already the current user, skips.
# 3. Otherwise calls sudo chown to correct ownership.
#
# recursive=false  →  chown the directory itself only (no -R).
#   Use for workspace roots and intermediate path components where the
#   contents are either bind-mounted or created by user processes.
#
# recursive=true   →  chown -R the entire subtree.
#   Use for named-volume IDE dirs so the backend can read/write freely.
#   Do NOT use on large trees (build/, install/, log/) — extremely slow.
#
# Failures are non-fatal: a warning is printed and the script continues so
# that a misconfigured sudo cannot prevent the container from starting.
# ─────────────────────────────────────────────────────────────────────────────
fix_ownership() {
    local dir="$1"
    local recursive="${2:-false}"

    # mkdir -p is idempotent; suppressing stderr keeps the log clean for
    # dirs that already exist (the common case after the first startup).
    if ! mkdir -p "${dir}" 2>/dev/null; then
        echo "[entrypoint] WARNING: cannot create ${dir} — skipping"
        return 0
    fi

    local current_uid
    current_uid=$(stat -c '%u' "${dir}" 2>/dev/null || echo "0")

    if [ "${current_uid}" = "${EXPECTED_UID}" ]; then
        echo "[entrypoint] OK     ${dir}"
        return 0
    fi

    echo "[entrypoint] Fixing ${dir}  (uid ${current_uid} → ${EXPECTED_UID})"
    if [ "${recursive}" = "true" ]; then
        sudo chown -R "${EXPECTED_UID}:${EXPECTED_GID}" "${dir}" \
            || echo "[entrypoint] WARNING: chown -R failed on ${dir}"
    else
        sudo chown "${EXPECTED_UID}:${EXPECTED_GID}" "${dir}" \
            || echo "[entrypoint] WARNING: chown failed on ${dir}"
    fi
}

# ── Colcon workspace ──────────────────────────────────────────────────────────
#
# Fix the workspace root only — never recurse.
#
# Why not recursive?
#   src/          bind-mount from the host; already owned by the host user
#   build/        created by colcon running as chris; correct owner already
#   install/      same
#   log/          same
#
# A recursive chown here on an active workspace (hundreds of thousands of
# files across build/ install/ log/) would be extremely slow for no benefit.
fix_ownership "/home/chris/shelfbot_ws"

# ── JetBrains named-volume mount points ──────────────────────────────────────
#
# Each JetBrains volume is mounted at a subdirectory:
#   clion_cache  →  /home/chris/.cache/JetBrains
#   clion_config →  /home/chris/.config/JetBrains
#   clion_local  →  /home/chris/.local/share/JetBrains
#
# Docker creates the volume mount point and any missing intermediate
# directories (e.g. .local/share) as root.  If those intermediates are
# root-owned, the JetBrains backend cannot traverse the path.
#
# Fix order: parent first (non-recursive, just the dir node itself), then
# the JetBrains subdir recursively so the backend can read/write freely.
fix_ownership "/home/chris/.cache"
fix_ownership "/home/chris/.cache/JetBrains"        true

fix_ownership "/home/chris/.config"
fix_ownership "/home/chris/.config/JetBrains"       true

fix_ownership "/home/chris/.local"
fix_ownership "/home/chris/.local/share"
fix_ownership "/home/chris/.local/share/JetBrains"  true

echo "[entrypoint] Done — handing off to: $*"
exec "$@"
