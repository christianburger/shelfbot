#!/usr/bin/env bash
# docker-entrypoint.sh
# Fixes ownership of named-volume mount points so the runtime user can write
# to them, then hands off to the container's CMD.
set -euo pipefail

EXPECTED_UID=$(id -u)
EXPECTED_GID=$(id -g)

echo "[entrypoint] Starting (UID=${EXPECTED_UID}, GID=${EXPECTED_GID})"

# ── Fix sudo hostname resolution ──────────────────────────────────────────────
# network_mode: host makes the container inherit the host's hostname, but the
# container's /etc/hosts has no entry for it.  sudo calls gethostbyname() for
# PAM/logging on every invocation and prints:
#   "sudo: unable to resolve host <hostname>: Name or service not known"
# Fix: append the entry to /etc/hosts once, before any other sudo call.
# stderr is suppressed on this one write so the warning never surfaces;
# if the write fails, subsequent sudo calls still work — the warning is
# cosmetic, not fatal.
CONTAINER_HOSTNAME=$(hostname 2>/dev/null || true)
if [ -n "${CONTAINER_HOSTNAME}" ] && \
   ! grep -qwF "${CONTAINER_HOSTNAME}" /etc/hosts 2>/dev/null; then
    echo "[entrypoint] Patching /etc/hosts for hostname '${CONTAINER_HOSTNAME}'"
    echo "127.0.0.1 ${CONTAINER_HOSTNAME}" \
        | sudo tee -a /etc/hosts >/dev/null 2>/dev/null \
        || echo "[entrypoint] NOTE: /etc/hosts patch failed — sudo warnings on subsequent calls are harmless"
fi

# ── Helper ────────────────────────────────────────────────────────────────────
# fix_ownership <dir> [recursive=false]
#
#   • Creates <dir> (and all parents) if missing.
#   • If the directory is not owned by the current user, uses sudo to correct
#     it.  Failures are warned rather than fatal so a misconfigured sudo cannot
#     prevent the container from starting.
#   • Pass "true" as the second argument to recurse into subdirectories.
#     Use sparingly — recursion on a large tree (e.g. the colcon workspace) is
#     very slow.
# ─────────────────────────────────────────────────────────────────────────────
fix_ownership() {
    local dir="$1"
    local recursive="${2:-false}"

    # Ensure the directory exists (mkdir -p is idempotent and creates parents).
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
# Fix the workspace root only — NOT recursively.
# • src/  is a bind-mount owned by the host user; no fix needed.
# • build/ install/ log/  are created by colcon under the correct user; a
#   recursive chown here would be extremely slow on an active workspace.
fix_ownership "/home/chris/shelfbot_ws"

# ── JetBrains named-volume mount points ──────────────────────────────────────
# Each volume is mounted at a *subdirectory* of ~/.cache / ~/.config / ~/.local.
# Docker may create the intermediate parent directories (e.g. .local/share) as
# root when it sets up the mount point.  Fix parents first (non-recursive) so
# the full path is traversable, then recurse into the JetBrains dirs so the
# IDE backend can read and write its config and plugin files freely.
fix_ownership "/home/chris/.cache"
fix_ownership "/home/chris/.cache/JetBrains"        true

fix_ownership "/home/chris/.config"
fix_ownership "/home/chris/.config/JetBrains"       true

fix_ownership "/home/chris/.local"
fix_ownership "/home/chris/.local/share"
fix_ownership "/home/chris/.local/share/JetBrains"  true

echo "[entrypoint] Done — handing off to: $*"
exec "$@"
