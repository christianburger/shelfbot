#!/usr/bin/env bash
# docker-entrypoint.sh
#
# Runs as chris (USER set in Dockerfile) on every container start.
# Uses NOPASSWD sudo to fix /workspace ownership if Docker auto-created
# the bind-mount directory as root (happens when shelfbot_ws is absent
# on the host at the time 'docker compose up' is called).
#
# This means 'docker compose up' is always safe even after 'rm -rf shelfbot_ws'.

set -e

EXPECTED_UID=$(id -u)
EXPECTED_GID=$(id -g)
WORKSPACE=/workspace

if [ -d "${WORKSPACE}" ]; then
    CURRENT_UID=$(stat -c '%u' "${WORKSPACE}")
    if [ "${CURRENT_UID}" != "${EXPECTED_UID}" ]; then
        echo "[entrypoint] /workspace owned by uid=${CURRENT_UID}, fixing to uid=${EXPECTED_UID}..."
        sudo chown "${EXPECTED_UID}:${EXPECTED_GID}" "${WORKSPACE}"
        echo "[entrypoint] /workspace ownership fixed."
    fi
fi

exec "$@"