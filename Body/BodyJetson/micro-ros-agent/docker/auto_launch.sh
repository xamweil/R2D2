#!/usr/bin/env bash
set -e

PORT="${MICROROS_AGENT_PORT:-8888}"
VERBOSITY="${MICROROS_AGENT_VERBOSITY:--v6}"

echo "Starting micro-ROS Agent on UDP port ${PORT}"
exec micro_ros_agent udp4 --port "${PORT}" "${VERBOSITY}"