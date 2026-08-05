#!/usr/bin/env bash
set -e

# shellcheck disable=SC1091
source /usr/local/bin/ros-env.sh

exec "$@"
