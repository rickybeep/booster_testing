#!/usr/bin/env bash

set -eo pipefail
source ros2_ws/install/setup.bash
exec "$@"
