#!/usr/bin/env bash
export SDKROOT="$(xcrun --sdk macosx --show-sdk-path)"
export CONDA_BUILD_SYSROOT="$SDKROOT"
export MACOSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-15.0}"
