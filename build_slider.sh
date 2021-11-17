#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

function exit_if_error {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    exit $1
  fi
}

TRIM_NONLISTED_KMI=${TRIM_NONLISTED_KMI:-0}
LTO=${LTO:-thin}
KMI_SYMBOL_LIST_STRICT_MODE=${ENABLE_STRICT_KMI:-0}
DEFAULT_CONFIG="gs/kernel/device-modules/build.config.slider"
DEVICE_KERNEL_BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG:-${DEFAULT_CONFIG}}
GKI_KERNEL_OUT_DIR=gki-android-mainline
GKI_KERNEL_BUILD_CONFIG=common/build.config.gki.aarch64

if [ "${LTO}" = "none" ]; then
  echo "LTO=none requires disabling KMI_SYMBOL_STRICT_MODE. Setting to 0..."
  KMI_SYMBOL_LIST_STRICT_MODE=0
fi

if [ -n "${BUILD_ABI}" ]; then
  echo "The ABI update workflow has changed. Please read go/gki-p21-workflow"
  echo "  for instructions on updating ABI/symbol list."
  exit_if_error 1 "BUILD_ABI is deprecated"
fi

# These are for build.sh, so they should be exported.
export LTO
export KMI_SYMBOL_LIST_STRICT_MODE
export TRIM_NONLISTED_KMI
export BASE_OUT=${OUT_DIR:-out}/mixed/
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}

DEVICE_KERNEL_BUILD_CONFIG=${DEVICE_KERNEL_BUILD_CONFIG} \
  GKI_KERNEL_BUILD_CONFIG=${GKI_KERNEL_BUILD_CONFIG} \
  GKI_KERNEL_OUT_DIR=${GKI_KERNEL_OUT_DIR} \
  GKI_DEFCONFIG_FRAGMENT=${GKI_DEFCONFIG_FRAGMENT} \
  ./build_mixed.sh "$@"

exit_if_error $? "Failed to create mixed build"
