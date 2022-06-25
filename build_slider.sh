#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

function exit_if_error {
  if [ $1 -ne 0 ]; then
    echo "ERROR: $2: retval=$1" >&2
    exit $1
  fi
}

export LTO=${LTO:-thin}
export BUILD_CONFIG=private/google-modules/soc/gs/build.config.slider

if [ -n "${SKIP_VENDOR_MODULES}" ] &&
   [ ! -f "${OUT_DIR:-out/android-mainline}/dist/boot.img" ]; then
  echo "WARNING: no \$DIST_DIR/boot.img found. Running full mixed build." >&2
  SKIP_VENDOR_MODULES=
fi

if [ -n "${SKIP_VENDOR_MODULES}" ]; then
  ORIG_OUT_DIR=${OUT_DIR:-out/android-mainline}
  export OUT_DIR=${ORIG_OUT_DIR}/gki_kernel

  BUILD_CONFIG=common/build.config.gki.aarch64 \
    build/kernel/build.sh "$@"
elif [ -z "${GKI_PREBUILTS_DIR}" ]; then
  GKI_BUILD_CONFIG=common/build.config.gki.aarch64 \
    build/kernel/build.sh "$@"
else
  build/kernel/build.sh "$@"
fi

exit_if_error $? "Failed to create mixed build"

# Update the boot.img in the main dist dir
if [ -n "${SKIP_VENDOR_MODULES}" ]; then
  cp ${OUT_DIR}/dist/boot.img ${ORIG_OUT_DIR}/dist/boot.img
fi
