#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

set -e

: ${OUT_DIR:="out/"}
export OUT_DIR

: ${BUILD_CONFIG:="private/google-modules/soc/gs/build.config.zuma_emulator"}

echo "Using build config ${BUILD_CONFIG}"

FAST_BUILD=1 \
BUILD_CONFIG="${BUILD_CONFIG}" \
GKI_BUILD_CONFIG=private/google-modules/soc/gs/build.config.zuma.gki \
CORE_KERNEL_FRAGMENT_DEFCONFIG=zuma_emulator_modified_gki.fragment \
build/build.sh "$@"

BASE_OUT=${OUT_DIR}/
DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
BUILDTOOLS_PREBUILT_BIN=build/kernel/build-tools/path/linux-x86
if ! BUILDTOOLS_PREBUILT_BIN_PATH=$(readlink -f $(dirname $0)/../../../../${BUILDTOOLS_PREBUILT_BIN}); then
	echo "Failed to find ${BUILDTOOLS_PREBUILT_BIN}" >&2
	exit 1
elif [ -d "${BUILDTOOLS_PREBUILT_BIN_PATH}" ]; then
	PATH=${BUILDTOOLS_PREBUILT_BIN_PATH}:${PATH}
fi
UNPACK_BOOTIMG_PATH="tools/mkbootimg/unpack_bootimg.py"
mkdir -p "${DIST_DIR}/zebu/"
#set -x
"$UNPACK_BOOTIMG_PATH" --boot_img "${DIST_DIR}/boot.img" --out "${DIST_DIR}/ext_bootimg"
"$UNPACK_BOOTIMG_PATH" --boot_img "${DIST_DIR}/vendor_boot.img" --out "${DIST_DIR}/ext_vendor_bootimg"
cat "${DIST_DIR}/ext_vendor_bootimg/vendor_ramdisk00" "${DIST_DIR}/ext_vendor_bootimg/vendor_ramdisk01" "${DIST_DIR}/ext_bootimg/ramdisk" > \
	      "${DIST_DIR}/zebu/zebu_ramdisk.img"
#set +x
cp -v "${DIST_DIR}/Image" "${DIST_DIR}/zebu/Image"
#cp -v "${DIST_DIR}/ext_vendor_bootimg/dtb" "${DIST_DIR}/zebu/devicetree.dtb"
#cp -v "${DIST_DIR}/zuma-out.dtb" "${DIST_DIR}/zebu/zuma-emulator.dtb"
#cp -v "${DIST_DIR}/zuma-out.dtb" "${DIST_DIR}/zebu/zuma-hybrid.dtb"
cp -v "${DIST_DIR}/zuma-out.dtb" "${DIST_DIR}/zebu/zuma-out.dtb"
