#!/bin/sh
# SPDX-License-Identifier: GPL-2.0

set -e

: ${OUT_DIR:="out/"}
export OUT_DIR

: ${BUILD_CONFIG:="private/gs-google/device-modules/build.config.zuma_hybrid"}

echo "Using build config ${BUILD_CONFIG}"

LTO=thin \
BUILD_CONFIG="${BUILD_CONFIG}" \
build/build.sh "$@"

BASE_OUT=${OUT_DIR}/
export DIST_DIR=${DIST_DIR:-${BASE_OUT}/dist/}
export UNPACK_BOOTIMG_PATH="tools/mkbootimg/unpack_bootimg.py"
mkdir -p "${DIST_DIR}/zebu/"
#set -x
python3 "$UNPACK_BOOTIMG_PATH" --boot_img "${DIST_DIR}/boot.img" --out "${DIST_DIR}/ext_bootimg"
python3 "$UNPACK_BOOTIMG_PATH" --boot_img "${DIST_DIR}/vendor_boot.img" --out "${DIST_DIR}/ext_vendor_bootimg"
cat "${DIST_DIR}/ext_vendor_bootimg/vendor_ramdisk00" "${DIST_DIR}/ext_vendor_bootimg/vendor_ramdisk01" "${DIST_DIR}/ext_bootimg/ramdisk" > \
	      "${DIST_DIR}/zebu/zebu_ramdisk.img"
#set +x
cp -v "${DIST_DIR}/Image" "${DIST_DIR}/zebu/Image"
#cp -v "${DIST_DIR}/ext_vendor_bootimg/dtb" "${DIST_DIR}/zebu/devicetree.dtb"
#cp -v "${DIST_DIR}/zuma-out.dtb" "${DIST_DIR}/zebu/zuma-emulator.dtb"
#cp -v "${DIST_DIR}/zuma-out.dtb" "${DIST_DIR}/zebu/zuma-hybrid.dtb"
cp -v "${DIST_DIR}/zuma-out.dtb" "${DIST_DIR}/zebu/zuma-out.dtb"
