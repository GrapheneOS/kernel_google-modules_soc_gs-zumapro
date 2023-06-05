# SPDX-License-Identifier: GPL-2.0
# Kleaf tests. These tests the functionality of Kleaf on a real device target.

load(
    "//build/kernel/kleaf:kernel.bzl",
    "kernel_images",
)
load(":slider.bzl", "SLIDER_DTBOS")

def define_slider_kleaf_tests():
    """Define some tests with the virtual device targets.
    These are useful because the GKI package (//common) does not have external
    modules. These tests allows us to test some functionalites of Kleaf.
    """
    NONE = "vendor_boot_None"
    VENDOR_BOOT = "vendor_boot"
    VENDOR_KERNEL_BOOT = "vendor_kernel_boot"
    targets = []
    for build_boot in (True, False):
        for vendor_boot in (NONE, VENDOR_BOOT, VENDOR_KERNEL_BOOT):
            for build_initramfs in (True, False):
                for build_vendor_dlkm in (True, False):
                    for build_dtbo in (True, False):
                        for build_system_dlkm in (True, False):
                            build_vendor_boot = vendor_boot == VENDOR_BOOT
                            build_vendor_kernel_boot = vendor_boot == VENDOR_KERNEL_BOOT
                            suffix = "initramfs_{initramfs}-boot_{boot}-{vendor_boot}-vendor_dlkm_{vendor_dlkm}-system_dlkm_{system_dlkm}-{dtbo}".format(
                                initramfs = str(build_initramfs),
                                boot = str(build_boot),
                                vendor_boot = vendor_boot,
                                vendor_dlkm = str(build_vendor_dlkm),
                                system_dlkm = str(build_system_dlkm),
                                dtbo = str(build_dtbo),
                            )
                            kernel_images(
                                name = "slider-{}".format(suffix),
                                build_initramfs = build_initramfs,
                                build_boot = build_boot,
                                build_vendor_boot = build_vendor_boot,
                                build_vendor_kernel_boot = build_vendor_kernel_boot,
                                build_vendor_dlkm = build_vendor_dlkm,
                                build_system_dlkm = build_system_dlkm,
                                base_kernel_images = "//common:kernel_aarch64_images_download_or_build",
                                build_dtbo = build_dtbo,
                                dtbo_srcs = [":slider/" + e for e in SLIDER_DTBOS] if build_dtbo else None,
                                kernel_build = ":slider",
                                kernel_modules_install = ":slider_modules_install",
                                modules_list = "vendor_boot_modules.gs101",
                                vendor_dlkm_modules_list = "vendor_dlkm_modules.slider" if build_vendor_dlkm else None,
                                vendor_dlkm_props = "vendor_dlkm.props.slider" if build_vendor_dlkm else None,
                                vendor_ramdisk_binaries = ["//prebuilts/boot-artifacts/ramdisks:vendor_ramdisk-oriole.img"],
                                deps = [
                                    "//prebuilts/boot-artifacts/selinux:vendor_dlkm_file_contexts",
                                ],
                            )
                            targets.append(":slider-{}".format(suffix))
    native.filegroup(
        name = "slider_test_images",
        srcs = targets,
    )
