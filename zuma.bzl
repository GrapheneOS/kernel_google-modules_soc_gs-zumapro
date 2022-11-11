# Copyright (C) 2022 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("//build/bazel_common_rules/dist:dist.bzl", "copy_to_dist_dir")
load("//build/kernel/kleaf:constants.bzl", "aarch64_outs")
load(
    "//build/kernel/kleaf:kernel.bzl",
    "kernel_build",
    "kernel_build_config",
    "kernel_images",
    "kernel_module",
    "kernel_modules_install",
    "merged_kernel_uapi_headers",
)
load("@kernel_toolchain_info//:dict.bzl", "BRANCH", "CLANG_VERSION")

# TODO(b/221278445): Use real GKI. Delete the zuma_gki targets.
def _define_zuma_gki():
    # Note: zuma_gki and zuma_X all writes to zuma_gki_defconfig,
    # which prevents them to be built without sandboxes (--config=local) in
    # parallel. This will be fixed by b/229309039.
    kernel_build(
        name = "zuma_gki",
        srcs = native.glob([
            "arch/arm64/configs/zuma*.fragment",
        ]) + [
            "//aosp-staging:kernel_aarch64_sources",
        ],
        outs = aarch64_outs,
        module_outs = [
            "drivers/block/zram/zram.ko",
            "drivers/net/can/dev/can-dev.ko",
            "drivers/net/can/slcan.ko",
            "drivers/net/can/vcan.ko",
            "mm/zsmalloc.ko",
            "net/can/can-bcm.ko",
            "net/can/can-gw.ko",
            "net/can/can-raw.ko",
            "net/mac80211/mac80211.ko",
            "net/wireless/cfg80211.ko",
        ],
        build_config = "build.config.zuma.gki",
        # For building system_dlkm (b/241855743)
        # Device fragments need to add: '# CONFIG_MODULE_SIG_ALL is not set'
        implicit_outs = [
            "scripts/sign-file",
            "certs/signing_key.pem",
            "certs/signing_key.x509",
        ],
    )

    kernel_modules_install(
        name = "zuma_gki_modules_install",
        kernel_modules = [],
        kernel_build = "zuma_gki",
    )

    kernel_images(
        name = "zuma_gki_images",
        kernel_build = "zuma_gki",
        kernel_modules_install = "zuma_gki_modules_install",
        build_system_dlkm = True,
    )

def define_zuma():
    _define_zuma_gki()

    zuma_in_tree_modules = {
        "emulator": [
            # keep sorted
            "drivers/i2c/i2c-dev.ko",
        ],
        "hybrid": [
            # keep sorted
            "drivers/block/virtio_blk.ko",
            "drivers/crypto/virtio/virtio_crypto.ko",
            "drivers/i2c/i2c-dev.ko",
            "drivers/net/net_failover.ko",
            "drivers/net/virtio_net.ko",
            "drivers/virtio/virtio_balloon.ko",
            "drivers/virtio/virtio_mmio.ko",
            "drivers/virtio/virtio_pci.ko",
            "drivers/virtio/virtio_pci_modern_dev.ko",
            "fs/9p/9p.ko",
            "fs/cachefiles/cachefiles.ko",
            "fs/fscache/fscache.ko",
            "fs/netfs/netfs.ko",
            "net/9p/9pnet.ko",
            "net/9p/9pnet_virtio.ko",
            "net/core/failover.ko",
        ],
    }

    zuma_soc_modules = {
        "emulator": [
            # keep sorted
            "drivers/clocksource/exynos_mct_v3.ko",
            "drivers/gpu/exynos/g2d/g2d.ko",
            "drivers/i2c/busses/i2c-exynos5.ko",
            "drivers/media/platform/exynos/smfc/smfc.ko",
            "drivers/pinctrl/gs/pinctrl-exynos-gs.ko",
            "drivers/soc/google/debug/dss.ko",
            "drivers/soc/google/debug/exynos-coresight.ko",
            "drivers/soc/google/debug/exynos-ecc-handler.ko",
            "drivers/soc/google/debug/itmon.ko",
            "drivers/soc/google/exynos-pd_el3.ko",
            "drivers/soc/google/gs-chipid.ko",
            "drivers/soc/google/vh/kernel/pixel_em/pixel_em.ko",
            "drivers/soc/google/vh/kernel/systrace.ko",
            "drivers/tty/serial/exynos_tty.ko",
        ],
        "hybrid": [
            # keep sorted
            "drivers/clocksource/exynos_mct.ko",
            "drivers/dma-buf/heaps/samsung/samsung_dma_heap.ko",
            "drivers/i2c/busses/i2c-exynos5.ko",
            "drivers/iommu/samsung-iommu-group.ko",
            "drivers/iommu/samsung_iommu_v9.ko",
            "drivers/pinctrl/gs/pinctrl-exynos-gs.ko",
            "drivers/soc/google/exynos-pd_el3.ko",
            "drivers/soc/google/gs-chipid.ko",
            "drivers/tty/serial/exynos_tty.ko",
        ],
    }

    for mode in ("emulator", "hybrid"):
        zuma_dtbos = [
            "google/zuma-{}.dtbo".format(mode),
        ]

        zuma_external_modules = [
            # keep sorted
            ":zuma_soc_{}".format(mode),
            "//private/google-modules/bms/misc:bms-misc.zuma_{}".format(mode),
            "//private/google-modules/display:samsung.zuma_{}".format(mode),
            "//private/google-modules/gpu/mali_kbase:mali_kbase.zuma_{}".format(mode),
            "//private/google-modules/gpu/mali_pixel:mali_pixel.zuma_{}".format(mode),
            "//private/google-modules/lwis:lwis.zuma_{}".format(mode),
        ]

        kernel_build(
            name = "zuma_{}".format(mode),
            srcs = native.glob([
                "build.config.*",
                "arch/arm64/configs/zuma*.fragment",
                "Kconfig.ext",
                "Kconfig.ext_modules",
                "**/Kconfig",
                "arch/arm64/boot/dts/**",
                "include/dt-bindings/**",
                "include/dtc/**",
            ]) + [
                "//aosp-staging:kernel_aarch64_sources",
                "//private/google-modules/bms/misc:bms-misc.kconfig",
                "//private/google-modules/power/reset:reset.kconfig",
                "//private/google-modules/touch/common:common.kconfig",
                "//private/google-modules/touch/fts/ftm5:ftm5.kconfig",
                "//private/google-modules/touch/sec:sec.kconfig",
            ],
            outs = [
                # Sync with build.config.zuma_emulator and build.config.zuma_hybrid
                "google/zuma-a0.dtb",
            ] + zuma_dtbos,
            # TODO(b/221278445): Use real GKI
            # base_kernel = "//aosp-staging:kernel_aarch64",
            base_kernel = ":zuma_gki",
            build_config = "build.config.zuma_{}".format(mode),
            dtstree = "//private/google-modules/soc/gs/arch/arm64/boot/dts:dt",
            kconfig_ext = "Kconfig.ext",
            module_outs = zuma_in_tree_modules[mode],
        )

        kernel_module(
            name = "zuma_soc_{}".format(mode),
            srcs = native.glob(
                ["**"],
                exclude = [
                    ".*",
                    ".*/**",
                    "BUILD.bazel",
                    "**/*.bzl",
                    "build.config.*",
                ],
            ) + [
                "//private/google-modules/bms/misc:headers",
            ],
            outs = zuma_soc_modules[mode],
            kernel_build = "//private/google-modules/soc/gs:zuma_{}".format(mode),
            kernel_module_deps = [
                "//private/google-modules/bms/misc:bms-misc.zuma_{}".format(mode),
            ],
            visibility = [
                # keep sorted
                "//private/google-modules:__subpackages__",
            ],
        )

        kernel_modules_install(
            name = "zuma_{}_modules_install".format(mode),
            kernel_modules = zuma_external_modules,
            kernel_build = ":zuma_{}".format(mode),
        )

        merged_kernel_uapi_headers(
            name = "zuma_{}_merged_uapi_headers".format(mode),
            kernel_modules = zuma_external_modules,
            kernel_build = ":zuma_{}".format(mode),
        )

        kernel_images(
            name = "zuma_{}_images".format(mode),
            build_boot = True,
            build_vendor_kernel_boot = True,
            build_vendor_dlkm = True,
            build_initramfs = True,
            build_dtbo = True,
            dtbo_srcs = [":zuma_{}/{}".format(mode, file) for file in zuma_dtbos],
            kernel_build = ":zuma_{}".format(mode),
            kernel_modules_install = ":zuma_{}_modules_install".format(mode),
            # Keep the following in sync with build.config.zuma:
            # No MODULES_LIST
            # No MODULES_BLOCKLIST
            vendor_dlkm_modules_list = "vendor_dlkm_modules.zuma",
            vendor_dlkm_modules_blocklist = "vendor_dlkm.blocklist.zuma",
            vendor_dlkm_props = "vendor_dlkm.props.zuma",
            deps = [
                # Keep the following in sync with vendor_dlkm.props.zuma:
                # selinux_fc
                "//prebuilts/boot-artifacts/selinux:file_contexts",
            ],
        )

        native.genrule(
            name = "zuma_{mode}_ufdt_overlay".format(mode = mode),
            srcs = [
                ":zuma_{mode}/google/zuma-a0.dtb".format(mode = mode),
                ":zuma_{mode}/google/zuma-{mode}.dtbo".format(mode = mode),
                "//build/kernel:hermetic-tools/ufdt_apply_overlay",
            ],
            outs = [
                # It is a limitation in Bazel that we can't name this
                # zuma-out.dtb, because the one from zuma_emulator and
                # zuma_hybrid conflicts.
                "zuma_{mode}-out.dtb".format(mode = mode),
            ],
            cmd = """set -e
                $(location //build/kernel:hermetic-tools/ufdt_apply_overlay) \\
                    $(location :zuma_{mode}/google/zuma-a0.dtb)              \\
                    $(location :zuma_{mode}/google/zuma-{mode}.dtbo)         \\
                    $(location zuma_{mode}-out.dtb)
            """.format(mode = mode),
        )

        copy_to_dist_dir(
            name = "zuma_{}_dist".format(mode),
            flat = True,
            data = [
                # TODO(b/221278445) use real GKI; replace with //common:kernel_aarch64_additional_artifacts.
                # This will include abi.xml, abi_symbollist and abi_symbollist.report.
                ":zuma_gki",
                ":zuma_gki_headers",
                ":zuma_gki_kmi_symbol_list",
                ":zuma_gki_images",
                "//build/kernel:gki_certification_tools",

                # Device-specific artifacts
                ":zuma_{}".format(mode),
                ":zuma_{}_modules_install".format(mode),
                # At the time of writing (2022-02-04), this intentionally diverges from
                # the behavior of build.sh-style mixed builds by also incorporating
                # UAPI headers of external modules, while build.sh-style mixed builds
                # always uses kernel-uapi-headers.tar.gz from GKI_DIST_DIR.
                # To use (zuma-)GKI's kernel-uapi-headers.tar.gz in DIST_DIR, use
                #     :zuma_gki_uapi_headers
                # instead.
                ":zuma_{}_merged_uapi_headers".format(mode),
                ":zuma_{}_images".format(mode),
                ":zuma_{}_ufdt_overlay".format(mode),
            ],
            dist_dir = "out/{branch}/dist".format(branch = BRANCH),
        )

    # Default is emulator, see build_zuma_zebu.sh
    native.alias(
        name = "zuma",
        actual = ":zuma_emulator",
    )

    native.alias(
        name = "zuma_dist",
        actual = ":zuma_emulator_dist",
    )
