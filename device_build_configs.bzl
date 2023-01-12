# SPDX-License-Identifier: GPL-2.0-or-later

"""Defines helper functions for creating debug and staging build configs."""

load("//build/kernel/kleaf:kernel.bzl", "kernel_build_config")

def staging_build_config(name, base_build_config, device_name, gki_build_config_fragment):
    """Creates a staging kernel build config.

    Defines these targets:
    - `{name}`
    - `{name}.gki`

    Args:
      name: name of the main `kernel_build_config` target
      base_build_config: the device build config
      device_name: name of the device
      gki_build_config_fragment: the staging kernel's build config fragment
    """

    native.genrule(
        name = "{}.gen".format(name),
        srcs = [
            gki_build_config_fragment,
        ],
        outs = ["{}.gen.generated".format(name)],
        cmd = """
            echo KERNEL_DIR=private/devices/google/{device_name} > $@
            echo GKI_KERNEL_DIR="aosp-staging" >> $@
            echo GKI_BUILD_CONFIG_FRAGMENT=$(location {gki_build_config_fragment}) >> $@
            """.format(
            device_name = device_name,
            gki_build_config_fragment = gki_build_config_fragment,
        ),
    )
    native.genrule(
        name = "{}.gki.gen".format(name),
        srcs = [
            gki_build_config_fragment,
        ],
        outs = ["{}.gki.gen.generated".format(name)],
        cmd = """
            echo KERNEL_DIR="aosp-staging" > $@
            echo GKI_BUILD_CONFIG_FRAGMENT=$(location {gki_build_config_fragment}) >> $@
            """.format(
            gki_build_config_fragment = gki_build_config_fragment,
        ),
    )

    kernel_build_config(
        name = name,
        srcs = [
            # do not sort
            ":{}.gen".format(name),
            base_build_config,
        ],
    )

    kernel_build_config(
        name = "{}.gki".format(name),
        srcs = [
            # do not sort
            ":{}.gki.gen".format(name),
            "//aosp-staging:build.config.gki.aarch64",
        ],
    )

def create_debug_fragment(
        name,
        base_build_config,
        device_name,
        debug_fragment,
        gki_kernel_dir,
        gki_build_config_fragment = None):
    """Creates a debug build config using the provided build config fragment.

    Defines these targets:
    - `{name}`
    - `{name}.gki`

    Args:
      name: name of the main `kernel_build_config` target
      base_build_config: the device build config
      device_name: name of the device
      debug_fragment: the debug build config fragment
      gki_kernel_dir: the GKI kernel's path to be used, e.g. aosp-staging
      gki_build_config_fragment: the staging kernel's build config fragment
    """

    native.genrule(
        name = "{}.gen".format(name),
        srcs = [
            debug_fragment,
        ],
        outs = ["{}.gen.generated".format(name)],
        cmd = """
            echo KERNEL_DIR=private/devices/google/{device_name}> $@
            echo GKI_KERNEL_DIR="{kernel_dir}" >> $@
            echo GKI_BUILD_CONFIG_FRAGMENT=$(location {debug_fragment}) >> $@
            """.format(
            device_name = device_name,
            debug_fragment = debug_fragment,
            kernel_dir = gki_kernel_dir,
        ),
    )
    native.genrule(
        name = "{}.gki.gen".format(name),
        srcs = [
            debug_fragment,
        ],
        outs = ["{}.gki.gen.generated".format(name)],
        cmd = """
            echo KERNEL_DIR="{kernel_dir}" > $@
            echo GKI_BUILD_CONFIG_FRAGMENT=$(location {debug_fragment}) >> $@
            """.format(
            kernel_dir = gki_kernel_dir,
            debug_fragment = debug_fragment,
        ),
    )

    kernel_build_config(
        name = name,
        srcs = [
            # do not sort
            ":{}.gen".format(name),
            base_build_config,
        ] + [
            # Since we can't source two fragments, we can just append this to
            # the end.
            gki_build_config_fragment,
        ] if gki_build_config_fragment else [],
    )

    kernel_build_config(
        name = "{}.gki".format(name),
        srcs = [
            # do not sort
            ":{}.gki.gen".format(name),
            "//{}:build.config.gki.aarch64".format(gki_kernel_dir),
        ] + [
            # Since we can't source two fragments, we can just append this to
            # the end.
            gki_build_config_fragment,
        ] if gki_build_config_fragment else [],
    )

def device_build_configs(
        name,
        base_build_config,
        device_name,
        gki_kernel_dir,
        gki_build_config_fragment = None,
        staging_kernel = None):
    """Creates the full set of debug configs for a pixel device.

    Defines these targets for each debug config:
    - `{name}.{debug_name}`
    - `{name}.{debug_name}.gki`

    Args:
      name: name of the base `kernel_build_config` target
      base_build_config: the device build config
      device_name: name of the device
      gki_kernel_dir: the GKI kernel's path to be used, e.g. aosp-staging
      gki_build_config_fragment: the staging kernel's build config fragment
      staging_kernel: If True, a staging build config is created
    """

    if staging_kernel:
        staging_build_config(
            name = "{}_staging".format(name),
            base_build_config = base_build_config,
            device_name = device_name,
            gki_build_config_fragment = gki_build_config_fragment,
        )

    debug_types = [
        "blktest",
        "debug_api",
        "debug_kmemleak",
        "debug_locking",
        "debug_memory",
        "debug_memory_accounting",
        "kasan",
        "khwasan",
    ]
    debug_configs_mapping = {}
    debug_gki_configs_mapping = {}
    for debug_name in debug_types:
        create_debug_fragment(
            name = "{name}.{debug_name}".format(name = name, debug_name = debug_name),
            base_build_config = base_build_config,
            device_name = device_name,
            debug_fragment = "//private/google-modules/soc/gs:build.config.slider.{}".format(debug_name),
            gki_kernel_dir = gki_kernel_dir,
            gki_build_config_fragment = gki_build_config_fragment,
        )
        debug_configs_mapping["//private/google-modules/soc/gs:{}".format(debug_name)] = \
            ["//private/devices/google/{device}:{name}.{debug_name}".format(
                name = name,
                device = device_name,
                debug_name = debug_name,
            )]
        debug_gki_configs_mapping["//private/google-modules/soc/gs:{}".format(debug_name)] = \
            ["//private/devices/google/{device}:{name}.{debug_name}.gki".format(
                name = name,
                device = device_name,
                debug_name = debug_name,
            )]

    debug_configs_mapping["//conditions:default"] = \
        [":{name}{staging}".format(
            name = name,
            staging = "_staging" if staging_kernel else "",
        )]
    debug_gki_configs_mapping["//conditions:default"] = \
        [":{name}{staging}.gki".format(
            name = name,
            staging = "_staging" if staging_kernel else "",
        )]

    native.filegroup(
        name = "device_build_config",
        srcs = select(debug_configs_mapping),
    )

    native.filegroup(
        name = "gki_build_config",
        srcs = select(debug_gki_configs_mapping),
    )
