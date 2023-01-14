# Building with Bazel

```shell
# Files are copied to out/slider/dist
$ tools/bazel run --config=fast //private/google-modules/soc/gs:slider_dist
```

See `build/kernel/kleaf/README.md` for details.

# ABI monitoring with Bazel (recommended)

**Note**: ABI monitoring is not supported on `android-mainline` branch.

```shell
# Update symbol list common/android/abi_gki_aarch64_pixel
$ tools/bazel run --config=fast //private/google-modules/soc/gs:slider_abi_update_symbol_list

# Update ABI common/android/abi_gki_aarch64.xml
$ tools/bazel run //common:kernel_aarch64_abi_update
```
