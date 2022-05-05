# Building with Bazel (recommended)

```shell
# Files are copied to out/{branch}/dist
$ tools/bazel run //gs/google-modules/soc/gs:slider_dist
```

See `build/kernel/kleaf/README.md` for details.

## Disable LTO

**Note**: This only works on `raviole-mainline` branch.

```shell
# Files are copied to out/{branch}/dist
$ tools/bazel run --lto=none //gs/google-modules/soc/gs:slider_dist
```

# Building with `build_slider.sh` (legacy)

```shell
$ build/build_slider.sh
```

## Disable LTO

**Note**: This only works on `raviole-mainline` branch.

```shell
$ LTO=none build/build_slider.sh
```
