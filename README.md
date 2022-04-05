# Building with Bazel (recommended)

```shell
# set --dist_dir to the distribution directory of your choice
$ tools/bazel run //gs/google-modules/soc-modules:slider_dist -- --dist_dir=out/dist
```

See `build/kernel/kleaf/README.md` for details.

## Disable LTO

**Note**: This only works on `raviole-mainline` branch.

```shell
# set --dist_dir to the distribution directory of your choice
$ tools/bazel run --lto=none //gs/google-modules/soc-modules:slider_dist -- --dist_dir=out/dist
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
