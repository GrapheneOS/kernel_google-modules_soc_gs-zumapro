#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

exec tools/bazel run --config=raviole --config=fast //private/google-modules/soc/gs:slider_dist "$@"
