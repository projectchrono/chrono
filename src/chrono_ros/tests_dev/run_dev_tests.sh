#!/usr/bin/env bash
# =============================================================================
# Dev-time test runner for the chrono_ros core (no ROS, no Chrono required).
#
# Compiles the dependency-free core sources together with each test executable
# under strict warnings and Address/UB sanitizers, then runs them. This is the
# fast inner loop for working on the core; the same tests are also registered
# with Chrono's CTest harness (src/tests/unit_tests/ros) for CI.
#
# Usage: ./run_dev_tests.sh [extra g++ flags...]
# =============================================================================
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_ROOT="$(cd "${HERE}/../.." && pwd)"   # chrono-wisc/src
BUILD_DIR="${HERE}/.build"
mkdir -p "${BUILD_DIR}"

CORE_SOURCES=(
    "${SRC_ROOT}/chrono_ros/core/ChROSCdr.cpp"
    "${SRC_ROOT}/chrono_ros/core/ChROSSchema.cpp"
    "${SRC_ROOT}/chrono_ros/core/ChROSMessageCodec.cpp"
    "${SRC_ROOT}/chrono_ros/core/ChROSControl.cpp"
    "${SRC_ROOT}/chrono_ros/core/transport/ChROSRingBuffer.cpp"
    "${SRC_ROOT}/chrono_ros/core/transport/ChROSSharedMemory.cpp"
    "${SRC_ROOT}/chrono_ros/core/transport/ChROSChannel.cpp"
)

CXX="${CXX:-g++}"
FLAGS=(-std=c++17 -Wall -Wextra -Werror -g -O1
       -fsanitize=address,undefined -fno-sanitize-recover=all
       -I"${SRC_ROOT}" -I"${HERE}" -pthread)

# --- stage 1: syntax-check the sim-side (Chrono-linked) sources against stub
# headers, so API-layer regressions surface without a Chrono install. The node
# (rclcpp-linked) sources cannot be checked here; they compile in CI/Docker.
SIM_SOURCES=(
    "${SRC_ROOT}/chrono_ros/ChROSMessage.cpp"
    "${SRC_ROOT}/chrono_ros/ChROSBridge.cpp"
    "${SRC_ROOT}/chrono_ros/ChROSHandler.cpp"
    "${SRC_ROOT}/chrono_ros/ChROSManager.cpp"
    "${SRC_ROOT}/chrono_ros/ChROSNodeProcess.cpp"
)
echo "== syntax-checking sim-side sources (stub Chrono headers)"
"${CXX}" -std=c++17 -Wall -Wextra -Werror -fsyntax-only \
    -I"${HERE}/.stubs" -I"${SRC_ROOT}" "${SIM_SOURCES[@]}"

# --- stage 2: build and run the core test suite
STATUS=0
for test_source in "${HERE}"/test_*.cpp; do
    name="$(basename "${test_source}" .cpp)"
    binary="${BUILD_DIR}/${name}"
    echo "== building ${name}"
    "${CXX}" "${FLAGS[@]}" "$@" "${test_source}" "${CORE_SOURCES[@]}" -lrt -o "${binary}"
    echo "== running ${name}"
    if ! "${binary}"; then
        STATUS=1
    fi
done

exit ${STATUS}
