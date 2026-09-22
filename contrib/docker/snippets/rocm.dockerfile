# SPDX-License-Identifier: MIT
# This snippet installs ROCm/HIP, the AMD counterpart to cuda.dockerfile.
#
# The packages come from Ubuntu's own archive rather than repo.radeon.com. AMD's apt
# repository publishes only jammy and noble, so it has nothing for this image's base,
# while Ubuntu 26.04 carries ROCm 7.1 in universe -- comfortably above Chrono's
# CHRONO_HIP_MIN_VERSION of 5.7.0 -- at about a tenth of the download size.

# Check the image is Ubuntu, error out if not
RUN if [ ! -f /etc/os-release ] || ! grep -q 'ID=ubuntu' /etc/os-release; then echo "rocm.dockerfile requires an Ubuntu image."; exit 1; fi

RUN sudo apt update && \
    sudo apt install -y --no-install-recommends software-properties-common && \
    sudo add-apt-repository -y universe && \
    sudo apt update && \
    sudo apt install -y --no-install-recommends \
    hipcc \
    libamdhip64-dev \
    librocprim-dev \
    libhipcub-dev \
    librocthrust-dev \
    librocrand-dev \
    libhiprand-dev \
    rocm-cmake \
    rocminfo \
    rocm-smi && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/* && \
    hipcc --version

# Update CMake options
#
# CHRONO_ROCM_ROOT: Chrono locates ROCm by looking for <root>/include/hip/hip_version.h under
# CHRONO_ROCM_ROOT, ROCM_PATH, ROCM_HOME, HIP_PATH, then /opt/rocm, /usr/lib/rocm and
# /usr/local/rocm (chrono_find_rocm, cmake/ChronoGPUToolchains.cmake). Ubuntu packages ROCm
# into /usr, which is not one of those roots, so the root has to be named or the toolkit is
# installed and then never found.
#
# It must be named as this CMake variable and NOT as the ROCM_PATH environment variable, even
# though chrono_find_rocm honours both. CMake's own CMakeDetermineHIPCompiler reads ROCM_PATH
# from the environment and then looks for <root>/llvm/bin/clang++, which under Ubuntu's layout
# is /usr/llvm/bin/clang++ and does not exist -- so exporting ROCM_PATH=/usr makes
# check_language(HIP) report NOTFOUND and silently drops every HIP module. Measured:
#   ROCM_PATH unset                          -> /usr/lib/llvm-21/bin/clang++
#   ROCM_PATH=/usr                           -> NOTFOUND
#   CHRONO_ROCM_ROOT=/usr (this variable)    -> /usr/lib/llvm-21/bin/clang++
#
# The vendor is declared rather than detected. `docker build` sees no GPU, so Layer 0 would
# fall back to inferring it from installed SDKs; that happens to give AMD here, but it is a
# guess, and it becomes ambiguous the moment an image carries both toolkits.
#
# CMAKE_HIP_FLAGS: CMake drives clang++ directly rather than through the hipcc wrapper, and
# clang does not locate the AMD device bitcode by itself under this layout -- every HIP
# translation unit fails with "cannot find ROCm device library". hipcc hides this by passing
# the path for you. rocm-device-libs installs the bitcode inside clang's own resource
# directory, so the glob tracks the LLVM version rather than pinning it.
#
# CMAKE_HIP_COMPILER: named explicitly rather than left to check_language(HIP). CMake's own
# search only finds clang under this layout from 4.x on, so on an older base (Ubuntu 24.04
# ships CMake 3.28) it reports no HIP compiler and every HIP module silently switches itself
# off. Naming it works on both. Measured with check_language(HIP):
#                          nothing set   ROCm root only   explicit compiler
#   24.04 / CMake 3.28     NOTFOUND      NOTFOUND         found
#   26.04 / CMake 4.2      found         found            found
#
# Both paths are derived from the bitcode directory, so the compiler and its device library
# always come from the same LLVM even when several are installed.
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCHRONO_GPU_VENDOR=AMD \
    -DCHRONO_ROCM_ROOT=/usr \
    -DCMAKE_HIP_COMPILER=$(ls -d /usr/lib/llvm-*/lib/clang/*/amdgcn/bitcode | sort -V | tail -1 | sed 's|/lib/clang/.*|/bin/clang++|') \
    -DCMAKE_HIP_FLAGS=--rocm-device-lib-path=$(ls -d /usr/lib/llvm-*/lib/clang/*/amdgcn/bitcode | sort -V | tail -1)"
