# SPDX-License-Identifier: MIT
# This snippet installs Intel oneAPI MKL for Chrono::PardisoMKL
#
# The Chrono::PardisoMKL installation guide requires Intel MKL, enables the
# module with CH_ENABLE_MODULE_PARDISO_MKL, and asks users to set MKL_DIR to the
# directory containing MKLConfig.cmake when CMake cannot find it automatically.
# The same guide notes that MKL 2023 is known to work with Eigen/Chrono, and
# contrib/build-scripts/linux/buildMUMPS.sh carries the matching warning that
# oneAPI 2025 is incompatible with Eigen. Pin this image to MKL 2024.2, a
# versioned pre-2025 package, to keep builds reproducible while avoiding the
# known incompatible 2025 release family.
#
# MKL_ROOT is passed in addition to MKL_DIR. This is still aligned with the
# current Chrono CMake code path: src/chrono_pardisomkl/CMakeLists.txt documents
# MKL_ROOT as the oneMKL root and the oneMKL CMake package also uses it during
# package discovery. Supplying both paths keeps Docker builds deterministic,
# especially when the CMake build directory is backed by a BuildKit cache.

ARG USERSHELLPROFILE

# Add Intel oneAPI APT repository and install MKL
RUN sudo apt update && \
    sudo apt install --no-install-recommends -y wget gnupg ca-certificates && \
    wget -qO- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
        | sudo gpg --dearmor -o /usr/share/keyrings/oneapi-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" \
        | sudo tee /etc/apt/sources.list.d/oneAPI.list && \
    sudo apt update && \
    sudo apt install --no-install-recommends -y intel-oneapi-mkl-devel-2024.2 && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Source oneAPI environment in shell startup so MKLROOT etc. are set
RUN echo ". /opt/intel/oneapi/setvars.sh --force > /dev/null 2>&1" >> ${USERSHELLPROFILE}

# Match the runtime environment recommended by the Chrono::PardisoMKL guide.
ENV MKL_INTERFACE_LAYER=LP64
ENV MKL_THREADING_LAYER=INTEL

# Update CMake options. The MKL CMake package ships with oneAPI under <mkl>/lib/cmake/mkl.
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PARDISO_MKL=ON \
    -DMKL_ROOT=/opt/intel/oneapi/mkl/latest \
    -DMKL_DIR=/opt/intel/oneapi/mkl/latest/lib/cmake/mkl"

# Source oneAPI before building so the compiler sees MKL libraries
ENV PRE_BUILD_SCRIPTS="${PRE_BUILD_SCRIPTS} . /opt/intel/oneapi/setvars.sh --force > /dev/null 2>&1 &&"
