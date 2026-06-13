# This snippet installs CUDA from NVIDIA's apt repository.
# NOTE: use a CUDA version that supports the base image's glibc. Ubuntu 26.04
# ships glibc 2.43, which the CUDA 13.1 in Ubuntu's own archive does not
# support (its crt/math_functions.h conflicts with glibc's C23 rsqrt); CUDA
# 13.3 from NVIDIA's repo validates 26.04 and is the right choice there.

ARG CUDA_VERSION

# Install dependencies
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    wget ca-certificates gnupg && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Add NVIDIA's CUDA repository for this Ubuntu release (e.g. ubuntu2604)
RUN UBUNTU_ID=$(. /etc/os-release && echo "${VERSION_ID}" | tr -d '.') && \
    ARCH=$(dpkg --print-architecture | sed 's/amd64/x86_64/;s/arm64/sbsa/') && \
    sudo wget -q "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu${UBUNTU_ID}/${ARCH}/cuda-keyring_1.1-1_all.deb" && \
    sudo dpkg -i cuda-keyring_1.1-1_all.deb && \
    sudo rm cuda-keyring_1.1-1_all.deb

# Install the CUDA toolkit
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    cuda-toolkit-${CUDA_VERSION} && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Ensure the unversioned /usr/local/cuda prefix exists
RUN if [ ! -e /usr/local/cuda ]; then \
        CUDA_PREFIX=$(ls -d /usr/local/cuda-* 2>/dev/null | sort -V | tail -n1); \
        [ -n "${CUDA_PREFIX}" ] && sudo ln -s "${CUDA_PREFIX}" /usr/local/cuda; \
    fi && /usr/local/cuda/bin/nvcc --version

# Set environment variables
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/local/cuda/lib:${LD_LIBRARY_PATH}"
