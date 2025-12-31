# This snippet install CUDA

ARG CUDA_VERSION

# Install dependencies
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    wget ca-certificates gnupg software-properties-common && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# NVIDIA repo setup (for Ubuntu & Debian)
# Detect architecture & distribution dynamically
RUN DISTRO=$(lsb_release -is | tr '[:upper:]' '[:lower:]') && \
    VERSION=$(lsb_release -rs | tr -d '.') && \
    ARCH=$(dpkg --print-architecture | sed 's/amd64/x86_64/') && \
    sudo wget https://developer.download.nvidia.com/compute/cuda/repos/${DISTRO}${VERSION}/${ARCH}/cuda-keyring_1.1-1_all.deb && \
    sudo dpkg -i cuda-keyring_1.1-1_all.deb && \
    sudo rm cuda-keyring_1.1-1_all.deb

# Install minimal CUDA runtime
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    cuda-toolkit-${CUDA_VERSION} && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib:${LD_LIBRARY_PATH}"