# This snippet installs CUDA (from the Ubuntu archive; on 26.04+ the toolkit
# ships in multiverse and NVIDIA's repo carries no ubuntu2604 toolkit)

ARG CUDA_VERSION

# Install dependencies and enable multiverse (where the CUDA toolkit lives)
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    wget ca-certificates gnupg software-properties-common && \
    sudo add-apt-repository -y multiverse && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Install the CUDA toolkit from the Ubuntu archive
RUN sudo apt update && sudo apt install -y --no-install-recommends \
    cuda-toolkit-${CUDA_VERSION} && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Ensure the unversioned /usr/local/cuda prefix exists
RUN if [ ! -e /usr/local/cuda ]; then \
        CUDA_PREFIX=$(ls -d /usr/local/cuda-* 2>/dev/null | sort -V | tail -n1); \
        if [ -n "${CUDA_PREFIX}" ]; then \
            sudo ln -s "${CUDA_PREFIX}" /usr/local/cuda; \
        else \
            echo "ERROR: CUDA toolkit installed but no /usr/local/cuda-* prefix found" && exit 1; \
        fi; \
    fi && /usr/local/cuda/bin/nvcc --version

# Set environment variables
ENV PATH="/usr/local/cuda/bin:${PATH}"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/local/cuda/lib:${LD_LIBRARY_PATH}"
