# SPDX-License-Identifier: MIT
# Enable both Chrono::FSI solvers. SPH needs a GPU backend (CUDA or HIP),
# while TDPF needs HDF5 and runs without a GPU toolkit. CMake disables SPH
# when no GPU backend is available, leaving TDPF usable.

RUN sudo apt update && \
    sudo apt install --no-install-recommends -y libhdf5-dev && \
    sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_FSI=ON \
    -DCH_ENABLE_MODULE_FSI_SPH=ON \
    -DCH_ENABLE_MODULE_FSI_TDPF=ON \
    -DCH_ENABLE_HDF5=ON"
