# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for PyChrono

ARG CHRONO_INSTALL_DIR

# Update shell config
RUN echo "export PYTHONPATH=\$PYTHONPATH:${CHRONO_INSTALL_DIR}/share/chrono/python" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} \
    -DCH_ENABLE_MODULE_PYTHON=ON"