# SPDX-License-Identifier: MIT
# This snippet installs necessary dependencies for Chrono::ROS
# ROS is required for Chrono::ROS and should be installed first

ARG ROS_DISTRO
ARG USERHOME
ARG USERSHELLPROFILE
ARG ROS_INSTALL_PREFIX
ARG ROS_WORKSPACE_DIR="${USERHOME}/packages/workspace"
ARG CHRONO_ROS_INTERFACES_DIR="${ROS_WORKSPACE_DIR}/src/chrono_ros_interfaces"

# Verify that ROS is installed, exit if not
RUN if [ ! -d "${ROS_INSTALL_PREFIX}" ]; then echo "ROS is required for Chrono::ROS."; exit 1; fi

# chrono_ros_interfaces
RUN mkdir -p ${CHRONO_ROS_INTERFACES_DIR} && \
    git clone https://github.com/projectchrono/chrono_ros_interfaces.git ${CHRONO_ROS_INTERFACES_DIR} && \
    cd ${ROS_WORKSPACE_DIR} && \
    . ${ROS_INSTALL_PREFIX}/setup.sh && \
    colcon build --packages-select chrono_ros_interfaces

# Update shell config
RUN echo ". ${ROS_WORKSPACE_DIR}/install/setup.sh" >> ${USERSHELLPROFILE}

# Update CMake options
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} -DCH_ENABLE_MODULE_ROS=ON"
# Update pre-build scripts
ENV PRE_BUILD_SCRIPTS="${PRE_BUILD_SCRIPTS} . ${ROS_WORKSPACE_DIR}/install/setup.sh &&"
