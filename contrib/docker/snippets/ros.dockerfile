# This snippet installs ROS 2 (apt source managed via the ros2-apt-source package)
# NOTE: ROS_DISTRO is a required ARG

ARG USERSHELL
ARG ROS_DISTRO
ARG USERSHELLPROFILE

# Check the image is Ubuntu, error out if not
RUN if [ ! -f /etc/os-release ] || ! grep -q 'ID=ubuntu' /etc/os-release; then echo "This snippet can only be used with an Ubuntu image."; exit 1; fi

# Configure the ROS 2 apt source
RUN sudo apt update && \
      sudo apt install -y --no-install-recommends curl ca-certificates software-properties-common && \
      sudo add-apt-repository -y universe && \
      ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
      curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb" && \
      sudo dpkg -i /tmp/ros2-apt-source.deb && \
      sudo rm /tmp/ros2-apt-source.deb && \
      sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Install ROS 2
RUN sudo apt update && \
      sudo -E apt install -y ros-${ROS_DISTRO}-ros-base python3-colcon-common-extensions build-essential && \
      sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Update shell config
ARG ROS_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}/"
RUN echo ". ${ROS_INSTALL_PREFIX}/setup.${USERSHELL}" >> ${USERSHELLPROFILE}
