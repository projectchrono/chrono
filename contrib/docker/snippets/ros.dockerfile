# This snippet install ROS
# NOTE: ROS_DISTRO is a required ARG

ARG USERSHELL
ARG ROS_DISTRO
ARG USERSHELLPROFILE

# Check the image is Ubuntu, error out if not
RUN if [ ! -f /etc/os-release ] || ! grep -q 'ID=ubuntu' /etc/os-release; then echo "This snippet can only be used with an Ubuntu image."; exit 1; fi

# Install ROS
RUN sudo apt update && \
      sudo apt install curl software-properties-common -y && \
      sudo add-apt-repository universe && \
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
      sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*
RUN sudo apt update && \
      sudo -E apt install -y ros-${ROS_DISTRO}-ros-base python3-colcon-common-extensions build-essential && \
      sudo apt clean && sudo apt autoremove -y && sudo rm -rf /var/lib/apt/lists/*

# Update shell config
ARG ROS_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}/"
RUN echo ". ${ROS_INSTALL_PREFIX}/setup.${USERSHELL}" >> ${USERSHELLPROFILE}