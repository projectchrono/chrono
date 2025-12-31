# This does the initial setup for the other dockerfiles in the build system.
ARG IMAGE_BASE
ARG IMAGE_TAG

FROM ${IMAGE_BASE}:${IMAGE_TAG}

# Check if the image is Ubuntu or Debian based
RUN cat /etc/os-release | grep -Eqi 'ubuntu|debian' || (echo "Error: Image (${IMAGE_BASE}:${IMAGE_TAG}) is not based on Ubuntu or Debian" && exit 1)

ARG PROJECT
ARG DEBIAN_FRONTEND=noninteractive

# Various arguments and user settings
ARG USERNAME="${PROJECT}"
ARG USERHOME="/home/${USERNAME}"
ARG USERSHELL="bash"
ARG USERSHELLPATH="/bin/${USERSHELL}"
ARG USERSHELLPROFILE="${USERHOME}/.${USERSHELL}rc"

# Install some global dependencies and clean up after
USER root
RUN apt-get update && \
        apt-get install --no-install-recommends -y sudo python3-pip && \
        apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*

# Add user and grant sudo permission.
ARG USER_UID=1000
ARG USER_GID=1000
ARG USER_GROUPS=""
RUN if getent passwd ${USER_UID}; then \
        userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi && \
    useradd --shell /bin/${USERSHELL} --create-home --password "" ${USERNAME} && \
    passwd -d ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME} && \
    groupmod -o -g ${USER_GID} ${USERNAME} && \
    usermod -u ${USER_UID} -g ${USER_GID} ${USERNAME} && \
    if [ -n "${USER_GROUPS}" ]; then \
        for g in ${USER_GROUPS}; do \
            getent group $g || groupadd $g; \
            usermod -aG $g ${USERNAME}; \
        done; \
    fi

# Change to the user
USER ${USERNAME}