# syntax = devthefuture/dockerfile-x
# Standalone Chrono image build entrypoint.
#
# Usage:
#   1. Download the OptiX installer required by Chrono::Sensor and place it at:
#      contrib/docker/data/NVIDIA-OptiX-SDK-9.0.0-linux64-x86_64.sh
#   2. Build from contrib/docker so the dockerfile-x INCLUDE paths and
#      OPTIX_SCRIPT path are resolved relative to this directory:
#
#   docker build -f release.dockerfile -t chrono/chrono:release .
#
# Common overrides:
#   docker build -f release.dockerfile -t chrono/chrono:release \
#     --build-arg CHRONO_CUDA_ARCHITECTURES="80;86;89" .
#
# This mirrors the docker-compose dev build defaults while allowing a direct
# docker build invocation.

ARG PROJECT=chrono
ARG IMAGE_BASE=ubuntu
ARG IMAGE_TAG=22.04
ARG USER_GROUPS="dialout video"
ARG PIP_REQUIREMENTS=black
ARG APT_DEPENDENCIES="vim cmake-curses-gui"
ARG USER_SHELL_ADD_ONS="alias python=python3"
ARG CUDA_VERSION=12-9
ARG ROS_DISTRO=humble
ARG OPTIX_SCRIPT=data/NVIDIA-OptiX-SDK-9.0.0-linux64-x86_64.sh

# Base user and OS setup
INCLUDE ./common/base.dockerfile

# Chrono and module snippets
INCLUDE ./snippets/chrono.dockerfile

# Common shell/package finishing steps
INCLUDE ./common/common.dockerfile

# Final user, working directory, and shell
INCLUDE ./common/final.dockerfile
