## =============================================================================
## PROJECT CHRONO - http://projectchrono.org
##
## Copyright (c) 2020 projectchrono.org
## All rights reserved.
##
## Use of this source code is governed by a BSD-style license that can be found
## in the LICENSE file at the top level of the distribution and at
## http://projectchrono.org/license-chrono.txt.
##
## =============================================================================
## Authors: Conlain Kelly, Colin Vanden Heuvel
## =============================================================================

#FROM nvidia/cuda:10.2-devel-ubuntu18.04
FROM ubuntu:focal

ARG DEBIAN_FRONTEND=noninteractive

## Add an unprivileged user for builds
RUN useradd --no-log-init -r chrono

## Install chrono dependencies and the bare minimum for build tools
RUN apt-get update \
	&& apt-get install -y --no-install-recommends \
		cmake \
		freeglut3-dev \
		gcc \
		g++ \
		libboost-dev \
		libc6-dev \
		libeigen3-dev \
		libglew-dev \
		libglfw3-dev \
		libglm-dev \
		libhdf5-dev \
		libirrlicht-dev \
		libopenmpi-dev \
		libthrust-dev \
		libxxf86vm-dev \
		make \
	&& rm -rf /var/lib/apt/lists \
	&& ldconfig 


## These settings are probably not necessary anymore
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

## Add version metadata
LABEL org.projectchrono.version=5.0.1

## Download blaze intermediates to /tmp
WORKDIR /tmp

## Download and Install third-party packages:
## - blaze [https://bitbucket.org/blaze-lib/blaze/src/master/]
## - Intel MKL
RUN apt-get update \
	&& apt-get install -y --no-install-recommends \
		curl \
		ca-certificates \
		gnupg \
	&& curl -L https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB \
		-o GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB \
	&& APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB \
	&& rm -f GPG-PUB-KEY-INTEL-SW-PRODUCTS-2019.PUB \
	&& echo "deb https://apt.repos.intel.com/mkl all main" > /etc/apt/sources.list.d/intel-mkl.list \
	&& apt-get update \
	&& apt-get install -y --no-install-recommends intel-mkl-64bit-2020.0-088 \
	&& curl -L https://bitbucket.org/blaze-lib/blaze/downloads/blaze-3.7.tar.gz \
		| tar -zxf - \
	&& cp -r blaze-3.7/blaze /usr/local/include \
	&& apt-get remove -y curl ca-certificates openssl \
	&& rm -rf blaze-3.7.tar.gz blaze-3.7 \
	&& rm -rf /var/lib/apt/lists

## Unpack a prebuilt version of chrono into the image. 
##
## This image is built with `-DCMAKE_INSTALL_PREFIX=/usr/local` and installed 
## to a clean directory tree using `fakeroot make DESTDIR=/tmp install`. The 
## resulting tree is compressed into a tarball to provide chrono-root.tar.gz
##
## NOTE: For best results, the archive should be built from the same base image
## as this container
ADD chrono-root.tar.gz prelaunch_env.sh /

## Build-Only Packages
#RUN apt-get update \
#	&& apt-get install -y --no-install-recommends \
#		fish \
#		sudo \
#		git \
#		cmake-curses-gui \
#	&& rm -rf /var/lib/apt/lists

## Set the unprivileged chrono user as default
USER chrono

## Use bash as a generic entrypoint for builds
#ENTRYPOINT ["/bin/bash"]

## Use a predefined environment for production images
ENTRYPOINT ["/bin/bash", "/prelaunch_env.sh"]

