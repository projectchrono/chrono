Building a Docker Image with Chrono {#docker_installation}
==========================

This guide describes how to build a Docker image with Chrono installed, including selected modules and dependencies. The image is created using a custom Dockerfile that aggregates multiple snippet files—each appending necessary CMake options and pre-build environment commands. Docker Compose orchestrates the build and run process.

## Prerequisites

Ensure you have:

- Docker installed on your system. If not, download and install Docker from the [official website](https://docs.docker.com/get-docker/).
- Docker Compose installed on your system. If not, download and install Docker Compose from the [official website](https://docs.docker.com/compose/install/). This is optional but recommended for orchestrating the build and run process. Docker compose will be used in this guide.
- You have cloned the Chrono repository.

## Background

The provided `docker-compose.yml` defines two services: `dev` and `vnc`. The `dev` service is the primary image which builds the Chrono library with selected modules and dependencies. The `vnc` service is optional and aids in visualization on systems where `X11` is not available and/or when remotely accessing a machine without a display.

\include docker-compose.yml

You may also provide additional dependencies or requirements in the `docker-compose.yml` at build time using the `APT_DEPENDENCIES` and `PIP_DEPENDENCIES` environment variables. You also can additional build args as necessary for your snippets. The base image must be `debian`-based (and some modules may require `ubuntu`-based images).

The default `docker-compose.yml` file will attach a NVIDIA GPU to the container if available. If you don't have a NVIDIA GPU, you can comment out the parts which follow `deploy` in the `docker-compose.yml` file.

To simplify the dockerfiles, we leverage an open source project called [`dockerfile-x`](https://github.com/devthefuture-org/dockerfile-x) (it doesn't require any installation). This project provides the `INCLUDE` directive, which allows us to include multiple files in a single Dockerfile. In this way, we can call `INCLUDE` only on the modules we need, and the final dockerfile will be generated automatically.

\include snippets/chrono.dockerfile

You can then comment out (or create new) snippets which include the modules you need. The `CMAKE_OPTIONS` variable should be updated to provide relevant CMake options for the modules you include to the Chrono build command. For instance, enabling the `Chrono::Vehicle` module would require setting setting the `CH_ENABLE_MODULE_VEHICLE` option to `ON`, as shown below:

```
ENV CMAKE_OPTIONS="${CMAKE_OPTIONS} -DCH_ENABLE_MODULE_VEHICLE=ON"
```

## Building the Docker Image

From anywhere in the Chrono repository, run the following command:

```bash
docker compose -f contrib/docker/docker-compose.yml build
```

This will build both the `dev` and `vnc` services. The `dev` service will build and install Chrono globally within the image. The default chrono build directory is at `/home/chrono/chrono` and the default install directory is at `/home/chrono/packages/chrono`. This can be changed with the `CHRONO_DIR` and `CHRONO_INSTALL_DIR` variables, respectively. By default, demos and testing are disabled to speed up the build process. 

By default, the following modules are enabled:

- PyChrono
- Chrono::Vehicle
- Chrono::Irrlicht
- Chrono::Parser
- Chrono::VSG
- Chrono::Sensor
- Chrono::ROS

<div class="ce-warning">
This may take some time, depending on your system and the modules you have included. For speed reasons, you may want to comment out some modules you don't need.
</div>

## Running the Docker Container

The intention of the `dev` service is to attach to the container within a shell. To do this, run the following command:

```bash
docker compose -f contrib/docker/docker-compose.yml run dev
```

By default, the initial directory is `/home/chrono/chrono-dev`, where that directory is a volume to the host's `chrono` directory. This means you can build template projects using the chrono build. You can add additional volumes, as needed.

### Visualizing GUI Applications

By default, the `/tmp/.X11-unix` directory is mounted to the container, which allows for GUI applications to be displayed on the host machine. If this isn't available to you for some reason, you can use the `vnc` service to visualize the container. This uses `NoVNC` to display the container's desktop in a browser. By default, the VNC server will be run on any port between `8080`. You can then navigate to [http://localhost:8080](http://localhost:8080) to view the desktop. If you are on a remote machine, ensure you forward the port to your local machine.

<div class="ce-info">
The `docker-compose.yml` file actually sets the ports to `8080-8099`, so if `8080` is in use, it will try the next available port. You can run `docker ps` to see which port is being used if things aren't working.
</div>

## Additional Notes

### Building the Docker Image without a NVIDIA GPU

As noted above, the default `docker-compose.yml` file will attach a NVIDIA GPU to the container if available. If you don't have a NVIDIA GPU, you can comment out the parts which follow `deploy` in the `docker-compose.yml` file.

### Installing Chrono::Sensor

To install Chrono::Sensor, you need support for CUDA, have a NVIDIA graphics card, and have an OptiX license and build script locally. If you have a NVIDIA graphics card, ensure the `cuda.dockerfile` is included _before_ `ch_sensor.dockerfile`. You can then download the [OptiX 7.7 installation script](https://developer.nvidia.com/designworks/optix/downloads/legacy) and place it at `contrib/docker/data`.