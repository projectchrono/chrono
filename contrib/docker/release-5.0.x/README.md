Chrono Docker Image
===================

The included `Dockerfile` can be used to build a release image for Chrono.
See the contents of the Dockerfile for instructions on preparing build assets.

### Instructions

To build the image, run the following from this directory:
```sh
docker build .
```

To publish a release, you can use the following:
```sh
docker tag <image ID> uwsbel/projectchrono:latest
docker tag <image ID> uwsbel/projectchrono:X.Y.Z
docker push uwsbel/projectchrono
```
Replace X.Y.Z with the corresponding Project Chrono release version.


#### CUDA Support

An image with CUDA support can be built from `Dockerfile.CUDA` with just a few
alterations to the instructions above.

To build the image:
```sh
docker build -f Dockerfile.CUDA .
```

To publish a release:
```sh
docker tag <image ID> uwsbel/projectchrono:X.Y.Z-cudaM.N
docker push uwsbel/projectchrono
```
Replace M.N with the Cuda version from the nvidia/cuda base image.

