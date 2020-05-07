Chrono Docker Image
===================

The included `Dockerfile` can be used to build a release image for Chrono.
See the contents of the Dockerfile for instructions on preparing build assets.

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
