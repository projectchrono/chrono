# syntax = devthefuture/dockerfile-x
# The INCLUDE directive is provided by the devthefuture/dockerfile-x project

# Will copy in the base configuration for the build
INCLUDE ./common/base.dockerfile

# Snippets
INCLUDE ./snippets/chrono.dockerfile

# Will copy in other common configurations for this build
INCLUDE ./common/common.dockerfile

# Complete the build
INCLUDE ./common/final.dockerfile
