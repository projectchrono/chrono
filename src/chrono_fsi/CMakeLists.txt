cmake_minimum_required (VERSION 2.8)

project (cudaSPHProj)

find_package("CUDA")
cuda_include_directories(${CUDA_SDK_ROOT_DIR}/common/inc)
#cuda_include_directories(/opt/cuda/4.1/cuda/include)
include_directories(${CUDA_SDK_ROOT_DIR}/common/inc)
cuda_include_directories(${CUDA_SDK_ROOT_DIR}/../shared/inc)

set(CUDA_SDK_LIB_DIR ${CUDA_SDK_ROOT_DIR}/common/lib ${CUDA_SDK_ROOT_DIR}/lib ${CUDA_SDK_ROOT_DIR}/../shared/lib)

# Detect what architecture we're on
# Hopefully this works...
# C equivalent: 8*sizeof(void*)
MATH (EXPR arch 8*${CMAKE_SIZEOF_VOID_P})

# Build kernels for desired compute capabilities
macro(set_compute_capability cc)
	list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=sm_${cc}")
	list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=compute_${cc}")
endmacro(set_compute_capability)

set_compute_capability("20")


# now for the fun stuff
# You should only have to edit these lines.
# This executable needs cutil, so only compile and link it if it was found
#### Library Thing
#cuda_add_library(FSI SHARED collideSphereSphere.cu SDKCollisionSystem.cu  printToFile.cu)
#cuda_add_executable(cudaSPHProjEXE main.cpp)
#target_link_libraries(cudaSPHProjEXE FSI)
####
cuda_add_executable(cudaSPHProjEXE main.cpp collideSphereSphere.cu SDKCollisionSystem.cu printToFile.cu)

