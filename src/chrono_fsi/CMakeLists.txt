cmake_minimum_required (VERSION 2.8)

project (cudaSPHProj)

#find_package("OpenMP")

#if(OPENMP_FOUND)
#	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#endif()

#find_package("MPI")

#find_package("OpenGL")
#find_package("GLUT")

find_package("CUDA")
cuda_include_directories(${CUDA_SDK_ROOT_DIR}/common/inc)
cuda_include_directories(/opt/cuda/4.1/cuda/include)
include_directories(${CUDA_SDK_ROOT_DIR}/common/inc)
cuda_include_directories(${CUDA_SDK_ROOT_DIR}/../shared/inc)

set(CUDA_SDK_LIB_DIR ${CUDA_SDK_ROOT_DIR}/common/lib ${CUDA_SDK_ROOT_DIR}/lib ${CUDA_SDK_ROOT_DIR}/../shared/lib)

# Detect what architecture we're on
# Hopefully this works...
# C equivalent: 8*sizeof(void*)
MATH (EXPR arch 8*${CMAKE_SIZEOF_VOID_P})

# Find path to shrutil libs, from CUDA SDK
find_library(LIBSHRUTIL NAMES 
	shrUtils${arch} shrutil_${CMAKE_SYSTEM_PROCESSOR} PATHS ${CUDA_SDK_LIB_DIR})
find_library(LIBSHRUTIL_DBG NAMES 
	shrUtils${arch}D shrutil_${CMAKE_SYSTEM_PROCESSOR}D PATHS ${CUDA_SDK_LIB_DIR})

## Find path to cutil libs, from CUDA SDK
#find_library(LIBCUTIL NAMES 
#	cutil${arch} cutil_${CMAKE_SYSTEM_PROCESSOR} PATHS ${CUDA_SDK_LIB_DIR})
#find_library(LIBCUTIL_DBG NAMES 
#	cutil${arch}D cutil_${CMAKE_SYSTEM_PROCESSOR}D PATHS ${CUDA_SDK_LIB_DIR})
#find_library(LIBCUDPP NAMES 
#	cudpp${arch} cudpp_${CMAKE_SYSTEM_PROCESSOR} PATHS ${CUDA_SDK_LIB_DIR} ${CUDA_SDK_LIB_DIR}/lib/linux/lib)


# Build kernels for desired compute capabilities
macro(set_compute_capability cc)
	list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=sm_${cc}")
	list(APPEND CUDA_NVCC_FLAGS "-gencode=arch=compute_${cc},code=compute_${cc}")
endmacro(set_compute_capability)

#set_compute_capability("11")
#set_compute_capability("13")
set_compute_capability("20")


# now for the fun stuff
# You should only have to edit these lines.
# This executable needs cutil, so only compile and link it if it was found
#### Library Thing
#cuda_add_library(FSI SHARED collideSphereSphere.cu SDKCollisionSystem.cu  printToFile.cu)
#cuda_add_executable(cudaSPHProjEXE main.cpp)
#target_link_libraries(cudaSPHProjEXE FSI)
####
cuda_add_executable(cudaSPHProjEXE main.cpp collideSphereSphere.cu SDKCollisionSystem.cu printToFile.cu FlexibleBodies.cu)
#IF (LIBCUTIL)
#	cuda_add_executable(cudaSPHProjEXE main.cpp collideSphereSphere.cu SDKCollisionSystem.cu)

#	target_link_libraries(cudaSPHProjEXE ${LIBCUTIL})
#ENDIF ()
