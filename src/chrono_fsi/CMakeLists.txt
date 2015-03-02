cmake_minimum_required(VERSION 2.8)
project(demo_Chrono_Parallel)


#set path for local find modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
find_package(ChronoEngine   COMPONENTS unit_IRRLICHT )
OPTION(BUILD_OPENGL "Build the ChronoEngine_OpenGL library" on)
if (${BUILD_OPENGL})
	find_package(ChronoEngine   COMPONENTS unit_OPENGL )
endif()

include(cmake/SBELUtils.cmake)
enable_openmp_support()
enable_cuda_support()

# User specified path to ChronoEngine
SET(CHRONO_INC           ""    CACHE   PATH    "Where is ChronoEngine Source located")
SET(CHRONO_PARALLEL_INC  ""    CACHE   PATH    "Where is ChronoParallel Source located")

# User specified path to ChronoEngine Libraries
SET(CHRONO_LIB_PATH           ""   CACHE    PATH    "Where is ChronoEngine Lib located")
SET(CHRONO_PARALLEL_LIB_PATH  ""   CACHE    PATH    "Where is ChronoParallel lib located")

# Find required Libraries
FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)
FIND_LIBRARY(CHRONO_LIB_IRRLICHT NAMES ChronoEngine_IRRLICHT PATHS ${CHRONO_LIB_PATH} REQUIRED)
FIND_LIBRARY(CHRONO_LIB_PARALLEL NAMES ChronoEngine_Parallel PATHS ${CHRONO_PARALLEL_LIB_PATH} REQUIRED)
FIND_LIBRARY(CHRONO_LIB_UTILS NAMES ChronoEngine_Utils PATHS ${CHRONO_PARALLEL_LIB_PATH} REQUIRED)

SET(LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONO_LIB_PARALLEL} ${CHRONO_LIB_UTILS})


MESSAGE( STATUS "CHRONOENGINE_INCLUDES:         " ${CHRONOENGINE_INCLUDES} )

include_directories(
    ${CHRONOENGINE_INCLUDES}
    ${CHRONO_PARALLEL_INC}
    ${GLFW_INCLUDE_DIR}
    "${CHRONO_PARALLEL_LIB_PATH}/../" 
    "${CUDA_TOOLKIT_ROOT_DIR}/include"
    "${CUDA_SDK_ROOT_DIR}/common/inc"
    )

# question: enabling the following line causes error. The reason is probably because the options are not good for NVCC.
# they are good for gcc. Is there anyway to have those options available in cpp file (for instance having openmp in cpp
# file?
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
#cuda_add_library(FSI SHARED collideSphereSphere.cu SDKCollisionSystem.cu  printToFile.cu)
#SET(LIBRARIES "${LIBRARIES} FSI")

# Add executables
cuda_add_executable(cudaSPHProjEXE main.cpp SphInterface.cpp InitializeSphMarkers.cpp collideSphereSphere.cu SDKCollisionSystem.cu printToFile.cu)
#ADD_EXECUTABLE(cudaSPHProjEXE main.cpp)
TARGET_LINK_LIBRARIES(cudaSPHProjEXE ${LIBRARIES})
ADD_DEPENDENCIES(cudaSPHProjEXE ${LIBRARIES})
INSTALL(TARGETS cudaSPHProjEXE DESTINATION bin)




