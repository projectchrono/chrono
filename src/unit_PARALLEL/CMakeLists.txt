#=============================================================================
# CHRONO::ENGINE   CMake configuration file for GPU unit
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


SET(ENABLE_UNIT_GPU      FALSE    CACHE BOOL   "Turn ON this to generate the Chrono::Engine GPU unit.")

IF(ENABLE_UNIT_GPU)

MARK_AS_ADVANCED(FORCE CUDA_BUILD_CUBIN)
MARK_AS_ADVANCED(FORCE CUDA_VERBOSE_BUILD)
MARK_AS_ADVANCED(FORCE CUDA_FAST_MATH)
MARK_AS_ADVANCED(FORCE CUDA_VERBOSE_PTX)
MARK_AS_ADVANCED(FORCE CUDA_DEVICE_VERSION)
MARK_AS_ADVANCED(FORCE CUDA_BUILD_EMULATION)
MARK_AS_ADVANCED(FORCE CH_CUDART)
MARK_AS_ADVANCED(FORCE CUDA_SM_VERSION)

SET(CUDA_DEVICE_VERSION "20" CACHE STRING "CUDA Device Version")
SET_PROPERTY(CACHE CUDA_DEVICE_VERSION PROPERTY STRINGS "10" "11" "12" "13" "20" "21" "30" "35")

SET(CUDA_SM_VERSION "21" CACHE STRING "CUDA SM Version")
SET_PROPERTY(CACHE CUDA_SM_VERSION PROPERTY STRINGS "10" "11" "12" "13" "20" "21" "30" "35")

SET(CUDA_FAST_MATH TRUE CACHE BOOL "Use Fast Math Operations")
SET(CUDA_VERBOSE_PTX FALSE CACHE BOOL "Show Verbose Kernel Info During Compilation")

#
# LIST THE FILES THAT MAKE THE GPU LIBRARY
# NOTE: to add a new source to this unit, just add its name here and re-run the CMake.
#

SET(ChronoEngine_unit_GPU_COLLISION
    collision/ChCAABBGenerator.cuh
    collision/ChCBroadphase.cuh
    collision/ChCNarrowphase.cuh
    collision/ChCAABBGenerator.cu
    collision/ChCBroadphase.cu
    collision/ChCNarrowphase.cu

    collision/ChCSupportFunctions.cuh
    collision/ChCCollisionModelGPU.h
    collision/ChCCollisionModelGPU.cpp
    collision/ChCCollisionSystemGPU.h
    collision/ChCCollisionSystemGPU.cpp
    collision/ChCCollisionSystemBulletGPU.h
    collision/ChCCollisionSystemBulletGPU.cpp
    collision/ChContactContainerGPU.h
    collision/ChContactContainerGPU.cpp
   )
SET(ChronoEngine_unit_GPU_SOLVER
    solver/ChSolverGPU.cuh
    solver/ChSolverGPU.cu
    #solver/ChSolverBlockJacobi.h
    solver/ChSolverBlockJacobi.cu
    solver/ChSolverCG.cu
    solver/ChSolverCGS.cu
    solver/ChSolverGD.cu
    solver/ChSolverSD.cu
    solver/ChSolverAPGD.cu
    solver/ChSolverMinRes.cu
    solver/ChSolverBiCG.cu
    solver/ChSolverBiCGStab.cu
   )



SET(ChronoEngine_unit_GPU_SOURCES
    #ChBodyGPU.cpp
    ChSystemGPU.cpp
   )

SET(ChronoEngine_unit_GPU_HEADERS
    ChApiGPU.h
    ChBaseGPU.h
    ChCudaDefines.h
    ChCudaMath.h
    ChThrustLinearAlgebra.cuh
    #ChBodyGPU.h
    ChSystemGPU.h
    ChDataManager.h
    ChLcpSolverGPU.h
    ChLcpSystemDescriptorGPU.h
    ChJacobianGPU.h
    ChComputeRHSGPU.h
    #ChIntegratorGPU.h

   )
SET(ChronoEngine_unit_GPU_CU
    ChLcpSolverGPU.cu
    ChBaseGPU.cu
    ChDataManager.cu
    ChJacobianGPU.cu
    ChComputeRHSGPU.cu
	#ChIntegratorGPU.cu
   )

SOURCE_GROUP(unit_GPU FILES
             ${ChronoEngine_unit_GPU_SOURCES}
             ${ChronoEngine_unit_GPU_HEADERS}
             ${ChronoEngine_unit_GPU_CU})
SOURCE_GROUP(unit_GPU\\collision FILES
             ${ChronoEngine_unit_GPU_COLLISION}
            )
SOURCE_GROUP(unit_GPU\\solver FILES
             ${ChronoEngine_unit_GPU_SOLVER}
            )
#-----------------------------------------------------------------------------
# In most cases, you do not need to edit the lines below.

# Use the FindCUDA.cmake to find the CUDA toolkit and set many interesting 
# variables and compilation rules. This saves eons of time.
# Interesting vars that are set: paths CUDA_SDK_ROOT_DIR and CUDA_TOOLKIT_ROOT_DIR

find_package(CUDA QUIET)

if (CUDA_FOUND)
    message(STATUS "CUDA found")
    else()
        message(FATAL_ERROR "CUDA not found! (you may switch off ENABLE_UNIT_GPU and build without this unit)")
        endif()

# Detect what architecture we're on
# Paths we'll use to find some of the extra libraries
# Ensure that correct path for architecture is selected
        IF(CMAKE_SIZEOF_VOID_P EQUAL 8)
        SET(CUDA_TOOLKIT_LIB_DIR
            ${CUDA_TOOLKIT_ROOT_DIR}/lib/x64
            ${CUDA_TOOLKIT_ROOT_DIR}/lib
            ${CUDA_TOOLKIT_ROOT_DIR}/lib64)
        SET(CUDA_SDK_LIB_DIR
            ${CUDA_SDK_ROOT_DIR}/common/lib/x64
            ${CUDA_SDK_ROOT_DIR}/common/lib/linux/x86_64
            ${CUDA_SDK_ROOT_DIR}/common/lib/darwin)
        ELSE()
        message(FATAL_ERROR "Architecture detected as 32-bit. Only 64-bit Builds Supported!")
        ENDIF()

        find_library(CH_CUDART NAMES cudart libcudart PATHS ${CUDA_TOOLKIT_LIB_DIR})

        SET(CUDA_NVCC_FLAGS "-gencode=arch=compute_${CUDA_DEVICE_VERSION},code=sm_${CUDA_SM_VERSION}" CACHE  STRING "" FORCE )
        SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-DCH_API_COMPILE_UNIT_GPU" )
#SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--disable-warnings")
        IF(CUDA_FAST_MATH)
        SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "-use_fast_math")
        ENDIF()
        IF(CUDA_VERBOSE_PTX)
        SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ptxas-options=-v")
        ENDIF()

        SET (CH_CUDA_INC     	"${CUDA_TOOLKIT_ROOT_DIR}/include" )
        SET (CH_CUDA_SDK_INC 	"${CUDA_SDK_ROOT_DIR}/common/inc"  )

# These need to get straightened out
# Anything using the SDK should only be used by nvcc, no need to include them for anything being sent to the system compiler
        CUDA_INCLUDE_DIRECTORIES(${CH_CUDA_INC} ${CH_CUDA_SDK_INC} "../")
        INCLUDE_DIRECTORIES( ${CH_CUDA_INC} ${CH_CUDA_SDK_INC} "../")

# The GPU library is added to the project, and some custom properties of this target are set.

        CUDA_ADD_LIBRARY(ChronoEngine_GPU SHARED
                         ${ChronoEngine_unit_GPU_COLLISION}
                         ${ChronoEngine_unit_GPU_SOLVER}
                         ${ChronoEngine_unit_GPU_SOURCES}
                         ${ChronoEngine_unit_GPU_HEADERS}
                         ${ChronoEngine_unit_GPU_CU})


        IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
        SET(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -DWIN32_LEAN_AND_MEAN")
        SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DWIN32_LEAN_AND_MEAN")
        ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")

        ELSEIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
        SET_TARGET_PROPERTIES( ChronoEngine_GPU PROPERTIES XCODE_ATTRIBUTE_ENABLE_OPENMP_SUPPORT YES )
        FIND_LIBRARY(CUDA_FRAMEWORK CUDA)
        ENDIF()


        IF(${CMAKE_CONFIGURATION_TYPES} MATCHES "Debug")

        ENDIF()


        SET_TARGET_PROPERTIES(ChronoEngine_GPU PROPERTIES
                              LINK_FLAGS "${CH_LINKERFLAG_GPU}"
                              COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_GPU")

# Let some variables be visible also from outside this directory, using the PARENT_SCOPE trick

        TARGET_LINK_LIBRARIES(ChronoEngine_GPU ChronoEngine ${CUDA_FRAMEWORK})
        ADD_DEPENDENCIES (ChronoEngine_GPU ChronoEngine)  # better, because not automatic

        SET (CH_CUDA_INC      		"${CUDA_TOOLKIT_ROOT_DIR}/include" 	PARENT_SCOPE )
        SET (CH_CUDA_SDK_INC  		"${CUDA_SDK_ROOT_DIR}/common/inc"  	PARENT_SCOPE )
        INSTALL(TARGETS ChronoEngine_GPU
                RUNTIME DESTINATION bin
                LIBRARY DESTINATION lib
                ARCHIVE DESTINATION lib
               )
        ELSE()

#MARK_AS_ADVANCED(FORCE CUDA_BUILD_CUBIN)
#MARK_AS_ADVANCED(FORCE CUDA_SDK_ROOT_DIR)
#MARK_AS_ADVANCED(FORCE CUDA_TOOLKIT_ROOT_DIR)
#MARK_AS_ADVANCED(FORCE CUDA_VERBOSE_BUILD)
#MARK_AS_ADVANCED(FORCE CUDA_FAST_MATH)
#MARK_AS_ADVANCED(FORCE CUDA_USE_CUSTOM_COMPILER)
#MARK_AS_ADVANCED(FORCE CUDA_VERBOSE_PTX)
#MARK_AS_ADVANCED(FORCE CUDA_DEVICE_VERSION)
#MARK_AS_ADVANCED(FORCE CUDA_BUILD_EMULATION)
#MARK_AS_ADVANCED(FORCE CH_CUDA_LIBSDKUTIL)
#MARK_AS_ADVANCED(FORCE CH_CUDART)
        ENDIF()



