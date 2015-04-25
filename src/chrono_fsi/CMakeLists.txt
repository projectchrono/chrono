cmake_minimum_required(VERSION 2.8)
project(demo_FSI_Chrono)


#set path for local find modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

find_package(ChronoEngine   COMPONENTS unit_IRRLICHT )
OPTION(BUILD_OPENGL "Build the ChronoEngine_OpenGL library" off)
if (${BUILD_OPENGL})
	find_package(ChronoEngine   COMPONENTS unit_OPENGL )
endif()
OPTION(ENABLE_DEMOS_VEHICLE "Build the demos using ChronoVehicle" OFF)
IF(ENABLE_DEMOS_VEHICLE)
    set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmakeVehicle")
    FIND_PACKAGE(ChronoVehicle)
ENDIF()

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

# for some reason, adding ChronoEngine lib does not seem to be necessary. Maybe because it is only called by ChronoEngine_Parallel
#FIND_LIBRARY(CHRONO_LIB NAMES ChronoEngine PATHS ${CHRONO_LIB_PATH} REQUIRED)  
#FIND_LIBRARY(CHRONO_LIB_IRRLICHT NAMES ChronoEngine_IRRLICHT PATHS ${CHRONO_LIB_PATH} REQUIRED)
FIND_LIBRARY(CHRONO_LIB_PARALLEL NAMES ChronoEngine_Parallel PATHS ${CHRONO_PARALLEL_LIB_PATH} REQUIRED)
FIND_LIBRARY(CHRONO_LIB_UTILS NAMES ChronoEngine_Utils PATHS ${CHRONO_PARALLEL_LIB_PATH} REQUIRED)

#SET(LIBRARIES ${CHRONO_LIB} ${CHRONOENGINE_LIBRARIES} ${CHRONO_LIB_PARALLEL} ${CHRONO_LIB_UTILS})
SET(LIBRARIES ${CHRONOENGINE_LIBRARIES} ${CHRONO_LIB_PARALLEL} ${CHRONO_LIB_UTILS} ${CHRONOVEHICLE_LIBRARIES})


MESSAGE( STATUS "CHRONOENGINE_INCLUDES:         " ${CHRONOENGINE_INCLUDES} )

include_directories(
    ${CHRONOENGINE_INCLUDES}
    ${CHRONO_PARALLEL_INC}
    ${CHRONOVEHICLE_INCLUDES}
    "${CMAKE_BINARY_DIR}/demos_vehicle"
    "${CHRONO_PARALLEL_LIB_PATH}/../" 
    "${CUDA_TOOLKIT_ROOT_DIR}/include"
    "${CUDA_SDK_ROOT_DIR}/common/inc"
    ${GLFW_INCLUDE_DIR}
    )

#    ${GLFW_INCLUDE_DIR}
# question: enabling the following line causes error. The reason is probably because the options are not good for NVCC.
# they are good for gcc. Is there anyway to have those options available in cpp file (for instance having openmp in cpp
# file?
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")

cuda_add_library(FSI SHARED SphInterface.cpp InitializeSphMarkers.cpp collideSphereSphere.cu SDKCollisionSystem.cu printToFile.cu checkPointReduced.cpp VehicleExtraProperties.cpp)
SET(LIBRARIES "${LIBRARIES}" FSI)

# Add executables
#cuda_add_executable(cudaSPHProjEXE main.cpp SphInterface.cpp InitializeSphMarkers.cpp collideSphereSphere.cu SDKCollisionSystem.cu printToFile.cu)
#cuda_add_executable(cudaSPHProjEXE hmmwv_fluid.cpp)
#TARGET_LINK_LIBRARIES(cudaSPHProjEXE ${LIBRARIES})
#ADD_DEPENDENCIES(cudaSPHProjEXE ${LIBRARIES})
#INSTALL(TARGETS cudaSPHProjEXE DESTINATION bin)





# ================================================================================
# non-vehicle builds
SET(DEMOS
    ballDrop
    #hmmwv_fluid2
)

MESSAGE(STATUS "FSI demo programs...")

FOREACH(PROGRAM ${DEMOS})
    MESSAGE(STATUS "...add ${PROGRAM}")

    cuda_add_executable(${PROGRAM}  "${PROGRAM}.cpp")
    #SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

ENDFOREACH(PROGRAM)
# ================================================================================
# vehicle builds
IF(ENABLE_DEMOS_VEHICLE)
	SET(DEMOS
	    hmmwv
	    fsi_hmmwv
	)

	MESSAGE(STATUS "FSI demo programs...")

	FOREACH(PROGRAM ${DEMOS})
	    MESSAGE(STATUS "...add ${PROGRAM}")

	    cuda_add_executable(${PROGRAM}  "${PROGRAM}.cpp")
	    #SOURCE_GROUP(""  FILES "${PROGRAM}.cpp")

	    TARGET_LINK_LIBRARIES(${PROGRAM} ${LIBRARIES})
	    ADD_DEPENDENCIES(${PROGRAM} ${LIBRARIES})

	    INSTALL(TARGETS ${PROGRAM} DESTINATION bin)

	ENDFOREACH(PROGRAM)
ENDIF()

