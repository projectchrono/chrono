#=============================================================================
# CHRONO::ENGINE   CMake configuration file for MPI unit
#
# Cannot be used stand-alone (it's loaded by CMake config. file in parent dir.)
#=============================================================================


#-----------------------------------------------------------------------------
#
# LIST THE FILES THAT MAKE THE GPU LIBRARY
# NOTE: to add a new source to this unit, just add its name
# here and re-run the CMake.
#

SET(ChronoMPI_SOURCES
    ChMpi.cpp
    ChLcpSystemDescriptorMPI.cpp
    ChLcpIterativeSchwarzMPI.cpp
    ChLcpSolverDEMMPI.cpp
    ChBodyDEMMPI.cpp
    ChSystemMPI.cpp
    ChBodyMPI.cpp
    ChDomainLatticePartitioning.cpp
    ChDomainGridPartitioning.cpp
    ChAssemblyMPI.cpp
    ChContactContainerDEMMPI.cpp
    )
SET(ChronoMPI_HEADERS
    ChApiMPI.h
    ChDomainNodeMPI.h
    ChMpi.h
    ChLcpSystemDescriptorMPI.h
    ChLcpIterativeSchwarzMPI.h
    ChLcpSolverDEMMPI.h
    ChBodyDEMMPI.h
    ChSystemMPI.h
    ChBodyMPI.h
    ChDomainLatticePartitioning.h
    ChDomainGridPartitioning.h
    ChAssemblyMPI.h
    ChContactContainerDEMMPI.h
    )
#SET_SOURCE_FILES_PROPERTIES(ChronoMPI_HEADERS PROPERTIES  HEADER_FILE_ONLY)
SOURCE_GROUP(unit_MPI FILES
    ${ChronoMPI_SOURCES}
    ${ChronoMPI_HEADERS})



#-----------------------------------------------------------------------------
# In most cases, you do not need to edit the lines below.

FIND_PACKAGE("MPI")

IF(NOT MPI_FOUND)
    MESSAGE("MPI not found. Please manually set MPI_LIBRARIES and MPI_INCLUDE_PATH")
    # MARK_AS_ADVANCED(CLEAR MPI_LIBRARY MPI_INCLUDE_PATH)
ENDIF()

IF ("${CH_OS}" STREQUAL "WINDOWS")
    SET (CH_JAVASCRIPTFLAGS "${CH_JAVASCRIPTFLAGS} -DXP_WIN")
    SET (CH_JAVASCRIPTINC   "${CH_JAVASRC}/src")
ENDIF()
IF ("${CH_OS}" STREQUAL "LINUX")
    MESSAGE( "WARNING!, MPI building under LINUX is not yet tested! Check libs paths etc!")
ENDIF()


SET (CH_CHRONOMPILIBRARYNAME "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_MPI${CH_LIBNAMEPOSTFIX}")
SET (CH_CHRONOMPISHAREDLNAME "${CH_BINDIR}/${CH_LIBNAME}_MPI${CH_LIBNAMEPOSTFIXSHARED}")
SET (CH_CHRONOMPILIB  "${CH_CHRONOMPILIBRARYNAME}")
SET (CH_CHRONOMPILIBRARYNAME_DBG "${CH_LIBDIR}/${CH_LIBNAMEPREFIX}${CH_LIBNAME}_MPIDBG${CH_LIBNAMEPOSTFIX}")
SET (CH_CHRONOMPISHAREDLNAME_DBG "${CH_BINDIR}/${CH_LIBNAME}_MPIDBG${CH_LIBNAMEPOSTFIXSHARED}")
SET (CH_CHRONOMPILIB_DBG  "${CH_CHRONOMPILIBRARYNAME_DBG}")


INCLUDE_DIRECTORIES( ${MPI_INCLUDE_PATH} )


# The MPI library is added to the project,
# and some custom properties of this target are set.

ADD_LIBRARY(chronompi SHARED
    ${ChronoMPI_SOURCES}
    ${ChronoMPI_HEADERS})

#SET_TARGET_PROPERTIES(chronompi PROPERTIES
#    LINK_FLAGS "${CH_LINKERFLAG_SHARED}"
#    COMPILE_DEFINITIONS "CH_API_COMPILE_UNIT_MPI")
#
#TARGET_LINK_LIBRARIES(chronompi
#    ${MPI_LIBRARIES}
#    ChronoEngine  )

SET_TARGET_PROPERTIES(chronompi PROPERTIES
    PUBLIC_HEADER ${ChronoMPI_HEADERS})


INSTALL(TARGETS chronompi
    EXPORT ChronoMPITargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT lib
    PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/chrono/mpi" COMPONENT dev
    )

