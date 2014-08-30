#--------------------------------------------------------------
# Always use full RPATH (differentiating between the build and install trees)

# use, i.e. don't skip the full RPATH for the build tree
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 

SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
IF("${isSystemDir}" STREQUAL "-1")
   SET(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
ENDIF("${isSystemDir}" STREQUAL "-1")

#--------------------------------------------------------------

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------
# Copy the data folder to the bin directory

IF(MSVC)
  FILE( COPY ${CMAKE_SOURCE_DIR}/models/data/ DESTINATION ${CMAKE_BINARY_DIR}/bin/data/ PATTERN .svn EXCLUDE)
ELSEIF(XCODE_VERSION)
  FILE( COPY ${CMAKE_SOURCE_DIR}/models/data/ DESTINATION ${CMAKE_BINARY_DIR}/bin/data/ PATTERN .svn EXCLUDE)
ELSE()
  FILE( COPY ${CMAKE_SOURCE_DIR}/models/data/ DESTINATION ${CMAKE_BINARY_DIR}/data/ PATTERN .svn EXCLUDE)
ENDIF()

#--------------------------------------------------------------

ADD_SUBDIRECTORY(slider_crank)
ADD_SUBDIRECTORY(demo_HMMWV9)
ADD_SUBDIRECTORY(demo_HMMWV)
ADD_SUBDIRECTORY(demo_PacejkaTest)


