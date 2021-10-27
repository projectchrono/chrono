#--------------------------------------------------------------
# OS-specific settings

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for ROBOT models...")

ADD_SUBDIRECTORY(robosimian)
ADD_SUBDIRECTORY(viper)
ADD_SUBDIRECTORY(copters)
ADD_SUBDIRECTORY(curiosity)
ADD_SUBDIRECTORY(turtlebot)