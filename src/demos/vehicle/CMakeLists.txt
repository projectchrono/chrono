#--------------------------------------------------------------
# OS-specific settings

IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  SET (CH_LINKERFLAG_EXE  "${CH_LINKERFLAG_EXE} -framework IOKit -framework Cocoa -framework OpenGL")
ENDIF()

#--------------------------------------------------------------

MESSAGE(STATUS "Demo programs for VEHICLE module...")

add_subdirectory(tracked_models)
add_subdirectory(wheeled_models)
add_subdirectory(test_rigs)

add_subdirectory(terrain)
add_subdirectory(controllers)

add_subdirectory(external_driver)

add_subdirectory(multicore)
add_subdirectory(articulated_vehicle)
add_subdirectory(two_cars)
add_subdirectory(ISO2631)
add_subdirectory(paths)
add_subdirectory(visualization)

add_subdirectory(cosimulation)
