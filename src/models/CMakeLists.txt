message(STATUS "==== Chrono models ====")

if(ENABLE_MODULE_VEHICLE)
  	add_subdirectory(vehicle)
endif()

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/
        DESTINATION include/models
        FILES_MATCHING PATTERN "*.h")
