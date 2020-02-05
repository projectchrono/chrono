message(STATUS "==== Chrono models ====")

if(ENABLE_MODULE_VEHICLE)
  	add_subdirectory(vehicle)
endif()

install(FILES ChApiModels.h DESTINATION include/chrono_models)

