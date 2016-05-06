message(STATUS "==== Chrono models ====")

if(ENABLE_MODULE_VEHICLE)
	option(BUILD_MODELS_VEHICLE "Build demo programs for Vehicle module" TRUE)
	mark_as_advanced(FORCE BUILD_MODELS_VEHICLE)
	if(BUILD_MODELS_VEHICLE)
  		add_subdirectory(vehicle)
  	endif()
endif()
