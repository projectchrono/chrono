message(STATUS "\n==== Chrono models ====\n")

add_subdirectory(robot)

if(ENABLE_MODULE_VEHICLE)
  option(ENABLE_MODULE_VEHICLE_MODELS "Build the Chrono vehicle models library" TRUE)
  mark_as_advanced(FORCE ENABLE_MODULE_VEHICLE_MODELS)
  if(ENABLE_MODULE_VEHICLE_MODELS)
  	add_subdirectory(vehicle)
  endif()

  set(ENABLE_MODULE_VEHICLE_MODELS  "${ENABLE_MODULE_VEHICLE_MODELS}" PARENT_SCOPE)  
endif()

install(FILES ChApiModels.h DESTINATION include/chrono_models)
