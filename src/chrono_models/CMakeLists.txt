message(STATUS "\n==== Chrono models ====\n")

add_subdirectory(robot)

if(CH_ENABLE_MODULE_VEHICLE)
  option(CH_ENABLE_MODULE_VEHICLE_MODELS "Build the Chrono vehicle models library" TRUE)
  mark_as_advanced(FORCE CH_ENABLE_MODULE_VEHICLE_MODELS)
  if(CH_ENABLE_MODULE_VEHICLE_MODELS)
  	add_subdirectory(vehicle)
  endif()

  set(CH_ENABLE_MODULE_VEHICLE_MODELS  "${CH_ENABLE_MODULE_VEHICLE_MODELS}" PARENT_SCOPE)  
endif()

install(FILES ChApiModels.h DESTINATION include/chrono_models)
