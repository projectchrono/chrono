message(STATUS "\n==== Chrono models ====\n")

option(ENABLE_MODULE_MODELS_ROBOT "Build the Chrono robot models library" TRUE)
mark_as_advanced(FORCE ENABLE_MODULE_MODELS_ROBOT)
if(ENABLE_MODULE_MODELS_ROBOT)
  add_subdirectory(robot)
endif()

if(ENABLE_MODULE_VEHICLE)
  option(ENABLE_MODULE_MODELS_VEHICLE "Build the Chrono vehicle models library" TRUE)
  mark_as_advanced(FORCE ENABLE_MODULE_MODELS_VEHICLE)
  if(ENABLE_MODULE_MODELS_VEHICLE)
  	add_subdirectory(vehicle)
  endif()
endif()

install(FILES ChApiModels.h DESTINATION include/chrono_models)
