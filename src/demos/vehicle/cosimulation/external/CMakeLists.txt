set(PROGRAM demo_VEH_Cosim_WheelRig_CustomTerrain)

add_executable(${PROGRAM} ${PROGRAM}.cpp)
source_group("" FILES ${PROGRAM}.cpp)

target_link_libraries(${PROGRAM} Chrono_vehicle_cosim)
if(MSVC)
  set_target_properties(${PROGRAM} PROPERTIES MSVC_RUNTIME_LIBRARY ${CH_MSVC_RUNTIME_LIBRARY})
endif()
if(CH_ENABLE_MODULE_IRRLICHT)
  target_link_libraries(${PROGRAM} Chrono_irrlicht)
endif()