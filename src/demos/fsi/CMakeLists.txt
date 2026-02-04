if(CH_ENABLE_MODULE_FSI_SPH)
  add_subdirectory(sph)
endif()

if(CH_ENABLE_MODULE_FSI_TDPF)
  add_subdirectory(tdpf)
endif()
