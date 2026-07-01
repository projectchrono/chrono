# FindScmGpu.cmake — locate pre-built scm_gpu_core (HIP contact-force library).
#
# Optional cache variables:
#   CHRONO_SCM_GPU_INCLUDE_DIR  (legacy; headers ship in-tree as SCMGpu.h)
#   CHRONO_SCM_GPU_LIB_DIR      directory containing libscm_gpu_core.so / .a

find_library(
    ScmGpu_CORE_LIBRARY
    NAMES scm_gpu_core
    HINTS
        ${CHRONO_SCM_GPU_LIB_DIR}
        ENV CHRONO_SCM_GPU_LIB_DIR
    PATH_SUFFIXES lib lib64
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ScmGpu DEFAULT_MSG ScmGpu_CORE_LIBRARY)

if(ScmGpu_FOUND)
    set(ScmGpu_LIBRARIES ${ScmGpu_CORE_LIBRARY})
endif()

mark_as_advanced(ScmGpu_CORE_LIBRARY)
