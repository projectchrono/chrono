# MUMPS 5.0.2 CMake file
# - all UPPER case "libraries" compile .obj files without linking
# WARNING: Some native build systems may not like targets that have only object files,
# so consider adding at least one real source file to any target that references $<TARGET_OBJECTS:objlib>

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(Mumps C Fortran)
SET(VERSION 5.0.2)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

#################### Mumps configuration ####################
# Arithmetic configuration
SET(ARITHMETIC "double" CACHE STRING "MUMPS arithmetic")
SET_PROPERTY(CACHE ARITHMETIC PROPERTY STRINGS float double complex complex-double)
IF (ARITHMETIC MATCHES "float")
	SET(ARITH_PREFIX "s")
ELSEIF (ARITHMETIC MATCHES "double")
	SET(ARITH_PREFIX "d")
ELSEIF (ARITHMETIC MATCHES "complex")
	SET(ARITH_PREFIX "c")
ELSEIF (ARITHMETIC MATCHES "complex-double")
	SET(ARITH_PREFIX "z")
ENDIF ()
MESSAGE(STATUS "Arithmetic set to '${ARITHMETIC}'. Added '${ARITH_PREFIX}' prefix...")


OPTION(BUILD_EXAMPLES "Build MUMPS examples" ON)
OPTION(ENABLE_OPENMP "Enable OpenMP multithreading." ON)
OPTION(BUILD_SHARED_LIBS "Build shared libraries." ON)


IF(CMAKE_SIZEOF_VOID_P MATCHES 4)
    SET(TARGET_SYSTEM "32")
ELSE()
    SET(TARGET_SYSTEM "64")
ENDIF()
MESSAGE(STATUS "Building ${TARGET_SYSTEM}bit target")

################# BLAS library ##########################
#BLAS configuration
SET(BLA_VENDOR "Intel10_64lp" CACHE STRING "BLAS library vendor")
SET_PROPERTY(CACHE BLA_VENDOR PROPERTY STRINGS  All Goto OpenBLAS ATLAS PhiPACK CXML DXML SunPerf SCSL SGIMATH IBMESSL
												Intel10_32 #(intel mkl v10 32 bit)
												Intel10_64lp #(intel mkl v10 64 bit lp thread model  lp64 model)
												Intel10_64lp_seq #(intel mkl v10 64 bit sequential code  lp64 model)
												Intel #( older versions of mkl 32 and 64 bit) 
												ACML ACML_MP ACML_GPU Apple NAS)

UNSET(BLAS_FOUND)
UNSET(BLAS_LIBRARIES)
SET(CMAKE_LIBRARY_PATH_BKP ${CMAKE_LIBRARY_PATH} )

# Automatic inclusion of MKL folders
if(CMAKE_SIZEOF_VOID_P MATCHES 4)
	SET(MKL_SUBFOLDERARCH "ia32")
elseif (CMAKE_SIZEOF_VOID_P MATCHES 8)
	SET(MKL_SUBFOLDERARCH "intel64")
endif()

if (WIN32)
	SET(PROGRAM_FILE_ENVVAR "PROGRAMFILES(x86)")
	FILE(TO_CMAKE_PATH "$ENV{${PROGRAM_FILE_ENVVAR}}" PRG_FOLD)
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/IntelSWTools/compilers_and_libraries/windows/mkl/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/IntelSWTools/compilers_and_libraries/windows/compiler/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later
	
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/Intel/Composer XE/mkl/lib/${MKL_SUBFOLDERARCH}") # default until ParallelStudioXE2015
	list(APPEND INTEL_LIBRARIES_DIRS "${PRG_FOLD}/Intel/Composer XE/compiler/lib/${MKL_SUBFOLDERARCH}") # default until ParallelStudioXE2015
elseif(UNIX AND NOT APPLE)
	list(APPEND INTEL_LIBRARIES_DIRS "/opt/intel/compilers_and_libraries/linux/mkl/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (root permissions)
	list(APPEND INTEL_LIBRARIES_DIRS "/opt/intel/compilers_and_libraries/linux/compiler/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (root permissions)
	
	list(APPEND INTEL_LIBRARIES_DIRS "$ENV{HOME}/intel/compilers_and_libraries/linux/mkl/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (no root permissions)
	list(APPEND INTEL_LIBRARIES_DIRS "$ENV{HOME}/intel/compilers_and_libraries/linux/compiler/lib/${MKL_SUBFOLDERARCH}") # default for ParallelStudioXE2016 and later (no root permissions)
endif()

SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} ${BLAS_LIBRARIES_USER} ${INTEL_LIBRARIES_DIRS} )


find_package(BLAS)
SET(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH_BKP}) # restore original CMAKE_LIBRARY_PATH

if (NOT BLAS_FOUND)
	SET(BLAS_LIBRARIES_USER "" CACHE PATH "Locate BLAS library directory") # let the user specifies its own BLAS library
	MARK_AS_ADVANCED(CLEAR BLAS_LIBRARIES_USER)
else()
	MESSAGE(STATUS "Linking BLAS library: ${BLAS_LIBRARIES}")
	MARK_AS_ADVANCED(FORCE BLAS_LIBRARIES_USER)
endif()

LINK_DIRECTORIES(${INTEL_LIBRARIES_DIRS})


#################### Compiler and linker flags ####################
# -O2 optimization flags are already set by generator
SET(GENERIC_COMPILER_FLAGS_FORTRAN #aka OPTF
			"-MD -Dintel_ -DALLOW_NON_INIT -fpp")

SET(GENERIC_COMPILER_FLAGS_C #aka OPTC
			"-MD")
			

find_package(OpenMP)
if (ENABLE_OPENMP AND OPENMP_FOUND)
	SET(GENERIC_COMPILER_FLAGS_FORTRAN "${GENERIC_COMPILER_FLAGS_FORTRAN} ${OpenMP_Fortran_FLAGS}")
	SET(GENERIC_COMPILER_FLAGS_C "${GENERIC_COMPILER_FLAGS_C} ${OpenMP_C_FLAGS}")
	
	SET(EXCLUDE_IOMP "")
	if (CMAKE_Fortran_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID STREQUAL "GNU")
		OPTION(LINK_INTEL_OMP "Let Intel compiler links its own Intel OpenMP library (iomp)" OFF)
		IF (NOT LINK_INTEL_OMP)
			SET(EXCLUDE_IOMP "-Wl,--as-needed")
		ENDIF()
	endif()
	
	IF (CMAKE_Fortran_COMPILER_ID STREQUAL "GNU")
		SET(GENERIC_COMPILER_FLAGS_FORTRAN "${GENERIC_COMPILER_FLAGS_FORTRAN} ${EXCLUDE_IOMP}")
	ENDIF()
	
	if (MSVC)
		SET(COMPILER_DEFS_C "${COMPILER_DEFS_C}" "_OPENMP_NOFORCE_MANIFEST")
		SET(COMPILER_DEFS_FORTRAN "${COMPILER_DEFS_FORTRAN}" "_OPENMP_NOFORCE_MANIFEST")
	endif()
	
endif()


SET(COMPILER_DEFS_C #aka CDEFS
			"Add_" "_CRT_SECURE_NO_WARNINGS")
SET(COMPILER_DEFS_FORTRAN #aka CDEF
			"Add_" "_CRT_SECURE_NO_WARNINGS")
			
SET(ORDERINGS_COMPILER_FLAGS_FORTRAN #aka ORDERINGSF
			"-Dpord")
SET(ORDERINGS_COMPILER_FLAGS_C #aka ORDERINGSC (modified)
			"-Dpord -DMUMPS_ARITH=MUMPS_ARITH_${ARITH_PREFIX}")
			
#################### Opt for shared libraries ####################
IF (BUILD_SHARED_LIBS)
	SET(COMPILER_DEFS_C "${COMPILER_DEFS_C}" "MUMPS_CALL=__declspec(dllexport)") # can be done also in mumps_c_compat.h?
	SET(COMPILER_DEFS_FORTRAN "${COMPILER_DEFS_FORTRAN}" "MUMPS_CALL=__declspec(dllexport)") # can be done also in mumps_c_compat.h?
	IF (CMAKE_C_COMPILER_ID STREQUAL "GNU")
		SET(GENERIC_COMPILER_FLAGS_FORTRAN "${GENERIC_COMPILER_FLAGS_FORTRAN}" "-fPIC" "DLLEXPORT")
		SET(GENERIC_COMPILER_FLAGS_FORTRAN "${GENERIC_COMPILER_FLAGS_FORTRAN}" "-fPIC")
	ENDIF()
ENDIF()
			
		
#################### Ordering libraries ####################
# Ordering libraries selection
SET(USE_METIS FALSE CACHE BOOL "Select if METIS ordering is used")

IF (USE_METIS)
	SET(METIS_DIR "${CMAKE_SOURCE_DIR}" CACHE PATH "Path to METIS folder")
	SET(ORDERINGS_COMPILER_FLAGS_C "${ORDERINGS_COMPILER_FLAGS_C} -Dmetis" )
	SET(ORDERINGS_COMPILER_FLAGS_FORTRAN "${ORDERINGS_COMPILER_FLAGS_FORTRAN} -Dmetis" )
	LIST(APPEND ORDERINGS_INCLUDE_DIRS "${METIS_DIR}/include")
	LIST(APPEND ORDERINGS_LIBRARIES "${METIS_DIR}/libmetis/$<CONFIG>/metis.lib") #aka LPORDDIR+LPORD
ENDIF (USE_METIS)

#######################################################
#################### TARGETS ##########################
#######################################################

############### PORD library ###############
SET(PORD_DIR "${CMAKE_SOURCE_DIR}/PORD")
SET(PORD_INCLUDE_DIRS "${PORD_DIR}/include")
SET(PORD_SOURCES
		"${PORD_DIR}/lib/bucket.c"
		"${PORD_DIR}/lib/ddbisect.c"
		"${PORD_DIR}/lib/ddcreate.c"
		"${PORD_DIR}/lib/gbipart.c"
		"${PORD_DIR}/lib/gbisect.c"
		"${PORD_DIR}/lib/gelim.c"
		"${PORD_DIR}/lib/graph.c"
		"${PORD_DIR}/lib/interface.c"
		"${PORD_DIR}/lib/minpriority.c"
		"${PORD_DIR}/lib/multisector.c"
		"${PORD_DIR}/lib/nestdiss.c"
		"${PORD_DIR}/lib/sort.c"
		"${PORD_DIR}/lib/symbfac.c"
		"${PORD_DIR}/lib/tree.c")
		
SET(PORD_HEAD
		"${PORD_INCLUDE_DIRS}/const.h"
		"${PORD_INCLUDE_DIRS}/eval.h"
		"${PORD_INCLUDE_DIRS}/macros.h"
		"${PORD_INCLUDE_DIRS}/params.h"
		"${PORD_INCLUDE_DIRS}/protos.h"
		"${PORD_INCLUDE_DIRS}/space.h"
		"${PORD_INCLUDE_DIRS}/types.h")

IF (NOT BUILD_SHARED_LIBS)
	ADD_LIBRARY(libpord STATIC
							${PORD_SOURCES}
							${PORD_HEAD})

	SET_TARGET_PROPERTIES(libpord PROPERTIES
							COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_C}"
							LINKER_LANGUAGE C)
							
	TARGET_INCLUDE_DIRECTORIES(libpord PUBLIC	"${PORD_INCLUDE_DIRS}")

	LIST(APPEND ORDERINGS_LIBRARIES libpord) #aka LORDERINGS #aka LPORDDIR+LPORD
ENDIF()


LIST(APPEND ORDERINGS_INCLUDE_DIRS "${PORD_INCLUDE_DIRS}")

############### LIBSEQ library ###############
SET(LIBSEQ_SRC_C
		"libseq/mpic.c"
		"libseq/elapse.c")

SET(LIBSEQ_SRC_FORTRAN
		"libseq/mpi.f")
		
SET(LIBSEQ_INCLUDE_DIRS
		"${CMAKE_SOURCE_DIR}/libseq") #aka INCSEQ	
	
SET(LIBSEQ_HEAD_C
		"${LIBSEQ_INCLUDE_DIRS}/mpi.h"
		"${LIBSEQ_INCLUDE_DIRS}/elapse.h")

SET(LIBSEQ_HEAD_FORTRAN
		"${LIBSEQ_INCLUDE_DIRS}/mpif.h")

# libseq object files (Fortran)
ADD_LIBRARY(LIBMPISEQ_FORTRAN OBJECT
						${LIBSEQ_SRC_FORTRAN}
						${LIBSEQ_HEAD_FORTRAN})

SET_TARGET_PROPERTIES(LIBMPISEQ_FORTRAN PROPERTIES
						COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_FORTRAN} -c"
						LINKER_LANGUAGE FORTRAN)

# libseq object files (C)
ADD_LIBRARY(LIBMPISEQ_C OBJECT
						${LIBSEQ_SRC_C}
						${LIBSEQ_HEAD_C})

SET_TARGET_PROPERTIES(LIBMPISEQ_C PROPERTIES
						COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_C} -c"
						COMPILE_DEFINITIONS "${COMPILER_DEFS_C}"
						LINKER_LANGUAGE C)

TARGET_INCLUDE_DIRECTORIES(LIBMPISEQ_C PUBLIC "${LIBSEQ_INCLUDE_DIRS}")

IF (NOT BUILD_SHARED_LIBS)
	# libseq library
	ADD_LIBRARY(libmpiseq STATIC
							${LIBSEQ_HEAD_C}
							${LIBSEQ_HEAD_FORTRAN}
							$<TARGET_OBJECTS:LIBMPISEQ_C>
							$<TARGET_OBJECTS:LIBMPISEQ_FORTRAN>)
ENDIF()


								
############### MUMPS library ###############
# Common MUMPS files

SET(MUMPS_INCLUDE_DIRS #aka INCS
			"${CMAKE_SOURCE_DIR}/include") #aka INCSEQ
			
SET(MUMPS_HEADERS # TODO never used. Actually they are only used by the external user to link the library
			"${MUMPS_INCLUDE_DIRS}/${ARITH_PREFIX}mumps_c.h"
			"${MUMPS_INCLUDE_DIRS}/${ARITH_PREFIX}mumps_root.h"
			"${MUMPS_INCLUDE_DIRS}/${ARITH_PREFIX}mumps_struc.h"
			"${MUMPS_INCLUDE_DIRS}/mumps_c_types.h"
			"${MUMPS_INCLUDE_DIRS}/mumps_compat.h")
			

SET (MUMPS_COMMON_SRC_FORTRAN
        src/ana_omp_m.F
		src/tools_common_mod.F
        src/mumps_static_mapping.F
        src/mumps_sol_es.F
		src/fac_future_niv2_mod.F
        src/mumps_comm_ibcast.F
		src/mumps_ooc_common.F
        src/double_linked_list.F
        src/fac_asm_build_sort_index_m.F
        src/fac_asm_build_sort_index_ELT_m.F
        src/omp_tps_common_m.F
        src/mumps_l0_omp_m.F
        src/front_data_mgt_m.F
        src/fac_maprow_data_m.F
        src/fac_descband_data_m.F
        src/fac_ibct_data_m.F
	)
		
SET (MUMPS_COMMON_OTHER_SRC_FORTRAN
        src/ana_orderings.F
		src/ana_AMDMF.F
        src/bcast_errors.F
        src/estim_flops.F
        src/mumps_type_size.F 
        src/mumps_type2_blocking.F 
        src/mumps_version.F 
        src/tools_common.F 
        src/mumps_print_defined.F 
		src/ana_set_ordering.F
	)

SET (MUMPS_COMMON_OTHER_SRC_C
		src/mumps_c.c
        src/mumps_common.c
        src/mumps_orderings.c
        src/mumps_size.c
        src/mumps_io.c
        src/mumps_io_basic.c
        src/mumps_io_thread.c
        src/mumps_io_err.c
        src/mumps_numa.c
	)
		
SET (MUMPS_COMMON_OTHER_HEAD
		src/mumps_common.h
		src/mumps_headers.h
		src/mumps_io.h
		src/mumps_io_basic.h
		src/mumps_io_err.h
		src/mumps_io_thread.h
		src/mumps_orderings.h
		src/mumps_size.h
		src/mumps_tags.h
	)
		


# Common MUMPS object files (Fortran)
ADD_LIBRARY(MUMPS_COMMON_OBJ_FORTRAN OBJECT
						${MUMPS_COMMON_OTHER_HEAD} # not sure IF needed
						${MUMPS_COMMON_SRC_FORTRAN}
						${MUMPS_COMMON_OTHER_SRC_FORTRAN})
				
SET_TARGET_PROPERTIES(MUMPS_COMMON_OBJ_FORTRAN PROPERTIES
						COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_FORTRAN} ${ORDERINGS_COMPILER_FLAGS_FORTRAN} -c"
						LINKER_LANGUAGE FORTRAN)

TARGET_INCLUDE_DIRECTORIES(MUMPS_COMMON_OBJ_FORTRAN	PUBLIC
						"${MUMPS_INCLUDE_DIRS}"
						"${LIBSEQ_INCLUDE_DIRS}")
						
# Common MUMPS object files (C)
ADD_LIBRARY(MUMPS_COMMON_OBJ_C OBJECT
						${MUMPS_COMMON_OTHER_HEAD} # not sure IF needed
						${MUMPS_COMMON_OTHER_SRC_C})
				

SET_TARGET_PROPERTIES(MUMPS_COMMON_OBJ_C PROPERTIES
						COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_C} ${ORDERINGS_COMPILER_FLAGS_C} -c"
						COMPILE_DEFINITIONS "${COMPILER_DEFS_C}"
						LINKER_LANGUAGE C)

TARGET_INCLUDE_DIRECTORIES(MUMPS_COMMON_OBJ_C	PUBLIC
						"${MUMPS_INCLUDE_DIRS}"
						"${LIBSEQ_INCLUDE_DIRS}"
						"${ORDERINGS_INCLUDE_DIRS}")

IF (NOT BUILD_SHARED_LIBS)
	# Common MUMPS library
	ADD_LIBRARY(libmumps_common STATIC
							$<TARGET_OBJECTS:MUMPS_COMMON_OBJ_FORTRAN>
							$<TARGET_OBJECTS:MUMPS_COMMON_OBJ_C>)
						
	TARGET_INCLUDE_DIRECTORIES(libmumps_common	PUBLIC
							"${MUMPS_INCLUDE_DIRS}"
							"${LIBSEQ_INCLUDE_DIRS}"
							"${ORDERINGS_INCLUDE_DIRS}")

	TARGET_LINK_LIBRARIES(libmumps_common
							libpord
							libmpiseq)
ENDIF()


						

		
		
# Arithmetic-specific MUMPS files
SET(${ARITH_PREFIX}MUMPS_SRC
		src/${ARITH_PREFIX}ana_aux_par.F 
		src/${ARITH_PREFIX}ana_lr.F
		src/${ARITH_PREFIX}fac_asm_master_m.F
		src/${ARITH_PREFIX}fac_asm_master_ELT_m.F
		src/${ARITH_PREFIX}omp_tps_m.F
		src/${ARITH_PREFIX}mumps_comm_buffer.F
		src/${ARITH_PREFIX}mumps_load.F
		src/${ARITH_PREFIX}mumps_lr_data_m.F
		src/${ARITH_PREFIX}mumps_ooc_buffer.F
		src/${ARITH_PREFIX}mumps_ooc.F
		src/${ARITH_PREFIX}mumps_struc_def.F
		src/${ARITH_PREFIX}static_ptr_m.F
		src/${ARITH_PREFIX}lr_core.F
		src/${ARITH_PREFIX}fac_lr.F
		src/${ARITH_PREFIX}fac_omp_m.F
		src/${ARITH_PREFIX}fac_par_m.F
		src/${ARITH_PREFIX}fac_front_LU_type1.F
		src/${ARITH_PREFIX}fac_front_LU_type2.F
		src/${ARITH_PREFIX}fac_front_LDLT_type1.F
		src/${ARITH_PREFIX}fac_front_LDLT_type2.F
		src/${ARITH_PREFIX}fac_front_aux.F
		src/${ARITH_PREFIX}fac_front_type2_aux.F
	)
			
SET(${ARITH_PREFIX}MUMPS_OTHER_SRC
		src/${ARITH_PREFIX}ini_driver.F
		src/${ARITH_PREFIX}ana_driver.F
		src/${ARITH_PREFIX}fac_driver.F
		src/${ARITH_PREFIX}sol_driver.F
		src/${ARITH_PREFIX}end_driver.F
		src/${ARITH_PREFIX}ana_aux_ELT.F
		src/${ARITH_PREFIX}ana_aux.F
		src/${ARITH_PREFIX}ana_dist_m.F
		src/${ARITH_PREFIX}ana_LDLT_preprocess.F
		src/${ARITH_PREFIX}ana_reordertree.F
		src/${ARITH_PREFIX}arrowheads.F
		src/${ARITH_PREFIX}bcast_int.F
		src/${ARITH_PREFIX}fac_asm_ELT.F
		src/${ARITH_PREFIX}fac_asm.F
		src/${ARITH_PREFIX}fac_b.F
		src/${ARITH_PREFIX}fac_distrib_distentry.F
		src/${ARITH_PREFIX}fac_distrib_ELT.F
		src/${ARITH_PREFIX}fac_lastrtnelind.F
		src/${ARITH_PREFIX}fac_mem_alloc_cb.F
		src/${ARITH_PREFIX}fac_mem_compress_cb.F
		src/${ARITH_PREFIX}fac_mem_free_block_cb.F
		src/${ARITH_PREFIX}fac_mem_stack_aux.F
		src/${ARITH_PREFIX}fac_mem_stack.F
		src/${ARITH_PREFIX}fac_process_band.F
		src/${ARITH_PREFIX}fac_process_blfac_slave.F
		src/${ARITH_PREFIX}fac_process_blocfacto_LDLT.F
		src/${ARITH_PREFIX}fac_process_blocfacto.F
		src/${ARITH_PREFIX}fac_process_bf.F
		src/${ARITH_PREFIX}fac_process_end_facto_slave.F
		src/${ARITH_PREFIX}fac_process_contrib_type1.F
		src/${ARITH_PREFIX}fac_process_contrib_type2.F
		src/${ARITH_PREFIX}fac_process_contrib_type3.F
		src/${ARITH_PREFIX}fac_process_maprow.F
		src/${ARITH_PREFIX}fac_process_master2.F
		src/${ARITH_PREFIX}fac_process_message.F
		src/${ARITH_PREFIX}fac_process_root2slave.F
		src/${ARITH_PREFIX}fac_process_root2son.F
		src/${ARITH_PREFIX}fac_process_rtnelind.F
		src/${ARITH_PREFIX}fac_root_parallel.F
		src/${ARITH_PREFIX}fac_scalings.F
		src/${ARITH_PREFIX}fac_determinant.F
		src/${ARITH_PREFIX}fac_scalings_simScaleAbs.F
		src/${ARITH_PREFIX}fac_scalings_simScale_util.F
		src/${ARITH_PREFIX}fac_sol_pool.F
		src/${ARITH_PREFIX}fac_type3_symmetrize.F
		src/${ARITH_PREFIX}ini_defaults.F
		src/${ARITH_PREFIX}mumps_driver.F
		src/${ARITH_PREFIX}mumps_f77.F
		src/${ARITH_PREFIX}mumps_iXamax.F
		src/${ARITH_PREFIX}ana_mtrans.F
		src/${ARITH_PREFIX}ooc_panel_piv.F
		src/${ARITH_PREFIX}rank_revealing.F
		src/${ARITH_PREFIX}sol_aux.F
		src/${ARITH_PREFIX}sol_bwd_aux.F
		src/${ARITH_PREFIX}sol_bwd.F
		src/${ARITH_PREFIX}sol_c.F
		src/${ARITH_PREFIX}sol_fwd_aux.F
		src/${ARITH_PREFIX}sol_fwd.F
		src/${ARITH_PREFIX}sol_matvec.F
		src/${ARITH_PREFIX}sol_root_parallel.F
		src/${ARITH_PREFIX}tools.F
		src/${ARITH_PREFIX}type3_root.F
		src/${ARITH_PREFIX}mumps_restart.F
	)
			
# Arithmetic-specific MUMPS object files
ADD_LIBRARY(MUMPS_SPECIFIC_OBJ_FORTRAN OBJECT
						"${MUMPS_COMMON_OTHER_HEAD}" # not sure IF needed
						"${${ARITH_PREFIX}MUMPS_SRC}"
						"${${ARITH_PREFIX}MUMPS_OTHER_SRC}")
							
SET_TARGET_PROPERTIES(MUMPS_SPECIFIC_OBJ_FORTRAN PROPERTIES
						COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_FORTRAN} ${ORDERINGS_COMPILER_FLAGS_FORTRAN} -c"
						LINKER_LANGUAGE FORTRAN)
			
TARGET_INCLUDE_DIRECTORIES(MUMPS_SPECIFIC_OBJ_FORTRAN PUBLIC
						"${MUMPS_INCLUDE_DIRS}"
						"${LIBSEQ_INCLUDE_DIRS}") #aka INCSEQ
							

IF (NOT BUILD_SHARED_LIBS)
	# Arithmetic-specific MUMPS library
	ADD_LIBRARY("lib${ARITH_PREFIX}mumps" STATIC
							$<TARGET_OBJECTS:MUMPS_SPECIFIC_OBJ_FORTRAN>
							$<TARGET_OBJECTS:MUMPS_COMMON_OBJ_C>)
	
	TARGET_INCLUDE_DIRECTORIES("lib${ARITH_PREFIX}mumps" PUBLIC
							"${MUMPS_INCLUDE_DIRS}"
							"${LIBSEQ_INCLUDE_DIRS}") #aka INCSEQ
						
	TARGET_LINK_LIBRARIES("lib${ARITH_PREFIX}mumps" 
							libmumps_common
							libpord)
ENDIF()
		
						

									
## Build shared library
IF (BUILD_SHARED_LIBS)
						
	ADD_LIBRARY(lib${ARITH_PREFIX}mumps SHARED
						$<TARGET_OBJECTS:MUMPS_COMMON_OBJ_FORTRAN>
						$<TARGET_OBJECTS:MUMPS_COMMON_OBJ_C>
						$<TARGET_OBJECTS:MUMPS_SPECIFIC_OBJ_FORTRAN>
						$<TARGET_OBJECTS:MUMPS_COMMON_OBJ_C>
						${LIBSEQ_HEAD_C}
						${LIBSEQ_HEAD_FORTRAN}
						$<TARGET_OBJECTS:LIBMPISEQ_C>
						$<TARGET_OBJECTS:LIBMPISEQ_FORTRAN>
						${PORD_SOURCES}
						${PORD_HEAD}
						)
						
	TARGET_LINK_LIBRARIES(lib${ARITH_PREFIX}mumps
                                "${BLAS_LIBRARIES}" #aka LIBBLAS
                                )
								
	# SET_TARGET_PROPERTIES(lib${ARITH_PREFIX}mumps PROPERTIES
							# LINKER_FLAGS "${GENERIC_COMPILER_FLAGS_C} ${ORDERINGS_COMPILER_FLAGS_C}")
						
						
	TARGET_INCLUDE_DIRECTORIES(lib${ARITH_PREFIX}mumps PUBLIC
								${ORDERINGS_INCLUDE_DIRS}
								${LIBSEQ_INCLUDE_DIRS}
								${MUMPS_INCLUDE_DIRS}
								)
ENDIF ()


############### Examples ###############
IF(BUILD_EXAMPLES)

	ADD_EXECUTABLE(${ARITH_PREFIX}simpletest
							examples/${ARITH_PREFIX}simpletest.F
							)
							
	TARGET_INCLUDE_DIRECTORIES(${ARITH_PREFIX}simpletest PUBLIC
							# "${BLAS_INCLUDE_DIRS}"
							"${LIBSEQ_INCLUDE_DIRS}"
							"${MUMPS_INCLUDE_DIRS}")
	
	IF (BUILD_SHARED_LIBS)
		
		TARGET_LINK_LIBRARIES(${ARITH_PREFIX}simpletest 	
							libsharedmumps
							"${BLAS_LIBRARIES}" #aka LIBBLAS
							)  
							
							
		ADD_DEPENDENCIES(${ARITH_PREFIX}simpletest
								libsharedmumps
								)
								
	ELSE ()
		
		TARGET_LINK_LIBRARIES(${ARITH_PREFIX}simpletest 	
							libmumps_common
							lib${ARITH_PREFIX}mumps
							libmpiseq #aka LIBS
							"${ORDERINGS_LIBRARIES}" #aka LORDERINGS
							"${BLAS_LIBRARIES}" #aka LIBBLAS
							)  
							
							
		ADD_DEPENDENCIES(${ARITH_PREFIX}simpletest
								libmumps_common
								lib${ARITH_PREFIX}mumps
								libmpiseq
								libpord
								)
	ENDIF ()
	
	
										
	SET_TARGET_PROPERTIES(${ARITH_PREFIX}simpletest PROPERTIES
							COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_FORTRAN} ${EXCLUDE_IOMP}"
							LINKER_LANGUAGE FORTRAN)
							
	INSTALL(TARGETS ${ARITH_PREFIX}simpletest
			RUNTIME DESTINATION examples
			LIBRARY DESTINATION lib
			ARCHIVE DESTINATION lib)
			
    IF (ARITH_PREFIX MATCHES "d")
        MESSAGE(STATUS "Added c_example") #TODO
        ADD_EXECUTABLE(c_example
                                examples/c_example.c)
                            
        TARGET_INCLUDE_DIRECTORIES(c_example PUBLIC
                                "${LIBSEQ_INCLUDE_DIRS}"
                                "${MUMPS_INCLUDE_DIRS}"
                                )
                                
                            
        IF (BUILD_SHARED_LIBS)
			TARGET_LINK_LIBRARIES(c_example
								libsharedmumps
								"${BLAS_LIBRARIES}" #aka LIBBLAS
								)  
								
								
			ADD_DEPENDENCIES(c_example
									libsharedmumps
									)
		ELSE ()
			TARGET_LINK_LIBRARIES(c_example 	
								libmumps_common
								lib${ARITH_PREFIX}mumps
								libmpiseq #aka LIBS
								"${ORDERINGS_LIBRARIES}" #aka LORDERINGS
								"${BLAS_LIBRARIES}" #aka LIBBLAS
								)  
								
								
			ADD_DEPENDENCIES(c_example
									libmumps_common
									lib${ARITH_PREFIX}mumps
									libmpiseq
									libpord
									)
		ENDIF ()
                            
        SET_TARGET_PROPERTIES(c_example PROPERTIES
                                COMPILE_FLAGS "${GENERIC_COMPILER_FLAGS_C}"
                                LINKER_LANGUAGE C)
        
        INSTALL(TARGETS c_example
            RUNTIME DESTINATION examples
            LIBRARY DESTINATION lib
            ARCHIVE DESTINATION lib)
        
    ENDIF ()
ENDIF(BUILD_EXAMPLES)

############### INSTALL ###############
#headers
INSTALL(FILES ${MUMPS_HEADERS}
			  # ${MUMPS_COMMON_OTHER_HEAD}
		DESTINATION "${CMAKE_INSTALL_PREFIX}/include"
)

INSTALL(FILES ${LIBSEQ_HEAD_C}
			  # ${LIBSEQ_HEAD_FORTRAN}
		DESTINATION "${CMAKE_INSTALL_PREFIX}/include"
)

INSTALL(FILES ${PORD_HEAD}
		DESTINATION "${CMAKE_INSTALL_PREFIX}/include"
)

#libraries
INSTALL(TARGETS lib${ARITH_PREFIX}mumps
		RUNTIME DESTINATION bin
		LIBRARY DESTINATION lib
		ARCHIVE DESTINATION lib)

IF (NOT BUILD_SHARED_LIBS)
	INSTALL(TARGETS libmumps_common
			RUNTIME DESTINATION bin
			LIBRARY DESTINATION lib
			ARCHIVE DESTINATION lib)

	INSTALL(TARGETS libmpiseq
			RUNTIME DESTINATION bin
			LIBRARY DESTINATION libseq
			ARCHIVE DESTINATION libseq)

	INSTALL(TARGETS libpord
			RUNTIME DESTINATION bin
			LIBRARY DESTINATION PORD/lib
			ARCHIVE DESTINATION PORD/lib)
ENDIF()
