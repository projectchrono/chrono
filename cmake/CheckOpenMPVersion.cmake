# This script checks for the highest level of OpenMP support on the host
# by compiling small C++ programs that use various features.
#
# TODO: needs more work. 


include(CheckCXXSourceCompiles)

# Set compiler flag to generate instructions for the host architecture.
set(CMAKE_REQUIRED_FLAGS "-march=native")

# Assume at least 2.0 support
SET(OMP_VERSION "2.0")
SET(OMP_20 1)
SET(OMP_30 0)
SET(OMP_40 0)

# Check for OpenMP 3.0 support.
check_cxx_source_compiles("
int main() {
#pragma omp parallel for
for (unsigned int i=0; i<10; i++) ;
}" OMP_30_DETECTED)
IF(OMP_30_DETECTED)
  SET(OMP_VERSION "3.0")
  SET(OMP_30 1)
ENDIF()

# Check for OpenMP 4.0 support.
check_cxx_source_compiles("
int main() {
#pragma omp parallel for simd safelen(4)
for (int i=0; i<10; i++) ;
}" OMP_40_DETECTED)
IF(OMP_40_DETECTED)
  SET(OMP_VERSION "4.0")
  SET(OMP_40 1)
ENDIF()


