#!/bin/bash

# -----------------------------------------------------------------------------------------
# Bash script for configuring and building Chrono on macOS
# -----------------------------------------------------------------------------------------
# The structure of the install directories for VSG and GL libraries is assumed to be as
# created by running the buildVSG and buildGL scripts. If using alternative installations
# of these libraries, modify as appropriate the CMake variables with paths to the various
# project configuration scripts.
#
# On macOS, some libraries are better installed via homebrew:
#     - eigen
#     - irrlicht
#     - libomp
#
# The following libraries should *NOT* be installed by homebrew for compatibility reasons:
#     - opencascade
#     - spectra
#
# Tools:
#     - gnuplot (optional)
#     - ninja (optional)
#     - cmake
#
# Unavailable on macOS:
#     - mkl (Apple Silicon)
#     - fastrtps
#     - cuda / optix
#     - csharp
#     - DEM / FSI (require CUDA)
#     - COSIMULATION
#     - SENSOR (requires CUDA/OptiX)
#     - SYNCHRONO (requires FastDDS)
#     - MATLAB (Apple Silicon)
#     - PARDISO_MKL / PARDISO_PROJECT (Apple Silicon)
# 
# -----------------------------------------------------------------------------------------
# 1. As needed, use the provided scripts to build and install various dependencies.
# 2. Edit this script to specify the installation directories for the various dependencies.
#    Dependencies for Chrono modules that are disabled are ignored.
# 3. As needed, modify the CMake generator (BUILDSYSTEM). We suggest using Ninja
#    (ninja-build.org/) and the "Ninja Multi-Config" CMake generator. Otherwise, you will
#    need to explicitly set the CMAKE_BUILD_TYPE variable).
# 4. Edit the CMake command to disable selected Chrono modules.
# 5. Run the script (sh ./buildChrono.sh).
# -----------------------------------------------------------------------------------------

set -e

# ------------------------------------------------------------------------
# Auto-detect architecture
# ------------------------------------------------------------------------
ARCH=$(uname -m)
echo "Detected architecture: ${ARCH}"

if [ "${ARCH}" = "arm64" ]; then
    echo "Apple Silicon detected — CUDA/FSI/DEM/SENSOR/SYNCHRONO/MATLAB/MKL are unavailable"
elif [ "${ARCH}" = "x86_64" ]; then
    echo "Intel Mac detected"
    echo "Note: MKL, MATLAB, and PARDISO may be available on Intel"
else
    echo "Unknown architecture: ${ARCH}"
    exit 1
fi

# ------------------------------------------------------------------------
# Pre-flight: verify required tools
# ------------------------------------------------------------------------
MISSING_DEPS=()

if ! command -v cmake &> /dev/null; then
    MISSING_DEPS+=("cmake")
fi

if ! command -v brew &> /dev/null; then
    echo "WARNING: Homebrew not found. Many dependencies require it."
else
    HOMEBREW_PREFIX=$(brew --prefix)
    echo "Homebrew prefix: ${HOMEBREW_PREFIX}"

    # Verify brewed dependencies if available
    for dep in eigen irrlicht libomp; do
        if ! brew list --formula "${dep}" &> /dev/null; then
            echo "WARNING: ${dep} not installed via Homebrew (brew install ${dep})"
        fi
    done
fi

if ! command -v ninja &> /dev/null; then
    echo "NOTE: ninja not found — falling back to default CMake generator"
    BUILDSYSTEM=""
else
    BUILDSYSTEM="Ninja Multi-Config"
fi

if [ ${#MISSING_DEPS[@]} -ne 0 ]; then
    echo "ERROR: Missing required tools: ${MISSING_DEPS[*]}"
    echo "Install with: brew install ${MISSING_DEPS[*]}"
    exit 1
fi

# ------------------------------------------------------------------------
# Configurable paths — edit these for your environment
# ------------------------------------------------------------------------
SOURCE_DIR="${HOME}/Source/chrono"
BUILD_DIR="${HOME}/Build/chrono"
INSTALL_DIR="${HOME}/Install/chrono"

EIGEN3_INSTALL_DIR="${HOMEBREW_PREFIX}/include/eigen3"
IRRLICHT_ROOT="${HOMEBREW_PREFIX}/include/irrlicht"

BLAZE_ROOT="${HOME}/Packages/blaze-3.8"
THRUST_INCLUDE_DIR="${HOME}/Packages/thrust"
CRG_INSTALL_DIR="${HOME}/Packages/OpenCRG"
VSG_INSTALL_DIR="${HOME}/Packages/vsg"

URDF_INSTALL_DIR="C:/Packages/urdf"

CASCADE_INSTALL_DIR="/usr/local/include/opencascade"
SPECTRA_INCLUDE_DIR="/usr/local/include"

SWIG_EXE="swig"

# Detect OpenMP paths from Homebrew (version-independent)
if [ -n "${HOMEBREW_PREFIX}" ] && brew list --formula libomp &> /dev/null; then
    LIPOMP_DIR="${HOMEBREW_PREFIX}/opt/libomp"
    LIPOMP_INCLUDE="${LIPOMP_DIR}/include"
    LIPOMP_LIB="${LIPOMP_DIR}/lib/libomp.dylib"
    if [ -f "${LIPOMP_LIB}" ]; then
        echo "Found OpenMP: ${LIPOMP_DIR}"
    else
        echo "WARNING: libomp.dylib not found at ${LIPOMP_LIB}"
        LIPOMP_DIR=""
    fi
else
    echo "NOTE: OpenMP (libomp) not found — install with: brew install libomp"
    LIPOMP_DIR=""
fi

# Detect Python from Homebrew
if [ -n "${HOMEBREW_PREFIX}" ]; then
    BREW_PYTHON_DIR=$(brew --prefix python@3 2>/dev/null || brew --prefix python@3.12 2>/dev/null || echo "")
    if [ -n "${BREW_PYTHON_DIR}" ] && [ -d "${BREW_PYTHON_DIR}" ]; then
        PYTHON_EXECUTABLE="${HOMEBREW_PREFIX}/bin/python3"
        BREW_PY_VERSION=$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')
        PYTHON_INCLUDE_DIR="${HOMEBREW_PREFIX}/opt/python@${BREW_PY_VERSION}/Frameworks/Python.framework/Versions/${BREW_PY_VERSION}/include/python${BREW_PY_VERSION}"
        PYTHON_LIBRARY="${HOMEBREW_PREFIX}/opt/python@${BREW_PY_VERSION}/Frameworks/Python.framework/Versions/${BREW_PY_VERSION}/lib/python${BREW_PY_VERSION}/config-${BREW_PY_VERSION}-darwin/libpython${BREW_PY_VERSION}.dylib"
        echo "Found Python ${BREW_PY_VERSION}: ${PYTHON_EXECUTABLE}"
    else
        echo "NOTE: Homebrew Python not found. PyChrono may not build."
    fi
fi

# ------------------------------------------------------------------------
# Clean and configure
# ------------------------------------------------------------------------
rm -rf "${BUILD_DIR}"

# Build CMake arguments array
CMAKE_ARGS=(
    -G "${BUILDSYSTEM}"
    -B "${BUILD_DIR}"
    -S "${SOURCE_DIR}"
    -DCMAKE_INSTALL_PREFIX:PATH="${INSTALL_DIR}"
    -DBUILD_BENCHMARKING:BOOL=ON
    -DBUILD_TESTING:BOOL=ON
    -DCH_ENABLE_MODULE_POSTPROCESS:BOOL=ON
    -DCH_ENABLE_MODULE_IRRLICHT:BOOL=ON
    -DCH_ENABLE_MODULE_MULTICORE:BOOL=ON
    -DCH_ENABLE_MODULE_MODAL:BOOL=ON
    -DCH_ENABLE_MODULE_VEHICLE:BOOL=ON
    -DCH_ENABLE_MODULE_VSG:BOOL=OFF
    -DCH_ENABLE_MODULE_CASCADE:BOOL=ON
    -DCH_ENABLE_MODULE_PYTHON:BOOL=ON
    -DCH_ENABLE_OPENCRG:BOOL=ON
    -DTHRUST_INCLUDE_DIR:PATH="${THRUST_INCLUDE_DIR}"
    -DBlaze_ROOT:PATH="${BLAZE_ROOT}"
    -DIrrlicht_ROOT:PATH="${IRRLICHT_ROOT}"
    -DOpenCRG_INCLUDE_DIR:PATH="${CRG_INSTALL_DIR}/include"
    -DOpenCRG_LIBRARY:FILEPATH="${CRG_INSTALL_DIR}/lib/libOpenCRG.a"
    -DOpenCASCADE_DIR:PATH="${CASCADE_INSTALL_DIR}/lib/cmake/opencascade"
    -DSpectra_INCLUDE_DIR:PATH="${SPECTRA_INCLUDE_DIR}"
    -Durdfdom_DIR:PATH="${URDF_INSTALL_DIR}/CMake"
    -Durdfdom_headers_DIR:PATH="${URDF_INSTALL_DIR}/CMake"
    -Dconsole_bridge_DIR:PATH="${URDF_INSTALL_DIR}/CMake"
    -DSWIG_EXECUTABLE:FILEPATH="${SWIG_EXE}"
)

# Add OpenMP flags if libomp is available
if [ -n "${LIPOMP_DIR}" ]; then
    CMAKE_ARGS+=(
        -DOpenMP_CXX_FLAGS:STRING="-Xclang -fopenmp -I${LIPOMP_INCLUDE}"
        -DOpenMP_C_FLAGS:STRING="-Xclang -fopenmp"
        -DOpenMP_C_INCLUDE_DIR:PATH="${LIPOMP_INCLUDE}"
        -DOpenMP_CXX_INCLUDE_DIR:PATH="${LIPOMP_INCLUDE}"
        -DOpenMP_C_LIB_NAMES:STRING=libomp
        -DOpenMP_CXX_LIB_NAMES:STRING=libomp
        -DOpenMP_libomp_LIBRARY:FILEPATH="${LIPOMP_LIB}"
    )
fi

# Add Python paths if found
if [ -n "${PYTHON_EXECUTABLE}" ]; then
    CMAKE_ARGS+=(
        -DPYTHON_EXECUTABLE:PATH="${PYTHON_EXECUTABLE}"
        -DPYTHON_INCLUDE_DIR:PATH="${PYTHON_INCLUDE_DIR}"
        -DPYTHON_LIBRARY:PATH="${PYTHON_LIBRARY}"
    )
fi

echo ""
echo "Running cmake with:"
for arg in "${CMAKE_ARGS[@]}"; do
    if [[ "${arg}" =~ ^-D ]] || [[ "${arg}" =~ ^-G ]] || [[ "${arg}" =~ ^-B ]] || [[ "${arg}" =~ ^-S ]]; then
        echo "  ${arg}"
    fi
done
echo ""

cmake "${CMAKE_ARGS[@]}"

# ------------------------------------------------------------------------
echo ""
echo "Configuration complete. To build:"
echo "  cmake --build ${BUILD_DIR}"
echo ""
echo "To build a specific configuration (multi-config generators):"
echo "  cmake --build ${BUILD_DIR} --config Release"
echo "  cmake --build ${BUILD_DIR} --config Debug"
echo ""
echo "To open in Xcode (if using Xcode generator):"
echo "  open ${BUILD_DIR}/Chrono.xcodeproj"
