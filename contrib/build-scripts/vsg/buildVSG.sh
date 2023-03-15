#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Shell script for building VSG based on the last official releases.
# - Requires cmake, wget, and unzip
# - Place in an arbitrary temporary directory.
# - Specify the locations for the VSG sources OR indicate that these should be downloaded.
# - Specify the install directory.
# - Optionally set the path to the doxygen and dot executables
# - Decide whether to build shared or static libraries, whether to also build debug libraries,
#   and whether to build the VSG documentation.
# - Run the script (sh ./buildVSG.sh).
# - The install directory will contain (under subdirectories of VSG_INSTALL_DIR/lib/shared) all VSG CMake
#   project configuration scripts required to configure Chrono with the Chorno::VSG module enabled.
#
# Notes:
# - This was tested with the following versions of VSG libraries:
#      VulkanSceneGraph (github.com/vsg-dev/VulkanSceneGraph.git): Commit #aa28d62
#      vsgXchange (github.com/vsg-dev/vsgXchange.git):             Commit #b37861b
#      vsgImGui (github.com/vsg-dev/vsgImGui.git):                 Commit #0ec0cd4
#      vsgExamples (github.com/vsg-dev/vsgExamples.git):           Commit #c789753
#      assimp (github.com/assimp/assimp):                          Commit #8c6b3fe
# - We suggest using Ninja (ninja-build.org/) and the "Ninja Multi-Config" CMake generator.
#   (otherwise, you will need to explicitly set the CMAKE_BUILD_TYPE variable)
# -------------------------------------------------------------------------------------------------------

VSG_SOURCE_DIR="$HOME/Repositories/VulkanSceneGraph"
VSGXCHANGE_SOURCE_DIR="$HOME/Repositories/vsgXchange"
VSGIMGUI_SOURCE_DIR="$HOME/Repositories/vsgImGui"
VSGEXAMPLES_SOURCE_DIR="$HOME/Repositories/vsgExamples"
ASSIMP_SOURCE_DIR="$HOME/Repositories/assimp"

DOWNLOAD=ON

VSG_INSTALL_DIR="$HOME/Packages/vsg"

DOXYGEN_EXE="doxygen"
DOT_EXE="dot"

BUILDSHARED=ON
BUILDDOCS=OFF
BUILDDEBUG=ON
BUILDSYSTEM="Ninja Multi-Config"

# ------------------------------------------------------------------------

if [ ${DOWNLOAD} = ON ]
then
    echo "Download sources from GitHub"

    rm -rf download_vsg
    mkdir download_vsg

    echo "  ... VulkanSceneGraph"
    wget https://github.com/vsg-dev/VulkanSceneGraph/archive/refs/heads/master.zip -O download_vsg/vsg.zip
    unzip -q download_vsg/vsg.zip -d download_vsg
    VSG_SOURCE_DIR="download_vsg/VulkanSceneGraph-master"

    echo "  ... vsgXchange"    
    wget https://github.com/vsg-dev/vsgXchange/archive/refs/heads/master.zip -OutFile download_vsg/vsgXchange.zip -O download_vsg/vsgXchange.zip
    unzip -q download_vsg/vsgXchange.zip -d download_vsg
    VSGXCHANGE_SOURCE_DIR="download_vsg/vsgXchange-master"

    echo "  ... vsgImGui"
    wget https://github.com/vsg-dev/vsgImGui/archive/refs/heads/master.zip -O download_vsg/vsgImGui.zip
    unzip -q download_vsg/vsgImGui.zip -d download_vsg
    VSGIMGUI_SOURCE_DIR="download_vsg/vsgImGui-master"

    echo "  ... ImGui"
    wget https://github.com/ocornut/imgui/archive/refs/heads/master.zip -O download_vsg/imgui.zip
    unzip -q download_vsg/imgui.zip -d download_vsg
    cp -r download_vsg/imgui-master/* download_vsg/vsgImGui-master/src/imgui/

    echo "  ... ImPlot"
    wget https://github.com/epezent/implot/archive/refs/heads/master.zip -O download_vsg/implot.zip
    unzip -q download_vsg/implot.zip -d download_vsg
    cp -r download_vsg/implot-master/* download_vsg/vsgImGui-master/src/implot/
    
    echo "  ... vsgExamples"
    wget https://github.com/vsg-dev/vsgExamples/archive/refs/heads/master.zip -OutFile download_vsg/vsgExamples.zip -O download_vsg/vsgExamples.zip
    unzip -q download_vsg/vsgExamples.zip -d download_vsg
    VSGEXAMPLES_SOURCE_DIR="download_vsg/vsgExamples-master"

    echo "  ... assimp"
    wget https://github.com/assimp/assimp/archive/refs/heads/master.zip -O download_vsg/assimp.zip
    unzip -q download_vsg/assimp.zip -d download_vsg
    ASSIMP_SOURCE_DIR="download_vsg/assimp-master"
else
    echo "Using provided source directories"
fi

echo -e "\nSources in:"
echo "  "  ${VSG_SOURCE_DIR}
echo "  "  ${VSGXCHANGE_SOURCE_DIR}
echo "  "  ${VSGIMGUI_SOURCE_DIR}
echo "  "  ${VSGEXAMPLES_SOURCE_DIR}
echo "  "  ${ASSIMP_SOURCE_DIR}

# ------------------------------------------------------------------------

rm -rf ${VSG_INSTALL_DIR}
mkdir ${VSG_INSTALL_DIR}

# --- assimp -------------------------------------------------------------

echo -e "\n------------------------ Configure assimp\n"
rm -rf build_assimp
cmake -G "${BUILDSYSTEM}" -B build_assimp -S ${ASSIMP_SOURCE_DIR} \
      -DBUILD_SHARED_LIBS:BOOL=OFF \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd \
      -DASSIMP_BUILD_TESTS:BOOL=OFF  \
      -DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF \
      -DASSIMP_BUILD_ZLIB:BOOL=ON \
      -DASSIMP_BUILD_DRACO:BOOL=ON

echo -e "\n------------------------ Build and install assimp\n"
cmake --build build_assimp --config Release
cmake --install build_assimp --config Release --prefix ${VSG_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_assimp --config Debug
    cmake --install build_assimp --config Debug --prefix ${VSG_INSTALL_DIR}
else
    echo "No Debug build of assimp"
fi

# --- vsg ----------------------------------------------------------------

echo -e "\n------------------------ Configure vsg\n"
rm -rf build_vsg
cmake  -G "${BUILDSYSTEM}" -B build_vsg -S ${VSG_SOURCE_DIR}  \
      -DBUILD_SHARED_LIBS:BOOL=${BUILDSHARED} \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd \
      -DDOXYGEN_EXECUTABLE:FILEPATH=${DOXYGEN_EXE} \
      -DDOXYGEN_DOT_EXECUTABLE:FILEPATH=${DOT_EXE}

echo -e "\n------------------------ Build and install vsg\n"
cmake --build build_vsg --config Release
cmake --install build_vsg --config Release --prefix ${VSG_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_vsg --config Debug
    cmake --install build_vsg --config Debug --prefix ${VSG_INSTALL_DIR}
else
    echo "No Debug build of vsg"
fi
if [ ${BUILDDOCS} = ON ]
then
    cmake --build build_vsg --config Release --target docs
    cp -r build_vsg/html ${VSG_INSTALL_DIR}/html
fi

# --- vsgXchange ---------------------------------------------------------

echo -e "\n------------------------ Configure vsgXchange\n"
rm -rf build_vsgXchange
cmake  -G "${BUILDSYSTEM}" -B build_vxgXchange -S ${VSGXCHANGE_SOURCE_DIR}  \
      -DBUILD_SHARED_LIBS:BOOL=${BUILDSHARED} \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd \
      -Dvsg_DIR:PATH=${VSG_INSTALL_DIR}/lib/cmake/vsg

echo -e "\n------------------------ Build and install vsgXchange\n"
cmake --build build_vxgXchange --config Release
cmake --install build_vxgXchange --config Release --prefix ${VSG_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_vxgXchange --config Debug
    cmake --install build_vxgXchange --config Debug --prefix ${VSG_INSTALL_DIR}
else
    echo "No Debug build of vsgXchange"
fi

# --- vsgImGui -----------------------------------------------------------

echo -e "\n------------------------ Configure vsgImGui\n"
rm -rf  build_vsgImGui
cmake -G "${BUILDSYSTEM}" -B build_vsgImGui -S ${VSGIMGUI_SOURCE_DIR} \
      -DBUILD_SHARED_LIBS:BOOL=${BUILDSHARED} \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd \
      -Dvsg_DIR:PATH=${VSG_INSTALL_DIR}/lib/cmake/vsg

echo -e "\n------------------------ Build and install vsgImGui\n"
cmake --build build_vsgImGui --config Release
cmake --install build_vsgImGui --config Release --prefix ${VSG_INSTALL_DIR}
if [ ${BUILDDEBUG} = ON ]
then
    cmake --build build_vsgImGui --config Debug
    cmake --install build_vsgImGui --config Debug --prefix ${VSG_INSTALL_DIR}
else
    echo "No Debug build of vsgImGui"
fi

# --- vsgExamples --------------------------------------------------------

echo -e "\n------------------------ Configure vsgExamples\n"
rm -rf  build_vsgExamples
cmake -G "${BUILDSYSTEM}" -B build_vsgExamples -S ${VSGEXAMPLES_SOURCE_DIR} \
      -Dvsg_DIR:PATH=${VSG_INSTALL_DIR}/lib/cmake/vsg

echo -e "\n------------------------ Build and install vsgExamples\n"
cmake --build build_vsgExamples --config Release
cmake --install build_vsgExamples --config Release --prefix ${VSG_INSTALL_DIR}

# --- VSG_FILE_PATH ------------------------------------------------------

if [ "${VSG_FILE_PATH}_" = "_" ]
then
    if [ $(basename $SHELL) = "zsh" ]
    then
        # zsh
        echo " Add VSG_FILE_PATH to .zprofile"
        export_line="export VSG_FILE_PATH=\"$VSG_INSTALL_DIR/share/vsgExamples\""
        echo ${export_line}
        echo ${export_line} >> ~/.zprofile
        if [ $(uname) = Darwin -a "${DYLIB_LIBRARY_PATH}_" = "_" ]
        then
            echo " Add DYLIB_LIBRARY_PATH to .zprofile"
            export_line="export DYLD_LIBRARY_PATH=\"$VSG_INSTALL_DIR/lib\""
            echo ${export_line} >> ~/.zprofile
        fi
    else
        # Default: bash
        echo "Add VSG_FILE_PATH to .bashrc"
        export_line="export VSG_FILE_PATH=\"$VSG_INSTALL_DIR/share/vsgExamples\""
        echo ${export_line}
        echo ${export_line} >> ~/.bashrc
    fi
else
    echo "VSG_FILE_PATH already exists and is set to $VSG_FILE_PATH"
    echo "Make sure it is set to $VSG_INSTALL_DIR/share/vsgExamples"
fi
