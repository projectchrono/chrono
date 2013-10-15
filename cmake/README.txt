This directory contains files for helping users that want to develop third-party
projects based on Chrono::Engine.

We reccommend using the free tool CMake for handling the build chain of
your projects. If you use CMake, you can use the useful "FindChronoEngine.cmake"
file that is included in this directory: it will help you by setting all the
includes and libraries.

An example of usage in your CMakeLists.txt can be:

   PROJECT(myproject)
   find_package(ChronoEngine COMPONENTS unit_POSTPROCESS unit_FEM)
   include(${CHRONOENGINE_INCLUDES})
   add_executable(myexe main.cpp)
   target_link_libraries(myexe ${CHRONOENGINE_LIBRARIES})

Note, to FindChronoEngine.cmake , either you copy it in the Modules/ directory of 
your CMake installation, or you put it in your project directory under a directory, 
say it is yourproject/cmake/ , and you write 
   set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")
right before find_package(..)
