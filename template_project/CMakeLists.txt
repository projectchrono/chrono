#--------------------------------------------------------------
#
# Example of file to be used in CMake to build an external 
# project based on Chrono::Engine
#
# Copy the entire directory template_project in some directory of yours,
# then modify this file in points highlighted with numbers 1,2,3.
#
#--------------------------------------------------------------
 

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#--------------------------------------------------------------
#		  === 1 === 
#         Modify the project name if you want: 
# 

PROJECT(my_project)


#--------------------------------------------------------------
# NOTE!   use find_package() to define ChronoEngine libraries and variables, 
#         along with some optional units (components).
#         This requires that you copied FindChronoEngine.cmake in
#         the Modules/ directory of your Cmake installation. If it 
#         is not there, you can copy it in a cmake/ directory in your project, 
#         and add that directory to the search path of CMake by typing
#         set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")


#--------------------------------------------------------------
#	      === 2 ===
#         Here add the Chrono::Engine units that you might want to link 
#         with your project. Append their name after the COMPONENTS keyword.
#         Some examples:  
#           unit_COSIMULATION unit_FEA  unit_PYPARSER unit_POSTPROCESS
#           unit_IRRLICHT unit_MATLAB unit_CASCADE
#         etc. (see the wiki for the list of available unit components.)
#         After a successfull run of this find script you will have two 
#         important variables automatically set for you: 
#           CHRONOENGINE_INCLUDES   with all directories for including .h headers
#           CHRONOENGINE_LIBRARIES  with all libraries that must be linked
#         Note: if you want to use, say, unit_IRRLICHT, you must have 
#         compiled Chrono::Engine with ENABLE_UNIT_IRRLICHT turned on. 
#         The same for other units. 

FIND_PACKAGE(ChronoEngine COMPONENTS unit_IRRLICHT)


#--------------------------------------------------------------
#         This is needed so your .cpp and .h files will find 
#         all Chrono::Engine .h headers.

INCLUDE_DIRECTORIES(${CHRONOENGINE_INCLUDES})


#--------------------------------------------------------------
#	      === 3 ===
#         Here append the .cpp file(s) of your project. 
#        

ADD_EXECUTABLE(myexe my_example.cpp)


#--------------------------------------------------------------
#         This is needed in order to link the libraries of 
#         Chrono::Engine and its optional units.

TARGET_LINK_LIBRARIES(myexe ${CHRONOENGINE_LIBRARIES})


#--------------------------------------------------------------
#	      === OPTIONAL ===
#         
#         Use this macro to copy the files in the data/ directory 
#         into a data/ directory in the binary directory, close 
#         to the executables. For example if you have textures, etc. 
#         This is because by default Chrono::Engine demos find 
#         their textures in ../data .

CHRONOENGINE_COPY_PROJECT_DATA_DIR()


#--------------------------------------------------------------
#	      === OPTIONAL ===
# 
#         Use this macro on Windows for enabling automatic copy of 
#         the .dll of the Chrono::Engine API from ChronoEngine binary 
#         directory into your project binary directory, at each build
#         of the solution. This done, the .exe files can find the dlls 
#         because they are in the same directory. 
#         Alternatively: A)you could copy the dlls by hand, or B)you 
#         can put them in a globally accessible PATH. 

CHRONOENGINE_ENABLE_CHRONODLLCOPY()
