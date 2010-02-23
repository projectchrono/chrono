# 
# CHRONO EXAMPLES: BASICS
#
# Makefile for building Chrono examples about BASIC FUCNTIONALITY
#
# This makefile must be used with the 'nmake' tool (it comes with 
# most Microsoft Visual C++ distributions), for example open a 
# command-line console, cd into this directory and and type:
#
#    nmake -f makefile
#
# Note that you must have compiler and linker correctly
# installed, and you should have edited the config.mak file
# as necessary (see in directory chrono/makes/)
# 


include ../Delta/ChronoEngine/makes/config.mak 


	# From now on, thank to config.mak, some useful variables
	# have been automatically set. Most important are:
	#   CH_CHRONODIR (the directory of Chrono sdk)
	#   CH_LIBRARIES (the Chrono API libraries to be linked), 
	#   CH_INCLUDES (directories of header files, needed by compiler), 
	#   CH_LINKEXEFLAGS (needed flags for linker, to make executables),
	#   CH_COMPILERFLAGS (needed flags for the compiler), 
	#   CH_COMPILERFLAG_FILEOUT (flag to set compiler output compiler,ex.=-Fo for MSVC)
	#   CC (the command to invoke the compiler,
	#   LD (the command to invoke the linker). 
	# Then, from now on, you can use them in your makefile,
	# as in the example below.

		


CPPFILE_DEMO_VEHICLE = \
	demo_vehicle.cpp 
CPPFILE_VEHICLE_CAR = \
	vehicle_car.cpp
CPPFILE_VEHICLE_GUI = \
	vehicle_gui.cpp  
CPPFILE_VEHICLE_SIMULATOR = \
	vehicle_simulator.cpp 


OFILE_DEMO_VEHICLE=$(CPPFILE_DEMO_VEHICLE:.cpp=.o) 
OFILE_VEHICLE_CAR=$(CPPFILE_VEHICLE_CAR:.cpp=.o) 
OFILE_VEHICLE_GUI=$(CPPFILE_VEHICLE_GUI:.cpp=.o)
OFILE_VEHICLE_SIMULATOR=$(CPPFILE_VEHICLE_SIMULATOR:.cpp=.o)

BINFILE_DEMO_VEHICLE="$(CH_BINDIR)$(D)racing.exe"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"



all: $(OFILE_DEMO_VEHICLE) \
     $(OFILE_VEHICLE_CAR) \
     $(OFILE_VEHICLE_GUI) \
     $(OFILE_VEHICLE_SIMULATOR)
	$(LD) $(CH_LINKEXEFLAGS) /FORCE:MULTIPLE $(CH_LIBRARIES) $(CH_IRRLICHTLIB) $(OFILE_DEMO_VEHICLE) $(OFILE_VEHICLE_CAR) $(OFILE_VEHICLE_GUI) $(OFILE_VEHICLE_SIMULATOR) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_VEHICLE)
	$(MT) -manifest $(BINFILE_DEMO_VEHICLE).manifest -outputresource:$(BINFILE_DEMO_VEHICLE);#1
	$(DEL) $(BINFILE_DEMO_VEHICLE).manifest
