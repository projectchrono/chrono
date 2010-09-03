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


include ../../makes/config.mak


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

		

CPPFILE_DEMO_BUILDSYSTEM = \
	demo_buildsystem.cpp 
	
CPPFILE_DEMO_POWERTRAIN = \
	demo_powertrain.cpp



OFILE_DEMO_BUILDSYSTEM=$(CPPFILE_DEMO_BUILDSYSTEM:.cpp=.o) 
OFILE_DEMO_POWERTRAIN=$(CPPFILE_DEMO_POWERTRAIN:.cpp=.o) 

BINFILE_DEMO_BUILDSYSTEM="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_BUILDSYSTEM:.cpp=.exe)"
BINFILE_DEMO_POWERTRAIN="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_POWERTRAIN:.cpp=.exe)"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"



$(BINFILE_DEMO_BUILDSYSTEM): $(OFILE_DEMO_BUILDSYSTEM)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_BUILDSYSTEM) $(CH_CHRONOLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_BUILDSYSTEM)
	$(MT) -manifest $(BINFILE_DEMO_BUILDSYSTEM).manifest -outputresource:$(BINFILE_DEMO_BUILDSYSTEM);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_BUILDSYSTEM).manifest
	
	
$(BINFILE_DEMO_POWERTRAIN): $(OFILE_DEMO_POWERTRAIN)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_POWERTRAIN) $(CH_CHRONOLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_POWERTRAIN)
	$(MT) -manifest $(BINFILE_DEMO_POWERTRAIN).manifest -outputresource:$(BINFILE_DEMO_POWERTRAIN);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_POWERTRAIN).manifest
	
	

all: $(BINFILE_DEMO_BUILDSYSTEM) \
     $(BINFILE_DEMO_POWERTRAIN)
     
purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_BUILDSYSTEM) \
             $(OFILE_DEMO_POWERTRAIN)
	-@$(DEL) $(BINFILE_DEMO_BUILDSYSTEM) \
             $(BINFILE_DEMO_POWERTRAIN)

