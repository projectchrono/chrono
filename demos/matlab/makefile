# 
# CHRONO MATLAB
#
# Makefile for building Chrono examples about interfacing to Matlab(TM)
# Do not compile this demo if you have not Matlab correctly installed
# on your system.
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

	
# Note! You must set the following CH_MATLABSDK variable to
# point to the directory where you have Matlab SDK installed.
# If you do not have Matlab installed, you cannot compile this 
# demo!
	
CH_MATLABSDK=C:/Programmi/MATLAB/R2006b/extern

CH_MATLABINC=-I"$(CH_MATLABSDK)/include"
CH_MATLABLIB=$(CH_MATLABSDK)/lib/win32/microsoft/libeng.lib \
             $(CH_MATLABSDK)/lib/win32/microsoft/libmx.lib


CPPFILE_DEMO_MATLAB = \
	demo_matlab.cpp 



OFILE_DEMO_MATLAB=$(CPPFILE_DEMO_MATLAB:.cpp=.o) 
BINFILE_DEMO_MATLAB="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_MATLAB:.cpp=.exe)"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) $(CH_MATLABINC) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"



all: $(OFILE_DEMO_MATLAB)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_MATLAB) $(CH_LIBRARIES) $(CH_MATLABLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_MATLAB)
	$(MT) -manifest $(BINFILE_DEMO_MATLAB).manifest -outputresource:$(BINFILE_DEMO_MATLAB);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_MATLAB).manifest

purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_MATLAB)
	-@$(DEL) $(BINFILE_DEMO_MATLAB)
