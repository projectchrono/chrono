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
	#   CH_CHRONOLIB (the Chrono API library to be linked), 
	#   CH_INCLUDES (directories of header files, needed by compiler), 
	#   CH_LINKEXEFLAGS (needed flags for linker, to make executables),
	#   CH_COMPILERFLAGS (needed flags for the compiler), 
	#   CH_COMPILERFLAG_FILEOUT (flag to set compiler output compiler,ex.=-Fo for MSVC)
	#   CC (the command to invoke the compiler,
	#   LD (the command to invoke the linker). 
	# Then, from now on, you can use them in your makefile,
	# as in the example below.



##############

CPPFILE_DEMO_LCPSOLVER = \
	demo_LCPsolver.cpp 


OFILE_DEMO_LCPSOLVER=$(CPPFILE_DEMO_LCPSOLVER:.cpp=.o) 

BINFILE_DEMO_LCPSOLVER="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_LCPSOLVER:.cpp=.exe)"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"



all: $(OFILE_DEMO_LCPSOLVER) $(CPPFILE_DEMO_LCPSOLVER)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_LCPSOLVER) $(CH_CHRONOLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_LCPSOLVER)
	$(MT) -manifest $(BINFILE_DEMO_LCPSOLVER).manifest -outputresource:$(BINFILE_DEMO_LCPSOLVER);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_LCPSOLVER).manifest


purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_LCPSOLVER) 
	-@$(DEL) $(BINFILE_DEMO_LCPSOLVER)

