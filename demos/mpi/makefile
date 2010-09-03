# 
# CHRONO EXAMPLES: MPI
#
# Makefile for building Chrono examples about advanced MPI cluster computing
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
	# have been automatically set.
	

CPPFILE_DEMO_MPIBASIC = \
	demo_mpibasic.cpp 




OFILE_DEMO_MPIBASIC=$(CPPFILE_DEMO_MPIBASIC:.cpp=.o) 

BINFILE_DEMO_MPIBASIC="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_MPIBASIC:.cpp=.exe)"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"


$(BINFILE_DEMO_MPIBASIC): $(OFILE_DEMO_MPIBASIC)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_MPIBASIC) "$(CH_CHRONOLIB)" "$(CH_CHRONOMPILIB)" $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_MPIBASIC)
	$(MT) -manifest $(BINFILE_DEMO_MPIBASIC).manifest -outputresource:$(BINFILE_DEMO_MPIBASIC);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_MPIBASIC).manifest
	
	
	

all: $(BINFILE_DEMO_MPIBASIC)
     
     
purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_MPIBASIC)
	-@$(DEL) $(BINFILE_DEMO_MPIBASIC) 

