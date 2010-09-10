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

CPPFILE_DEMO_LCP_SChWARZ = \
	demo_lcp_schwarz.cpp 
	


OFILE_DEMO_MPIBASIC=$(CPPFILE_DEMO_MPIBASIC:.cpp=.o) 
OFILE_DEMO_LCP_SChWARZ=$(CPPFILE_DEMO_LCP_SChWARZ:.cpp=.o) 

BINFILE_DEMO_MPIBASIC="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_MPIBASIC:.cpp=.exe)"
BINFILE_DEMO_LCP_SChWARZ="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_LCP_SChWARZ:.cpp=.exe)"


.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"


$(BINFILE_DEMO_MPIBASIC): $(OFILE_DEMO_MPIBASIC)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_MPIBASIC) "$(CH_CHRONOLIB)" "$(CH_CHRONOMPILIB)" $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_MPIBASIC)
	$(MT) -manifest $(BINFILE_DEMO_MPIBASIC).manifest -outputresource:$(BINFILE_DEMO_MPIBASIC);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_MPIBASIC).manifest
	
$(BINFILE_DEMO_LCP_SChWARZ): $(OFILE_DEMO_LCP_SChWARZ)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_LCP_SChWARZ) "$(CH_CHRONOLIB)" "$(CH_CHRONOMPILIB)" $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_LCP_SChWARZ)
	$(MT) -manifest $(BINFILE_DEMO_LCP_SChWARZ).manifest -outputresource:$(BINFILE_DEMO_LCP_SChWARZ);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_LCP_SChWARZ).manifest
	
	
	

all: $(BINFILE_DEMO_MPIBASIC) \
	 $(BINFILE_DEMO_LCP_SChWARZ)
     
     
purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_MPIBASIC) \
			 $(OFILE_DEMO_LCP_SChWARZ)
	
	-@$(DEL) $(BINFILE_DEMO_MPIBASIC) \
			 $(BINFILE_DEMO_LCP_SChWARZ)

