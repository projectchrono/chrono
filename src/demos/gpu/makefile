# 
# CHRONO EXAMPLES: GPU
#
# Makefile for building Chrono examples about GPU computing
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
	

CPPFILE_DEMO_GPUBASIC = \
	demo_gpubasic.cpp 




OFILE_DEMO_GPUBASIC=$(CPPFILE_DEMO_GPUBASIC:.cpp=.o) 

BINFILE_DEMO_GPUBASIC="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_GPUBASIC:.cpp=.exe)"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"


$(BINFILE_DEMO_GPUBASIC): $(OFILE_DEMO_GPUBASIC)
	$(LD)  $(CH_LINKEXEFLAGS) $(OFILE_DEMO_GPUBASIC) "$(CH_CHRONOLIB)" "$(CH_CHRONOCUDALIB)" $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_GPUBASIC)
	$(MT) -manifest $(BINFILE_DEMO_GPUBASIC).manifest -outputresource:$(BINFILE_DEMO_GPUBASIC);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_GPUBASIC).manifest
	
	
	

all: $(BINFILE_DEMO_GPUBASIC)
     
     
purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_GPUBASIC)
	-@$(DEL) $(BINFILE_DEMO_GPUBASIC) 

