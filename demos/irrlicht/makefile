# 
# CHRONO EXAMPLES
#
# Makefile for building Chrono examples with Irrlicht 3D realtime preview
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

             

##############

CPPFILE_DEMO_COLLISION2 = \
	demo_collision2.cpp 

CPPFILE_DEMO_INCLINEDPLANE = \
	demo_inclinedplane.cpp 

CPPFILE_DEMO_ROLLING = \
	demo_rolling.cpp

CPPFILE_DEMO_POLYCOLLISION = \
	demo_polycollision.cpp
	
	
OFILE_DEMO_COLLISION2=$(CPPFILE_DEMO_COLLISION2:.cpp=.o) 
OFILE_DEMO_INCLINEDPLANE=$(CPPFILE_DEMO_INCLINEDPLANE:.cpp=.o)
OFILE_DEMO_ROLLING=$(CPPFILE_DEMO_ROLLING:.cpp=.o)
OFILE_DEMO_POLYCOLLISION=$(CPPFILE_DEMO_POLYCOLLISION:.cpp=.o)


BINFILE_DEMO_COLLISION2="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_COLLISION2:.cpp=.exe)"
BINFILE_DEMO_INCLINEDPLANE="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_INCLINEDPLANE:.cpp=.exe)"
BINFILE_DEMO_ROLLING="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_ROLLING:.cpp=.exe)"
BINFILE_DEMO_POLYCOLLISION="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_POLYCOLLISION:.cpp=.exe)"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"

     
$(BINFILE_DEMO_COLLISION2): $(OFILE_DEMO_COLLISION2)
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_COLLISION2) $(CH_LIBRARIES) $(CH_IRRLICHTLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_COLLISION2) -lpthread -lX11 -lGL
	$(MT) -manifest $(BINFILE_DEMO_COLLISION2).manifest -outputresource:$(BINFILE_DEMO_COLLISION2);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_COLLISION2).manifest

$(BINFILE_DEMO_INCLINEDPLANE):    $(OFILE_DEMO_INCLINEDPLANE)
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_INCLINEDPLANE) $(CH_LIBRARIES) $(CH_IRRLICHTLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_INCLINEDPLANE) -lpthread -lX11 -lGL
	$(MT) -manifest $(BINFILE_DEMO_INCLINEDPLANE).manifest -outputresource:$(BINFILE_DEMO_INCLINEDPLANE);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_INCLINEDPLANE).manifest

$(BINFILE_DEMO_ROLLING):    $(OFILE_DEMO_ROLLING)
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_ROLLING) $(CH_LIBRARIES) $(CH_IRRLICHTLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_ROLLING) -lpthread -lX11 -lGL
	$(MT) -manifest $(BINFILE_DEMO_ROLLING).manifest -outputresource:$(BINFILE_DEMO_ROLLING);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_ROLLING).manifest	

$(BINFILE_DEMO_POLYCOLLISION):    $(OFILE_DEMO_POLYCOLLISION)
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_POLYCOLLISION) $(CH_LIBRARIES) $(CH_IRRLICHTLIB) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_POLYCOLLISION) -lpthread -lX11 -lGL
	$(MT) -manifest $(BINFILE_DEMO_POLYCOLLISION).manifest -outputresource:$(BINFILE_DEMO_POLYCOLLISION);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_POLYCOLLISION).manifest		


all: $(BINFILE_DEMO_INCLINEDPLANE) \
     $(BINFILE_DEMO_ROLLING) \
     $(BINFILE_DEMO_POLYCOLLISION) \
     $(BINFILE_DEMO_COLLISION2)


purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_INCLINEDPLANE) \
	       $(OFILE_DEMO_ROLLING) \
	       $(OFILE_DEMO_POLYCOLLISION) \
	       $(OFILE_DEMO_COLLISION2)

	-@$(DEL) $(BINFILE_DEMO_COLLISION2) \
		$(BINFILE_DEMO_ROLLING) \
		$(BINFILE_DEMO_POLYCOLLISION) \
		$(BINFILE_DEMO_INCLINEDPLANE)
