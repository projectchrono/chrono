# 
# CHRONO EXAMPLES: CORE FUNCTIONALITIES
#
# Makefile for building Chrono examples about 'core' features
# like file, streams, smart pointers, etc.
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

		



CPPFILE_DEMO_MATH = \
	demo_math.cpp 

CPPFILE_DEMO_STREAM = \
	demo_stream.cpp 

CPPFILE_DEMO_COORDS = \
	demo_coords.cpp 

CPPFILE_DEMO_SHAREDPTR = \
	demo_sharedptr.cpp 



OFILE_DEMO_MATH=$(CPPFILE_DEMO_MATH:.cpp=.o) 
OFILE_DEMO_STREAM=$(CPPFILE_DEMO_STREAM:.cpp=.o) 
OFILE_DEMO_COORDS=$(CPPFILE_DEMO_COORDS:.cpp=.o)  
OFILE_DEMO_SHAREDPTR=$(CPPFILE_DEMO_SHAREDPTR:.cpp=.o)  

BINFILE_DEMO_MATH="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_MATH:.cpp=.exe)"
BINFILE_DEMO_STREAM="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_STREAM:.cpp=.exe)"
BINFILE_DEMO_COORDS="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_COORDS:.cpp=.exe)"
BINFILE_DEMO_SHAREDPTR="$(CH_BINDIR)$(D)$(CPPFILE_DEMO_SHAREDPTR:.cpp=.exe)"

.SUFFIXES:
.SUFFIXES: .cpp .c .o

.cpp.o:
	$(CC) $(CH_COMPILERFLAGS) $(CH_INCLUDES) -c $< $(CH_COMPILERFLAG_FILEOUT)"$*.o"



all: $(OFILE_DEMO_MATH) $(OFILE_DEMO_STREAM) $(OFILE_DEMO_COORDS) $(OFILE_DEMO_SHAREDPTR)
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_MATH) $(CH_LIBRARIES) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_MATH)
	$(MT) -manifest $(BINFILE_DEMO_MATH).manifest -outputresource:$(BINFILE_DEMO_MATH);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_MATH).manifest
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_STREAM) $(CH_LIBRARIES) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_STREAM)
	$(MT) -manifest $(BINFILE_DEMO_STREAM).manifest -outputresource:$(BINFILE_DEMO_STREAM);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_STREAM).manifest
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_COORDS) $(CH_LIBRARIES) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_COORDS)
	$(MT) -manifest $(BINFILE_DEMO_COORDS).manifest -outputresource:$(BINFILE_DEMO_COORDS);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_COORDS).manifest
	$(LD) $(CH_LINKEXEFLAGS) $(OFILE_DEMO_SHAREDPTR) $(CH_LIBRARIES) $(CH_LINKERFLAG_FILEOUT)$(BINFILE_DEMO_SHAREDPTR)
	$(MT) -manifest $(BINFILE_DEMO_SHAREDPTR).manifest -outputresource:$(BINFILE_DEMO_SHAREDPTR);#1 $(S)\
	$(DEL) $(BINFILE_DEMO_SHAREDPTR).manifest

purge:
	-@$(DEL) -f .*~ *~ #*

clean:
	-@$(DEL) $(OFILE_DEMO_MATH) $(OFILE_DEMO_STREAM) $(OFILE_DEMO_COORDS) $(OFILE_DEMO_SHAREDPTR)
	-@$(DEL) $(BINFILE_DEMO_MATH) $(BINFILE_DEMO_STREAM) $(BINFILE_DEMO_COORDS) $(BINFILE_DEMO_SHAREDPTR)


	
