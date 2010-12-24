# 
# Makefile for compiling all examples at once
#

default: all

include ../makes/config.mak

all depend strings clean:
	@echo "Making $@ in core"     $(S) cd core     $(S) $(CH_MAKETOOL) build=$(build) $@  $(S)  cd ..
	@echo "Making $@ in basics"   $(S) cd basics   $(S) $(CH_MAKETOOL) build=$(build) $@  $(S)  cd ..
	@echo "Making $@ in irrlicht" $(S) cd irrlicht $(S) $(CH_MAKETOOL) build=$(build) $@  $(S)  cd ..
	@echo "Making $@ in lcp_solver" $(S) cd lcp_solver $(S) $(CH_MAKETOOL) build=$(build) $@  $(S) cd ..
	$(CH_IF_MATLAB) @echo "Making $@ in matlab" $(S) cd matlab $(S) $(CH_MAKETOOL) build=$(build) $@  $(S) cd ..
	$(CH_IF_MPI) @echo "Making $@ in mpi" $(S) cd mpi $(S) $(CH_MAKETOOL) build=$(build) $@  $(S) cd ..
	$(CH_IF_GPU) @echo "Making $@ in gpu" $(S) cd gpu $(S) $(CH_MAKETOOL) build=$(build) $@  $(S) cd ..


