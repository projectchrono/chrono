# Warnings
WFLAGS	:= -Wall -Wextra -Wsign-conversion -Wsign-compare
MOREFLAGS	:= -lm -fopenmp #-g -pg -no-pie 
# Optimization and architecture
OPT		:= -O3 -mavx -msse #-fno-inline-small-functions 
ARCH   	:= -march=native

# Language standard
CCSTD	:= -std=c99
CXXSTD	:= -std=c++11

# Linker options
LDOPT 	:= $(OPT) 
LDFLAGS := -g -pg
EXEC 	:=  distance distance_gpu
.DEFAULT_GOAL := all

.PHONY: debug
debug : OPT  := -O0 -g 
debug : clean $(EXEC)

all : Makefile $(EXEC)

distance: distance.cpp Makefile timer.h # Rebuild on Makefile or header changes
	$(CXX) $(MOREFLAGS) $(CXXSTD) $(WFLAGS) $(FLAGS) $(OPT) $(CXXFLAGS) $< -o $@
	
distance_gpu: distance_gpu.cu Makefile timer.h # Rebuild on Makefile or header changes
	nvcc -O3 -Xcompiler -fopenmp $< -o $@

.PHONY: clean
clean:
	rm -f *.o $(EXEC)
