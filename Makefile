# NOTE: To use this file, make sure it's named "Makefile".
# BEGIN CONFIG SECTION
# Set this to the model name
# (e.g., "mip1" if your source file is "mip1.cpp")
MODEL_NAMES = rob_parallel
# END CONFIG SECTION

# BEGIN MAKEFILE
CCC = g++
# Options with and without debug info
CCOPT = -m64 -std=c++11 -g -DIL_STD
#CCOPT = -m64 -std=c++11 -O -DNDEBUG -DIL_STD -Wno-ignored-attributes

CONCERTDIR    = $(CPLEX_ROOT)/concert
CPLEXDIR      = $(CPLEX_ROOT)/cplex
CPOPTDIR      = $(CPLEX_ROOT)/cpoptimizer
CONCERTINCDIR = $(CONCERTDIR)/include
CPOPTINCDIR   = $(CPOPTDIR)/include
CCFLAGS = $(CCOPT) -I$(CPOPTINCDIR) -I$(CONCERTINCDIR)

SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic
CPOPTLIBDIR   = $(CPOPTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CCLNDIRS  = -L$(CPOPTLIBDIR) -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR)

CCLNFLAGS = -lcp -lcplex -lconcert -lm -lpthread -ldl

.PHONY: all
all: $(MODEL_NAMES)

$(MODEL_NAMES): % : %.cpp
	$(CCC) $(CCFLAGS) $(CCLNDIRS) -o $@ $< $(CCLNFLAGS)
.PHONY: clean
clean:
	rm -f $(MODEL_NAMES) *.o *~
