# specify location of libraries
BOOST = /usr/include/boost
EIGEN = /usr/include/eigen
BSO = /home/tessa/toolbox
ALL_LIB = -I$(BOOST) -I$(EIGEN) -I$(BSO)

# compiler settings
CPP = g++ -std=c++14
FLAGS = -O3 -march=native -lglut -lGL -lGLU -lpthread

# specify file(s) to be compiled
MAINFILE = main.cpp

# specify name of executable
EXE = example

.PHONY: all cls clean

# definition of arguments for make command:
# this argument clears the screen, so that it is easier to find the beginning of the compiler output
cls:
	@clear
	@clear
# argument to call compiler and compile executable called "example"
all:
	$(CPP) -o example $(ALL_LIB) $(MAINFILE) $(FLAGS)

# remove previously compiled executable
clean:
	@rm -f $(EXE)
