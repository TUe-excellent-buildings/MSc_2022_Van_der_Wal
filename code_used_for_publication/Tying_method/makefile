# specify location of libraries
BOOST   = /usr/include/boost
EIGEN   = /usr/include/eigen
BSO     = /usr/include/MSc_2022_Van_der_Wal/toolbox
ALL_LIB = -I$(BOOST) -I$(EIGEN) -I$(BSO)

# compiler settings
CPP   = g++ -std=c++14
FLAGS  = -O3 -march=native -lglut -lGL -lGLU -lpthread

# specify file(s) to be compiled
MAINFILE = main.cpp

.PHONY: all cls clean

# definition of arguments for make command
# argument to call compiler and compile executable called "main", example terminal command: "make cls clean all"
cls:
	# this argument clears the screen, so that it is easier to find the beginning of the compiler output
	@clear
	@clear
all:
	$(CPP) -o main $(ALL_LIB) $(MAINFILE) $(FLAGS)

# remove previously compiled executable
clean:
	@rm -f main
