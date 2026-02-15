CXX := g++
CXXFLAGS := -std=c++11 -Wall -Wextra -pthread -Ipi/include

PI_SRC := $(wildcard pi/src/*.cpp)
PI_BIN := Alex-pi

.PHONY: all pi clean

all: pi

pi: $(PI_SRC)
	$(CXX) $(CXXFLAGS) $^ -o $(PI_BIN)

clean:
	rm -f $(PI_BIN)
