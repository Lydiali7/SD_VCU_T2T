CXX = g++
CXXFLAGS = -O3 -mavx512f -mavx512bw -msse4.2 -pthread -I./include
LDFLAGS = -pthread

SRC = src/main.cpp src/perception.cpp
OBJ = $(SRC:.cpp=.o)
TARGET = vcu_sim

$(TARGET): $(OBJ)
	$(CXX) $(OBJ) -o $(TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f src/*.o $(TARGET)

.PHONY: clean