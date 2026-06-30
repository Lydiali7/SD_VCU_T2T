# Compiler and Flags
CXX = g++
CXXFLAGS = -O3 -march=alderlake -mavx2 -msse4.2 -pthread -I./include
LDFLAGS = -pthread

# Targets (Only the active ones)
TARGET_SERVER = world_server
TARGET_NODE = vcu_node

# Object files (Only the active ones)
OBJ_PERCEPTION = src/perception.o
OBJ_SERVER = src/world_server.o
OBJ_NODE = src/vcu_node.o

# Default rule: build only active targets
all: $(TARGET_SERVER) $(TARGET_NODE)

# Link World Server
$(TARGET_SERVER): $(OBJ_SERVER) $(OBJ_PERCEPTION)
	$(CXX) $(OBJ_SERVER) $(OBJ_PERCEPTION) -o $(TARGET_SERVER) $(LDFLAGS)

# Link VCU Node
$(TARGET_NODE): $(OBJ_NODE) $(OBJ_PERCEPTION)
	$(CXX) $(OBJ_NODE) $(OBJ_PERCEPTION) -o $(TARGET_NODE) $(LDFLAGS)

# Generic rule for compiling .cpp to .o (matches src/*.cpp)
src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f src/*.o $(TARGET_SERVER) $(TARGET_NODE)

.PHONY: all clean