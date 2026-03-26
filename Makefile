CXX = g++
CXXFLAGS = -O3 -mavx512f -mavx512bw -msse4.2 -pthread -I./include
LDFLAGS = -pthread

# Define all targets
TARGET_SIM = vcu_sim
TARGET_SERVER = world_server
TARGET_NODE = vcu_node

# Common object file used by all targets
OBJ_PERCEPTION = src/perception.o

# Default rule: build all three programs
all: $(TARGET_SIM) $(TARGET_SERVER) $(TARGET_NODE)

# Target 1: The original multi-thread simulator
$(TARGET_SIM): src/main.o $(OBJ_PERCEPTION)
	$(CXX) src/main.o $(OBJ_PERCEPTION) -o $(TARGET_SIM) $(LDFLAGS)

# Target 2: The new UDP Physics Server
$(TARGET_SERVER): src/world_server.o $(OBJ_PERCEPTION)
	$(CXX) src/world_server.o $(OBJ_PERCEPTION) -o $(TARGET_SERVER) $(LDFLAGS)

# Target 3: The new UDP VCU Client Node
$(TARGET_NODE): src/vcu_node.o $(OBJ_PERCEPTION)
	$(CXX) src/vcu_node.o $(OBJ_PERCEPTION) -o $(TARGET_NODE) $(LDFLAGS)

# Generic rule for compiling .cpp to .o
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f src/*.o $(TARGET_SIM) $(TARGET_SERVER) $(TARGET_NODE)

.PHONY: all clean