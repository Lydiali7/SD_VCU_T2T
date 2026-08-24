# Compiler and Flags
CXX = g++
CXXFLAGS = -O3 -march=alderlake -mavx2 -msse4.2 -pthread -I./include -MMD -MP
LDFLAGS = -pthread

# Targets (Only the active ones)
TARGET_SERVER = world_server
TARGET_NODE = vcu_node
TARGET_TEST_ATP_ADAPTER = test_atp_adapter
TARGET_T2T_SNAPSHOT_DUMP = t2t_snapshot_dump
TARGET_MOCK_PEER = mock_peer

# Object files (Only the active ones)
OBJ_PERCEPTION = src/perception.o
OBJ_ATP_ADAPTER = src/atp_adapter.o
OBJ_SERVER = src/world_server.o
OBJ_NODE = src/vcu_node.o
OBJ_INFRA = src/infra/rt_system.o
DEPS = $(OBJ_PERCEPTION:.o=.d) $(OBJ_ATP_ADAPTER:.o=.d) $(OBJ_SERVER:.o=.d) $(OBJ_NODE:.o=.d) $(OBJ_INFRA:.o=.d)


# ... 保持其他规则不变 ...
# Default rule: build only active targets
all: $(TARGET_SERVER) $(TARGET_NODE) $(TARGET_TEST_ATP_ADAPTER) $(TARGET_T2T_SNAPSHOT_DUMP) $(TARGET_MOCK_PEER)

# Link World Server
$(TARGET_SERVER): $(OBJ_SERVER) $(OBJ_PERCEPTION)
	$(CXX) $(OBJ_SERVER) $(OBJ_PERCEPTION) -o $(TARGET_SERVER) $(LDFLAGS)

# Link VCU Node
# 链接 vcu_node 目标 (加入了 OBJ_INFRA)
$(TARGET_NODE): $(OBJ_NODE) $(OBJ_PERCEPTION) $(OBJ_ATP_ADAPTER) $(OBJ_INFRA)
	$(CXX) $(OBJ_NODE) $(OBJ_PERCEPTION) $(OBJ_ATP_ADAPTER) $(OBJ_INFRA) -o $(TARGET_NODE) $(LDFLAGS)

$(TARGET_TEST_ATP_ADAPTER): tests/test_atp_adapter.cpp $(OBJ_ATP_ADAPTER)
	$(CXX) $(CXXFLAGS) tests/test_atp_adapter.cpp $(OBJ_ATP_ADAPTER) -o $(TARGET_TEST_ATP_ADAPTER) $(LDFLAGS)

$(TARGET_T2T_SNAPSHOT_DUMP): tools/t2t_snapshot_dump.cpp
	$(CXX) $(CXXFLAGS) tools/t2t_snapshot_dump.cpp -o $(TARGET_T2T_SNAPSHOT_DUMP) $(LDFLAGS)

$(TARGET_MOCK_PEER): tools/mock_peer.cpp
	$(CXX) $(CXXFLAGS) tools/mock_peer.cpp -o $(TARGET_MOCK_PEER) $(LDFLAGS)

# Generic rule for compiling .cpp to .o (matches src/*.cpp)
src/%.o: src/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean rule
clean:
	rm -f src/*.o src/*.d src/infra/*.o src/infra/*.d $(TARGET_SERVER) $(TARGET_NODE) $(TARGET_TEST_ATP_ADAPTER) $(TARGET_T2T_SNAPSHOT_DUMP) $(TARGET_MOCK_PEER)

.PHONY: all clean

-include $(DEPS)
