INC_DIR := include
SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

CXXFLAGS := -Iinclude -I/opt/ros/melodic/include -I$(HOME)/catkin_ws/devel/include
CFLAGS   := -Wall
LDFLAGS  := -Llib
LDLIBS   := -lm

.PHONY: all clean

all: mavros_components_tests

# MAVROSComponents tests ---------------------------------------------

mavros_components_tests: mavros_components_tests.cpp $(OBJ_DIR)/MAVROSComponents.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/mavros_components_tests.cpp $(OBJ_DIR)/MAVROSComponents.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/MAVROSComponents.o: $(SRC_DIR)/MAVROSComponents.cpp $(INC_DIR)/MAVROSComponents.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/MAVROSComponents.o -c $(SRC_DIR)/MAVROSComponents.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

# MISC ---------------------------------------------------------------

$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

clean:
	@$(RM) -rv $(BIN_DIR) $(OBJ_DIR)

-include $(OBJ:.o=.d)
