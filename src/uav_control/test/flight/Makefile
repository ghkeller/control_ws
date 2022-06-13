INC_DIR := include
SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

CXXFLAGS := -Iinclude -I/opt/ros/melodic/include -I$(HOME)/catkin_ws/devel/include
CFLAGS   := -Wall
LDFLAGS  := -Llib
LDLIBS   := -lm

.PHONY: all clean

all: flight_class_tests 

# FLIGHT CLASS TESTS ---------------------------------------------

flight_class_tests: flight_class_tests.cpp $(OBJ_DIR)/ExampleFlight.o $(OBJ_DIR)/MAVROSComponents.o $(OBJ_DIR)/Flight.o $(OBJ_DIR)/SetpointScheme.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/flight_class_tests flight_class_tests.cpp  $(OBJ_DIR)/ExampleFlight.o $(OBJ_DIR)/MAVROSComponents.o $(OBJ_DIR)/Flight.o $(OBJ_DIR)/SetpointScheme.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/ExampleFlight.o: ExampleFlight.cpp ExampleFlight.h $(INC_DIR)/StateMachine.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/ExampleFlight.o -c ExampleFlight.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/Flight.o: $(SRC_DIR)/Flight.cpp $(INC_DIR)/Flight.h $(INC_DIR)/StateMachine.h $(OBJ_DIR)/MAVROSComponents.o $(OBJ_DIR)/SetpointScheme.o $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/Flight.o -c $(SRC_DIR)/Flight.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/MAVROSComponents.o: $(SRC_DIR)/MAVROSComponents.cpp $(INC_DIR)/MAVROSComponents.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/MAVROSComponents.o -c $(SRC_DIR)/MAVROSComponents.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/SetpointScheme.o: $(SRC_DIR)/SetpointScheme.cpp $(INC_DIR)/SetpointScheme.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/SetpointScheme.o -c $(SRC_DIR)/SetpointScheme.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

clean:
	@$(RM) -rv $(BIN_DIR) $(OBJ_DIR)

-include $(OBJ:.o=.d)
