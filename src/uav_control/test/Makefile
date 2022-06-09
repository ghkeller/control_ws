INC_DIR := include
SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

#EXE := $(BIN_DIR)/test
#SRC := $(wildcard $(SRC_DIR)/*.cpp)
#OBJ := $(SRC:$(SRC_DIR)/%.cpp=$(OBJ_DIR)/%.o)

CXXFLAGS := -Iinclude -I/opt/ros/melodic/include -I$(HOME)/catkin_ws/devel/include
CFLAGS   := -Wall
LDFLAGS  := -Llib
LDLIBS   := -lm

.PHONY: all clean

all: timer_tests parsing_tests position_target_scheme_tests flight_class_tests mission_state_machine_tests

# TIMER TEST ---------------------------------------------

timer_tests: timer_tests.cpp $(OBJ_DIR)/Timer.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/timer_tests timer_tests.cpp $(OBJ_DIR)/Timer.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/Timer.o: $(SRC_DIR)/Timer.cpp $(INC_DIR)/Timer.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/Timer.o -c $(SRC_DIR)/Timer.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

# PARSING FLIGHT FILES TEST ---------------------------------------------

parsing_tests: parsing_tests.cpp $(OBJ_DIR)/Parsing.o $(OBJ_DIR)/SetpointScheme.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/parsing_tests parsing_tests.cpp $(OBJ_DIR)/Parsing.o $(OBJ_DIR)/SetpointScheme.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/Parsing.o: $(SRC_DIR)/Parsing.cpp $(INC_DIR)/Parsing.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/Parsing.o -c $(SRC_DIR)/Parsing.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

# MISSION STATE MACHINE TEST ---------------------------------------------

mission_state_machine_tests: mission_state_machine_tests.cpp $(OBJ_DIR)/StateMachine.o $(OBJ_DIR)/MissionStateMachine.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/mission_state_machine_tests mission_state_machine_tests.cpp $(OBJ_DIR)/StateMachine.o $(OBJ_DIR)/MissionStateMachine.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/MissionStateMachine.o: $(SRC_DIR)/MissionStateMachine.cpp $(INC_DIR)/MissionStateMachine.h $(INC_DIR)/StateMachine.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/MissionStateMachine.o -c $(SRC_DIR)/MissionStateMachine.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/StateMachine.o: $(SRC_DIR)/StateMachine.cpp $(INC_DIR)/StateMachine.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/StateMachine.o -c $(SRC_DIR)/StateMachine.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

# POSITION TARGET SCHEME ---------------------------------------------

position_target_scheme_tests: position_target_scheme_test.cpp $(OBJ_DIR)/SetpointScheme.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/positon_target_scheme_tests position_target_scheme_test.cpp $(OBJ_DIR)/SetpointScheme.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/SetpointScheme.o: $(SRC_DIR)/SetpointScheme.cpp $(INC_DIR)/SetpointScheme.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/SetpointScheme.o -c $(SRC_DIR)/SetpointScheme.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

# FLIGHT CLASS --------------------------------------------------------

flight_class_tests: flight_class_tests.cpp $(OBJ_DIR)/Flight.o $(OBJ_DIR)/FlightParsing.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/flight_class_tests flight_class_tests.cpp $(OBJ_DIR)/Flight.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)
	
$(OBJ_DIR)/Flight.o: $(SRC_DIR)/Flight.cpp $(INC_DIR)/Flight.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/Flight.o -c $(SRC_DIR)/Flight.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/FlightParsing.o: $(SRC_DIR)/FlightParsing.cpp $(INC_DIR)/FlightParsing.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/FlightParsing.o -c $(SRC_DIR)/FlightParsing.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

# MISC ---------------------------------------------------------------

$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

clean:
	@$(RM) -rv $(BIN_DIR) $(OBJ_DIR)

-include $(OBJ:.o=.d)
