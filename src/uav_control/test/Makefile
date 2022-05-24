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

all: position_target_scheme_tests flight_class_tests

# POSITION TARGET SCHEME ---------------------------------------------

position_target_scheme_tests: position_target_scheme_test.cpp $(OBJ_DIR)/PositionTargetScheme.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/positon_target_scheme_tests position_target_scheme_test.cpp $(OBJ_DIR)/PositionTargetScheme.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/PositionTargetScheme.o: $(SRC_DIR)/PositionTargetScheme.cpp $(INC_DIR)/PositionTargetScheme.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/PositionTargetScheme.o -c $(SRC_DIR)/PositionTargetScheme.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

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
