INC_DIR := include
SRC_DIR := src
OBJ_DIR := obj
BIN_DIR := bin

CXXFLAGS := -Iinclude 
CFLAGS   := -Wall
LDFLAGS  := -Llib
LDLIBS   := -lm

.PHONY: all clean

all: state_machine_tests 

# STATE MACHINE TESTS ---------------------------------------------

state_machine_tests: state_machine_tests.cpp $(OBJ_DIR)/ExampleStateMachines.o $(BIN_DIR)
	g++ -o $(BIN_DIR)/state_machine_tests state_machine_tests.cpp  $(OBJ_DIR)/ExampleStateMachines.o $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(OBJ_DIR)/ExampleStateMachines.o: ExampleStateMachines.cpp ExampleStateMachines.h $(INC_DIR)/StateMachine.h $(OBJ_DIR)
	g++ -o $(OBJ_DIR)/ExampleStateMachines.o -c ExampleStateMachines.cpp $(LDFLAGS) $(LDLIBS) $(CXXFLAGS) $(CFLAGS)

$(BIN_DIR) $(OBJ_DIR):
	mkdir -p $@

clean:
	@$(RM) -rv $(BIN_DIR) $(OBJ_DIR)

-include $(OBJ:.o=.d)
