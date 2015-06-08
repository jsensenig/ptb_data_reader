# Variables
MACHINE    := $(shell uname -m)
CC         := g++
BIN        := $(PWD)/bin
DEF_CFLAGS := -Wall -I$(PWD)/contrib/PracticalSocket -I$(PWD)/contrib/pugixml/pugixml-1.6/src -I$(PWD)/src
DEF_LFLAGS := -lpthread -lrt
#DEF_LFLAGS := -lpthread -lrt -lbz2 


# TPB Build
ifeq ($(MACHINE), armv7l)
   $(info Compiling for uZed)
   DEF      := -DARM
   CFLAGS   := -g $(DEF_CFLAGS)
   LFLAGS   := $(DEF_LFLAGS)
   EXT_TARS := 
   CLN      := 
   UTL_DIR  := $(PWD)/util/ptb
   OBJ      := $(PWD)/.ptb_obj
# Server Build
else
   $(info Compiling for Server) 
   DEF      :=
   CFLAGS   := $(DEF_CFLAGS) -fpermissive
   LFLAGS   := $(DEF_LFLAGS) 
   EXT_TARS := gui 
   CLN      := server_clean
   UTL_DIR  := $(PWD)/util/server
   OBJ      := $(PWD)/.obj
endif

############
#
# Define directories to be compiled
#
############
# pugiXML library.
XML_DIR := $(PWD)/contrib/pugixml/pugixml-1.6/src
XML_SRC := $(wildcard $(XML_DIR)/*.cpp)
XML_HDR := $(wildcard $(XML_DIR)/*.hpp)
XML_OBJ := $(patsubst $(XML_DIR)/%.cpp,$(OBJ)/%.o,$(XML_SRC))

# Practical Socket
TCP_DIR := $(PWD)/contrib/PracticalSocket
TCP_SRC := $(wildcard $(TCP_DIR)/*.cpp)
TCP_HDR := $(wildcard $(TCP_DIR)/*.h)
TCP_OBJ := $(patsubst $(TCP_DIR)/%.cpp,$(OBJ)/%.o,$(TCP_SRC))

# My own sources
PTB_DIR := $(PWD)/src
PTB_SRC := $(wildcard $(PTB_DIR)/*.cpp)
PTB_HDR := $(wildcard $(PTB_DIR)/*.h)
PTB_OBJ := $(patsubst $(PTB_DIR)/%.cpp,$(OBJ)/%.o,$(PTB_SRC))

# Util Sources
UTL_SRC := $(wildcard $(UTL_DIR)/*.cpp)
UTL_BIN := $(patsubst $(UTL_DIR)/%.cpp,$(BIN)/%,$(UTL_SRC))

all: dir $(XML_OBJ) $(TCP_OBJ) $(PTB_OBJ)

dir:
	test -d $(OBJ) || mkdir $(OBJ)

clean: $(CLN)
	rm -rf $(OBJ)
	rm -f $(UTL_BIN)

server_clean:
	echo ""
	
## No library is created. Everything is compiled into a single object
# Compile XML parser Sources
$(OBJ)/%.o: $(XML_DIR)/%.cpp $(XML_DIR)/%.hpp
	$(CC) -c $(CFLAGS) $(DEF) -o $@ $<

# Compile TCP socket Sources
$(OBJ)/%.o: $(TCP_DIR)/%.cpp $(TCP_DIR)/%.h
	$(CC) -c $(CFLAGS) $(DEF) -o $@ $<

# Compile PTB Sources
$(OBJ)/%.o: $(PTB_DIR)/%.cpp $(PTB_DIR)/%.h
	$(CC) -c $(CFLAGS) $(DEF) -o $@ $<

# Comile utilities
#$(BIN)/%: $(UTL_DIR)/%.cpp $(XML_OBJ) $(TCP_OBJ) $(PTB_OBJ)
#	$(CC) $(CFLAGS) $(DEF) $(OBJ)/* -o $@ $< $(LFLAGS) 

