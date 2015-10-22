# Variables
MACHINE    := $(shell uname -m)
CC         := g++
BIN        := $(PWD)/bin
DEF_CFLAGS := -std=c++11 -m32 -Wall -I$(PWD)/contrib/PracticalSocket 
DEF_CFLAGS += -I$(PWD)/contrib/pugixml/pugixml-1.6/src -I$(PWD)/src 
ifeq (($MACHINE),armv7l)
   $(info Adding extra flags for uZed compilation...)
   DEF_CFLAGS += -DDATA_READER
   DEF_CFLAGS += -I$(PWD)/contrib/linux_dma/zynq-xdma/dev
   DEF_CFLAGS += -I$(PWD)/src -I$(PWD)/contrib/linux_dma/zynq-xdma/lib 
endif
DEF_LFLAGS := -lpthread 
#-lrt
#DEF_LFLAGS := -lpthread -lrt -lbz2 


# TPB Build
ifeq ($(MACHINE), armv7l)
   $(info ======================)
   $(info ++++Compiling for uZed)
   $(info ======================)
   DEF      := -DARM
   CFLAGS   := -g -O0 $(DEF_CFLAGS)
   LFLAGS   := $(DEF_LFLAGS) -L/usr/local/lib -lxdma
   EXT_TARS := 
   CLN      := 
   UTL_DIR  := $(PWD)/util/ptb
   OBJ      := $(PWD)/.ptb_obj
# Server Build
else
   $(info +++ Compiling for Server) 
   DEF      :=
   CFLAGS   := $(DEF_CFLAGS) -fpermissive -g
   LFLAGS   := $(DEF_LFLAGS)
   EXT_TARS := 
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

# Tests
TEST_DIR := $(PWD)/test
TEST_SRC := $(wildcard $(TEST_DIR)/*.cpp)
TEST_HDR := $(wildcard $(TEST_DIR)/*.h)
TEST_OBJ := $(patsubst $(TEST_DIR)/%.cpp,$(OBJ)/%.o,$(TEST_SRC))
TEST_BIN := $(patsubst $(TEST_DIR)/%.cpp,$(TEST_DIR)/%,$(TEST_SRC))

# Util Sources
UTL_SRC := $(wildcard $(UTL_DIR)/*.cpp)
UTL_BIN := $(patsubst $(UTL_DIR)/%.cpp,$(BIN)/%,$(UTL_SRC))

all: dir $(XML_OBJ) $(TCP_OBJ) $(PTB_OBJ) $(TEST_BIN)

dir:
	test -d $(OBJ) || mkdir $(OBJ)

clean: $(CLN)
	rm -rf $(OBJ)
	rm -f $(UTL_BIN)
	rm -f $(TEST_BIN)

server_clean:
	echo ""

## No library is created. Everything is compiled into a single object
# Compile XML parser Sources
$(OBJ)/%.o: $(XML_DIR)/%.cpp $(XML_DIR)/%.hpp
	$(CC) -c $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) -o $@ $<

# Compile TCP socket Sources
$(OBJ)/%.o: $(TCP_DIR)/%.cpp $(TCP_DIR)/%.h
	$(CC) -c $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) -o $@ $<

# Compile PTB Sources
$(OBJ)/%.o: $(PTB_DIR)/%.cpp $(PTB_DIR)/%.h
	$(CC) -c $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) -o $@ $<

# Compile utilities
#$(BIN)/%: $(UTL_DIR)/%.cpp $(XML_OBJ) $(TCP_OBJ) $(PTB_OBJ)
#	$(CC) $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) $(OBJ)/* -o $@ $< $(LFLAGS) 


# Compile tests
$(TEST_DIR)/%: $(TEST_DIR)/%.cpp $(XML_OBJ) $(PTB_OBJ) $(TCP_OBJ)
#	echo " <: $< "
#	echo " @: $@ "
#	echo " ?: $? "
#	echo " %: $% "
#	echo " ^: $^ "
#	echo " *:  $*"
#	echo " $(GEN_OBJ)"
	$(CC) $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) $(OBJ)/* -o $@ $< $(LFLAGS) 

# Compile server


