# Variables
MACHINE    := $(shell uname -m)
CC         := g++
BIN        := $(PWD)/bin
DEF_CFLAGS := -std=c++11 -Wall -I$(PWD)/contrib/PracticalSocket 
DEF_CFLAGS += -I$(PWD)/contrib/linux_dma/pothos-zynq/driver -I$(PWD)/contrib/linux_dma/pothos-zynq/kernel
DEF_CFLAGS += -I$(PWD)/src -I./
#DEF_CFLAGS += -I$(PWD)/contrib/pugixml/pugixml-1.6/src
#DEF_CFLAGS += -I/usr/local/boost/1.66.00/include

BOOST_CFLAGS_SERVER := -I/usr/local/boost/1.66.00/include
BOOST_LFLAGS_SERVER := /usr/local/boost/1.66.00/lib/libboost_thread.a /usr/local/boost/1.66.00/lib/libboost_system.a
BOOST_FLAGS_CTB := /usr/lib/arm-linux-gnueabihf/libboost_thread.a /usr/lib/arm-linux-gnueabihf/libboost_system.a
BOOST_CFLAGS_CTB := 

DEF_LFLAGS := -lpthread
#-lrt
#DEF_LFLAGS := -lpthread -lrt -lbz2 


# TPB Build
ifeq ($(MACHINE), armv7l)
   $(info ==========================)
   $(info +++ Compiling for uZed +++)
   $(info ==========================)
    LFLAGS   := -g $(DEF_LFLAGS)
#   CFLAGS   := -g -O0 $(DEF_CFLAGS)
   CFLAGS   := -O3 -march=armv7-a $(DEF_CFLAGS)
#   CFLAGS   := -O1 -march=armv7-a -mcpu=cortex-a9 $(DEF_CFLAGS)
#   CFLAGS   := -g -march=armv7-a -O0 $(DEF_CFLAGS) $(DMA_TYPE)
   EXT_TARS := 
   CLN      := 
   UTL_DIR  := $(PWD)/util/ptb
   OBJ      := $(PWD)/.ptb_obj
# Server Build
else 
   $(info ============================)
   $(info +++ Compiling for Server +++)
   $(info ============================)
#   CC         := arm-xilinx-linux-gnueabi-g++ 
   ifdef CROSS_COMPILE
      CC       := $(CROSS_COMPILE)g++
      CFLAGS   := $(DEF_CFLAGS) -fpermissive
      LFLAGS   := 
   else
      CC       := g++
      CFLAGS   := $(DEF_CFLAGS) -fpermissive -m32 $(BOOST_CFLAGS_SERVER)
      LFLAGS   := $(DEF_LFLAGS) $(BOOST_LFLAGS_SERVER)
   endif
   DEF      := 
#   CFLAGS   := $(DEF_CFLAGS) -fpermissive -g
#   CFLAGS   := $(DEF_CFLAGS) -fpermissive -g -O0 -m32 
   #CFLAGS   += $(DEF_CFLAGS) -fpermissive -m32
   #LFLAGS   += $(DEF_LFLAGS) 
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
#XML_DIR := $(PWD)/contrib/pugixml/pugixml-1.6/src
#XML_SRC := $(wildcard $(XML_DIR)/*.cpp)
#XML_HDR := $(wildcard $(XML_DIR)/*.hpp)
#XML_OBJ := $(patsubst $(XML_DIR)/%.cpp,$(OBJ)/%.o,$(XML_SRC))

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

# Application
APP_DIR := $(PWD)/bin
APP_SRC := $(wildcard $(APP_DIR)/*.cpp)
APP_HDR := $(wildcard $(APP_DIR)/*.h)
APP_OBJ := $(patsubst $(APP_DIR)/%.cpp,$(OBJ)/%.o,$(APP_SRC))
APP_BIN := $(patsubst $(APP_DIR)/%.cpp,$(APP_DIR)/%,$(APP_SRC))

# Tests
TEST_DIR := $(PWD)/test
TEST_SRC := $(wildcard $(TEST_DIR)/*.cpp)
TEST_HDR := $(wildcard $(TEST_DIR)/*.h)
TEST_OBJ := $(patsubst $(TEST_DIR)/%.cpp,$(OBJ)/%.o,$(TEST_SRC))
TEST_BIN := $(patsubst $(TEST_DIR)/%.cpp,$(TEST_DIR)/%,$(TEST_SRC))

# Util Sources
UTL_SRC := $(wildcard $(UTL_DIR)/*.cpp)
UTL_BIN := $(patsubst $(UTL_DIR)/%.cpp,$(BIN)/%,$(UTL_SRC))

all: dir $(TCP_OBJ) $(PTB_OBJ) $(APP_BIN)

dir:
	test -d $(OBJ) || mkdir $(OBJ)

clean: $(CLN)
	rm -rf $(OBJ)
	rm -f $(UTL_BIN)
	rm -f $(TEST_BIN)
	rm -rf $(APP_BIN)

server_clean:
	echo ""

## No library is created. Everything is compiled into a single object
# Compile XML parser Sources
#$(OBJ)/%.o: $(XML_DIR)/%.cpp $(XML_DIR)/%.hpp
#	$(CC) -c $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) -o $@ $<

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
$(APP_DIR)/%: $(APP_DIR)/%.cpp $(XML_OBJ) $(PTB_OBJ) $(TCP_OBJ)
	$(CC) $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) $(OBJ)/* -o $@ $< $(LFLAGS) 

# Compile tests
#$(TEST_DIR)/%: $(TEST_DIR)/%.cpp $(XML_OBJ) $(PTB_OBJ) $(TCP_OBJ)
#	echo " <: $< "
#	echo " @: $@ "
#	echo " ?: $? "
#	echo " %: $% "
#	echo " ^: $^ "
#	echo " *:  $*"
#	echo " $(GEN_OBJ)"
#	$(CC) $(CFLAGS) -D__FILENAME__=\"$(notdir $<)\" $(DEF) $(OBJ)/* -o $@ $< $(LFLAGS) 

# Compile server


