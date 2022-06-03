#
# Makefile for ilitek_ld_tool
#

program := ilitek_ld
objects := ILITek_Main.c \
	   ILITek_Wifi.c \
	   ILITek_Device.c \
	   ILITek_Protocol_3X.c \
	   ILITek_Protocol_6X.c \
	   ILITek_Debug.c \
	   API/ILITek_Frequency.c \
	   API/ILITek_RawData.c \
	   API/ILITek_SensorTest.c \
	   API/ILITek_Upgrade.c \
	   API/ILITek_MpResult.c

libraries := stdc++ rt pthread m

include_path := ./include
source_path := ./src

CXX ?= gcc

CXXFLAGS = -Wall -ansi -O3 -g
CXXFLAGS += -D__ENABLE_DEBUG__
CXXFLAGS += -D__ENABLE_OUTBUF_DEBUG__
CXXFLAGS += -D__ENABLE_INBUF_DEBUG__
CXXFLAGS += -D__ENABLE_LOG_FILE_DEBUG__

CXXFLAGS += -DCONFIG_ILITEK_USE_LIBUSB
libraries += usb

CXXFLAGS += -static

INC_FLAGS += $(addprefix -I, $(include_path))
LIB_FLAGS += $(addprefix -l, $(libraries))
VPATH = $(include_path)
vpath %.h $(include_path)
vpath %.c $(source_path)
vpath %.cpp $(source_path)
.SUFFIXS: .c .cpp .h

.PHONY: all
all: $(objects)
	$(CXX) $^ $(CXXFLAGS) $(LIB_FLAGS) -o $(program)
	@chmod 777 $(program)

%.o: %.cpp
	$(CXX) -c $< $(CXXFLAGS) $(LIB_FLAGS)

.PHONY: clean
clean:
	@rm -rf $(program)
