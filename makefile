#
# Makefile for ilitek_ld_tool
#

program := ilitek_ld
objects := ILITek_Main.c		\
	   ILITek_Device.c		\
	   ILITek_Protocol_3X.c		\
	   ILITek_Protocol_6X.c		\
	   API/ILITek_Upgrade.c

libraries := stdc++ rt pthread m

include_path := ./include
source_path := ./src

CXX ?= gcc

CXXFLAGS = -Wall -Wextra -O3 -g
CXXFLAGS += -D__ENABLE_DEBUG__
CXXFLAGS += -D__ENABLE_OUTBUF_DEBUG__
CXXFLAGS += -D__ENABLE_INBUF_DEBUG__
CXXFLAGS += -D__ENABLE_LOG_FILE_DEBUG__
CXXFLAGS += -static

##
##Selet using libusb: usb=y
ifeq ($(usb), y)
  CXXFLAGS += -DCONFIG_ILITEK_USE_LIBUSB
  libraries += usb
endif

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

.PHONY: help
help: makefile
	@sed -n 's/^##//p' $<
	@echo ''
