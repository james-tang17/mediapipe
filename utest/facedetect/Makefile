TOP = ../..

BIN = fd
SOURCES = fd.c facedetect.c
VPATH=.:$(TOP)

OBJECTS = $(SOURCES:.c=.o)

ifeq ($(BUILD_TYPE), rel)
CFLAGS = -O2 -DNDEBUG
else
CFLAGS = -O0 -g -DDEBUG
endif

CFLAGS += -Wall -Werror
CFLAGS += -I$(TOP)

ifneq ($(OPENCV_INC),)
CFLAGS += -I$(OPENCV_INC)
endif

OPENCV_LIB=`pkg-config --libs opencv`

%.o: %.c
	$(CXX) $(CFLAGS) -c $< -o $@

$(BIN): $(OBJECTS)
	$(CXX) -o $@ $(CFLAGS) $^ $(OPENCV_LIB)

clean:
	rm -rf $(BIN) $(OBJECTS)
