# gstreamer 1.0 API
GST_LIB=$(shell pkg-config gstreamer-1.0 --libs)
GST_CFLAGS=$(shell pkg-config gstreamer-1.0 --cflags)

# gstreamer-video-1.0 API
GST_VIDEO_LIB=$(shell pkg-config gstreamer-video-1.0 --libs)
GST_VIDEO_CFLAGS=$(shell pkg-config gstreamer-video-1.0 --cflags)

# gstreamer-vaapi-1.2 API
GST_VAAPI_LIB=$(shell pkg-config gstreamer-vaapi-1.2 --libs)
GST_VAAPI_CFLAGS=$(shell pkg-config gstreamer-vaapi-1.2 --cflags)

# gstreamer-app-1.0 API
GST_APP_LIB=$(shell pkg-config gstreamer-app-1.0 --libs)
GST_APP_CFLAGS=$(shell pkg-config gstreamer-app-1.0 --cflags)

# Mini-xml API
MXML_LIB=$(shell pkg-config mxml --libs)
MXML_CFLAGS=$(shell pkg-config mxml --cflags)

# libjpeg_interface API
JPEG_LIB=$(shell pkg-config libjpeg_interface --libs)
JPEG_CFLAGS=$(shell pkg-config libjpeg_interface --cflags)

#CC=gcc -g

CFLAGS+=$(GST_VIDEO_CFLAGS) $(GST_CFLAGS) $(GST_VAAPI_CFLAGS) $(MXML_CFLAGS) $(JPEG_CFLAGS) $(GST_APP_CFLAGS) -I.  -g -O0 -DDEBUG -Wall 
LDFLAGS+=$(GST_VIDEO_LIB) $(GST_LIB) $(GST_VAAPI_LIB) $(MXML_LIB) $(JPEG_LIB) $(GST_APP_LIB)

SOURCES= simplecodec.c
OBJECTS=$(SOURCES:.c=.o)
BIN=simplecodec

all : $(BIN)

$(BIN) : $(OBJECTS)
	$(CC)  $(CFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

.c.o:
	$(CC) $(CFLAGS)   -c $< -o $@ 

clean:
	rm -f $(OBJECTS) $(BIN)
