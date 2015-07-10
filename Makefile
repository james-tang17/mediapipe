# gstreamer 1.0 API
GST_LIB=$(shell pkg-config gstreamer-1.0 gstreamer-rtsp-server-1.0 gstreamer-rtsp-1.0 --libs)
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

# opecv API
OPENCV_LIB=-lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_superres -lopencv_imgcodecs -lopencv_videostab -lopencv_video

#rabbitmq API
RABBITMQ_LIB=-lrabbitmq -lm

#CC=gcc -g

CFLAGS+=$(GST_VIDEO_CFLAGS) $(GST_CFLAGS) $(GST_VAAPI_CFLAGS) $(MXML_CFLAGS) $(JPEG_CFLAGS) $(GST_APP_CFLAGS) -I.  -g -DDEBUG -Wall -fpermissive -std=gnu++11
LDFLAGS+=$(GST_VIDEO_LIB) $(GST_LIB) $(GST_VAAPI_LIB) $(MXML_LIB) $(JPEG_LIB) $(GST_APP_LIB) $(OPENCV_LIB) $(RABBITMQ_LIB)

CFLAGS += `pkg-config --cflags xcam_core`
LDFLAGS += '-lgstxcaminterface'  # /usr/lib/libgstxcaminterface.so

CFLAGS += -fopenmp
LDFLAGS += -fopenmp

SOURCES= mediapipe.c main.c facedetect.c utils.c cJSON.c videostab.c
OBJECTS=$(SOURCES:.c=.o)
BIN=mediapipe

all : $(BIN)

$(BIN) : $(OBJECTS)
	$(CXX)  $(CFLAGS) -o $@ $(OBJECTS) $(LDFLAGS)

.c.o:
	$(CXX) $(CFLAGS)   -c $< -o $@ 

clean:
	rm -f $(OBJECTS) $(BIN)
