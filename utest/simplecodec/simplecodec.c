/*
* source -- capsfilter -- encoder - sink
* 
* purpose: test wireframe after scale out. Check result by check saved jpeg picture.
*/
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>
#include <string.h>

#include <gst/gst.h>
#include <gst/vaapi/videopreproc.h>
#include <gst/vaapi/gstvaapiencoder.h>


#define CHECK_RET(ret, val) \
if (ret != val) { \
   g_printerr("Error: failed at line: %d\n", __LINE__); \
   return -1; \
}

#define LOG_PRINT(format, ...)   \
    printf (format "\n", ## __VA_ARGS__)

#define CONFIG_PICTURE_WIDTH 1920
#define CONFIG_PICTURE_HEIGHT 1080

/********************************************************************************************************************/
static guint keyframe_period_global = 100; 
static gint frame_bit_rate_global = 400; /* use kbps as input */
static guint enable_roi_global = 0;

static inline gboolean
reconfig_keyframe_period(GstElement *encoder)
{
	guint period = 0;
	g_object_set (G_OBJECT(encoder), "keyframe-period", keyframe_period_global, NULL);	   

	g_object_get(G_OBJECT(encoder), "keyframe-period", &period, NULL);
	g_print ("The keyframe-period of the element is '%d'.\n", period);
}

static inline gboolean
reconfig_bitrate(GstElement *encoder)
{
	guint frame_bit_rate = frame_bit_rate_global;	

	g_object_set (G_OBJECT(encoder), "bitrate", frame_bit_rate, NULL);	   

	g_object_get(G_OBJECT(encoder), "bitrate", &frame_bit_rate, NULL);
	g_print ("The bitrate change to '%d kbps'.\n", frame_bit_rate);
}

#define MAX_ROI_NUM 4
  gboolean
reconfig_roi(GstElement *encoder)
{
	GstVaapiParameterROI roi_param; 
	const GstVaapiROI roi_region[MAX_ROI_NUM] = {
			{{0, 0, 480, 270}, -20},
			{{960, 0, 480, 270}, -10},
			{{0, 540, 480, 270}, 10},
			{{960,540, 480, 270}, 20}
		};	
	guint i;
	guint reorder = 1;
	
	enable_roi_global = (enable_roi_global<=MAX_ROI_NUM)?enable_roi_global:MAX_ROI_NUM;	
	guint roi_num = enable_roi_global;

	memset(&roi_param, 1, sizeof(roi_param));
	roi_param.num_roi = roi_num;
	roi_param.roi = &roi_region[0];

	LOG_PRINT("reconfig_roi to %d regions.", roi_num);
	g_object_set (G_OBJECT(encoder), 
		"roi", &roi_param, 
		NULL);	   
}

static gboolean
handle_keyboard (GIOChannel *source, GIOCondition cond, gpointer data)
{
   gchar *str = NULL;   
   GstElement *myencoder = (GstElement *)data;

   if (g_io_channel_read_line (source, &str, NULL, NULL, NULL) == G_IO_STATUS_NORMAL) {	
	  if (str[0] == 'k' && 
	  	  str[1] == '+') {
	  	 keyframe_period_global += 10;
		 reconfig_keyframe_period(myencoder);
      }  else if (str[0] == 'k' &&
      			  str[1] == '-') {
         keyframe_period_global -= 10;
         reconfig_keyframe_period(myencoder);
      }  else if (str[0] == 'b' &&
      			  str[1] == '+') {
      	 frame_bit_rate_global = 1000;
         reconfig_bitrate(myencoder);
      }	 else if (str[0] == 'b' &&
      			  str[1] == '-') {
      	 frame_bit_rate_global = 200;
         reconfig_bitrate(myencoder);
      }  else if (str[0] == 'r' &&
      			  str[1] == '+') {
      	 enable_roi_global += 1;
         reconfig_roi(myencoder);
      }	 else if (str[0] == 'r' &&
      			  str[1] == '-') {
      	 enable_roi_global -= 1;
         reconfig_roi(myencoder);
      }  else {      
         LOG_PRINT(" =========== mediapipe commands ==========");
         LOG_PRINT(" ===== 'k+' : increate keyframe period 10   =====");
         LOG_PRINT(" ===== 'k-' : decreate keyframe period 10   =====");
         LOG_PRINT(" ===== 'b+' : increate bitrate to 4000kbps  =====");
         LOG_PRINT(" ===== 'b-' : decreate bitrate to 300kbps  =====");
         LOG_PRINT(" ===== 'r+' : increate roi regions  =====");
         LOG_PRINT(" ===== 'r-' : decreate roi regions  =====");		 
         LOG_PRINT(" =========================================");
      }
   }

   g_free (str);
   return TRUE;
}


/********************************************************************************************************************/
/*
	source -capsfilter - preproc - queue -sink
*/
guint rate_control = 2;	/*1:CQP, 2: CBR, 4: VBR*/
guint source_chosen = 1; /*0: VIDEOTESTSRC, 1: V4L2SRC*/
guint enable_roi = 1;

void usage_help(void)
{
	g_print("Usage:\n");
	g_print("-s <source (0:VideoTestSrc,1:V4L2Src)> -c <ratecontrol (1:CQP,2:CBR,4:VBR)> -r <roi-enable (num of roi regions)>\n");
}

int main (int   argc, char *argv[])
{
	GMainLoop *myloop;
	GIOChannel *io_stdin = NULL;

	GstElement *mypipeline, *mysource, *mycaps_filter, *myqueue, *myencoder, *mysink;
	GstElement *decqueue, *myparser, *mydecoder;

	GstCaps *caps;
	gboolean ret;

	/*configuration*/
	gboolean enable_3a = FALSE;
	gint sensor_id = 3;
	gint capture_mode = 1;

	/*args*/
	gint cmd;
	char* const short_options = "s:c:r:";
	struct option long_options[] = {	
		{ "source",	  1,   NULL,	's' 	},	
		{ "ratecontrol",  1,   NULL,	'c' 	},	
		{ "enable-roi",	  1,   NULL,	'r' 	},	
		{ NULL,	  0,	 NULL, 	0},	
	};
	
	while((cmd = getopt_long (argc, argv, short_options, long_options, NULL)) != -1) {
		switch (cmd) {
			case 's':
				source_chosen = atoi(optarg);
				if(source_chosen > 1)
					source_chosen = 1;
				break;
			case 'c':
				rate_control = atoi(optarg);
				if(rate_control != 1 && rate_control != 2 && rate_control != 4) 
					rate_control = 2; 
				break;
			case 'r':
				enable_roi = atoi(optarg);
				enable_roi %= (MAX_ROI_NUM+1);
				break;			
			default:
				usage_help();
				return;
		}
	}
	g_print("Command: \n source %d, rate-control %d, enable-ROI %d\n", 
		source_chosen, rate_control, enable_roi);
	enable_roi_global = enable_roi;

	/* init */
	gst_init (&argc, &argv);

	/* create pipeline, add elements */
	mypipeline = gst_pipeline_new ("simple-pipe");

	//source
	switch (source_chosen) {
	case 0:
		mysource   = gst_element_factory_make ("videotestsrc",       "my-source");
		g_object_set (G_OBJECT (mysource), "is-live", TRUE, NULL);	
		break;
	case 1:
	default:
		mysource   = gst_element_factory_make ("v4l2src",		"my-source");
		g_object_set (G_OBJECT (mysource), 
				"io-mode", 4, 
				"sensor-id", sensor_id,
	            "enable-3a", enable_3a, 
	            "capture-mode", capture_mode, 
	            "device", "/dev/video3", 
	            NULL);	
		break;
	}

	//caps filter
	mycaps_filter = gst_element_factory_make ("capsfilter",    "my-caps-filter");
	caps = gst_caps_new_simple ("video/x-raw",  \
	     "format", G_TYPE_STRING, "NV12",      \
	     "width", G_TYPE_INT, CONFIG_PICTURE_WIDTH, "height", G_TYPE_INT, CONFIG_PICTURE_HEIGHT, \
	     "framerate", GST_TYPE_FRACTION, 30, 1, \
	     "interlace-mode", G_TYPE_STRING, "progressive",  \
	     NULL);
	g_object_set (G_OBJECT (mycaps_filter), "caps", caps, NULL);
	gst_caps_unref (caps);

	//queue
	myqueue = gst_element_factory_make ("queue", "my-queue");
	g_object_set (myqueue, 
		"max-size-buffers", 0, 
		"max-size-time", (guint64)0, 
		"max-size-bytes", 0, 
		NULL);	

   	//vaapi h.264 encoder
	myencoder  = gst_element_factory_make ("vaapiencode_h264", "my-encoder");
    g_object_set (myencoder,
          "rate-control", rate_control, 
          "bitrate", frame_bit_rate_global, 
          "cabac", 1,
          "enable-mv", 0,
          "keyframe-period", keyframe_period_global,
          "max-bframes", 0,
          NULL);
	reconfig_roi(myencoder);

	if(0) {
		//sink
		mysink = gst_element_factory_make ("filesink",  "my-sink");
		g_object_set (G_OBJECT (mysink), 
			"location", "myencoder.264", 
			"async", FALSE, 
			NULL);

		if (!mypipeline || !mysource || !mycaps_filter || !myqueue || !myencoder || !mysink) {
			g_printerr ("%s: One element could not be created. Exiting.\n", __FUNCTION__);
			return -1;
		}

		//add elements into pipeline
		gst_bin_add_many (GST_BIN (mypipeline),mysource, mycaps_filter, myqueue, myencoder, mysink, NULL);	

		ret = gst_element_link_many (mysource, mycaps_filter, myqueue, myencoder, mysink, NULL);
		CHECK_RET(ret, TRUE);
	} else {
		//queue
		decqueue = gst_element_factory_make ("queue", "queue-encdec");
		g_object_set (decqueue, 
			"max-size-buffers", 0, 
			"max-size-time", (guint64)0, 
			"max-size-bytes", 0, 
			NULL);	
		
		//h264parse
		myparser = gst_element_factory_make ("h264parse", "myh264parse");
		
		//vaapidecoder
		mydecoder = gst_element_factory_make ("vaapidecode", "mydeocder");
		
		//vaapisink
		mysink = gst_element_factory_make ("vaapisink",  "my-sink");
		g_object_set (G_OBJECT (mysink), 
			"sync", FALSE, 
			NULL);

		if (!mypipeline || !mysource || !mycaps_filter || !myqueue || !myencoder || !decqueue || !myparser || !mydecoder || !mysink) {
			g_printerr ("%s: One element could not be created. Exiting.\n", __FUNCTION__);
			return -1;
		}

		//add elements into pipeline
		gst_bin_add_many (GST_BIN (mypipeline),mysource, mycaps_filter, myqueue, myencoder, decqueue, myparser, mydecoder, mysink, NULL);	

		ret = gst_element_link_many (mysource, mycaps_filter, myqueue, myencoder, decqueue, myparser, mydecoder, mysink, NULL);
		CHECK_RET(ret, TRUE);	
	}	
   
	/* add handler */
	myloop = g_main_loop_new (NULL, FALSE);

	/**/
    io_stdin = g_io_channel_unix_new (fileno (stdin));
    g_io_add_watch (io_stdin, G_IO_IN, (GIOFunc)handle_keyboard, myencoder);	

	/* Set the pipeline to "playing" state*/
	gst_element_set_state (mypipeline, GST_STATE_PLAYING);
	/* Iterate */
	g_print ("Running...\n");

	g_main_loop_run (myloop);
	
end:
	/* Out of the main loop, clean up nicely */
	g_print ("Returned, stopping playback\n");
	gst_element_set_state (mypipeline, GST_STATE_NULL);

	g_print ("Deleting pipeline\n");
	gst_object_unref (GST_OBJECT (mypipeline));
	g_main_loop_unref (myloop);

	return 0;   
}

