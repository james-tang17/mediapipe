/*
* source -- capsfilter -- encoder - sink
* 
* purpose: test wireframe after scale out. Check result by check saved jpeg picture.
*/
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>

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

/********************************************************************************************************************/
guint keyframe_period_global = 17; 


static gboolean
handle_keyboard (GIOChannel *source, GIOCondition cond, gpointer data)
{
   gchar *str = NULL;   
   GstElement *myencoder = (GstElement *)data;
   gboolean config = FALSE;

   if (g_io_channel_read_line (source, &str, NULL, NULL, NULL) == G_IO_STATUS_NORMAL) {	
	  if (str[0] == '+') {
	  	 keyframe_period_global += 10;
		 config = TRUE;
      }  else if (str[0] == '-') {
         keyframe_period_global -= 10;
		 config = TRUE;
      }  else {
         LOG_PRINT(" =========== mediapipe commands ==========");
         LOG_PRINT(" ===== '+' : increate keyframe period 10   =====");
         LOG_PRINT(" ===== '-' : decreate keyframe period 10   =====");
         LOG_PRINT(" =========================================");
      }

	  if(config) {
	  	guint period = 0;
	  	g_object_set (G_OBJECT(myencoder), "keyframe-period", keyframe_period_global, NULL);	 

		g_object_get(G_OBJECT(myencoder), "keyframe-period", &period, NULL);
		g_print ("The keyframe-period of the element is '%d'.\n", period);
	  }

   }

   g_free (str);
   return TRUE;
}


/********************************************************************************************************************/
/*
	source -capsfilter - preproc - queue -sink
*/
guint rate_control = 2;	/*1:CQP, 2: CBR*/
guint source_chosen = 1; /*0: VIDEOTESTSRC, 1: V4L2SRC*/
guint enable_roi = 1;

void usage_help(void)
{
	g_print("Usage:\n");
	g_print("-s [source] -c [ratecontrol] -r [roi-enable]\n");
}

int main (int   argc, char *argv[])
{
	GMainLoop *myloop;
	GIOChannel *io_stdin = NULL;

	GstElement *mypipeline, *mysource, *mycaps_filter, *myqueue, *myencoder, *mysink;

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
				if(rate_control > 2 || rate_control < 1) 
					rate_control = 2; 
				break;
			case 'r':
				enable_roi = atoi(optarg);
				if(enable_roi>0) enable_roi = 1;
				break;			
			default:
				usage_help();
				return;
		}
	}
	g_print("Command: \n source %d, rate-control %d, enable-ROI %d\n", 
		source_chosen, rate_control, enable_roi);

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
	     "width", G_TYPE_INT, 1920, "height", G_TYPE_INT, 1080, \
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
          "bitrate", 0, 
          "cabac", 1,
          "enable-mv", 0,
          "keyframe-period", keyframe_period_global,
          "max-bframes", 1,
          NULL);	

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
	
   
	/* add handler */
	myloop = g_main_loop_new (NULL, FALSE);

	/**/
    //io_stdin = g_io_channel_unix_new (fileno (stdin));
    //g_io_add_watch (io_stdin, G_IO_IN, (GIOFunc)handle_keyboard, myencoder);	

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

