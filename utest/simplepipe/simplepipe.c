/*
* source -- capsfilter -- preproc(any src) --queue -+- sink
*                                                                           |
*                                                                          +
*                                                                     GstBuffer
*								                    +------->|jpeg encoder|
* 
* purpose: test wireframe after scale out. Check result by check saved jpeg picture.
*/

#include <gst/gst.h>
#include <gst/vaapi/videopreproc.h>
#include <jpeglib_interface.h>
#define USE_SMART_META 1

#define CHECK_RET(ret, val) \
if (ret != val) { \
   g_printerr("Error: failed at line: %d\n", __LINE__); \
   return -1; \
}


/***************************************************************************************************************************************************************/
typedef struct _MyChannelInfo
{
    gchar              src_pad_name[128];

    guint               width;
    guint               height;
}MyChannelInfo;

const MyChannelInfo myChannelInfo[] = 
{
    {"src",                 1920,  1080},  //0
    {"src_720p",            1280,  720},
    {"src_d1",              704,   576},
    {"src_cif",             352,   288}, //3
    {"src_480_270",         480,   270},
    {"src_352_198",         352,   198},
    {"src_480_272",         480,   272},
    {"src_352_200",         352,   200},	//7
    {"src_smart",           0,		0},	//8
};

#define TESTCHANNEL 0			/*0~8*/


#define PREPARED_WIRE_FRME_NUM 6
const GstVideoPreprocWireFrame wire_frames[PREPARED_WIRE_FRME_NUM] = {
      {{4,   2,    16,   18},  {88, 122, 33},  2, TRUE },
      {{500, 260,  210,  190}, {188,122, 133}, 2, TRUE },
      {{250, 200,  1600, 500}, {56, 78,  93},  2, TRUE },
      {{100, 160,  200,  100}, {18, 12,  213}, 4, TRUE },
      {{320, 100,  60,   70},  {96, 98,  193}, 2, TRUE },
      {{10,  20,   190,  280}, {122,244, 55},  6, TRUE },
   };


#ifndef USE_SMART_META
static void
configure_wire_frame_cb(GstVideoPreproc *preproc, GstClockTime ts, GstVideoPreprocBufferUsage usage, gpointer user_data);
#endif


static gboolean
my_bus_call (GstBus     *bus,
      GstMessage *msg,
      gpointer    data)
{
   GMainLoop *loop = (GMainLoop *) data;

   switch (GST_MESSAGE_TYPE (msg)) {
      case GST_MESSAGE_EOS:
		{
			g_print ("End of stream\n");
			g_main_loop_quit (loop);
			break;
      	}
      case GST_MESSAGE_ERROR: 
		{
			gchar  *debug;
			GError *error;

			gst_message_parse_error (msg, &error, &debug);
			g_free (debug);

			g_printerr ("Error: %s\n", error->message);
			g_error_free (error);

			g_main_loop_quit (loop);
			break;
		}
      default:
			break;
   }

   return TRUE;
}

static gboolean jpeg_inited = FALSE;

static GstPadProbeReturn
my_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info, 
                        gpointer user_data)
{
    GstVideoPreproc *preproc;
    GstBuffer *buf;
    GstVideoPreprocBuffer *preBuf;

	static guint frame_num = 0;
    gchar filename[128] = "";
	
    buf = info->data;

	preproc = GST_VIDEO_PREPROC((GstVideoPreproc *)user_data);

    if(buf == NULL || !GST_IS_BUFFER(buf))
    {
    	g_print("gst buffer invalid (%p).\n", buf);
        return GST_PAD_PROBE_DROP;
    }
    
    preBuf = gst_video_preproc_buffer_get_from_gst_buffer(
                    preproc,
                    buf);
	g_print("my probe cb.\n");
    
    //do something here
    /*jpeg encoder into jpg file, in convinience of 
    checking the result of feature -- "wire-frame after scaling".*/	
	if(preBuf != NULL && frame_num++ ==60 )
	{
		//
		if (!jpeg_inited) {
		   jpeg_init(USE_OPENCL);
		   jpeg_inited = TRUE;
		}
		
		//
		JpegInputBuffer jpeg_input_buf;

		jpeg_input_buf.handle = preBuf->handle;
		jpeg_input_buf.w = preBuf->w;
		jpeg_input_buf.h = preBuf->h;
		jpeg_input_buf.ystride = preBuf->ystride;
		jpeg_input_buf.uvstride = preBuf->uvstride;
		jpeg_input_buf.yoffset = preBuf->yoffset;
		jpeg_input_buf.uvoffset = preBuf->uvoffset;	

		sprintf(filename, "simplepipe%d.jpg", frame_num);
	    jpeg_encode_file(&jpeg_input_buf, 50, filename, NULL, 0, NULL);			
	}

    gst_video_preproc_destroy_buffer(
                    preproc,
                    preBuf);    
                  
    return GST_PAD_PROBE_OK;
}

/**********************************************************************************************************************************/

/*
	source -capsfilter - preproc - queue -sink
*/
int main (int   argc, char *argv[])
{
	GMainLoop *myloop;

	GstElement *mypipeline, *mysource, *mycaps_filter, *mypreproc, *myqueue, *mysink;

	GstCaps *caps;
	GstBus *mybus;
	guint bus_watch_id;

	gboolean ret;

	/*configuration*/
	gboolean enable_3a = FALSE;
	gint sensor_id = 3;
	gint capture_mode = 1;

	GstVideoPreprocWireFrame *cur_wire_frames;
	int testchan = TESTCHANNEL;

	if(argc>1)testchan = atoi(argv[1]);
	g_print("test channel %d\n", testchan);

	/* init */
	gst_init (&argc, &argv);

	/* create pipeline, add elements */
	mypipeline = gst_pipeline_new ("simple-pipe");

	//source
#if 0	
	mysource   = gst_element_factory_make ("v4l2src",		"my-source");
	g_object_set (G_OBJECT (mysource), "io-mode", 4, "sensor-id", sensor_id,
            "enable-3a", enable_3a, "capture-mode", capture_mode, NULL);
#else
	mysource   = gst_element_factory_make ("videotestsrc",       "my-source");
	g_object_set (G_OBJECT (mysource), "is-live", TRUE, NULL);	
#endif

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

   	//vaapi preproc
	mypreproc  = gst_element_factory_make ("vaapipreproc",     "my-preproc");

	//queue
	myqueue = gst_element_factory_make ("queue",     "my-queue");

	//sink
	mysink = gst_element_factory_make ("fakesink",  "my-fakesink");
	g_object_set (G_OBJECT (mysink), "async", FALSE, NULL);

	if (!mypipeline || !mysource || !mycaps_filter || !mypreproc || !myqueue || !mysink) {
		g_printerr ("%s: One element could not be created. Exiting.\n", __FUNCTION__);
		return -1;
	}

	/*enable rotation*/	
	//gst_video_preproc_set_rotate (GST_VIDEO_PREPROC(mypreproc), GST_VIDEO_PREPROC_ROTATE_90);

	/*enable wire-frame*/
	{
		guint i;

		cur_wire_frames = g_new0(GstVideoPreprocWireFrame, PREPARED_WIRE_FRME_NUM);

		gfloat sfh = (gfloat)myChannelInfo[testchan].width / 1920.0;
		gfloat sfv = (gfloat)myChannelInfo[testchan].height / 1080.0;

		for(i=0;i<PREPARED_WIRE_FRME_NUM;i++) {
			cur_wire_frames[i].region.x = sfh * wire_frames[i].region.x;
			cur_wire_frames[i].region.y = sfv * wire_frames[i].region.y;
			cur_wire_frames[i].region.w = sfh * wire_frames[i].region.w;
			cur_wire_frames[i].region.h = sfv * wire_frames[i].region.h;
			cur_wire_frames[i].border = wire_frames[i].border;
			cur_wire_frames[i].color = wire_frames[i].color;
			cur_wire_frames[i].enable = wire_frames[i].enable;

			//
			cur_wire_frames[i].region.x &= ~0x1;
			cur_wire_frames[i].region.y &= ~0x1;
			cur_wire_frames[i].region.w &= ~0x1;
			cur_wire_frames[i].region.h &= ~0x1;
			cur_wire_frames[i].border &= ~0x1;

			g_print ("{%d,   %d,    %d,   %d} \n", cur_wire_frames[i].region.x, cur_wire_frames[i].region.y, cur_wire_frames[i].region.w, cur_wire_frames[i].region.h);
		}		
			
#ifndef USE_SMART_META
		g_signal_connect (GST_VIDEO_PREPROC(mypreproc), "configure-wire-frame", G_CALLBACK (configure_wire_frame_cb), cur_wire_frames);
#endif 
	}

	//add elements into pipeline
	gst_bin_add_many (GST_BIN (mypipeline),mysource, mycaps_filter, mypreproc, myqueue, mysink, NULL);	

	ret = gst_element_link_many (mysource, mycaps_filter, mypreproc,NULL);
	CHECK_RET(ret, TRUE);
	
	ret = gst_element_link_pads_full (mypreproc, myChannelInfo[testchan].src_pad_name, myqueue, "sink", GST_PAD_LINK_CHECK_NOTHING);
	CHECK_RET(ret, TRUE);

	ret = gst_element_link(myqueue, mysink);
	CHECK_RET(ret, TRUE);
   
	/* add handler */
	myloop = g_main_loop_new (NULL, FALSE);
	mybus = gst_pipeline_get_bus (GST_PIPELINE (mypipeline));
	bus_watch_id = gst_bus_add_watch (mybus, my_bus_call, myloop);
	gst_object_unref (mybus);

	/*probe*/
	GstPad *pad;

	pad = gst_element_get_static_pad(mysink, "sink");
#if 1	
	gst_pad_add_probe (pad, 
					GST_PAD_PROBE_TYPE_BUFFER | GST_PAD_PROBE_TYPE_PUSH | GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM, 
					(GstPadProbeCallback)my_probe_callback, mypreproc/*userdata*/, NULL);
#else
	g_signal_connect (pad, 
					  "linked", 
					  G_CALLBACK (preproc_pad_linked_callback), 
					  pipe);

	g_signal_connect (pad, 
					  "unlinked", 
					  G_CALLBACK (src_preproc_pad_unlinked_callback), 
					  pipe);
#endif
	gst_object_unref(pad);

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
	g_source_remove (bus_watch_id);
	g_main_loop_unref (myloop);

	//
	jpeg_release();

	//
	g_free(cur_wire_frames);
	
	return 0;   
}



/**********************************************************************************************************************************************************************************/

#ifndef USE_SMART_META
static void
configure_wire_frame_cb(GstVideoPreproc *preproc, GstClockTime ts, GstVideoPreprocBufferUsage usage, gpointer user_data)
{
   int i;
   GstVideoPreprocWireFrame *cur_wire_frames = NULL;
   guint wire_frame_max = gst_video_preproc_get_wire_frame_num_supported  (preproc);
   
   g_print ("%s: ts : %" GST_TIME_FORMAT ", usage : %d \n", __FUNCTION__, GST_TIME_ARGS(ts), usage );

   /* we prepare 6 wire frames for testing */
   if(usage == GST_VIDEO_PREPROC_BUFFER_USAGE_STAGE1) {
   		cur_wire_frames = wire_frames;

		//donothing
		return;
   }
   else {
	   	//for each resolution, feed different wire frames.
	   	//
   		cur_wire_frames = (GstVideoPreprocWireFrame *)user_data;
   }

   for (i = 0; i < wire_frame_max && i < PREPARED_WIRE_FRME_NUM; i++){
      if (TRUE != gst_video_preproc_set_wire_frame (GST_VIDEO_PREPROC(preproc), i, usage, &cur_wire_frames[i])) {
         g_print ("set mask_regions[%d] failed\n", i);
         break;
      }
   }
}
#endif

