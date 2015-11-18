/*
 * main.c - Main for sample code
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Wind Yuan <feng.yuan@intel.com>
 */
#include <mxml.h>
#include <string.h>

#include <unistd.h>
#include "mediapipe.h"
#include "mediapipe_impl.h"
#include "osd_template.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <sstream>
#include <sys/stat.h>
#include <errno.h>
#include <cstdlib>
#include <ctime>
#include <jpeglib_interface.h>
#include <math.h>
#include <facedetect.h>

#include <amqp_tcp_socket.h>
#include <amqp.h>
#include <amqp_framing.h>
#include <assert.h>
#include "utils.h"
#include "cJSON.h"

using namespace std;

using namespace cv;
class MyCircle
{
public:
    Point center;
    int radius;
};

gboolean
media_pipe_set_mode (MediaPipe *pipe, int value);

#include <sys/time.h>
/* FPS Calculation for DEBUG */
#define FPS_CALCULATION(objname)                     \
    do{                                              \
        static guint num_frame = 0;                  \
        static struct timeval last_sys_time;         \
        static struct timeval first_sys_time;        \
        static int b_last_sys_time_init = FALSE;     \
        if (!b_last_sys_time_init) {                 \
          gettimeofday (&last_sys_time, NULL);       \
          gettimeofday (&first_sys_time, NULL);      \
          b_last_sys_time_init = TRUE;               \
        } else {                                     \
          if ((num_frame%50)==0) {                   \
            double total, current;                   \
            struct timeval cur_sys_time;             \
            gettimeofday (&cur_sys_time, NULL);      \
            total = (cur_sys_time.tv_sec - first_sys_time.tv_sec)*1.0f +       \
                   (cur_sys_time.tv_usec - first_sys_time.tv_usec)/1000000.0f; \
            current = (cur_sys_time.tv_sec - last_sys_time.tv_sec)*1.0f +      \
                    (cur_sys_time.tv_usec - last_sys_time.tv_usec)/1000000.0f; \
            printf("%s Current fps: %.2f, Total avg fps: %.2f\n",              \
                    #objname, (float)50.0f/current, (float)num_frame/total);   \
            last_sys_time = cur_sys_time;            \
          }                                          \
        }                                            \
        ++num_frame;                                 \
    }while(0)

#define LOG_ERROR(format, ...)    \
    printf ("ERROR :" format "\n", ## __VA_ARGS__)

#define LOG_WARNING(format, ...)   \
    printf ("WARNING:" format "\n", ## __VA_ARGS__)

#define LOG_PRINT(format, ...)   \
    printf (format "\n", ## __VA_ARGS__)

#ifndef DEBUG
#define LOG_DEBUG(format, ...)
#else
#define LOG_DEBUG(format, ...)   \
    printf ("DEBUG :" format "\n", ## __VA_ARGS__)
#endif

#define CONFIG_FILE_NAME "/etc/mediapipe/conf.xml"

#define SUPPORT_MV 1

#if SUPPORT_MV
#include <gst/vaapi/gstvaapimvmeta.h>
#endif

static gboolean
handle_keyboard (GIOChannel *source, GIOCondition cond, gpointer data);
static gboolean
parse_3aconf_callback(Config3A *config_camera);


const gchar *src_name[SRC_NUM] = {
   "videotestsrc",
   "xcamsrc",
   "filesrc"
};

gboolean smart_bright = FALSE;
gfloat smart_factor = 0.25;
int dump_smart_analysis_raw_data = 0;
gboolean smart_control = FALSE;
gboolean enable_qos = FALSE;
int face_detect_number = 0;
int face_getpic_number = 0;
int face_getpic_frame_number = 0;
int face_getpic_state = 0;
string dirpath;
char person_name_c[255];
int pnum;
unsigned int warning_level = 3;
unsigned int hdrtable_id = 1;
unsigned int hdrtable_rgb_id = 1;
unsigned int face_detect_interval = 10;

#define LOCATION_STR_LEN 128
#define DEVICE_STR_LEN 128
typedef struct _Src_Setting
{
    SrcType             src_type;
    gchar               location[LOCATION_STR_LEN];
    gboolean            use_v4l2_src;
    guint               v4l2_sensor_id;
    guint               v4l2_io_mode;
    gboolean            enable_3a;
    guint               capture_mode;
    gboolean            enable_autohdr;
    guint               autohdr_mode;

    guint               frame_rate;

    guint               run_time_in_sec;
    SmartResolution     smart_resolution;
    gchar               v4l2_device[DEVICE_STR_LEN];
    guint               image_processor;
    guint               analyzer;
    guint               cl_hdr_mode;
    guint               cl_denoise_mode;
    guint               cl_gamma_mode;
    gboolean            enable_dpc;
}Src_Setting;

typedef struct _Video_Channel_Config
{
    gboolean enable_channel;

    gboolean enable_h264;
    guint    rate_control;
    guint    bitrate;
    gboolean enable_cabac;
    gboolean enable_dct8x8;
    guint    mv;
    guint    gop_M;
    guint    gop_N;
    guint    profile;

    MediaSinkType sink_type;
    //for tcpclientsink
    gchar    host_ip[128];
    gint     port;
}Video_Channel_Config;

typedef struct _Video_Impl_Config
{
    GstVideoPreprocRotateMode   rotation;
    gboolean                    enable_mask;
    guint                       luma_gain; /*percentage of original luminance*/
    Video_Channel_Config        channel_config[VIDEO_CHANNEL_MAX];
    gboolean                    enable_wireframe;
}Video_Impl_Config;

typedef enum _JpegOutputMode
{
    JPEG_OUTPUT_BUFFER = 0,
    JPEG_OUTPUT_FILE
}JpegOutputMode;

typedef struct _JpgEnc_Misc_Config
{
	GMutex  		lock;

    JpegOutputMode jpg_out_mode;
    gint           jpg_interval;
    gint           jpeg_keyboard_flag;
    gint           jpeg_capture_flag;
    gchar          capture_filename[128];
    gboolean jpeg_crop_enable;
    gint    jpeg_crop_ox;
    gint    jpeg_crop_oy;
    gint    jpeg_crop_width;
    gint    jpeg_crop_height;
    //
}JpgEnc_Misc_Config;

typedef struct _Rect_Facedetect_Jpeg
{
int x;
int y;
int width;
int height;
int flag;  //indicate if face recognise data avaliable
}Rect_Facedetect_Jpeg;

Src_Setting src_setting = {(SrcType)0};
gboolean enable_facedetect;
gboolean facedetect_conf;
guint facedetect_mode;
gboolean enable_getfacepic;
gboolean enable_facerecognise;
gboolean enable_lumagain_threshold;
guint lumagain_threshold_value;
gboolean enable_hdr;
gboolean enable_hdr_custom;
guint hdr_darkpixels_value;
gboolean enable_hdr_custom_rgb;
GstVideoPreprocAutoHDRMode auto_hdr_mode;
gboolean enable_bright_compensation;
// global variable for libxcam CL mode, TODO: replace them with right logic when new libxcam is ready.
int xcam_mode;
int hdr_cl_mode;
int denoise_mode;
gboolean enable_dvs;

Video_Impl_Config config = {(GstVideoPreprocRotateMode)0};
JpgEnc_Misc_Config     jpgenc_misc_config;
JpgEnc_Misc_Config     jpgenc_misc_config2;
Rect_Facedetect_Jpeg rect_jpeg = {0,0,1920,1080,0};
unsigned int crop_jpeg_num = 0;
GstVideoPreprocFlipMode global_flip_mode = GST_VIDEO_PREPROC_FLIP_INVALID;
gboolean global_enable_osd = FALSE;
gboolean global_enable_mask = FALSE;
gboolean global_enable_wireframe = FALSE;
gboolean test_toggle_channel = FALSE;
gfloat rot_00;
gfloat rot_01;
gfloat rot_10;
gfloat rot_11;
gint dvs_offset_x = 0;
gint dvs_offset_y = 0;
int global_v4l2src_color_effect = 0;

const gchar *channel_names[VIDEO_CHANNEL_MAX] =
{
    "1080P  ",
    "JPEG   ",
    "720P   ",
    "D1     ",
    "CIF    ",
    "480x270",
    "352x198",
    "480x272",
    "352x200",
    "smart  "
};

GstVideoPreprocRegion src_region[OSD_REGION_MAX_NUM] = {
    {0, 128,   128,  64},
    {128, 16,   128,  64},
    {0,  16,   128,  64},
    {12,  200,  64,   32},
    {1000,300,  128,  128},
    {500, 1000, 1000, 32},
};

GstVideoPreprocOsdCfg dest_region_cfg[OSD_REGION_MAX_NUM] = {
    {{0, 128,   128,  64}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{128, 16,   128,  64}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{0,  16,   128,  64}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{32,  190, 64,    32}, 0, {COLOR_KEY_Y, 0, 0}, FALSE},
    {{1100,310, 128,   128},0, {COLOR_KEY_Y, 0, 0}, FALSE},
    {{552, 1004,1000,  32}, 0, {COLOR_KEY_Y, 0, 0}, FALSE},
};

/* we prepare 6 mask regions for testing */

typedef struct _WireFrameData
{
    guint reserve;
    GstClockTime ts;
    GstVideoPreprocWireFrame *wire_frames;
}WireFrameData;
//static GList *wire_frame_queue;
extern GstVideoPreprocWireFrame cv_wire_frames[WIRE_FRAME_REGION_MAX_NUM];
extern GMutex jpeg_lock;
extern GList *jpeg_frames;
extern GList *frames_do_jpeg_encoding;
/*
vector<Mat> images;
vector<int> labels;
vector<string> names;
map <int, string> label_name;
*/

/* rabbitmq integration */
#define MAX_MSG_LENGTH 512
#define UI_EXPOSE_WINDOW_NUM 3
char const *hostname;
int port, status;
char const *queuename_w2m;
char const *queuename_m2w;
amqp_socket_t *socket = NULL;
amqp_connection_state_t conn;
extern GstVideoPreprocMaskCfg sample_masks[MASK_REGION_MAX_NUM];
extern GstVideoPreprocWireFrame sample_wire_frames[MASK_REGION_MAX_NUM];

static gboolean
video_frame_jpeg_callback (GstVideoPreprocBuffer *video_buf, gpointer user_data, VideoChannelIndex channel);
static char *
capture_jpeg_frame (void);
static char *
capture_raw_frame (MediaPipe *pipe, int flag);
static void reset_hdr_rgb_table(guchar flag);
static void reset_hdr_table(guchar flag);
gboolean
mediapipe_set_jpegenc_crop (gboolean enable, gint cropOx, gint cropOy,
	gint cropWidth, gint cropHeight);
gboolean
mediapipe_set_jpegenc_output (JpegOutputMode outputmode, guint interval);

static void
rabbitmq_init()
{
	LOG_DEBUG("Enter rabbitmq-init\n");
	hostname = "localhost";
    port = 5672;
    queuename_w2m = "queue-w2m";
    queuename_m2w = "queue-m2w";

	conn = amqp_new_connection();
	socket = amqp_tcp_socket_new(conn);
	if (!socket) {
	die("creating TCP socket");
	}
	LOG_DEBUG("created TCP socket\n");

	status = amqp_socket_open(socket, hostname, port);
	if (status) {
	die("opening TCP socket");
	}
	LOG_DEBUG("opened TCP socket\n");

	die_on_amqp_error(amqp_login(conn, "/", 0, 131072, 0, AMQP_SASL_METHOD_PLAIN, "guest", "guest"),
		            "Logging in");
	LOG_DEBUG("amqp_logein\n");
	amqp_channel_open(conn, 1);
	die_on_amqp_error(amqp_get_rpc_reply(conn), "Opening channel");
	LOG_DEBUG("amqp_get_rpc_reply\n");

	amqp_queue_declare_ok_t *r = amqp_queue_declare(conn, 1, amqp_cstring_bytes(queuename_w2m), 0, 1, 0, 0,
                                 amqp_empty_table);
    die_on_amqp_error(amqp_get_rpc_reply(conn), "Declaring queue");
    LOG_DEBUG("Declared queue: %s\n", queuename_w2m);

    r = amqp_queue_declare(conn, 1, amqp_cstring_bytes(queuename_m2w), 0, 1, 0, 0,
                                 amqp_empty_table);
    die_on_amqp_error(amqp_get_rpc_reply(conn), "Declaring queue");
    LOG_DEBUG("Declared queue: %s\n", queuename_m2w);

	amqp_basic_consume(conn, 1, amqp_cstring_bytes(queuename_w2m), amqp_empty_bytes, 0, 1, 0, amqp_empty_table);
	LOG_DEBUG("amqp_basic_consume\n");
	die_on_amqp_error(amqp_get_rpc_reply(conn), "Consuming");
	LOG_DEBUG("amqp_get_rpc_reply\n");
}

gpointer
rabbitmq_listening_func (gpointer data)
{
	LOG_DEBUG("start mq listening...\n");
    //amqp_connection_state_t conn;
    //conn = (amqp_connection_state_t)data;

	MediaPipe *pipe = (MediaPipe *)data;
   	MediaPipeImpl *impl = IMPL_CAST (pipe);

    while (1) {
		amqp_rpc_reply_t res;
		amqp_envelope_t envelope;
		gboolean set_luma = FALSE;

		amqp_maybe_release_buffers(conn);

		res = amqp_consume_message(conn, &envelope, NULL, 0);

		if (AMQP_RESPONSE_NORMAL != res.reply_type) {
			LOG_DEBUG("Error in mq receiving! Exit receiving loop!");
			break;
		}
		/*
		printf("Delivery %u, exchange %.*s routingkey %.*s\n",
			 (unsigned) envelope.delivery_tag,
			 (int) envelope.exchange.len, (char *) envelope.exchange.bytes,
			 (int) envelope.routing_key.len, (char *) envelope.routing_key.bytes);

		if (envelope.message.properties._flags & AMQP_BASIC_CONTENT_TYPE_FLAG) {
		printf("Content-type: %.*s\n",
			   (int) envelope.message.properties.content_type.len,
			   (char *) envelope.message.properties.content_type.bytes);
		}*/
		printf("---Recieved msg---\n");
		amqp_dump(envelope.message.body.bytes, envelope.message.body.len);

		char *recmsg;
		char msgbuff[MAX_MSG_LENGTH];
		char retbuff[MAX_MSG_LENGTH];
		int gain = 0;
		cJSON *json = NULL;
		recmsg = (char*)envelope.message.body.bytes;
		LOG_DEBUG("message length: %d", envelope.message.body.len);

		if(envelope.message.body.len > MAX_MSG_LENGTH)
		{
			LOG_DEBUG("Message length too long!");
			sprintf(retbuff, "Message length too long!");
			goto end;
		}
		for(unsigned int i=0; i<envelope.message.body.len; i++)
		{
			msgbuff[i] = recmsg[i];
		}
		msgbuff[envelope.message.body.len] = '\0';
		LOG_DEBUG("message: %s", msgbuff);

		// parse json string
		json=cJSON_Parse(msgbuff);

		if (!json)
		{
			LOG_DEBUG("JSON parse Error before: [%s]\n",cJSON_GetErrorPtr());
		}
		else
		{
			cJSON *ptype = cJSON_GetObjectItem ( json, "type" );
			if(ptype == NULL)
			{
				LOG_DEBUG("JSON parse Error before: [%s]\n",cJSON_GetErrorPtr());
				sprintf(retbuff, "Invalid parameter!");
				goto end;
			}
			//char *stype = cJSON_Print( ptype );
			if(ptype->type == cJSON_String)
			{
				if(strcmp(ptype->valuestring, "config") == 0)
				{
					cJSON *ppara = cJSON_GetObjectItem ( json, "para" );
					if(ppara == NULL)
					{
						LOG_DEBUG("JSON parse Error before: [%s]\n",cJSON_GetErrorPtr());
						goto end;
					}
					if(strcmp(ppara->valuestring, "luma_gain") == 0)
					{
						cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							gain = pval->valueint;
							if(gain > 0 && gain < 300)
							{
								LOG_DEBUG("Setting luma gain to %d ...", gain);
								impl->preproc.luma_gain = gain;
								set_luma = TRUE;
							}
							else
							{
								LOG_DEBUG("Invalid parameter!");
								sprintf(retbuff, "Invalid parameter!");
							}
							if (set_luma == TRUE) {
								if(!gst_video_preproc_set_luma_gain(GST_VIDEO_PREPROC(impl->preproc.element),impl->preproc.luma_gain))
								{
									LOG_WARNING ("Set luma gain (%d) for preproc FAIL!", impl->preproc.luma_gain);
									sprintf(retbuff, "Set luma gain (%d) for preproc FAIL!", impl->preproc.luma_gain);
								}else {
									LOG_DEBUG ("Set luma gain (%d) for preproc SUCCEED!", impl->preproc.luma_gain);
									sprintf(retbuff, "Set luma gain (%d) for preproc SUCCEED!", impl->preproc.luma_gain);
								}
							}
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "hdr_mode") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
					    if(pval)
					    {
					        if(strcmp(pval->valuestring, "off") == 0)
					        {
					            enable_hdr = 0;
					            //src_setting.enable_autohdr = 0;
                                enable_hdr_custom = 0;
                                enable_hdr_custom_rgb = 0;
                                LOG_DEBUG ("set hdr mode: off");
                                sprintf(retbuff, "set hdr mode: off");
                                goto end;
					        }
					        else if(strcmp(pval->valuestring, "auto") == 0)
					        {
					            enable_hdr = 1;
					            //src_setting.enable_autohdr = 1;
                                enable_hdr_custom = 0;
                                enable_hdr_custom_rgb = 0;
                                LOG_DEBUG ("set hdr mode: auto");
                                sprintf(retbuff, "set hdr mode: auto");
                                goto end;
					        }
					        else if(strcmp(pval->valuestring, "custom") == 0)
					        {
					            enable_hdr = 0;
					            //src_setting.enable_autohdr = 0;
                                enable_hdr_custom = 1;
                                enable_hdr_custom_rgb = 0;
                                LOG_DEBUG ("set hdr mode: custom");
                                sprintf(retbuff, "set hdr mode: custom");
                                goto end;
					        }
					        else if(strcmp(pval->valuestring, "custom-rgb") == 0)
					        {
					            enable_hdr = 0;
					            //src_setting.enable_autohdr = 0;
                                enable_hdr_custom = 0;
                                enable_hdr_custom_rgb = 1;
                                LOG_DEBUG ("set hdr mode: custom-rgb");
                                sprintf(retbuff, "set hdr mode: custom-rgb");
                                goto end;
					        }
					    }
					}
					else if(strcmp(ppara->valuestring, "autohdr_mode") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
					    if(pval)
					    {
					    	auto_hdr_mode = (GstVideoPreprocAutoHDRMode)pval->valueint;
					        LOG_DEBUG ("set autohdr mode %d", auto_hdr_mode);
                            sprintf(retbuff, "set autohdr mode value %d", auto_hdr_mode);
                            goto end;
					    }
					}
					else if(strcmp(ppara->valuestring, "hdr_custom") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
					    if(pval)
					    {
					        guchar value = (guchar)pval->valueint;
					        reset_hdr_table(value);
					        LOG_DEBUG ("set hdr custom value %d", value);
                            sprintf(retbuff, "set hdr custom value %d", value);
                            goto end;
					    }
					}
					else if(strcmp(ppara->valuestring, "hdr_custom_rgb") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
					    if(pval)
					    {
					        guchar value = (guchar)pval->valueint;
					        reset_hdr_rgb_table(value);
					        LOG_DEBUG ("set hdr custom rgb value %d", value);
                            sprintf(retbuff, "set hdr custom rgb value %d", value);
                            goto end;
					    }
					}
					else if(strcmp(ppara->valuestring, "enable_wireframe") == 0)
					{
						cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							global_enable_wireframe = pval->valueint;
							GstVideoPreprocWireFrame *wf = sample_wire_frames;
							if(global_enable_wireframe == 0) {
							    LOG_DEBUG ("Now disable all wire frames...");
							    for (gint j = 0; j < MASK_REGION_MAX_NUM; j++) {
                                    wf[j].enable = FALSE;
                                    global_enable_wireframe = 1;    // force to clear
                                }
							} else {
							    LOG_DEBUG ("Now enable all wire frames...");
                                for (gint j = 0; j < MASK_REGION_MAX_NUM; j++) {
                                    wf[j].enable = TRUE;
                                    // only enable first one
                                    break;
                                }
							}
							LOG_DEBUG ("set wireframe enable = %d", global_enable_wireframe);
							sprintf(retbuff, "set wireframe enable = %d", global_enable_wireframe);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "exposure_window") == 0)
					{
					    cJSON *px = cJSON_GetObjectItem ( json, "pos_x" );
					    cJSON *py = cJSON_GetObjectItem ( json, "pos_y" );
					    cJSON *pw = cJSON_GetObjectItem ( json, "width" );
					    cJSON *ph = cJSON_GetObjectItem ( json, "height" );
					    GstVideoPreprocWireFrame *wf = sample_wire_frames;
					    if(px && py && pw && ph)
					    {
					        guint xpos = (guint)px->valueint;
					        guint ypos = (guint)py->valueint;
					        guint width = (guint)pw->valueint;
					        guint height = (guint)ph->valueint;
					        LOG_DEBUG ("Now set exposure window...");
					        for (gint j = 0; j < MASK_REGION_MAX_NUM; j++) {
                                    wf[j].region.x = xpos;
                                    wf[j].region.y = ypos;
                                    wf[j].region.w = width;
                                    wf[j].region.h = height;
                                    // only enable first one
                                    break;
                            }
                            LOG_DEBUG ("set exposure window, x=%d, y=%d, w=%d, h=%d", xpos, ypos, width, height);
							sprintf(retbuff, "set exposure window, x=%d, y=%d, w=%d, h=%d", xpos, ypos, width, height);
							goto end;
					    }
					}
					else if(strcmp(ppara->valuestring, "enable_mask") == 0)
					{
						cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							global_enable_mask = pval->valueint;
							GstVideoPreprocMaskCfg *masks = sample_masks;
							if(global_enable_mask == 0) {
							    LOG_DEBUG ("Now disable all yuv masks...");
							    for (gint j = 0; j < MASK_REGION_MAX_NUM; j++) {
                                    masks[j].enable = FALSE;
                                    global_enable_mask = 1;     // force to clear
                                }
							} else {
							    LOG_DEBUG ("Now enable all yuv mask...");
                                for (gint j = 0; j < MASK_REGION_MAX_NUM; j++) {
                                    masks[j].enable = TRUE;
                                }
							}
							LOG_DEBUG ("set mask enable = %d", global_enable_mask);
							sprintf(retbuff, "set mask enable = %d", global_enable_mask);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "enable_qos") == 0)
					{
						cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							enable_qos = pval->valueint;
							LOG_DEBUG ("set qos enable = %d", enable_qos);
							sprintf(retbuff, "set qos enable = %d", enable_qos);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "enable_facedetect") == 0)
					{
						cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							enable_facedetect = pval->valueint;
							LOG_DEBUG ("set facedetect enable = %d", enable_facedetect);
							sprintf(retbuff, "set facedetect enable = %d", enable_facedetect);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "enable_bright_compensation") == 0)
					{
						cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							enable_bright_compensation = pval->valueint;
							LOG_DEBUG ("set bright compensation enable = %d", enable_bright_compensation);
							sprintf(retbuff, "set bright compensation enable = %d", enable_bright_compensation);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "enable_3a") == 0)
					{
						cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							gboolean enable_3a = pval->valueint;
							media_pipe_set_v4l2_src_enable_3a(pipe, enable_3a);
							LOG_DEBUG ("set 3A enable = %d", enable_3a);
							sprintf(retbuff, "set 3A enable = %d", enable_3a);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "facedetect_mode") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
						    facedetect_mode = pval->valueint;
						    LOG_DEBUG ("set facedetect mode = %d", facedetect_mode);
							sprintf(retbuff, "set facedetect mode = %d", facedetect_mode);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "flip") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
						    global_flip_mode = (GstVideoPreprocFlipMode)pval->valueint;
							if(!gst_video_preproc_set_flip(GST_VIDEO_PREPROC(impl->preproc.element), global_flip_mode))
							{
								LOG_WARNING ("Set flip mode for preproc FAIL!");
								sprintf(retbuff, "Set flip mode for preproc FAIL!");
							} else {
								LOG_DEBUG ("set flip mode = %d", global_flip_mode);
								sprintf(retbuff, "set flip mode = %d", global_flip_mode);
							}
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "bitrate") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						cJSON *pchannel = cJSON_GetObjectItem ( json, "channel" );
						
						if(pval && pchannel)
						{
							VideoChannelIndex channelIndex = (VideoChannelIndex)pchannel->valueint;
							guint valBitrate = (guint)pval->valueint;
							gboolean result = TRUE;
							result = media_pipe_set_channel_encoder_bitrate(
                                            pipe,
                                            channelIndex,
                                            valBitrate);
				            if(!result)
				            {
				                LOG_WARNING ("Set encode bitrate FAIL!");
								sprintf(retbuff, "Set encode bitrate FAIL!");
				            } else {
				            	LOG_DEBUG ("Set encode bitrate SUCCEED!");
								sprintf(retbuff, "Set encode bitrate SUCCEED!");
				            }
							goto end;
						}
					}
					else if (strcmp(ppara->valuestring, "reconfig_3a") == 0)
					{
					    if(!media_pipe_reconfig_3a(pipe))
                        {
                            LOG_WARNING ("Reconfigure 3A for preproc FAIL!");
                            sprintf(retbuff, "Reconfigure 3A for preproc FAIL!");
                        }else {
                            LOG_DEBUG ("Reconfigure 3A for preproc SUCCEED!");
                            sprintf(retbuff, "Reconfigure 3A for preproc SUCCEED!");
                        }
					    goto end;
					}
					else if (strcmp(ppara->valuestring, "denoise_mode") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							denoise_mode = pval->valueint;
                            media_pipe_set_cl_feature(pipe, CL_DENOISE, denoise_mode);
                            LOG_DEBUG ("set denoise mode = %d", denoise_mode);
							sprintf(retbuff, "set denoise mode = %d", denoise_mode);
							goto end;
						}
					}
                                        else if(strcmp(ppara->valuestring, "enable_dpc") == 0)
                                        {
                                            cJSON *pval = cJSON_GetObjectItem ( json, "val" );
                                            if(pval)
                                            {
                                                gboolean enable_dpc= pval->valueint;
                                                media_pipe_set_cl_feature(pipe, CL_DPC, enable_dpc);
                                                LOG_DEBUG ("set dpc enable = %d", enable_dpc);
                                                sprintf(retbuff, "set dpc enable = %d", enable_dpc);
                                                goto end;
                                            }
                                        }
					else if (strcmp(ppara->valuestring, "enable_dvs") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
							enable_dvs = pval->valueint;
							LOG_DEBUG ("set enable dvs = %d", enable_dvs);
							sprintf(retbuff, "set enable dvs = %d", enable_dvs);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "xcam_mode") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
						    xcam_mode = pval->valueint;
						    LOG_DEBUG ("set xcam mode = %d", xcam_mode);
							sprintf(retbuff, "set xcam mode = %d", xcam_mode);
							goto end;
						}
					}
					else if(strcmp(ppara->valuestring, "hdr_cl_mode") == 0)
					{
					    cJSON *pval = cJSON_GetObjectItem ( json, "val" );
						if(pval)
						{
						    hdr_cl_mode = pval->valueint;
                            media_pipe_set_cl_feature(pipe, CL_HDR, hdr_cl_mode);
						    LOG_DEBUG ("set hdr-cl mode = %d", hdr_cl_mode);
							sprintf(retbuff, "set hdr-cl mode = %d", hdr_cl_mode);
							goto end;
						}
					}
					else if (strcmp(ppara->valuestring, "stop") == 0)
					{
					    LOG_DEBUG("Now stop mediapipe...");
					    media_pipe_stop(pipe);
					    sprintf(retbuff, "Mediapipe stopped.");
					    goto end;
					}
				}
				else if(strcmp(ptype->valuestring, "capture") == 0)
				{
					cJSON *ppara = cJSON_GetObjectItem ( json, "para" );
					if(ppara == NULL)
					{
						LOG_DEBUG("JSON parse Error before: [%s]\n",cJSON_GetErrorPtr());
						sprintf(retbuff, "Invalid parameter!");
						goto end;
					}
					if(strcmp(ppara->valuestring, "jpeg") == 0)
					{
						LOG_DEBUG("Capture jpeg...");
					    char * imgPath = capture_jpeg_frame();
					    LOG_DEBUG("Image captured. Path: %s", imgPath);
					    sprintf(retbuff, "Image captured. Path: %s", imgPath);
	                    goto end;
					}
					else if (strcmp(ppara->valuestring, "yuv") == 0)
					{
					    LOG_DEBUG("Capture yuv...");
					    char *imgPath = capture_raw_frame (pipe, 0);
					    LOG_DEBUG("Row YUV captured. Path: %s", imgPath);
					    sprintf(retbuff, "Row YUV captured. Path: %s", imgPath);
						g_free(imgPath);
						goto end;
					}
					else if (strcmp(ppara->valuestring, "crop") == 0)
					{
					    cJSON *px = cJSON_GetObjectItem ( json, "pos_x" );
					    cJSON *py = cJSON_GetObjectItem ( json, "pos_y" );
					    cJSON *pw = cJSON_GetObjectItem ( json, "width" );
					    cJSON *ph = cJSON_GetObjectItem ( json, "height" );
					    if(px && py && pw && ph)
					    {
					        gint xpos = (gint)px->valueint;
					        gint ypos = (gint)py->valueint;
					        gint width = (gint)pw->valueint;
					        gint height = (gint)ph->valueint;
					        if (mediapipe_set_jpegenc_crop (TRUE, xpos, ypos, width, height))
					        {
					            LOG_DEBUG("Capture crop jpeg...");
					            char * imgPath = capture_jpeg_frame();
					            LOG_DEBUG("Crop image captured. Path: %s", imgPath);
                                sprintf(retbuff, "Crop image captured. Path: %s", imgPath);
                                goto end;
					        }
					    }
					}
				}
			}
		}

		LOG_DEBUG("Invalid parameter!");
		sprintf(retbuff, "Invalid parameter!");

end:
		//send msg to another queue
		if(json != NULL)
		{
		    cJSON_Delete(json);
		}

		{
		amqp_basic_properties_t props;
		props._flags = AMQP_BASIC_CONTENT_TYPE_FLAG | AMQP_BASIC_DELIVERY_MODE_FLAG;
		props.content_type = amqp_cstring_bytes("text/plain");
		props.delivery_mode = 2; /* persistent delivery mode */
		die_on_error(amqp_basic_publish(conn,
				                        1,
				                        amqp_cstring_bytes(""),
				                        amqp_cstring_bytes("queue-m2w"),
				                        0,
				                        0,
				                        &props,
										amqp_cstring_bytes(retbuff)),
				     "Publishing");
		}

		amqp_destroy_envelope(&envelope);
    }

    return NULL;
}
/* finish rabbitmq integration */

static gboolean
h264_frame_callback (GstBuffer *encode_buf, gpointer user_data, VideoChannelIndex channel)
{
    static guint frame_num = 0;

    if(encode_buf == NULL || !GST_IS_BUFFER(encode_buf))
    {
        goto Exit;
    }

    /*
        *   Can access MV data here
        */
#if SUPPORT_MV
    if(GST_BUFFER_FLAG_IS_SET(encode_buf, GST_BUFFER_FLAG_DELTA_UNIT))
    {
        // current frame is not I frame
    }
#endif

    // TODO: Send H264/MV data to upper interface.

    if(channel == VIDEO_CHANNEL_1080P)
    {
        frame_num++;
    }

 Exit:

    return TRUE;
}


static gboolean
video_frame_jpeg_callback (GstVideoPreprocBuffer *video_buf, gpointer user_data, VideoChannelIndex channel)
{
    static guint frame_num = 0;
    static guint frame_keyboard_num = 0;
    gchar filename[128] = "";
    JpgEnc_Misc_Config *pConfig = &jpgenc_misc_config;

    if(video_buf == NULL)
    {
        goto Exit;
    }

    JpegInputBuffer jpeg_input_buf;

    jpeg_input_buf.handle = video_buf->handle;
    jpeg_input_buf.w = video_buf->w;
    jpeg_input_buf.h = video_buf->h;
    jpeg_input_buf.ystride = video_buf->ystride;
    jpeg_input_buf.uvstride = video_buf->uvstride;
    jpeg_input_buf.yoffset = video_buf->yoffset;
    jpeg_input_buf.uvoffset = video_buf->uvoffset;

    // test encode one frame every 20 frames
    if((frame_num % (pConfig->jpg_interval == 0 ? 1 : pConfig->jpg_interval)) == 0)
    {

       FPS_CALCULATION(jpeg_encoder);

        if(!pConfig || pConfig->jpg_out_mode == JPEG_OUTPUT_FILE)
        {
            sprintf(filename, "frame_%d.jpg", frame_num);
                jpeg_encode_file(&jpeg_input_buf, 100, filename, NULL, 0, NULL);
        }
        else
        {
           // to check defination of JpegOutputBuffer, see /usr/lib/jpeglib_interface.h
           JpegOutputBuffer jpeg_output_buf;

           jpeg_encode_buffer(&jpeg_input_buf, 100, NULL, 0, &jpeg_output_buf);

           // after using jpeg buffer, must release it
           jpeg_destroy_buffer();
        }
    }

    //test regional jpeg encoding every 15 frames
    if(pConfig && pConfig->jpeg_crop_enable)
    {
        //JpegOutputBuffer jpeg_output_buf = {()0};

        //configure your own crop region
        JpegCropRect cropRect;

        if(enable_facerecognise && rect_jpeg.flag == 1) {
                    cropRect.x = rect_jpeg.x;
                    cropRect.y = rect_jpeg.y;
                    cropRect.w = rect_jpeg.width;
                    cropRect.h = rect_jpeg.height;
                    sprintf(filename, "frame_crop_%d.jpg", crop_jpeg_num);
                    jpeg_encode_file(&jpeg_input_buf, 100, filename, NULL, 0, &cropRect);
                    crop_jpeg_num++;
                    rect_jpeg.flag = 0;
        }

        if((!enable_facerecognise) && frame_num % 15 == 0)
        {
            cropRect.x = pConfig->jpeg_crop_ox;
            cropRect.y = pConfig->jpeg_crop_oy;
            cropRect.w = pConfig->jpeg_crop_width;
            cropRect.h = pConfig->jpeg_crop_height;
                      sprintf(filename, "frame_crop_%d.jpg", crop_jpeg_num);
                      jpeg_encode_file(&jpeg_input_buf, 100, filename, NULL, 0, &cropRect);
                      crop_jpeg_num++;
        }
    }

    if(pConfig && pConfig->jpeg_keyboard_flag)
    {
            sprintf(filename, "frame_keyboard_%d.jpg", frame_keyboard_num);
            jpeg_encode_file(&jpeg_input_buf, 100, filename, NULL, 0, NULL);
            frame_keyboard_num++;
            pConfig->jpeg_keyboard_flag = 0;
    }

    if(jpgenc_misc_config2.jpeg_capture_flag)
    {
       JpegCropRect cropRect;
       JpegCropRect *rect = NULL;
       if(jpgenc_misc_config2.jpeg_crop_enable) {
          cropRect.x = jpgenc_misc_config2.jpeg_crop_ox;
          cropRect.y = jpgenc_misc_config2.jpeg_crop_oy;
          cropRect.w = jpgenc_misc_config2.jpeg_crop_width;
          cropRect.h = jpgenc_misc_config2.jpeg_crop_height;
          rect = &cropRect;
       }
       jpeg_encode_file(&jpeg_input_buf, 100, jpgenc_misc_config2.capture_filename, NULL, 0, rect);
       jpgenc_misc_config2.jpeg_capture_flag = 0;
    }
    frame_num++;
 Exit:
    return TRUE;
}

static guchar check_range(int a)
{
if(a>255)
    return 255;
else if(a<0)
    return 0;
else
    return (guchar)a;
}
static void NV12toBGR(int width, int height,int stride, unsigned char *src, unsigned char *dst, int uvoffset)
{
    int i,j;
    for(j=0;j<height;j++)
        {
            for (i = 0; i < width; i++)
                {
                    if(j%2 == 0)
                        {
                            if(i%2 == 0)  //BGR
                                {
                                    *dst++ = check_range(src[stride*j+ i]-16 + 2.03 * (src[uvoffset + stride*j/2 +i]-128)); //B=Y+2.03U
                                    *dst++ = check_range(src[stride*j+ i]-16 - 0.39 * (src[uvoffset + stride*j/2 +i]-128) - 0.58*(src[uvoffset + stride*j/2 + i +1]-128));  //G=Y-0.39U-0.58V
                                    *dst++ = check_range(src[stride*j+ i]-16 + 1.14 * (src[uvoffset + stride*j/2 + i +1]-128));  //R=Y +1.14V
                                }
                            else
                                {
                                    *dst++ = check_range(src[stride*j+ i]-16 + 2.03 * (src[uvoffset + stride*j/2 + i-1]-128)); //B=Y+2.03U
                                    *dst++ = check_range(src[stride*j+ i]-16 - 0.39 * (src[uvoffset + stride*j/2 + i -1]-128) - 0.58*(src[uvoffset + stride*j/2 + i]-128)); //G=Y-0.39U-0.58V
                                    *dst++ = check_range(src[stride*j+ i]-16 + 1.14 * (src[uvoffset + stride*j/2+ i]-128));  //R=Y +1.14V
                                }
                        }
                    else
                        {
                            if(i%2 == 0)
                                {
                                    *dst++ = check_range(src[stride*j+ i]-16 + 2.03 * (src[uvoffset + stride*(j-1)/2 +i]-128)); //B=Y+2.03U
                                    *dst++ = check_range(src[stride*j+ i]-16 - 0.39 * (src[uvoffset + stride*(j-1)/2 +i]-128) - 0.58*(src[uvoffset + stride*(j-1)/2 + i +1]-128));  //G=Y-0.39U-0.58V
                                    *dst++ = check_range(src[stride*j+ i]-16 + 1.14 * (src[uvoffset + stride*(j-1)/2 + i +1]-128));  //R=Y +1.14V
                                }
                            else
                                {
                                    *dst++ = check_range(src[stride*j+ i]-16 + 2.03 * (src[uvoffset + stride*(j-1)/2 + i-1]-128)); //B=Y+2.03U
                                    *dst++ = check_range(src[stride*j+ i]-16 - 0.39 * (src[uvoffset + stride*(j-1)/2+ i -1]-128) - 0.58*(src[uvoffset + stride*(j-1)/2 + i]-128));
                                    *dst++ = check_range(src[stride*j+ i]-16 + 1.14 * (src[uvoffset + stride*(j-1)/2 + i]-128)); //R=Y +1.14V
                                }
                        }
                }
        }
}

static void get_gray_from_gst(int width, int height,int stride, unsigned char *src, unsigned char *dst)
{
    int i;
    for(i=0;i<height;i++)
        {
            memcpy(dst,src,sizeof(char)*width);
            dst+=width;
            src+=stride;
        }
}

static void read_csv(const string& filename, vector<Mat>& images, vector<int>& labels, vector<string>& names, char separator = ';')
{
    std::ifstream file(filename.c_str(), std::ifstream::in);
    if (!file) {
        string error_message = "No valid input file was given, please check the given filename.";
        CV_Error(CV_StsBadArg, error_message);
    }
    string line;
    while (getline(file, line)) {
        string path, classlabel, class_name;
        stringstream liness(line);
        getline(liness, path, separator);
        getline(liness, classlabel, separator);
        getline(liness, class_name);
        if(!path.empty() && !classlabel.empty()) {
            images.push_back(imread(path, 0));
            labels.push_back(atoi(classlabel.c_str()));
        if(!class_name.empty()){
            //cout<<class_name<<endl;
            //names.push_back(class_name);
            //label_name[atoi(classlabel.c_str())] = class_name;
        }
        }
    }
}

int makepath(string s,mode_t mode)
{
    size_t pre=0,pos;
    string dir;
    int mdret;

    if(s[s.size()-1]!='/'){
        // force trailing / so we can handle everything in loop
        s+='/';
    }

    while((pos=s.find_first_of('/',pre))!=std::string::npos){
        dir=s.substr(0,pos++);
        pre=pos;
        if(dir.size()==0) continue; // if leading / first time is 0 length
        if((mdret=::mkdir(dir.c_str(),mode)) && errno!=EEXIST){
            return mdret;
        }
    }
    return mdret;
}

static gboolean
video_frame_bright_callback (
                            GList *smart_queue,
                            GList *smart_1080p_queue,
                            gpointer user_data)
{
    // TODO: use smart_queue to do smart analyzing, store result wire frame list in a self-defined cache
    if(!enable_bright_compensation)
	    return;

    MediaPipeImpl *impl = (MediaPipeImpl *)user_data;
    MediaPipe *pipe = (MediaPipe *)&impl->pipe;
    SmartData *data_smart;

    vector<Rect> boxes;
    Rect focusarea;
    static int hdr_enabled = 0, init_3a_auto=0,init_3a_bright=0;
    int pixel_sfilter;
    int maxpixels = 0;

    data_smart = (SmartData *)g_list_nth_data(smart_queue, 0);
    Mat img_in;

    img_in.dims = 2;
    img_in.cols = data_smart->preBuf->w;
    img_in.rows = data_smart->preBuf->h;

    //img_in.imageSize =  img_in->width* img_in->height;
    img_in.data = (unsigned char*)malloc(sizeof(unsigned char)*data_smart->preBuf->w*data_smart->preBuf->h);
    img_in.datastart = img_in.data;
    img_in.dataend = img_in.data + data_smart->preBuf->w*data_smart->preBuf->h;
    img_in.datalimit = img_in.dataend;
    img_in.step = img_in.cols;
    img_in.step.buf[1] = 1;

    get_gray_from_gst(img_in.cols,  img_in.rows, data_smart->preBuf->ystride,
		    (unsigned char*)(data_smart->preBuf->handle), (unsigned char*)(img_in.data));

    pixel_sfilter = img_in.rows*img_in.cols/1000;
    //Mat yuv;
    //cvtColor(img_in, yuv, COLOR_BGR2YUV);

    Mat bw;
    inRange(img_in, Scalar(250,0,0), Scalar(255,255,255), bw);
    
    vector<vector<Point> > contours_bright;
    vector<Vec4i> hierarchy_bright;
    findContours(bw, contours_bright, hierarchy_bright, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    for (size_t i = 0; i < contours_bright.size(); i++)
    {
	    Rect rect = boundingRect(contours_bright[i]);
	    if ( maxpixels < rect.width*rect.height )
	    {
		    maxpixels = rect.width*rect.height;
	    }
    }

    if(maxpixels < 2*pixel_sfilter)
    {
	    fprintf(stderr, "zjuan:maxpixels = %d,pixel_sfilter=%d \n\n\n", maxpixels, pixel_sfilter);
	    if(!init_3a_auto)
	    {
		    if(!media_pipe_reconfig_3a(pipe))
		    {
			    LOG_WARNING ("Reconfigure 3A for preproc FAIL!");
		    }else {
			    LOG_DEBUG ("Reconfigure 3A for preproc SUCCEED!");
			    init_3a_auto=1;
			    init_3a_bright=0;
		    }
	    }
	    enable_hdr=0;
	    gst_video_preproc_disable_autohdr(GST_VIDEO_PREPROC(impl->preproc.element));
	    return;
    }
    else
	    maxpixels = 0;
    

    inRange(img_in, Scalar(0,0,0), Scalar(50,255,255), bw);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(bw, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    for (size_t i = 0; i < contours.size(); i++)
    {
	    Rect rect = boundingRect(contours[i]);
	    if (rect.width * rect.height > pixel_sfilter)
	    {
		    int x2 = min(rect.x + rect.width * 1.2, img_in.cols - 0.0);
		    int y2 = min(rect.y + rect.height * 1.2, img_in.rows - 0.0);

		    rect.x = max(rect.x-cvRound(rect.width*0.2), 0);
		    rect.y = max(rect.y-cvRound(rect.height*0.2), 0);
		    rect.width = x2 - rect.x;
		    rect.height = y2 - rect.y;

		    boxes.push_back(rect);
	    }
    }

    for (size_t i = 0; i < boxes.size(); i++) {
	    Rect bRect = boxes[i];
	    rectangle( img_in, cvPoint(bRect.x, bRect.y), cvPoint(bRect.x+bRect.width,bRect.y+bRect.height),
			    CV_RGB(255,0,0),1,8,0);
	    if ( maxpixels < bRect.width*bRect.height )
	    {
		    focusarea = bRect;
		    maxpixels = bRect.width*bRect.height;
	    }
    }
    if (maxpixels != 0)
    {
	    rectangle( img_in, cvPoint(focusarea.x, focusarea.y), cvPoint(focusarea.x+focusarea.width,focusarea.y+focusarea.height),
			    CV_RGB(255,255,0),1,8,0);

	    GstElement  *v4l2src;
	    GstXCam3A   *xcam;
	    GstXCam3AInterface      *xcam_interface;
	    Cameara3a_Exposure * ep_config;
	    gboolean ret = TRUE;


	    if(!pipe->pipe_running)
	    {
		    LOG_ERROR("Cannot reconfig 3a settings while pipeline is not running.");
		    return FALSE;
	    }

	    if(!impl->src_source.v4l2_enable_3a)
	    {
		    LOG_ERROR("Cannot reconfig 3a settings while 3a not enabled.");
		    return FALSE;
	    }

	    /**/
	    v4l2src = impl->src_source.gen_src;
	    {
		    GstElementFactory *factory = gst_element_get_factory (v4l2src);
		    const gchar *pluginname = gst_plugin_feature_get_name(GST_PLUGIN_FEATURE(factory));

		    if(strcmp(pluginname,"xcamsrc")!=0)
		    {
			    LOG_DEBUG("The '%s' element is a member of the category %s.\n"
					    "Description: %s\n",
					    pluginname,
					    gst_element_factory_get_klass(factory),
					    gst_element_factory_get_description(factory));

			    LOG_ERROR("Cannot reconfig 3a settings while it isn't a v4l2src.");
			    return FALSE;
		    }
	    }
	    if (!init_3a_auto && !init_3a_bright){
		    if(!media_pipe_reconfig_3a(pipe))
		    {
			    LOG_WARNING ("Reconfigure 3A for preproc FAIL!");
		    }else {
			    LOG_DEBUG ("Reconfigure 3A for preproc SUCCEED!");
			    init_3a_auto=1;
			    init_3a_bright=0;
		    }
	    }

	    {
		    init_3a_auto=0;
		    init_3a_bright=1;
		    fprintf(stderr, "adjusting ae window!\n");
		    ep_config=(Cameara3a_Exposure *)malloc(sizeof(Cameara3a_Exposure));
		    ep_config->val_ep_mode = 0;
		    ep_config->val_meter_mode= 1;
		    ep_config->val_ep_window[0].x_start = focusarea.x;
		    ep_config->val_ep_window[0].y_start = focusarea.y;
		    ep_config->val_ep_window[0].x_end = focusarea.x+focusarea.width;
		    ep_config->val_ep_window[0].y_end = focusarea.y+focusarea.height;
		    ep_config->val_ep_window[0].weight = 50;
		    //ep_config->val_ep_manual_analoggain =40;
		    //ep_config->val_ep_manual_time = 4000;
		    ep_config->val_ep_max_analoggain=100;
		    ep_config->val_ep_timerange_min=0;
		    ep_config->val_ep_timerange_max=10000;
		    xcam = GST_XCAM_3A (v4l2src);
		    xcam_interface = GST_XCAM_3A_GET_INTERFACE (xcam);
		    xcam_interface->set_exposure_mode(xcam, (XCamAeMode)(ep_config->val_ep_mode));
		    xcam_interface->set_ae_metering_mode(xcam, (XCamAeMeteringMode)(ep_config->val_meter_mode));
		    xcam_interface->set_exposure_window(xcam, &ep_config->val_ep_window[0], 1);
		    //xcam_interface->set_manual_analog_gain(xcam, ep_config->val_ep_manual_analoggain);
		    //xcam_interface->set_manual_exposure_time(xcam, ep_config->val_ep_manual_time);
		    //xcam_interface->set_max_analog_gain(xcam, ep_config->val_ep_max_analoggain);
		    //xcam_interface->set_exposure_time_range(xcam, ep_config->val_ep_timerange_min, ep_config->val_ep_timerange_max);
		    enable_hdr=1;
		    auto_hdr_mode=1;
		    if (!hdr_enabled)
		    {
			    ret = gst_video_preproc_set_autohdr(GST_VIDEO_PREPROC(impl->preproc.element),(GstVideoPreprocAutoHDRMode)auto_hdr_mode);
			    if (ret == 0)
			    	hdr_enabled = 1;
		    }
		    free(ep_config);
	    }
   }
   // free(img_in.data);
   // free(&bw);

} 

static gboolean
video_frame_smart_callback (
                            GList *smart_queue,
                            GList *smart_1080p_queue,
                            gpointer user_data)
{
    // TODO: use smart_queue to do smart analyzing, store result wire frame list in a self-defined cache

    //guint index;
    MediaPipeImpl *impl = (MediaPipeImpl *)user_data;
    Smart1080pData *data_1080p = NULL;
    SmartData *data_smart;

    data_smart = (SmartData *)g_list_nth_data(smart_queue, 0);
    // you got one frame data in pointer data_smart->preBuf->handle, do whatever you want


    Mat cvframe;
    double scale = 1.0;
    char cascadename[100];
    const string fn_csv = "/usr/share/OpenCV/csv.txt";
    const string dataname  = "/usr/share/OpenCV/Fisher.yml";
    const string path = "/usr/share/OpenCV";

    //static ocl::OclCascadeClassifier cascade;
   // static CascadeClassifier  cpu_cascade;
    static int init_state = 0;
    vector<Rect> faces;
    int i=0;
    int im_width,im_height;
    int prediction;
    double confidence;

    data_smart = (SmartData *)g_list_nth_data(smart_queue, 0);
    if(facedetect_conf)
      strcpy(cascadename,"/usr/share/OpenCV/lbpcascades/lbpcascade_frontalface.xml");
    else
      strcpy(cascadename, "/usr/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml");
    // you got one frame data in pointer data_smart->preBuf->handle, do whatever you want

    // Smart analyzing...
    //
    if(!init_state)
	{
         fd_init(cascadename);
	init_state = 1;
	}
    face_getpic_frame_number++;
    if((enable_getfacepic)&&(face_getpic_frame_number >30))
        {
              //todo get gray pic from v4l2.
       /*
        stringstream sst;
        if(face_getpic_state ==0)
            {
                cout<<"The person's name is :";
                cin.getline (person_name_c, sizeof(person_name_c));
                cout<<"The person ID is :";
                cin >>pnum;
                sst<<path<<"/s"<<pnum;
                sst>>dirpath;
                if(makepath(dirpath,0755)) cout<<"mkdir success!"<<endl;
                face_getpic_state = 1;
            }
        ofstream outcsv(fn_csv.c_str(),ios::app);

        iplImg = (IplImage*)malloc(sizeof(IplImage));
        memset(iplImg, 0, sizeof(IplImage));
        iplImg->depth = 8;
        strcpy(iplImg->colorModel, "RGB");
        strcpy(iplImg->channelSeq, "RGB");
        iplImg->width = data_smart->preBuf->w;
        iplImg->height = data_smart->preBuf->h;

        iplImg->imageSize =  iplImg->width* iplImg->height;
        iplImg->nChannels = 1;
        iplImg->imageData = (char*)malloc(sizeof(char)*data_smart->preBuf->w*data_smart->preBuf->h);
        iplImg->widthStep =  iplImg->width;

        get_gray_from_gst(iplImg->width,  iplImg->height, data_smart->preBuf->ystride,
            (unsigned char*)(data_smart->preBuf->handle), (unsigned char*)(iplImg->imageData));
        //NV12toRGB(iplImg->width,  iplImg->height, data_smart->preBuf->ystride, data_smart->preBuf->handle, iplImg->imageData);
        cvframe = iplImg;


        //detect(cvframe, faces, cascade, scale, false);
        // detectCPU(cvframe, faces, cpu_cascade, scale, false);
        fd_detect_cpu(cvframe, faces)
        if(!faces.empty())
            {
        Point center;
        int rw,rh;
        center.x = cvRound((faces[0].x + faces[0].width*0.5)*scale);
        center.y = cvRound((faces[0].y + faces[0].height*0.5)*scale);
        rw = cvRound(faces[0].width*scale);
        rh = cvRound(faces[0].height*scale);
        Rect temp(center.x-rw*0.4, center.y-rh*0.5, rw*0.8, rh);
        Mat pic = cvframe(temp);
        resize(pic,pic,Size(90,112),0,0,INTER_LINEAR);
        stringstream ss;
        ss<<dirpath<<"/"<<face_getpic_number+1<<".pgm";
        string name;
        ss>>name;
        imwrite(name,pic);
        face_getpic_number++;
        cout<<"saving: "<<name<<endl;
        outcsv<<name<<";"<<pnum<<";";
        if(face_getpic_number  == 1)
        outcsv<<person_name_c;
        outcsv<<endl;

        cout<<"Finish get and got "<<face_getpic_number<<" images!"<<endl;
        vector<Mat> images;
        vector<int> labels;
        vector<string> names;
        try{
            read_csv(fn_csv, images, labels,names);
        }
        catch (cv::Exception& e) {
            cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
            exit(1);
        }
        bool multi_class = false;
        int first_class = labels[0];
        for(guint k = 0; k < labels.size(); k++)
            if(labels[k] != first_class){
                multi_class = true;
                break;
            }
        if(multi_class){
            Ptr<FaceRecognizer> model = createFisherFaceRecognizer();
            model->train(images, labels);
            model->save(dataname);
        }
        }
    face_getpic_frame_number = 0;
    if(face_getpic_number >= 10)
        {
        printf("get face pic finished \n");
        exit(1);
        }
    */
    }

    if(enable_facedetect)
    {
        if(face_detect_number > face_detect_interval)
        {
            // LOG_DEBUG("*** face detect *** \n");
            if(enable_facerecognise)
            {
            /*
                try{
                  read_csv(fn_csv, images, labels, names);
                  }
                catch (cv::Exception& e) {
                  cerr << "Error opening file \"" << fn_csv << "\". Reason: " << e.msg << endl;
                  // nothing more we can do
                  exit(1);
                  }

                try{
                  model->load(dataname);
                  printf("Load success!\n");
                  im_width=90;
                  im_height=112;
                  }
                catch (cv::Exception& e)
                  {
                    im_width = images[0].cols;
                    im_height = images[0].rows;
                    model->train(images, labels);
                    model->save(dataname);
                }
                */
            }
            rect_jpeg = {0,0,1920,1080,0};

            if(facedetect_mode == 0)
            {
                cvframe.cols = data_smart->preBuf->w;
                cvframe.rows = data_smart->preBuf->h;
                cvframe.dims = 2;
                cvframe.data = (uchar*)malloc(sizeof(char)*data_smart->preBuf->w*data_smart->preBuf->h);
                cvframe.datastart = cvframe.data;
                cvframe.dataend = cvframe.data + data_smart->preBuf->w*data_smart->preBuf->h;
                cvframe.datalimit = cvframe.dataend;
                cvframe.step =  cvframe.cols;
                cvframe.step.buf[1] = 1;
                get_gray_from_gst(cvframe.cols, cvframe.rows, data_smart->preBuf->ystride,(unsigned char*)(data_smart->preBuf->handle), (unsigned char*)(cvframe.data));
                fd_detect_gpu(cvframe, faces);
               //reset wire frames
                for (i = 0; i < WIRE_FRAME_REGION_MAX_NUM; i++) {
                   cv_wire_frames[i].enable = FALSE;
                }
                i = 0;
                for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
                {
                   if(i == WIRE_FRAME_REGION_MAX_NUM) break;
                   if(enable_facerecognise)
                   {
                   /*
                       Rect face_i((r->x+r->width*0.1),r->y,r->width*0.8,r->height);
                       Mat face = cvframe(face_i);
                       Mat face_resized;
                       cv::resize(face,face_resized,Size(im_width,im_height),1.0,1.0,INTER_CUBIC);
                       Mat grayface;
                       model->predict(face_resized,prediction,confidence);
                       if(confidence<100.0)
                           {
                              if(!smart_control)
                                 g_mutex_lock(&impl->src_preproc.smart_lock);
                               cv_wire_frames[i]= {{r->x*(guint)1920/iplImg->width/(guint)2*(guint)2,r->y*(guint)1080/iplImg->height/(guint)2*(guint)2,
                                  r->width*(guint)1920/iplImg->width/(guint)2*(guint)2,r->height*(guint)1080/iplImg->height/(guint)2*(guint)2}, {18, 12,  213}, 4, TRUE};
                              if(!smart_control)
                                 g_mutex_unlock(&impl->src_preproc.smart_lock);
                               rect_jpeg.x = r->x*(guint)1920/iplImg->width/(guint)4*(guint)4;
                               rect_jpeg.y = r->y*(guint)1080/iplImg->height/(guint)4*(guint)4;
                               rect_jpeg.width = r->width*(guint)1920/iplImg->width/(guint)16*(guint)16;
                               rect_jpeg.height = r->height*(guint)1080/iplImg->height/(guint)16*(guint)16;
                               rect_jpeg.flag = 1;
                           }
                       LOG_DEBUG(" face recognise info: i=%d x=%d y=%d width=%d height=%d confidence=%f\n",
                          i,r->x*(guint)1920/iplImg->width/(guint)2*(guint)2,r->y*(guint)1080/iplImg->height/(guint)2*(guint)2,
                          r->width*(guint)1920/iplImg->width/(guint)2*(guint)2,r->height*(guint)1080/iplImg->height/(guint)2*(guint)2,confidence);
                       */
                   }
                    else
                    {
                       if(!smart_control)
                          g_mutex_lock(&impl->src_preproc.smart_lock);
                        cv_wire_frames[i]= {{r->x*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,r->y*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2,
                            r->width*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,r->height*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2}, {18, 12,  213}, 4, TRUE};
                       if(!smart_control)
                          g_mutex_unlock(&impl->src_preproc.smart_lock);
                        LOG_DEBUG(" face detect info: i=%d x=%d y=%d width=%d height=%d\n",i,r->x*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,
                            r->y*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2,r->width*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,r->height*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2);
                    }
                 }
              }
                else if(facedetect_mode ==1)
                {
                    cvframe.cols = data_smart->preBuf->w;
                    cvframe.rows = data_smart->preBuf->h;
                    cvframe.data = (uchar*)malloc(sizeof(char)*data_smart->preBuf->w*data_smart->preBuf->h*3);
                    cvframe.step =  cvframe.cols*3;
                    cvframe.step.buf[1] = 3;
                    cvframe.datastart = cvframe.data;
                    cvframe.dataend = cvframe.datastart +  cvframe.cols*cvframe.rows*3;
                    cvframe.datalimit = cvframe.dataend;
                    cvframe.dims  =2;
                    cvframe.flags = 1124024336;
                    NV12toBGR(cvframe.cols,  cvframe.rows, data_smart->preBuf->ystride, data_smart->preBuf->handle, cvframe.data, data_smart->preBuf->uvoffset);
                    for (i = 0; i < WIRE_FRAME_REGION_MAX_NUM; i++) {
                    cv_wire_frames[i].enable = FALSE;
                }
                    fd_detect_withskin(cvframe, faces);
                    i = 0;
                    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
                    {
                       if(!smart_control)
                          g_mutex_lock(&impl->src_preproc.smart_lock);
                        cv_wire_frames[i]= {{r->x*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,r->y*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2,
                            r->width*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,r->height*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2}, {18, 12,  213}, 4, TRUE};
                       if(!smart_control)
                          g_mutex_unlock(&impl->src_preproc.smart_lock);
                        LOG_DEBUG(" face detect info: i=%d x=%d y=%d width=%d height=%d\n",i,r->x*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,
                            r->y*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2,r->width*(guint)1920/data_smart->preBuf->w/(guint)2*(guint)2,r->height*(guint)1080/data_smart->preBuf->h/(guint)2*(guint)2);
                    }
                }
                #if 0        //  blur ocl gpu
                {
                                                        iplImg->width = data_smart->preBuf->w;
                    iplImg->height = data_smart->preBuf->h*(guint)3/(guint)2;
                    iplImg->align = data_smart->preBuf->w;
                    iplImg->imageSize =  iplImg->width* iplImg->height*(guint)3;
                    iplImg->widthStep =  iplImg->width;
                    iplImg->nChannels = 1;
                    iplImg->imageData = (char*)malloc(sizeof(char)*data_smart->preBuf->w*data_smart->preBuf->h*(guint)3/(guint)2);
                    char *tmp1 = (char*)(data_smart->preBuf->handle);
                    char *tmp2 = (char*)(iplImg->imageData);
                    for(i=0;i<iplImg->height;i++)
                    {
                            memcpy(tmp2,tmp1,sizeof(char)*iplImg->width);
                            tmp1 += data_smart->preBuf->ystride;
                            tmp2 += iplImg->width;
                    }
                    cvframe = iplImg;
                    detect_blur_ocl(impl, cvframe, faces, cascade, scale);
                }
               #endif
            free(cvframe.data);
            face_detect_number = 0;
        }
        else
            face_detect_number++;
   }

    /*
     * After analysis, determine which 1080p frame to do jpeg encode. In this sample code,
     * always mark current frame as need to jpeg encode.
     */
    GstClockTime *ts = g_new0(GstClockTime, 1);
    *ts = GST_BUFFER_TIMESTAMP(data_smart->buf);

    g_mutex_lock(&jpeg_lock);
    frames_do_jpeg_encoding = g_list_append(frames_do_jpeg_encoding, ts);
    g_mutex_unlock(&jpeg_lock);

    if (smart_control) {
       // after analying, need to mark push flag for unused frames AND update wireframe queue

       /*
        * push all the 1080p frames that prior to this smart frame.
        * Note: This is just an example,  you should design your own policy to push
        * 1080p frame, according to analysis algorithm.
        */
       GstClockTime ts = GST_BUFFER_TIMESTAMP(data_smart->buf);
       GST_DEBUG("smart frame timestamp: %" GST_TIME_FORMAT, GST_TIME_ARGS(ts));
       while (smart_1080p_queue) {
          data_1080p = (Smart1080pData *)(smart_1080p_queue->data);
          if (data_1080p && GST_BUFFER_TIMESTAMP(data_1080p->buf) <= ts) {
             GST_DEBUG("can push 1080p frame, timestamp: %" GST_TIME_FORMAT, GST_TIME_ARGS(ts));
             data_1080p->can_push = TRUE;
          }
          smart_1080p_queue = smart_1080p_queue->next;
       }
    }

    return TRUE;
}

static gboolean
message_callback (GstMessage *mesg, gpointer user_data)
{
    return TRUE;
}

gpointer
dynamic_channel_thread_func (gpointer data)
{
    MediaPipe *media_pipe;
    media_pipe = (MediaPipe *)data;

    usleep(5000000);

    LOG_DEBUG("Test disable channel D1 dynamically");

    media_pipe_disable_video_channel(
                    media_pipe,
                    VIDEO_CHANNEL_D1);
    usleep(5000000);

    LOG_DEBUG("Test enable channel CIF dynamically");

    media_pipe_enable_video_channel(
                    media_pipe,
                    VIDEO_CHANNEL_CIF);
    return NULL;
}

gpointer
run_timer_func (gpointer data)
{
    MediaPipe *media_pipe;
    media_pipe = (MediaPipe *)data;

    if(src_setting.run_time_in_sec == 0)
        return NULL;

    sleep(src_setting.run_time_in_sec);

    LOG_DEBUG("Time out, now terminate pipeline");
    media_pipe_stop(media_pipe);

    return NULL;
}

gboolean
mediapipe_set_jpegenc_crop (gboolean enable, gint cropOx, gint cropOy,
	gint cropWidth, gint cropHeight)
{
	g_mutex_lock(&jpgenc_misc_config2.lock);
	jpgenc_misc_config2.jpeg_crop_enable = enable;
	jpgenc_misc_config2.jpeg_crop_ox = cropOx;
	jpgenc_misc_config2.jpeg_crop_oy = cropOy;
	jpgenc_misc_config2.jpeg_crop_width = cropWidth;
	jpgenc_misc_config2.jpeg_crop_height = cropHeight;
	g_mutex_unlock(&jpgenc_misc_config2.lock);

	return TRUE;
}

gboolean
mediapipe_set_jpegenc_output (JpegOutputMode outputmode, guint interval)
{
	g_mutex_lock(&jpgenc_misc_config2.lock);
	jpgenc_misc_config2.jpg_out_mode = outputmode;
	jpgenc_misc_config2.jpg_interval = interval;
	g_mutex_unlock(&jpgenc_misc_config2.lock);

	return TRUE;
}

gpointer
dynamic_update_params_thread_func (gpointer data)
{
    MediaPipe *media_pipe;
    media_pipe = (MediaPipe *)data;

    usleep(1400000);

    guint i;

	//
	LOG_DEBUG("Test forcing Key frame for all channels");
    for(i=0; i<VIDEO_CHANNEL_MAX; i++ )
    {
        media_pipe_set_channel_key_frame(
                                 media_pipe,
                                 (VideoChannelIndex)i);
    }

	//
	usleep(5000000);
	LOG_DEBUG("Test updating jpeg enc params");
	mediapipe_set_jpegenc_crop(TRUE, 10, 20, 20, 20);

	//
	usleep(5000000);
	mediapipe_set_jpegenc_output(JPEG_OUTPUT_FILE, 1);

	//

	return NULL;
}


gboolean
parse_config()
{
    gboolean result = TRUE;
    FILE *fp = NULL;
    mxml_node_t *tree = NULL;
    mxml_node_t *node = NULL;

    guint channel = 0;

    fp = fopen(CONFIG_FILE_NAME, "r");
    if(fp == NULL)
    {
        LOG_ERROR("no config file: /etc/mediapipe/conf.xml");
        return FALSE;
    }

    tree = mxmlLoadFile(NULL,
                        fp,
                        MXML_NO_CALLBACK);
    if(tree == NULL)
    {
        LOG_ERROR("loading config file Fail");
        result = FALSE;
        goto Exit;
    }

    // Parsing Channel config
    node = mxmlFindElement(tree, tree, "pipeline", NULL, NULL,MXML_DESCEND);
    while(node != NULL)
    {
        const gchar *name = mxmlGetElement(node);


        if(strcmp(name, "src") == 0)
        {
            src_setting.src_type = (SrcType)atoi(node->child->value.text.string);
            LOG_DEBUG("using %s", src_name[src_setting.src_type]);

        }else
        if(strcmp(name, "location") == 0)
        {
            g_strlcpy(src_setting.location, node->child->value.text.string, LOCATION_STR_LEN);
            LOG_DEBUG("filesrc location %s", src_setting.location);
        }else
        if(strcmp(name, "v4l2src-device") == 0)
        {
            g_strlcpy(src_setting.v4l2_device, node->child->value.text.string, DEVICE_STR_LEN);
            LOG_DEBUG("v4l2_device  %s", src_setting.v4l2_device);

        }else
        if(strcmp(name, "v4l2src-color-effect") == 0)
        {
            global_v4l2src_color_effect = atoi(node->child->value.text.string);
            LOG_DEBUG("v4l2src_color_effect %d", global_v4l2src_color_effect);

        }else
        if(strcmp(name, "v4l2src-sensor-id") == 0)
        {
            src_setting.v4l2_sensor_id = atoi(node->child->value.text.string);

            LOG_DEBUG("v4l2_sensor_id %d", src_setting.v4l2_sensor_id);

        }else
        if(strcmp(name, "v4l2src-io-mode") == 0)
        {
            src_setting.v4l2_io_mode = atoi(node->child->value.text.string);

            if(src_setting.v4l2_io_mode != MMAP_MODE &&
               src_setting.v4l2_io_mode != DMA_MODE)
            {
                LOG_ERROR("unsupported V4L2 io-mode setting");
                result = FALSE;
                goto Exit;
            }

            LOG_DEBUG("v4l2_io_mode %d", src_setting.v4l2_io_mode);

        }else
        if(strcmp(name, "enable-3a") == 0)
        {
            src_setting.enable_3a = atoi(node->child->value.text.string);

            LOG_DEBUG("enable 3A: %d", src_setting.enable_3a);

        }else
#if 0
        if(strcmp(name, "capture-mode") == 0)
        {
            src_setting.capture_mode = atoi(node->child->value.text.string);

            LOG_DEBUG("capture-mode %d", src_setting.capture_mode);

        }else
#endif
        if(strcmp(name, "frame-rate") == 0)
        {
            src_setting.frame_rate = atoi(node->child->value.text.string);

            LOG_DEBUG("frame rate %d", src_setting.frame_rate);

        }else
        if(strcmp(name, "run-time") == 0)
        {
            src_setting.run_time_in_sec = atoi(node->child->value.text.string);

            LOG_DEBUG("running time in second %d", src_setting.run_time_in_sec);

        }else
        if(strcmp(name, "smart-factor") == 0)
        {
           smart_factor=
              atof(node->child->value.text.string);

           LOG_DEBUG("\t smart-factor:%f", smart_factor);

        }else
        if(strcmp(name, "smart-resolution") == 0)
        {
            src_setting.smart_resolution =
                (SmartResolution)atoi(node->child->value.text.string);

            LOG_DEBUG("\t smart-resolution :%d", src_setting.smart_resolution);

        }else
        if(strcmp(name, "enable-facedetect") == 0)
        {
           enable_facedetect = atoi(node->child->value.text.string);
           LOG_DEBUG("\t enable-facedetect :%d", enable_facedetect);
        }else
        if(strcmp(name, "facedetect-interval") == 0)
        {
           face_detect_interval = atoi(node->child->value.text.string);
           LOG_DEBUG("\t facedetect interval:%d", face_detect_interval);
        }else
        if(strcmp(name, "facedetect-conf") == 0)
        {
           facedetect_conf = atoi(node->child->value.text.string);
           LOG_DEBUG("\t facedetect-conf :%d", facedetect_conf);
        }else
        if(strcmp(name, "facedetect-mode") == 0)
        {
           facedetect_mode = atoi(node->child->value.text.string);
           LOG_DEBUG("\t facedetect-mode :%d", facedetect_mode);
        }else
        if(strcmp(name, "enable-getfacepic") == 0)
        {
           enable_getfacepic = atoi(node->child->value.text.string);
           LOG_DEBUG("\t enable-getfacepic :%d", enable_getfacepic);
        }else
        if(strcmp(name, "enable-facerecognise") == 0)
        {
           enable_facerecognise = atoi(node->child->value.text.string);
           LOG_DEBUG("\t enable-facerecognise :%d", enable_facerecognise);
        }else
        if(strcmp(name, "enable-lumagain-threshold") == 0)
        {
           enable_lumagain_threshold = atoi(node->child->value.text.string);
           LOG_DEBUG("\t enable-lumagain-threshold :%d", enable_lumagain_threshold);
        }else
        if(strcmp(name, "lumagain-threshold-value") == 0)
        {
           lumagain_threshold_value = atoi(node->child->value.text.string);
           LOG_DEBUG("\t lumagain-threshold-value :%d", lumagain_threshold_value);
        }else
        if(strcmp(name, "enable-autohdr") == 0)
        {
           enable_hdr = atoi(node->child->value.text.string);
           LOG_DEBUG("\t enable-autohdr :%d", enable_hdr);
        }else
       if(strcmp(name, "autohdr-mode") == 0)
        {
           auto_hdr_mode = (GstVideoPreprocAutoHDRMode)atoi(node->child->value.text.string);
           LOG_DEBUG("\t autohdr_mode :%d", (int)auto_hdr_mode);
        }else
        if(strcmp(name, "enable-hdr-custom") == 0)
        {
           enable_hdr_custom = atoi(node->child->value.text.string);
           LOG_DEBUG("\t enable-hdr-custom :%d", enable_hdr_custom);
        }else
        if(strcmp(name, "enable-hdr-custom-rgb") == 0)
        {
           enable_hdr_custom_rgb = atoi(node->child->value.text.string);
           LOG_DEBUG("\t enable-hdr-custom-rgb :%d", enable_hdr_custom_rgb);
        }else
        if(strcmp(name, "hdr-custom-table") == 0)
        {
            guint hdr_custom_table = atoi(node->child->value.text.string);
            reset_hdr_table((guchar)hdr_custom_table);
            LOG_DEBUG("hdr-custom-table %d", hdr_custom_table);
        }else
        if(strcmp(name, "hdr-custom-rgb-table") == 0)
        {
            guint hdr_custom_rgb_table = atoi(node->child->value.text.string);
            reset_hdr_rgb_table((guchar)hdr_custom_rgb_table);
            LOG_DEBUG("hdr-custom-rgb-table %d", hdr_custom_rgb_table);
        }else
	if(strcmp(name, "enable-bright-compensation") == 0)
	{
		enable_bright_compensation = (atoi(node->child->value.text.string) == 1 ? TRUE : FALSE);
		LOG_DEBUG("\t enable_bright_compensation: %d", enable_bright_compensation);
	}else
        if(strcmp(name, "enable-qos") == 0)
        {
            enable_qos = (atoi(node->child->value.text.string) == 1 ? TRUE : FALSE);
            LOG_DEBUG("\t enable-qos: %d", enable_qos);

        }else
        if(strcmp(name, "rotation") == 0)
        {
            config.rotation =
                (GstVideoPreprocRotateMode)atoi(node->child->value.text.string);

            LOG_DEBUG("\t rotation :%d", config.rotation);

        }else
        if(strcmp(name, "flip") == 0)
        {
            global_flip_mode=
                (GstVideoPreprocFlipMode)atoi(node->child->value.text.string);

            LOG_DEBUG("\t flip :%d", global_flip_mode);

        }else
        if(strcmp(name, "enable-mask") == 0)
        {
            global_enable_mask =
                atoi(node->child->value.text.string) == 0?FALSE:TRUE;

            LOG_DEBUG("\t global_enable_mask :%d", global_enable_mask);

        }else
        if(strcmp(name, "enable-wireframe") == 0)
        {
            global_enable_wireframe=
                atoi(node->child->value.text.string) == 0?FALSE:TRUE;

            LOG_DEBUG("\t global_enable_wireframe:%d", global_enable_wireframe);

        }else
           if(strcmp(name, "test-osd") == 0)
        {
            global_enable_osd=
                atoi(node->child->value.text.string) == 0?FALSE:TRUE;

            LOG_DEBUG("\t global_enable_osd:%d", global_enable_osd);

        }else
        if(strcmp(name, "test-toggle-channel") == 0)
        {
            test_toggle_channel =
                atoi(node->child->value.text.string) == 0?FALSE:TRUE;

            LOG_DEBUG("\t test_toggle_channel:%d", test_toggle_channel);

        }else

        if(strcmp(name, "warning-level") == 0)
        {
            warning_level =
                atoi(node->child->value.text.string);

            LOG_DEBUG("\t warning level:%u", warning_level);

        }else
        if(strcmp(name, "smart-control") == 0)
        {
            smart_control =
                atoi(node->child->value.text.string) == 0?FALSE:TRUE;

            LOG_DEBUG("\t smart_control:%d", smart_control);

        }else
        if(strcmp(name, "dvs-rot-00") == 0)
        {
            rot_00 =
                atof(node->child->value.text.string);
            LOG_DEBUG("\t dvs-rot-00:%f", rot_00);
        }else
        if(strcmp(name, "dvs-rot-01") == 0)
        {
            rot_01 =
                atof(node->child->value.text.string);
            LOG_DEBUG("\t dvs-rot-01:%f", rot_01);
        }else
        if(strcmp(name, "dvs-rot-10") == 0)
        {
            rot_10 =
                atof(node->child->value.text.string);
            LOG_DEBUG("\t dvs-rot-10:%f", rot_10);
        }else
        if(strcmp(name, "dvs-rot-11") == 0)
        {
            rot_11 =
                atof(node->child->value.text.string);
            LOG_DEBUG("\t dvs-rot-11:%f", rot_11);
        }else
        if(strcmp(name, "dvs-offset-x") == 0)
        {
            dvs_offset_x =
                atoi(node->child->value.text.string);
            LOG_DEBUG("\t dvs_offset_x:%d", dvs_offset_x);
        }else
        if(strcmp(name, "dvs-offset-y") == 0)
        {
            dvs_offset_y =
                atoi(node->child->value.text.string);
            LOG_DEBUG("\t dvs_offset_y:%d", dvs_offset_y);
        }else
        if(strcmp(name, "smart-bright") == 0)
        {
            smart_bright =
                atoi(node->child->value.text.string);
            LOG_DEBUG("\t smart_bright:%d", smart_bright);
        }else
        if(strcmp(name, "luma-gain") == 0)
        {
            config.luma_gain = atoi(node->child->value.text.string);

            LOG_DEBUG("\t default luma gain :%d", config.luma_gain);

        }else
        if (strcmp(name, "xcam-mode") == 0)
        {
            src_setting.image_processor = atoi(node->child->value.text.string);
            LOG_DEBUG("\t xcam image processor: %s", src_setting.image_processor?"CL":"ISP");
        }else
        if (strcmp(name, "analyzer") == 0)
        {
            src_setting.analyzer = atoi(node->child->value.text.string);
            LOG_DEBUG("\t xcam analyzer: %d", src_setting.analyzer);
        }else
        if (strcmp(name, "autohdr-cl-mode") == 0)
        {
            src_setting.cl_hdr_mode = atoi(node->child->value.text.string);
            LOG_DEBUG("\t xcam cl hdr mode: %d", src_setting.cl_hdr_mode);
        }else
        if (strcmp(name, "denoise-mode") == 0)
        {
            src_setting.cl_denoise_mode = atoi(node->child->value.text.string);
            LOG_DEBUG("\t xcam cl denoise mode: %d", src_setting.cl_denoise_mode);
        }else
        if (strcmp(name, "gamma-mode") == 0)
        {
            src_setting.cl_gamma_mode = atoi(node->child->value.text.string);
            LOG_DEBUG("\t xcam cl gamma mode: %d", src_setting.cl_gamma_mode);
        }else
        if (strcmp(name, "enable-dpc") == 0)
        {
            src_setting.enable_dpc = atoi(node->child->value.text.string);
            LOG_DEBUG("\t xcam cl dpc mode: %d", src_setting.enable_dpc);
        }else
        if(strcmp(name, "channel") == 0)
        {
            channel = atoi(mxmlElementGetAttr(node, "id"));
            if(channel >= VIDEO_CHANNEL_MAX)
            {
                LOG_ERROR("only %d channels supported", VIDEO_CHANNEL_MAX);
                result = FALSE;
                goto Exit;
            }

            config.channel_config[channel].enable_channel =
                atoi(mxmlElementGetAttr(node, "enable")) == 0?FALSE:TRUE;

            LOG_DEBUG("\t parsing video channel:%d, enable:%d",
                       channel,
                       config.channel_config[channel].enable_channel);

        }else
        if(strcmp(name, "encoder") == 0)
        {
            config.channel_config[channel].enable_h264 =
                    atoi(mxmlElementGetAttr(node, "enable")) == 0?FALSE:TRUE;

            LOG_DEBUG("\t\t enable h264 encoder:%d", config.channel_config[channel].enable_h264);

            if(!config.channel_config[channel].enable_h264)
            {
                config.channel_config[channel].bitrate = 0;
                config.channel_config[channel].enable_cabac = TRUE;
                config.channel_config[channel].enable_dct8x8 = TRUE;
                config.channel_config[channel].mv = 0;
                config.channel_config[channel].gop_M = 30;
                config.channel_config[channel].gop_N = 0;
            }else
            {
                config.channel_config[channel].rate_control =
                            atoi(mxmlElementGetAttr(node, "rate-control"));
                if(config.channel_config[channel].rate_control != VA_CQP &&
                   config.channel_config[channel].rate_control != VA_CBR &&
                   config.channel_config[channel].rate_control != VA_VBR &&
                   config.channel_config[channel].rate_control != VA_VBR_CONSTRAINED)
                {
                    LOG_ERROR("Wrong setting for rate-control");
                    result = FALSE;
                    goto Exit;
                }

                config.channel_config[channel].bitrate =
                            atoi(mxmlElementGetAttr(node, "bitrate"));
                config.channel_config[channel].enable_cabac =
                            atoi(mxmlElementGetAttr(node, "enable-cabac")) == 0?FALSE:TRUE;
                config.channel_config[channel].enable_dct8x8 =
                            atoi(mxmlElementGetAttr(node, "enable-dct8x8")) == 0?FALSE:TRUE;
                config.channel_config[channel].mv =
                            atoi(mxmlElementGetAttr(node, "mv"));

                config.channel_config[channel].gop_M =
                            atoi(mxmlElementGetAttr(node, "gop-m"));
                config.channel_config[channel].gop_N =
                            atoi(mxmlElementGetAttr(node, "gop-n"));

                if(config.channel_config[channel].gop_N > 10)
                {
                    LOG_ERROR("Wrong setting for gop_N, should be <= 10");
                    result = FALSE;
                    goto Exit;
                }

                config.channel_config[channel].profile =
                            atoi(mxmlElementGetAttr(node, "profile"));
                if(config.channel_config[channel].profile > 2)
                {
                    LOG_ERROR("Wrong setting for profile");
                    result = FALSE;
                    goto Exit;
                }
            }

            LOG_DEBUG("\t\t rate-control:%d, bitrate:%d, cabac:%d, dct_8x8:%d, mv:%d, gop:%d/%d, profile:%d",
                        config.channel_config[channel].rate_control,
                        config.channel_config[channel].bitrate,
                        config.channel_config[channel].enable_cabac,
                        config.channel_config[channel].enable_dct8x8,
                        config.channel_config[channel].mv,
                        config.channel_config[channel].gop_M,
                        config.channel_config[channel].gop_N,
                        config.channel_config[channel].profile);
        }else
        if(strcmp(name, "sink-type") == 0)
        {
            config.channel_config[channel].sink_type =
                    (MediaSinkType)atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t sink_type:%d", config.channel_config[channel].sink_type);

        }else
        if(strcmp(name, "host-ip") == 0)
        {
           strcpy(config.channel_config[channel].host_ip, node->child->value.text.string);
            LOG_DEBUG("\t\t host-ip:%s", config.channel_config[channel].host_ip);
        }else
        if(strcmp(name, "port") == 0)
        {
           config.channel_config[channel].port=
              atoi(node->child->value.text.string);
            LOG_DEBUG("\t\t port:%d", config.channel_config[channel].port);
        }else
        if(strcmp(name, "jpeg-out-mode") == 0)
        {
            jpgenc_misc_config.jpg_out_mode =
                    (JpegOutputMode)atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t jpeg output mode:%d", jpgenc_misc_config.jpg_out_mode);

        }else
        if(strcmp(name, "jpeg-interval") == 0)
        {
            jpgenc_misc_config.jpg_interval=
                    atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t jpeg inverval:%d", jpgenc_misc_config.jpg_interval);

        }else
        if(strcmp(name, "jpeg-crop") == 0)
        {
            jpgenc_misc_config.jpeg_crop_enable =
                    atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t jpeg crop enable:%d", jpgenc_misc_config.jpeg_crop_enable);

        }else
        if(strcmp(name, "jpeg-crop-offset-x") == 0)
        {
            jpgenc_misc_config.jpeg_crop_ox =
                    atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t jpeg crop startx:%d", jpgenc_misc_config.jpeg_crop_ox);

        }else
        if(strcmp(name, "jpeg-crop-offset-y") == 0)
        {
            jpgenc_misc_config.jpeg_crop_oy =
                    atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t jpeg crop starty:%d", jpgenc_misc_config.jpeg_crop_oy);

        }else
        if(strcmp(name, "jpeg-crop-width") == 0)
        {
            jpgenc_misc_config.jpeg_crop_width =
                    atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t jpeg crop width:%d", jpgenc_misc_config.jpeg_crop_width);

        }else
        if(strcmp(name, "jpeg-crop-height") == 0)
        {
            jpgenc_misc_config.jpeg_crop_height =
                    atoi(node->child->value.text.string);

            LOG_DEBUG("\t\t jpeg crop height:%d", jpgenc_misc_config.jpeg_crop_height);
        }

        node = mxmlFindElement(node, tree, NULL, NULL, NULL, MXML_DESCEND);
    }

    if(!config.channel_config[VIDEO_CHANNEL_1080P].enable_channel)
    {
        LOG_ERROR("Wrong setting for default channel: Channel_%d",
                   VIDEO_CHANNEL_1080P);
        result = FALSE;
        goto Exit;
    }

Exit:
    fclose(fp);

    return result;
}

static void
init_config()
{
	memset(&jpgenc_misc_config, 0, sizeof(jpgenc_misc_config));
	g_mutex_init(&jpgenc_misc_config.lock);
        memset(&jpgenc_misc_config2, 0, sizeof(jpgenc_misc_config2));
	g_mutex_init(&jpgenc_misc_config2.lock);

   //only config global option
   config.rotation = GST_VIDEO_PREPROC_ROTATE_90; //2;
   config.enable_mask = 1;
   config.luma_gain   = 100;
   config.enable_wireframe = 0;
}

gboolean init_reconfig_3a(int argc, char *argv[])
{	int c;
	int optA = 0;

	while ( (c = getopt(argc, argv, "A")) != -1) {
		switch (c) {
		case 'A':
			optA = 1;
			break;
		default:
			printf ("?? getopt returned character code 0%o ??\n", c);
		}
	}

	return (optA == 1);
}

int main (int argc,  char *agrv[])
{
    gint ret = 0;
    gboolean result = TRUE;
    MediaPipe *media_pipe;
    GIOChannel *io_stdin = NULL;
    gboolean reconfig_3a;

    guint channel;
    printf("Mediapipe: Build Date %s %s version unknown\r\n",__DATE__,__TIME__);
    g_mutex_init(&jpeg_lock);
    init_config();
    if(!parse_config())
    {
        LOG_ERROR("config file error");
        return 0;
    }

    // init rabbitmq
    rabbitmq_init();

    LOG_DEBUG ("----------------Config parsing done----------------------");

    media_pipe = media_pipe_create (argc, agrv);
    g_assert (media_pipe);
    if(media_pipe == NULL)
    {
        LOG_ERROR("Create main MediaPipe Fail");
        result = FALSE;
        goto end;
    }

    /*
    *   Source configuration
    */
    result = media_pipe_set_filesrc_type (media_pipe, src_setting.src_type);
    if(!result)
    {
        LOG_ERROR("Set source type Fail");
        goto end;
    }

    result = media_pipe_set_filesrc_location (media_pipe, src_setting.location);
    if(!result)
    {
        LOG_ERROR("Set source location Fail");
        goto end;
    }

    result = media_pipe_set_use_v4l2_src (media_pipe, src_setting.use_v4l2_src);
    if(!result)
    {
        LOG_ERROR("Set use_v4l2_src Fail");
        goto end;
    }

    result = media_pipe_set_v4l2_src_device (media_pipe, src_setting.v4l2_device);
    if(!result)
    {
       LOG_ERROR("Set v4l2_device Fail");
       goto end;
    }

    result = media_pipe_set_v4l2_src_sensor_id (media_pipe, src_setting.v4l2_sensor_id);
    if(!result)
    {
        LOG_ERROR("Set v4l2_sensor_id Fail");
        goto end;
    }

    result = media_pipe_set_v4l2_src_io_mode (media_pipe, src_setting.v4l2_io_mode);
    if(!result)
    {
        LOG_ERROR("Set v4l2_io_mode Fail");
        goto end;
    }

    result = media_pipe_set_v4l2_src_enable_3a(media_pipe, src_setting.enable_3a);
    if(!result)
    {
        LOG_ERROR("Set enable_3a Fail");
        goto end;
    }

    result = media_pipe_set_v4l2_src_capture_mode(media_pipe, src_setting.capture_mode);
    if(!result)
    {
        LOG_ERROR("Set capture mode Fail");
        goto end;
    }

    result = media_pipe_set_src_frame_rate (media_pipe, src_setting.frame_rate);
    if(!result)
    {
        LOG_ERROR("Set frame rate Fail");
        goto end;
    }

    result = media_pipe_set_src_image_processor (media_pipe, src_setting.image_processor, src_setting.analyzer);
    if(!result)
    {
        LOG_ERROR("Set image processor Fail");
        goto end;
    }

    media_pipe_set_cl_feature (media_pipe, CL_HDR, src_setting.cl_hdr_mode);
    media_pipe_set_cl_feature (media_pipe, CL_DENOISE, src_setting.cl_denoise_mode);
    media_pipe_set_cl_feature (media_pipe, CL_GAMMA, src_setting.cl_gamma_mode);
    media_pipe_set_cl_feature (media_pipe, CL_DPC, src_setting.enable_dpc);

    result = media_pipe_set_src_size (media_pipe, 1920, 1080);
    if(!result)
    {
        LOG_ERROR("Set Src resolution Fail");
        goto end;
    }

    result = media_pipe_set_src_format (media_pipe, GST_VIDEO_FORMAT_NV12);
    if(!result)
    {
        LOG_ERROR("Set Src format Fail");
        goto end;
    }

    // set smart  config
    result = media_pipe_set_src_smart_resolution (
                                        media_pipe,
                                        src_setting.smart_resolution);
    if(!result)
    {
        LOG_ERROR("Set smart resolution Fail");
        goto end;
    }

    if(smart_bright)
    {
       result = media_pipe_set_src_frame_smart_callback (media_pipe,
             video_frame_bright_callback,
             NULL);
    }else {
       result = media_pipe_set_src_frame_smart_callback (media_pipe,
             video_frame_smart_callback,
             NULL);
    }

    if(!result)
    {
        LOG_ERROR("Set smart resolution Fail");
        goto end;
    }

    result = media_pipe_set_src_parse_3aconf_callback (
                                                    media_pipe,
                                                    (Parse3AConfCallback)parse_3aconf_callback,
                                                    NULL);
    if(!result)
    {
        LOG_ERROR("Set src parse 3aconf callback Fail");
        goto end;
    }

    result = media_pipe_set_vpp_src_enable_autohdr(media_pipe, src_setting.enable_autohdr);
    if(!result)
    {
        LOG_ERROR("Set enable_autohdr Fail");
        goto end;
    }
    result = media_pipe_set_vpp_src_set_autohdrmode(media_pipe, src_setting.autohdr_mode);
    if(!result)
    {
        LOG_ERROR("Set set_autohdrmode Fail");
        goto end;
    }
    /*
        *   All channels configuration
        */
    // rotation
    result = media_pipe_set_video_preproc_rotation (
                                        media_pipe,
                                        config.rotation);
    if(!result)
    {
        LOG_ERROR("Set Preproc rotation_mode Fail");
        goto end;
    }

    // luminance gain
    result = media_pipe_set_video_preproc_luma (
                                              media_pipe,
                                              config.luma_gain);
    if(!result)
    {
       LOG_ERROR("Set Preproc luma gain Fail");
       goto end;
    }

    // channels
    for(channel=0; channel<VIDEO_CHANNEL_MAX; channel++)
    {
        // h264 toggle
        result = media_pipe_set_channel_encoder_toggle(
                                        media_pipe,
                                        (VideoChannelIndex)channel,
                                        config.channel_config[channel].enable_h264);
        if(!result)
        {
            LOG_ERROR("Enable channel encoder Fail");
            goto end;
        }


        result = media_pipe_set_channel_encoder_frame_callback (media_pipe,
                                                (VideoChannelIndex)channel,
                                                (EncodeFrameCallback)h264_frame_callback,
                                                NULL);
        if(!result)
        {
            LOG_ERROR("Set encoder_frame callback Fail");
            goto end;
        }

        // encoder properties setting
        if(config.channel_config[channel].enable_h264)
        {

            result = media_pipe_set_channel_encoder_rate_control(
                                            media_pipe,
                                            (VideoChannelIndex)channel,
                                            config.channel_config[channel].rate_control);
            if(!result)
            {
                LOG_ERROR("Set encoder rate_control Fail");
                goto end;
            }

            result = media_pipe_set_channel_encoder_bitrate(
                                            media_pipe,
                                            (VideoChannelIndex)channel,
                                            config.channel_config[channel].bitrate);
            if(!result)
            {
                LOG_ERROR("Set encoder bitrate Fail");
                goto end;
            }

            result = media_pipe_set_channel_encoder_enable_cabac(
                                            media_pipe,
                                            (VideoChannelIndex)channel,
                                            config.channel_config[channel].enable_cabac);
            if(!result)
            {
                LOG_ERROR("Enable channel encoder cabac Fail");
                goto end;
            }

            result = media_pipe_set_channel_encoder_enable_dct8x8(
                                            media_pipe,
                                            (VideoChannelIndex)channel,
                                            config.channel_config[channel].enable_dct8x8);
            if(!result)
            {
                LOG_ERROR("Enable channel encoder cabac Fail");
                goto end;
            }

            result = media_pipe_set_channel_encoder_mv(
                                            media_pipe,
                                            (VideoChannelIndex)channel,
                                            config.channel_config[channel].mv);
            if(!result)
            {
                LOG_ERROR("Enable channel encoder MV Fail");
                goto end;
            }

            result = media_pipe_set_channel_encoder_gop(
                                            media_pipe,
                                            (VideoChannelIndex)channel,
                                            config.channel_config[channel].gop_M,
                                            config.channel_config[channel].gop_N);
            if(!result)
            {
                LOG_ERROR("Set encoder GOP Fail");
                goto end;
            }

            result = media_pipe_set_channel_encoder_profile(
                                            media_pipe,
                                            (VideoChannelIndex)channel,
                                            config.channel_config[channel].profile);
            if(!result)
            {
                LOG_ERROR("Set encoder profile Fail");
                goto end;
            }
        }

        // sink type setting
        result = media_pipe_set_channel_sink_type(
                                        media_pipe,
                                        (VideoChannelIndex)channel,
                                        config.channel_config[channel].sink_type,
                                        config.channel_config[channel].host_ip,
                                        config.channel_config[channel].port);
        if(!result)
        {
            LOG_ERROR("Set channel sink type Fail");
            goto end;
        }

        // enable/disable channel
        if(config.channel_config[channel].enable_channel)
        {
            result = media_pipe_enable_video_channel(media_pipe, (VideoChannelIndex)channel);
        }else
        {
            result = media_pipe_disable_video_channel(media_pipe, (VideoChannelIndex)channel);
        }

        if(!result)
        {
            LOG_ERROR("enable/disable channel Fail");
            goto end;
        }
    }

    //can't use jpeg if we use kms sink due to the "master" drm device issue
    result = media_pipe_set_video_preproc_frame_jpeg_callback (media_pipe,
          video_frame_jpeg_callback,
          &jpgenc_misc_config);
    if(!result)
    {
       LOG_ERROR("Set video_frame jpeg callback Fail");
       goto end;
    }

    media_pipe_set_message_callback (media_pipe, message_callback, NULL);

    // timer thread to terminate pipeline  when timeout
    g_thread_unref(g_thread_new(NULL, run_timer_func, media_pipe));
    // test thread to turn on/off channels dynamically
    if (test_toggle_channel)
       g_thread_unref(g_thread_new(NULL, dynamic_channel_thread_func, media_pipe));
	// test thread to test some dynamically changable properties
	//g_thread_unref(g_thread_new(NULL, dynamic_update_params_thread_func, media_pipe));

	// rabbitmq listening thread
	g_thread_unref(g_thread_new(NULL, rabbitmq_listening_func, media_pipe));

    io_stdin = g_io_channel_unix_new (fileno (stdin));
    g_io_add_watch (io_stdin, G_IO_IN, (GIOFunc)handle_keyboard, media_pipe);
    reconfig_3a = init_reconfig_3a (argc, agrv);
    if (!media_pipe_start(media_pipe, reconfig_3a)) {
        ret = -1;
        LOG_ERROR ("pipeline start failed");
        goto end;
    }
    
end:
    media_pipe_destroy (media_pipe);
    if (io_stdin)
       g_io_channel_unref (io_stdin);

	g_mutex_clear(&jpgenc_misc_config.lock);
	g_mutex_clear(&jpgenc_misc_config2.lock);

	g_mutex_clear(&jpeg_lock);

    LOG_DEBUG("--------------MediaPipe ends correctly---------------");
    return ret;
}

#define CONFIG_3ACONF_FILE_NAME "/etc/mediapipe/conf_3a.xml"

static gboolean
generate_gammatable(double *lut,    double fgamma)
{
    gint i;
    const gint maxindex = GAMMATABLESIZE-1;

    for( i = 0; i <= maxindex; i++ )
    {
        lut[i] = pow(double((double)i/(double)(maxindex)), fgamma);
    }

    g_print("gamma table (%.4f): \n", fgamma);
    for( i = 0; i <= maxindex; i++ )
    {
        g_print("%s%d:%.4f", i%8?" ":"\n", i, lut[i]);
    }
    g_print("\n");

    return TRUE;
}

static void
reset_3a_config_white_balance (Cameara3a_WhiteBalance *wb_config)
{
    memset (wb_config, 0, sizeof (*wb_config));
    wb_config->val_wb_mode = XCAM_AWB_MODE_AUTO;
    wb_config->val_awb_speed = 1.0;
    wb_config->val_awb_cct_min = 0;
    wb_config->val_awb_cct_max = 0;
    wb_config->val_awb_gr = 0.0;
    wb_config->val_awb_r = 0.0;
    wb_config->val_awb_gb = 0.0;
    wb_config->val_awb_b = 0.0;
}

static void
reset_3a_config_exposure (Cameara3a_Exposure *ep_config)
{
    /* reset all ep_config */
    memset (ep_config, 0, sizeof (*ep_config));
    ep_config->val_ep_mode = XCAM_AE_MODE_AUTO;
    ep_config->val_meter_mode = XCAM_AE_METERING_MODE_AUTO;
    //ep_config->val_ep_window[0] = 0;
    ep_config->val_ep_offset = 0.0;
    ep_config->val_ep_speed = 1.0;
    ep_config->val_ep_flicker = XCAM_AE_FLICKER_MODE_AUTO;
    ep_config->val_ep_manual_time = 0;
    ep_config->val_ep_manual_analoggain = 0.0;
    ep_config->val_ep_max_analoggain = 0.0;
    ep_config->val_ep_timerange_min = 0;
    ep_config->val_ep_timerange_max = 0;
}

static void
reset_3a_config_picture_quality (Cameara3a_PicQuality *pq_config)
{
    /* reset all pq_config */
    memset (pq_config, 0, sizeof (*pq_config));
    pq_config->val_noise_reduction_level = 128;
    pq_config->val_tnr_level = 128;
    pq_config->val_tnr_mode = 0;
    pq_config->val_pq_brightness = 128;
    pq_config->val_pq_contrast = 128;
    pq_config->val_pq_hue = 128;
    pq_config->val_pq_saturation = 128;
    pq_config->val_pq_sharpness = 128;
}

static void
reset_3a_config_others (Cameara3a_Others *others_config)
{
    /* reset others_config */
    memset (others_config, 0, sizeof (*others_config));
    others_config->conf_gm_table = FALSE;
    others_config->val_gm_gbce = FALSE;
    others_config->val_night_mode = FALSE;
}


static void
reset_3a_config (Config3A *config_3a)
{
    config_3a->flags = 0;
    reset_3a_config_white_balance (&config_3a->wb);
    reset_3a_config_exposure (&config_3a->ep);
    reset_3a_config_picture_quality (&config_3a->pq);
    reset_3a_config_others (&config_3a->others);
}

static gboolean
parse_3a_config_white_balance(mxml_node_t *wb_node, Cameara3a_WhiteBalance *wb_config )
{
    mxml_node_t *node;

    LOG_DEBUG ("parsing white balance");
    for (node = mxmlGetFirstChild (wb_node); node != NULL; node = mxmlGetNextSibling (node))
    {
        gchar *name;
        if (MXML_ELEMENT != node->type)
            continue;

        name = (gchar*)mxmlGetElement (node);

        if (strcmp(name, "mode") == 0) {
            wb_config->val_wb_mode = atoi(node->child->value.text.string);
            LOG_DEBUG ("\tparsing wb-mode (%d)", wb_config->val_wb_mode );
        }else
        if (strcmp(name, "speed") == 0) {
            wb_config->val_awb_speed = atof(node->child->value.text.string);
            LOG_DEBUG ("\tparsing awb-speed (%.3f)", wb_config->val_awb_speed);
        }else
        if(strcmp(name, "color-temperature") == 0)
        {
            wb_config->val_awb_cct_min = atoi(mxmlElementGetAttr(node, "cct_min"));
            wb_config->val_awb_cct_max = atoi(mxmlElementGetAttr(node, "cct_max"));
            LOG_DEBUG("\tparsing awb-color-temperature (%d, %d)", wb_config->val_awb_cct_min, wb_config->val_awb_cct_max );
        }else
        if(strcmp(name, "manual-gain") == 0)
        {
            wb_config->val_awb_gr  = atof(mxmlElementGetAttr(node, "gr"));
            wb_config->val_awb_r  = atof(mxmlElementGetAttr(node, "r"));
            wb_config->val_awb_gb  = atof(mxmlElementGetAttr(node, "gb"));
            wb_config->val_awb_b  = atof(mxmlElementGetAttr(node, "b"));

            LOG_DEBUG("\tparsing awb-manual-gain (%.3f, %.3f, %.3f, %.3f)", wb_config->val_awb_gr, wb_config->val_awb_r, wb_config->val_awb_gb, wb_config->val_awb_b );
        }
    }
    return TRUE;
}

static gboolean
parse_3a_config_exposure(mxml_node_t *ep_node, Cameara3a_Exposure *ep_config)
{
    mxml_node_t *node;

    LOG_DEBUG ("parsing exposure");
    for (node = mxmlGetFirstChild (ep_node); node != NULL; node = mxmlGetNextSibling (node))
    {
        gchar *name;
        if (MXML_ELEMENT != node->type)
            continue;

        name = (gchar*)mxmlGetElement (node);

        if (strcmp(name, "mode") == 0) {
            ep_config->val_ep_mode = atoi(node->child->value.text.string);
            LOG_DEBUG ("\tparsing exposure-mod (%d)", ep_config->val_ep_mode );
        } else
        if (strcmp(name, "meter-mode") == 0)
        {
            ep_config->val_meter_mode= atoi(node->child->value.text.string);

			GstVideoPreprocWireFrame *wf = sample_wire_frames;
			gint wf_window_num = UI_EXPOSE_WINDOW_NUM;
			// spot
			if(ep_config->val_meter_mode == 1) {
				// disable/enable wire frame
				for (gint j = 0; j < WIRE_FRAME_REGION_MAX_NUM; j++) {
					if(j == 0) {
						LOG_DEBUG ("Now set exposure window %d", j);
						wf[j].enable = TRUE; 
					} else {
						LOG_DEBUG ("Now clean exposure window %d", j);
						wf[j].enable = FALSE; 
					}
				}
			}
			// weighted windows
			else if(ep_config->val_meter_mode == 3) {
				// disable/enable wire frame
				for (gint j = 0; j < WIRE_FRAME_REGION_MAX_NUM; j++) {
					if(j < wf_window_num) {
						LOG_DEBUG ("Now set exposure window %d", j);
						wf[j].enable = TRUE; 
					} else {
						LOG_DEBUG ("Now clean exposure window %d", j);
						wf[j].enable = FALSE; 
					}
				}
			}
			else {
				for (gint j = 0; j < WIRE_FRAME_REGION_MAX_NUM; j++) {
					LOG_DEBUG ("Now clean exposure window %d", j);
					wf[j].enable = FALSE; 
				}
			}
			
            LOG_DEBUG("\tparsing ae-meter-mode (%d)", ep_config->val_meter_mode );
        } else
        if (strcmp(name, "meter-window-count") == 0)
        {
            ep_config->val_ep_window_count = atof(node->child->value.text.string);
            if (XCAM_AE_MAX_METERING_WINDOW_COUNT < ep_config->val_ep_window_count) {
                LOG_WARNING("\tparsing metering window count(%d) error! reset count!", ep_config->val_ep_window_count);
                ep_config->val_ep_window_count = XCAM_AE_MAX_METERING_WINDOW_COUNT;
            }
            LOG_DEBUG("\tparsing metering window count (%d)", ep_config->val_ep_window_count);
        } else
        if (strcmp(name, "window") == 0)
        {
            for (gint i = 0; i < ep_config->val_ep_window_count; i++) {
                char xStart[16] = { 0 };
                char yStart[16] = { 0 };
                char xEnd[16] = { 0 };
                char yEnd[16] = { 0 };
                char weight[16] = { 0 };

                sprintf(xStart, "%s%d", "startx", i);				
                sprintf(yStart, "%s%d", "starty", i);
                sprintf(xEnd, "%s%d", "endx", i);
                sprintf(yEnd, "%s%d", "endy", i);
                sprintf(weight, "%s%d", "weight", i);

                LOG_DEBUG("\tparsing metering window, %s, %s, %s, %s, %s", xStart, yStart, xEnd, yEnd, weight);
                ep_config->val_ep_window[i].x_start = atoi(mxmlElementGetAttr(node, xStart));
                ep_config->val_ep_window[i].y_start = atoi(mxmlElementGetAttr(node, yStart));
                ep_config->val_ep_window[i].x_end = atoi(mxmlElementGetAttr(node, xEnd));
                ep_config->val_ep_window[i].y_end = atoi(mxmlElementGetAttr(node, yEnd));
                ep_config->val_ep_window[i].weight = atoi(mxmlElementGetAttr(node, weight));
            }
            LOG_DEBUG("\tparsing exposure window (%d, %d - %d, %d, weight: %d)", ep_config->val_ep_window[0].x_start, ep_config->val_ep_window[0].y_start,
                                    ep_config->val_ep_window[0].x_end, ep_config->val_ep_window[0].y_end, ep_config->val_ep_window[0].weight);

            // set wire frame as exposure window
            GstVideoPreprocWireFrame *wf = sample_wire_frames;
            for (gint j = 0; j < ep_config->val_ep_window_count; j++) {
				guint xpos = (guint)ep_config->val_ep_window[j].x_start;
				guint ypos = (guint)ep_config->val_ep_window[j].y_start;
				guint width = (guint)(ep_config->val_ep_window[j].x_end - ep_config->val_ep_window[j].x_start);
				guint height = (guint)(ep_config->val_ep_window[j].y_end - ep_config->val_ep_window[j].y_start);
				LOG_DEBUG ("Now set wire frame %d (x: %d, y: %d, w: %d, h: %d)", j, xpos, ypos, width, height);
				wf[j].region.x = xpos;
				wf[j].region.y = ypos;
				wf[j].region.w = width;
				wf[j].region.h = height;
            }
			
        }else
        if (strcmp(name, "ev-shift") == 0)
        {
            ep_config->val_ep_offset = atof(node->child->value.text.string);
            LOG_DEBUG("\tparsing ev shift (%.3f)", ep_config->val_ep_offset);
        } else
        if (strcmp(name, "speed") == 0)
        {
            ep_config->val_ep_speed = atof(node->child->value.text.string);
            LOG_DEBUG("\tparsing exposure speed (%.3f)", ep_config->val_ep_speed);
        } else
        if(strcmp(name, "flicker-mode") == 0)
        {
            ep_config->val_ep_flicker = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing exposure flicker mode (%d)", ep_config->val_ep_flicker);
        }else
        if (strcmp(name, "manual-exposure-time") == 0)
        {
            ep_config->val_ep_manual_time = atol(node->child->value.text.string);
            LOG_DEBUG("\tparsing exposure manual-exposure-time (%lld)", ep_config->val_ep_manual_time);
        } else
        if (strcmp(name, "manual-analog-gain") == 0)
        {
            ep_config->val_ep_manual_analoggain = atof(node->child->value.text.string);
            LOG_DEBUG("\tparsing exposure manual-analog-gain (%f)", ep_config->val_ep_manual_analoggain);
        }else
        if (strcmp(name, "max-analog-gain") == 0)
        {
            ep_config->val_ep_max_analoggain = atof(node->child->value.text.string);
            LOG_DEBUG("\tparsing exposure max-analog-gain (%f)", ep_config->val_ep_max_analoggain);
        } else
        if(strcmp(name, "exposure-time-range") == 0)
        {

            ep_config->val_ep_timerange_min = atoi(mxmlElementGetAttr(node, "min_time_us"));
            ep_config->val_ep_timerange_max = atoi(mxmlElementGetAttr(node, "max_time_us"));
            LOG_DEBUG("\tparsing exposure exposure-time-range (%lld - %lld)", ep_config->val_ep_timerange_min, ep_config->val_ep_timerange_max);
        }
    }
    return TRUE;

}


static gboolean
parse_3a_config_others (mxml_node_t *others_node, Cameara3a_Others *others_config)
{
    mxml_node_t *node;

    LOG_DEBUG ("parsing others");
    for (node = mxmlGetFirstChild (others_node); node != NULL; node = mxmlGetNextSibling (node))
    {
        gchar *name;
        if (MXML_ELEMENT != node->type)
            continue;

        name = (gchar*)mxmlGetElement (node);

        if(strcmp(name, "gamma-table") == 0)
        {
            double fgamma_r, fgamma_g, fgamma_b;

            fgamma_r = atof(mxmlElementGetAttr(node, "fgamma_r"));
            fgamma_g = atof(mxmlElementGetAttr(node, "fgamma_g"));
            fgamma_b = atof(mxmlElementGetAttr(node, "fgamma_b"));

            if (fgamma_r >= 0.1 && fgamma_g >= 0.1 && fgamma_b >= 0.1) {
                others_config->conf_gm_table = TRUE;
                generate_gammatable(&others_config->val_gm_table_r[0], fgamma_r);
                generate_gammatable(&others_config->val_gm_table_g[0], fgamma_g);
                generate_gammatable(&others_config->val_gm_table_b[0], fgamma_b);
            } else
                others_config->conf_gm_table = FALSE;

            LOG_DEBUG("\tparsing gamma-table (r:%.4f, g:%.4f, b:%.4f), enable:%s",
                fgamma_r, fgamma_g, fgamma_b,
                (others_config->conf_gm_table ? "TRUE" : "FALSE"));
        } else
        if(strcmp(name, "gbce") == 0)
        {
            others_config->val_gm_gbce = (atoi(node->child->value.text.string) > 0) ? TRUE:FALSE;
            LOG_DEBUG("\tparsing gbce (%d)", others_config->val_gm_gbce);
        } else
        if(strcmp(name, "night-mode") == 0)
        {
            others_config->val_night_mode = (atoi(node->child->value.text.string) > 0) ? TRUE:FALSE;
            LOG_DEBUG("\tparsing night mode (%d)", others_config->val_night_mode);
        }
        if(strcmp(name, "analyze-interval") == 0)
        {
            others_config->val_analyze_interval = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing analyze interval (%d)", others_config->val_analyze_interval);
        }
    }
    return TRUE;
}

static gboolean
parse_3a_config_picture_quality (mxml_node_t *pq_node, Cameara3a_PicQuality *pq_config)
{
    mxml_node_t *node;

    LOG_DEBUG ("parsing picture quality");
    for (node = mxmlGetFirstChild (pq_node); node != NULL; node = mxmlGetNextSibling (node))
    {
        gchar *name;
        if (MXML_ELEMENT != node->type)
            continue;

        name = (gchar*)mxmlGetElement (node);

        if (strcmp(name, "noise-reduction") == 0)
        {
            pq_config->val_noise_reduction_level = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing noise reduction level (%d)", pq_config->val_noise_reduction_level );
        } else
        if (strcmp(name, "temporal-noise-reduction") == 0)
        {
            pq_config->val_tnr_level = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing temporal noise reduction level (%d)", pq_config->val_tnr_level );
        } else
        if (strcmp(name, "tnr-mode") == 0)
        {
            pq_config->val_tnr_mode = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing temporal noise reduction mode (%d)", pq_config->val_tnr_mode );
        } else
        if (strcmp(name, "brightness") == 0)
        {
            pq_config->val_pq_brightness = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing mannual brightness (%d)", pq_config->val_pq_brightness );
        }else
        if(strcmp(name, "contrast") == 0)
        {
            pq_config->val_pq_contrast = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing mannual contrast (%d)", pq_config->val_pq_contrast );
        }else
        if(strcmp(name, "hue") == 0)
        {
            pq_config->val_pq_hue = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing mannual hue (%d)", pq_config->val_pq_hue );
        }else
        if(strcmp(name, "saturation") == 0)
        {
            pq_config->val_pq_saturation = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing mannual saturation (%d)", pq_config->val_pq_saturation );
        }else
        if(strcmp(name, "sharpness") == 0)
        {
            pq_config->val_pq_sharpness = atoi(node->child->value.text.string);
            LOG_DEBUG("\tparsing mannual sharpness (%d)", pq_config->val_pq_sharpness );
        }
    }

    return TRUE;
}

static gboolean
parse_3aconf_callback (Config3A *config_camera)
{
    gboolean result = TRUE;
    FILE *fp = NULL;
    mxml_node_t *tree = NULL;
    mxml_node_t *node = NULL;

    gboolean enable = FALSE;

    fp = fopen(CONFIG_3ACONF_FILE_NAME, "r");
    if(fp == NULL)
    {
        LOG_ERROR("no config file: /etc/mediapipe/conf_3a.xml");
        return FALSE;
    }

    tree = mxmlLoadFile(NULL,
                        fp,
                        MXML_NO_CALLBACK);
    if(tree == NULL)
    {
        LOG_ERROR("loading 3a config file Fail");
        result = FALSE;
        goto Exit;
    }

    // Parsing 3a config
    node = mxmlFindElement(tree, tree, "conf3a", NULL, NULL, MXML_DESCEND);
    reset_3a_config (config_camera);

    for (node = mxmlGetFirstChild (node); node != NULL; node = mxmlGetNextSibling (node))
    {

        gchar *name = (gchar*)mxmlGetElement(node);

        if (MXML_ELEMENT != node->type)
            continue;

        /*white-balance*/
        if (strcmp(name, "white-balance") == 0)
        {
            enable = (atoi(mxmlElementGetAttr(node, "enable")) > 0) ? TRUE : FALSE;
            if(enable && parse_3a_config_white_balance (node, &config_camera->wb))
                config_camera->flags |= CONFIGFLAG_3A_WHITEBALANCE;

            LOG_DEBUG("parsed white balance (%s)",
                ((config_camera->flags & CONFIGFLAG_3A_WHITEBALANCE) ? "true" : "false"));
        } else
        /*exposure*/
        if (strcmp(name, "exposure") == 0)
        {
            enable = (atoi(mxmlElementGetAttr(node, "enable")) > 0) ? TRUE : FALSE;
            if(enable && parse_3a_config_exposure (node, &config_camera->ep))
                config_camera->flags |= CONFIGFLAG_3A_EXPOSURE;

            LOG_DEBUG("parsed exposure (%s)",
                ((config_camera->flags & CONFIGFLAG_3A_EXPOSURE) ? "true" : "false"));
        } else
        /*manual pic quality*/
        if (strcmp(name, "manual-pq") == 0)
        {
            enable = (atoi(mxmlElementGetAttr(node, "enable")) > 0) ? TRUE : FALSE;
            if(enable && parse_3a_config_picture_quality (node, &config_camera->pq))
                config_camera->flags |= CONFIGFLAG_3A_PICQUALITY;

            LOG_DEBUG("parsed manual picture quality (%s)",
                ((config_camera->flags & CONFIGFLAG_3A_PICQUALITY) ? "true" : "false"));

        } else
        /* others */
        if (strcmp(name, "others") == 0)
        {
            enable = (atoi(mxmlElementGetAttr(node, "enable")) > 0) ? TRUE : FALSE;
            if(enable && parse_3a_config_others (node, &config_camera->others))
                config_camera->flags |= CONFIGFLAG_3A_OTHERS;

            LOG_DEBUG("parsed 3a others (%s)",
                ((config_camera->flags & CONFIGFLAG_3A_OTHERS) ? "true" : "false"));

        }

    }

Exit:
    fclose(fp);

    return result;
}

struct _RawFrameProbeData {
    MediaPipe *pipe;
    gulong     probe_id;
    GMutex     mutex;
    gchar      filename[128];
};

static GstPadProbeReturn
capture_raw_frame_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    struct _RawFrameProbeData *probedata = (struct _RawFrameProbeData *)user_data;
    MediaPipeImpl *impl = (MediaPipeImpl *)probedata->pipe;
    GstBuffer *buf = (GstBuffer *)(info->data);
    GstVideoPreprocBuffer *preBuf;
    GstElement *preproc = impl->src_preproc.element;

    g_mutex_lock (&probedata->mutex);

    preBuf = gst_video_preproc_buffer_get_from_gst_buffer(
                GST_VIDEO_PREPROC(preproc), buf);

    if(buf == NULL || !GST_IS_BUFFER(buf) || !preBuf)
    {
        LOG_WARNING ("capture raw frame failed, buffer is NULL");
        goto end;
    }
    gst_video_preproc_map_buffer(GST_VIDEO_PREPROC(preproc), preBuf);
    dump_nv12(preBuf, probedata->filename);
    gst_video_preproc_unmap_buffer(GST_VIDEO_PREPROC(preproc), preBuf);
    gst_video_preproc_destroy_buffer(
          GST_VIDEO_PREPROC(preproc), preBuf);

end:
    gst_pad_remove_probe(pad, probedata->probe_id);
    probedata->probe_id = 0;
    g_mutex_unlock (&probedata->mutex);
    return GST_PAD_PROBE_OK;
}

//free returned string after usage
static char *
capture_jpeg_frame (void)
{
   time_t timep;
   struct tm *cur_time;

   jpgenc_misc_config2.jpeg_capture_flag = 1;
   time(&timep);
   cur_time = localtime(&timep);
   snprintf (jpgenc_misc_config2.capture_filename,
         sizeof(jpgenc_misc_config2.capture_filename),
         "/etc/capture-%02d%02d%02d.jpg",
         cur_time->tm_hour, cur_time->tm_min, cur_time->tm_sec);
   return strdup(jpgenc_misc_config2.capture_filename);
}

//free the returned location after usage
static char *
capture_raw_frame (MediaPipe *pipe, int flag)
{
    MediaPipeImpl *impl = IMPL_CAST (pipe);
    static struct _RawFrameProbeData static_data;
    GstPad *pad = NULL;
    time_t timep;
    struct tm *cur_time;
    char *location = NULL;

    if (flag == 0) {
      pad = gst_element_get_static_pad (impl->src_preproc.element, "sink");
    } else {
      pad = gst_element_get_static_pad (impl->preproc.element, "src");
    }

    if (!pad) {
        LOG_WARNING ("capture failed, preproc not started");
        return NULL;
    }

    g_mutex_lock (&static_data.mutex);

    if (static_data.probe_id) {
       LOG_WARNING ("capture is already in running, try later");
       goto end;
    }

    time(&timep);
    cur_time = localtime(&timep);
    snprintf (static_data.filename, sizeof(static_data.filename),
          "/etc/%s-%02d%02d%02d.nv12", flag == 0 ? "raw" : "preprocessed",
          cur_time->tm_hour, cur_time->tm_min, cur_time->tm_sec);

    static_data.pipe = pipe;
    static_data.probe_id = gst_pad_add_probe (pad, GST_PAD_PROBE_TYPE_BUFFER,
            (GstPadProbeCallback) capture_raw_frame_probe_callback,
            &static_data, NULL);
    gst_object_unref(pad);

    location = strndup(static_data.filename, sizeof(static_data.filename));

end:
    g_mutex_unlock (&static_data.mutex);
    return location;
}

static void reset_hdr_table(guchar flag)
{
if(flag == 1)
    hdrtable_id++;
else if(flag == 0)
    hdrtable_id--;
else if(flag <= 16)
    hdrtable_id = (unsigned int)flag;
if(hdrtable_id > 14)
    hdrtable_id = 2;
if(hdrtable_id < 2)
    hdrtable_id = 14;
}

static void reset_hdr_rgb_table(guchar flag)
{
if(flag == 1)
    hdrtable_rgb_id++;
else if(flag == 0)
    hdrtable_rgb_id--;
else if(flag <= 16)
    hdrtable_rgb_id = (unsigned int)flag;
if(hdrtable_rgb_id > 16)
    hdrtable_rgb_id = 2;
if(hdrtable_rgb_id < 2)
    hdrtable_rgb_id = 16;
}

static gboolean
handle_keyboard (GIOChannel *source, GIOCondition cond, gpointer data)
{
   gchar *str = NULL;
   MediaPipe *pipe = (MediaPipe *)data;
   MediaPipeImpl *impl = IMPL_CAST (pipe);

   if (g_io_channel_read_line (source, &str, NULL, NULL, NULL) == G_IO_STATUS_NORMAL) {
      if (pipe->pipe_running != TRUE)
         goto out;
      gboolean set_luma = FALSE;
      gboolean reconfig_3a = FALSE;
      gboolean read_3a_setting = FALSE;
      if (str[0] == '+') {
         impl->preproc.luma_gain += 10;
         set_luma = TRUE;
      } else if (str[0] == '-') {
         impl->preproc.luma_gain -= 10;
         set_luma = TRUE;
      } else if (str[0] == '=') {
         impl->preproc.luma_gain = 100;
         set_luma = TRUE;
      }  else if (str[0] == 's') {
         reconfig_3a = TRUE;
      }  else if (str[0] == 'r') {
         read_3a_setting = TRUE;
      }  else if (str[0] == 'j') {
         reset_hdr_table(1);
      }  else if (str[0] == 'f') {
        reset_hdr_table(0);
      }  else if (str[0] == 'd') {
        reset_hdr_rgb_table(1);
      }  else if (str[0] == 'k') {
         reset_hdr_rgb_table(0);
      } else if (str[0] == 'c') {
         //0 : to capture 1080p raw data before any preprocessing
         g_free(capture_raw_frame (pipe, 0));
      } else if (str[0] == 'p') {
         //0 : to capture 1080p raw data after preprocessing but before 264 encoding
         g_free(capture_raw_frame (pipe, 1));
      } else if (str[0] == 'q') {
         //jpgenc_misc_config.jpeg_keyboard_flag = 1;
	     media_pipe_set_src_frame_rate(pipe, 10);
         printf("set frame rate to 10\n");
      } else if (str[0] == 'a') {
         dump_smart_analysis_raw_data = 1;
      } else if (str[0] == '3') {
	      media_pipe_set_v4l2_src_enable_3a(pipe, TRUE);
	      LOG_PRINT("resume 3A");
      } else if (str[0] == '#') {
	      media_pipe_set_v4l2_src_enable_3a(pipe, FALSE);
	      LOG_PRINT("pause 3A");
      } else if (str[0] == 'i') {
	      media_pipe_set_encoder_frame_rate(pipe, VIDEO_CHANNEL_1080P, 30);
	      LOG_PRINT("set encoder frame rate to 30fps for 1080p channel\n");
      } else if (str[0] == 'u') {
	      media_pipe_set_encoder_frame_rate(pipe, VIDEO_CHANNEL_1080P, 5);
	      LOG_PRINT("set encoder frame rate to 5fps for 1080p channel\n");
      } else {
         LOG_PRINT(" =========== mediapipe commands ==========");
         LOG_PRINT(" ===== '+' : increate luminace 10%%   =====");
         LOG_PRINT(" ===== '-' : decreate luminace 10%%   =====");
         LOG_PRINT(" ===== '=' : reset luma              =====");
         LOG_PRINT(" ===== 'c' : capture 1080p nv12 raw frame before preprocessing =====");
         LOG_PRINT(" ===== 'p' : capture 1080p nv12 raw frame after preprocessing but before encoding =====");
         LOG_PRINT(" ===== 's' : reconfig camera 3a      =====");
         LOG_PRINT(" ===== 'r' : read config of camera 3a=====");
         LOG_PRINT(" ===== 'h' : print this help text    =====");
         LOG_PRINT(" ===== 'f' : last hdr table    =====");
         LOG_PRINT(" ===== 'j' : next hdr table    =====");
         LOG_PRINT(" ===== 'd' : last hdr-rgb table    =====");
         LOG_PRINT(" ===== 'k' : next hdr-rgb table    =====");
         LOG_PRINT(" ===== 'q' : save jpeg    =====");
         LOG_PRINT(" ===== 'a' : dump smart analysis raw data =====");
         LOG_PRINT(" ===== '3' : enable 3A =====");
	 LOG_PRINT(" ===== '#' : disable 3A =====");
         LOG_PRINT(" ===== 'i' : set encoder frame rate to 30fps for 1080p channel =====");
         LOG_PRINT(" ===== 'u' : set encoder frame rate to 5fps for 1080p channel =====");
         LOG_PRINT(" =========================================");
      }
      if (set_luma == TRUE) {
         if(!gst_video_preproc_set_luma_gain(GST_VIDEO_PREPROC(impl->preproc.element),impl->preproc.luma_gain))
         {
            LOG_WARNING ("Set luma gain (%d) for preproc FAIL!", impl->preproc.luma_gain);
         }else {
            LOG_DEBUG ("Set luma gain (%d) for preproc SUCCEED!", impl->preproc.luma_gain);
         }
      }

      /*configure 3a by reading from configure file*/
      if (reconfig_3a == TRUE) {
         if(!media_pipe_reconfig_3a(pipe))
         {
            LOG_WARNING ("Reconfigure 3A for preproc FAIL!");
         }else {
            LOG_DEBUG ("Reconfigure 3A for preproc SUCCEED!");
         }
      }
      if(read_3a_setting == TRUE)
      {
        //TODO
      }

   }
out:
   g_free (str);
   return TRUE;
}

gboolean dump_nv12_timed(GstVideoPreprocBuffer *preBuf, char *prefix)
{
   time_t timep;
   struct tm *cur_time;
   char filename[128];

   time(&timep);
   cur_time = localtime(&timep);
   snprintf (filename, sizeof(filename),
         "/etc/%s-%02d%02d%02d.nv12",
         prefix, cur_time->tm_hour, cur_time->tm_min, cur_time->tm_sec);
   return dump_nv12(preBuf, filename);
}

gboolean dump_nv12(GstVideoPreprocBuffer *preBuf, char *filename)
{
   FILE *fp;
   guint i;
   guint8 *ptr;

   fp = fopen (filename, "wb");
   if (!fp) {
      LOG_WARNING ("create nv12 file(%s) failed", filename);
      return false;
   }

   ptr = (guint8*)preBuf->handle + preBuf->yoffset;
   for (i = 0; i < preBuf->h; ++i) {
      fwrite (ptr, 1, preBuf->w, fp);
      ptr += preBuf->ystride;
   }
   ptr = (guint8*)preBuf->handle + preBuf->uvoffset;
   for (i = 0; i < preBuf->h/2; ++i) {
      fwrite (ptr, 1, preBuf->w, fp);
      ptr += preBuf->uvstride;
   }
   fclose (fp);
   LOG_PRINT (" dump raw data ok, saved to %s", filename);
   return true;
}
