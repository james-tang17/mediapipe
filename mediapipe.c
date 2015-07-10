/*
 * mediapipe.c - Media pipe implementation
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

#include "mediapipe.h"
#include "mediapipe_impl.h"
#include "osd_template2.h"
#include "osddata.h"
#include <gst/vaapi/gstvaapismartmeta.h>
#include <time.h>
#include <stdlib.h>
#include <glib.h>
#include <string.h>
#include <unistd.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
#include <jpeglib_interface.h>


#define LOG_ERROR(format, ...)    \
    printf ("ERROR :" format "\n", ## __VA_ARGS__)

#define LOG_WARNING(format, ...)   \
    printf ("WARNING:" format "\n", ## __VA_ARGS__)

#ifndef DEBUG
#define LOG_DEBUG(format, ...)
#else
#define LOG_DEBUG(format, ...)   \
    printf ("DEBUG :" format "\n", ## __VA_ARGS__)
#endif


#define AVAILABLE_FRAME_RATE_1 25
#define AVAILABLE_FRAME_RATE_2 30
#define AVAILABLE_FRAME_RATE_3 60
#define MEDIA_VIDEO_DEFAULT_SENSOR_ID 0
#define MEDIA_VIDEO_DEFAULT_WIDTH  1920
#define MEDIA_VIDEO_DEFAULT_HEIGHT 1080
#define MEDIA_VIDEO_DEFAULT_FORMAT GST_VIDEO_FORMAT_NV12
#define MEDIA_VIDEO_DEFAULT_FPS_N 30
#define MEDIA_VIDEO_DEFAULT_FPS_D 1
#define MEDIA_VIDEO_DEFAULT_KEY_PERIOD 30


#define MEDIA_PIPE_TEST_SOURCE_NAME      "videotestsrc"
#define MEDIA_PIPE_V4L2_SOURCE_NAME      "xcamsrc"

#define MEDIA_PIPE_CAPSFILTER_NAME  "capsfilter"
#define MEDIA_PIPE_POSTPROC_NAME    "vaapipostproc"
#define MEDIA_PIPE_PREPROC_NAME     "vaapipreproc"
#define MEDIA_PIPE_ENCODER_NAME     "vaapiencode_h264"
#define MEDIA_PIPE_FILE_SINK_NAME        "filesink"
#define MEDIA_PIPE_FAKE_SINK_NAME        "fakesink"
#define MEDIA_PIPE_TCP_SINK_NAME        "tcpclientsink"
#define MEDIA_PIPE_KMS_SINK_NAME        "vaapisink"
#define MEDIA_PIPE_APP_SRC_NAME     "appsrc"
#define MEDIA_PIPE_VIDEORATE_NAME  "videorate"


#define QUEUE_PLUGIN_NAME     "queue"
#define MP4MUX_PLUGIN_NAME     "mp4mux"
#define TEE_PLUGIN_NAME     "tee"

static gboolean local_preview = FALSE;
static gboolean jpeg_inited = FALSE;
extern gfloat   smart_factor;
extern int      global_v4l2src_color_effect;
extern int      dump_smart_analysis_raw_data;
extern gboolean smart_control;
extern gboolean enable_qos;
extern gboolean enable_lumagain_threshold;
extern guint lumagain_threshold_value;
extern gboolean enable_hdr;
extern GstVideoPreprocAutoHDRMode auto_hdr_mode;
extern gboolean enable_hdr_custom;
extern gboolean enable_hdr_custom_rgb;
extern unsigned int hdrtable_id;
extern unsigned int hdrtable_rgb_id;
extern unsigned warning_level;
extern gboolean enable_facedetect;
extern gboolean facedetect_conf;
extern gboolean global_enable_osd;
extern gboolean global_enable_mask;
extern GstVideoPreprocFlipMode global_flip_mode;
extern gboolean global_enable_wireframe;
extern gfloat rot_00;
extern gfloat rot_01;
extern gfloat rot_10;
extern gfloat rot_11;
extern gint dvs_offset_x;
extern gint dvs_offset_y;
GstVideoPreprocWireFrame cv_wire_frames[WIRE_FRAME_REGION_MAX_NUM] = {
      {{4,   2,    16,   18},  {88, 122, 33},  6, FALSE },
      {{500, 260,  210,  190}, {188,122, 133}, 2, FALSE },
      {{250, 200,  800,  500}, {56, 78,  93},  2, FALSE },
      {{100, 160,  200,  100}, {18, 12,  213}, 4, FALSE },
      {{320, 100,  60,   70},  {96, 98,  193}, 2, FALSE },
      {{10,  20,   190,  280}, {122,244, 55},  2, FALSE },
};

GstVideoPreprocWireFrame sample_wire_frames[WIRE_FRAME_REGION_MAX_NUM] = {
      {{500, 260,  210,  190}, {0, 0,  255}, 6, TRUE },
      {{4,   2,    16,   18},  {88, 122, 33},  6, FALSE },
      {{250, 200,  800,  500}, {56, 78,  93},  6, FALSE },
      {{100, 160,  200,  100}, {18, 12,  213}, 4, FALSE },
      {{320, 100,  60,   70},  {96, 98,  193}, 2, FALSE },
      {{10,  20,   190,  280}, {122,244, 55},  2, FALSE },
};

GstVideoPreprocMaskCfg sample_masks[MASK_REGION_MAX_NUM] = {
    {{0,   0,    100,  100}, {255,0,0}, TRUE,  GST_VIDEO_PREPROC_MASK_TYPE_YUV},
    {{100, 100,  200,  200}, {0,255,0}, TRUE,  GST_VIDEO_PREPROC_MASK_TYPE_YUV},
    {{300, 300,  416,  416}, {0,},      TRUE,  GST_VIDEO_PREPROC_MASK_TYPE_BLUR},
    {{800, 800,  200,  200}, {0,},      TRUE, GST_VIDEO_PREPROC_MASK_TYPE_MOSAIC},
    {{100, 300,  128,  16},  {0,},  FALSE, GST_VIDEO_PREPROC_MASK_TYPE_BLUR},
    {{500, 1000, 1000, 32},  {0,},  FALSE, GST_VIDEO_PREPROC_MASK_TYPE_BLUR},
};

const gchar *profile_name[3] =
{
    "baseline",
    "main",
    "high"
};

typedef struct _StringContainer
{
    gchar               pad_name[128];
    gchar               file_name[128];
}StringContainer;

typedef struct _ResolutionPack
{
    guint               width;
    guint               height;
}ResolutionPack;

const gchar *channel_name[VIDEO_CHANNEL_MAX] = {
    "1080p",
    "jpeg",
    "720p",
    "d1",
    "cif",
    "480_270",
    "352_198",
    "480_272",
    "352_200",
    "smart"
};

const StringContainer tempString[VIDEO_CHANNEL_MAX] =
{
    {"src",                 "1080p.264"},
    {"src_jpeg",            "jpeg"},
    {"src_720p",            "720p.264"},
    {"src_d1",              "d1.264"},
    {"src_cif",             "cif.264"},
    {"src_480_270",         "480x270.264"},
    {"src_352_198",         "352x198.264"},
    {"src_480_272",         "480x272.264"},
    {"src_352_200",         "352x200.264"},
    {"src_smart",           "smart.264"},
};

ResolutionPack normal_resolution[VIDEO_CHANNEL_MAX] =
{
    {1920,  1080},
    {1920,  1080},
    {1280,  720},
    {704,   576},
    {352,   288},
    {480,   270},
    {352,   198},
    {480,   272},
    {352,   200},
    {0,     0},
};

ResolutionPack rotate_resolution[VIDEO_CHANNEL_MAX] =
{
    {1080,  1920},
    {1080,  1920},
    {720,   1280},
    {576,   704},
    {288,   352},
    {270,   480},
    {198,   352},
    {272,   480},
    {200,   352},
    {0,     0},
};

static GList *prepare_osd_infos(GstElement *elm/* used to create vaapi image */);
static void _smart_meta_osd_info_free(GstVideoPreprocOsdInfo *info);
static void _smart_meta_wf_info_free(GstVideoPreprocWireFrameInfo *info);
static void _smart_meta_mask_info_free(GstVideoPreprocMaskInfo *info);

static void
prepare_dvs_info(gpointer q, gpointer dvs_info, GstBuffer *buf, gpointer data)
{
   //should align the dvs info to the right frame according to buffer timestamp 
   GST_DEBUG("buffer = %p, ts= %" GST_TIME_FORMAT "\n", buf, GST_TIME_ARGS(GST_BUFFER_TIMESTAMP(buf)));

   GstVaapiDvsInfo *info = (GstVaapiDvsInfo *)dvs_info;
   info->need_dvs = FALSE;
   info->rot_00 = rot_00;
   info->rot_01 = rot_01;
   info->rot_10 = rot_10;
   info->rot_11 = rot_11;
   info->offset_x = dvs_offset_x;
   info->offset_y = dvs_offset_y;
   return;
}

/* ---------------------tool functions---------------------------------*/

/************** RTSP server support *******************/
static GstPadProbeReturn
main_rtsp_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    DynamicChannelData    *dynamic_data = (DynamicChannelData *)user_data;
    MediaPipeImpl         *impl = IMPL_CAST (dynamic_data->media_pipe);
    VideoChannelIndex     channel_num = dynamic_data->channel_num;
    VideoChannel          *channel = &impl->channel[channel_num];
    GstBuffer *buf;
    GstFlowReturn ret;

    buf = (GstBuffer *)(info->data);
    if(buf == NULL || !GST_IS_BUFFER(buf))
    {
        return GST_PAD_PROBE_OK;
    }

    GST_BUFFER_PTS (buf) = channel->sink.timestamp;
    GST_BUFFER_DURATION (buf) =
	    gst_util_uint64_scale_int (1, GST_SECOND, channel->videorate.fps_n);
    channel->sink.timestamp += GST_BUFFER_DURATION (buf);

    g_signal_emit_by_name (channel->sink.rtsp_src, "push-buffer", buf, &ret);
    if (ret != GST_FLOW_OK) {
        LOG_ERROR ("failed to push buffer to rtsp server!\n");
        gst_pad_remove_probe(pad, info->id);
    }

    return GST_PAD_PROBE_OK;
}

static void
media_configure (GstRTSPMediaFactory * factory, GstRTSPMedia * media,
		    gpointer user_data)
{
    GstElement            *pipeline, *appsrc;
    GstElement            *srcqueue, *codecqueue;
    DynamicChannelData    *dynamic_data = (DynamicChannelData *)user_data;
    MediaPipeImpl         *impl = IMPL_CAST (dynamic_data->media_pipe);
    VideoChannelIndex     channel_num = dynamic_data->channel_num;
    VideoChannel          *channel = &impl->channel[channel_num];
    GstCaps               *caps = NULL;
    GstPad                *pad = NULL;


    LOG_DEBUG ("RTSP client connected. \n");

    /* get the element used for providing the streams of the media */
    pipeline = gst_rtsp_media_get_element (media);
    appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "src");
    channel->sink.rtsp_src = appsrc;

    /* appsrc */
    if(impl->preproc.rotate_mode == GST_VIDEO_PREPROC_ROTATE_NONE ||
       impl->preproc.rotate_mode == GST_VIDEO_PREPROC_ROTATE_180)
    {
        caps =
            gst_caps_new_simple ("video/x-raw",
                "format", G_TYPE_STRING, gst_video_format_to_string(impl->main_source.format),
                "width", G_TYPE_INT, normal_resolution[channel_num].width,
                "height", G_TYPE_INT, normal_resolution[channel_num].height,
                "framerate", GST_TYPE_FRACTION, channel->videorate.fps_n, channel->videorate.fps_d,
                "interlace-mode", G_TYPE_STRING, "progressive",
                NULL);
    }else
    {
        caps =
            gst_caps_new_simple ("video/x-raw",
                "format", G_TYPE_STRING, gst_video_format_to_string(impl->main_source.format),
                "width", G_TYPE_INT, rotate_resolution[channel_num].width,
                "height", G_TYPE_INT, rotate_resolution[channel_num].height,
                "framerate", GST_TYPE_FRACTION, channel->videorate.fps_n, channel->videorate.fps_d,
                "interlace-mode", G_TYPE_STRING, "progressive",
                NULL);
    }

    g_object_set (G_OBJECT (appsrc), "caps", caps, NULL);
    gst_caps_unref (caps);
    g_object_set (G_OBJECT (appsrc),
	         "stream-type", 0,
	         "format", GST_FORMAT_TIME, NULL);

    /*srcqueue*/
    srcqueue = gst_bin_get_by_name(GST_BIN(pipeline), "srcqueue");
    g_object_set (G_OBJECT (srcqueue), "max-size-buffers", 200, "max-size-time",
			                       (guint64) 0, "max-size-bytes", 0, "leaky", 1, NULL);	

    /* vaapiencode_h264 */
    channel->encoder.element = gst_bin_get_by_name(GST_BIN(pipeline), "rtsp_enc");
    g_object_set (channel->encoder.element,
                      "rate-control", channel->encoder.rate_control,
                      "bitrate", channel->encoder.bitrate,
                      "cabac", channel->encoder.enable_cabac,
                      "enable-mv", channel->encoder.mv,
                      "keyframe-period", channel->encoder.gop_M,
                      "max-bframes", channel->encoder.gop_N,
                      "dct8x8", channel->encoder.enable_dct8x8,
                      NULL);

    /* capsfilter */
    channel->profile_converter = gst_bin_get_by_name(GST_BIN(pipeline), "rtsp_profile");
    caps = gst_caps_new_simple ("video/x-h264",
					"profile", G_TYPE_STRING, profile_name[channel->encoder.profile],
					NULL);
    g_object_set (channel->profile_converter, "caps", caps, NULL);
    gst_caps_unref (caps);

    /*codecqueue*/
    codecqueue = gst_bin_get_by_name(GST_BIN(pipeline), "codecqueue");
    g_object_set (G_OBJECT (codecqueue), "max-size-buffers", 200, "max-size-time",
		                   (guint64) 0, "max-size-bytes", 0, "leaky", 1, NULL);	

    /* reset the timestamp */
    channel->sink.timestamp = 0;

    /* the sink need to be fakesink */
    pad = gst_element_get_static_pad (channel->sink.element, "sink");
    gst_pad_add_probe (
             pad,
             GST_PAD_PROBE_TYPE_BUFFER,
             (GstPadProbeCallback) main_rtsp_probe_callback,
             dynamic_data,
             NULL);
    gst_object_unref (pad);
}

static gint
rtsp_server_create (MediaPipeImpl *impl, VideoChannelIndex channel_num)
{
    VideoChannel        *channel = &impl->channel[channel_num];
    GstRTSPServer       *server = NULL;
    GstRTSPMountPoints  *mounts;
    GstRTSPMediaFactory *factory;
    gchar               *mountpoint_name;


    if (!impl->rtsp_server) {
        impl->rtsp_server = gst_rtsp_server_new();
        if (!impl->rtsp_server) {
       	    LOG_ERROR ("Create RTSP server error!\n");  
            return -1;
        }
    }

    server = impl->rtsp_server;
    channel->sink.server = impl->rtsp_server;
    factory = gst_rtsp_media_factory_new ();
    channel->sink.factory = factory;
    gst_rtsp_media_factory_set_launch (factory,
    	"appsrc name=src ! queue name=srcqueue ! vaapiencode_h264 name=rtsp_enc ! capsfilter name=rtsp_profile ! queue name=codecqueue ! rtph264pay name=pay0 pt=96");

#if 1
    /* support multiple client connections */
    gst_rtsp_media_factory_set_shared (factory, TRUE);
#endif

    /*
     * notify when our media is ready, This is called whenever someone asks for
     * the media and a new pipeline is created
     */
    DynamicChannelData *dynamic_data =
                        (DynamicChannelData *)g_new0 (DynamicChannelData, 1);
    dynamic_data->media_pipe = (MediaPipe *)impl;
    dynamic_data->channel_num = channel_num;
    g_signal_connect (factory, "media-configure", (GCallback) media_configure,
                       dynamic_data);

    mountpoint_name = g_strdup_printf ("/%s", channel_name[channel_num]);
    LOG_DEBUG ("rtsp://127.0.0.1%s is ready\n", mountpoint_name);
    mounts = gst_rtsp_server_get_mount_points (server);
    gst_rtsp_mount_points_add_factory (mounts,
                   mountpoint_name,
                   factory);
    g_free (mountpoint_name);
    g_object_unref (mounts);

    channel->sink.source_id = gst_rtsp_server_attach(server, NULL);

    return 0;
}
/************** RTSP server support *******************/

static void
free_post_link_info (PostLinkInfo *info)
{
    g_slice_free (PostLinkInfo,info);
}

static void
post_link(GstElement *src, GstPad *pad, gpointer data)
{
    PostLinkInfo *info = (PostLinkInfo*)data;
    g_assert (src == info->src);

    if (gst_element_link_pads (src, NULL, info->sink, NULL)) {
        g_signal_handler_disconnect (src, info->signal);
        LOG_DEBUG ("Post link %s and %s", GST_ELEMENT_NAME(info->src), GST_ELEMENT_NAME(info->sink));
        return;
    }

    LOG_ERROR ("Post link %s and %s failed",
               GST_ELEMENT_NAME(info->src),
               GST_ELEMENT_NAME(info->sink));
}

static GstElement *
create_filesrc_bin (gchar *location)
{
   GstPad *pad, *gpad;
   GstElement *bin, *filesrc, *h264parse, *queue, *h264dec, *postproc;

   bin = gst_bin_new("filesrc-bin");
   filesrc = gst_element_factory_make("filesrc", NULL);
   h264parse = gst_element_factory_make("h264parse", NULL);
   queue = gst_element_factory_make("queue", "filesrc-bin-queue");
   h264dec = gst_element_factory_make("avdec_h264", NULL);
   postproc = gst_element_factory_make("vaapipostproc", NULL);

   if (!bin || !filesrc || !h264parse || !queue || !h264dec || !postproc) {
        LOG_ERROR ("some element in filesrc bin failed");
        return NULL;
   }

   g_object_set (filesrc, "location", location, NULL);
   g_object_set (postproc, "format", 23, NULL);

   gst_bin_add_many(GST_BIN(bin), filesrc, h264parse, queue, h264dec, postproc, NULL);
   gst_element_link_many (filesrc, h264parse, queue, h264dec, postproc, NULL);
   pad = gst_element_get_static_pad(postproc, "src");
   gpad = gst_ghost_pad_new("src", pad);
   gst_object_unref(pad);
   gst_element_add_pad(bin, gpad);
   return bin;
}



static GstElement *
create_element (const char *plugin, const char *name)
{
    GstElement *element;

    g_assert (plugin);

    element = gst_element_factory_make(plugin, name);
    if (!element) {
        LOG_ERROR ("create element:%s failed", plugin);
        return NULL;
    }
    LOG_DEBUG ("plugin(%s/%s) created.", plugin, (name ? name : "null"));
    return element;
}


static gboolean
link_element (GstElement *src, GstElement *sink)
{
    PostLinkInfo *info;
    g_return_val_if_fail (src && sink,  FALSE);

    if (gst_element_link_pads (src, NULL, sink, NULL)) {
        LOG_DEBUG ("Directly linked %s and %s", GST_ELEMENT_NAME(src), GST_ELEMENT_NAME(sink));
        return TRUE;
    }

    info = g_slice_new0 (PostLinkInfo);
    info->src = src;
    info->sink = sink;
    info->signal = g_signal_connect_data (src, "pad-added",
          G_CALLBACK (post_link), info, (GClosureNotify) free_post_link_info, (GConnectFlags)0);
    return TRUE;
}

/* ------------------------src pipeline functions------------------------- */
static gboolean
src_message_loop (GstBus *bus, GstMessage * message, gpointer user_data)
{
     MediaPipeImpl *impl = IMPL_CAST (user_data);
     gboolean ret = TRUE;

     switch (GST_MESSAGE_TYPE (message))
     {
        case GST_MESSAGE_EOS:
        {
            gst_app_src_end_of_stream(GST_APP_SRC(impl->main_source.app_src));

            LOG_DEBUG ("src pipeline: End of stream\n");
            g_main_loop_quit (impl->main_loop);
            break;
        }
        case GST_MESSAGE_ERROR:
        {
            gchar  *debug;
            GError *error;

            gst_message_parse_error (message, &error, &debug);
            g_free (debug);

            LOG_ERROR ("src pipeline: Error: %s\n", error->message);
            g_error_free (error);

            g_main_loop_quit (impl->main_loop);
            break;
        }
        default:
            break;
   }

    if (impl->src_msg_callback)
        ret = impl->src_msg_callback (message, impl->src_msg_user_data);

    return ret;
}

static GstPadProbeReturn
src_preproc_1080p_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    MediaPipeImpl *impl = (MediaPipeImpl *)user_data;
    GstBuffer *buf = (GstBuffer *)(info->data);

    if(buf == NULL || !GST_IS_BUFFER(buf))
    {
        return GST_PAD_PROBE_OK;
    }

    if (smart_control) {
       // need to be freed after buffer being pushed
       Smart1080pData *frame_data = g_new0(Smart1080pData, 1);
       frame_data->buf = gst_buffer_ref(buf);
       frame_data->can_push = FALSE;

       g_mutex_lock(&impl->src_preproc.smart_lock);
       impl->src_preproc.smart_1080p_queue =
          g_list_append(impl->src_preproc.smart_1080p_queue, frame_data);
       g_mutex_unlock(&impl->src_preproc.smart_lock);
    } else {
       /* smart meta sample code start */
       GList *osd_infos = NULL;
       GList *wf_infos = NULL;
       GList *mask_infos = NULL;
       GstVaapiDvsInfo dvs_info;
       gboolean need_jpeg_encode = TRUE;
       gint j;
       static guint cnt = 0;

       /* OSD code start */
       if (global_enable_osd) {
          osd_infos = prepare_osd_infos(impl->preproc.element);
       }
       /* OSD code stop */

       /* wireframe code start */
       if (global_enable_wireframe) {
          GstVideoPreprocWireFrame *wf;

          if(enable_facedetect) {
             if (!smart_control)
                g_mutex_lock(&impl->preproc.smart_lock);
             wf = cv_wire_frames;
          } else {
             wf = sample_wire_frames;
          }
          GstVideoPreprocWireFrameInfo *wf_info;
          for (j = 0; j < WIRE_FRAME_REGION_MAX_NUM; j++) {
             wf_info = (GstVideoPreprocWireFrameInfo *)g_malloc0(sizeof(*wf_info));
             wf_info->wireframe = wf[j];
             wf_info->id        = j;
             wf_info->channel = GST_VIDEO_PREPROC_CHANNEL_1080P;
             wf_infos = g_list_append(wf_infos, wf_info);
          }
          if (!smart_control) {
             g_mutex_unlock(&impl->preproc.smart_lock);
          }

       }
       /* wireframe code stop */

       /* mask code start */
       if (global_enable_mask) {
          GstVideoPreprocMaskCfg *masks;
          masks = sample_masks;
          for (j = 0; j < MASK_REGION_MAX_NUM; j++) {
             GstVideoPreprocMaskInfo *mask_info;
             mask_info = (GstVideoPreprocMaskInfo *)g_malloc0(sizeof(*mask_info));
             mask_info->cfg = masks[j];
             mask_info->id = j;
             mask_info->channel = GST_VIDEO_PREPROC_CHANNEL_1080P;
             mask_infos = g_list_append(mask_infos, mask_info);
          }
       }
       /* mask code stop */

       dvs_info.offset_x = dvs_offset_x;
       dvs_info.offset_y = dvs_offset_y;
       if (osd_infos || wf_infos || mask_infos || dvs_info.offset_x != 0 || dvs_info.offset_y != 0) {
          buf = gst_buffer_make_writable(buf);
          //this function will create an GstVaapiOsdMeta object to hold osd_infos, and bind this object to GstBuffer
          gst_buffer_add_vaapi_smart_meta(buf, osd_infos, wf_infos, mask_infos, need_jpeg_encode, &dvs_info);

          GstVaapiSmartMeta *smart_meta = gst_buffer_get_vaapi_smart_meta(buf);
          GST_DEBUG("mediapipe, cnt=%u, smart_meta=%p, osd_infos_len=%u, wf_infos_lens=%u, mask_infos_lens=%u, need_jpeg_encode=%d, gst_buffer = %p, ts= %" GST_TIME_FORMAT "\n",
                cnt,
                smart_meta,
                g_list_length(smart_meta->osd_infos),
                g_list_length(smart_meta->wf_infos),
                g_list_length(smart_meta->mask_infos),
                smart_meta->need_jpeg_encode,
                buf,
                GST_TIME_ARGS(GST_BUFFER_TIMESTAMP(buf)));
       }

       if (osd_infos) {
          g_list_free_full (osd_infos, (GDestroyNotify)_smart_meta_osd_info_free);
       }
       if (wf_infos) {
          g_list_free_full (wf_infos, (GDestroyNotify)_smart_meta_wf_info_free);
       }
       if (mask_infos) {
          g_list_free_full (mask_infos, (GDestroyNotify)_smart_meta_mask_info_free);
       }
       cnt++;
       /* smart meta sample code stop */

       gst_app_src_push_buffer(GST_APP_SRC(impl->main_source.app_src), gst_buffer_ref(buf));
    }

    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
src_preproc_luma_gain_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    MediaPipeImpl *impl = (MediaPipeImpl *)user_data;
    GstBuffer *buf = (GstBuffer *)(info->data);
    GstVideoPreprocBuffer *preBuf;
    guint i,j;
    guint sum=0;
    unsigned char *p;
    guint luma_gain=0;
    if(buf == NULL || !GST_IS_BUFFER(buf))
    {
        return GST_PAD_PROBE_OK;
    }
    preBuf = gst_video_preproc_buffer_get_from_gst_buffer(
                GST_VIDEO_PREPROC(impl->preproc.element),
                buf);
    gst_video_preproc_map_buffer(GST_VIDEO_PREPROC(impl->preproc.element), preBuf);
    p = (unsigned char*)preBuf->handle;
    for(i=0;i<preBuf->h;i++)
        {
            for(j=0;j<preBuf->w;j++)
                {
                    sum += (*p);
                    p++;
            }
            p += (preBuf->ystride - preBuf->w);
        }
    sum = sum/(preBuf->w*preBuf->h);
    luma_gain = 100.0*lumagain_threshold_value/sum;
    //printf("-------------- average = %d  gain=%d\n----------",sum,luma_gain);
    if(!gst_video_preproc_set_luma_gain(GST_VIDEO_PREPROC(impl->preproc.element),luma_gain))
    {
         LOG_WARNING ("Set luma gain for preproc FAIL!");
    }

    gst_video_preproc_unmap_buffer(GST_VIDEO_PREPROC(impl->preproc.element), preBuf);
    gst_video_preproc_destroy_buffer(
          GST_VIDEO_PREPROC(impl->preproc.element),
          preBuf);
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
src_preproc_hdr_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    MediaPipeImpl *impl = (MediaPipeImpl *)user_data;
    GstBuffer *buf = (GstBuffer *)(info->data);
    if(buf == NULL || !GST_IS_BUFFER(buf))
    {
        return GST_PAD_PROBE_OK;
    }
if(enable_hdr)
    {
        if(!gst_video_preproc_set_autohdr(GST_VIDEO_PREPROC(impl->preproc.element),auto_hdr_mode))
          {
              LOG_WARNING ("Set hdr for preproc FAIL!");
         }
        if(!gst_video_preproc_disable_auto_hdr_custom(GST_VIDEO_PREPROC(impl->preproc.element)))
          {
              LOG_WARNING ("disable hdr custom for preproc FAIL!");
         }
        if(!gst_video_preproc_disable_auto_hdr_custom_rgb(GST_VIDEO_PREPROC(impl->preproc.element)))
          {
              LOG_WARNING ("disable hdr custom rgb for preproc FAIL!");
         }
    }
else if(enable_hdr_custom)
    {
        if(!gst_video_preproc_set_customhdr(GST_VIDEO_PREPROC(impl->preproc.element), hdrtable_id, 1024))
        {
             LOG_WARNING ("Set custom hdr for preproc FAIL!");
        }
        if(!gst_video_preproc_disable_autohdr(GST_VIDEO_PREPROC(impl->preproc.element)))
          {
              LOG_WARNING ("disable hdr custom for preproc FAIL!");
         }
        if(!gst_video_preproc_disable_auto_hdr_custom_rgb(GST_VIDEO_PREPROC(impl->preproc.element)))
          {
              LOG_WARNING ("disable hdr custom rgb for preproc FAIL!");
         }
    }
else if(enable_hdr_custom_rgb)
    {
        if(!gst_video_preproc_set_customhdr_rgb(GST_VIDEO_PREPROC(impl->preproc.element), hdrtable_rgb_id, 1024))
        {
             LOG_WARNING ("Set custom hdr for preproc FAIL!");
        }
        if(!gst_video_preproc_disable_auto_hdr_custom(GST_VIDEO_PREPROC(impl->preproc.element)))
          {
              LOG_WARNING ("disable hdr for preproc FAIL!");
         }
        if(!gst_video_preproc_disable_autohdr(GST_VIDEO_PREPROC(impl->preproc.element)))
          {
              LOG_WARNING ("disable hdr custom rgb for preproc FAIL!");
         }
    }
else
    {
    if(!gst_video_preproc_disable_autohdr(GST_VIDEO_PREPROC(impl->preproc.element)))
    {
         LOG_WARNING ("diaable hdr for preproc FAIL!");
    }
    if(!gst_video_preproc_disable_auto_hdr_custom(GST_VIDEO_PREPROC(impl->preproc.element)))
      {
          LOG_WARNING ("disable hdr custom for preproc FAIL!");
     }
    if(!gst_video_preproc_disable_auto_hdr_custom_rgb(GST_VIDEO_PREPROC(impl->preproc.element)))
      {
          LOG_WARNING ("disable hdr custom rgb for preproc FAIL!");
     }
    }
return GST_PAD_PROBE_OK;
}

/*
 *OSD basic idea
 * The app uses GstVaapiOsdMeta to hold all the osd infos of a frame. And bind this meta to GstBuffer.
 * And push GstBuffer downstream. The GstVaapiPreproc plugin will retrive and parse the meta and apply
 * the osd accordingly.
 *
 * This sample code demos 3 typical use case of osd:
 * case 1: initialise 2 osd regions which display current system time with hh:mm:ss style on
 *         1080p channel, and an chinese words on D1 channel respectively.
 * case 2: update partially the time osd with current system time.
 * case 3: replace time time osd with another one which use different font.
 *
 * All the pixel data are stored in osddata.h
 */


#define OSD_CHN_WIDTH 128
#define OSD_CHN_HEIGHT 64
#define CHANNEL_1080P_OSD_0 0
#define CHANNEL_D1_OSD_0 0
#define BIG_FONT_WIDTH 64
#define BIG_FONT_HEIGHT 64
#define SMALL_FONT_WIDTH 32
#define SMALL_FONT_HEIGHT 32
#define ELM_NUM 11
#define COLON_DATA_INDEX (ELM_NUM - 1)
// NV12 image has 2 plane
#define PLANE_NUM 2
#define OSD_DATA_WIDTH 1024
#define OSD_DATA_HEIGHT 1024
#define SAMPLE_OSD_DATA_STRIDE (OSD_DATA_WIDTH)
#define UV_PLANE_OFFSET (OSD_DATA_WIDTH*OSD_DATA_HEIGHT)
#define OSD_ENG  0
#define OSD_CHN  1
#define OSD_TIME 2

// the time style is hh:mm:ss
typedef enum {
    TIME_SLOT_HH_0 = 0,
    TIME_SLOT_HH_1,
    TIME_SLOT_MM_0,
    TIME_SLOT_MM_1,
    TIME_SLOT_SS_0,
    TIME_SLOT_SS_1,
    TIME_SLOT_NUM
}time_slot_e;

GstVideoPreprocOsdCfg osd_cfg[OSD_REGION_MAX_NUM] = {
    {{0,   0,   128,  64}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{0,   0,   128,  64}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{0,   0,   128,  64}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{32,  190, 64,    32}, 0, {COLOR_KEY_Y, 0, 0}, FALSE},
    {{1100,310, 128,   128},0, {COLOR_KEY_Y, 0, 0}, FALSE},
    {{552, 1004,1000,  32}, 0, {COLOR_KEY_Y, 0, 0}, FALSE},
};

GstVideoPreprocOsdCfg update_osd_cfg_big[TIME_SLOT_NUM] = {
    {{0,                    0,   BIG_FONT_WIDTH, BIG_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{1 * BIG_FONT_WIDTH,   0,   BIG_FONT_WIDTH, BIG_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{3 * BIG_FONT_WIDTH,   0,   BIG_FONT_WIDTH, BIG_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{4 * BIG_FONT_WIDTH,   0,   BIG_FONT_WIDTH, BIG_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{6 * BIG_FONT_WIDTH,   0,   BIG_FONT_WIDTH, BIG_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{7 * BIG_FONT_WIDTH,   0,   BIG_FONT_WIDTH, BIG_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
};

GstVideoPreprocOsdCfg update_osd_cfg_small[TIME_SLOT_NUM] = {
    {{0,                      0,   SMALL_FONT_WIDTH,  SMALL_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{1 * SMALL_FONT_WIDTH,   0,   SMALL_FONT_WIDTH,  SMALL_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{3 * SMALL_FONT_WIDTH,   0,   SMALL_FONT_WIDTH,  SMALL_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{4 * SMALL_FONT_WIDTH,   0,   SMALL_FONT_WIDTH,  SMALL_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{6 * SMALL_FONT_WIDTH,   0,   SMALL_FONT_WIDTH,  SMALL_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
    {{7 * SMALL_FONT_WIDTH,   0,   SMALL_FONT_WIDTH,  SMALL_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE},
};


GstVideoPreprocOsdCfg initial_osd_cfg = {{16,   16,   BIG_FONT_WIDTH * 8,  BIG_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE};
GstVideoPreprocOsdCfg initial_osd_cfg_small = {{16,   16,   SMALL_FONT_WIDTH * 8,  SMALL_FONT_HEIGHT}, 0, {COLOR_KEY_Y, 0, 0}, TRUE};
GstVideoPreprocOsdCfg osd_cfg_part    = {{64,   0,    64,  64}, 0, {COLOR_KEY_Y, 0, 0}, TRUE};

static gboolean image_copy(guchar *dst, guint dst_stride, guchar *src, guint w, guint h, guint src_stride)
{
   guint i;
   for (i = 0; i < h; i++) {
      memcpy(dst, src, w);
      dst += dst_stride;
      src += src_stride;
   }
   return TRUE;
}

static GList *
add_osd_update(GList *osd_infos, guint id, GstVideoPreprocOsdCfg *cfg,
    GstVideoPreprocImageRaw *raw, GstVideoPreproc *preproc)
{
    GstVideoPreprocOsdInfo *osd_info;
    GstVaapiImage *osd_image;

    osd_info          = (GstVideoPreprocOsdInfo *)g_malloc0(sizeof(GstVideoPreprocOsdInfo));
    osd_info->id      = id;
    osd_info->type    = GST_VIDEO_PREPROC_OSD_TYPE_UPDATE;//update osd partially, use GST_VIDEO_PREPROC_OSD_TYPE_UPDATE
    osd_info->channel = GST_VIDEO_PREPROC_CHANNEL_1080P;
    osd_info->cfg     = *cfg;
    osd_image         = gst_video_preproc_create_image(preproc, raw);
    osd_info->image   = osd_image;

    return g_list_append(osd_infos, osd_info);
}

static void
get_current_time(struct tm** p)
{
    time_t timep;

    time(&timep);
   *p = localtime(&timep);
}

static void
convert_time_slot(struct tm* curret_time, guint *time_slot_tmp)
{
    time_slot_tmp[0] = curret_time->tm_hour / 10;
    time_slot_tmp[1] = curret_time->tm_hour % 10;
    time_slot_tmp[2] = curret_time->tm_min / 10;
    time_slot_tmp[3] = curret_time->tm_min % 10;
    time_slot_tmp[4] = curret_time->tm_sec / 10;
    time_slot_tmp[5] = curret_time->tm_sec % 10;
}

static GList *
prepare_osd_infos(GstElement *elm/* used to create vaapi image */)
{

   GstVaapiImage *osd_image;
   GList *osd_infos = NULL;
   static guint cnt = 0;
   gint i;
   GstVideoPreprocOsdInfo *osd_info;

   static guint time_slot_cur[TIME_SLOT_NUM];
   static gboolean osd_data_inited = FALSE;
   static guchar *big_elm[ELM_NUM][PLANE_NUM];
   static guchar *small_elm[ELM_NUM][PLANE_NUM];
   if (osd_data_inited == FALSE) {
      /*
       * we have 2 set of pixel data for "0"--"9" and ":" in osddata.h
       * the big set is 64x64 pixel for each character, and the small
       * set is 32x32 pixel for each.
       */

      //get the offset of 0-9 characters
      //big set
      guint origin_offset_y = 64 * SAMPLE_OSD_DATA_STRIDE;
      guint origin_offset_uv = UV_PLANE_OFFSET + 32 * SAMPLE_OSD_DATA_STRIDE;
      for (i = 0; i < ELM_NUM; i++) {
         big_elm[i][0] = (guchar *)osddata + origin_offset_y + i * BIG_FONT_WIDTH;
         big_elm[i][1] = (guchar *)osddata + origin_offset_uv + i * BIG_FONT_WIDTH;
      }

      //small set
      origin_offset_y  = 128 * SAMPLE_OSD_DATA_STRIDE;
      origin_offset_uv = UV_PLANE_OFFSET + 64 * SAMPLE_OSD_DATA_STRIDE;
      for (i = 0; i < ELM_NUM; i++) {
         small_elm[i][0] = (guchar *)osddata + origin_offset_y + i * SMALL_FONT_WIDTH;
         small_elm[i][1] = (guchar *)osddata + origin_offset_uv + i * SMALL_FONT_WIDTH;
      }
      osd_data_inited = TRUE;
   }

   struct tm *p = NULL;
   guint time_slot_tmp[TIME_SLOT_NUM];
   GstVideoPreprocImageRaw raw;

   raw.fmt = GST_VIDEO_FORMAT_NV12; //only support NV12 format now.
   get_current_time(&p);
   convert_time_slot(p, time_slot_tmp);
   if (cnt == 0) {
      /* case 1: setup an new osd */
      //for 1080p channel, we demo the time osd.
      //build the osd data by composing the elements, time osd is like hh:mm:ss
      guint initial_osd_width = BIG_FONT_WIDTH * 8;
      guint initial_osd_height = BIG_FONT_HEIGHT;
      guchar initial_osd_data[BIG_FONT_WIDTH * 8 * BIG_FONT_HEIGHT * 3 / 2];
      guint  uv_offset = initial_osd_width * initial_osd_height;

      //Y plane
      image_copy(initial_osd_data,                      initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_HH_0]][0], BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 1 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_HH_1]][0], BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 2 * BIG_FONT_WIDTH, initial_osd_width, big_elm[COLON_DATA_INDEX][0],              BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 3 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_MM_0]][0], BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 4 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_MM_1]][0], BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 5 * BIG_FONT_WIDTH, initial_osd_width, big_elm[COLON_DATA_INDEX][0],              BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 6 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_SS_0]][0], BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 7 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_SS_1]][0], BIG_FONT_WIDTH, BIG_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      //UV plane
      image_copy(initial_osd_data + uv_offset,                      initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_HH_0]][1], BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 1 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_HH_1]][1], BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 2 * BIG_FONT_WIDTH, initial_osd_width, big_elm[COLON_DATA_INDEX][1],              BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 3 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_MM_0]][1], BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 4 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_MM_1]][1], BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 5 * BIG_FONT_WIDTH, initial_osd_width, big_elm[COLON_DATA_INDEX][1],              BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 6 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_SS_0]][1], BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 7 * BIG_FONT_WIDTH, initial_osd_width, big_elm[time_slot_tmp[TIME_SLOT_SS_1]][1], BIG_FONT_WIDTH, BIG_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);

      raw.width  = initial_osd_width;
      raw.height = initial_osd_height;
      raw.plane[0] = initial_osd_data;
      raw.plane[1] = initial_osd_data + uv_offset;
      raw.stride[0] = initial_osd_width;
      raw.stride[1] = initial_osd_width;

      osd_image = gst_video_preproc_create_image(GST_VIDEO_PREPROC(elm), &raw);
      osd_info         = (GstVideoPreprocOsdInfo *)g_malloc0(sizeof(*osd_info));
      osd_info->id     = CHANNEL_1080P_OSD_0;           //one channel may have several osd region, so use id to identify them.
      osd_info->type   = GST_VIDEO_PREPROC_OSD_TYPE_NEW;//first time to set osd, use GST_VIDEO_PREPROC_OSD_TYPE_NEW type.
      osd_info->image  = osd_image;                     //GstVaapiOsdMeta takes ownnership of this image, do not need to free it manually.
      osd_info->channel= GST_VIDEO_PREPROC_CHANNEL_1080P; // apply this osd to 1080p channel
      osd_info->cfg    = initial_osd_cfg;//osd geometry, position to put this osd, color key
      osd_infos = g_list_append(osd_infos, osd_info); //link this info

      //for d1 channel, we demo the chinese character osd
      GstVideoPreprocImageRaw chinese_osd_raw;
      chinese_osd_raw.fmt = GST_VIDEO_FORMAT_NV12;
      chinese_osd_raw.width  = OSD_CHN_WIDTH;
      chinese_osd_raw.height = OSD_CHN_HEIGHT;
      chinese_osd_raw.plane[0] = (guchar *)chinese_osd2;
      chinese_osd_raw.plane[1] = chinese_osd_raw.plane[0] + chinese_osd_raw.width * chinese_osd_raw.height;
      chinese_osd_raw.stride[0] = chinese_osd_raw.width;
      chinese_osd_raw.stride[1] = chinese_osd_raw.width;

      osd_image = gst_video_preproc_create_image(GST_VIDEO_PREPROC(elm), &chinese_osd_raw);
      osd_info         = (GstVideoPreprocOsdInfo *)g_malloc0(sizeof(*osd_info));
      osd_info->id     = CHANNEL_D1_OSD_0;
      osd_info->type   = GST_VIDEO_PREPROC_OSD_TYPE_NEW;//first time to set osd, use GST_VIDEO_PREPROC_OSD_TYPE_NEW type.
      osd_info->image  = osd_image;
      osd_info->channel= GST_VIDEO_PREPROC_CHANNEL_D1;
      osd_info->cfg    = osd_cfg[OSD_CHN];
      osd_infos = g_list_append(osd_infos, osd_info);
   } else if (cnt < 200){
      /* case 2: update partially */
      raw.width  = BIG_FONT_WIDTH;
      raw.height = BIG_FONT_HEIGHT;
      raw.stride[0] = SAMPLE_OSD_DATA_STRIDE;
      raw.stride[1] = SAMPLE_OSD_DATA_STRIDE;

#define CHECK_UPDATE_OSD(time_slot, font) \
      if(time_slot_tmp[time_slot] != time_slot_cur[time_slot]) \
      {                                                        \
         raw.plane[0] = font##_elm[time_slot_tmp[time_slot]][0];  \
         raw.plane[1] = font##_elm[time_slot_tmp[time_slot]][1];  \
         osd_infos = add_osd_update(osd_infos, CHANNEL_1080P_OSD_0, &update_osd_cfg_##font[time_slot], &raw, GST_VIDEO_PREPROC(elm)); \
      }

      CHECK_UPDATE_OSD(TIME_SLOT_HH_0, big);
      CHECK_UPDATE_OSD(TIME_SLOT_HH_1, big);
      CHECK_UPDATE_OSD(TIME_SLOT_MM_0, big);
      CHECK_UPDATE_OSD(TIME_SLOT_MM_1, big);
      CHECK_UPDATE_OSD(TIME_SLOT_SS_0, big);
      CHECK_UPDATE_OSD(TIME_SLOT_SS_1, big);
   }else if(cnt == 200) {
      /* case 3: change font */
      //change big font to small font
      guint initial_osd_width = SMALL_FONT_WIDTH * 8;
      guint initial_osd_height = SMALL_FONT_HEIGHT;
      guchar initial_osd_data[SMALL_FONT_WIDTH * 8 * SMALL_FONT_HEIGHT * 3 / 2];
      guint  uv_offset = initial_osd_width * initial_osd_height;

      //Y plane
      image_copy(initial_osd_data,                        initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_0]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 1 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_1]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 2 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][0],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 3 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_0]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 4 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_1]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 5 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][0],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 6 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_0]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 7 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_1]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      //UV plane
      image_copy(initial_osd_data + uv_offset,                        initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_0]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 1 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_1]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 2 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][1],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 3 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_0]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 4 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_1]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 5 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][1],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 6 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_0]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 7 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_1]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);

      raw.width  = initial_osd_width;
      raw.height = initial_osd_height;
      raw.plane[0] = initial_osd_data;
      raw.plane[1] = initial_osd_data + uv_offset;
      raw.stride[0] = initial_osd_width;
      raw.stride[1] = initial_osd_width;

      osd_image = gst_video_preproc_create_image(GST_VIDEO_PREPROC(elm), &raw);
      osd_info         = (GstVideoPreprocOsdInfo *)g_malloc0(sizeof(*osd_info));
      osd_info->id     = CHANNEL_1080P_OSD_0;           //osd you want to replace
      osd_info->type   = GST_VIDEO_PREPROC_OSD_TYPE_NEW;//replace the osd with small font osd, so use GST_VIDEO_PREPROC_OSD_TYPE_NEW type.
      osd_info->image  = osd_image;
      osd_info->channel= GST_VIDEO_PREPROC_CHANNEL_1080P;
      osd_info->cfg    = initial_osd_cfg_small;
      osd_infos = g_list_append(osd_infos, osd_info);
   } else if (cnt > 200 && cnt < 400){
      //update partailly
      raw.width  = SMALL_FONT_WIDTH;
      raw.height = SMALL_FONT_HEIGHT;
      raw.stride[0] = SAMPLE_OSD_DATA_STRIDE;
      raw.stride[1] = SAMPLE_OSD_DATA_STRIDE;

      CHECK_UPDATE_OSD(TIME_SLOT_HH_0, small);
      CHECK_UPDATE_OSD(TIME_SLOT_HH_1, small);
      CHECK_UPDATE_OSD(TIME_SLOT_MM_0, small);
      CHECK_UPDATE_OSD(TIME_SLOT_MM_1, small);
      CHECK_UPDATE_OSD(TIME_SLOT_SS_0, small);
      CHECK_UPDATE_OSD(TIME_SLOT_SS_1, small);
   } else if (cnt == 400){
      //remove osd on GST_VIDEO_PREPROC_CHANNEL_1080P
      osd_info         = (GstVideoPreprocOsdInfo *)g_malloc0(sizeof(*osd_info));
      osd_info->id     = CHANNEL_1080P_OSD_0;           //osd you want to replace
      osd_info->type   = GST_VIDEO_PREPROC_OSD_TYPE_INVALID;
      osd_info->channel= GST_VIDEO_PREPROC_CHANNEL_1080P;
      osd_infos = g_list_append(osd_infos, osd_info);
   } else if (cnt == 500){
      //show osd of small font time again on GST_VIDEO_PREPROC_CHANNEL_1080P
      guint initial_osd_width = SMALL_FONT_WIDTH * 8;
      guint initial_osd_height = SMALL_FONT_HEIGHT;
      guchar initial_osd_data[SMALL_FONT_WIDTH * 8 * SMALL_FONT_HEIGHT * 3 / 2];
      guint  uv_offset = initial_osd_width * initial_osd_height;

      //Y plane
      image_copy(initial_osd_data,                        initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_0]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 1 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_1]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 2 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][0],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 3 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_0]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 4 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_1]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 5 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][0],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 6 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_0]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + 7 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_1]][0], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT, SAMPLE_OSD_DATA_STRIDE);
      //UV plane
      image_copy(initial_osd_data + uv_offset,                        initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_0]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 1 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_HH_1]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 2 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][1],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 3 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_0]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 4 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_MM_1]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 5 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[COLON_DATA_INDEX][1],              SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 6 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_0]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);
      image_copy(initial_osd_data + uv_offset + 7 * SMALL_FONT_WIDTH, initial_osd_width, small_elm[time_slot_tmp[TIME_SLOT_SS_1]][1], SMALL_FONT_WIDTH, SMALL_FONT_HEIGHT / 2, SAMPLE_OSD_DATA_STRIDE);

      raw.width  = initial_osd_width;
      raw.height = initial_osd_height;
      raw.plane[0] = initial_osd_data;
      raw.plane[1] = initial_osd_data + uv_offset;
      raw.stride[0] = initial_osd_width;
      raw.stride[1] = initial_osd_width;

      osd_image = gst_video_preproc_create_image(GST_VIDEO_PREPROC(elm), &raw);
      osd_info         = (GstVideoPreprocOsdInfo *)g_malloc0(sizeof(*osd_info));
      osd_info->id     = CHANNEL_1080P_OSD_0;           //osd you want to replace
      osd_info->type   = GST_VIDEO_PREPROC_OSD_TYPE_NEW;//replace the osd with small font osd, so use GST_VIDEO_PREPROC_OSD_TYPE_NEW type.
      osd_info->image  = osd_image;
      osd_info->channel= GST_VIDEO_PREPROC_CHANNEL_1080P;
      osd_info->cfg    = initial_osd_cfg_small;
      osd_infos = g_list_append(osd_infos, osd_info);
   } else if (cnt > 500){
      //update partailly
      raw.width  = SMALL_FONT_WIDTH;
      raw.height = SMALL_FONT_HEIGHT;
      raw.stride[0] = SAMPLE_OSD_DATA_STRIDE;
      raw.stride[1] = SAMPLE_OSD_DATA_STRIDE;

      CHECK_UPDATE_OSD(TIME_SLOT_HH_0, small);
      CHECK_UPDATE_OSD(TIME_SLOT_HH_1, small);
      CHECK_UPDATE_OSD(TIME_SLOT_MM_0, small);
      CHECK_UPDATE_OSD(TIME_SLOT_MM_1, small);
      CHECK_UPDATE_OSD(TIME_SLOT_SS_0, small);
      CHECK_UPDATE_OSD(TIME_SLOT_SS_1, small);
   }
   //update current time
   convert_time_slot(p, time_slot_cur);
   cnt ++;
   return osd_infos;
}

static GstPadProbeReturn
src_preproc_smart_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    MediaPipeImpl *impl = (MediaPipeImpl *)user_data;
    GstBuffer *buf = (GstBuffer *)(info->data);

    guint queue_length = 0;
    guint i;
    Smart1080pData *data_1080p = NULL;
    SmartData *data_smart;

    if(buf == NULL || !GST_IS_BUFFER(buf))
    {
        return GST_PAD_PROBE_OK;
    }

    if (smart_control)
       g_mutex_lock(&impl->src_preproc.smart_lock);
    // need to be freed after using
    data_smart = g_new0(SmartData, 1);
    data_smart->buf = gst_buffer_ref(buf);
    data_smart->preBuf = gst_video_preproc_buffer_get_from_gst_buffer(
                    GST_VIDEO_PREPROC(impl->src_preproc.element),
                    buf);
    gst_video_preproc_map_buffer(GST_VIDEO_PREPROC(impl->src_preproc.element), data_smart->preBuf);
    if (dump_smart_analysis_raw_data == 1) {
       dump_nv12_timed(data_smart->preBuf, (char *)"smart-raw-data");
       dump_smart_analysis_raw_data = 0;
    }
    impl->src_preproc.smart_queue =
            g_list_append(impl->src_preproc.smart_queue, data_smart);

    if(impl->src_preproc.smart_analyze_callback)
    {
        impl->src_preproc.smart_analyze_callback(
                                        impl->src_preproc.smart_queue,
                                        impl->src_preproc.smart_1080p_queue,
                                        impl);
    }

    /* After smart analysis */

    //According to KEDA's input, we always free this smart frame after analysis.
    gst_video_preproc_unmap_buffer(GST_VIDEO_PREPROC(impl->src_preproc.element), data_smart->preBuf);
    gst_video_preproc_destroy_buffer(
          GST_VIDEO_PREPROC(impl->src_preproc.element),
          data_smart->preBuf);
    impl->src_preproc.smart_queue =
       g_list_remove(impl->src_preproc.smart_queue, data_smart);
    gst_buffer_unref(data_smart->buf);
    g_free(data_smart);

    if (smart_control) {
       queue_length = g_list_length(impl->src_preproc.smart_1080p_queue);
       for(i=0; i<queue_length; i++)
       {
          data_1080p = (Smart1080pData *)g_list_nth_data(impl->src_preproc.smart_1080p_queue, 0);
          if(!data_1080p->can_push)
          {
             // rest frames are still being used for smart analysis
             break;
          }

          impl->src_preproc.smart_1080p_queue =
             g_list_remove(impl->src_preproc.smart_1080p_queue, data_1080p);

          /* smart meta sample code start */
          GList *osd_infos = NULL;
          GList *wf_infos = NULL;
          GList *mask_infos = NULL;
          GstVaapiDvsInfo dvs_info;
          gboolean need_jpeg_encode = TRUE;
          gint j;
          static guint cnt = 0;

          /* OSD code start */
          if (global_enable_osd) {
             osd_infos = prepare_osd_infos(impl->preproc.element);
          }
          /* OSD code stop */

          /* wireframe code start */
          if (global_enable_wireframe) {
             GstVideoPreprocWireFrame *wf;
             if(enable_facedetect) {
                wf = cv_wire_frames;
             } else {
                wf = sample_wire_frames;
             }
             GstVideoPreprocWireFrameInfo *wf_info;
             for (j = 0; j < WIRE_FRAME_REGION_MAX_NUM; j++) {
                wf_info = (GstVideoPreprocWireFrameInfo *)g_malloc0(sizeof(*wf_info));
                wf_info->wireframe = wf[j];
                wf_info->id        = j;
                wf_info->channel = GST_VIDEO_PREPROC_CHANNEL_1080P;
                wf_infos = g_list_append(wf_infos, wf_info);
             }
          }
          /* wireframe code stop */

          /* mask code start */
          if (global_enable_mask) {
             GstVideoPreprocMaskCfg *masks;
             masks = sample_masks;
             for (j = 0; j < MASK_REGION_MAX_NUM; j++) {
                GstVideoPreprocMaskInfo *mask_info;
                mask_info = (GstVideoPreprocMaskInfo *)g_malloc0(sizeof(*mask_info));
                mask_info->cfg = masks[j];
                mask_info->id = j;
                mask_info->channel = GST_VIDEO_PREPROC_CHANNEL_1080P;
                mask_infos = g_list_append(mask_infos, mask_info);
             }
          }
          /* mask code stop */

          dvs_info.offset_x = dvs_offset_x;
          dvs_info.offset_y = dvs_offset_y;
          if (osd_infos || wf_infos || mask_infos || dvs_info.offset_x != 0 || dvs_info.offset_y != 0) {
             data_1080p->buf = gst_buffer_make_writable(data_1080p->buf);
             //this function will create an GstVaapiOsdMeta object to hold osd_infos, and bind this object to GstBuffer
             gst_buffer_add_vaapi_smart_meta(data_1080p->buf, osd_infos, wf_infos, mask_infos, need_jpeg_encode, &dvs_info);

             GstVaapiSmartMeta *smart_meta = gst_buffer_get_vaapi_smart_meta(data_1080p->buf);
             GST_DEBUG("mediapipe, cnt=%u, smart_meta=%p, osd_infos_len=%u, wf_infos_lens=%u, mask_infos_lens=%u, need_jpeg_encode=%d, gst_buffer = %p, ts= %" GST_TIME_FORMAT "\n",
                   cnt,
                   smart_meta,
                   g_list_length(smart_meta->osd_infos),
                   g_list_length(smart_meta->wf_infos),
                   g_list_length(smart_meta->mask_infos),
                   smart_meta->need_jpeg_encode,
                   data_1080p->buf,
                   GST_TIME_ARGS(GST_BUFFER_TIMESTAMP(data_1080p->buf)));
          }

          if (osd_infos) {
             g_list_free_full (osd_infos, (GDestroyNotify)_smart_meta_osd_info_free);
          }
          if (wf_infos) {
             g_list_free_full (wf_infos, (GDestroyNotify)_smart_meta_wf_info_free);
          }
          if (mask_infos) {
             g_list_free_full (mask_infos, (GDestroyNotify)_smart_meta_mask_info_free);
          }
          cnt++;
          /* smart meta sample code stop */

          gst_app_src_push_buffer(GST_APP_SRC(impl->main_source.app_src), data_1080p->buf);

          g_free(data_1080p);
       }

       g_mutex_unlock(&impl->src_preproc.smart_lock);
    }

    return GST_PAD_PROBE_OK;
}


static void
src_preproc_pad_linked_callback2(GstPad *pad, GstPad *peer, gpointer data)
{
    if(data == NULL)
    {
        return;
    }
    gst_pad_add_probe (
            pad,
            GST_PAD_PROBE_TYPE_BUFFER,
            (GstPadProbeCallback)src_preproc_luma_gain_probe_callback,
            data,
            NULL);
}

static void
src_preproc_pad_linked_callback3(GstPad *pad, GstPad *peer, gpointer data)
{
    if(data == NULL)
    {
        return;
    }
    gst_pad_add_probe (
                pad,
                GST_PAD_PROBE_TYPE_BUFFER,
                (GstPadProbeCallback)src_preproc_hdr_probe_callback,
                data,
                NULL);
}

/* ------------------------main pipeline functions------------------------- */
static gboolean
main_message_loop (GstBus *bus, GstMessage * message, gpointer user_data)
{
     MediaPipeImpl *impl = IMPL_CAST (user_data);
     gboolean ret = TRUE;

     switch (GST_MESSAGE_TYPE (message))
     {
        case GST_MESSAGE_EOS:
        {
            LOG_DEBUG ("main pipeline: End of stream\n");
            g_main_loop_quit (impl->main_loop);
            break;
        }
        case GST_MESSAGE_ERROR:
        {
            gchar  *debug;
            GError *error;

            gst_message_parse_error (message, &error, &debug);
            g_free (debug);

            LOG_ERROR ("mian pipeline: Error: %s\n", error->message);
            g_error_free (error);

            g_main_loop_quit (impl->main_loop);
            break;
        }
        default:
            break;
   }

    if (impl->main_msg_callback)
        ret = impl->main_msg_callback (message, impl->main_msg_user_data);

    return ret;
}

#define JPEG_QUEUE_LEN (30)
GList *jpeg_frames = NULL;
GList *frames_do_jpeg_encoding = NULL;
GMutex jpeg_lock;

static GstBuffer *
find_frame_by_ts(GList *frames, GstClockTime *ts)
{
   while(frames) {
      GstBuffer *buf = (GstBuffer *)frames->data;
      if(*ts == GST_BUFFER_TIMESTAMP(buf))
         return buf;
      frames = frames->next;
   }
   return NULL;
}

static void
do_qos(MediaPipeImpl *impl)
{
   int i;
   unsigned int level = 0;
   for(i=0;i<VIDEO_CHANNEL_MAX;i++)
   {
      if(impl->channel[i].encoder_queue)
      {
         g_object_get(G_OBJECT(impl->channel[i].encoder_queue), "current-level-buffers", &level, NULL);
         while(level > 0) {
            usleep(500000);
            g_object_get(G_OBJECT(impl->channel[i].encoder_queue), "current-level-buffers", &level, NULL);
         }
      }
   }
}

static gboolean
jpeg_enc(MediaPipeImpl *impl, GstBuffer *buf)
{
   GstVideoPreprocBuffer *preBuf;
   gboolean need_sleep_urgent = FALSE;
   gboolean need_sleep = FALSE;

   if (!jpeg_inited) {
      jpeg_init(USE_OPENCL);
      jpeg_inited = TRUE;
   }

   if (enable_qos) {
      do_qos(impl);
   }

   preBuf = gst_video_preproc_buffer_get_from_gst_buffer(
         GST_VIDEO_PREPROC(impl->preproc.element),
         buf);


   // only 1080p channel has been probed
   if (impl->preproc.jpeg_callback)
      (impl->preproc.jpeg_callback) (
            preBuf,
            impl->preproc.jpeg_user_data,
            VIDEO_CHANNEL_1080P);

   gst_video_preproc_destroy_buffer(
         GST_VIDEO_PREPROC(impl->preproc.element),
         preBuf);

   return TRUE;
}

static GstPadProbeReturn
main_1080p_jpeg_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    MediaPipeImpl *impl = (MediaPipeImpl *)user_data;
    GstBuffer *buf;
    GstClockTime oldest_ts;

    buf = (GstBuffer *)(info->data);
    if(buf == NULL || !GST_IS_BUFFER(buf))
    {
        return GST_PAD_PROBE_OK;
    }

    g_mutex_lock(&jpeg_lock);
    jpeg_frames = g_list_append(jpeg_frames, gst_buffer_ref(buf));
    oldest_ts = GST_BUFFER_TIMESTAMP((GstBuffer *)jpeg_frames->data);

    GList *l = frames_do_jpeg_encoding;
    GList *buffers = NULL;
    
    while(l) {
       GList *next = l->next;
       GstClockTime *ts = (GstClockTime *)l->data;
       if (*ts < oldest_ts) {
          frames_do_jpeg_encoding = g_list_remove(frames_do_jpeg_encoding, ts);
          g_free(ts);
       } else {
          GstBuffer *buf_to_encode = find_frame_by_ts(jpeg_frames, ts);
          if (buf_to_encode) {
             frames_do_jpeg_encoding = g_list_remove(frames_do_jpeg_encoding, ts);
             buffers                 = g_list_append(buffers, buf_to_encode);
             jpeg_frames             = g_list_remove(jpeg_frames, buf_to_encode);
          }
       }
       l = next;
    }

    g_mutex_unlock(&jpeg_lock);

    l = buffers;
    while(l) {
       GList *next = l->next;
       buf  = (GstBuffer *)l->data;
       jpeg_enc(impl, buf);
       g_list_remove(l, buf);
       gst_buffer_unref(buf);
       l = next;
    }

    while(g_list_length(jpeg_frames) > JPEG_QUEUE_LEN) {
       GstBuffer *oldest_buf = (GstBuffer *)jpeg_frames->data;
       jpeg_frames = g_list_remove(jpeg_frames, oldest_buf);
       gst_buffer_unref(oldest_buf);
    }
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
main_encoder_src_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    GstBuffer *buf;
    VideoChannel *channel;

    buf = (GstBuffer *)(info->data);

    if(buf == NULL || !GST_IS_BUFFER(buf) || user_data == NULL)
    {
        return GST_PAD_PROBE_OK;
    }

    channel = (VideoChannel *)user_data;

    if(channel->encoder.element == NULL)
    {
        return GST_PAD_PROBE_OK;
    }

    if (channel->encoder.callback)
    {
        (channel->encoder.callback) (buf, channel->encoder.user_data, (VideoChannelIndex)(channel->channel_index));
    }

    return GST_PAD_PROBE_OK;
}


static void
main_encoder_src_pad_linked_callback(GstPad *pad, GstPad *peer, gpointer data)
{
    VideoChannel *channel;
    GstPad *target_pad;

    if(data == NULL)
    {
        return;
    }

    channel = (VideoChannel *)data;


    if(channel->encoder.element == NULL)
    {
        return;
    }

    target_pad = gst_element_get_static_pad(channel->encoder.element, "src");


    // probe callback for h264 frame
    if(channel->encoder.frame_probe_id == 0)
    {
        channel->encoder.frame_probe_id =
            gst_pad_add_probe (
                    target_pad,
                    GST_PAD_PROBE_TYPE_BUFFER,
                    (GstPadProbeCallback)main_encoder_src_probe_callback,
                    channel,
                    NULL);

    }

    gst_object_unref(target_pad);
}

static void
main_encoder_src_pad_unlinked_callback(GstPad *pad, GstPad *peer, gpointer data)
{
    VideoChannel *channel;
    GstPad *target_pad;

    if(data == NULL)
    {
        return;
    }

    channel = (VideoChannel *)data;

    if(channel->encoder.element == NULL)
    {
        return;
    }

    target_pad = gst_element_get_static_pad(channel->encoder.element, "src");

    // remove probe callback for h264 frame
    if(channel->encoder.frame_probe_id)
    {
        gst_pad_remove_probe(target_pad, channel->encoder.frame_probe_id);
    }
    channel->encoder.frame_probe_id = 0;

    gst_object_unref(target_pad);
}

/* ------------------------impl->channel functions------------------------- */
static gboolean
need_enable_video_impl_channel(MediaPipeImpl *impl)
{
    guint i;
    g_assert(impl);

    for(i=0;i<VIDEO_CHANNEL_MAX;i++)
    {
        if(impl->channel[i].enable)
        {
            return TRUE;
        }
    }

    return FALSE;
}

static gboolean
disable_video_channel(MediaPipe *pipe, VideoChannelIndex channel_num)
{
    MediaPipeImpl *impl;
    VideoChannel *channel;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);
    channel = &impl->channel[channel_num];

    if(!channel->channel_on)
    {
        // Channel already disabled.
        return TRUE;
    }

    if(impl->preproc.element == NULL)
    {
        LOG_ERROR("impl->preproc.element not been enabled, cannot unlink channel from impl");
        return FALSE;
    }

    if(channel_num == VIDEO_CHANNEL_JPEG)
    {
        if (jpeg_inited)
           jpeg_release();
    }

    {
       if(channel->encoder.enable)
       {
          gst_element_unlink_pads (impl->preproc.element,
                tempString[channel_num].pad_name,
                channel->encoder_queue,
                "sink");// gst_element_unlink_pads doesn't support NULL pad name
          gst_element_unlink(channel->encoder_queue, channel->encoder.element);
          gst_element_unlink(channel->encoder.element, channel->profile_converter);
          gst_element_unlink(channel->profile_converter, channel->sink.element);

          gst_element_set_state(channel->encoder_queue, GST_STATE_NULL);
          gst_element_set_state(channel->encoder.element, GST_STATE_NULL);
          gst_element_set_state(channel->profile_converter, GST_STATE_NULL);
          gst_element_set_state(channel->sink.element, GST_STATE_NULL);

          gst_bin_remove_many(
                GST_BIN (impl->main_pipeline),
                channel->encoder_queue,
                channel->encoder.element,
                channel->profile_converter,
                channel->sink.element,
                NULL
                );

          channel->encoder_queue = NULL;
          channel->encoder.element = NULL;
          channel->profile_converter = NULL;
          channel->sink.element = NULL;
       }else
       {
          gst_element_unlink_pads (impl->preproc.element,
                tempString[channel_num].pad_name,
                channel->encoder_queue,
                "sink");// gst_element_unlink_pads doesn't support NULL pad name
          gst_element_unlink(channel->encoder_queue, channel->sink.element);
          gst_element_set_state(channel->encoder_queue, GST_STATE_NULL);
          gst_element_set_state(channel->sink.element, GST_STATE_NULL);

          gst_bin_remove_many(
                GST_BIN (impl->main_pipeline),
                channel->encoder_queue,
                channel->sink.element,
                NULL
                );
          channel->encoder_queue = NULL;
          channel->sink.element = NULL;
       }
    }

    channel->channel_on = FALSE;

    LOG_DEBUG("***Channel %s Disabled***", tempString[channel_num].pad_name);

    return TRUE;
}

static gboolean
disable_video_impl_preproc(MediaPipe *pipe)
{
    MediaPipeImpl *impl;
    guint i;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(impl->main_source.src_tee == NULL ||
       impl->preproc.element == NULL)
    {
        return TRUE;
    }

    for(i = 0; i < VIDEO_CHANNEL_MAX; i++)
    {
        if(impl->channel[i].channel_on)
        {
            disable_video_channel(pipe,(VideoChannelIndex)i);
        }
    }

    return TRUE;
}

static gboolean
enable_video_channel(MediaPipe *pipe, VideoChannelIndex channel_num)
{
    MediaPipeImpl *impl;
    VideoChannel *channel;
    gboolean h264_enable;
    GstPad *pad;
	GstCaps *caps;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);
    channel = &impl->channel[channel_num];

    if(impl->preproc.element == NULL)
    {
        LOG_ERROR("Video preproc.element has not been enabled");
        return FALSE;
    }

    if(channel->channel_on)
    {
        // channel already enabled.
        return TRUE;
    }

    if(channel_num == VIDEO_CHANNEL_JPEG)
    {
        if(!channel->jpeg_queue)
        {
            channel->jpeg_queue = create_element (QUEUE_PLUGIN_NAME, "jpeg_queue");
            g_return_val_if_fail (channel->jpeg_queue, FALSE);
            g_object_set (G_OBJECT (channel->jpeg_queue), "max-size-buffers", 200, "max-size-time",
                           (guint64) 0, "max-size-bytes", 0, "leaky", 1, NULL);
        }
        if(!channel->jpeg_sink.element)
        {
            channel->jpeg_sink.element = create_element (MEDIA_PIPE_FAKE_SINK_NAME, NULL);
            g_return_val_if_fail (channel->jpeg_sink.element, FALSE);
        }

        gst_bin_add_many (
            GST_BIN (impl->main_pipeline),
            channel->jpeg_queue,
            channel->jpeg_sink.element,
            NULL);
        gst_element_sync_state_with_parent(channel->jpeg_queue);
        gst_element_sync_state_with_parent(channel->jpeg_sink.element);

        gst_element_link_pads (impl->preproc.element,
                           tempString[channel_num].pad_name,
                           channel->jpeg_queue,
                           NULL);
        link_element(channel->jpeg_queue, channel->jpeg_sink.element);

        pad = gst_element_get_static_pad(channel->jpeg_sink.element, "sink");

        gst_pad_add_probe (
            pad,
            GST_PAD_PROBE_TYPE_BUFFER,
            (GstPadProbeCallback)main_1080p_jpeg_probe_callback,
            pipe,
            NULL);

        gst_object_unref(pad);
    }
    else
    {
        /*
        *   Create encoder queue plugin in one channel
        */
        if(!channel->encoder_queue)
        {
           gchar *queue_name;

           queue_name = g_strdup_printf("%s-encode-queue", channel_name[channel_num]);
           channel->encoder_queue = create_element (QUEUE_PLUGIN_NAME, queue_name);
           g_free(queue_name);
           g_return_val_if_fail (channel->encoder_queue, FALSE);
        }

        if (!channel->videorate.element) {
            channel->videorate.element = create_element (MEDIA_PIPE_VIDEORATE_NAME, NULL);
            g_return_val_if_fail (channel->videorate.element, FALSE);

            channel->videorate.capsfilter = create_element (MEDIA_PIPE_CAPSFILTER_NAME, NULL);
            g_return_val_if_fail (channel->videorate.capsfilter, FALSE);

            if (!channel->videorate.fps_n) {
                channel->videorate.fps_n = impl->main_source.fps_n;
                channel->videorate.fps_d = impl->main_source.fps_d;
            }

            caps = gst_caps_new_simple ("video/x-raw",
                "framerate", GST_TYPE_FRACTION, channel->videorate.fps_n, channel->videorate.fps_d,
                NULL);
            g_object_set (G_OBJECT (channel->videorate.capsfilter), "caps", caps, NULL);
            gst_caps_unref (caps);
        }
 
        h264_enable = channel->encoder.enable && (channel->sink.type != RTSP_SINK);

        /*
        *   Create H264 encoder plugin in one channel
        */
        if(h264_enable)
        {
            if(!channel->encoder.element)
            {
                channel->encoder.element = create_element (MEDIA_PIPE_ENCODER_NAME, NULL);
                g_return_val_if_fail (channel->encoder.element, FALSE);

                GstPad *signal_pad = gst_element_get_static_pad(channel->encoder.element,"src");
                g_signal_connect (signal_pad,
                                  "linked",
                                  G_CALLBACK (main_encoder_src_pad_linked_callback),
                                  channel);

                g_signal_connect (signal_pad,
                                  "unlinked",
                                  G_CALLBACK (main_encoder_src_pad_unlinked_callback),
                                  channel);
                gst_object_unref(signal_pad);

                g_object_set (channel->encoder.element,
                      "rate-control", channel->encoder.rate_control,
                      "bitrate", channel->encoder.bitrate,
                      "cabac", channel->encoder.enable_cabac,
                      "enable-mv", channel->encoder.mv,
                      "keyframe-period", channel->encoder.gop_M,
                      "max-bframes", channel->encoder.gop_N,
                      "dct8x8", channel->encoder.enable_dct8x8,
                      NULL);

            }

            if(!channel->profile_converter)
            {
                GstCaps *caps;
                channel->profile_converter = create_element (MEDIA_PIPE_CAPSFILTER_NAME, NULL);

                caps = gst_caps_new_simple ("video/x-h264",
                        "profile", G_TYPE_STRING, profile_name[channel->encoder.profile],
                        NULL);
                g_object_set (channel->profile_converter, "caps", caps, NULL);
                gst_caps_unref (caps);
            }
        }

        /*
        *   Create sink in one channel
        */
        if(!channel->sink.element)
        {
            switch(channel->sink.type)
            {
                case FILE_SINK:
                    channel->sink.element = create_element (MEDIA_PIPE_FILE_SINK_NAME, NULL);
                    g_return_val_if_fail (channel->sink.element, FALSE);
                    g_object_set (channel->sink.element,
                                  "location", tempString[channel_num].file_name,
                                  "async", FALSE,
                                  NULL);
                    break;

                case TCP_SINK:
                    channel->sink.element = create_element (MEDIA_PIPE_TCP_SINK_NAME, NULL);
                    g_return_val_if_fail (channel->sink.element, FALSE);
                    g_object_set (G_OBJECT (channel->sink.element), "host", channel->sink.host_ip, "port", channel->sink.port, "blocksize", 1024000, NULL);
                    break;
                case KMS_SINK:
                    local_preview = TRUE;
                    channel->sink.element = create_element (MEDIA_PIPE_KMS_SINK_NAME, NULL);
                    g_return_val_if_fail (channel->sink.element, FALSE);
                    g_object_set (G_OBJECT (channel->sink.element), "display", 4, "async", FALSE, "sync", FALSE, NULL);
                    g_signal_connect (channel->sink.element, "prepare-dvs-info", G_CALLBACK(prepare_dvs_info), NULL);
                    break;
                case RTSP_SINK:
                    channel->sink.element = create_element (MEDIA_PIPE_FAKE_SINK_NAME, NULL);
                    g_object_set (channel->sink.element,
                                  "async", FALSE,
                                  NULL);
                    g_return_val_if_fail (channel->sink.element, FALSE);
		            rtsp_server_create(impl, channel_num);
		            break;
                case FAKE_SINK:
                case UDP_SINK:
                case V4L2_SINK:
                default:
                    channel->sink.element = create_element (MEDIA_PIPE_FAKE_SINK_NAME, NULL);
                    g_object_set (channel->sink.element,
                                  "async", FALSE,
                                  NULL);
                    g_return_val_if_fail (channel->sink.element, FALSE);
                    break;
            }
        }

        if(h264_enable)
        {
            gst_bin_add_many (
                GST_BIN (impl->main_pipeline),
                channel->encoder_queue,
                channel->videorate.element,
                channel->videorate.capsfilter,
                channel->encoder.element,
                channel->profile_converter,
                channel->sink.element,
                NULL);

            gst_element_sync_state_with_parent(channel->encoder.element);
            gst_element_sync_state_with_parent(channel->videorate.element);
            gst_element_sync_state_with_parent(channel->videorate.capsfilter);
            gst_element_sync_state_with_parent(channel->encoder_queue);
            gst_element_sync_state_with_parent(channel->profile_converter);
            gst_element_sync_state_with_parent(channel->sink.element);

            gst_element_link_pads (impl->preproc.element,
                               tempString[channel_num].pad_name,
                               channel->encoder_queue,
                               NULL);
            link_element(channel->encoder_queue, channel->videorate.element);
            link_element(channel->videorate.element, channel->videorate.capsfilter);
            link_element(channel->videorate.capsfilter, channel->encoder.element);
            link_element(channel->encoder.element, channel->profile_converter);
            link_element(channel->profile_converter, channel->sink.element);
        }
        else
        {
            gst_bin_add_many (
                        GST_BIN (impl->main_pipeline),
                        channel->encoder_queue,
                        channel->videorate.element,
                        channel->videorate.capsfilter,
                        channel->sink.element,
                        NULL);

            gst_element_sync_state_with_parent(channel->encoder_queue);
            gst_element_sync_state_with_parent(channel->videorate.element);
            gst_element_sync_state_with_parent(channel->videorate.capsfilter);
            gst_element_sync_state_with_parent(channel->sink.element);

            gst_element_link_pads (impl->preproc.element,
                               tempString[channel_num].pad_name,
                               channel->encoder_queue,
                               NULL);
            link_element(channel->encoder_queue, channel->videorate.element);
	    link_element(channel->videorate.element, channel->videorate.capsfilter);
            link_element(channel->videorate.capsfilter, channel->sink.element);
        }
    }

    channel->channel_index = channel_num;
    channel->channel_on = TRUE;

    LOG_DEBUG("***Channel %s Enabled***", tempString[channel_num].pad_name);

    return TRUE;
}

static gboolean
enable_video_impl_preproc(MediaPipe *pipe)
{
    MediaPipeImpl *impl;
    guint i = 0;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(impl->main_source.src_tee == NULL)
    {
        LOG_ERROR("Main Pipeline has not been initialized");
        return FALSE;
    }

    if(impl->preproc.element != NULL)
    {
        return TRUE;
    }

    /*
    *   Create Preproc plugin
    */
    if(!impl->preproc.element)
    {
        impl->preproc.element = create_element (MEDIA_PIPE_PREPROC_NAME, NULL);
        g_return_val_if_fail (impl->preproc.element, FALSE);
    }

    /*
    *   Set flip parameter for preproc
    */
    LOG_DEBUG ("******Set flip to %d ", global_flip_mode);
    if(!gst_video_preproc_set_flip(GST_VIDEO_PREPROC(impl->preproc.element), global_flip_mode))
    {
        LOG_WARNING ("Set flip mode for preproc FAIL!");
    }

     /*
      *   Set luma gain parameter for preproc
      */
      LOG_DEBUG ("******Set luma gain to %d ", impl->preproc.luma_gain);
      if(!gst_video_preproc_set_luma_gain(GST_VIDEO_PREPROC(impl->preproc.element),impl->preproc.luma_gain))
      {
         LOG_WARNING ("Set luma gain for preproc FAIL!");
      }

    // Add/Link impl elements
    gst_bin_add_many (
            GST_BIN (impl->main_pipeline),
            impl->preproc.element,
            NULL);


    link_element(impl->main_source.src_tee, impl->preproc.element);

    // Enable channels
    for(i = 0; i < VIDEO_CHANNEL_MAX; i++)
    {
        if(impl->channel[i].enable)
        {
            enable_video_channel(pipe,(VideoChannelIndex)i);
        }

    }

    /*
    *   Set Rotation parameter for preproc
    */
    if (local_preview) {
       LOG_DEBUG ("******Don't support rotate in local preview mode");
    }else{
       LOG_DEBUG ("******Set rotation to %d ", impl->preproc.rotate_mode);
       if(!gst_video_preproc_set_rotate(GST_VIDEO_PREPROC(impl->preproc.element),impl->preproc.rotate_mode))
       {
          LOG_WARNING ("Set rotation for preproc FAIL!");
       }
    }

    return TRUE;
}

static GstPadProbeReturn
dynamic_enable_channel_push_cap_event_callback (
                                    GstPad *pad,
                                    GstPadProbeInfo *info,
                                    gpointer user_data)
{
    DynamicChannelData *dynamic_data = (DynamicChannelData *)user_data;
    MediaPipeImpl *impl = IMPL_CAST(dynamic_data->media_pipe);

    gst_pad_remove_probe(pad, info->id);

    GstCaps *caps;

    if(impl->preproc.rotate_mode == GST_VIDEO_PREPROC_ROTATE_NONE ||
       impl->preproc.rotate_mode == GST_VIDEO_PREPROC_ROTATE_180)
    {
        caps =
            gst_caps_new_simple ("video/x-raw",
                "format", G_TYPE_STRING, gst_video_format_to_string(impl->main_source.format),
                "width", G_TYPE_INT, normal_resolution[dynamic_data->channel_num].width,
                "height", G_TYPE_INT, normal_resolution[dynamic_data->channel_num].height,
                "framerate", GST_TYPE_FRACTION, impl->main_source.fps_n, impl->main_source.fps_d,
                "interlace-mode", G_TYPE_STRING, "progressive",
                NULL);
    }else
    {
        caps =
            gst_caps_new_simple ("video/x-raw",
                "format", G_TYPE_STRING, gst_video_format_to_string(impl->main_source.format),
                "width", G_TYPE_INT, rotate_resolution[dynamic_data->channel_num].width,
                "height", G_TYPE_INT, rotate_resolution[dynamic_data->channel_num].height,
                "framerate", GST_TYPE_FRACTION, impl->main_source.fps_n, impl->main_source.fps_d,
                "interlace-mode", G_TYPE_STRING, "progressive",
                NULL);

    }


    gst_pad_push_event(pad, gst_event_new_caps(caps));

    gst_caps_unref (caps);

    g_free(dynamic_data);
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
dynamic_enable_channel_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    DynamicChannelData *dynamic_data = (DynamicChannelData *)user_data;
    MediaPipeImpl *impl = IMPL_CAST(dynamic_data->media_pipe);
    GstPad *dynamic_pad;
    gst_pad_remove_probe(pad, info->id);

    if(impl->channel[dynamic_data->channel_num].channel_on)
    {
        // if channel already enabled, skip
        g_free(dynamic_data);
        goto exit;
    }

    enable_video_channel(dynamic_data->media_pipe, dynamic_data->channel_num);


    // caps event for encoder initialization must be sent after new stream event
    // so, add a probe for first pack of frame data to send caps event
    dynamic_pad =
                gst_element_get_static_pad(
                    impl->channel[dynamic_data->channel_num].encoder_queue,
                    "src");

    gst_pad_add_probe (dynamic_pad,
                       GST_PAD_PROBE_TYPE_DATA_DOWNSTREAM,
                       (GstPadProbeCallback)dynamic_enable_channel_push_cap_event_callback,
                       user_data,
                       NULL);
exit:
    return GST_PAD_PROBE_OK;
}

static GstPadProbeReturn
dynamic_disable_channel_probe_callback (
                        GstPad *pad,
                        GstPadProbeInfo *info,
                        gpointer user_data)
{
    DynamicChannelData *dynamic_data = (DynamicChannelData *)user_data;

    gst_pad_remove_probe(pad, info->id);

    disable_video_channel(dynamic_data->media_pipe, dynamic_data->channel_num);

    g_free(dynamic_data);

    return GST_PAD_PROBE_OK;
}

static gboolean
build_pipeline (MediaPipe *pipe)
{
    MediaPipeImpl *impl = IMPL_CAST (pipe);
    GstBus *bus;
    GstCaps *caps;
    g_return_val_if_fail (impl, FALSE);

    /* ---------------initialize src pipeline members---------------------*/

    // create pipeline
    impl->src_pipeline = gst_pipeline_new("src-pipeline");

    // Create Source plugin
    switch (impl->src_source.src_type) {
       case SRC_TYPE_VIDEO_TEST:
          LOG_DEBUG("***Using Videotestsrc***");
          impl->src_source.gen_src = create_element (MEDIA_PIPE_TEST_SOURCE_NAME, "test-src");
          g_object_set (impl->src_source.gen_src,
                "is-live", TRUE,
                NULL);
          break;
       case SRC_TYPE_V4L2:
          LOG_DEBUG("***Using v4l2src***");
          impl->src_source.gen_src = create_element (MEDIA_PIPE_V4L2_SOURCE_NAME, "xcam-src");

          g_object_set (impl->src_source.gen_src,
                "sensor-id", impl->src_source.v4l2_src_sensor_id,
                "io-mode", impl->src_source.v4l2_src_io_mode,
                "enable-3a", impl->src_source.v4l2_enable_3a,
                "device", impl->src_source.v4l2_src_device,
                "capture-mode", impl->src_source.v4l2_capture_mode,
                "coloreffect", global_v4l2src_color_effect,
                "imageprocessor", impl->src_source.image_processor,
                "analyzer", impl->src_source.analyzer,
                "fpsn", impl->src_source.fps_n,
                "fpsd", impl->src_source.fps_d,
                NULL);
          break;
       case SRC_TYPE_FILE:
          impl->src_source.gen_src = create_filesrc_bin(impl->src_source.location);
          break;
       default:
          break;
    }

    // Create Caps-Filter
    impl->src_source.src_filter = create_element (MEDIA_PIPE_CAPSFILTER_NAME, "src-filter");

    caps = gst_caps_new_simple ("video/x-raw",
            "format", G_TYPE_STRING, gst_video_format_to_string(impl->src_source.format),
            "width", G_TYPE_INT, impl->src_source.width,
            "height", G_TYPE_INT, impl->src_source.height,
            "framerate", GST_TYPE_FRACTION, impl->src_source.fps_n, impl->src_source.fps_d,
            NULL);
    g_object_set (impl->src_source.src_filter, "caps", caps, NULL);
    gst_caps_unref (caps);

    impl->src_source.src_queue = create_element (QUEUE_PLUGIN_NAME, "src-queue");
    g_object_set (G_OBJECT (impl->src_source.src_queue), "max-size-buffers", 10, "max-size-time",
          (guint64) 0, "max-size-bytes", 0, "leaky", 2, NULL);

    // Create preproc for src pipeline
    impl->src_preproc.element = create_element (MEDIA_PIPE_PREPROC_NAME, "src-preproc");
    g_object_set (impl->src_preproc.element,
                  "smart-factor", smart_factor,
                  "smart-resolution", impl->src_preproc.smart_resolution,
                  "test-autohdr", impl->src_preproc.vpp_enable_autohdr,
                  "need-copy", TRUE,
                  "need-adjust-timestamp", TRUE,
                  "capture-fps", impl->src_source.fps_n/(float)impl->src_source.fps_d,
                  NULL);
    if(enable_lumagain_threshold)
    {
        GstPad *src_preproc_pad = gst_element_get_static_pad(
                                        impl->src_preproc.element,
                                        "src");
        g_signal_connect (src_preproc_pad,
                          "linked",
                          G_CALLBACK (src_preproc_pad_linked_callback2),
                          pipe);
        gst_object_unref(src_preproc_pad);
    }

    GstPad *src_preproc_pad = gst_element_get_static_pad(
          impl->src_preproc.element,
          "src");
    g_signal_connect (src_preproc_pad,
          "linked",
          G_CALLBACK (src_preproc_pad_linked_callback3),
          pipe);
    gst_object_unref(src_preproc_pad);

    // Create fakesink for src pipeline (use 1080p & smart channel only)
    impl->src_queue[VIDEO_CHANNEL_1080P] = create_element(QUEUE_PLUGIN_NAME, "src-queue-1080p");
    impl->src_fakesink[VIDEO_CHANNEL_1080P].element =
       create_element (MEDIA_PIPE_FAKE_SINK_NAME, "src-fakesink-1080p");

    g_object_set (impl->src_fakesink[VIDEO_CHANNEL_1080P].element,
          "async", FALSE,
          NULL);

    GstPad *src_1080p_pad = gst_element_get_static_pad(
          impl->src_queue[VIDEO_CHANNEL_1080P], "src");
    if(impl->src_preproc.smart_frame_1080p_probe_id == 0)
    {
       impl->src_preproc.smart_frame_1080p_probe_id =
          gst_pad_add_probe (
                src_1080p_pad,
                GST_PAD_PROBE_TYPE_BUFFER,
                (GstPadProbeCallback)src_preproc_1080p_probe_callback,
                impl,
                NULL);
    }
    gst_object_unref(src_1080p_pad);


    impl->src_queue[VIDEO_CHANNEL_SMART] = create_element(QUEUE_PLUGIN_NAME, "src-queue-smart");
    g_object_set (G_OBJECT (impl->src_queue[VIDEO_CHANNEL_SMART]), "max-size-buffers", 100, "max-size-time",
          (guint64) 0, "max-size-bytes", 0, "leaky", 1, NULL);
    GstPad *probe_pad = gst_element_get_static_pad(
                                    impl->src_queue[VIDEO_CHANNEL_SMART], "src");

    if(impl->src_preproc.smart_frame_smart_probe_id == 0)
    {
       impl->src_preproc.smart_frame_smart_probe_id =
          gst_pad_add_probe (
                probe_pad,
                GST_PAD_PROBE_TYPE_BUFFER,
                (GstPadProbeCallback)src_preproc_smart_probe_callback,
                impl,
                NULL);
    }
    gst_object_unref(probe_pad);

    impl->src_fakesink[VIDEO_CHANNEL_SMART].element =
                                create_element (MEDIA_PIPE_FAKE_SINK_NAME, "src-fakesink-smart");

    g_object_set (impl->src_fakesink[VIDEO_CHANNEL_SMART].element,
                  "async", FALSE,
                  NULL);

    // register probe callbacks for src pipeline

    gst_bin_add_many (
                GST_BIN (impl->src_pipeline),
                impl->src_source.gen_src,
                impl->src_source.src_filter,
                impl->src_source.src_queue,
                impl->src_preproc.element,
                impl->src_queue[VIDEO_CHANNEL_1080P],
                impl->src_fakesink[VIDEO_CHANNEL_1080P].element,
                impl->src_queue[VIDEO_CHANNEL_SMART],
                impl->src_fakesink[VIDEO_CHANNEL_SMART].element,
                NULL);

    /* ---------------initialize main pipeline members---------------------*/

    impl->main_loop = g_main_loop_new (NULL, FALSE);
    //Create Pipeline
    impl->main_pipeline = gst_pipeline_new("main-pipeline");

    // Create appsrc for main pipeline
    impl->main_source.app_src = create_element(MEDIA_PIPE_APP_SRC_NAME, "main-appsrc");
    g_object_set (impl->main_source.app_src,
                  "block", TRUE,
                  "is-live", TRUE,
                  "max-bytes", 10000000ULL, //10M
                  "format", GST_FORMAT_TIME, // For videorate element, non-time format segment event fails it
                  NULL);
    /*
    ** Because there's no format converter (postproc) in main pipeline:
    ** 1. appsrc in main pipeline should output video with format of GST_VIDEO_FORMAT_NV12 which is only supported by preproc
    ** 2. appsrc in main pipeline must be set with "progressive" interlace-mode which is only supported by vaapiencode_h264
    */
    caps = gst_caps_new_simple ("video/x-raw",
            "format", G_TYPE_STRING, gst_video_format_to_string(impl->main_source.format),
            "width", G_TYPE_INT, impl->main_source.width,
            "height", G_TYPE_INT, impl->main_source.height,
            "framerate", GST_TYPE_FRACTION, impl->main_source.fps_n, impl->main_source.fps_d,
            "interlace-mode", G_TYPE_STRING, "progressive",
            NULL);
    g_object_set (impl->main_source.app_src, "caps", caps, NULL);
    gst_caps_unref (caps);


    // Create main tee plugin
    impl->main_source.src_tee = create_element (TEE_PLUGIN_NAME, "main-tee");

    /* ---------------add/link main pipeline members---------------------*/

    gst_bin_add_many (
                GST_BIN (impl->main_pipeline),
                impl->main_source.app_src,
                impl->main_source.src_tee,
                NULL);

    // Create impl->preproc
    gboolean result;
    if(need_enable_video_impl_channel(impl))
    {
        result = enable_video_impl_preproc(pipe);
        if(!result)
        {
            GST_ERROR ("Enable video impl->preproc Fail");
        }
    }

    link_element(impl->main_source.app_src, impl->main_source.src_tee);

    gst_object_ref (GST_OBJECT_CAST (impl->main_source.app_src));

    /* ---------------link/start src pipeline members---------------------*/
    if (impl->src_source.src_type == SRC_TYPE_FILE) {
       link_element(impl->src_source.gen_src, impl->src_preproc.element);
    }else {
       link_element(impl->src_source.gen_src, impl->src_source.src_filter);
       link_element(impl->src_source.src_filter, impl->src_source.src_queue);
       link_element(impl->src_source.src_queue, impl->src_preproc.element);
    }

    gst_element_link_pads (impl->src_preproc.element,
                           tempString[VIDEO_CHANNEL_1080P].pad_name,
                           impl->src_queue[VIDEO_CHANNEL_1080P],
                           NULL);

    link_element(impl->src_queue[VIDEO_CHANNEL_1080P], impl->src_fakesink[VIDEO_CHANNEL_1080P].element);

    gst_element_link_pads (impl->src_preproc.element,
                           tempString[VIDEO_CHANNEL_SMART].pad_name,
                           impl->src_queue[VIDEO_CHANNEL_SMART],
                           NULL);
    link_element(impl->src_queue[VIDEO_CHANNEL_SMART], impl->src_fakesink[VIDEO_CHANNEL_SMART].element);

    // Add Bus Message callbacks for src pipeline
    bus = gst_element_get_bus (impl->src_pipeline);
    impl->src_bus_watch = gst_bus_add_watch (bus, src_message_loop, (gpointer)pipe);
    if (!impl->src_bus_watch) {
        LOG_WARNING ("Add message watcher to bus failed");
    }

    // Add Bus Message callbacks for main pipeline
    bus = gst_element_get_bus (impl->main_pipeline);
    impl->main_bus_watch = gst_bus_add_watch (bus, main_message_loop, (gpointer)pipe);
    if (!impl->main_bus_watch) {
        LOG_WARNING ("Add message watcher to bus failed");
    }

    return TRUE;
}

static gboolean
destroy_pipeline (MediaPipe *pipe)
{
    MediaPipeImpl *impl = IMPL_CAST (pipe);

    if (!impl->main_pipeline)
        return TRUE;

    // release dynamical memory
    if(impl->src_source.location)
       g_free(impl->src_source.location);

    if(impl->src_source.v4l2_src_device)
       g_free(impl->src_source.v4l2_src_device);

    if(impl->preproc.element != NULL )
    {
        guint i;
        for(i=0; i<VIDEO_CHANNEL_MAX; i++)
        {
            if(impl->channel[i].osd_buffer != NULL)
            {
                gst_video_preproc_destroy_buffer(
                                        GST_VIDEO_PREPROC(impl->preproc.element),
                                        impl->channel[i].osd_buffer);
            }
        }
    }

    impl->main_bus_watch = 0;
	impl->src_bus_watch = 0;

    // unlink elements of each preproc and release source
    disable_video_impl_preproc(pipe);

    gst_element_set_state (impl->main_pipeline, GST_STATE_NULL);
	gst_element_set_state (impl->src_pipeline, GST_STATE_NULL);
    pipe->pipe_running = FALSE;

    gst_object_replace ((GstObject**) (&impl->main_pipeline), NULL);
	gst_object_replace((GstObject**) (&impl->src_pipeline), NULL);
    gst_object_replace ((GstObject**) (&impl->main_source.app_src), NULL);

    g_mutex_clear(&impl->src_preproc.smart_lock);

    return TRUE;
}

MediaPipe*
media_pipe_create (int argc,  char *agrv[])
{
    MediaPipeImpl *impl;
    guint i;

    impl = g_new0 (MediaPipeImpl, 1);
    g_assert (impl);

    impl->pipe.size = sizeof(MediaPipeImpl);
    impl->pipe.pipe_running = FALSE;

    /* default setting of src pipeline members */
    // src video source properties
    impl->src_source.width = MEDIA_VIDEO_DEFAULT_WIDTH;
    impl->src_source.height = MEDIA_VIDEO_DEFAULT_HEIGHT;
    impl->src_source.format = MEDIA_VIDEO_DEFAULT_FORMAT;
    impl->src_source.fps_n = MEDIA_VIDEO_DEFAULT_FPS_N;
    impl->src_source.fps_d = MEDIA_VIDEO_DEFAULT_FPS_D;

    impl->src_source.use_v4l2_src = FALSE;
    impl->src_source.v4l2_src_sensor_id = MEDIA_VIDEO_DEFAULT_SENSOR_ID;
    impl->src_source.v4l2_src_io_mode = DMA_MODE;

    // src preproc doesn't enable rotate/mask/osd etc.
    impl->src_preproc.rotate_mode = GST_VIDEO_PREPROC_ROTATE_NONE;
    impl->src_preproc.vpp_autohdr_mode = GST_VIDEO_PREPROC_AUTOHDR_INVALID;
    impl->src_preproc.vpp_enable_autohdr = FALSE;
    impl->src_preproc.enable_mask = FALSE;
    impl->src_preproc.luma_gain = 100; //100 means original luminance
    g_mutex_init(&impl->src_preproc.smart_lock);

    // src pipeline use fakesink only
    for(i = 0; i < VIDEO_CHANNEL_MAX; i++)
    {
        impl->src_fakesink[i].type = FAKE_SINK;
    }

    /* default setting of main pipeline members */
    impl->main_source.width = MEDIA_VIDEO_DEFAULT_WIDTH;
    impl->main_source.height = MEDIA_VIDEO_DEFAULT_HEIGHT;
    impl->main_source.format = MEDIA_VIDEO_DEFAULT_FORMAT;
    impl->main_source.fps_n = MEDIA_VIDEO_DEFAULT_FPS_N;
    impl->main_source.fps_d = MEDIA_VIDEO_DEFAULT_FPS_D;

    impl->preproc.rotate_mode = GST_VIDEO_PREPROC_ROTATE_NONE;

    // Enable 1080p channel
    VideoChannel *channel_0;
    channel_0 = &impl->channel[VIDEO_CHANNEL_1080P];

    channel_0->enable = TRUE;
    channel_0->encoder.enable = TRUE;
    channel_0->encoder.mv = 1;
    channel_0->sink.type = FILE_SINK;

    if (!gst_is_initialized ())
        gst_init (&argc, &agrv);

    return (MediaPipe*)impl;
}

void
media_pipe_destroy (MediaPipe *pipe)
{
    MediaPipeImpl *impl;

    g_assert(pipe);
    if (!pipe)
        return;

    impl = IMPL_CAST(pipe);

    destroy_pipeline (pipe);

    g_free (impl);
    LOG_DEBUG ("media pipe destroyed.");
}

gboolean
media_pipe_set_filesrc_location (MediaPipe *pipe, gchar *location)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting filesrc location dynamically");
        return FALSE;
    }

    impl->src_source.location = g_strdup(location);

    return TRUE;
}

gboolean
media_pipe_set_filesrc_type (MediaPipe *pipe, SrcType src_type)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting src type dynamically");
        return FALSE;
    }

    impl->src_source.src_type = src_type;

    return TRUE;
}


gboolean
media_pipe_set_use_v4l2_src (MediaPipe *pipe, gboolean use_v4l2_src)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting use_v4l2_src dynamically");
        return FALSE;
    }

    impl->src_source.use_v4l2_src = use_v4l2_src;

    return TRUE;
}

gboolean
media_pipe_set_v4l2_src_device (MediaPipe *pipe, gchar *v4l2_src_device)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting device dynamically");
        return FALSE;
    }

    impl->src_source.v4l2_src_device = g_strdup(v4l2_src_device);

    return TRUE;
}

gboolean
media_pipe_set_v4l2_src_sensor_id (MediaPipe *pipe, guint v4l2_src_sensor_id)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting sensor_id dynamically");
        return FALSE;
    }

    impl->src_source.v4l2_src_sensor_id = v4l2_src_sensor_id;

    return TRUE;
}

gboolean
media_pipe_set_v4l2_src_io_mode (MediaPipe *pipe, guint v4l2_src_io_mode)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting io_mode dynamically");
        return FALSE;
    }

    impl->src_source.v4l2_src_io_mode = v4l2_src_io_mode;

    return TRUE;
}

gboolean
media_pipe_set_v4l2_src_enable_3a (MediaPipe *pipe, gboolean enable_3a)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    // Now we support enable_3a dynamically
    // if(pipe->pipe_running)
    // {
    //     LOG_ERROR("Pipeline is running, not support setting enable_3a dynamically");
    //     return FALSE;
    // }
    g_object_set (impl->src_source.gen_src,
		  "enable-3a", enable_3a,
		  NULL);
    impl->src_source.v4l2_enable_3a = enable_3a;

    return TRUE;
}

gboolean
media_pipe_set_v4l2_src_capture_mode (MediaPipe *pipe, guint capture_mode)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting sensor_id dynamically");
        return FALSE;
    }

    impl->src_source.v4l2_capture_mode = capture_mode;

    return TRUE;
}

gboolean
media_pipe_set_vpp_src_enable_autohdr (MediaPipe *pipe, gboolean enable_autohdr)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting enable_3a dynamically");
        return FALSE;
    }

    printf("zjuan  enable_autohdr=%u", enable_autohdr);
    impl->src_preproc.vpp_enable_autohdr = enable_autohdr;

    return TRUE;
}

gboolean
media_pipe_set_vpp_src_set_autohdrmode (MediaPipe *pipe, guint autohdr_mode)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting enable_3a dynamically");
        return FALSE;
    }

    printf("zjuan  autohdr_mode=%u", autohdr_mode);
    impl->src_preproc.vpp_autohdr_mode = (GstVideoPreprocAutoHDRMode)autohdr_mode;

    return TRUE;

}

gboolean
media_pipe_set_src_parse_3aconf_callback (
                            MediaPipe *pipe,
                            Parse3AConfCallback parse_3aconf_callback,
                            gpointer user_data)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting 3aconf callback dynamically");
        return FALSE;
    }

    impl->src_preproc.parse_3aconf_callback = parse_3aconf_callback;
    impl->src_preproc.parse_3aconf_user_data = user_data;

    return TRUE;
}

gboolean
media_pipe_reconfig_3a (MediaPipe *pipe)
{
    MediaPipeImpl           *impl;
    GstElement              *v4l2src;
    GstXCam3A               *xcam;
    GstXCam3AInterface      *xcam_interface;
    gboolean ret = TRUE;

    if(pipe == NULL)
    {
        LOG_ERROR("Invalid mediapipe.");
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

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

    xcam = GST_XCAM_3A (v4l2src);
    xcam_interface = GST_XCAM_3A_GET_INTERFACE (xcam);
    if(xcam_interface == NULL)
    {
        LOG_ERROR("Failed to get xcam interface.");
        return FALSE;
    }

    /**/
    Config3A config_camera = {0}; //clear all to zero
    if(impl->src_preproc.parse_3aconf_callback) {
        if(FALSE == impl->src_preproc.parse_3aconf_callback(&config_camera, impl->src_preproc.parse_3aconf_user_data))
        {
            LOG_ERROR("Failed to parse 3a config.");
            return FALSE;
        }

        /*apply 3a setting to camera*/
        if(config_camera.flags & CONFIGFLAG_3A_WHITEBALANCE) {
            ret = xcam_interface->set_white_balance_mode(xcam, (XCamAwbMode)(config_camera.wb.val_wb_mode));
            ret = xcam_interface->set_awb_speed (xcam, config_camera.wb.val_awb_speed);
            ret = xcam_interface->set_wb_color_temperature_range(xcam, config_camera.wb.val_awb_cct_min, config_camera.wb.val_awb_cct_max);
            ret = xcam_interface->set_manual_wb_gain(xcam, config_camera.wb.val_awb_gr, config_camera.wb.val_awb_r,
                                        config_camera.wb.val_awb_b, config_camera.wb.val_awb_gb);
        }

        if(config_camera.flags & CONFIGFLAG_3A_EXPOSURE) {
            ret = xcam_interface->set_exposure_mode(xcam, (XCamAeMode)(config_camera.ep.val_ep_mode));
            ret = xcam_interface->set_ae_metering_mode(xcam, (XCamAeMeteringMode)(config_camera.ep.val_meter_mode));
            if (config_camera.ep.val_meter_mode == XCAM_AE_METERING_MODE_SPOT) {
                ret = xcam_interface->set_exposure_window(xcam, &config_camera.ep.val_ep_window[0], 1);
            }
            else if (config_camera.ep.val_meter_mode == XCAM_AE_METERING_MODE_WEIGHTED_WINDOW) {
                ret = xcam_interface->set_exposure_window(xcam, &config_camera.ep.val_ep_window[0], config_camera.ep.val_ep_window_count);
            }

            ret = xcam_interface->set_exposure_value_offset(xcam, config_camera.ep.val_ep_offset);
            ret = xcam_interface->set_ae_speed(xcam, config_camera.ep.val_ep_speed);
            ret = xcam_interface->set_exposure_flicker_mode(xcam, (XCamFlickerMode)(config_camera.ep.val_ep_flicker));
            ret = xcam_interface->set_manual_exposure_time(xcam, config_camera.ep.val_ep_manual_time);
            ret = xcam_interface->set_manual_analog_gain(xcam, config_camera.ep.val_ep_manual_analoggain);
            ret = xcam_interface->set_max_analog_gain(xcam, config_camera.ep.val_ep_max_analoggain);
            ret = xcam_interface->set_exposure_time_range(xcam, config_camera.ep.val_ep_timerange_min, config_camera.ep.val_ep_timerange_max);
        }

        if(config_camera.flags & CONFIGFLAG_3A_PICQUALITY) {
            ret = xcam_interface->set_noise_reduction_level(xcam, config_camera.pq.val_noise_reduction_level);
            ret = xcam_interface->set_temporal_noise_reduction_level(xcam, config_camera.pq.val_tnr_level);
            ret = xcam_interface->set_manual_brightness(xcam, config_camera.pq.val_pq_brightness);
            ret = xcam_interface->set_manual_contrast(xcam, config_camera.pq.val_pq_contrast);
            ret = xcam_interface->set_manual_hue(xcam, config_camera.pq.val_pq_hue);
            ret = xcam_interface->set_manual_saturation(xcam, config_camera.pq.val_pq_saturation);
            ret = xcam_interface->set_manual_sharpness(xcam, config_camera.pq.val_pq_sharpness);
        }

        if(config_camera.flags & CONFIGFLAG_3A_OTHERS) {
            if (config_camera.others.conf_gm_table)
                ret = xcam_interface->set_gamma_table(xcam, &config_camera.others.val_gm_table_r[0], &config_camera.others.val_gm_table_g[0], &config_camera.others.val_gm_table_b[0]);
            else
                ret = xcam_interface->set_gamma_table(xcam, NULL, NULL, NULL);
            ret = xcam_interface->set_gbce(xcam, config_camera.others.val_gm_gbce);
            ret = xcam_interface->set_night_mode(xcam, config_camera.others.val_night_mode);
        }
        //TODO: more settings

	xcam_interface->get_exposure_flicker_mode(xcam);
	xcam_interface->get_max_analog_gain(xcam);
	xcam_interface->get_current_analog_gain(xcam);
	xcam_interface->get_current_exposure_time(xcam);
    }

    if (impl->src_source.image_processor == CL_IMAGE_PROCESSOR) {
        ret = xcam_interface->set_hdr_mode(xcam, impl->src_source.cl_hdr_mode);
        ret = xcam_interface->set_denoise_mode(xcam, impl->src_source.cl_denoise_mode);
        ret = xcam_interface->set_gamma_mode(xcam, impl->src_source.cl_gamma_mode);
    }

    //FIXME:
    //Any resource to release here?/?

    return ret;
}

gboolean
media_pipe_set_src_size(MediaPipe *pipe, guint width, guint height)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting Src_size dynamically");
        return FALSE;
    }


    impl->src_source.width = width;
    impl->src_source.height = height;

    impl->main_source.width = width;
    impl->main_source.height = height;

    return TRUE;
}

gboolean
media_pipe_set_src_format (MediaPipe *pipe, GstVideoFormat format)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting Src_format dynamically");
        return FALSE;
    }

    impl->src_source.format = format;
    impl->main_source.format = format;

    return TRUE;
}

gboolean
media_pipe_set_src_frame_rate(MediaPipe *pipe, guint frame_rate)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting frame rate dynamically");
        return FALSE;
    }

    if (frame_rate != AVAILABLE_FRAME_RATE_1 && frame_rate != AVAILABLE_FRAME_RATE_2 && frame_rate != AVAILABLE_FRAME_RATE_3)
       frame_rate = AVAILABLE_FRAME_RATE_1;
    impl->src_source.fps_n = frame_rate;
    impl->main_source.fps_n = frame_rate;

    impl->src_source.fps_d = 1;
    impl->main_source.fps_d= 1;

    return TRUE;
}

gboolean
media_pipe_set_encoder_frame_rate(MediaPipe *pipe, VideoChannelIndex channel_num, guint frame_rate)
{
    MediaPipeImpl *impl;
    VideoChannel *channel;
    GstCaps *caps;

    if(pipe == NULL)
    {
        return FALSE;
    }

    if (frame_rate <=0 || frame_rate >= 255) {
	/*
	 * TODO: Due to MaxMBPS, there is LIMIT 255. But it's for 1080P.
	 * to be more accurate, need calculate different limits for other resolution.
	 */
	LOG_ERROR ("Invalid frame rate!\n");
	return FALSE;
    }

    impl = IMPL_CAST(pipe);
    channel = &impl->channel[channel_num];

    channel->videorate.fps_n = frame_rate;
    channel->videorate.fps_d = 1;
    
    if (channel->videorate.capsfilter) {
        caps = gst_caps_new_simple ("video/x-raw",
                "framerate", GST_TYPE_FRACTION, frame_rate, 1,
                NULL);
        g_object_set (G_OBJECT (channel->videorate.capsfilter), "caps", caps, NULL);
        gst_caps_unref (caps);
    }

    return TRUE;
}

gboolean
media_pipe_set_src_image_processor (MediaPipe *pipe, guint image_processor)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

#if 0    
    if (image_processor) {
        impl->src_source.image_processor =  CL_IMAGE_PROCESSOR;
        impl->src_source.analyzer = SIMPLE_ANALYZER;
    }
    else {
        impl->src_source.image_processor =  ISP_IMAGE_PROCESSOR;
        impl->src_source.analyzer = AIQ_ANALYZER;
    }
#else
    // force to ISP+AIQ until CL pipeline gets stable
    impl->src_source.image_processor =  ISP_IMAGE_PROCESSOR;
    impl->src_source.analyzer = AIQ_ANALYZER;
#endif
}

gboolean
media_pipe_set_src_smart_resolution (
                                     MediaPipe *pipe,
                                     SmartResolution resolution)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting smart resolution dynamically");
        return FALSE;
    }

    impl->src_preproc.smart_resolution = resolution;

    if (smart_factor > 0)
    {
       normal_resolution[VIDEO_CHANNEL_SMART].width =    smart_factor * 1920;
       normal_resolution[VIDEO_CHANNEL_SMART].height =   smart_factor * 1080;
       rotate_resolution[VIDEO_CHANNEL_SMART].width =    smart_factor * 1080;
       rotate_resolution[VIDEO_CHANNEL_SMART].height =   smart_factor * 1920;
    }
    else
    {
       switch(resolution)
       {
          case SMART_RES_176_100:
             normal_resolution[VIDEO_CHANNEL_SMART].width =    176;
             normal_resolution[VIDEO_CHANNEL_SMART].height =   100;
             rotate_resolution[VIDEO_CHANNEL_SMART].width =    100;
             rotate_resolution[VIDEO_CHANNEL_SMART].height =   176;
             break;
          case SMART_RES_352_198:
             normal_resolution[VIDEO_CHANNEL_SMART].width =    352;
             normal_resolution[VIDEO_CHANNEL_SMART].height =   198;
             rotate_resolution[VIDEO_CHANNEL_SMART].width =    198;
             rotate_resolution[VIDEO_CHANNEL_SMART].height =   352;
             break;
          case SMART_RES_480_270:
             normal_resolution[VIDEO_CHANNEL_SMART].width =    480;
             normal_resolution[VIDEO_CHANNEL_SMART].height =   270;
             rotate_resolution[VIDEO_CHANNEL_SMART].width =    270;
             rotate_resolution[VIDEO_CHANNEL_SMART].height =   480;
             break;
          case SMART_RES_352_200:
             normal_resolution[VIDEO_CHANNEL_SMART].width =    352;
             normal_resolution[VIDEO_CHANNEL_SMART].height =   200;
             rotate_resolution[VIDEO_CHANNEL_SMART].width =    200;
             rotate_resolution[VIDEO_CHANNEL_SMART].height =   352;
             break;
          case SMART_RES_480_272:
             normal_resolution[VIDEO_CHANNEL_SMART].width =    480;
             normal_resolution[VIDEO_CHANNEL_SMART].height =   272;
             rotate_resolution[VIDEO_CHANNEL_SMART].width =    272;
             rotate_resolution[VIDEO_CHANNEL_SMART].height =   480;
             break;
          case SMART_RES_CIF:
             normal_resolution[VIDEO_CHANNEL_SMART].width =    352;
             normal_resolution[VIDEO_CHANNEL_SMART].height =   288;
             rotate_resolution[VIDEO_CHANNEL_SMART].width =    288;
             rotate_resolution[VIDEO_CHANNEL_SMART].height =   352;
             break;
          case SMART_RES_D1:
             normal_resolution[VIDEO_CHANNEL_SMART].width =    704;
             normal_resolution[VIDEO_CHANNEL_SMART].height =   576;
             rotate_resolution[VIDEO_CHANNEL_SMART].width =    576;
             rotate_resolution[VIDEO_CHANNEL_SMART].height =   704;
             break;
          default:
             break;
       }
    }

    return TRUE;
}

gboolean
media_pipe_set_src_frame_smart_callback (
                            MediaPipe *pipe,
                            SmartFrameCallback smart_analyze_callback,
                            gpointer user_data)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting smart callback dynamically");
        return FALSE;
    }

    impl->src_preproc.smart_analyze_callback = smart_analyze_callback;
    impl->src_preproc.smart_user_data = user_data;

    return TRUE;
}

gboolean
media_pipe_set_video_preproc_rotation (
                    MediaPipe *pipe,
                    GstVideoPreprocRotateMode rotation_mode)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting rotation_mode dynamically");
        return FALSE;
    }

    impl->preproc.rotate_mode = rotation_mode;

    return TRUE;
}

gboolean
media_pipe_set_video_preproc_luma (
                          MediaPipe *pipe,
                          guint gain)

{
    MediaPipeImpl *impl;
    gboolean result = TRUE;

    if(pipe == NULL)
    {
        LOG_DEBUG("pipeline null");
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    impl->preproc.luma_gain= gain;

    if((pipe->pipe_running) &&
       impl->preproc.element!= NULL)
    {
        // update luminance gain
       result = gst_video_preproc_set_luma_gain (GST_VIDEO_PREPROC(impl->preproc.element),gain);
    }

    return result;
}

gboolean
media_pipe_set_channel_encoder_toggle(MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              gboolean enable)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting channel encoder dynamically");
        return FALSE;
    }

    impl->channel[channel].encoder.enable = enable;

    return TRUE;
}

gboolean
media_pipe_set_channel_encoder_rate_control (
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint rate_control)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support encoder rate_control dynamically");
        return FALSE;
    }

    impl->channel[channel].encoder.rate_control = rate_control;

    return TRUE;
}

gboolean
media_pipe_set_channel_encoder_bitrate (
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel_num,
                                              guint bitrate)
{
    MediaPipeImpl *impl;
    VideoChannel *channel;


    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);
    channel = &impl->channel[channel_num];

    if (channel->encoder.element) {
        g_object_set (channel->encoder.element,
                      "bitrate", bitrate,
                      NULL);
    }

    channel->encoder.bitrate = bitrate;

    return TRUE;
}

gboolean
media_pipe_set_channel_encoder_enable_cabac(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              gboolean enable)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support encoder cabac dynamically");
        return FALSE;
    }

    impl->channel[channel].encoder.enable_cabac = enable;

    return TRUE;
}

gboolean
media_pipe_set_channel_encoder_enable_dct8x8(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              gboolean enable)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support encoder cabac dynamically");
        return FALSE;
    }

    impl->channel[channel].encoder.enable_dct8x8 = enable;

    return TRUE;
}


gboolean
media_pipe_set_channel_encoder_mv (MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint mv)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting mv dynamically");
        return FALSE;
    }

    impl->channel[channel].encoder.mv = mv;

    return TRUE;
}

gboolean
media_pipe_set_channel_encoder_gop(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channelIdx,
                                              guint m,
                                              guint n)
{
    MediaPipeImpl *impl;
    VideoChannel *channel;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);
    channel = &impl->channel[channelIdx];

    channel->encoder.gop_M = m;
	channel->encoder.gop_N = n;

   	if(pipe->pipe_running) {
		if(!channel->enable)
			return FALSE;

	    if (channel->encoder.element) {
	        g_object_set (G_OBJECT(channel->encoder.element),
	                      "keyframe-period", m,
	                      NULL);
	    }
   	}

    return TRUE;
}

gboolean
media_pipe_set_channel_encoder_profile(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint profile)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support encoder profile dynamically");
        return FALSE;
    }

    impl->channel[channel].encoder.profile = profile;

    return TRUE;
}

gboolean
media_pipe_set_channel_sink_type(MediaPipe *pipe,
                                          VideoChannelIndex channel,
                                          MediaSinkType sink_type,
                                          gchar *host_ip,
                                          gint port)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting sink type dynamically");
        return FALSE;
    }

    impl->channel[channel].sink.type = sink_type;
    if (sink_type == TCP_SINK) {
       strcpy(impl->channel[channel].sink.host_ip, host_ip);
       impl->channel[channel].sink.port = port;
    }

    return TRUE;
}

gboolean
media_pipe_set_channel_encoder_frame_callback (MediaPipe *pipe,
                                                     VideoChannelIndex channel,
                                                     EncodeFrameCallback callback,
                                                     gpointer user_data)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting encoder_frame callback dynamically");
        return FALSE;
    }

    impl->channel[channel].encoder.callback = callback;
    impl->channel[channel].encoder.user_data = user_data;

    return TRUE;
}

gboolean
media_pipe_set_video_preproc_frame_jpeg_callback (
                            MediaPipe *pipe,
                            VideoFrameCallback jpeg_callback,
                            gpointer user_data)
{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(pipe->pipe_running)
    {
        LOG_ERROR("Pipeline is running, not support setting video_frame callback dynamically");
        return FALSE;
    }

    impl->preproc.jpeg_callback = jpeg_callback;
    impl->preproc.jpeg_user_data = user_data;

    return TRUE;
}

gboolean
media_pipe_set_channel_key_frame(
                                        MediaPipe *pipe,
                                        VideoChannelIndex channel)

{
    MediaPipeImpl *impl;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST (pipe);

    if(!pipe->pipe_running)
    {
        LOG_ERROR("Cannot set key frame when pipeline is not running");
        return FALSE;
    }

    if(impl->preproc.element == NULL)
    {
        LOG_ERROR("mpl->preproc hasn't been started");
        return FALSE;
    }

    if(impl->channel[channel].channel_on)
    {
        // currently just use GST_CLOCK_TIME_NONE for building force key frame event.
        GstEvent *ev = gst_video_event_new_downstream_force_key_unit(
                                                            GST_CLOCK_TIME_NONE,
                                                            GST_CLOCK_TIME_NONE,
                                                            GST_CLOCK_TIME_NONE,
                                                            TRUE,
                                                            1);

        GstPad *pad = gst_element_get_static_pad(
                                        impl->preproc.element,
                                        tempString[channel].pad_name
                                        );

        gst_pad_push_event(pad, ev);
        LOG_DEBUG ("Force-Key-Frame sent to channel %d", channel);

        gst_object_unref(pad);
    }
    return TRUE;
}

gboolean
media_pipe_enable_video_channel(
                            MediaPipe *pipe,
                            VideoChannelIndex channel_num)
{
    MediaPipeImpl *impl;
    gboolean result = TRUE;
    VideoChannel *channel;

    if(pipe == NULL)
    {
        return FALSE;
    }

    impl = IMPL_CAST(pipe);
    channel = &impl->channel[channel_num];

    channel->enable = TRUE;

    if(pipe->pipe_running)
    {
        GstPad *pad = gst_element_get_static_pad(impl->preproc.element, "sink");

        // !!! must be released if not using anymore !!!
        DynamicChannelData *dynamic_data =
                        (DynamicChannelData *)g_new0 (DynamicChannelData, 1);
        dynamic_data->media_pipe = pipe;
        dynamic_data->channel_num = channel_num;


        gst_pad_add_probe (pad,
                           GST_PAD_PROBE_TYPE_DATA_DOWNSTREAM,
                           (GstPadProbeCallback)dynamic_enable_channel_probe_callback,
                           dynamic_data,
                           NULL);
        gst_object_unref(pad);
    }


    return result;
}

gboolean
media_pipe_disable_video_channel(
                            MediaPipe *pipe,
                            VideoChannelIndex channel_num)
{
    MediaPipeImpl *impl;
    gboolean result = TRUE;
    VideoChannel *channel;

    if(pipe == NULL)
    {
        return FALSE;
    }

    if(channel_num == VIDEO_CHANNEL_1080P)
    {
        LOG_ERROR("Channel %d in impl cannot be disabled", channel_num);
        return FALSE;
    }

    impl = IMPL_CAST(pipe);
    channel = &impl->channel[channel_num];

    channel->enable = FALSE;


    if(pipe->pipe_running)
    {
        GstPad *pad = gst_element_get_static_pad(impl->preproc.element, "sink");

        // !!! must be released if not using anymore !!!
        DynamicChannelData *dynamic_data =
                        (DynamicChannelData *)g_new0 (DynamicChannelData, 1);
        dynamic_data->media_pipe = pipe;
        dynamic_data->channel_num = channel_num;


        gst_pad_add_probe (pad,
                           GST_PAD_PROBE_TYPE_DATA_DOWNSTREAM,
                           (GstPadProbeCallback)dynamic_disable_channel_probe_callback,
                           dynamic_data,
                           NULL);
        gst_object_unref(pad);
    }

    return result;
}

void
media_pipe_set_message_callback (MediaPipe *pipe, MessageCallback callback, gpointer user_data)
{
    MediaPipeImpl *impl = IMPL_CAST (pipe);

    impl->main_msg_callback = callback;
    impl->main_msg_user_data = user_data;
}

gboolean
media_pipe_start (MediaPipe *pipe, gboolean reconfig_3a)
{
    gboolean ret;
    MediaPipeImpl *impl = IMPL_CAST (pipe);

    ret = build_pipeline (pipe);
    if (!ret) {
        LOG_ERROR ("build pipeline failed");
        return FALSE;
    }

    gst_element_set_state (impl->src_pipeline, GST_STATE_PLAYING);
    gst_element_set_state (impl->main_pipeline, GST_STATE_PLAYING);
    pipe->pipe_running = TRUE;

    if (reconfig_3a) {
	    if(!media_pipe_reconfig_3a(pipe)) {
		    LOG_WARNING ("Reconfigure 3A for preproc FAIL!");
	    } else {
		    LOG_DEBUG ("Reconfigure 3A for preproc SUCCEED!");
	    }
    }

    g_main_loop_run(impl->main_loop);

    // Running...

    // Stopped
    g_main_loop_unref (impl->main_loop);

    return TRUE;
}

void
media_pipe_stop (MediaPipe *pipe)
{
    MediaPipeImpl *impl = IMPL_CAST (pipe);

    gst_pad_push_event(GST_BASE_SRC_PAD(impl->src_source.gen_src), gst_event_new_eos());

}

static void
_smart_meta_mask_info_free(GstVideoPreprocMaskInfo *info)
{
   if (info) {
      g_free(info);
   }
}

static void
_smart_meta_wf_info_free(GstVideoPreprocWireFrameInfo *info)
{
   if (info) {
      g_free(info);
   }
}

static void
_smart_meta_osd_info_free(GstVideoPreprocOsdInfo *info)
{
   if (info) {
      if (info->image)
         gst_vaapi_object_unref(info->image);
      g_free(info);
   }
}

gboolean
media_pipe_set_cl_feature (MediaPipe *pipe, CLFeature feature, int mode)
{
    MediaPipeImpl           *impl;
    GstElement              *v4l2src;
    GstXCam3A               *xcam;
    GstXCam3AInterface      *xcam_interface;
    gboolean ret = TRUE;

    if(pipe == NULL)
    {
        LOG_ERROR("Invalid mediapipe.");
        return FALSE;
    }

    impl = IMPL_CAST(pipe);

    if(!impl->pipe.pipe_running)
    {
        LOG_DEBUG("Save cl feature settings before pipe running");
        switch (feature) {
        case CL_HDR:
            impl->src_source.cl_hdr_mode = mode;
            break;
        case CL_DENOISE:
            impl->src_source.cl_denoise_mode = mode;
            break;
        case CL_GAMMA:
            impl->src_source.cl_gamma_mode = mode;
            break;
        default:
            LOG_ERROR("%s Unsupported CL feature\n", __func__);
           break;
        }
        return TRUE;
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

    xcam = GST_XCAM_3A (v4l2src);
    xcam_interface = GST_XCAM_3A_GET_INTERFACE (xcam);
    if(xcam_interface == NULL)
    {
        LOG_ERROR("Failed to get xcam interface.");
        return FALSE;
    }
      
    switch (feature) {
    case CL_HDR:
        ret = xcam_interface->set_hdr_mode(xcam, mode);
        break;
    case CL_DENOISE:
        ret = xcam_interface->set_denoise_mode(xcam, mode);
        break;
    case CL_GAMMA:
        ret = xcam_interface->set_gamma_mode(xcam, mode);
        break;
    default:
        LOG_ERROR("%s Unsupported CL feature\n", __func__);
        break;
    }
    
    return ret;
}
