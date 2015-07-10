/*
 * mediapipe_impl.h - Media pipe APIs
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

#ifndef MEDIA_PIPE_IMPL_H
#define MEDIA_PIPE_IMPL_H

#include <mediapipe.h>

#include <gst/rtsp-server/rtsp-server.h>

typedef struct _MediaSource{
    guint           width;
    guint           height;
    GstVideoFormat  format;
    guint           fps_n;
    guint           fps_d;
    
    // source pipeline members
    SrcType         src_type;
    gchar           *location;
    gboolean        use_v4l2_src;
    guint           v4l2_src_sensor_id;
    guint           v4l2_src_io_mode;
    gchar           *v4l2_src_device;
    gboolean        v4l2_enable_3a;
    guint           v4l2_capture_mode;
    
    ImageProcessorType image_processor;
    AnalyzerType    analyzer;
    guint           cl_hdr_mode;
    guint           cl_denoise_mode;
    guint           cl_gamma_mode;

    GstElement      *gen_src;

    GstElement      *src_filter;
    GstElement      *src_queue;

    // appsrc members in main pipeline
    GstElement      *app_src;
    GstElement      *src_tee;
}MediaSource;

typedef struct _MediaPreProc{
    GstVideoPreprocRotateMode   rotate_mode;

    gboolean                    enable_wireframe;
    gboolean                    enable_mask;
    GstVideoPreprocMaskCfg      mask_cfg[MASK_REGION_MAX_NUM];

    guint                  luma_gain;//percentage of original luminance of each pixel

    VideoFrameCallback          jpeg_callback;
    gpointer                    jpeg_user_data;
    gulong                      jpeg_frame_probe_id;
    
    SmartResolution             smart_resolution;
    gulong                      smart_frame_1080p_probe_id;
    gulong                      smart_frame_smart_probe_id;
    
    SmartFrameCallback          smart_analyze_callback;
    gpointer                    smart_user_data;
    
    GMutex                      smart_lock;
    GList                       *smart_queue;                   // contain SmartData *
    GList                       *smart_1080p_queue;             // contain SmartFrameData *

    Parse3AConfCallback         parse_3aconf_callback;
    gpointer                    parse_3aconf_user_data;

    gboolean        vpp_enable_autohdr;
    GstVideoPreprocAutoHDRMode vpp_autohdr_mode;
    GstElement                  *element;
}MediaPreProc;



typedef struct _MediaEncoder{
    gboolean        enable;

    guint           rate_control;
    guint           bitrate;  // kpbs
    gboolean        enable_cabac;
    gboolean        enable_dct8x8;
    guint           mv;
    guint           gop_M; // default 30
    guint           gop_N; // default 0
    guint           profile;

    GstElement      *element;

    /* frame callback */
    gulong          frame_probe_id;
    EncodeFrameCallback   callback;
    gpointer        user_data;
}MediaEncoder;

typedef struct _MediaVideorate {
    GstElement      *element;
    GstElement      *capsfilter;
    guint           fps_n;
    guint           fps_d;
}MediaVideorate;

typedef struct _MediaSink{
    MediaSinkType   type;

    union  
    {
        gchar       *location;
        gulong      param_type_reserve1;
        gulong      param_type_reserve2;
    }u;

    GstElement      *element;
    //for tcpclientsink
    gchar           host_ip[128];
    gint            port;

    //for rtspsink
    GstRTSPServer	*server;
    GstRTSPMediaFactory *factory;
    gint		source_id;
    GstElement	        *rtsp_src;
    GstClockTime        timestamp;
}MediaSink;


typedef struct _VideoChannel
{
    guint           channel_index;
    gboolean        enable;

    gboolean        channel_on;
    GstElement      *encoder_queue;
    GstElement      *jpeg_queue;
    MediaEncoder    encoder;
    MediaVideorate  videorate;
    GstElement      *profile_converter;

    gboolean                    enable_osd;
    VideoFrameCallback          osd_callback;
    gpointer                    osd_user_data;
    GstVideoPreprocBuffer       *osd_buffer;
    
    MediaSink       sink;
    MediaSink       jpeg_sink;
}VideoChannel;

typedef struct {
    MediaPipe       pipe;
    
    /* private */   

    // source thread pipeline
    GstElement      *src_pipeline;

    MediaSource     src_source;
    MediaPreProc    src_preproc;
    GstElement      *src_queue[VIDEO_CHANNEL_MAX];
    MediaSink       src_fakesink[VIDEO_CHANNEL_MAX];

    guint           src_bus_watch;
    MessageCallback src_msg_callback;
    gpointer        src_msg_user_data;


    // main thread pipeline
    GstElement      *main_pipeline;
    GMainLoop       *main_loop;
    
    MediaSource     main_source;    

    MediaPreProc    preproc;
    VideoChannel    channel[VIDEO_CHANNEL_MAX];
    
    guint           main_bus_watch;
    MessageCallback main_msg_callback;
    gpointer        main_msg_user_data;
    
    GstRTSPServer	*rtsp_server;
} MediaPipeImpl;

#define IMPL_CAST(obj)  \
    ((MediaPipeImpl*) (obj))

typedef struct _DynamicChannelData
{
    MediaPipe                  *media_pipe;
    VideoChannelIndex           channel_num;
}DynamicChannelData;


typedef struct {
    gulong signal;
    GstElement *src;
    GstElement *sink;
} PostLinkInfo;

//helper
gboolean dump_nv12(GstVideoPreprocBuffer *buf, char *filename);
gboolean dump_nv12_timed(GstVideoPreprocBuffer *buf, char *prefix);
#endif //MEDIA_PIPE_IMPL_H

