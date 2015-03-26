/*!
 * @file
 */

/*
 * mediapipe.h - Media pipe APIs
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

/*!
 * @mainpage
 *
 * Media Pipe is a reusable component for embedded systems that leverages
 * open-source technologies like GStreamer and Open CL to control
 * and accelerate the video capture, pre-processing, encoding, recording and network streaming.
 * Additionally it also supports smart analysis for the captured video such as face detection and recognition.
 * Currently, Media Pipe is designed to have the following features: \n
 *
 * \li Video capture through v4l2src
 * \li Video rotation with Open CL acceleration
 * \li Mask, OSD and wireframe adding with Open CL acceleration
 * \li Video recording encoded as H264 with Intel LIBVA acceleration
 * \li Video Snapshot with Open CL based JPEG image compression
 * \li Local video preview
 *
 * Media Pipe is designed to support multiple video flows, but currently only enable one main flow.
 * One video flow could have multiple video channels with different resolutions or different usages.
 * Except 1080P channel, you’re able to enable or disable any of these channels.
 * And you could also choose where the data of these channels should go. Such as saving as a local file,
 * streaming over network, etc.
 *
 * Media Pipe APIs could be divided into several parts which allows you to control lots of settings
 * from front camera to the last sink for each channel.
 * \n
 * By calling below APIs, you could control the Media Pipe itself. \n
 *
 * \li media_pipe_create
 * \li	media_pipe_destroy
 * \li	media_pipe_start
 * \li	media_pipe_stop
 *
 * To control the camera behavior, you could use the following APIs after choose v4l2src as the source
 * by media_pipe_set_use_v4l2_src \n
 *
 * \li media_pipe_set_v4l2_src_sensor_id
 * \li	media_pipe_set_v4l2_src_io_mode
 * \li media_pipe_set_src_size
 * \li	media_pipe_set_src_format
 *
 * As above figure shows, all pre-processing including OSD, rotation, mask, wireframe will be applied
 * to the original 1080P channel. Which means all other channels of the main video flow would have these settings. \n
 * \n
 * You are able to change the encoding settings such as bit-rate control method, CABAC enabling,
 * the GOP and H264 profile for every channel separately with below APIs:
 *
 * \li media_pipe_set_channel_encoder_rate_control
 * \li	media_pipe_set_channel_encoder_bitrate
 * \li	media_pipe_set_channel_encoder_enable_cabac
 * \li	media_pipe_set_channel_encoder_enable_dct8x8
 * \li media_pipe_set_channel_encoder_gop
 * \li media_pipe_set_channel_encoder_profile
 */


#ifndef MEDIA_PIPE_H
#define MEDIA_PIPE_H

#include <gst/gst.h>
#include <gst/video/video.h>
#include <gst/base/base.h>
#include <stdio.h>

#include <gst/vaapi/videopreproc.h>
#include "gst/gstxcaminterface.h"

#define OSD_REFINE 
#define MASK_REGION_MAX_NUM 6
#define OSD_REGION_MAX_NUM 6
#define WIRE_FRAME_REGION_MAX_NUM 6


#define COLOR_KEY_Y 169

#define VIDEO_FORMAT_NV12 23

#define DMA_MODE 4
#define MMAP_MODE 2


#define VA_CQP 1
#define VA_CBR 2
#define VA_VBR 4
#define VA_VBR_CONSTRAINED 5


typedef struct _MediaPipe MediaPipe;

/*!
 * Source element type
 */
typedef enum _SrcType
{
   SRC_TYPE_VIDEO_TEST, /*!< video will be from videotestsrc, for debug */
   SRC_TYPE_V4L2, /*!< video from camera, normal usage */
   SRC_TYPE_FILE, /*!< video from file system, for demo or debug */
   SRC_NUM
}SrcType;

typedef enum _MediaSinkType
{
    FAKE_SINK = 0,
    FILE_SINK,
    TCP_SINK,
    KMS_SINK,
    RTSP_SINK,
    V4L2_SINK,
    UDP_SINK,
    SINK_TYPE_RESERVE_1,
    SINK_TYPE_RESERVE_2,
    SINK_TYPE_RESERVE_3,
    SINK_MAX
}MediaSinkType;

typedef enum _VideoChannelIndex
{
    VIDEO_CHANNEL_1080P = 0,
    VIDEO_CHANNEL_JPEG,
    VIDEO_CHANNEL_720P,
    VIDEO_CHANNEL_D1,
    VIDEO_CHANNEL_CIF,
    VIDEO_CHANNEL_480_270,
    VIDEO_CHANNEL_352_198,
    VIDEO_CHANNEL_480_272,
    VIDEO_CHANNEL_352_200,
    VIDEO_CHANNEL_SMART,
    VIDEO_CHANNEL_MAX
}VideoChannelIndex;

typedef enum _SmartResolution
{
    SMART_RES_176_100 = 0,
    SMART_RES_352_198,
    SMART_RES_480_270,
    SMART_RES_352_200,
    SMART_RES_480_272,
    SMART_RES_CIF,
    SMART_RES_D1
}SmartResolution;

typedef struct{
   GMutex lock;
   gint   need_resource;
   gint   need_resource_urgent;
}Qos;


typedef gboolean (*EncodeFrameCallback)   (GstBuffer *encode_buf, gpointer user_data, VideoChannelIndex channel);
typedef gboolean (*VideoFrameCallback)   (GstVideoPreprocBuffer *video_buf, gpointer user_data, VideoChannelIndex channel);
typedef gboolean (*SmartFrameCallback)   (GList *smart_queue, GList *smart_1080p_queue, gpointer user_data);

typedef gboolean (*MessageCallback)   (GstMessage *mesg, gpointer user_data);

/*!
 * MediaPipe public context
 */
struct _MediaPipe {
    guint       size; /*!< the size of MediaPipe private context */
    gboolean    pipe_running; /*!< MediaPipe running flag */
};

typedef struct _SmartData
{
    GstBuffer   *buf;
    GstVideoPreprocBuffer *preBuf;
}SmartData;

typedef struct _Smart1080pData
{
    GstBuffer   *buf;
    gboolean    can_push;
}Smart1080pData;

/*3a config*/
typedef enum _Camera3AReconfigFlags
{
    CONFIGFLAG_3A_WHITEBALANCE 	= 0x00000001,
    CONFIGFLAG_3A_EXPOSURE 		= 0x00000002,
    CONFIGFLAG_3A_PICQUALITY 	= 0x00000004,
    CONFIGFLAG_3A_OTHERS        = 0x00000008,
}Camera3AReconfigFlags;

typedef struct _Cameara3a_WhiteBalance
{
	//mode
	gint		val_wb_mode;

	//speed
	gdouble		val_awb_speed;

	//color temperature
	guint 		val_awb_cct_min;
	guint	 	val_awb_cct_max;

	//mode
	gdouble		val_awb_gr;	
	gdouble		val_awb_r;	
	gdouble		val_awb_gb;	
	gdouble		val_awb_b;	
}Cameara3a_WhiteBalance;

typedef struct _Cameara3a_Exposure
{
	//mode
	gint		val_ep_mode;

	//meter mode
	gint		val_meter_mode;	

	//expusre window
	XCam3AWindow	val_ep_window;	

	//exposure offset
	gdouble		val_ep_offset;

    //exposure speed
	gdouble		val_ep_speed;

	//exposure-flicker-mode
	gint		val_ep_flicker;	

	//manual-exposure-time
	gint64		val_ep_manual_time;		

	//manual-analog-gain
	gdouble		val_ep_manual_analoggain;

	//max-analog-gain
	gdouble		val_ep_max_analoggain;	

	//exposure-time-range
	gint64 		val_ep_timerange_min;
	gint64	 	val_ep_timerange_max;	

}Cameara3a_Exposure;

typedef struct _Cameara3a_PicQuality
{
	guint8      val_noise_reduction_level;
    guint8      val_tnr_level;
	guint8		val_pq_brightness;
	guint8		val_pq_contrast;
	guint8		val_pq_hue;
	guint8		val_pq_saturation;	
	guint8		val_pq_sharpness;
}Cameara3a_PicQuality;


#define	GAMMATABLESIZE	257
typedef struct _Cameara3a_Others
{
	//gamma table
	gboolean	conf_gm_table;
	double		val_gm_table_r[GAMMATABLESIZE];
	double		val_gm_table_g[GAMMATABLESIZE];
	double		val_gm_table_b[GAMMATABLESIZE];	

	//gbce
	gboolean	val_gm_gbce;
    gboolean    val_night_mode;
}Cameara3a_Others;

typedef struct _3a_Config
{
	/*white-balance*/
	gint				            flags;
	
    Cameara3a_WhiteBalance		 	 wb;

	/*exposure*/
    Cameara3a_Exposure		 		 ep;

	/*manual picture quality*/
	Cameara3a_PicQuality		 	 pq;

	/*others */
    Cameara3a_Others		 		 others;
}Config3A;

typedef gboolean (*Parse3AConfCallback)   (Config3A *ptrConf, gpointer user_data);

/*!
 * Initialize the MeidaPipe and Gstreamer library. All Gstreamer commandline options are supported.
 * It enables 1080P channel of VIDEO_MAIN_FLOW_INSTANCE by default.
 *
 * @param argc	application's argc
 * @param argv	application's argv
 *
 * @return a pointer to MediaPipe context, assert if memory couldn't be allocated.
 */
MediaPipe*
media_pipe_create (int argc,  char *agrv[]);

/*!
 * Clean up any resources created by media_pipe_create, destroy internal Gstreamer pipeline.
 *
 * @param pipe	MeidaPipe context allocated by media_pipe_create
 *
 */
void
media_pipe_destroy (MediaPipe *pipe);

// config scope: pipeline

/*!
 * Choose video source.
 *
 * @param pipe	MediaPipe context
 * @param src_type	Three sources could be selected
 * 				\li \c SRC_TYPE_VIDEO_TEST	video from 'videotestsrc' element, for debug only
 * 											when there is no camera avaliable.
 *				\li \c SRC_TYPE_V4L2	video from camera, normal usage
 * 				\li \c SRC_TYPE_FILE	video from H264 raw file, for debug
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_filesrc_type (MediaPipe *pipe, SrcType src_type);

/*!
 * Specify the file location when choose SRC_TYPE_FILE as video source
 *
 * @param pipe	MediaPipe context
 * @param location	H264 raw file path
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_filesrc_location (MediaPipe *pipe, gchar *location);

/*!
 * Enable v4l2src element or not. It's disabled by default. If you need to capture data from camera, 
 * set this before media_pipe_start.
 * Otherwise MediaPipe will take videotestsrc as source for debugging.
 *
 * @param pipe	MediaPipe context
 * @param use_v4l2_src	enable v4l2src element or not
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_use_v4l2_src (MediaPipe *pipe, gboolean use_v4l2_src);

/*!
 * Choose the type of sensors. It largely depends on the your target device.
 *
 * @param pipe	MediaPipe context
 * @param v4l2_src_sensor_id	sensor id
 *					\li \c 0	ov5640_1
 *					\li \c 1	aptina
 *					\li \c 2	ov5640_2
 *					\li \c 3	imx185
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 *
 * @note Get more details by running 'gst-inspect-1.0 v4l2src' on target, check 'sensor-id' property.
 */
gboolean
media_pipe_set_v4l2_src_sensor_id (MediaPipe *pipe, guint v4l2_src_sensor_id);

/*!
 * Choose the video device. 
 *
 * @param pipe	MediaPipe context
 * @param v4l2_src_device device used by v4l2src
 *
 * @return FALSE if failed, otherwize TRUE.
 *
 * @note Get more details by running 'gst-inspect-1.0 v4l2src' on target, check 'device' property.
 */
gboolean
media_pipe_set_v4l2_src_device(MediaPipe *pipe, gchar *v4l2_src_device);


/*!
 * Set IO mode for v4l2src element. Generally v4l2src supports four types of IO mode: RW, MMAP, USERPTR, DMABUF.
 * But not every v4l2 driver supports all of them. For better performance, MediaPipe chooses DMABUF by default for all supported devices.
 *
 * @param pipe	MediaPipe context
 * @param v4l2_src_io_mode	v4l2src IO mode
 *					\li \c 2	Memory Map mode
 * 					\li \c 4	DMABUF mode
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 *
 */
gboolean
media_pipe_set_v4l2_src_io_mode (MediaPipe *pipe, guint v4l2_src_io_mode);

/*!
 * Enable 3A for v4l2src element, disabled by default. Don't support eanble after MediaPipe is already running.
 *
 * @param[in] pipe	MediaPipe context
 * @param[in] enable_3a	true for enable
 * 
 * @return	FALSE indicates errors.
 */
gboolean
media_pipe_set_v4l2_src_enable_3a (MediaPipe *pipe, gboolean enable_3a);

/*!
 * Choose capture mode for v4l2src element. Currently only support video mode.
 * 
 * @param[in] pipe	MediaPipe context
 * @param[in] capture_mode	
 *							\li \c 0	still mode
 *							\li \c 1	video mode
 * 							\li \c 2	preview mode	
 *
 * @return	FALSE if MediaPipe is already running, otherwize TRUE.	 
 */
gboolean
media_pipe_set_v4l2_src_capture_mode (MediaPipe *pipe, guint capture_mode);

/*!
 * Set resolution for the source element which could be v4l2src or videotestsrc. Default setting is 1920x1080.
 * Unsupported resolution won't cause errors in this function immediately. It will be reported in media_pipe_start.
 *
 * @param pipe	MediaPipe context
 * @param width	source width in pixel
 * @param height source height in pixel
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_src_size (MediaPipe *pipe, guint width, guint height);

/*!
 * Set callback function for 3A configuration. In the callback, you should fill Config3A structure with all or 
 * part of 3A configuration including white balance, exposure, denoise, gamma, picture quality settings.
 * 
 * This callback will be called in media_pipe_reconfig_3a.
 * @param[in] pipe	MediaPipe context
 * @param[in] parse_3aconf_callback	callback function pointer
 * @param[in] user_data	customized data passed to the callback
 *
 * @return	FALSE if MediaPipe is already running or other errors, otherwize TRUE. 
 */
gboolean
media_pipe_set_src_parse_3aconf_callback (
							MediaPipe *pipe, 
							Parse3AConfCallback parse_3aconf_callback, 
							gpointer user_data);

/*!
 * Reconfigure 3A settings. It will call callback function set by media_pipe_set_parse_3aconf_callback to get 
 * new settings, then do reconfiguration. It can only be called when MediaPipe is running and 3A is enabled by
 * media_pipe_set_v4l2_src_enable_3a.
 *
 * @param[in] pipe	MediaPipe context
 *
 * @return FALSE indicates errors. Normally 3A is disabled, Parse3AConfCallback returns errors would lead to errors.
 */
gboolean
media_pipe_reconfig_3a (MediaPipe *pipe);

/*!
 * Set color format for source element. Refer to GstVideoFormat manual for details.
 * You can't choose any format as you like, it largely depends on the sensor and its driver.
 * Default setting is NV12.
 *
 * @param pipe	MediaPipe context
 * @param format	video color format, refer to GstVideoFormat manual
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_src_format (MediaPipe *pipe, GstVideoFormat format);

/*!
 * Set global frame rate for MediaPipe. It's 30fps by default.
 * Example: set frame_rate to 30 to get 30fps.
 * 
 * @param[in] pipe MediaPipe context
 * @param[in] frame_rate	numerator of the frame rate, denominator is always 1	 
 * 
 * @return FALSE if MediaPipe is running, otherwize TRUE. 
 */
gboolean
media_pipe_set_src_frame_rate(MediaPipe *pipe, guint frame_rate);

/*!
 * Choose a resolution for all video flows to do the the smart video analystic. 
 * For performance issue, we only support analystic for smaller resolution below D1 
 * instead of the full HD video.
 * 
 * @param[in] pipe MediaPipe context
 * @param[in] resolution  smart resolution value, refer to SmartResolution
 * 
 * @return	FALSE if MediaPipe is already running, otherwize TRUE.
 */ 
gboolean
media_pipe_set_src_smart_resolution (
                                     MediaPipe *pipe, 
                                     SmartResolution resolution);

/*!
 * Set callback function for smart analysis.
 *
 * 		gboolean (*SmartFrameCallback)(GList *smart_queue, 
 *									GList *smart_1080p_queue, 
 *									gpointer user_data)
 *
 * You can get video frames from smart_queue for analysis. Every buffer is stored as a SmartData.
 * Detailed buffer information could be found from SmartData.preBuf such as frame width, height, buffer 
 * address etc.
 * 
 * smart_1080p_queue is a queue of Smart1080pData, you could design customized policy to push 1080p frame
 * down by setting Smart1080pData->can_push as TRUE.
 *
 * @param[in] pipe	MediaPipe context
 * @param[in] smart_analyze_callback callback function pointer
 * @param[in] user_data	user data pointer
 * 
 * @return FALSE if MediaPipe is already running, otherwize TRUE. 
 */
gboolean
media_pipe_set_src_frame_smart_callback (
                                     MediaPipe *pipe, 
                                     SmartFrameCallback smart_analyze_callback,
                                     gpointer user_data);

// config scope: flow

/*!
 * Set rotation mode for one video flow. It will be applied to all channels of this video flow.
 *
 * @param pipe	MediaPipe context
 * @param rotation_mode		refer to GstVideoPreprocRotateMode
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_video_preproc_rotation (MediaPipe *pipe,
                                        GstVideoPreprocRotateMode rotation_mode);

/*!
 * Set luminance for a specific video flow. 
 *
 * @param[in] pipe MediaPipe context
 * @param[in] gain	percentage of orignal luminance. 100 means 100%
 * 
 * @return FALSE indicates errors. 
 */
gboolean
media_pipe_set_video_preproc_luma (MediaPipe *pipe,
                              guint gain);

/*!
 * Set callback functions for specific video flow to do the JPEG encoding.
 *
 * 		gboolean (*VideoFrameCallback)(GstVideoPreprocBuffer *video_buf, 
 * 										gpointer user_data, 
 * 										VideoChannelIndex channel);
 *
 * 		\li \c video_buf	contains information required for JPEG encoding such as frame resolution, buffer address etc.
 *		\li \c user_data	user's own data
 *		\li \c channel	always VIDEO_CHANNEL_1080P
 *
 * Video frame here is copy from 1080P channel. Normally, try libjpeg-cl to do the encoding with hardware acceleration.
 *
 * @param[in] pipe MediaPipe context
 * @param[in] jpeg_callback	callback function pointer
 * @param[in] user_data	user's data pointer
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.  
 */
gboolean
media_pipe_set_video_preproc_frame_jpeg_callback (
                                     MediaPipe *pipe, 
                                     VideoFrameCallback jpeg_callback, 
                                     gpointer user_data);

// config scope: channel

/*!
 * Enable or disable H264 encoding for one channel. It's enabled by default.
 * 
 * @param[in] pipe	MediaPipe context
 * @param[in] channel	channel number 
 * @param[in] enable	enable flag
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE. 
 */
gboolean
media_pipe_set_channel_encoder_toggle(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              gboolean enable);
/*!
 * Choose the rate control method for internal H264 encoder.
 * 
 * @param[in] pipe	MediaPipe context
 * @param[in] channel	channel number
 * @param[in] rate_control rate control method, discrete value can be choosen
 *							\li \c 1	constant QP (default) ?
 * 							\li \c 2	CBR
 *							\li \c 4	VBR
 *							\li \c 5	VBR constrained
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_channel_encoder_rate_control (
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint rate_control);

/*!
 * Set bit rate of internal H264 encoder for one channel. (Default value?)
 * 
 * @param[in] pipe MediaPipe context
 * @param[in] channel	video channel number
 * @param[in] bitrate	bitrate in kbps. Range: 0~102400
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_channel_encoder_bitrate (
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint bitrate);

/*!
 * Enable or disable CABAC for internal H264 encoder. It's disabled by default.
 *
 * @param[in] pipe MediaPipe context
 * @param[in] channel	video channel number
 * @param[in] enable	enable flag
 * 
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_channel_encoder_enable_cabac(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              gboolean enable);

/*!
 * Enable or disable adaptive use of 8x8 transforms in I-frames for internal H264 encoder. It's disabled by default.
 *
 * @param[in] pipe MediaPipe context
 * @param[in] channel	video channel number
 * @param[in] enable	enable flag
 * 
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_channel_encoder_enable_dct8x8(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              gboolean enable);


/*!
 * Enable or disable motion vector dump for a specific video channel. (TBD: By enabling mv, you could observe it on the encoded video)
 * Except the 1080P channel, motion vector for other channels is disabled by default. There are 2 types motion vectors to enable: 16x16 block, 4x4 block.
 *
 * @param[in] pipe	MediaPipe context
 * @param[in] channel 	channel number
 * @param[in] mv	mv mode
 *					\li \c 0	disable
 *					\li \c 1	enable with 16x16 mv block
 *					\li \c 2	enable with 4x4 mv block
 *
 * @return FALSE if MediaPipe already running, otherwize TRUE.
 */
gboolean
media_pipe_set_channel_encoder_mv (
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint mv);
/*!
 * Set key frames period and maximal B-frames between I and P of H264 encoder for one channel.
 * 
 * @param[in] pipe MediaPipe context
 * @param[in] channel	channel number
 * @param[in] m	key frames period
 *			\li Range: 0-300
 *			\li Default: 0 for auto-calculate
 * @param[in] n maximal B-frames between I and P
 *			\li Range: 0-10
 *			\li Default: 0
 * 
 * @return FALSE if MediaPipe already running, otherwize TRUE
 */
gboolean
media_pipe_set_channel_encoder_gop(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint m,
                                              guint n);

/*!
 * Choose H264 encoder profile for one channel. (TBD: capsfilter can do?)
 * 
 * @param[in] pipe MediaPipe context
 * @param[in] channel	channel number
 * @param[in] profile	H264 profile
 * 						\li \c 0	baseline(default)
 *						\li \c 1	main
 *						\li \c 2	high
 *  
 * @return FALSE if MediaPipe already running, otherwize TRUE
 */
gboolean
media_pipe_set_channel_encoder_profile(
                                              MediaPipe *pipe,
                                              VideoChannelIndex channel,
                                              guint profile);

/*!
 * Enable OSD on a specified video flow.
 *
 * @param pipe	MediaPipe context
 * @param channel	video channel number
 * @param enable	enable or disable flag
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 *
 */
gboolean
media_pipe_set_channel_enable_osd (
                                     MediaPipe *pipe, 
                                     VideoChannelIndex channel,
                                     gboolean enable);

/*!
 * TBD: removed?
 *
 * Set callback function for one channel to do OSD (add content into video).
 *
 * 		gboolean (*VideoFrameCallback)(GstVideoPreprocBuffer *video_buf, 
 * 										gpointer user_data, 
 * 										VideoChannelIndex channel);
 *
 * 		\li \c video_buf	information required for YUV video frame such as resolution, buffer address etc.
 *		\li \c user_data	user's own data
 *		\li \c channel	video channel number	
 *
 * @param[in] pipe MediaPipe context
 * @param[in] channel	channel number
 * @param[in] osd_callback	callback function pointer
 * @param[in] user_data	user's data pointer
 *
 * @return 
 * 
 */                                     
gboolean
media_pipe_set_channel_osd_callback (
                                     MediaPipe *pipe, 
                                     VideoChannelIndex channel,
                                     VideoFrameCallback osd_callback, 
                                     gpointer user_data);

/*!
 * TBD(new implementation)
 *
 * Choose sink type for a channel. It determines where the encoded data goes. Except FILE_SINK, 
 * all other sink types will be treated as FAKE_SINK internally.
 * Type FILE_SINK will save the encoded data of this channel as a file in the same directory 
 * as the application calling the MediePipe. \n
 * The file name's format is 'Flow_0_width x height.264'. Type FAKE_SINK will simply drop the encoded data. 
 * But you could get the encoded data from the callback function of media_pipe_set_channel_encoder_frame_callback.
 *
 * @param pipe	MediaPipe context
 * @param channel	channel number
 * @param sink_type	basically only FILE_SINK and FAKE_SINK matters
 * @param host_ip
 * @param port
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_channel_sink_type(MediaPipe *pipe,
                                          VideoChannelIndex channel,
                                          MediaSinkType sink_type, gchar *host_ip, gint port);

/*!
 * Set callback to get encoded frame of a channel. 
 * (TBD: Only callback for 1080P channel works in current implementation)
 *
 * @param pipe	MediaPipe context
 * @param channel	channel number
 * @param callback	callback function
 * @param user_data		user data to be passed to callback function
 *
 * @return FALSE if MediaPipe is already running, otherwize TRUE.
 */
gboolean
media_pipe_set_channel_encoder_frame_callback (
                                                     MediaPipe *pipe, 
                                                     VideoChannelIndex channel,
                                                     EncodeFrameCallback callback, 
                                                     gpointer user_data);
/*!
 * TBD: how to sync?
 * 
 * Trigger a key frame event for a specific channel. Must be called when MediaPipe is running 
 * and specified channel is on.
 *
 * @param pipe	MediaPipe context
 * @param channel	channel number
 *
 * @return FALSE if channel is off or MediaPipe isn't running, otherwize TRUE 
 */
gboolean
media_pipe_set_channel_key_frame(MediaPipe *pipe,
                                              VideoChannelIndex channel);

/*!
 * Enable a channel for a video flow. Need to be called before media_pipe_start.
 * The 1080P channel @ main video flow is enabled by default.
 *
 * @param pipe	MediaPipe context
 * @param channel 	channel number
 *
 *
 * @return TRUE if setting is successfully. FALSE if not.
 */
gboolean
media_pipe_enable_video_channel(
                                        MediaPipe *pipe, 
                                        VideoChannelIndex channel);

/*!
 * Disable a channel for a video flow. The 1080P channel @ main video flow could not be disabled.
 * Need to be called before media_pipe_start too.
 *
 * @param pipe	MediaPipe context
 * @param channel	channel number
 *
 * @return TRUE if setting is successfully. FALSE if not.
 */
gboolean
media_pipe_disable_video_channel(
                                        MediaPipe *pipe,
                                        VideoChannelIndex channel);


gboolean
media_pipe_set_vpp_src_enable_autohdr (MediaPipe *pipe, gboolean enable_autohdr);

gboolean
media_pipe_set_vpp_src_set_autohdrmode (MediaPipe *pipe, guint autohdr_mode);


/*!
 * Specify callback to receive message from internal gstreamer bus including EOS, errors etc.
 *
 * @param[in] pipe MediaPipe context
 * @param[in] user_data user's data
 */
void
media_pipe_set_message_callback (MediaPipe *pipe, MessageCallback callback, gpointer user_data);

/*!
 * Build and start the internal Gstreamer pipeline, run a Glib main loop until pipeline occurs errors or receives EOS.
 *
 * @param pipe	MediaPipe context
 * @param init_reconfig_3a  whether to run reconfig_3a (to read and apply 3A configure in conf.xml) at start up
 *
 * @return FALSE indicates errors
 *
 * @note can't start again after stop
 */
gboolean
media_pipe_start (MediaPipe *pipe, gboolean reconfig_3a);

/*!
 * Stop the Glib main loop by sending an EOS to internal Gstreamer pipeline. \n
 * Any calls to media_pipe_start will return.
 *
 * @param pipe	MediaPipe context
 */
void
media_pipe_stop (MediaPipe *pipe);

#endif //MEDIA_PIPE_H
