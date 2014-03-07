#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <stdbool.h>

#include "time.h"
#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "RaspiCamControl.h"
#include "RaspiPreview.h"
#include "RaspiCLI.h"

#include <semaphore.h>
#include <cv.h>
#include <highgui.h>

#include "Utils.h"

/// Camera number to use - we only have one camera, indexed from 0.
#define CAMERA_NUMBER 0

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

// Video format information
#define VIDEO_FRAME_RATE_NUM 30
#define VIDEO_FRAME_RATE_DEN 1

/// Video render needs at least 2 buffers.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

// Max bitrate we allow for recording
const int MAX_BITRATE = 30000000; // 30Mbits/s

// Configuration
#define VERBOSE 0
int w = 320;
int h = 240;
int stableWait = 30;
int calibrationFrames = 30;
int maxThreshold = 25;
int frameSkip = 5;
bool save = false;
bool gotPrevious = false;
bool calibrating = true;
int meanDelta = -1;
int thresholdMultiplier = 2;

// For stats
int nCount=0;
IplImage *py, *lastpy, *pu, *pv, *pu_big, *pv_big, *image,* dstImage;

int mmal_status_to_int(MMAL_STATUS_T status);

typedef struct
{
        int timeout;                        /// Time taken before frame is grabbed and app then shuts down. Units are milliseconds
        int width;                          /// Requested width of image
        int height;                         /// requested height of image
        int bitrate;                        /// Requested bitrate
        int framerate;                      /// Requested frame rate (fps)
        int graymode;            /// capture in gray only (2x faster)
        int immutableInput;      /// Flag to specify whether encoder works in place or creates a new buffer. Result is preview can display either
        /// the camera output or the encoder output (with compression artifacts)
        RASPIPREVIEW_PARAMETERS preview_parameters;   /// Preview setup parameters
        RASPICAM_CAMERA_PARAMETERS camera_parameters; /// Camera setup parameters

        MMAL_COMPONENT_T *camera_component;    /// Pointer to the camera component
        MMAL_COMPONENT_T *encoder_component;   /// Pointer to the encoder component
        MMAL_CONNECTION_T *preview_connection; /// Pointer to the connection from camera to preview
        MMAL_CONNECTION_T *encoder_connection; /// Pointer to the connection from camera to encoder

        MMAL_POOL_T *video_pool; /// Pointer to the pool of buffers used by encoder output port

} RASPIVID_STATE;

typedef struct
{
        FILE *file_handle;                   /// File handle to write buffer data to.
        VCOS_SEMAPHORE_T complete_semaphore; /// semaphore which is posted when we reach end of frame (indicates end of capture or fault)
        RASPIVID_STATE *pstate;            /// pointer to our state in case required in callback
} PORT_USERDATA;

static void default_status(RASPIVID_STATE *state)
{
    if (!state)
    {
        vcos_assert(0);
        return;
    }

    // Default everything to zero
    memset(state, 0, sizeof(RASPIVID_STATE));

    // Now set anything non-zero
    state->timeout           = 1;
    state->width             = w;      // use a multiple of 320 (640, 1280)
    state->height            = h;      // use a multiple of 240 (480, 960)
    state->bitrate           = 17000000; // This is a decent default bitrate for 1080p
    state->framerate         = VIDEO_FRAME_RATE_NUM;
    state->immutableInput    = 1;
    state->graymode          = 0;

    // Setup preview window defaults
    raspipreview_set_defaults(&state->preview_parameters);

    // Set up the camera_parameters to default
    raspicamcontrol_set_defaults(&state->camera_parameters);
}

static void save_image(MMAL_BUFFER_HEADER_T *buffer)
{
    int h4 = h / 4;
    memcpy(pu->imageData, buffer->data + w * h, w * h4); // read U
    memcpy(pv->imageData, buffer->data + w * h + w * h4, w * h4); // read v

    cvResize(pu, pu_big, CV_INTER_NN);
    cvResize(pv, pv_big, CV_INTER_NN);  //CV_INTER_LINEAR looks better but it's slower
    cvMerge(py, pu_big, pv_big, NULL, image);

    cvCvtColor(image, dstImage, CV_YCrCb2RGB);    // convert in RGB color space (slow)
    
    char strBuf[32];
    
    sprintf(strBuf, "image%d.jpg", nCount);
    
    cvSaveImage(strBuf, dstImage, 0);
    
    /*
    char strBufGrey[32];
    
    sprintf(strBufGrey, "image_g_%d.png", nCount);
    
    cvSaveImage(strBufGrey, py, 0);
    
    char strBufU[32];
    
    sprintf(strBufU, "image_u_%d.png", nCount);
    
    cvSaveImage(strBufU, pu_big, 0);
    
    char strBufV[32];
    
    sprintf(strBufV, "image_v_%d.png", nCount);
    
    cvSaveImage(strBufV, pv_big, 0);
    */
    
    #if VERBOSE
    printf("Written image %d\n", nCount);
    #endif
}

static void video_buffer_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer)
{
    MMAL_BUFFER_HEADER_T *new_buffer;
    PORT_USERDATA *pData = (PORT_USERDATA *)port->userdata;

    if (pData)
    {
        if (buffer->length)
        {
            mmal_buffer_header_mem_lock(buffer);

            if(nCount % frameSkip == 0 && nCount > stableWait)
            {
                int w = pData->pstate->width; // get image size
                int h = pData->pstate->height;

                if(!gotPrevious)
                {
                    gotPrevious = true;
                    memcpy(lastpy->imageData, buffer->data, w * h); // read Y
                }
                else
                {
                    memcpy(py->imageData, buffer->data, w * h); // read Y

                    if(save)
                    {
                        save_image(buffer);
                    }
                    
                    int sum = 0;
                    int max = 0;
                    int diff = 0;
                    
                    int i;
                    for(i = 0; i < w * h; i++)
                    {
                        diff = abs(py->imageData[i] - lastpy->imageData[i]);
                        
                        if(diff > max)
                        {
                            max = diff;
                        }
                        
                        sum += diff;
                    }
                    
                    if(calibrating)
                    {
                        int realNumFrames = nCount / frameSkip;

                        if(meanDelta == -1)
                        {
                            meanDelta = sum;
                        }
                        else
                        {
                            meanDelta = (meanDelta * realNumFrames + sum) / (realNumFrames + 1);
                        }

                        #if VERBOSE
                        printf("meanDelta: %d after %d frames\n", meanDelta, realNumFrames);                            
                        #endif
                        
                        if(realNumFrames > calibrationFrames)
                        {
                            calibrating = false;
                        }
                    }
                    else if(sum > (meanDelta * thresholdMultiplier) && max > maxThreshold)
                    {
                        #if VERBOSE
                        printf("sum: %d, max: %d\n", sum, max);
                        #endif
                        
                        save_image(buffer);
                        
                        exit(1);
                    }
                    
                    memcpy(lastpy->imageData, buffer->data, w * h); // read Y
                    
                    #if VERBOSE
                    printf("Processed frame %d, (%d)\n", nCount, sum);
                    #endif
                }
            }

            mmal_buffer_header_mem_unlock(buffer);
            
            nCount++;
        }
        else 
        {
            vcos_log_error("buffer null");
        }
    }
    else
    {
        vcos_log_error("Received a encoder buffer callback with no state");
    }

    // release buffer back to the pool
    mmal_buffer_header_release(buffer);

    // and send one back to the port (if still open)
    if (port->is_enabled)
    {
        MMAL_STATUS_T status;

        new_buffer = mmal_queue_get(pData->pstate->video_pool->queue);

        if (new_buffer)
        {
            status = mmal_port_send_buffer(port, new_buffer);
        }

        if (!new_buffer || status != MMAL_SUCCESS)
        {
            vcos_log_error("Unable to return a buffer to the encoder port");
        }
    }

}

static MMAL_COMPONENT_T *create_camera_component(RASPIVID_STATE *state)
{
    MMAL_COMPONENT_T *camera = 0;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *video_port = NULL, *still_port = NULL;
    MMAL_STATUS_T status;

    /* Create the component */
    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);

    if (status != MMAL_SUCCESS)
    {
        vcos_log_error("Failed to create camera component");
        goto error;
    }

    if (!camera->output_num)
    {
        vcos_log_error("Camera doesn't have output ports");
        goto error;
    }

    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
    still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

    //  set up the camera configuration
    MMAL_PARAMETER_CAMERA_CONFIG_T cam_config =
    {
                    { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config) },
                    .max_stills_w = state->width,
                    .max_stills_h = state->height,
                    .stills_yuv422 = 0,
                    .one_shot_stills = 0,
                    .max_preview_video_w = state->width,
                    .max_preview_video_h = state->height,
                    .num_preview_video_frames = 3,
                    .stills_capture_circular_buffer_height = 0,
                    .fast_preview_resume = 0,
                    .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
    };
    mmal_port_parameter_set(camera->control, &cam_config.hdr);
    // Set the encode format on the video  port

    format = video_port->format;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->encoding = MMAL_ENCODING_I420;
    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = state->framerate;
    format->es->video.frame_rate.den = VIDEO_FRAME_RATE_DEN;

    status = mmal_port_format_commit(video_port);
    if (status)
    {
        vcos_log_error("camera video format couldn't be set");
        goto error;
    }

    // PR : plug the callback to the video port 
    status = mmal_port_enable(video_port, video_buffer_callback);
    if (status)
    {
        vcos_log_error("camera video callback2 error");
        goto error;
    }

    // Ensure there are enough buffers to avoid dropping frames
    if (video_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    {
        video_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    }

    // Set the encode format on the still  port
    format = still_port->format;
    format->encoding = MMAL_ENCODING_OPAQUE;
    format->encoding_variant = MMAL_ENCODING_I420;
    format->es->video.width = state->width;
    format->es->video.height = state->height;
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = state->width;
    format->es->video.crop.height = state->height;
    format->es->video.frame_rate.num = 1;
    format->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(still_port);
    if (status)
    {
        vcos_log_error("camera still format couldn't be set");
        goto error;
    }

    //PR : create pool of message on video port
    MMAL_POOL_T *pool;
    video_port->buffer_size = video_port->buffer_size_recommended;
    video_port->buffer_num = video_port->buffer_num_recommended;
    pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);
    if (!pool)
    {
        vcos_log_error("Failed to create buffer header pool for video output port");
    }
    state->video_pool = pool;

    /* Ensure there are enough buffers to avoid dropping frames */
    if (still_port->buffer_num < VIDEO_OUTPUT_BUFFERS_NUM)
    {
        still_port->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    }

    /* Enable component */
    status = mmal_component_enable(camera);

    if (status)
    {
        vcos_log_error("camera component couldn't be enabled");
        goto error;
    }

    raspicamcontrol_set_all_parameters(camera, &state->camera_parameters);

    state->camera_component = camera;

    return camera;

    error:

    if (camera)
    {
        mmal_component_destroy(camera);
    }

    return 0;
}

static void destroy_camera_component(RASPIVID_STATE *state)
{
    if (state->camera_component)
    {
        mmal_component_destroy(state->camera_component);
        state->camera_component = NULL;
    }
}

static void check_disable_port(MMAL_PORT_T *port)
{
    if (port && port->is_enabled)
    {
        mmal_port_disable(port);
    }
}

int main(int argc, const char **argv)
{
    #if VERBOSE
    printf("Initialising... ");
    fflush(stdout);
    #endif    
    
    // Our main data storage vessel..
    RASPIVID_STATE state;

    MMAL_STATUS_T status = -1;
    MMAL_PORT_T *camera_video_port = NULL;
    MMAL_PORT_T *camera_still_port = NULL;

    #if VERBOSE
    printf("bcm_host_init, ");
    fflush(stdout);
    #endif
    
    bcm_host_init();

    // init windows and OpenCV Stuff
    if(find_argument(argc, argv, "-w") != -1 && find_argument(argc, argv, "-h") != -1)
    {
        parse_argument(argc, argv, "-w", &w);
        parse_argument(argc, argv, "-h", &h);
    }
    
    save = find_argument(argc, argv, "-s") != -1;
    
    // read default status
    default_status(&state);
    
    dstImage = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);
    py = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);       // Y component of YUV I420 frame
    lastpy = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);   // Y component of previous YUV I420 frame
    pu = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);   // U component of YUV I420 frame
    pv = cvCreateImage(cvSize(w/2,h/2), IPL_DEPTH_8U, 1);   // V component of YUV I420 frame
    pu_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    pv_big = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 1);
    image = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3);    // final picture to display
    
    #if VERBOSE
    printf("create_camera_component, ");
    fflush(stdout);
    #endif
    
    // create camera
    if (!create_camera_component(&state))
    {
        vcos_log_error("%s: Failed to create camera component", __func__);
    }
    else if ((status = raspipreview_create(&state.preview_parameters)) != MMAL_SUCCESS)
    {
        vcos_log_error("%s: Failed to create preview component", __func__);
        destroy_camera_component(&state);
    }
    else
    {
        PORT_USERDATA callback_data;

        camera_video_port   = state.camera_component->output[MMAL_CAMERA_VIDEO_PORT];
        camera_still_port   = state.camera_component->output[MMAL_CAMERA_CAPTURE_PORT];

        VCOS_STATUS_T vcos_status;

        callback_data.pstate = &state;

        vcos_status = vcos_semaphore_create(&callback_data.complete_semaphore, "RaspiStill-sem", 0);
        vcos_assert(vcos_status == VCOS_SUCCESS);

        // assign data to use for callback
        camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T *)&callback_data;

        #if VERBOSE
        printf("starting capture at %dx%d!\n", w, h);
        fflush(stdout);
        #endif
        
        // start capture
        if (mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1) != MMAL_SUCCESS)
        {
            goto error;
        }

        // Send all the buffers to the video port

        int num = mmal_queue_length(state.video_pool->queue);
        int q;
        
        for (q = 0; q < num; q++)
        {
            MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(state.video_pool->queue);

            if (!buffer)
            {
                vcos_log_error("Unable to get a required buffer %d from pool queue", q);
            }

            if (mmal_port_send_buffer(camera_video_port, buffer)!= MMAL_SUCCESS)
            {
                vcos_log_error("Unable to send a buffer to encoder output port (%d)", q);
            }
        }

        // Now sleep forever
        while(true)
        {
            sleep(1);
        }

error:
        mmal_status_to_int(status);

        // Disable all our ports that are not handled by connections
        check_disable_port(camera_still_port);

        if (state.camera_component)
        {
            mmal_component_disable(state.camera_component);
        }

        raspipreview_destroy(&state.preview_parameters);
    }
    
    if(status != 0)
    {
        raspicamcontrol_check_configuration(128);
    }

    cvReleaseImage(&dstImage);
    cvReleaseImage(&pu);
    cvReleaseImage(&pv);
    cvReleaseImage(&py);
    cvReleaseImage(&lastpy);
    cvReleaseImage(&pu_big);
    cvReleaseImage(&pv_big);

    return 0;
}

