/*
 * Copyright 2015 Rockchip Electronics Co. LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#if defined(_WIN32)
#include "vld.h"
#endif

#define MODULE_TAG "mpi_dec_test"
#define SZ_1M (1024*1024)
#define SZ_1K  (4*1024)
#include <string.h>
#include "rk_mpi.h"
#include "mpp_log.h"
#include "mpp_mem.h"
#include "mpp_env.h"
#include "mpp_time.h"
#include "mpp_common.h"
#include "v4l2/v4l2.h" 
#include "utils.h"
extern "C" {
#include "rkdrm/bo.h"
#include "rkdrm/dev.h"
#include "rkdrm/modeset.h"
}
#define MPI_DEC_LOOP_COUNT          4
#define MPI_DEC_STREAM_SIZE         (SZ_4K)
#define MAX_FILE_NAME_LENGTH        256
#define DEV_VIDEO      "/dev/video0"
#define CODEC_ALIGN(x, a)   (((x)+(a)-1)&~((a)-1))

V4L2 v4l2_;
sp_dev *mDev;
sp_plane **mPlanes;
sp_crtc *mCrtc;
sp_plane *mTestPlane;
int frame_size    = 0;
typedef struct {
    MppCtx          ctx;
    MppApi          *mpi;

    /* end of stream flag when set quit the loop */
    RK_U32          eos;

    /* buffer for stream data reading */
    char            *buf;

    /* input and output */
    MppBufferGroup  frm_grp;
    MppBufferGroup  pkt_grp;
    MppPacket       packet;
    size_t          packet_size;
    MppFrame        frame;

    FILE            *fp_input;
    FILE            *fp_output;
    FILE            *fp_config;
    RK_S32          frame_count;
    RK_S32          frame_num;
    size_t          max_usage;
} MpiDecLoopData;

typedef struct {
    char            file_input[MAX_FILE_NAME_LENGTH];
    char            file_output[MAX_FILE_NAME_LENGTH];
    char            file_config[MAX_FILE_NAME_LENGTH];
    MppCodingType   type;
    MppFrameFormat  format;
    RK_U32          width;
    RK_U32          height;
    RK_U32          debug;

    RK_U32          have_input;
    RK_U32          have_output;
    RK_U32          have_config;

    RK_U32          simple;
    RK_S32          timeout;
    RK_S32          frame_num;
    size_t          pkt_size;

    // report information
    size_t          max_usage;
} MpiDecTestCmd;

static OptionInfo mpi_dec_cmd[] = {
    {"c",               "ops_file",             "input operation config file"},
    {"w",               "width",                "the width of input bitstream"},
    {"h",               "height",               "the height of input bitstream"},
    {"t",               "type",                 "input stream coding type"},
    {"f",               "format",               "output frame format type"},
    {"d",               "debug",                "debug flag"},
    {"x",               "timeout",              "output timeout interval"},
    {"n",               "frame_number",         "max output frame number"},
};

long get_time() {
    struct timeval tv_date;
    gettimeofday(&tv_date, NULL);
    return (long)tv_date.tv_sec * 1000000 + (long)tv_date.tv_usec;
}

int drm_show_frmae(MppFrame frame) {
    sp_bo *bo;
    uint32_t handles[4], pitches[4], offsets[4];
    int width, height;
    int frm_size, ret, fd, err;

    err = mpp_frame_get_errinfo(frame) |
          mpp_frame_get_discard(frame);
    if (err) {
        printf("get err info %d discard %d,go back.\n",
                mpp_frame_get_errinfo(frame),
                mpp_frame_get_discard(frame));
        return -1;
    }


    width = mpp_frame_get_width(frame);
    height = mpp_frame_get_height(frame);
    width = CODEC_ALIGN(width, 16);
    height = CODEC_ALIGN(height, 16);
    frm_size = width * height * 3 / 2;
    fd = mpp_buffer_get_fd(mpp_frame_get_buffer(frame));

    bo = (struct sp_bo *)calloc(1, sizeof(struct sp_bo));
    if (!bo) {
        printf("failed to calloc bo.\n");
        return -2;
    }

    drmPrimeFDToHandle(mDev->fd, fd, &bo->handle);
    bo->dev = mDev;
    bo->width = width;
    bo->height = height;
    bo->depth = 16;
    bo->bpp = 32;
    bo->format = DRM_FORMAT_NV12;
    bo->flags = 0;

    handles[0] = bo->handle;
    pitches[0] = width;
    offsets[0] = 0;
    handles[1] = bo->handle;
    pitches[1] = width;
    offsets[1] = width * height;
    ret = drmModeAddFB2(mDev->fd, bo->width, bo->height,
                        bo->format, handles, pitches, offsets,
                        &bo->fb_id, bo->flags);
    if (ret != 0) {
        printf("failed to exec drmModeAddFb2.\n");
        return -3;
    }

    ret = drmModeSetPlane(mDev->fd, mTestPlane->plane->plane_id,
                          mCrtc->crtc->crtc_id, bo->fb_id, 0, 0, 0,
                          mCrtc->crtc->mode.hdisplay,
                          mCrtc->crtc->mode.vdisplay,
                          0, 0, bo->width << 16, bo->height << 16);
    if (ret) {
        printf("failed to exec drmModeSetPlane.\n");
        return -3;
    }

    if (mTestPlane->bo) {
        if (mTestPlane->bo->fb_id) {
            ret = drmModeRmFB(mDev->fd, mTestPlane->bo->fb_id);
            if (ret)
                printf("failed to exec drmModeRmFB.\n");
        }
        if (mTestPlane->bo->handle) {
            struct drm_gem_close req = {
                    .handle = mTestPlane->bo->handle,
            };

            drmIoctl(bo->dev->fd, DRM_IOCTL_GEM_CLOSE, &req);
        }
        free(mTestPlane->bo);
    }
    mTestPlane->bo = bo;

    return 0;
}

static int decode_simple(MpiDecLoopData *data)
{
    RK_U32 pkt_done = 0;
    RK_U32 pkt_eos  = 0;
    RK_U32 err_info = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = data->ctx;
    MppApi *mpi = data->mpi;
    char   *buf = data->buf;
    MppPacket packet = data->packet;
    MppFrame  frame  = NULL;
    size_t read_size = 0;
    size_t packet_size = data->packet_size;

    do {
        if (data->fp_config) {
            char line[MAX_FILE_NAME_LENGTH];
            char *ptr = NULL;

            do {
                ptr = fgets(line, MAX_FILE_NAME_LENGTH, data->fp_config);
                if (ptr) {
                    OpsLine info;
                    RK_S32 cnt = parse_config_line(line, &info);

                    // parser for packet message
                    if (cnt >= 3 && 0 == strncmp("pkt", info.cmd, sizeof(info.cmd))) {
                        packet_size = info.value2;
                        break;
                    }

                    // parser for reset message at the end
                    if (0 == strncmp("rst", info.cmd, 3)) {
                        mpp_log("get reset cmd\n");
                        packet_size = 0;
                        break;
                    }
                } else {
                    mpp_log("get end of cfg file\n");
                    packet_size = 0;
                    break;
                }
            } while (1);
        }

        // when packet size is valid read the input binary file
        if (packet_size)
            read_size = fread(buf, 1, packet_size, data->fp_input);

        if (!packet_size || read_size != packet_size || feof(data->fp_input)) {
            mpp_log("get error and check frame_num\n");
            if (data->frame_num < 0) {
                clearerr(data->fp_input);
                rewind(data->fp_input);
                if (data->fp_config) {
                    clearerr(data->fp_config);
                    rewind(data->fp_config);
                }
                data->eos = pkt_eos = 0;
                mpp_log("loop again\n");
            } else {
                // setup eos flag
                data->eos = pkt_eos = 1;
                mpp_log("found last packet\n");
                break;
            }
        }
    } while (!read_size);

    // write data to packet
    mpp_packet_write(packet, 0, buf, read_size);
    // reset pos and set valid length
    mpp_packet_set_pos(packet, buf);
    mpp_packet_set_length(packet, read_size);
    // setup eos flag
    if (pkt_eos)
        mpp_packet_set_eos(packet);

    do {
        RK_S32 times = 5;
        // send the packet first if packet is not done
        if (!pkt_done) {
            ret = mpi->decode_put_packet(ctx, packet);
            if (MPP_OK == ret)
                pkt_done = 1;
        }

        // then get all available frame and release
        do {
            RK_S32 get_frm = 0;
            RK_U32 frm_eos = 0;

        try_again:
            ret = mpi->decode_get_frame(ctx, &frame);
            if (MPP_ERR_TIMEOUT == ret) {
                if (times > 0) {
                    times--;
                    msleep(2);
                    goto try_again;
                }
                mpp_err("decode_get_frame failed too much time\n");
            }
            if (MPP_OK != ret) {
                mpp_err("decode_get_frame failed ret %d\n", ret);
                break;
            }

            if (frame) {
                if (mpp_frame_get_info_change(frame)) {
                    RK_U32 width = mpp_frame_get_width(frame);
                    RK_U32 height = mpp_frame_get_height(frame);
                    RK_U32 hor_stride = mpp_frame_get_hor_stride(frame);
                    RK_U32 ver_stride = mpp_frame_get_ver_stride(frame);
                    RK_U32 buf_size = mpp_frame_get_buf_size(frame);

                    mpp_log("decode_get_frame get info changed found\n");
                    mpp_log("decoder require buffer w:h [%d:%d] stride [%d:%d] buf_size %d",
                            width, height, hor_stride, ver_stride, buf_size);

                    /*
                     * NOTE: We can choose decoder's buffer mode here.
                     * There are three mode that decoder can support:
                     *
                     * Mode 1: Pure internal mode
                     * In the mode user will NOT call MPP_DEC_SET_EXT_BUF_GROUP
                     * control to decoder. Only call MPP_DEC_SET_INFO_CHANGE_READY
                     * to let decoder go on. Then decoder will use create buffer
                     * internally and user need to release each frame they get.
                     *
                     * Advantage:
                     * Easy to use and get a demo quickly
                     * Disadvantage:
                     * 1. The buffer from decoder may not be return before
                     * decoder is close. So memroy leak or crash may happen.
                     * 2. The decoder memory usage can not be control. Decoder
                     * is on a free-to-run status and consume all memory it can
                     * get.
                     * 3. Difficult to implement zero-copy display path.
                     *
                     * Mode 2: Half internal mode
                     * This is the mode current test code using. User need to
                     * create MppBufferGroup according to the returned info
                     * change MppFrame. User can use mpp_buffer_group_limit_config
                     * function to limit decoder memory usage.
                     *
                     * Advantage:
                     * 1. Easy to use
                     * 2. User can release MppBufferGroup after decoder is closed.
                     *    So memory can stay longer safely.
                     * 3. Can limit the memory usage by mpp_buffer_group_limit_config
                     * Disadvantage:
                     * 1. The buffer limitation is still not accurate. Memory usage
                     * is 100% fixed.
                     * 2. Also difficult to implement zero-copy display path.
                     *
                     * Mode 3: Pure external mode
                     * In this mode use need to create empty MppBufferGroup and
                     * import memory from external allocator by file handle.
                     * On Android surfaceflinger will create buffer. Then
                     * mediaserver get the file handle from surfaceflinger and
                     * commit to decoder's MppBufferGroup.
                     *
                     * Advantage:
                     * 1. Most efficient way for zero-copy display
                     * Disadvantage:
                     * 1. Difficult to learn and use.
                     * 2. Player work flow may limit this usage.
                     * 3. May need a external parser to get the correct buffer
                     * size for the external allocator.
                     *
                     * The required buffer size caculation:
                     * hor_stride * ver_stride * 3 / 2 for pixel data
                     * hor_stride * ver_stride / 2 for extra info
                     * Total hor_stride * ver_stride * 2 will be enough.
                     *
                     * For H.264/H.265 20+ buffers will be enough.
                     * For other codec 10 buffers will be enough.
                     */

                    if (NULL == data->frm_grp) {
                        /* If buffer group is not set create one and limit it */
                        ret = mpp_buffer_group_get_internal(&data->frm_grp, MPP_BUFFER_TYPE_ION);
                        if (ret) {
                            mpp_err("get mpp buffer group failed ret %d\n", ret);
                            break;
                        }

                        /* Set buffer to mpp decoder */
                        ret = mpi->control(ctx, MPP_DEC_SET_EXT_BUF_GROUP, data->frm_grp);
                        if (ret) {
                            mpp_err("set buffer group failed ret %d\n", ret);
                            break;
                        }
                    } else {
                        /* If old buffer group exist clear it */
                        ret = mpp_buffer_group_clear(data->frm_grp);
                        if (ret) {
                            mpp_err("clear buffer group failed ret %d\n", ret);
                            break;
                        }
                    }

                    /* Use limit config to limit buffer count to 24 with buf_size */
                    ret = mpp_buffer_group_limit_config(data->frm_grp, buf_size, 24);
                    if (ret) {
                        mpp_err("limit buffer group failed ret %d\n", ret);
                        break;
                    }

                    /*
                     * All buffer group config done. Set info change ready to let
                     * decoder continue decoding
                     */
                    ret = mpi->control(ctx, MPP_DEC_SET_INFO_CHANGE_READY, NULL);
                    if (ret) {
                        mpp_err("info change ready failed ret %d\n", ret);
                        break;
                    }
                } else {
                    err_info = mpp_frame_get_errinfo(frame) | mpp_frame_get_discard(frame);
                    if (err_info) {
                        mpp_log("decoder_get_frame get err info:%d discard:%d.\n",
                                mpp_frame_get_errinfo(frame), mpp_frame_get_discard(frame));
                    }
                    data->frame_count++;
                    mpp_log("decode_get_frame get frame %d\n", data->frame_count);
                    if (data->fp_output && !err_info)
                        dump_mpp_frame_to_file(frame, data->fp_output);
                }
                frm_eos = mpp_frame_get_eos(frame);
                mpp_frame_deinit(&frame);
                frame = NULL;
                get_frm = 1;
            }

            // try get runtime frame memory usage
            if (data->frm_grp) {
                size_t usage = mpp_buffer_group_usage(data->frm_grp);
                if (usage > data->max_usage)
                    data->max_usage = usage;
            }

            // if last packet is send but last frame is not found continue
            if (pkt_eos && pkt_done && !frm_eos) {
                msleep(10);
                continue;
            }

            if (frm_eos) {
                mpp_log("found last frame\n");
                break;
            }

            if (data->frame_num > 0 && data->frame_count >= data->frame_num) {
                data->eos = 1;
                break;
            }

            if (get_frm)
                continue;
            break;
        } while (1);

        if (data->frame_num > 0 && data->frame_count >= data->frame_num) {
            data->eos = 1;
            mpp_log("reach max frame number %d\n", data->frame_count);
            break;
        }

        if (pkt_done)
            break;

        /*
         * why sleep here:
         * mpi->decode_put_packet will failed when packet in internal queue is
         * full,waiting the package is consumed .Usually hardware decode one
         * frame which resolution is 1080p needs 2 ms,so here we sleep 3ms
         * * is enough.
         */
        msleep(3);
    } while (1);

    return ret;
}
char *swap_buff=NULL;
char *mjpeg_buff=NULL;
int swap_size;
static int decode_advanced(MpiDecLoopData *data)
{
    RK_U32 pkt_eos  = 0;
    MPP_RET ret = MPP_OK;
    MppCtx ctx  = data->ctx;
    MppApi *mpi = data->mpi;
    char   *buf = data->buf;
    MppPacket packet = data->packet;
    MppFrame  frame  = data->frame;
    MppTask task = NULL;

    // reset pos
    mpp_packet_set_pos(packet, buf);
    mpp_packet_set_length(packet, data->packet_size);
    // setup eos flag
    if (pkt_eos)
        mpp_packet_set_eos(packet);

    ret = mpi->poll(ctx, MPP_PORT_INPUT, MPP_POLL_BLOCK);
    if (ret) {
        mpp_err("mpp input poll failed\n");
        return ret;
    }

    ret = mpi->dequeue(ctx, MPP_PORT_INPUT, &task);  /* input queue */
    if (ret) {
        mpp_err("mpp task input dequeue failed\n");
        return ret;
    }
    mpp_assert(task);

    mpp_task_meta_set_packet(task, KEY_INPUT_PACKET, packet);
    mpp_task_meta_set_frame (task, KEY_OUTPUT_FRAME,  frame);

    ret = mpi->enqueue(ctx, MPP_PORT_INPUT, task);  /* input queue */
    if (ret) {
        mpp_err("mpp task input enqueue failed\n");
        return ret;
    }
    /* poll and wait here */
    ret = mpi->poll(ctx, MPP_PORT_OUTPUT, MPP_POLL_BLOCK);
    if (ret) {
        mpp_err("mpp output poll failed\n");
        return ret;
    }

    ret = mpi->dequeue(ctx, MPP_PORT_OUTPUT, &task); /* output queue */
    if (ret) {
        mpp_err("mpp task output dequeue failed\n");
        return ret;
    }
    mpp_assert(task);

    if (task) {
        MppFrame frame_out = NULL;
        mpp_task_meta_get_frame(task, KEY_OUTPUT_FRAME, &frame_out);
        //mpp_assert(packet_out == packet);

        if (frame) {
            /* write frame to file here */

            data->frame_count++;
            mpp_log("decoded frame %d\n", data->frame_count);
            drm_show_frmae(frame);
            if (mpp_frame_get_eos(frame_out))
                mpp_log("found eos frame\n");
        }

        /* output queue */
        ret = mpi->enqueue(ctx, MPP_PORT_OUTPUT, task);
        if (ret)
            mpp_err("mpp task output enqueue failed\n");
    }

    return ret;
}

int mpi_dec_test_decode(MpiDecTestCmd *cmd)
{
    MPP_RET ret         = MPP_OK;
    // base flow context
    MppCtx ctx          = NULL;
    MppApi *mpi         = NULL;

    // input / output
    MppPacket packet    = NULL;
    MppFrame  frame     = NULL;

    MpiCmd mpi_cmd      = MPP_CMD_BASE;
    MppParam param      = NULL;
    RK_U32 need_split   = 1;
    MppPollType timeout = cmd->timeout;

    // paramter for resource malloc
    RK_U32 width        = cmd->width;
    RK_U32 height       = cmd->height;
    MppCodingType type  = cmd->type;

    // resources
    char *buf           = NULL;
    
    size_t packet_size  = cmd->pkt_size;
    printf("packet_size:%d\n",packet_size);
    MppBuffer pkt_buf   = NULL;
    MppBuffer frm_buf   = NULL;

    MpiDecLoopData data;

    printf("mpi_dec_test start\n");
    memset(&data, 0, sizeof(data));

    if (cmd->simple) {
        buf = mpp_malloc(char, packet_size);
        if (NULL == buf) {
            mpp_err("mpi_dec_test malloc input stream buffer failed\n");
            goto MPP_TEST_OUT;
        }

        ret = mpp_packet_init(&packet, buf, packet_size);
        if (ret) {
            mpp_err("mpp_packet_init failed\n");
            goto MPP_TEST_OUT;
        }
    } else {
        RK_U32 hor_stride = MPP_ALIGN(width, 16);
        RK_U32 ver_stride = MPP_ALIGN(height, 16);

        ret = mpp_buffer_group_get_internal(&data.frm_grp, MPP_BUFFER_TYPE_ION);
        if (ret) {
            mpp_err("failed to get buffer group for input frame ret %d\n", ret);
            goto MPP_TEST_OUT;
        }

        ret = mpp_buffer_group_get_internal(&data.pkt_grp, MPP_BUFFER_TYPE_ION);
        if (ret) {
            mpp_err("failed to get buffer group for output packet ret %d\n", ret);
            goto MPP_TEST_OUT;
        }

        ret = mpp_frame_init(&frame); /* output frame */
        if (MPP_OK != ret) {
            mpp_err("mpp_frame_init failed\n");
            goto MPP_TEST_OUT;
        }
        /*
         * NOTE: For jpeg could have YUV420 and YUV422 the buffer should be
         * larger for output. And the buffer dimension should align to 16.
         * YUV420 buffer is 3/2 times of w*h.
         * YUV422 buffer is 2 times of w*h.
         * So create larger buffer with 2 times w*h.
         */
        ret = mpp_buffer_get(data.frm_grp, &frm_buf, hor_stride * ver_stride * 4);
        if (ret) {
            mpp_err("failed to get buffer for input frame ret %d\n", ret);
            goto MPP_TEST_OUT;
        }

        // NOTE: for mjpeg decoding send the whole file

        ret = mpp_buffer_get(data.pkt_grp, &pkt_buf, packet_size);
        if (ret) {
            mpp_err("failed to get buffer for input frame ret %d\n", ret);
            goto MPP_TEST_OUT;
        }
        mpp_packet_init_with_buffer(&packet, pkt_buf);
        buf = mpp_buffer_get_ptr(pkt_buf);

        mpp_frame_set_buffer(frame, frm_buf);
    }

    mpp_log("mpi_dec_test decoder test start w %d h %d type %d\n", width, height, type);

    // decoder demo
    ret = mpp_create(&ctx, &mpi);

    if (MPP_OK != ret) {
        mpp_err("mpp_create failed\n");
        goto MPP_TEST_OUT;
    }

    // NOTE: decoder split mode need to be set before init
    mpi_cmd = MPP_DEC_SET_PARSER_SPLIT_MODE;
    param = &need_split;
    ret = mpi->control(ctx, mpi_cmd, param);
    if (MPP_OK != ret) {
        mpp_err("mpi->control failed\n");
        goto MPP_TEST_OUT;
    }

    // NOTE: timeout value please refer to MppPollType definition
    //  0   - non-block call (default)
    // -1   - block call
    // +val - timeout value in ms
    if (timeout) {
        param = &timeout;
        ret = mpi->control(ctx, MPP_SET_OUTPUT_TIMEOUT, param);
        if (MPP_OK != ret) {
            mpp_err("Failed to set output timeout %d ret %d\n", timeout, ret);
            goto MPP_TEST_OUT;
        }
    }

    ret = mpp_init(ctx, MPP_CTX_DEC, type);
    if (MPP_OK != ret) {
        mpp_err("mpp_init failed\n");
        goto MPP_TEST_OUT;
    }

    data.ctx            = ctx;
    data.mpi            = mpi;
    data.eos            = 0;
    data.buf            = buf;
    data.packet         = packet;
    data.packet_size    = packet_size;
    data.frame          = frame;
    data.frame_count    = 0;
    data.frame_num      = cmd->frame_num;

    if (cmd->simple) {
        while (!data.eos) {
            decode_simple(&data);
        }
    } else {
        /* NOTE: change output format before jpeg decoding */
        if (cmd->format < MPP_FMT_BUTT)
            ret = mpi->control(ctx, MPP_DEC_SET_OUTPUT_FORMAT, &cmd->format);

        while (!data.eos) {
                long  mTimeE = get_time();
                int totalSize=v4l2_.read_frame(mjpeg_buff);

                memcpy(data.buf,mjpeg_buff,totalSize);
                memset(data.buf+totalSize,0,packet_size-totalSize);
                decode_advanced(&data);
                double timeDiff = double( get_time()-mTimeE)/1000.0;
                printf("每帧耗时:%lf ms  dec size:%d\n",timeDiff,totalSize);
        }
    }

    cmd->max_usage = data.max_usage;

    ret = mpi->reset(ctx);
    if (MPP_OK != ret) {
        mpp_err("mpi->reset failed\n");
        goto MPP_TEST_OUT;
    }

MPP_TEST_OUT:
    if (packet) {
        mpp_packet_deinit(&packet);
        packet = NULL;
    }

    if (frame) {
        mpp_frame_deinit(&frame);
        frame = NULL;
    }

    if (ctx) {
        mpp_destroy(ctx);
        ctx = NULL;
    }

    if (cmd->simple) {
        if (buf) {
            mpp_free(buf);
            buf = NULL;
        }
    } else {
        if (pkt_buf) {
            mpp_buffer_put(pkt_buf);
            pkt_buf = NULL;
        }

        if (frm_buf) {
            mpp_buffer_put(frm_buf);
            frm_buf = NULL;
        }
    }

    if (data.pkt_grp) {
        mpp_buffer_group_put(data.pkt_grp);
        data.pkt_grp = NULL;
    }

    if (data.frm_grp) {
        mpp_buffer_group_put(data.frm_grp);
        data.frm_grp = NULL;
    }

    if (data.fp_output) {
        fclose(data.fp_output);
        data.fp_output = NULL;
    }

    if (data.fp_input) {
        fclose(data.fp_input);
        data.fp_input = NULL;
    }

    return ret;
}

static void mpi_dec_test_help()
{
    mpp_log("usage: mpi_dec_test [options]\n");
    show_options(mpi_dec_cmd);
    mpp_show_support_format();
}

static RK_S32 mpi_dec_test_parse_options(int argc, char **argv, MpiDecTestCmd* cmd)
{
    const char *opt;
    const char *next;
    RK_S32 optindex = 1;
    RK_S32 handleoptions = 1;
    RK_S32 err = MPP_NOK;

    if ((argc < 2) || (cmd == NULL)) {
        err = 1;
        return err;
    }

    /* parse options */
    while (optindex < argc) {
        opt  = (const char*)argv[optindex++];
        next = (const char*)argv[optindex];

        if (handleoptions && opt[0] == '-' && opt[1] != '\0') {
            if (opt[1] == '-') {
                if (opt[2] != '\0') {
                    opt++;
                } else {
                    handleoptions = 0;
                    continue;
                }
            }

            opt++;

            switch (*opt) {
            case 'd':
                if (next) {
                    cmd->debug = atoi(next);;
                } else {
                    mpp_err("invalid debug flag\n");
                    goto PARSE_OPINIONS_OUT;
                }
                break;
            case 'w':
                if (next) {
                    cmd->width = atoi(next);
                } else {
                    mpp_err("invalid input width\n");
                    goto PARSE_OPINIONS_OUT;
                }
                break;
            case 'h':
                if ((*(opt + 1) != '\0') && !strncmp(opt, "help", 4)) {
                    mpi_dec_test_help();
                    err = 1;
                    goto PARSE_OPINIONS_OUT;
                } else if (next) {
                    cmd->height = atoi(next);
                } else {
                    mpp_log("input height is invalid\n");
                    goto PARSE_OPINIONS_OUT;
                }
                break;
            case 't':
                if (next) {
                    cmd->type = (MppCodingType)atoi(next);
                    err = mpp_check_support_format(MPP_CTX_DEC, cmd->type);
                }

                if (!next || err) {
                    mpp_err("invalid input coding type\n");
                    goto PARSE_OPINIONS_OUT;
                }
                break;
            case 'f':
                if (next) {
                    cmd->format = (MppFrameFormat)atoi(next);
                }

                if (!next || err) {
                    mpp_err("invalid input coding type\n");
                    goto PARSE_OPINIONS_OUT;
                }
                break;
            case 'x':
                if (next) {
                    cmd->timeout = atoi(next);
                }

                if (!next || cmd->timeout < 0) {
                    mpp_err("invalid output timeout interval\n");
                    goto PARSE_OPINIONS_OUT;
                }
                break;
            case 'n':
                if (next) {
                    cmd->frame_num = atoi(next);
                    if (cmd->frame_num < 0)
                        mpp_log("infinite loop decoding mode\n");
                } else {
                    mpp_err("invalid frame number\n");
                    goto PARSE_OPINIONS_OUT;
                }
                break;
            default:
                goto PARSE_OPINIONS_OUT;
                break;
            }

            optindex++;
        }
    }

    err = 0;

PARSE_OPINIONS_OUT:
    return err;
}

static void mpi_dec_test_show_options(MpiDecTestCmd* cmd)
{
    mpp_log("cmd parse result:\n");
    mpp_log("width      : %4d\n", cmd->width);
    mpp_log("height     : %4d\n", cmd->height);
    mpp_log("type       : %d\n", cmd->type);
    mpp_log("debug flag : %x\n", cmd->debug);
    mpp_log("max frames : %d\n", cmd->frame_num);
}

int main(int argc, char **argv)
{
    swap_size=0;
    swap_buff = (char*)malloc(5000*1024);
    mjpeg_buff = (char*)malloc(5000*1024);

    RK_S32 ret = 0;
    MpiDecTestCmd  cmd_ctx;
    MpiDecTestCmd* cmd = &cmd_ctx;

    memset((void*)cmd, 0, sizeof(*cmd));
    cmd->format = MPP_FMT_BUTT;
    cmd->pkt_size = SZ_1M;

    // parse the cmd option
    ret = mpi_dec_test_parse_options(argc, argv, cmd);
    if (ret) {
        if (ret < 0) {
            mpp_err("mpi_dec_test_parse_options: input parameter invalid\n");
        }

        mpi_dec_test_help();
        return ret;
    }

    mDev = create_sp_dev();
    if (!mDev) {
        printf("failed to exec create_sp_dev.\n");
        return -10;
    }

    ret = initialize_screens(mDev);
    if (ret != 0) {
        printf("failed to exec initialize_screens.\n");
        return -11;
    }

    mPlanes = (sp_plane **)calloc(mDev->num_planes, sizeof(*mPlanes));
    if (!mPlanes) {
        printf("failed to calloc mPlanes.\n");
        return -12;
    }

    mCrtc = &mDev->crtcs[0];
    for (int i = 0; i < mCrtc->num_planes; i++) {
        mPlanes[i] = get_sp_plane(mDev, mCrtc);
        if (is_supported_format(mPlanes[i], DRM_FORMAT_NV12))
            mTestPlane = mPlanes[i];
    }

    if (!mTestPlane) {
        printf("failed to get mTestPlane.\n");
        return -13;
    }

    v4l2_.init(DEV_VIDEO,cmd->width,cmd->height);
    v4l2_.open_device();
    v4l2_.init_device();
    v4l2_.start_capturing();
    for(int i=0;i<10;i++)
    {
        frame_size=v4l2_.read_frame(mjpeg_buff);
        if(frame_size>0)
            break;
    }
    if(frame_size<=0)
        return 1;
    mpi_dec_test_show_options(cmd);
    mpp_env_set_u32("mpi_debug", 1);

    cmd->simple = (cmd->type != MPP_VIDEO_CodingMJPEG) ? (1) : (0);
    ret = mpi_dec_test_decode(cmd);
    if (MPP_OK == ret)
        mpp_log("test success max memory %.2f MB\n", cmd->max_usage / (float)(1 << 20));
    else
        mpp_err("test failed ret %d\n", ret);

    mpp_env_set_u32("mpi_debug", 0x0);
    v4l2_.stop_capturing();
    v4l2_.uninit_device();
    return ret;
}

