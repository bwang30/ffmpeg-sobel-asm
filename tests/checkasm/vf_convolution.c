/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with FFmpeg; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <string.h>
#include "checkasm.h"
#include "libavfilter/avfilter.h"
#include "libavfilter/convolution.h"
#include "libavutil/intreadwrite.h"
#include "libavutil/mem_internal.h"

#define WIDTH 512
#define HEIGHT 512
#define SRC_STRIDE 512
#define PIXELS (WIDTH * HEIGHT)

#define randomize_buffers(buf, size)      \
    do {                                  \
        int j;                            \
        uint8_t *tmp_buf = (uint8_t *)buf;\
        for (j = 0; j< size; j++)         \
            tmp_buf[j] = rnd() & 0xFF;    \
    } while (0)

static void check_sobel(void)
{
    LOCAL_ALIGNED_32(uint8_t, src,     [PIXELS]);
    LOCAL_ALIGNED_32(uint8_t, dst_ref, [PIXELS]);
    LOCAL_ALIGNED_32(uint8_t, dst_new, [PIXELS]);
    const int height = WIDTH;
    const int width  = HEIGHT;
    const int stride = SRC_STRIDE;
    const int dstride = SRC_STRIDE;
    int mode = 0;
    const uint8_t *c[49];
    const int radius = 1;
    const int bpc = 1;
    const int step = mode == MATRIX_COLUMN ? 16 : 1;
    const int slice_start = 0;
    const int slice_end = height;
    int y;
    const int sizew = mode == MATRIX_COLUMN ? height : width;

    AVFilterContext *filter_in, *filter_sobel;
    AVFilterGraph *filter_graph;
    AVFilterLink *inlink;
    ConvolutionContext *s;
    char args[255];
    
    filter_graph = avfilter_graph_alloc();
    snprintf(args, sizeof(args), "video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d", WIDTH, HEIGHT, AV_PIX_FMT_YUV420P, 1, 1, 1, 1);
    avfilter_graph_create_filter(&filter_in, avfilter_get_by_name("buffer"), "in", args, NULL, filter_graph);
    avfilter_graph_create_filter(&filter_sobel, avfilter_get_by_name("sobel"), NULL, NULL, NULL, filter_graph);
    avfilter_link(filter_in, 0, filter_sobel, 0);

    inlink = filter_sobel->inputs[0];
    inlink->format = AV_PIX_FMT_YUV420P;
    ff_filter_param_init(filter_sobel);
    s = filter_sobel->priv;

    declare_func(void, uint8_t *dst, int width, float scale, float delta, const int *const matrix,
                 const uint8_t *c[], int peak, int radius, int dstride, int stride, int size);

    float scale = 2;
    float delta = 10;

    memset(dst_ref, 0, PIXELS);
    memset(dst_new, 0, PIXELS);
    randomize_buffers(src, PIXELS);

    if (check_func(s->filter[0], "sobel")) {
        for (y = slice_start; y < slice_end; y += step) {
            const int xoff = mode == MATRIX_COLUMN ? (y - slice_start) * bpc : radius * bpc;
            const int yoff = mode == MATRIX_COLUMN ? radius * dstride : 0;

            s->setup[0](radius, c, src, stride, radius, width, y, height, bpc);
            call_ref(dst_ref + yoff + xoff, sizew - 2 * radius,
                     scale, delta, NULL, c, 0, radius,
                     dstride, stride, slice_end - step);
            call_new(dst_new + yoff + xoff, sizew - 2 * radius,
                     scale, delta, NULL, c, 0, radius,
                     dstride, stride, slice_end - step);
            if (memcmp(dst_ref + yoff + xoff, dst_new + yoff + xoff, slice_end - step))
                fail();
            bench_new(dst_new + yoff + xoff, sizew - 2 * radius,
                      scale, delta, NULL, c, 0, radius,
                      dstride, stride, slice_end - step);
            if (mode != MATRIX_COLUMN)
                dst_ref += dstride;
        }
    }

    avfilter_free(filter_in);
    avfilter_free(filter_sobel);
    avfilter_graph_free(&filter_graph);
}

void checkasm_check_vf_convolution(void)
{
    check_sobel();
    report("convolution:soble");
}
