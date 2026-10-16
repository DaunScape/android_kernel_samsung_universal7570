// SPDX-License-Identifier: GPL-2.0
/*
 * j4cam - V4L2 front-end for the Galaxy J4's cameras.
 *
 * The FIMC-IS2 driver exposes ~20 low-level nodes (sensor, 3AA, ISP, MCSC...)
 * that need a Samsung-HAL-style sequence to produce a picture. This module
 * hides all of that behind two ordinary V4L2 capture nodes ("J4 Rear Camera",
 * "J4 Front Camera") that standard apps (Cheese, GStreamer, browsers) can use.
 *
 * On STREAMON it opens the FIMC-IS nodes from the kernel, runs the same
 * bring-up sequence the camd daemon uses (SS -> 3AA -> ISP -> MCSC, OTF),
 * then a kernel thread recycles the per-frame shot metadata and copies each
 * finished NV12 frame - rotated upright - into the application's buffer.
 * The camera is powered only while somebody is streaming.
 *
 * Both cameras share one 3AA/ISP/MCSC chain, so only one can stream at a time;
 * a second STREAMON fails with -EBUSY.
 *
 * Everything lives in the kernel: no daemon, no udev rules. The raw FIMC-IS
 * nodes are opened from a kernel worker (so it works for sandboxed apps too)
 * and are closed to everyone else (see fimc_is_set_raw_opener()).
 * Options are kernel command-line parameters: j4cam.pub_w=, j4cam.pub_h=,
 * j4cam.rear_rot=, j4cam.front_rot=, j4cam.rear_nr=, j4cam.front_nr=.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/uaccess.h>
#include <linux/kallsyms.h>
#include <linux/workqueue.h>
#include <linux/completion.h>
#include <linux/videodev2.h>
#include <linux/videodev2_exynos_camera.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-fh.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>
#include <media/videobuf2-ion.h>

#include "fimc-is-video.h"	/* struct fimc_is_video_ctx: reach the driver's vb2 queues */
#include "fimc-is-metadata.h"	/* struct camera2_shot_ext */

#define J4_NAME		"j4cam"

static int pub_w = 480;
static int pub_h = 640;
static int rear_rot = 90;
static int front_rot = 270;
static int rear_nr = 22;
static int front_nr = 23;
module_param(pub_w, int, 0444);
MODULE_PARM_DESC(pub_w, "published picture width (after rotation), default 480");
module_param(pub_h, int, 0444);
MODULE_PARM_DESC(pub_h, "published picture height (after rotation), default 640");
module_param(rear_rot, int, 0444);
MODULE_PARM_DESC(rear_rot, "clockwise rotation for the rear camera: 0, 90, 180 or 270");
module_param(front_rot, int, 0444);
MODULE_PARM_DESC(front_rot, "clockwise rotation for the front camera: 0, 90, 180 or 270");
module_param(rear_nr, int, 0444);
MODULE_PARM_DESC(rear_nr, "/dev/videoN number of the rear camera");
module_param(front_nr, int, 0444);
MODULE_PARM_DESC(front_nr, "/dev/videoN number of the front camera");

/* FIMC-IS video ids (device node = /dev/video(100 + id)) */
#define VID_30S		10
#define VID_30P		12
#define VID_I0S		30
#define VID_I0P		32
#define VID_M0S		60
#define VID_M0P		70

#define SPARE_SIZE	(32 * 1024)	/* trailing plane of every FIMC-IS buffer: shot metadata */
#define NBUF		4
#define OUT_FPS		30
#define FRAME_TIMEOUT_MS	2000

#define CID_IS_S_STREAM		V4L2_CID_IS_S_STREAM
#define CID_IS_END_OF_STREAM	V4L2_CID_IS_END_OF_STREAM
#define CID_IS_FORCE_DONE	V4L2_CID_IS_FORCE_DONE

struct j4_profile {
	const char *label;
	int sensor_id;		/* module id for S_INPUT */
	const char *name;	/* "rear" / "front", for the thread name */
	int ss_vid;		/* sensor video id: SS0 = 1, SS1 = 2 */
	int sen_w, sen_h;	/* sensor mode, 4:3 */
	int *rot;		/* clockwise rotation module parameter */
	int *nr;		/* video node number module parameter */
};

static const struct j4_profile profiles[] = {
	/* S5K3L2: mode 3, 2072x1552 @30, 4 lanes, mounted sideways */
	{ "J4 Rear Camera", 8, "rear", 1, 2072, 1552, &rear_rot, &rear_nr },
	/* S5K5E3: mode 1, 1280x960 @30, 2 lanes */
	{ "J4 Front Camera", 17, "front", 2, 1280, 960, &front_rot, &front_nr },
};
#define NPROFILES ARRAY_SIZE(profiles)

/* ------------------------------------------------------------------ */
/* the FIMC-IS pipeline (shared by both cameras)                       */
/* ------------------------------------------------------------------ */

enum { N_SS, N_3AA, N_3AAP, N_ISP, N_ISPP, N_MCS, N_MCSP, N_COUNT };

struct jnode {
	int vid;			/* FIMC-IS video id: node is /dev/video(100 + vid) */
	enum v4l2_buf_type type;
	struct file *f;
	struct vb2_queue *vbq;	/* the driver's own queue, for kernel pointers */
	int nplanes;
};

static struct jnode pn[N_COUNT] = {
	[N_SS]   = { 0,       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE },
	[N_3AA] = { VID_30S,  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE },
	[N_3AAP] = { VID_30P,  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE },
	[N_ISP] = { VID_I0S,  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE },
	[N_ISPP] = { VID_I0P,  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE },
	[N_MCS] = { VID_M0S,  V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE },
	[N_MCSP] = { VID_M0P,  V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE },
};

static DEFINE_MUTEX(pipe_lock);	/* serialises pipeline bring-up/tear-down */

struct j4_cam;
static struct j4_cam *active;	/* the camera currently streaming, if any */
static bool pipe_streaming;	/* sensor front-start issued */

static int out_w, out_h;	/* MCSC output size, in sensor orientation */

/* call a V4L2 ioctl of another driver's node from kernel context */
static int kioctl(struct file *f, unsigned int cmd, void *arg)
{
	mm_segment_t old = get_fs();
	long ret;

	set_fs(KERNEL_DS);
	ret = f->f_op->unlocked_ioctl(f, cmd, (unsigned long)arg);
	set_fs(old);
	return ret;
}

/*
 * The nodes are closed from a kernel worker, where fput() defers the final
 * release to a workqueue. Wait for it, so that a node is really closed (and the
 * FIMC-IS state really reset) before the next start can open it again.
 */
#ifdef MODULE
static void (*flush_fput_fn)(void);

static void flush_fput(void)
{
	if (flush_fput_fn)
		flush_fput_fn();
	else
		msleep(200);
}
#else
static void flush_fput(void)
{
	flush_delayed_fput();
}
#endif

static int jn_open(struct jnode *n)
{
	char path[80];
	struct fimc_is_video_ctx *vctx;
	struct file *f;

	/* devtmpfs creates these by itself, with or without udev */
	snprintf(path, sizeof(path), "/dev/video%d", 100 + n->vid);
	f = filp_open(path, O_RDWR | O_NONBLOCK, 0);
	if (IS_ERR(f)) {
		pr_err(J4_NAME ": open %s: %ld\n", path, PTR_ERR(f));
		return PTR_ERR(f);
	}
	n->f = f;
	vctx = f->private_data;
	n->vbq = vctx ? vctx->queue.vbq : NULL;
	return 0;
}

static void jn_close(struct jnode *n)
{
	if (n->f)
		filp_close(n->f, NULL);
	n->f = NULL;
	n->vbq = NULL;
	n->nplanes = 0;
}

static int jn_s_ctrl(struct jnode *n, unsigned int id, int value)
{
	struct v4l2_control c = { .id = id, .value = value };

	return kioctl(n->f, VIDIOC_S_CTRL, &c);
}

static int jn_s_input(struct jnode *n, unsigned int input)
{
	return kioctl(n->f, VIDIOC_S_INPUT, &input);
}

/* S_INPUT layout: stream[25:24] module[23:16] vindex[15:8] intype[7:4] leader[3:0] */
static unsigned int mk_input(int module, int vindex, int intype, int leader)
{
	return (module << 16) | (vindex << 8) | (intype << 4) | leader;
}

static int jn_s_fmt(struct jnode *n, u32 pixfmt, int w, int h)
{
	struct v4l2_format f;

	memset(&f, 0, sizeof(f));
	f.type = n->type;
	f.fmt.pix_mp.width = w;
	f.fmt.pix_mp.height = h;
	f.fmt.pix_mp.pixelformat = pixfmt;
	f.fmt.pix_mp.field = V4L2_FIELD_NONE;
	f.fmt.pix_mp.num_planes = 1;
	return kioctl(n->f, VIDIOC_S_FMT, &f);
}

/* MMAP buffers allocated by the driver itself; we only ever use their kernel mappings */
static int jn_reqbufs(struct jnode *n, int count)
{
	struct v4l2_requestbuffers rb = {
		.count = count, .type = n->type, .memory = V4L2_MEMORY_MMAP,
	};
	int ret = kioctl(n->f, VIDIOC_REQBUFS, &rb);

	if (ret)
		return ret;
	if (count && n->vbq && n->vbq->bufs[0])
		n->nplanes = n->vbq->bufs[0]->num_planes;
	return 0;
}

static int jn_qbuf(struct jnode *n, int index)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct v4l2_buffer b;

	memset(planes, 0, sizeof(planes));
	memset(&b, 0, sizeof(b));
	b.type = n->type;
	b.memory = V4L2_MEMORY_MMAP;
	b.index = index;
	b.m.planes = planes;
	b.length = n->nplanes;
	return kioctl(n->f, VIDIOC_QBUF, &b);
}

/* returns the buffer index, -EAGAIN if nothing is ready, or another -errno */
static int jn_dqbuf(struct jnode *n, u32 *flags)
{
	struct v4l2_plane planes[VIDEO_MAX_PLANES];
	struct v4l2_buffer b;
	int ret;

	memset(planes, 0, sizeof(planes));
	memset(&b, 0, sizeof(b));
	b.type = n->type;
	b.memory = V4L2_MEMORY_MMAP;
	b.m.planes = planes;
	b.length = n->nplanes;
	ret = kioctl(n->f, VIDIOC_DQBUF, &b);
	if (ret)
		return ret;
	if (flags)
		*flags = b.flags;
	return b.index;
}

static int jn_streamon(struct jnode *n)
{
	int type = n->type;

	return kioctl(n->f, VIDIOC_STREAMON, &type);
}

static void jn_streamoff(struct jnode *n)
{
	int type = n->type;

	if (n->f)
		kioctl(n->f, VIDIOC_STREAMOFF, &type);
}

static void fill_crop(u32 crop[4], int w, int h)
{
	crop[0] = 0;
	crop[1] = 0;
	crop[2] = w;
	crop[3] = h;
}

/*
 * One frame's shot for the head leader (3AA). Its node_group describes the
 * capture nodes of every group in the OTF chain: the 3AA and ISP junctions
 * and the MCSC output.
 */
static void fill_shot(const struct j4_profile *p, int index, u32 fcount)
{
	struct jnode *n = &pn[N_3AA];
	struct vb2_buffer *vb = n->vbq->bufs[index];
	struct camera2_shot_ext *s = vb2_plane_vaddr(vb, n->nplanes - 1);
	size_t size = vb2_plane_size(vb, n->nplanes - 1);

	if (!s)
		return;
	memset(s, 0, min_t(size_t, size, SPARE_SIZE));

	s->shot.magicNumber = SHOT_MAGIC_NUMBER;
	s->shot.ctl.request.frameCount = fcount;

	s->shot.ctl.aa.mode = AA_CONTROL_AUTO;
	s->shot.ctl.aa.aeMode = AA_AEMODE_ON;
	s->shot.ctl.aa.awbMode = AA_AWBMODE_WB_AUTO;
	s->shot.ctl.aa.afMode = AA_AFMODE_OFF;
	s->shot.ctl.aa.effectMode = AA_EFFECT_OFF;
	s->shot.ctl.aa.sceneMode = AA_SCENE_MODE_DISABLED;
	s->shot.ctl.aa.captureIntent = AA_CAPTURE_INTENT_PREVIEW;
	s->shot.ctl.aa.aeTargetFpsRange[0] = OUT_FPS;
	s->shot.ctl.aa.aeTargetFpsRange[1] = OUT_FPS;
	s->shot.ctl.sensor.exposureTime = 16666666;
	s->shot.ctl.sensor.frameDuration = 1000000000 / OUT_FPS;
	s->shot.ctl.sensor.sensitivity = 100;
	fill_crop(s->shot.ctl.scaler.cropRegion, p->sen_w, p->sen_h);

	s->node_group.leader.vid = n->vid;
	s->node_group.leader.request = 1;
	fill_crop(s->node_group.leader.input.cropRegion, p->sen_w, p->sen_h);
	fill_crop(s->node_group.leader.output.cropRegion, p->sen_w, p->sen_h);

	s->node_group.capture[0].vid = VID_30P;
	s->node_group.capture[0].request = 1;
	fill_crop(s->node_group.capture[0].input.cropRegion, p->sen_w, p->sen_h);
	fill_crop(s->node_group.capture[0].output.cropRegion, p->sen_w, p->sen_h);

	s->node_group.capture[1].vid = VID_I0P;
	s->node_group.capture[1].request = 1;
	fill_crop(s->node_group.capture[1].input.cropRegion, p->sen_w, p->sen_h);
	fill_crop(s->node_group.capture[1].output.cropRegion, p->sen_w, p->sen_h);

	s->node_group.capture[2].vid = VID_M0P;
	s->node_group.capture[2].request = 1;
	fill_crop(s->node_group.capture[2].input.cropRegion, p->sen_w, p->sen_h);
	fill_crop(s->node_group.capture[2].output.cropRegion, out_w, out_h);
}

static void pipe_stop(void)
{
	int i;

	if (pn[N_3AA].f && pipe_streaming) {
		/*
		 * Cancel pending shots and stop the head group (3AA) first: that
		 * cancels the queued shots for the whole OTF chain. Stopping the
		 * ISP/MCSC groups first makes each wait ~4.5 s for 3AA shots only
		 * the 3AA stop can cancel.
		 */
		jn_s_ctrl(&pn[N_3AA], CID_IS_FORCE_DONE, 1);
		jn_s_ctrl(&pn[N_ISP], CID_IS_FORCE_DONE, 1);
		jn_s_ctrl(&pn[N_MCS], CID_IS_FORCE_DONE, 1);
		if (pn[N_SS].f)
			jn_s_ctrl(&pn[N_SS], CID_IS_S_STREAM, 0);
		jn_streamoff(&pn[N_3AA]);
	}
	pipe_streaming = false;

	/* closing a node runs the driver's stop/release path */
	for (i = N_COUNT - 1; i >= 0; i--)
		jn_close(&pn[i]);
	flush_fput();
	pr_info(J4_NAME ": camera stopped\n");
}

static int pipe_start(const struct j4_profile *p)
{
	int i, ret;

	pn[N_SS].vid = p->ss_vid;

	for (i = 0; i < N_COUNT; i++) {
		ret = jn_open(&pn[i]);
		if (ret)
			goto fail;
	}

	ret = jn_s_input(&pn[N_SS], p->sensor_id);
	if (ret) {
		pr_err(J4_NAME ": S_INPUT sensor: %d\n", ret);
		goto fail;
	}
	/* all ischain nodes are open: let the driver bring the chain up */
	ret = jn_s_ctrl(&pn[N_3AA], CID_IS_END_OF_STREAM, 0);
	if (ret) {
		pr_err(J4_NAME ": END_OF_STREAM: %d\n", ret);
		goto fail;
	}
	ret = jn_s_input(&pn[N_3AA], mk_input(p->sensor_id, pn[N_SS].vid, 1, 1));
	if (!ret)
		ret = jn_s_input(&pn[N_ISP], mk_input(p->sensor_id, VID_30P, 1, 0));
	if (!ret)
		ret = jn_s_input(&pn[N_MCS], mk_input(p->sensor_id, VID_I0P, 1, 0));
	if (ret) {
		pr_err(J4_NAME ": S_INPUT chain: %d\n", ret);
		goto fail;
	}

	ret = jn_s_fmt(&pn[N_SS], V4L2_PIX_FMT_SBGGR10, p->sen_w, p->sen_h);
	if (!ret)
		ret = jn_s_fmt(&pn[N_3AA], V4L2_PIX_FMT_SBGGR10, p->sen_w, p->sen_h);
	if (!ret)
		ret = jn_s_fmt(&pn[N_3AAP], V4L2_PIX_FMT_SBGGR10, p->sen_w, p->sen_h);
	if (!ret)
		ret = jn_s_fmt(&pn[N_ISP], V4L2_PIX_FMT_SBGGR10, p->sen_w, p->sen_h);
	if (!ret)
		ret = jn_s_fmt(&pn[N_ISPP], V4L2_PIX_FMT_NV21, p->sen_w, p->sen_h);
	if (!ret)
		ret = jn_s_fmt(&pn[N_MCS], V4L2_PIX_FMT_NV21, p->sen_w, p->sen_h);
	if (!ret)
		ret = jn_s_fmt(&pn[N_MCSP], V4L2_PIX_FMT_NV12, out_w, out_h);
	if (ret) {
		pr_err(J4_NAME ": S_FMT: %d\n", ret);
		goto fail;
	}

	/* the sensor queue needs buffers allocated before STREAMON (never queued) */
	ret = jn_reqbufs(&pn[N_SS], 2);
	if (!ret)
		ret = jn_reqbufs(&pn[N_3AA], NBUF);
	if (!ret)
		ret = jn_reqbufs(&pn[N_ISP], NBUF);
	if (!ret)
		ret = jn_reqbufs(&pn[N_MCS], NBUF);
	if (!ret)
		ret = jn_reqbufs(&pn[N_MCSP], NBUF);
	if (ret) {
		pr_err(J4_NAME ": REQBUFS: %d\n", ret);
		goto fail;
	}
	if (!pn[N_3AA].vbq || !pn[N_MCSP].vbq || !pn[N_3AA].nplanes || !pn[N_MCSP].nplanes) {
		pr_err(J4_NAME ": cannot reach the driver's queues\n");
		ret = -EIO;
		goto fail;
	}
	if (vb2_plane_size(pn[N_MCSP].vbq->bufs[0], 0) < out_w * out_h * 3 / 2) {
		pr_err(J4_NAME ": MCSC output plane too small\n");
		ret = -EIO;
		goto fail;
	}

	for (i = 0; i < NBUF; i++) {
		fill_shot(p, i, i + 1);
		ret = jn_qbuf(&pn[N_MCSP], i);
		if (ret) {
			pr_err(J4_NAME ": QBUF m0p: %d\n", ret);
			goto fail;
		}
	}

	/* stream on: downstream first */
	if (jn_streamon(&pn[N_MCSP]) || jn_streamon(&pn[N_MCS]) ||
	    jn_streamon(&pn[N_ISP]) || jn_streamon(&pn[N_3AA]) ||
	    jn_streamon(&pn[N_SS])) {
		pr_err(J4_NAME ": STREAMON failed\n");
		ret = -EIO;
		goto fail;
	}
	pipe_streaming = true;	/* from here pipe_stop() must stop streams first */

	/* only the head group's leader (3AA) takes shots in the OTF chain */
	for (i = 0; i < NBUF; i++) {
		ret = jn_qbuf(&pn[N_3AA], i);
		if (ret) {
			pr_err(J4_NAME ": QBUF 3aa: %d\n", ret);
			goto fail;
		}
	}
	ret = jn_s_ctrl(&pn[N_SS], CID_IS_S_STREAM, 1);
	if (ret) {
		pr_err(J4_NAME ": S_STREAM: %d\n", ret);
		goto fail;
	}
	pr_info(J4_NAME ": %s streaming (sensor %dx%d -> %dx%d NV12)\n", p->label,
		p->sen_w, p->sen_h, out_w, out_h);
	return 0;

fail:
	pipe_stop();
	return ret < 0 ? ret : -EIO;
}

/* ------------------------------------------------------------------ */
/* the app-facing V4L2 device                                          */
/* ------------------------------------------------------------------ */

struct j4_buf {
	struct vb2_buffer vb;
	struct list_head list;
};

struct j4_cam {
	struct video_device vdev;
	struct vb2_queue vq;
	struct mutex lock;
	spinlock_t qlock;
	struct list_head queued;
	const struct j4_profile *p;
	int rot;		/* clockwise degrees */
	int w, h;		/* published size */
	struct task_struct *thr;
	u32 seq;
};

static struct v4l2_device v4l2_dev;
static struct j4_cam cams[NPROFILES];

#define TILE 32

/*
 * Tiled rotation of a w x h plane of T-sized elements (T = u8 for Y, u16 for
 * the interleaved UV pairs), one function per angle so the inner loop is a
 * plain strided copy.
 */
#define DEFINE_ROT(name, T, DI)							\
static void name(const T *src, T *dst, int w, int h)				\
{										\
	int x0, y0, x, y;							\
										\
	for (y0 = 0; y0 < h; y0 += TILE) {					\
		int ye = min(y0 + TILE, h);					\
										\
		for (x0 = 0; x0 < w; x0 += TILE) {				\
			int xe = min(x0 + TILE, w);				\
										\
			for (y = y0; y < ye; y++) {				\
				const T *sp = src + (size_t)y * w;		\
				for (x = x0; x < xe; x++)			\
					dst[DI] = sp[x];			\
			}							\
		}								\
	}									\
}

/* clockwise 90: (x,y) -> (h-1-y, x), destination is h wide */
DEFINE_ROT(rot90_8,   u8,  (size_t)x * h + (h - 1 - y))
DEFINE_ROT(rot90_16,  u16, (size_t)x * h + (h - 1 - y))
/* clockwise 270: (x,y) -> (y, w-1-x) */
DEFINE_ROT(rot270_8,  u8,  (size_t)(w - 1 - x) * h + y)
DEFINE_ROT(rot270_16, u16, (size_t)(w - 1 - x) * h + y)
/* 180: (x,y) -> (w-1-x, h-1-y), destination is w wide */
DEFINE_ROT(rot180_8,  u8,  (size_t)(h - 1 - y) * w + (w - 1 - x))
DEFINE_ROT(rot180_16, u16, (size_t)(h - 1 - y) * w + (w - 1 - x))

/* sw x sh NV12 source (sensor orientation) -> destination rotated clockwise */
static void rotate_nv12(const u8 *src, u8 *dst, int sw, int sh, int deg)
{
	const u8 *suv = src + sw * sh;
	u8 *duv = dst + sw * sh;

	switch (deg) {
	case 90:
		rot90_8(src, dst, sw, sh);
		rot90_16((const u16 *)suv, (u16 *)duv, sw / 2, sh / 2);
		break;
	case 270:
		rot270_8(src, dst, sw, sh);
		rot270_16((const u16 *)suv, (u16 *)duv, sw / 2, sh / 2);
		break;
	case 180:
		rot180_8(src, dst, sw, sh);
		rot180_16((const u16 *)suv, (u16 *)duv, sw / 2, sh / 2);
		break;
	default:
		memcpy(dst, src, sw * sh * 3 / 2);
		break;
	}
}

/* hand a finished camera frame to the next queued application buffer */
static void deliver(struct j4_cam *cam, int idx)
{
	struct vb2_buffer *src_vb = pn[N_MCSP].vbq->bufs[idx];
	struct j4_buf *b;
	unsigned long flags;
	size_t size = (size_t)out_w * out_h * 3 / 2;
	const u8 *src;
	u8 *dst;

	spin_lock_irqsave(&cam->qlock, flags);
	if (list_empty(&cam->queued)) {
		spin_unlock_irqrestore(&cam->qlock, flags);
		return;		/* nobody wants this frame */
	}
	b = list_first_entry(&cam->queued, struct j4_buf, list);
	list_del(&b->list);
	spin_unlock_irqrestore(&cam->qlock, flags);

	src = vb2_plane_vaddr(src_vb, 0);
	dst = vb2_plane_vaddr(&b->vb, 0);
	if (!src || !dst) {
		vb2_buffer_done(&b->vb, VB2_BUF_STATE_ERROR);
		return;
	}
	/* the hardware wrote the frame behind the CPU's cache */
	vb2_ion_sync_for_cpu(vb2_plane_cookie(src_vb, 0), 0, size, DMA_FROM_DEVICE);

	rotate_nv12(src, dst, out_w, out_h, cam->rot);

	vb2_set_plane_payload(&b->vb, 0, size);
	v4l2_get_timestamp(&b->vb.v4l2_buf.timestamp);
	b->vb.v4l2_buf.sequence = cam->seq++;
	b->vb.v4l2_buf.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&b->vb, VB2_BUF_STATE_DONE);
}

static int j4cam_thread(void *data)
{
	struct j4_cam *cam = data;
	u32 shot_count = NBUF;
	unsigned long last = jiffies;

	while (!kthread_should_stop()) {
		u32 flags = 0;
		int idx = jn_dqbuf(&pn[N_MCSP], &flags);

		if (idx == -EAGAIN) {
			if (time_after(jiffies, last + msecs_to_jiffies(FRAME_TIMEOUT_MS))) {
				pr_err(J4_NAME ": no frame for %d ms, giving up\n", FRAME_TIMEOUT_MS);
				vb2_queue_error(&cam->vq);
				while (!kthread_should_stop())
					msleep(20);
				break;
			}
			usleep_range(2000, 4000);
			continue;
		}
		if (idx < 0) {
			pr_err(J4_NAME ": DQBUF m0p: %d\n", idx);
			vb2_queue_error(&cam->vq);
			while (!kthread_should_stop())
				msleep(20);
			break;
		}
		last = jiffies;

		if (!(flags & V4L2_BUF_FLAG_ERROR))
			deliver(cam, idx);
		if (jn_qbuf(&pn[N_MCSP], idx))
			pr_err(J4_NAME ": QBUF m0p #%d failed\n", idx);

		/* recycle every shot buffer the 3AA has finished with */
		for (;;) {
			int sh = jn_dqbuf(&pn[N_3AA], NULL);

			if (sh < 0)
				break;
			fill_shot(cam->p, sh, ++shot_count);
			if (jn_qbuf(&pn[N_3AA], sh))
				pr_err(J4_NAME ": QBUF 3aa #%d failed\n", sh);
		}
	}
	return 0;
}

static void return_buffers(struct j4_cam *cam, enum vb2_buffer_state state)
{
	struct j4_buf *b, *tmp;
	unsigned long flags;
	LIST_HEAD(tmplist);

	spin_lock_irqsave(&cam->qlock, flags);
	list_splice_init(&cam->queued, &tmplist);
	spin_unlock_irqrestore(&cam->qlock, flags);

	list_for_each_entry_safe(b, tmp, &tmplist, list) {
		list_del(&b->list);
		vb2_buffer_done(&b->vb, state);
	}
}

/* --- bring-up / tear-down run in a kernel worker ---
 *
 * The FIMC-IS nodes are opened by path, so this must not run in the context of
 * whichever application called STREAMON: a sandboxed one (Flatpak, a container)
 * may not even see /dev/video10x. A kworker always sees the real root, and it is
 * also the one task allowed to open the (otherwise closed) raw nodes.
 */
enum { J_START, J_STOP };

struct j4_job {
	struct work_struct work;
	struct completion done;
	struct j4_cam *cam;
	int op;
	int ret;
};

static struct workqueue_struct *j4_wq;

static int do_start(struct j4_cam *cam)
{
	int ret;

	out_w = cam->w;
	out_h = cam->h;
	if (cam->rot == 90 || cam->rot == 270) {
		out_w = cam->h;
		out_h = cam->w;
	}
	ret = pipe_start(cam->p);
	if (ret)
		return ret;

	cam->seq = 0;
	cam->thr = kthread_run(j4cam_thread, cam, "j4cam-%s", cam->p->name);
	if (IS_ERR(cam->thr)) {
		ret = PTR_ERR(cam->thr);
		cam->thr = NULL;
		pipe_stop();
		return ret;
	}
	active = cam;
	return 0;
}

static void do_stop(struct j4_cam *cam)
{
	if (cam->thr) {
		kthread_stop(cam->thr);
		cam->thr = NULL;
	}
	pipe_stop();
	active = NULL;
}

static void job_fn(struct work_struct *work)
{
	struct j4_job *job = container_of(work, struct j4_job, work);

	fimc_is_set_raw_opener(current);
	if (job->op == J_START)
		job->ret = do_start(job->cam);
	else
		do_stop(job->cam);
	fimc_is_set_raw_opener(NULL);
	complete(&job->done);
}

/* caller holds pipe_lock */
static int run_job(int op, struct j4_cam *cam)
{
	struct j4_job job = { .cam = cam, .op = op };

	INIT_WORK_ONSTACK(&job.work, job_fn);
	init_completion(&job.done);
	queue_work(j4_wq, &job.work);
	wait_for_completion(&job.done);
	destroy_work_on_stack(&job.work);
	return job.ret;
}

/* --- vb2 ops --- */

static int j4_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
			  unsigned int *nbuffers, unsigned int *nplanes,
			  unsigned int sizes[], void *alloc_ctxs[])
{
	struct j4_cam *cam = vb2_get_drv_priv(vq);
	unsigned int size = cam->w * cam->h * 3 / 2;

	if (*nbuffers < 3)
		*nbuffers = 3;
	if (fmt && fmt->fmt.pix.sizeimage < size)
		return -EINVAL;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int j4_buf_prepare(struct vb2_buffer *vb)
{
	struct j4_cam *cam = vb2_get_drv_priv(vb->vb2_queue);
	unsigned long size = cam->w * cam->h * 3 / 2;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void j4_buf_queue(struct vb2_buffer *vb)
{
	struct j4_cam *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct j4_buf *b = container_of(vb, struct j4_buf, vb);
	unsigned long flags;

	spin_lock_irqsave(&cam->qlock, flags);
	list_add_tail(&b->list, &cam->queued);
	spin_unlock_irqrestore(&cam->qlock, flags);
}

static int j4_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct j4_cam *cam = vb2_get_drv_priv(vq);
	int ret;

	mutex_lock(&pipe_lock);
	if (active) {
		pr_warn(J4_NAME ": the other camera is streaming\n");
		ret = -EBUSY;
	} else {
		ret = run_job(J_START, cam);
	}
	mutex_unlock(&pipe_lock);

	if (ret)
		return_buffers(cam, VB2_BUF_STATE_QUEUED);
	return ret;
}

static void j4_stop_streaming(struct vb2_queue *vq)
{
	struct j4_cam *cam = vb2_get_drv_priv(vq);

	mutex_lock(&pipe_lock);
	if (active == cam)
		run_job(J_STOP, cam);
	mutex_unlock(&pipe_lock);
	return_buffers(cam, VB2_BUF_STATE_ERROR);
}

static struct vb2_ops j4_vb2_ops = {
	.queue_setup = j4_queue_setup,
	.buf_prepare = j4_buf_prepare,
	.buf_queue = j4_buf_queue,
	.start_streaming = j4_start_streaming,
	.stop_streaming = j4_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* --- V4L2 ioctls --- */

static void j4_fill_fmt(struct j4_cam *cam, struct v4l2_pix_format *pix)
{
	memset(pix, 0, sizeof(*pix));
	pix->width = cam->w;
	pix->height = cam->h;
	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = cam->w;
	pix->sizeimage = cam->w * cam->h * 3 / 2;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
}

static int j4_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	struct j4_cam *cam = video_drvdata(file);

	strlcpy(cap->driver, J4_NAME, sizeof(cap->driver));
	strlcpy(cap->card, cam->p->label, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform:" J4_NAME, sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int j4_enum_fmt(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV12;
	strlcpy(f->description, "Y/CbCr 4:2:0", sizeof(f->description));
	return 0;
}

/* a single fixed format: report it whatever was asked for */
static int j4_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	j4_fill_fmt(video_drvdata(file), &f->fmt.pix);
	return 0;
}

static int j4_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct j4_cam *cam = video_drvdata(file);

	if (vb2_is_busy(&cam->vq))
		return -EBUSY;
	j4_fill_fmt(cam, &f->fmt.pix);
	return 0;
}

static int j4_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fsize)
{
	struct j4_cam *cam = video_drvdata(file);

	if (fsize->index || fsize->pixel_format != V4L2_PIX_FMT_NV12)
		return -EINVAL;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = cam->w;
	fsize->discrete.height = cam->h;
	return 0;
}

static int j4_enum_frameintervals(struct file *file, void *priv, struct v4l2_frmivalenum *fival)
{
	struct j4_cam *cam = video_drvdata(file);

	if (fival->index || fival->pixel_format != V4L2_PIX_FMT_NV12 ||
	    fival->width != cam->w || fival->height != cam->h)
		return -EINVAL;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = OUT_FPS;
	return 0;
}

static int j4_g_parm(struct file *file, void *priv, struct v4l2_streamparm *p)
{
	if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	memset(&p->parm, 0, sizeof(p->parm));
	p->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	p->parm.capture.timeperframe.numerator = 1;
	p->parm.capture.timeperframe.denominator = OUT_FPS;
	return 0;
}

static int j4_enum_input(struct file *file, void *priv, struct v4l2_input *inp)
{
	struct j4_cam *cam = video_drvdata(file);

	if (inp->index)
		return -EINVAL;
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	strlcpy(inp->name, cam->p->label, sizeof(inp->name));
	return 0;
}

static int j4_g_input(struct file *file, void *priv, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int j4_s_input(struct file *file, void *priv, unsigned int i)
{
	return i ? -EINVAL : 0;
}

static const struct v4l2_ioctl_ops j4_ioctl_ops = {
	.vidioc_querycap		= j4_querycap,
	.vidioc_enum_fmt_vid_cap	= j4_enum_fmt,
	.vidioc_g_fmt_vid_cap		= j4_g_fmt,
	.vidioc_s_fmt_vid_cap		= j4_s_fmt,
	.vidioc_try_fmt_vid_cap		= j4_g_fmt,
	.vidioc_enum_framesizes		= j4_enum_framesizes,
	.vidioc_enum_frameintervals	= j4_enum_frameintervals,
	.vidioc_g_parm			= j4_g_parm,
	.vidioc_s_parm			= j4_g_parm,
	.vidioc_enum_input		= j4_enum_input,
	.vidioc_g_input			= j4_g_input,
	.vidioc_s_input			= j4_s_input,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations j4_fops = {
	.owner		= THIS_MODULE,
	.open		= v4l2_fh_open,
	.release	= vb2_fop_release,
	.read		= vb2_fop_read,
	.poll		= vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.unlocked_ioctl	= video_ioctl2,
};

/* ------------------------------------------------------------------ */
/* module init / exit                                                  */
/* ------------------------------------------------------------------ */

static int valid_rot(int r)
{
	return r == 0 || r == 90 || r == 180 || r == 270;
}

static void j4_unregister(int count)
{
	int i;

	for (i = 0; i < count; i++)
		video_unregister_device(&cams[i].vdev);
}

static int __init j4cam_init(void)
{
	int i, ret;

	if (!valid_rot(rear_rot) || !valid_rot(front_rot)) {
		pr_err(J4_NAME ": rotation must be 0, 90, 180 or 270\n");
		return -EINVAL;
	}
	if (pub_w < 64 || pub_h < 64 || pub_w > 1920 || pub_h > 1920) {
		pr_err(J4_NAME ": bad published size %dx%d\n", pub_w, pub_h);
		return -EINVAL;
	}
	/* the MCSC scaler works in sensor orientation; its stride is 16-byte aligned */
	for (i = 0; i < (int)NPROFILES; i++) {
		int rot = *profiles[i].rot;
		int sw = (rot == 90 || rot == 270) ? pub_h : pub_w;
		int sh = (rot == 90 || rot == 270) ? pub_w : pub_h;

		if ((sw % 16) || (sh % 2)) {
			pr_err(J4_NAME ": %s: scaler size %dx%d needs width %% 16 == 0 and even height\n",
			       profiles[i].label, sw, sh);
			return -EINVAL;
		}
	}

#ifdef MODULE
	flush_fput_fn = (void (*)(void))kallsyms_lookup_name("flush_delayed_fput");
#endif
	j4_wq = alloc_ordered_workqueue("j4cam", 0);
	if (!j4_wq)
		return -ENOMEM;

	snprintf(v4l2_dev.name, sizeof(v4l2_dev.name), J4_NAME);
	ret = v4l2_device_register(NULL, &v4l2_dev);
	if (ret) {
		destroy_workqueue(j4_wq);
		return ret;
	}

	for (i = 0; i < (int)NPROFILES; i++) {
		struct j4_cam *cam = &cams[i];
		struct vb2_queue *vq = &cam->vq;

		cam->p = &profiles[i];
		cam->rot = *profiles[i].rot;
		cam->w = pub_w;
		cam->h = pub_h;
		mutex_init(&cam->lock);
		spin_lock_init(&cam->qlock);
		INIT_LIST_HEAD(&cam->queued);

		vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
		vq->drv_priv = cam;
		vq->buf_struct_size = sizeof(struct j4_buf);
		vq->ops = &j4_vb2_ops;
		vq->mem_ops = &vb2_vmalloc_memops;
		vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
		vq->lock = &cam->lock;
		ret = vb2_queue_init(vq);
		if (ret)
			goto err;

		strlcpy(cam->vdev.name, cam->p->label, sizeof(cam->vdev.name));
		cam->vdev.v4l2_dev = &v4l2_dev;
		cam->vdev.fops = &j4_fops;
		cam->vdev.ioctl_ops = &j4_ioctl_ops;
		cam->vdev.release = video_device_release_empty;
		cam->vdev.vfl_dir = VFL_DIR_RX;
		cam->vdev.queue = vq;
		cam->vdev.lock = &cam->lock;
		video_set_drvdata(&cam->vdev, cam);

		ret = video_register_device(&cam->vdev, VFL_TYPE_GRABBER, *cam->p->nr);
		if (ret) {
			pr_err(J4_NAME ": register %s: %d\n", cam->p->label, ret);
			goto err;
		}
		pr_info(J4_NAME ": %s -> /dev/video%d (%dx%d NV12, rotated %d)\n",
			cam->p->label, cam->vdev.num, cam->w, cam->h, cam->rot);
	}
	return 0;

err:
	j4_unregister(i);
	v4l2_device_unregister(&v4l2_dev);
	destroy_workqueue(j4_wq);
	return ret;
}

static void __exit j4cam_exit(void)
{
	j4_unregister(NPROFILES);
	v4l2_device_unregister(&v4l2_dev);
	destroy_workqueue(j4_wq);
}

module_init(j4cam_init);
module_exit(j4cam_exit);

MODULE_DESCRIPTION("V4L2 front-end for the Galaxy J4 (FIMC-IS2) cameras");
MODULE_LICENSE("GPL");
