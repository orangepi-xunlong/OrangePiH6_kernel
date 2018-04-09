#include "disp_capture.h"
#include <video/sunxi_display2.h>
#include <linux/file.h>
#include <linux/sync.h>
#include <linux/sw_sync.h>

/*
 * Description of the buffer slot state.
 *
 * SLOT_INVALID: buffer slot is not initial yet
 * SLOT_FREE   : free to writeback
 * SLOT_WRITING: writing by capture
 * SLOT_READY  : ready for hwcomposer to acquire
 * SLOT_READING: reading by display engine
 */
enum buffer_slot_state {
	SLOT_INVALID = 0,
	SLOT_FREE    = 1,
	SLOT_WRITING = 2,
	SLOT_READY   = 3,
	SLOT_READING = 4,
};

struct disp_capture_info_list {
	struct disp_capture_info info;
	struct list_head list;
};

#define MAX_CAPTURE_LIST_SIZE	16

struct disp_buffer_node {
	struct list_head list;
	struct disp_capture_buffer *buf;
};

struct buffer_slot {
	struct disp_capture_buffer buf;
	struct dmabuf_item *dma_map;
	struct disp_capture_info_inner info;
	u32 pt_value;
	int state;
	struct timespec timestamp;
	struct list_head list;
};

struct disp_capture_private_data {
	u32 reg_base;
	u32 enabled;

	/*
	 * indicate the last write back result
	 * 0 -- means every thing is ok
	 * others -- fail/busy/error
	 */
	s32 status;

	/*
	 * writebback period in timespec
	 */
	struct timespec period;

	u32 timeline_max;
	struct sw_sync_timeline *timeline;

	struct disp_capture_fmt fmt;
	struct buffer_slot *writing;
	struct buffer_slot slots[MAX_CAPTURE_LIST_SIZE];
	int buffer_count;

	spinlock_t empty_lock;
	spinlock_t available_lock;
	struct list_head empty_list;
	struct list_head available_list;

	s32(*shadow_protect) (u32 sel, bool protect);

	struct clk *clk;
#if defined(__LINUX_PLAT__)
	struct mutex mlock;
	spinlock_t data_lock;
#else
	int mlock;
	int data_lock;
#endif
};

static struct disp_capture *captures;
static struct disp_capture_private_data *capture_private;

struct disp_capture *disp_get_capture(u32 disp)
{
	u32 num_screens;

	num_screens = bsp_disp_feat_get_num_screens();
	if (disp >= num_screens) {
		DE_WRN("disp %d out of range\n", disp);
		return NULL;
	}

	if (!bsp_disp_feat_is_support_capture(disp)) {
		DE_INF("screen %d not support capture\n", disp);
		return NULL;
	}

	return &captures[disp];
}

static struct disp_capture_private_data *
disp_capture_get_priv(struct disp_capture *cptr)
{
	if (NULL == cptr) {
		DE_WRN("NULL hdl!\n");
		return NULL;
	}

	if (!bsp_disp_feat_is_support_capture(cptr->disp)) {
		DE_WRN("screen %d not support capture\n", cptr->disp);
		return NULL;
	}

	return &capture_private[cptr->disp];
}

s32 disp_capture_shadow_protect(struct disp_capture *capture, bool protect)
{
	struct disp_capture_private_data *capturep =
	    disp_capture_get_priv(capture);

	if ((NULL == capture) || (NULL == capturep)) {
		DE_WRN("NULL hdl!\n");
		return -1;
	}

	if (capturep->shadow_protect)
		return capturep->shadow_protect(capture->disp, protect);

	return -1;
}

static s32 disp_capture_clk_init(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if ((NULL == cptr) || (NULL == cptrp)) {
		DE_WRN("NULL hdl!\n");
		return 0;
	}

	/*
	 * todo: int the clock
	 */

	return 0;
}

static s32 disp_capture_clk_exit(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if ((NULL == cptr) || (NULL == cptrp)) {
		DE_WRN("NULL hdl!\n");
		return 0;
	}

	/*
	 * todo: colse the clock
	 */

	return 0;
}

static s32 disp_capture_clk_enable(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if ((NULL == cptr) || (NULL == cptrp)) {
		DE_WRN("NULL hdl!\n");
		return 0;
	}

	clk_prepare_enable(cptrp->clk);

	return 0;
}

static s32 disp_capture_clk_disable(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if ((NULL == cptr) || (NULL == cptrp)) {
		DE_WRN("NULL hdl!\n");
		return 0;
	}

	clk_disable(cptrp->clk);

	return 0;
}

s32 disp_capture_apply(struct disp_capture *cptr)
{
	return 0;
}

s32 disp_capture_force_apply(struct disp_capture *cptr)
{
	return 0;
}

s32 disp_capture_start(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if (NULL == cptr || NULL == cptrp) {
		DE_WRN("NULL hdl!\n");
		return -1;
	}
	DE_INF("cap %d\n", cptr->disp);

	mutex_lock(&cptrp->mlock);
	if (1 == cptrp->enabled) {
		DE_WRN("capture %d already started!\n", cptr->disp);
		mutex_unlock(&cptrp->mlock);
		return -1;
	}
	disp_capture_clk_enable(cptr);
	disp_al_capture_init(cptr->disp);
	cptrp->enabled = 1;
	mutex_unlock(&cptrp->mlock);

	return 0;
}

s32 disp_capture_stop(struct disp_capture *cptr)
{
	unsigned long irqflag;
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if (NULL == cptr || NULL == cptrp) {
		DE_WRN("NULL hdl!\n");
		return -1;
	}
	DE_INF("cap %d\n", cptr->disp);

	mutex_lock(&cptrp->mlock);
	if (1 == cptrp->enabled) {
		spin_lock_irqsave(&cptrp->data_lock, irqflag);
		disp_al_capture_exit(cptr->disp);
		disp_capture_clk_disable(cptr);
		cptrp->enabled = 0;
		spin_unlock_irqrestore(&cptrp->data_lock, irqflag);
	}
	mutex_unlock(&cptrp->mlock);

	return 0;
}

s32 disp_capture_commit(struct disp_capture *cptr,
			struct disp_capture_info *info)
{
	/* TODO: compatible width disp_capture */
	return 0;
}

s32 disp_capture_query(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if (NULL == cptr || NULL == cptrp) {
		DE_WRN("NULL hdl!\n");
		return 0;
	}

	return cptrp->status;
}

static int
_setup_capture_config(struct disp_capture *cptr,
						struct disp_capture_config *config,
						struct buffer_slot *slot) {

	struct disp_device *dispdev = NULL;
	enum disp_csc_type cs = DISP_CSC_TYPE_RGB;
	u32 width = 0, height = 0;

	dispdev = cptr->manager->device;
	memset(config, 0, sizeof(struct disp_capture_config));
	config->disp = cptr->disp;

	/*
	 * setup output frame
	 */
	config->out_frame.format  = slot->buf.format;
	config->out_frame.crop    = slot->buf.crop;
	config->out_frame.addr[0] = slot->info.out_frame.addr[0];
	config->out_frame.addr[1] = slot->info.out_frame.addr[1];
	config->out_frame.addr[2] = slot->info.out_frame.addr[2];
	config->out_frame.size[0].width  = slot->buf.width;
	config->out_frame.size[0].height = slot->buf.height;

	/*
	 * setup input frame
	 */
	config->in_frame.crop = slot->buf.window;

	if (dispdev->get_input_csc) {
		cs = dispdev->get_input_csc(dispdev);
	}
	if (DISP_CSC_TYPE_RGB == cs)
		config->in_frame.format = DISP_FORMAT_ARGB_8888;
	else if (DISP_CSC_TYPE_YUV444  == cs)
		config->in_frame.format = DISP_FORMAT_YUV444_P;
	else if (DISP_CSC_TYPE_YUV422 == cs)
		config->in_frame.format = DISP_FORMAT_YUV422_P;
	else
		config->in_frame.format = DISP_FORMAT_YUV420_P;

	if (dispdev->get_resolution) {
		dispdev->get_resolution(dispdev, &width, &height);
	}
	config->in_frame.size[0].width  = width;
	config->in_frame.size[1].width  = width;
	config->in_frame.size[2].width  = width;
	config->in_frame.size[0].height = height;
	config->in_frame.size[1].height = height;
	config->in_frame.size[2].height = height;
	if ((0 == config->in_frame.crop.width)
			|| (0 == config->in_frame.crop.height)) {
		config->in_frame.crop.width = width;
		config->in_frame.crop.height = height;
	}

	return 0;
}

s32 disp_capture_sync(struct disp_capture *cptr)
{
	unsigned long flags, irqflag;
	struct disp_capture_private_data *private = disp_capture_get_priv(cptr);
	struct disp_manager *mgr = NULL;
	struct disp_device *dispdev = NULL;
	static struct timespec prev_timestamp;
	struct buffer_slot *slot = 0;

	if ((NULL == cptr) || (NULL == private)) {
		DE_WRN("NULL hdl!\n");
		return 0;
	}

	mgr = cptr->manager;
	if ((NULL == mgr) || (0 == mgr->is_enabled(mgr))) {
		DE_WRN("mgr is disable!\n");
		return 0;
	}
	dispdev = mgr->device;
	if (NULL == dispdev) {
		DE_WRN("disp device is NULL!\n");
		return 0;
	}

	if (private->writing) {
		spin_lock_irqsave(&private->available_lock, flags);
		private->writing->state = SLOT_READY;
		spin_unlock_irqrestore(&private->available_lock, flags);

		/*
		 * Update timeline to signal the fence binding to this buffer,
		 * So that it can reading by the display engine.
		 */
		sw_sync_timeline_inc(private->timeline, 1);

		ktime_get_ts(&private->writing->timestamp);
		private->period = timespec_sub(private->writing->timestamp, prev_timestamp);
		prev_timestamp = private->writing->timestamp;
		private->writing = NULL;
	}

	private->status = disp_al_capture_get_status(cptr->disp);
	spin_lock_irqsave(&private->data_lock, irqflag);
	if (!private->enabled) {
		spin_unlock_irqrestore(&private->data_lock, irqflag);
		return 0;
	}

	/*
	 * find an empty buffer for next writeback cycle
	 */
	spin_lock_irqsave(&private->empty_lock, flags);
	if (!list_empty(&private->empty_list)) {
		slot = list_entry(private->empty_list.next, struct buffer_slot, list);
		list_del(&slot->list);
	}
	spin_unlock_irqrestore(&private->empty_lock, flags);

	if (!slot) {
		/*
		 * If we can't find any buffer on empty_list, it means that
		 * the consumer is slower than producer. So, pick a buffer
		 * from the available_list.
		 */
		spin_lock_irqsave(&private->available_lock, flags);
		if (!list_empty(&private->available_list)
			&& (private->available_list.next->next != (&private->available_list))) {

			slot = list_entry(private->available_list.next, struct buffer_slot, list);
			list_del(&slot->list);
		}
		spin_unlock_irqrestore(&private->available_lock, flags);
	}

	if (slot) {
		struct disp_capture_config config;

		private->writing = slot;
		private->timeline_max++;
		slot->pt_value = private->timeline_max;
		slot->state = SLOT_WRITING;

		_setup_capture_config(cptr, &config, slot);
		disp_al_capture_apply(cptr->disp, &config);
		disp_al_capture_sync(cptr->disp);

		/*
		 * move the current writeback buffer to the available_list,
		 * and set the buffer state to SLOT_READY.
		 */
		spin_lock_irqsave(&private->available_lock, flags);
		list_add_tail(&slot->list, &private->available_list);
		spin_unlock_irqrestore(&private->available_lock, flags);
	} else {
		pr_err("no empty buffer\n");
	}

	spin_unlock_irqrestore(&private->data_lock, irqflag);
	return 0;
}

s32 disp_capture_set_manager(struct disp_capture *cptr,
		struct disp_manager *mgr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);
	if ((NULL == cptr) || (NULL == mgr)) {
		DE_WRN("NULL hdl!\n");
		return -1;
	}
	mutex_lock(&cptrp->mlock);
	cptr->manager = mgr;
	if (mgr)
		mgr->cptr = cptr;
	mutex_unlock(&cptrp->mlock);
	return 0;
}

s32 disp_capture_unset_manager(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);
	if (NULL == cptr) {
		DE_WRN("NULL hdl!\n");
		return -1;
	}
	mutex_lock(&cptrp->mlock);
	if (cptr->manager)
		cptr->manager->cptr = NULL;
	cptr->manager = NULL;
	mutex_unlock(&cptrp->mlock);
	return 0;
}

static s32 disp_capture_set_fmt(struct disp_capture *cptr, struct disp_capture_fmt *fmt)
{
	struct disp_capture_private_data *private = disp_capture_get_priv(cptr);

	if (!cptr || !private) {
		DE_WRN("capture NULL hdl!\n");
		return -EINVAL;
	}

	if (private->enabled) {
		pr_err("can't reset capture fmt while running\n");
		return -EBUSY;
	}

	mutex_lock(&private->mlock);
	memcpy(&private->fmt, fmt, sizeof(*fmt));
	mutex_unlock(&private->mlock);
	return 0;
}

static int setup_dma_buffer(struct buffer_slot *slot)
{
	struct fb_address_transfer fb;
	struct dmabuf_item *item;

	item = disp_dma_map(slot->buf.fd);
	if (item == NULL) {
		pr_err("disp dma map failed\n");
		return -1;
	}
	fb.format = slot->buf.format;
	memcpy(fb.size, slot->buf.size, sizeof(struct disp_rectsz) * 3);
	fb.dma_addr = item->dma_addr;
	fb.align[0] = 4;
	fb.align[1] = 4;
	fb.align[2] = 4;
	disp_set_fb_info(&fb, true);
	slot->info.out_frame.addr[0] = fb.addr[0];
	slot->info.out_frame.addr[1] = fb.addr[1];
	slot->info.out_frame.addr[2] = fb.addr[2];
	slot->dma_map = item;
	return 0;
}

static s32 disp_capture_buffer_list_init(struct disp_capture *cptr, struct disp_capture_buffer *buf, int size)
{
	int i;
	struct buffer_slot *slot;
	struct disp_capture_private_data *private = disp_capture_get_priv(cptr);

	if (!cptr || !private) {
		DE_WRN("capture NULL hdl!\n");
		return -EINVAL;
	}

	if (private->enabled) {
		pr_err("can't initial capture buffer list while running\n");
		return -EBUSY;
	}

	mutex_lock(&private->mlock);
	INIT_LIST_HEAD(&private->empty_list);
	INIT_LIST_HEAD(&private->available_list);
	spin_lock_init(&private->empty_lock);
	spin_lock_init(&private->available_lock);

	private->buffer_count = 0;
	memset(private->slots, 0, sizeof(struct buffer_slot) * MAX_CAPTURE_LIST_SIZE);
	for (i = 0; i < size; i++, buf++) {
		if (buf->handle < 0 || buf->handle >= MAX_CAPTURE_LIST_SIZE) {
			pr_err("unsupport buffer handle, out of range\n");
			continue;
		}

		slot = &private->slots[buf->handle];

		/*
		 * if the slot had already binding to other buffer,
		 * Just del it from empty_list, And replace with
		 * current buffer.
		 */
		if (slot->state != SLOT_INVALID) {
			list_del(&slot->list);
			pr_err("replace slot: 0x%016llx\n", slot->info.out_frame.addr[0]);
		}

		memcpy(&slot->buf, buf, sizeof(struct disp_capture_buffer));
		setup_dma_buffer(slot);
		slot->state = SLOT_FREE;
		list_add_tail(&slot->list, &private->empty_list);
		private->buffer_count++;
	}
	mutex_unlock(&private->mlock);
	return 0;
}

static s32 disp_capture_buffer_list_clear(struct disp_capture *cptr)
{
	int i;
	unsigned long irqflag;
	struct disp_capture_private_data *private = disp_capture_get_priv(cptr);

	if (!cptr || !private) {
		DE_WRN("capture NULL hdl!\n");
		return -EINVAL;
	}

	if (private->enabled) {
		pr_err("can't clear capture buffer list while running\n");
		return -EBUSY;
	}

	mutex_lock(&private->mlock);
	spin_lock_irqsave(&private->data_lock, irqflag);
	INIT_LIST_HEAD(&private->empty_list);
	INIT_LIST_HEAD(&private->available_list);

	for (i = 0; i < MAX_CAPTURE_LIST_SIZE; i++) {
		if (private->slots[i].dma_map)
			disp_dma_unmap(private->slots[i].dma_map);
	}

	memset(private->slots, 0, sizeof(struct buffer_slot) * MAX_CAPTURE_LIST_SIZE);
	private->buffer_count = 0;
	spin_unlock_irqrestore(&private->data_lock, irqflag);
	mutex_unlock(&private->mlock);
	return 0;
}

static s32 disp_capture_acquire_buffer(struct disp_capture *cptr, struct disp_capture_handle *out)
{
	int fd;
	struct sync_pt *pt;
	struct sync_fence *fence;
	unsigned long flags;
	struct buffer_slot *slot = 0;
	struct disp_capture_private_data *private = disp_capture_get_priv(cptr);

	/*
	 * if can't find any active buffer, just return -1,
	 * it measn no available buffer to server.
	 */
	out->handle = -1;
	out->fencefd = -1;

	if (!cptr || !private) {
		DE_WRN("capture NULL hdl!\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&private->available_lock, flags);
	if (!list_empty(&private->available_list)) {
		slot = list_entry(private->available_list.next, struct buffer_slot, list);
		list_del(&slot->list);
		slot->state = SLOT_READING;
		out->handle = slot->buf.handle;
    }
	spin_unlock_irqrestore(&private->available_lock, flags);

	if (slot) {
		fd = get_unused_fd();
		if (fd < 0) {
			pr_err("get unused fd failed\n");
			goto __err_fd;
		}

		pt = sw_sync_pt_create(private->timeline, slot->pt_value);
		if (pt == NULL) {
			pr_err("cannot create sync point\n");
			goto __err_pt;
		}

		fence = sync_fence_create("sunxi-writeback", pt);
		if (fence == NULL) {
			pr_err("cannot create fence\n");
			goto __err_fence;
		}

		sync_fence_install(fence, fd);
		out->fencefd = fd;
	}
	return 0;

__err_fence:
	sync_pt_free(pt);
__err_pt:
	put_unused_fd(fd);
__err_fd:
	out->fencefd = -1;
	return 0;
}

static s32 disp_capture_release_buffer(struct disp_capture *cptr, struct disp_capture_handle *in)
{
	unsigned long flags;
	struct buffer_slot *slot;
	struct disp_capture_private_data *private = disp_capture_get_priv(cptr);

	if (!cptr || !private) {
		DE_WRN("capture NULL hdl!\n");
		return -EINVAL;
	}

	if (!in || in->handle < 0 || in->handle >= MAX_CAPTURE_LIST_SIZE) {
		pr_err("try to release a invalid handle\n");
		return -EINVAL;
	}

	slot = &private->slots[in->handle];
#if 0
	if (slot->state != SLOT_READING) {
		pr_err("slot state error, expect %d, actually %d\n",
			SLOT_READING, slot->state);
		return -EINVAL;
	}
#endif

	spin_lock_irqsave(&private->empty_lock, flags);
	slot->state = SLOT_FREE;
	list_add_tail(&slot->list, &private->empty_list);
	spin_unlock_irqrestore(&private->empty_lock, flags);
	return 0;
}

s32 disp_capture_suspend(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if ((NULL == cptr) || (NULL == cptrp)) {
		DE_WRN("capture NULL hdl!\n");
		return -1;
	}

	return 0;
}

s32 disp_capture_resume(struct disp_capture *cptr)
{
	struct disp_capture_private_data *cptrp = disp_capture_get_priv(cptr);

	if ((NULL == cptr) || (NULL == cptrp)) {
		DE_WRN("capture NULL hdl!\n");
		return -1;
	}

	return 0;

}

s32 disp_capture_init(struct disp_capture *cptr)
{
	struct disp_capture_private_data *capturep =
	    disp_capture_get_priv(cptr);

	if ((NULL == cptr) || (NULL == capturep)) {
		DE_WRN("capture NULL hdl!\n");
		return -1;
	}

	if (!bsp_disp_feat_is_support_capture(cptr->disp)) {
		DE_WRN("capture %d is not support\n", cptr->disp);
		return -1;
	}

	disp_capture_clk_init(cptr);
	return 0;
}

int disp_capture_dump(struct disp_capture *cptr, char *out)
{
	struct disp_capture_private_data *private = disp_capture_get_priv(cptr);
	struct buffer_slot *writing = private->writing;
	struct buffer_slot *search = &private->slots[0];
	int count = 0;
	uint64_t period = timespec_to_ns(&private->period);

	do_div(period, NSEC_PER_MSEC);
	count += sprintf(out, "\nCapture state: %08x\n", private->status);
	count += sprintf(out + count, "write back period: %lld ms\n", period);
	while (search != &private->slots[private->buffer_count]) {
		count += sprintf(out + count, "  %s[%p] : %016llx %d\n",
					search == writing ? ">" : " ", search, search->info.out_frame.addr[0], search->state);
		search++;
	}
	return count;
}

extern s32 disp_register_ioctl_func(unsigned int cmd, int (*proc)(unsigned int cmd, unsigned long arg));
static int disp_capture_ioctl(unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	unsigned long *ubuf = (unsigned long *)arg;
	int dispid = ubuf[0];
	unsigned int subcmd = ubuf[1];
	int buffer_cnt;
	struct disp_capture *capture;
	struct disp_capture_handle handle;
	struct disp_capture_fmt fmt;
	struct disp_capture_buffer buffers[MAX_CAPTURE_LIST_SIZE];

	if (!bsp_disp_feat_is_support_capture(dispid))
		return -EINVAL;

	capture = &captures[dispid];

	switch (subcmd) {
	case DISP_CAPTURE_E_SET_FMT:
		if (copy_from_user(&fmt, (void __user *)ubuf[2], sizeof(fmt))) {
			retval = -EINVAL;
			goto __out;
		}
		retval = disp_capture_set_fmt(capture, &fmt);
		break;
	case DISP_CAPTURE_E_BUFFER_LIST_INIT:
		buffer_cnt = ubuf[3];
		if (buffer_cnt <= 0 || buffer_cnt >= MAX_CAPTURE_LIST_SIZE) {
			retval = -EINVAL;
			goto __out;
		}
		if (copy_from_user(buffers, (void __user *)ubuf[2],
			sizeof(struct disp_capture_buffer) * buffer_cnt)) {
			retval = -EINVAL;
			goto __out;
		}
		retval = disp_capture_buffer_list_init(capture, buffers, buffer_cnt);
		break;
	case DISP_CAPTURE_E_BUFFER_LIST_CLEAR:
		retval = disp_capture_buffer_list_clear(capture);
		break;
	case DISP_CAPTURE_E_ACQUIRE_BUFFER:
		retval = disp_capture_acquire_buffer(capture, &handle);
		if (retval ||
			copy_to_user((void __user *)ubuf[2], &handle, sizeof(handle))) {
			retval = -EBUSY;
			goto __out;
		}
		break;
	case DISP_CAPTURE_E_RELEASE_BUFFER:
		if (copy_from_user(&handle, (void __user *)ubuf[2], sizeof(handle))) {
			retval = -EINVAL;
			goto __out;
		}
		retval = disp_capture_release_buffer(capture, &handle);
		break;
	case DISP_CAPTURE_E_CTRL:
		if (ubuf[2] == 1)
			retval = disp_capture_start(capture);
		else
			retval = disp_capture_stop(capture);
		break;
	default:
		break;
	}

__out:
	return retval;
}

s32 disp_capture_exit(struct disp_capture *cptr)
{
	if (!bsp_disp_feat_is_support_capture(cptr->disp)) {
		DE_WRN("capture %d is not support\n", cptr->disp);
		return -1;
	}
	disp_capture_clk_exit(cptr);

	return 0;
}

s32 disp_init_capture(disp_bsp_init_para *para)
{
	u32 num_screens;
	u32 capture_id;
	struct disp_capture *capture;
	struct disp_capture_private_data *capturep;
	char timeline_tag[32] = {0};

	num_screens = bsp_disp_feat_get_num_screens();
	captures =
	    (struct disp_capture *)kmalloc(sizeof(struct disp_capture) *
					   num_screens,
					   GFP_KERNEL | __GFP_ZERO);
	if (NULL == captures) {
		DE_WRN("malloc memory fail!\n");
		return DIS_FAIL;
	}
	capture_private =
	    (struct disp_capture_private_data *)
	    kmalloc(sizeof(struct disp_capture_private_data)
		    * num_screens, GFP_KERNEL | __GFP_ZERO);
	if (NULL == capture_private) {
		DE_WRN("malloc memory fail!\n");
		return DIS_FAIL;
	}

	for (capture_id = 0; capture_id < num_screens; capture_id++) {
		if (!bsp_disp_feat_is_support_capture(capture_id))
			continue;

		capture = &captures[capture_id];
		capturep = &capture_private[capture_id];
#if defined(__LINUX_PLAT__)
		mutex_init(&capturep->mlock);
		spin_lock_init(&(capturep->data_lock));
#endif

		capturep->clk = para->mclk[DISP_MOD_DE];
		switch (capture_id) {
		case 0:
			capture->disp = 0;
			capture->name = "capture0";
			break;

		case 1:
			capture->disp = 1;
			capture->name = "capture1";
			break;

		case 2:
			capture->disp = 2;
			capture->name = "capture2";
			break;
		}

		capturep->shadow_protect = para->shadow_protect;
		capture->set_manager = disp_capture_set_manager;
		capture->unset_manager = disp_capture_unset_manager;
		capture->start = disp_capture_start;
		capture->stop = disp_capture_stop;
		capture->sync = disp_capture_sync;
		capture->init = disp_capture_init;
		capture->exit = disp_capture_exit;
		capture->commmit = disp_capture_commit;
		capture->query = disp_capture_query;
		capture->dump = disp_capture_dump;

		sprintf(timeline_tag, "sunxi-capture.%d", capture_id);
		capturep->timeline = sw_sync_timeline_create(timeline_tag);
		capturep->timeline_max = 0;

		disp_capture_init(capture);
	}

	disp_register_ioctl_func(DISP_CAPTURE_EXTEND, disp_capture_ioctl);
	return 0;
}
