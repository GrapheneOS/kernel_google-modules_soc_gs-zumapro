/*
 *  linux/drivers/block/elevator.c
 *
 *  Block device elevator/IO-scheduler.
 *
 *  Copyright (C) 2000 Andrea Arcangeli <andrea@suse.de> SuSE
 *
 * 30042000 Jens Axboe <axboe@suse.de> :
 *
 * Split the elevator a bit so that it is possible to choose a different
 * one or even write a new "plug in". There are three pieces:
 * - elevator_fn, inserts a new request in the queue list
 * - elevator_merge_fn, decides whether a new buffer can be merged with
 *   an existing request
 * - elevator_dequeue_fn, called when a request is taken off the active list
 *
 * 20082000 Dave Jones <davej@suse.de> :
 * Removed tests for max-bomb-segments, which was breaking elvtune
 *  when run without -bN
 *
 * Jens:
 * - Rework again to work with bio instead of buffer_heads
 * - loose bi_dev comparisons, partition handling is right now
 * - completely modularize elevator setup and teardown
 *
 */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/config.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/delay.h>

#include <asm/uaccess.h>

static DEFINE_SPINLOCK(elv_list_lock);
static LIST_HEAD(elv_list);

/*
 * can we safely merge with this request?
 */
inline int elv_rq_merge_ok(struct request *rq, struct bio *bio)
{
	if (!rq_mergeable(rq))
		return 0;

	/*
	 * different data direction or already started, don't merge
	 */
	if (bio_data_dir(bio) != rq_data_dir(rq))
		return 0;

	/*
	 * same device and no special stuff set, merge is ok
	 */
	if (rq->rq_disk == bio->bi_bdev->bd_disk &&
	    !rq->waiting && !rq->special)
		return 1;

	return 0;
}
EXPORT_SYMBOL(elv_rq_merge_ok);

inline int elv_try_merge(struct request *__rq, struct bio *bio)
{
	int ret = ELEVATOR_NO_MERGE;

	/*
	 * we can merge and sequence is ok, check if it's possible
	 */
	if (elv_rq_merge_ok(__rq, bio)) {
		if (__rq->sector + __rq->nr_sectors == bio->bi_sector)
			ret = ELEVATOR_BACK_MERGE;
		else if (__rq->sector - bio_sectors(bio) == bio->bi_sector)
			ret = ELEVATOR_FRONT_MERGE;
	}

	return ret;
}
EXPORT_SYMBOL(elv_try_merge);

static struct elevator_type *elevator_find(const char *name)
{
	struct elevator_type *e = NULL;
	struct list_head *entry;

	list_for_each(entry, &elv_list) {
		struct elevator_type *__e;

		__e = list_entry(entry, struct elevator_type, list);

		if (!strcmp(__e->elevator_name, name)) {
			e = __e;
			break;
		}
	}

	return e;
}

static void elevator_put(struct elevator_type *e)
{
	module_put(e->elevator_owner);
}

static struct elevator_type *elevator_get(const char *name)
{
	struct elevator_type *e;

	spin_lock_irq(&elv_list_lock);

	e = elevator_find(name);
	if (e && !try_module_get(e->elevator_owner))
		e = NULL;

	spin_unlock_irq(&elv_list_lock);

	return e;
}

static int elevator_attach(request_queue_t *q, struct elevator_type *e,
			   struct elevator_queue *eq)
{
	int ret = 0;

	memset(eq, 0, sizeof(*eq));
	eq->ops = &e->ops;
	eq->elevator_type = e;

	q->elevator = eq;

	if (eq->ops->elevator_init_fn)
		ret = eq->ops->elevator_init_fn(q, eq);

	return ret;
}

static char chosen_elevator[16];

static void elevator_setup_default(void)
{
	struct elevator_type *e;

	/*
	 * check if default is set and exists
	 */
	if (chosen_elevator[0] && (e = elevator_get(chosen_elevator))) {
		elevator_put(e);
		return;
	}

#if defined(CONFIG_IOSCHED_AS)
	strcpy(chosen_elevator, "anticipatory");
#elif defined(CONFIG_IOSCHED_DEADLINE)
	strcpy(chosen_elevator, "deadline");
#elif defined(CONFIG_IOSCHED_CFQ)
	strcpy(chosen_elevator, "cfq");
#elif defined(CONFIG_IOSCHED_NOOP)
	strcpy(chosen_elevator, "noop");
#else
#error "You must build at least 1 IO scheduler into the kernel"
#endif
}

static int __init elevator_setup(char *str)
{
	strncpy(chosen_elevator, str, sizeof(chosen_elevator) - 1);
	return 0;
}

__setup("elevator=", elevator_setup);

int elevator_init(request_queue_t *q, char *name)
{
	struct elevator_type *e = NULL;
	struct elevator_queue *eq;
	int ret = 0;

	INIT_LIST_HEAD(&q->queue_head);
	q->last_merge = NULL;
	q->end_sector = 0;
	q->boundary_rq = NULL;

	elevator_setup_default();

	if (!name)
		name = chosen_elevator;

	e = elevator_get(name);
	if (!e)
		return -EINVAL;

	eq = kmalloc(sizeof(struct elevator_queue), GFP_KERNEL);
	if (!eq) {
		elevator_put(e->elevator_type);
		return -ENOMEM;
	}

	ret = elevator_attach(q, e, eq);
	if (ret) {
		kfree(eq);
		elevator_put(e->elevator_type);
	}

	return ret;
}

void elevator_exit(elevator_t *e)
{
	if (e->ops->elevator_exit_fn)
		e->ops->elevator_exit_fn(e);

	elevator_put(e->elevator_type);
	e->elevator_type = NULL;
	kfree(e);
}

/*
 * Insert rq into dispatch queue of q.  Queue lock must be held on
 * entry.  If sort != 0, rq is sort-inserted; otherwise, rq will be
 * appended to the dispatch queue.  To be used by specific elevators.
 */
void elv_dispatch_sort(request_queue_t *q, struct request *rq)
{
	sector_t boundary;
	struct list_head *entry;

	if (q->last_merge == rq)
		q->last_merge = NULL;

	boundary = q->end_sector;

	list_for_each_prev(entry, &q->queue_head) {
		struct request *pos = list_entry_rq(entry);

		if (pos->flags & (REQ_SOFTBARRIER|REQ_HARDBARRIER|REQ_STARTED))
			break;
		if (rq->sector >= boundary) {
			if (pos->sector < boundary)
				continue;
		} else {
			if (pos->sector >= boundary)
				break;
		}
		if (rq->sector >= pos->sector)
			break;
	}

	list_add(&rq->queuelist, entry);
}

int elv_merge(request_queue_t *q, struct request **req, struct bio *bio)
{
	elevator_t *e = q->elevator;
	int ret;

	if (q->last_merge) {
		ret = elv_try_merge(q->last_merge, bio);
		if (ret != ELEVATOR_NO_MERGE) {
			*req = q->last_merge;
			return ret;
		}
	}

	if (e->ops->elevator_merge_fn)
		return e->ops->elevator_merge_fn(q, req, bio);

	return ELEVATOR_NO_MERGE;
}

void elv_merged_request(request_queue_t *q, struct request *rq)
{
	elevator_t *e = q->elevator;

	if (e->ops->elevator_merged_fn)
		e->ops->elevator_merged_fn(q, rq);

	q->last_merge = rq;
}

void elv_merge_requests(request_queue_t *q, struct request *rq,
			     struct request *next)
{
	elevator_t *e = q->elevator;

	if (e->ops->elevator_merge_req_fn)
		e->ops->elevator_merge_req_fn(q, rq, next);

	q->last_merge = rq;
}

void elv_requeue_request(request_queue_t *q, struct request *rq)
{
	elevator_t *e = q->elevator;

	/*
	 * it already went through dequeue, we need to decrement the
	 * in_flight count again
	 */
	if (blk_account_rq(rq)) {
		q->in_flight--;
		if (blk_sorted_rq(rq) && e->ops->elevator_deactivate_req_fn)
			e->ops->elevator_deactivate_req_fn(q, rq);
	}

	rq->flags &= ~REQ_STARTED;

	/*
	 * if this is the flush, requeue the original instead and drop the flush
	 */
	if (rq->flags & REQ_BAR_FLUSH) {
		clear_bit(QUEUE_FLAG_FLUSH, &q->queue_flags);
		rq = rq->end_io_data;
	}

	__elv_add_request(q, rq, ELEVATOR_INSERT_FRONT, 0);
}

void __elv_add_request(request_queue_t *q, struct request *rq, int where,
		       int plug)
{
	if (rq->flags & (REQ_SOFTBARRIER | REQ_HARDBARRIER)) {
		/*
		 * barriers implicitly indicate back insertion
		 */
		if (where == ELEVATOR_INSERT_SORT)
			where = ELEVATOR_INSERT_BACK;

		/*
		 * this request is scheduling boundary, update end_sector
		 */
		if (blk_fs_request(rq)) {
			q->end_sector = rq_end_sector(rq);
			q->boundary_rq = rq;
		}
	} else if (!(rq->flags & REQ_ELVPRIV) && where == ELEVATOR_INSERT_SORT)
		where = ELEVATOR_INSERT_BACK;

	if (plug)
		blk_plug_device(q);

	rq->q = q;

	switch (where) {
	case ELEVATOR_INSERT_FRONT:
		rq->flags |= REQ_SOFTBARRIER;

		list_add(&rq->queuelist, &q->queue_head);
		break;

	case ELEVATOR_INSERT_BACK:
		rq->flags |= REQ_SOFTBARRIER;

		while (q->elevator->ops->elevator_dispatch_fn(q, 1))
			;
		list_add_tail(&rq->queuelist, &q->queue_head);
		/*
		 * We kick the queue here for the following reasons.
		 * - The elevator might have returned NULL previously
		 *   to delay requests and returned them now.  As the
		 *   queue wasn't empty before this request, ll_rw_blk
		 *   won't run the queue on return, resulting in hang.
		 * - Usually, back inserted requests won't be merged
		 *   with anything.  There's no point in delaying queue
		 *   processing.
		 */
		blk_remove_plug(q);
		q->request_fn(q);
		break;

	case ELEVATOR_INSERT_SORT:
		BUG_ON(!blk_fs_request(rq));
		rq->flags |= REQ_SORTED;
		q->elevator->ops->elevator_add_req_fn(q, rq);
		if (q->last_merge == NULL && rq_mergeable(rq))
			q->last_merge = rq;
		break;

	default:
		printk(KERN_ERR "%s: bad insertion point %d\n",
		       __FUNCTION__, where);
		BUG();
	}

	if (blk_queue_plugged(q)) {
		int nrq = q->rq.count[READ] + q->rq.count[WRITE]
			- q->in_flight;

		if (nrq >= q->unplug_thresh)
			__generic_unplug_device(q);
	}
}

void elv_add_request(request_queue_t *q, struct request *rq, int where,
		     int plug)
{
	unsigned long flags;

	spin_lock_irqsave(q->queue_lock, flags);
	__elv_add_request(q, rq, where, plug);
	spin_unlock_irqrestore(q->queue_lock, flags);
}

static inline struct request *__elv_next_request(request_queue_t *q)
{
	struct request *rq;

	if (unlikely(list_empty(&q->queue_head) &&
		     !q->elevator->ops->elevator_dispatch_fn(q, 0)))
		return NULL;

	rq = list_entry_rq(q->queue_head.next);

	/*
	 * if this is a barrier write and the device has to issue a
	 * flush sequence to support it, check how far we are
	 */
	if (blk_fs_request(rq) && blk_barrier_rq(rq)) {
		BUG_ON(q->ordered == QUEUE_ORDERED_NONE);

		if (q->ordered == QUEUE_ORDERED_FLUSH &&
		    !blk_barrier_preflush(rq))
			rq = blk_start_pre_flush(q, rq);
	}

	return rq;
}

struct request *elv_next_request(request_queue_t *q)
{
	struct request *rq;
	int ret;

	while ((rq = __elv_next_request(q)) != NULL) {
		if (!(rq->flags & REQ_STARTED)) {
			elevator_t *e = q->elevator;

			/*
			 * This is the first time the device driver
			 * sees this request (possibly after
			 * requeueing).  Notify IO scheduler.
			 */
			if (blk_sorted_rq(rq) &&
			    e->ops->elevator_activate_req_fn)
				e->ops->elevator_activate_req_fn(q, rq);

			/*
			 * just mark as started even if we don't start
			 * it, a request that has been delayed should
			 * not be passed by new incoming requests
			 */
			rq->flags |= REQ_STARTED;
		}

		if (!q->boundary_rq || q->boundary_rq == rq) {
			q->end_sector = rq_end_sector(rq);
			q->boundary_rq = NULL;
		}

		if ((rq->flags & REQ_DONTPREP) || !q->prep_rq_fn)
			break;

		ret = q->prep_rq_fn(q, rq);
		if (ret == BLKPREP_OK) {
			break;
		} else if (ret == BLKPREP_DEFER) {
			/*
			 * the request may have been (partially) prepped.
			 * we need to keep this request in the front to
			 * avoid resource deadlock.  REQ_STARTED will
			 * prevent other fs requests from passing this one.
			 */
			rq = NULL;
			break;
		} else if (ret == BLKPREP_KILL) {
			int nr_bytes = rq->hard_nr_sectors << 9;

			if (!nr_bytes)
				nr_bytes = rq->data_len;

			blkdev_dequeue_request(rq);
			rq->flags |= REQ_QUIET;
			end_that_request_chunk(rq, 0, nr_bytes);
			end_that_request_last(rq);
		} else {
			printk(KERN_ERR "%s: bad return=%d\n", __FUNCTION__,
								ret);
			break;
		}
	}

	return rq;
}

void elv_dequeue_request(request_queue_t *q, struct request *rq)
{
	BUG_ON(list_empty(&rq->queuelist));

	list_del_init(&rq->queuelist);

	/*
	 * the time frame between a request being removed from the lists
	 * and to it is freed is accounted as io that is in progress at
	 * the driver side.
	 */
	if (blk_account_rq(rq))
		q->in_flight++;
}

int elv_queue_empty(request_queue_t *q)
{
	elevator_t *e = q->elevator;

	if (!list_empty(&q->queue_head))
		return 0;

	if (e->ops->elevator_queue_empty_fn)
		return e->ops->elevator_queue_empty_fn(q);

	return 1;
}

struct request *elv_latter_request(request_queue_t *q, struct request *rq)
{
	struct list_head *next;

	elevator_t *e = q->elevator;

	if (e->ops->elevator_latter_req_fn)
		return e->ops->elevator_latter_req_fn(q, rq);

	next = rq->queuelist.next;
	if (next != &q->queue_head && next != &rq->queuelist)
		return list_entry_rq(next);

	return NULL;
}

struct request *elv_former_request(request_queue_t *q, struct request *rq)
{
	struct list_head *prev;

	elevator_t *e = q->elevator;

	if (e->ops->elevator_former_req_fn)
		return e->ops->elevator_former_req_fn(q, rq);

	prev = rq->queuelist.prev;
	if (prev != &q->queue_head && prev != &rq->queuelist)
		return list_entry_rq(prev);

	return NULL;
}

int elv_set_request(request_queue_t *q, struct request *rq, struct bio *bio,
		    gfp_t gfp_mask)
{
	elevator_t *e = q->elevator;

	if (e->ops->elevator_set_req_fn)
		return e->ops->elevator_set_req_fn(q, rq, bio, gfp_mask);

	rq->elevator_private = NULL;
	return 0;
}

void elv_put_request(request_queue_t *q, struct request *rq)
{
	elevator_t *e = q->elevator;

	if (e->ops->elevator_put_req_fn)
		e->ops->elevator_put_req_fn(q, rq);
}

int elv_may_queue(request_queue_t *q, int rw, struct bio *bio)
{
	elevator_t *e = q->elevator;

	if (e->ops->elevator_may_queue_fn)
		return e->ops->elevator_may_queue_fn(q, rw, bio);

	return ELV_MQUEUE_MAY;
}

void elv_completed_request(request_queue_t *q, struct request *rq)
{
	elevator_t *e = q->elevator;

	/*
	 * request is released from the driver, io must be done
	 */
	if (blk_account_rq(rq)) {
		q->in_flight--;
		if (blk_sorted_rq(rq) && e->ops->elevator_completed_req_fn)
			e->ops->elevator_completed_req_fn(q, rq);
	}
}

int elv_register_queue(struct request_queue *q)
{
	elevator_t *e = q->elevator;

	e->kobj.parent = kobject_get(&q->kobj);
	if (!e->kobj.parent)
		return -EBUSY;

	snprintf(e->kobj.name, KOBJ_NAME_LEN, "%s", "iosched");
	e->kobj.ktype = e->elevator_type->elevator_ktype;

	return kobject_register(&e->kobj);
}

void elv_unregister_queue(struct request_queue *q)
{
	if (q) {
		elevator_t *e = q->elevator;
		kobject_unregister(&e->kobj);
		kobject_put(&q->kobj);
	}
}

int elv_register(struct elevator_type *e)
{
	spin_lock_irq(&elv_list_lock);
	if (elevator_find(e->elevator_name))
		BUG();
	list_add_tail(&e->list, &elv_list);
	spin_unlock_irq(&elv_list_lock);

	printk(KERN_INFO "io scheduler %s registered", e->elevator_name);
	if (!strcmp(e->elevator_name, chosen_elevator))
		printk(" (default)");
	printk("\n");
	return 0;
}
EXPORT_SYMBOL_GPL(elv_register);

void elv_unregister(struct elevator_type *e)
{
	spin_lock_irq(&elv_list_lock);
	list_del_init(&e->list);
	spin_unlock_irq(&elv_list_lock);
}
EXPORT_SYMBOL_GPL(elv_unregister);

/*
 * switch to new_e io scheduler. be careful not to introduce deadlocks -
 * we don't free the old io scheduler, before we have allocated what we
 * need for the new one. this way we have a chance of going back to the old
 * one, if the new one fails init for some reason.
 */
static void elevator_switch(request_queue_t *q, struct elevator_type *new_e)
{
	elevator_t *old_elevator, *e;

	/*
	 * Allocate new elevator
	 */
	e = kmalloc(sizeof(elevator_t), GFP_KERNEL);
	if (!e)
		goto error;

	/*
	 * Turn on BYPASS and drain all requests w/ elevator private data
	 */
	spin_lock_irq(q->queue_lock);

	set_bit(QUEUE_FLAG_ELVSWITCH, &q->queue_flags);

	while (q->elevator->ops->elevator_dispatch_fn(q, 1))
		;

	while (q->rq.elvpriv) {
		spin_unlock_irq(q->queue_lock);
		msleep(10);
		spin_lock_irq(q->queue_lock);
	}

	spin_unlock_irq(q->queue_lock);

	/*
	 * unregister old elevator data
	 */
	elv_unregister_queue(q);
	old_elevator = q->elevator;

	/*
	 * attach and start new elevator
	 */
	if (elevator_attach(q, new_e, e))
		goto fail;

	if (elv_register_queue(q))
		goto fail_register;

	/*
	 * finally exit old elevator and turn off BYPASS.
	 */
	elevator_exit(old_elevator);
	clear_bit(QUEUE_FLAG_ELVSWITCH, &q->queue_flags);
	return;

fail_register:
	/*
	 * switch failed, exit the new io scheduler and reattach the old
	 * one again (along with re-adding the sysfs dir)
	 */
	elevator_exit(e);
	e = NULL;
fail:
	q->elevator = old_elevator;
	elv_register_queue(q);
	clear_bit(QUEUE_FLAG_ELVSWITCH, &q->queue_flags);
	kfree(e);
error:
	elevator_put(new_e);
	printk(KERN_ERR "elevator: switch to %s failed\n",new_e->elevator_name);
}

ssize_t elv_iosched_store(request_queue_t *q, const char *name, size_t count)
{
	char elevator_name[ELV_NAME_MAX];
	struct elevator_type *e;

	memset(elevator_name, 0, sizeof(elevator_name));
	strncpy(elevator_name, name, sizeof(elevator_name));

	if (elevator_name[strlen(elevator_name) - 1] == '\n')
		elevator_name[strlen(elevator_name) - 1] = '\0';

	e = elevator_get(elevator_name);
	if (!e) {
		printk(KERN_ERR "elevator: type %s not found\n", elevator_name);
		return -EINVAL;
	}

	if (!strcmp(elevator_name, q->elevator->elevator_type->elevator_name))
		return count;

	elevator_switch(q, e);
	return count;
}

ssize_t elv_iosched_show(request_queue_t *q, char *name)
{
	elevator_t *e = q->elevator;
	struct elevator_type *elv = e->elevator_type;
	struct list_head *entry;
	int len = 0;

	spin_lock_irq(q->queue_lock);
	list_for_each(entry, &elv_list) {
		struct elevator_type *__e;

		__e = list_entry(entry, struct elevator_type, list);
		if (!strcmp(elv->elevator_name, __e->elevator_name))
			len += sprintf(name+len, "[%s] ", elv->elevator_name);
		else
			len += sprintf(name+len, "%s ", __e->elevator_name);
	}
	spin_unlock_irq(q->queue_lock);

	len += sprintf(len+name, "\n");
	return len;
}

EXPORT_SYMBOL(elv_dispatch_sort);
EXPORT_SYMBOL(elv_add_request);
EXPORT_SYMBOL(__elv_add_request);
EXPORT_SYMBOL(elv_requeue_request);
EXPORT_SYMBOL(elv_next_request);
EXPORT_SYMBOL(elv_dequeue_request);
EXPORT_SYMBOL(elv_queue_empty);
EXPORT_SYMBOL(elv_completed_request);
EXPORT_SYMBOL(elevator_exit);
EXPORT_SYMBOL(elevator_init);
