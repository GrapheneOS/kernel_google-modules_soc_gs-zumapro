// SPDX-License-Identifier: GPL-2.0-only
/* sysfs_node.c
 *
 * Android Vendor Hook Support
 *
 * Copyright 2020 Google LLC
 */

#include <linux/kobject.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/sched/task.h>
#include <linux/sysfs.h>
#include <linux/lockdep.h>
#include <kernel/sched/sched.h>

#include "sched.h"

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
extern void reset_uclamp_stats(void);
DECLARE_PER_CPU(struct uclamp_stats, uclamp_stats);
#endif

unsigned int __read_mostly vendor_sched_uclamp_threshold;
unsigned int __read_mostly vendor_sched_util_threshold = DEF_UTIL_THRESHOLD;
unsigned int __read_mostly vendor_sched_high_capacity_start_cpu = MAX_CAPACITY_CPU;
unsigned int __read_mostly vendor_sched_util_post_init_scale = DEF_UTIL_POST_INIT_SCALE;
static struct kobject *vendor_sched_kobj;
static struct proc_dir_entry *vendor_sched;

extern void update_sched_capacity_margin(unsigned int util_threshold);
extern void initialize_vendor_group_property(void);
extern void rvh_uclamp_eff_get_pixel_mod(void *data, struct task_struct *p, enum uclamp_id clamp_id,
					 struct uclamp_se *uclamp_max, struct uclamp_se *uclamp_eff,
					 int *ret);

extern struct vendor_group_property *get_vendor_group_property(enum vendor_group group);
static void apply_uclamp_change(enum vendor_group group, enum uclamp_id clamp_id);

static struct uclamp_se uclamp_default[UCLAMP_CNT];

#define SET_TASK_GROUP_STORE(__grp, __vg)						      \
		static ssize_t set_task_group_##__grp##_store (struct kobject *kobj,	      \
							       struct kobj_attribute *attr,   \
							       const char *buf, size_t count) \
		{									      \
			int ret = update_vendor_task_attribute(buf, VTA_GROUP, __vg);	      \
			return ret ?: count;						      \
		}									      \
		static struct kobj_attribute set_task_group_##__grp##_attribute =	      \
							__ATTR_WO(set_task_group_##__grp);


#define VENDOR_GROUP_BOOL_ATTRIBUTE(__grp, __attr, __vg)				      \
		static ssize_t __grp##_##__attr##_show(struct kobject *kobj,		      \
						       struct kobj_attribute *attr,char *buf) \
		{									      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			return scnprintf(buf, PAGE_SIZE, "%s\n",			\
					gp->__attr==true? "true":"false");		      \
		}									      \
		static ssize_t __grp##_##__attr##_store (struct kobject *kobj,		      \
							 struct kobj_attribute *attr,	      \
							 const char *buf, size_t count)	      \
		{									      \
			bool val;							      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			if (kstrtobool(buf, &val))					      \
				return -EINVAL;						      \
			gp->__attr = val;						      \
			return count;							      \
		}									      \
		static struct kobj_attribute __grp##_##__attr##_##attribute =		      \
							__ATTR_RW(__grp##_##__attr);

#define VENDOR_GROUP_UINT_ATTRIBUTE(__grp, __attr, __vg)				      \
		static ssize_t __grp##_##__attr##_show(struct kobject *kobj,		      \
						       struct kobj_attribute *attr,char *buf) \
		{									      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			return scnprintf(buf, PAGE_SIZE, "%u\n",	gp->__attr);	\
		}									      \
		static ssize_t __grp##_##__attr##_store (struct kobject *kobj,		      \
							 struct kobj_attribute *attr,	      \
							 const char *buf, size_t count)	      \
		{									      \
			unsigned int val;					\
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			if (kstrtouint(buf, 10, &val))					      \
				return -EINVAL;						      \
			gp->__attr = val;						      \
			return count;							      \
		}									      \
		static struct kobj_attribute __grp##_##__attr##_##attribute =		      \
							__ATTR_RW(__grp##_##__attr);

#define VENDOR_GROUP_UCLAMP_ATTRIBUTE(__grp, __attr, __vg, __cid)			      \
		static ssize_t __grp##_##__attr##_show(struct kobject *kobj,		      \
						       struct kobj_attribute *attr,char *buf) \
		{									      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			return sprintf(buf, "%u\n", gp->uc_req[__cid].value);		      \
		}									      \
		static ssize_t __grp##_##__attr##_store (struct kobject *kobj,		      \
							 struct kobj_attribute *attr,	      \
							 const char *buf, size_t count)	      \
		{									      \
			unsigned int val;						      \
			struct vendor_group_property *gp = get_vendor_group_property(__vg);   \
			if (kstrtouint(buf, 0, &val))					      \
				return -EINVAL;						      \
			if (val > 1024)							      \
				return -EINVAL;						      \
			gp->uc_req[__cid].value = val;					      \
			gp->uc_req[__cid].bucket_id = get_bucket_id(val);		      \
			gp->uc_req[__cid].user_defined = false;				      \
			apply_uclamp_change(__vg, __cid);				      \
			return count;							      \
		}									      \
		static struct kobj_attribute __grp##_##__attr##_##attribute =		      \
							__ATTR_RW(__grp##_##__attr);

#define DUMP_TASK_GROUP_SHOW(__grp, __vg)						      \
		static int dump_task_group_##__grp##_show(struct seq_file *m, void *v)	      \
		{									      \
			return dump_vendor_group_task(m, v, __vg);	      \
		}

#define CREATE_DUMP_FILE(__grp)	\
		proc_create_single("dump_task_group_"#__grp,	\
				   0, vendor_sched, dump_task_group_##__grp##_show)

/// ******************************************************************************** ///
/// ********************* Create vendor group sysfs nodes*************************** ///
/// ******************************************************************************** ///

VENDOR_GROUP_BOOL_ATTRIBUTE(ta, prefer_idle, VG_TOPAPP);
VENDOR_GROUP_BOOL_ATTRIBUTE(ta, prefer_high_cap, VG_TOPAPP);
VENDOR_GROUP_BOOL_ATTRIBUTE(ta, task_spreading, VG_TOPAPP);
VENDOR_GROUP_UINT_ATTRIBUTE(ta, group_throttle, VG_TOPAPP);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(ta, uclamp_min, VG_TOPAPP, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(ta, uclamp_max, VG_TOPAPP, UCLAMP_MAX);

VENDOR_GROUP_BOOL_ATTRIBUTE(fg, prefer_idle, VG_FOREGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(fg, prefer_high_cap, VG_FOREGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(fg, task_spreading, VG_FOREGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(fg, group_throttle, VG_FOREGROUND);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(fg, uclamp_min, VG_FOREGROUND, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(fg, uclamp_max, VG_FOREGROUND, UCLAMP_MAX);

VENDOR_GROUP_BOOL_ATTRIBUTE(sys, prefer_idle, VG_SYSTEM);
VENDOR_GROUP_BOOL_ATTRIBUTE(sys, prefer_high_cap, VG_SYSTEM);
VENDOR_GROUP_BOOL_ATTRIBUTE(sys, task_spreading, VG_SYSTEM);
VENDOR_GROUP_UINT_ATTRIBUTE(sys, group_throttle, VG_SYSTEM);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sys, uclamp_min, VG_SYSTEM, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sys, uclamp_max, VG_SYSTEM, UCLAMP_MAX);

VENDOR_GROUP_BOOL_ATTRIBUTE(cam, prefer_idle, VG_CAMERA);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam, prefer_high_cap, VG_CAMERA);
VENDOR_GROUP_BOOL_ATTRIBUTE(cam, task_spreading, VG_CAMERA);
VENDOR_GROUP_UINT_ATTRIBUTE(cam, group_throttle, VG_CAMERA);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(cam, uclamp_min, VG_CAMERA, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(cam, uclamp_max, VG_CAMERA, UCLAMP_MAX);

VENDOR_GROUP_BOOL_ATTRIBUTE(bg, prefer_idle, VG_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(bg, prefer_high_cap, VG_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(bg, task_spreading, VG_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(bg, group_throttle, VG_BACKGROUND);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(bg, uclamp_min, VG_BACKGROUND, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(bg, uclamp_max, VG_BACKGROUND, UCLAMP_MAX);

VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, prefer_idle, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, prefer_high_cap, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_BOOL_ATTRIBUTE(sysbg, task_spreading, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UINT_ATTRIBUTE(sysbg, group_throttle, VG_SYSTEM_BACKGROUND);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sysbg, uclamp_min, VG_SYSTEM_BACKGROUND, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(sysbg, uclamp_max, VG_SYSTEM_BACKGROUND, UCLAMP_MAX);

VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, prefer_idle, VG_NNAPI_HAL);
VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, prefer_high_cap, VG_NNAPI_HAL);
VENDOR_GROUP_BOOL_ATTRIBUTE(nnapi, task_spreading, VG_NNAPI_HAL);
VENDOR_GROUP_UINT_ATTRIBUTE(nnapi, group_throttle, VG_NNAPI_HAL);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(nnapi, uclamp_min, VG_NNAPI_HAL, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(nnapi, uclamp_max, VG_NNAPI_HAL, UCLAMP_MAX);

VENDOR_GROUP_BOOL_ATTRIBUTE(rt, prefer_idle, VG_RT);
VENDOR_GROUP_BOOL_ATTRIBUTE(rt, prefer_high_cap, VG_RT);
VENDOR_GROUP_BOOL_ATTRIBUTE(rt, task_spreading, VG_RT);
VENDOR_GROUP_UINT_ATTRIBUTE(rt, group_throttle, VG_RT);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(rt, uclamp_min, VG_RT, UCLAMP_MIN);
VENDOR_GROUP_UCLAMP_ATTRIBUTE(rt, uclamp_max, VG_RT, UCLAMP_MAX);

/// ******************************************************************************** ///
/// ********************* From upstream code for uclamp **************************** ///
/// ******************************************************************************** ///
static inline unsigned int uclamp_none(enum uclamp_id clamp_id)
{
	if (clamp_id == UCLAMP_MIN)
		return 0;
	return SCHED_CAPACITY_SCALE;
}

static inline unsigned int
uclamp_idle_value(struct rq *rq, enum uclamp_id clamp_id,
		  unsigned int clamp_value)
{
	/*
	 * Avoid blocked utilization pushing up the frequency when we go
	 * idle (which drops the max-clamp) by retaining the last known
	 * max-clamp.
	 */
	if (clamp_id == UCLAMP_MAX) {
		rq->uclamp_flags |= UCLAMP_FLAG_IDLE;
		return clamp_value;
	}

	return uclamp_none(UCLAMP_MIN);
}

static inline void uclamp_idle_reset(struct rq *rq, enum uclamp_id clamp_id,
				     unsigned int clamp_value)
{
	/* Reset max-clamp retention only on idle exit */
	if (!(rq->uclamp_flags & UCLAMP_FLAG_IDLE))
		return;

	WRITE_ONCE(rq->uclamp[clamp_id].value, clamp_value);
}

static inline
unsigned int uclamp_rq_max_value(struct rq *rq, enum uclamp_id clamp_id,
				   unsigned int clamp_value)
{
	struct uclamp_bucket *bucket = rq->uclamp[clamp_id].bucket;
	int bucket_id = UCLAMP_BUCKETS - 1;

	/*
	 * Since both min and max clamps are max aggregated, find the
	 * top most bucket with tasks in.
	 */
	for ( ; bucket_id >= 0; bucket_id--) {
		if (!bucket[bucket_id].tasks)
			continue;
		return bucket[bucket_id].value;
	}

	/* No tasks -- default clamp values */
	return uclamp_idle_value(rq, clamp_id, clamp_value);
}

/*
 * The effective clamp bucket index of a task depends on, by increasing
 * priority:
 * - the task specific clamp value, when explicitly requested from userspace
 * - the task group effective clamp value, for tasks not either in the root
 *   group or in an autogroup
 * - the system default clamp value, defined by the sysadmin
 */

static inline struct uclamp_se
uclamp_tg_restrict(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct uclamp_se uc_req = p->uclamp_req[clamp_id];
#ifdef CONFIG_UCLAMP_TASK_GROUP
	struct uclamp_se uc_max;

	/*
	 * Tasks in autogroups or root task group will be
	 * restricted by system defaults.
	 */
	if (task_group_is_autogroup(task_group(p)))
		return uc_req;
	if (task_group(p) == &root_task_group)
		return uc_req;

	uc_max = task_group(p)->uclamp[clamp_id];
	if (uc_req.value > uc_max.value || !uc_req.user_defined)
		return uc_max;
#endif

	return uc_req;
}

/*
 * The effective clamp bucket index of a task depends on, by increasing
 * priority:
 * - the task specific clamp value, when explicitly requested from userspace
 * - the task group effective clamp value, for tasks not either in the root
 *   group or in an autogroup
 * - the system default clamp value, defined by the sysadmin
 */
static inline struct uclamp_se
uclamp_eff_get(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct uclamp_se uc_req = uclamp_tg_restrict(p, clamp_id);
	struct uclamp_se uc_max = uclamp_default[clamp_id];
	struct uclamp_se uc_eff;
	int ret = 0;

	// Instead of calling trace_android_*, call vendor func directly
	rvh_uclamp_eff_get_pixel_mod(NULL, p, clamp_id, &uc_max, &uc_eff, &ret);
	if (ret)
		return uc_eff;

	/* System default restrictions always apply */
	if (unlikely(uc_req.value > uc_max.value))
		return uc_max;

	return uc_req;
}

/*
 * When a task is enqueued on a rq, the clamp bucket currently defined by the
 * task's uclamp::bucket_id is refcounted on that rq. This also immediately
 * updates the rq's clamp value if required.
 *
 * Tasks can have a task-specific value requested from user-space, track
 * within each bucket the maximum value for tasks refcounted in it.
 * This "local max aggregation" allows to track the exact "requested" value
 * for each bucket when all its RUNNABLE tasks require the same clamp.
 */
static inline void uclamp_rq_inc_id(struct rq *rq, struct task_struct *p,
				    enum uclamp_id clamp_id)
{
	struct uclamp_rq *uc_rq = &rq->uclamp[clamp_id];
	struct uclamp_se *uc_se = &p->uclamp[clamp_id];
	struct uclamp_bucket *bucket;

	lockdep_assert_held(&rq->lock);

	/* Update task effective clamp */
	p->uclamp[clamp_id] = uclamp_eff_get(p, clamp_id);

	bucket = &uc_rq->bucket[uc_se->bucket_id];
	bucket->tasks++;
	uc_se->active = true;

	uclamp_idle_reset(rq, clamp_id, uc_se->value);

	/*
	 * Local max aggregation: rq buckets always track the max
	 * "requested" clamp value of its RUNNABLE tasks.
	 */
	if (bucket->tasks == 1 || uc_se->value > bucket->value)
		bucket->value = uc_se->value;

	if (uc_se->value > READ_ONCE(uc_rq->value))
		WRITE_ONCE(uc_rq->value, uc_se->value);
}

/*
 * When a task is dequeued from a rq, the clamp bucket refcounted by the task
 * is released. If this is the last task reference counting the rq's max
 * active clamp value, then the rq's clamp value is updated.
 *
 * Both refcounted tasks and rq's cached clamp values are expected to be
 * always valid. If it's detected they are not, as defensive programming,
 * enforce the expected state and warn.
 */
static inline void uclamp_rq_dec_id(struct rq *rq, struct task_struct *p,
				    enum uclamp_id clamp_id)
{
	struct uclamp_rq *uc_rq = &rq->uclamp[clamp_id];
	struct uclamp_se *uc_se = &p->uclamp[clamp_id];
	struct uclamp_bucket *bucket;
	unsigned int bkt_clamp;
	unsigned int rq_clamp;

	lockdep_assert_held(&rq->lock);

	/*
	 * If sched_uclamp_used was enabled after task @p was enqueued,
	 * we could end up with unbalanced call to uclamp_rq_dec_id().
	 *
	 * In this case the uc_se->active flag should be false since no uclamp
	 * accounting was performed at enqueue time and we can just return
	 * here.
	 *
	 * Need to be careful of the following enqeueue/dequeue ordering
	 * problem too
	 *
	 *	enqueue(taskA)
	 *	// sched_uclamp_used gets enabled
	 *	enqueue(taskB)
	 *	dequeue(taskA)
	 *	// Must not decrement bukcet->tasks here
	 *	dequeue(taskB)
	 *
	 * where we could end up with stale data in uc_se and
	 * bucket[uc_se->bucket_id].
	 *
	 * The following check here eliminates the possibility of such race.
	 */
	if (unlikely(!uc_se->active))
		return;

	bucket = &uc_rq->bucket[uc_se->bucket_id];

	SCHED_WARN_ON(!bucket->tasks);
	if (likely(bucket->tasks))
		bucket->tasks--;

	uc_se->active = false;

	/*
	 * Keep "local max aggregation" simple and accept to (possibly)
	 * overboost some RUNNABLE tasks in the same bucket.
	 * The rq clamp bucket value is reset to its base value whenever
	 * there are no more RUNNABLE tasks refcounting it.
	 */
	if (likely(bucket->tasks))
		return;

	rq_clamp = READ_ONCE(uc_rq->value);
	/*
	 * Defensive programming: this should never happen. If it happens,
	 * e.g. due to future modification, warn and fixup the expected value.
	 */
	SCHED_WARN_ON(bucket->value > rq_clamp);
	if (bucket->value >= rq_clamp) {
		bkt_clamp = uclamp_rq_max_value(rq, clamp_id, uc_se->value);
		WRITE_ONCE(uc_rq->value, bkt_clamp);
	}
}

static inline void
uclamp_update_active(struct task_struct *p, enum uclamp_id clamp_id)
{
	struct rq_flags rf;
	struct rq *rq;

	/*
	 * Lock the task and the rq where the task is (or was) queued.
	 *
	 * We might lock the (previous) rq of a !RUNNABLE task, but that's the
	 * price to pay to safely serialize util_{min,max} updates with
	 * enqueues, dequeues and migration operations.
	 * This is the same locking schema used by __set_cpus_allowed_ptr().
	 */
	rq = task_rq_lock(p, &rf);

	/*
	 * Setting the clamp bucket is serialized by task_rq_lock().
	 * If the task is not yet RUNNABLE and its task_struct is not
	 * affecting a valid clamp bucket, the next time it's enqueued,
	 * it will already see the updated clamp bucket value.
	 */
	if (p->uclamp[clamp_id].active) {
		uclamp_rq_dec_id(rq, p, clamp_id);
		uclamp_rq_inc_id(rq, p, clamp_id);

		if (rq->uclamp_flags & UCLAMP_FLAG_IDLE)
			rq->uclamp_flags &= ~UCLAMP_FLAG_IDLE;
	}

	task_rq_unlock(rq, p, &rf);
}

/// ******************************************************************************** ///
/// ********************* New code section ***************************************** ///
/// ******************************************************************************** ///

static void apply_uclamp_change(enum vendor_group group, enum uclamp_id clamp_id)
{
	struct task_struct *p, *t;
	struct vendor_task_struct *vp;

	rcu_read_lock();

	for_each_process_thread(p, t) {
		vp = get_vendor_task_struct(t);
		if (t->on_rq && vp->group == group) {
			uclamp_update_active(t, clamp_id);
		}
	}

	rcu_read_unlock();
}

static int update_vendor_task_attribute(const char *buf,
					enum vendor_task_attribute vta, unsigned int val)
{
	struct vendor_task_struct *vp;
	struct task_struct *p;
	enum uclamp_id clamp_id;

	pid_t pid;

	if (kstrtoint(buf, 0, &pid) || pid <= 0)
		return -EINVAL;

	rcu_read_lock();
	p = find_task_by_vpid(pid);
	if (!p) {
		rcu_read_unlock();
		return -ESRCH;
	}

	get_task_struct(p);
	vp = get_vendor_task_struct(p);

	switch (vta) {
	case VTA_GROUP:
		vp->group = val;
		for (clamp_id = 0; clamp_id < UCLAMP_CNT; clamp_id++)
			uclamp_update_active(p, clamp_id);
		break;
	default:
		break;
	}
	rcu_read_unlock();

	put_task_struct(p);

	return 0;
}

static int dump_vendor_group_task(struct seq_file *m, void *v, unsigned int val)
{
	struct task_struct *p, *t;
	struct vendor_task_struct *vp;

	rcu_read_lock();

	for_each_process_thread(p, t) {
		vp = get_vendor_task_struct(t);
		if (vp->group == val) {
			seq_printf(m, "%u\n", t->pid);
		}
	}

	rcu_read_unlock();

	return 0;
}

SET_TASK_GROUP_STORE(ta, VG_TOPAPP);
SET_TASK_GROUP_STORE(fg, VG_FOREGROUND);
SET_TASK_GROUP_STORE(sys, VG_SYSTEM);
SET_TASK_GROUP_STORE(cam, VG_CAMERA);
SET_TASK_GROUP_STORE(bg, VG_BACKGROUND);
SET_TASK_GROUP_STORE(sysbg, VG_SYSTEM_BACKGROUND);
SET_TASK_GROUP_STORE(nnapi, VG_NNAPI_HAL);
SET_TASK_GROUP_STORE(rt, VG_RT);

DUMP_TASK_GROUP_SHOW(ta, VG_TOPAPP);
DUMP_TASK_GROUP_SHOW(fg, VG_FOREGROUND);
DUMP_TASK_GROUP_SHOW(sys, VG_SYSTEM);
DUMP_TASK_GROUP_SHOW(cam, VG_CAMERA);
DUMP_TASK_GROUP_SHOW(bg, VG_BACKGROUND);
DUMP_TASK_GROUP_SHOW(sysbg, VG_SYSTEM_BACKGROUND);
DUMP_TASK_GROUP_SHOW(nnapi, VG_NNAPI_HAL);
DUMP_TASK_GROUP_SHOW(rt, VG_RT);

static ssize_t clear_group_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t count)
{
	int ret = update_vendor_task_attribute(buf, VTA_GROUP, VG_SYSTEM);

	return ret ?: count;
}
static struct kobj_attribute clear_group_attribute = __ATTR_WO(clear_group);

static ssize_t uclamp_threshold_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vendor_sched_uclamp_threshold);
}

static ssize_t uclamp_threshold_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val > SCHED_CAPACITY_SCALE)
		return -EINVAL;

	vendor_sched_uclamp_threshold = val;

	return count;
}

static struct kobj_attribute uclamp_threshold_attribute = __ATTR_RW(uclamp_threshold);

static ssize_t util_threshold_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vendor_sched_util_threshold);
}

static ssize_t util_threshold_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	vendor_sched_util_threshold = val;
	update_sched_capacity_margin(val);

	return count;
}

static struct kobj_attribute util_threshold_attribute = __ATTR_RW(util_threshold);

#if IS_ENABLED(CONFIG_UCLAMP_STATS)
static ssize_t uclamp_stats_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	int i, j, index;
	struct uclamp_stats *stats;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len, "V, T(ms), %%\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);
		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - total time: %llu ms\n",
				 i, stats->total_time / NSEC_PER_MSEC);
		len += scnprintf(buf + len, PAGE_SIZE - len, "uclamp.min\n");
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %d%%\n", index,
					stats->time_in_state_min[j] / NSEC_PER_MSEC,
					stats->time_in_state_min[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}

		len += scnprintf(buf + len, PAGE_SIZE - len, "uclamp.max\n");
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %d%%\n", index,
					stats->time_in_state_max[j] / NSEC_PER_MSEC,
					stats->time_in_state_max[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}
	}

	return len;
}

static struct kobj_attribute uclamp_stats_attribute = __ATTR_RO(uclamp_stats);

static ssize_t uclamp_effective_stats_show(struct kobject *kobj, struct kobj_attribute *attr,
					   char *buf)
{
	int i, j, index;
	struct uclamp_stats *stats;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len, "V, T(ms), %%(Based on T in uclamp_stats)\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);

		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d\n", i);
		len += scnprintf(buf + len, PAGE_SIZE - len, "uclamp.min\n");
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->effect_time_in_state_min[j] / NSEC_PER_MSEC,
					stats->effect_time_in_state_min[j] /
					(stats->time_in_state_min[j] / 100));
			if (len >= PAGE_SIZE)
				break;
		}

		len += scnprintf(buf + len, PAGE_SIZE - len, "uclamp.max\n");
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->effect_time_in_state_max[j] / NSEC_PER_MSEC,
					stats->effect_time_in_state_max[j] /
					(stats->time_in_state_max[j] / 100));
			if (len >= PAGE_SIZE)
				break;
		}
	}

	return len;
}

static struct kobj_attribute uclamp_effective_stats_attribute = __ATTR_RO(uclamp_effective_stats);

static ssize_t uclamp_util_diff_stats_show(struct kobject *kobj, struct kobj_attribute *attr,
					   char *buf)
{
	int i, j, index;
	struct uclamp_stats *stats;
	ssize_t len = 0;

	len += scnprintf(buf + len, PAGE_SIZE - len, "V, T(ms), %%\n");
	for (i = 0; i < CONFIG_VH_SCHED_CPU_NR; i++) {
		stats = &per_cpu(uclamp_stats, i);
		len += scnprintf(buf + len, PAGE_SIZE - len, "CPU %d - total time: %llu ms\n",
				 i, stats->total_time / NSEC_PER_MSEC);
		len += scnprintf(buf + len, PAGE_SIZE - len, "util_diff_min\n");
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index += UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->util_diff_min[j] / NSEC_PER_MSEC,
					stats->util_diff_min[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}

		len += scnprintf(buf + len, PAGE_SIZE - len, "util_diff_max\n");
		if (len >= PAGE_SIZE)
			break;
		for (j = 0, index = 0; j < UCLAMP_STATS_SLOTS; j++, index -= UCLAMP_STATS_STEP) {
			len += scnprintf(buf + len, PAGE_SIZE - len, "%d, %llu, %llu%%\n", index,
					stats->util_diff_max[j] / NSEC_PER_MSEC,
					stats->util_diff_max[j] / (stats->total_time / 100));
			if (len >= PAGE_SIZE)
				break;
		}
	}

	return len;
}

static struct kobj_attribute uclamp_util_diff_stats_attribute = __ATTR_RO(uclamp_util_diff_stats);


static ssize_t reset_uclamp_stats_store(struct kobject *kobj, struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	bool reset;

	if (kstrtobool(buf, &reset))
		return -EINVAL;

	if (reset)
		reset_uclamp_stats();

	return count;
}

static struct kobj_attribute reset_uclamp_stats_attribute = __ATTR_WO(reset_uclamp_stats);
#endif


static ssize_t high_capacity_start_cpu_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vendor_sched_high_capacity_start_cpu);
}

static ssize_t high_capacity_start_cpu_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val != MID_CAPACITY_CPU && val != MAX_CAPACITY_CPU)
		return -EINVAL;

	vendor_sched_high_capacity_start_cpu = val;

	return count;
}

static struct kobj_attribute high_capacity_start_cpu_attribute = __ATTR_RW(high_capacity_start_cpu);

static ssize_t util_post_init_scale_show(struct kobject *kobj,
					struct kobj_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vendor_sched_util_post_init_scale);
}

static ssize_t util_post_init_scale_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	unsigned int val;

	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	if (val > 1024)
		return -EINVAL;

	vendor_sched_util_post_init_scale = val;

	return count;
}

static struct kobj_attribute util_post_init_scale_attribute = __ATTR_RW(util_post_init_scale);

static struct attribute *attrs[] = {
	// Topapp group attributes
	&ta_prefer_idle_attribute.attr,
	&ta_prefer_high_cap_attribute.attr,
	&ta_task_spreading_attribute.attr,
	&ta_group_throttle_attribute.attr,
	&ta_uclamp_min_attribute.attr,
	&ta_uclamp_max_attribute.attr,
	// Foreground group attributes
	&fg_prefer_idle_attribute.attr,
	&fg_prefer_high_cap_attribute.attr,
	&fg_task_spreading_attribute.attr,
	&fg_group_throttle_attribute.attr,
	&fg_uclamp_min_attribute.attr,
	&fg_uclamp_max_attribute.attr,
	// System group attributes
	&sys_prefer_idle_attribute.attr,
	&sys_prefer_high_cap_attribute.attr,
	&sys_task_spreading_attribute.attr,
	&sys_group_throttle_attribute.attr,
	&sys_uclamp_min_attribute.attr,
	&sys_uclamp_max_attribute.attr,
	// Camera group attributes
	&cam_prefer_idle_attribute.attr,
	&cam_prefer_high_cap_attribute.attr,
	&cam_task_spreading_attribute.attr,
	&cam_group_throttle_attribute.attr,
	&cam_uclamp_min_attribute.attr,
	&cam_uclamp_max_attribute.attr,
	// Background group attributes
	&bg_prefer_idle_attribute.attr,
	&bg_prefer_high_cap_attribute.attr,
	&bg_task_spreading_attribute.attr,
	&bg_group_throttle_attribute.attr,
	&bg_uclamp_min_attribute.attr,
	&bg_uclamp_max_attribute.attr,
	// System Background group attributes
	&sysbg_prefer_idle_attribute.attr,
	&sysbg_prefer_high_cap_attribute.attr,
	&sysbg_task_spreading_attribute.attr,
	&sysbg_group_throttle_attribute.attr,
	&sysbg_uclamp_min_attribute.attr,
	&sysbg_uclamp_max_attribute.attr,
	// Nnapi-HAL group attributes
	&nnapi_prefer_idle_attribute.attr,
	&nnapi_prefer_high_cap_attribute.attr,
	&nnapi_task_spreading_attribute.attr,
	&nnapi_group_throttle_attribute.attr,
	&nnapi_uclamp_min_attribute.attr,
	&nnapi_uclamp_max_attribute.attr,
	// RT group attributes
	&rt_prefer_idle_attribute.attr,
	&rt_prefer_high_cap_attribute.attr,
	&rt_task_spreading_attribute.attr,
	&rt_group_throttle_attribute.attr,
	&rt_uclamp_min_attribute.attr,
	&rt_uclamp_max_attribute.attr,
	// Vendor task attributes
	&set_task_group_ta_attribute.attr,
	&set_task_group_fg_attribute.attr,
	&set_task_group_sys_attribute.attr,
	&set_task_group_cam_attribute.attr,
	&set_task_group_bg_attribute.attr,
	&set_task_group_sysbg_attribute.attr,
	&set_task_group_nnapi_attribute.attr,
	&set_task_group_rt_attribute.attr,
	&clear_group_attribute.attr,
	// Uclamp stats
#if IS_ENABLED(CONFIG_UCLAMP_STATS)
	&uclamp_stats_attribute.attr,
	&uclamp_effective_stats_attribute.attr,
	&uclamp_util_diff_stats_attribute.attr,
	&reset_uclamp_stats_attribute.attr,
#endif
	&uclamp_threshold_attribute.attr,
	&util_threshold_attribute.attr,
	&high_capacity_start_cpu_attribute.attr,
	&util_post_init_scale_attribute.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};


static int create_procfs_node(void)
{
	vendor_sched = proc_mkdir("vendor_sched", NULL);

	if (!vendor_sched)
		return -ENOMEM;

	if (!CREATE_DUMP_FILE(ta)) {
		goto out;
	}

	if (!CREATE_DUMP_FILE(fg)) {
		goto out;
	}

	if (!CREATE_DUMP_FILE(sys)) {
		goto out;
	}

	if (!CREATE_DUMP_FILE(cam)) {
		goto out;
	}

	if (!CREATE_DUMP_FILE(bg)) {
		goto out;
	}

	if (!CREATE_DUMP_FILE(sysbg)) {
		goto out;
	}

	if (!CREATE_DUMP_FILE(nnapi)) {
		goto out;
	}

	if (!CREATE_DUMP_FILE(rt)) {
		goto out;
	}

	return 0;

out:
	remove_proc_entry("vendor_sched", NULL);
	return -ENOMEM;
}

int create_sysfs_node(void)
{
	int ret;
	struct uclamp_se uc_max = {};
	enum uclamp_id clamp_id;

	vendor_sched_kobj = kobject_create_and_add("vendor_sched", kernel_kobj);

	if (!vendor_sched_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(vendor_sched_kobj, &attr_group);
	if (ret)
		goto out;

	ret = create_procfs_node();
	if (ret)
		goto out;

	uc_max.value = uclamp_none(UCLAMP_MAX);
	uc_max.bucket_id = get_bucket_id(uc_max.value);
	uc_max.user_defined = false;
	for (clamp_id = 0; clamp_id < UCLAMP_CNT; clamp_id++) {
		uclamp_default[clamp_id] = uc_max;
	}

	initialize_vendor_group_property();

	return ret;

out:
	kobject_put(vendor_sched_kobj);
	return -ENOMEM;
}
