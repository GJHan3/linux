// SPDX-License-Identifier: GPL-2.0
/*
 * Block rq-qos base io controller
 *
 * This works similar to wbt with a few exceptions
 *
 * - It's bio based, so the latency covers the whole block layer in addition to
 *   the actual io.
 * - We will throttle all IO that comes in here if we need to.
 * - We use the mean latency over the 100ms window.  This is because writes can
 *   be particularly fast, which could give us a false sense of the impact of
 *   other workloads on our protected workload.
 * - By default there's no throttling, we set the queue_depth to UINT_MAX so
 *   that we can have as many outstanding bio's as we're allowed to.  Only at
 *   throttle time do we pay attention to the actual queue depth.
 *
 * The hierarchy works like the cpu controller does, we track the latency at
 * every configured node, and each configured node has it's own independent
 * queue depth.  This means that we only care about our latency targets at the
 * peer level.  Some group at the bottom of the hierarchy isn't going to affect
 * a group at the end of some other path if we're only configred at leaf level.
 *
 * Consider the following
 *
 *                   root blkg
 *             /                     \
 *        fast (target=5ms)     slow (target=10ms)
 *         /     \                  /        \
 *       a        b          normal(15ms)   unloved
 *
 * "a" and "b" have no target, but their combined io under "fast" cannot exceed
 * an average latency of 5ms.  If it does then we will throttle the "slow"
 * group.  In the case of "normal", if it exceeds its 15ms target, we will
 * throttle "unloved", but nobody else.
 *
 * In this example "fast", "slow", and "normal" will be the only groups actually
 * accounting their io latencies.  We have to walk up the heirarchy to the root
 * on every submit and complete so we can do the appropriate stat recording and
 * adjust the queue depth of ourselves if needed.
 *
 * There are 2 ways we throttle IO.
 *
 * 1) Queue depth throttling.  As we throttle down we will adjust the maximum
 * number of IO's we're allowed to have in flight.  This starts at (u64)-1 down
 * to 1.  If the group is only ever submitting IO for itself then this is the
 * only way we throttle.
 *
 * 2) Induced delay throttling.  This is for the case that a group is generating
 * IO that has to be issued by the root cg to avoid priority inversion. So think
 * REQ_META or REQ_SWAP.  If we are already at qd == 1 and we're getting a lot
 * of work done for us on behalf of the root cg and are being asked to scale
 * down more then we induce a latency at userspace return.  We accumulate the
 * total amount of time we need to be punished by doing
 *
 * total_time += min_lat_nsec - actual_io_completion
 *
 * and then at throttle time will do
 *
 * throttle_time = min(total_time, NSEC_PER_SEC)
 *
 * This induced delay will throttle back the activity that is generating the
 * root cg issued io's, wethere that's some metadata intensive operation or the
 * group is using so much memory that it is pushing us into swap.
 *
 * Copyright (C) 2018 Josef Bacik
 */
#include <linux/kernel.h>
#include <linux/blk_types.h>
#include <linux/backing-dev.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/memcontrol.h>
#include <linux/sched/loadavg.h>
#include <linux/sched/signal.h>
#include <trace/events/block.h>
#include <linux/blk-mq.h>
#include "blk-rq-qos.h"
#include "blk-stat.h"
#include "blk.h"

#define DEFAULT_SCALE_COOKIE 1000000U

static struct blkcg_policy blkcg_policy_iolatency;
struct iolatency_grp;

struct blk_iolatency {
	struct rq_qos rqos;
	struct timer_list timer;
	atomic_t enabled;
};

static inline struct blk_iolatency *BLKIOLATENCY(struct rq_qos *rqos)
{
	return container_of(rqos, struct blk_iolatency, rqos);
}

static inline bool blk_iolatency_enabled(struct blk_iolatency *blkiolat)
{
	return atomic_read(&blkiolat->enabled) > 0;
}

struct child_latency_info {
	spinlock_t lock;

	/* Last time we adjusted the scale of everybody. */
	u64 last_scale_event;

	/* The latency that we missed. */
	u64 scale_lat;

	/* Total io's from all of our children for the last summation. */
	u64 nr_samples;

	/* The guy who actually changed the latency numbers. */
	struct iolatency_grp *scale_grp;

	/* Cookie to tell if we need to scale up or down. */
	atomic_t scale_cookie;
};

struct percentile_stats {
	u64 total;
	u64 missed;
};

struct latency_stat {
	union {
		struct percentile_stats ps;
		struct blk_rq_stat rqs;
	};
};

struct iolatency_grp {
	struct blkg_policy_data pd;
	struct latency_stat __percpu *stats;
	struct latency_stat cur_stat;
	struct blk_iolatency *blkiolat;
	struct rq_depth rq_depth;
	struct rq_wait rq_wait;
	atomic64_t window_start;
	atomic_t scale_cookie;
	u64 min_lat_nsec;
	u64 cur_win_nsec;

	/* total running average of our io latency. */
	u64 lat_avg;

	/* Our current number of IO's for the last summation. */
	u64 nr_samples;

	bool ssd;
	struct child_latency_info child_lat;
};

#define BLKIOLATENCY_MIN_WIN_SIZE (100 * NSEC_PER_MSEC)
#define BLKIOLATENCY_MAX_WIN_SIZE NSEC_PER_SEC
/*
 * These are the constants used to fake the fixed-point moving average
 * calculation just like load average.  The call to calc_load() folds
 * (FIXED_1 (2048) - exp_factor) * new_sample into lat_avg.  The sampling
 * window size is bucketed to try to approximately calculate average
 * latency such that 1/exp (decay rate) is [1 min, 2.5 min) when windows
 * elapse immediately.  Note, windows only elapse with IO activity.  Idle
 * periods extend the most recent window.
 */
#define BLKIOLATENCY_NR_EXP_FACTORS 5
#define BLKIOLATENCY_EXP_BUCKET_SIZE (BLKIOLATENCY_MAX_WIN_SIZE / \
				      (BLKIOLATENCY_NR_EXP_FACTORS - 1))
static const u64 iolatency_exp_factors[BLKIOLATENCY_NR_EXP_FACTORS] = {
	2045, // exp(1/600) - 600 samples
	2039, // exp(1/240) - 240 samples
	2031, // exp(1/120) - 120 samples
	2023, // exp(1/80)  - 80 samples
	2014, // exp(1/60)  - 60 samples
};

static inline struct iolatency_grp *pd_to_lat(struct blkg_policy_data *pd)
{
	return pd ? container_of(pd, struct iolatency_grp, pd) : NULL;
}

static inline struct iolatency_grp *blkg_to_lat(struct blkcg_gq *blkg)
{
	return pd_to_lat(blkg_to_pd(blkg, &blkcg_policy_iolatency));
}

static inline struct blkcg_gq *lat_to_blkg(struct iolatency_grp *iolat)
{
	return pd_to_blkg(&iolat->pd);
}

static inline void latency_stat_init(struct iolatency_grp *iolat,
				     struct latency_stat *stat)
{
	if (iolat->ssd) {
		stat->ps.total = 0;
		stat->ps.missed = 0;
	} else
		blk_rq_stat_init(&stat->rqs);
}

static inline void latency_stat_sum(struct iolatency_grp *iolat,
				    struct latency_stat *sum,
				    struct latency_stat *stat)
{
	if (iolat->ssd) {
		sum->ps.total += stat->ps.total;
		sum->ps.missed += stat->ps.missed;
	} else
		blk_rq_stat_sum(&sum->rqs, &stat->rqs);
}

static inline void latency_stat_record_time(struct iolatency_grp *iolat,
					    u64 req_time)
{
	struct latency_stat *stat = get_cpu_ptr(iolat->stats);
	if (iolat->ssd) {
		if (req_time >= iolat->min_lat_nsec)
			stat->ps.missed++;
		stat->ps.total++;
	} else
		blk_rq_stat_add(&stat->rqs, req_time);
	put_cpu_ptr(stat);
}

static inline bool latency_sum_ok(struct iolatency_grp *iolat,
				  struct latency_stat *stat)
{
	if (iolat->ssd) {
		/* miss io占总io超过1/10即触发 */
		u64 thresh = div64_u64(stat->ps.total, 10);
		thresh = max(thresh, 1ULL);
		return stat->ps.missed < thresh;
	}
	return stat->rqs.mean <= iolat->min_lat_nsec;
}

static inline u64 latency_stat_samples(struct iolatency_grp *iolat,
				       struct latency_stat *stat)
{
	if (iolat->ssd)
		return stat->ps.total;
	return stat->rqs.nr_samples;
}

static inline void iolat_update_total_lat_avg(struct iolatency_grp *iolat,
					      struct latency_stat *stat)
{
	int exp_idx;

	if (iolat->ssd)
		return;

	/*
	 * calc_load() takes in a number stored in fixed point representation.
	 * Because we are using this for IO time in ns, the values stored
	 * are significantly larger than the FIXED_1 denominator (2048).
	 * Therefore, rounding errors in the calculation are negligible and
	 * can be ignored.
	 */
	exp_idx = min_t(int, BLKIOLATENCY_NR_EXP_FACTORS - 1,
			div64_u64(iolat->cur_win_nsec,
				  BLKIOLATENCY_EXP_BUCKET_SIZE));
	iolat->lat_avg = calc_load(iolat->lat_avg,
				   iolatency_exp_factors[exp_idx],
				   stat->rqs.mean);
}

static void iolat_cleanup_cb(struct rq_wait *rqw, void *private_data)
{
	atomic_dec(&rqw->inflight);
	wake_up(&rqw->wait);
}

static bool iolat_acquire_inflight(struct rq_wait *rqw, void *private_data)
{
	struct iolatency_grp *iolat = private_data;
	return rq_wait_inc_below(rqw, iolat->rq_depth.max_depth);
}

static void __blkcg_iolatency_throttle(struct rq_qos *rqos,
				       struct iolatency_grp *iolat,
				       bool issue_as_root,
				       bool use_memdelay)
{
	struct rq_wait *rqw = &iolat->rq_wait;
	unsigned use_delay = atomic_read(&lat_to_blkg(iolat)->use_delay);

	/* 若使用delay直接调用blkcg_schedule_throttle */
	if (use_delay)
		blkcg_schedule_throttle(rqos->q, use_memdelay);

	/*
	 * To avoid priority inversions we want to just take a slot if we are
	 * issuing as root.  If we're being killed off there's no point in
	 * delaying things, we may have been killed by OOM so throttling may
	 * make recovery take even longer, so just let the IO's through so the
	 * task can go away.
	 */
	if (issue_as_root || fatal_signal_pending(current)) {
		atomic_inc(&rqw->inflight);
		return;
	}
	/* 这里是调度点 
	 * inflight < max_depth 时执行，并增加inflight 
	 * inflight >= max_depth 时 io_schedule() 
	 */
	rq_qos_wait(rqw, iolat, iolat_acquire_inflight, iolat_cleanup_cb);
}

#define SCALE_DOWN_FACTOR 2
#define SCALE_UP_FACTOR 4

static inline unsigned long scale_amount(unsigned long qd, bool up)
{
	return max(up ? qd >> SCALE_UP_FACTOR : qd >> SCALE_DOWN_FACTOR, 1UL);
}

/*
 * We scale the qd down faster than we scale up, so we need to use this helper
 * to adjust the scale_cookie accordingly so we don't prematurely get
 * scale_cookie at DEFAULT_SCALE_COOKIE and unthrottle too much.
 *
 * Each group has their own local copy of the last scale cookie they saw, so if
 * the global scale cookie goes up or down they know which way they need to go
 * based on their last knowledge of it.
 */
static void scale_cookie_change(struct blk_iolatency *blkiolat,
				struct child_latency_info *lat_info,
				bool up)
{
	unsigned long qd = blkiolat->rqos.q->nr_requests;
	/* 增加则是nr_requests的1/16，减少是nr_requests的1/4， 最少变化为1*/
	unsigned long scale = scale_amount(qd, up);
	unsigned long old = atomic_read(&lat_info->scale_cookie);
	unsigned long max_scale = qd << 1;
	unsigned long diff = 0;

	if (old < DEFAULT_SCALE_COOKIE)
		diff = DEFAULT_SCALE_COOKIE - old;

	/* 是否为增加 */
	if (up) {
		/* 增加的值 + scale_amount 大于默认值，则取默认值*/
		if (scale + old > DEFAULT_SCALE_COOKIE)
			atomic_set(&lat_info->scale_cookie,
				   DEFAULT_SCALE_COOKIE);
		else if (diff > qd)
			/* 对lat_info->scale_cookie + 1*/
			atomic_inc(&lat_info->scale_cookie);
		else
			atomic_add(scale, &lat_info->scale_cookie);
	} else {
		/*
		 * We don't want to dig a hole so deep that it takes us hours to
		 * dig out of it.  Just enough that we don't throttle/unthrottle
		 * with jagged workloads but can still unthrottle once pressure
		 * has sufficiently dissipated.
		 */
		/* 如果差值大于queue的最大request数*/
		if (diff > qd) {
			/* 小于4倍的scale, 只减1 */
			if (diff < max_scale)
				atomic_dec(&lat_info->scale_cookie);
		} else {
			/* 如果差值不大，则只减scale. */
			atomic_sub(scale, &lat_info->scale_cookie);
		}
	}
}

/*
 * Change the queue depth of the iolatency_grp.  We add/subtract 1/16th of the
 * queue depth at a time so we don't get wild swings and hopefully dial in to
 * fairer distribution of the overall queue depth.
 */
static void scale_change(struct iolatency_grp *iolat, bool up)
{
	unsigned long qd = iolat->blkiolat->rqos.q->nr_requests;
	unsigned long scale = scale_amount(qd, up);
	unsigned long old = iolat->rq_depth.max_depth;

	/* 最大设置为queue Max Request数量, 默认128， 和设备相关 */
	if (old > qd)
		old = qd;

	/* 如果是增加 */
	if (up) {
		if (old == 1 && blkcg_unuse_delay(lat_to_blkg(iolat)))
			return;

		if (old < qd) {
			/* 增加 scale */
			old += scale;
			old = min(old, qd);
			/* 重点: 这里关联了max_depth与scale */
			iolat->rq_depth.max_depth = old;
			wake_up_all(&iolat->rq_wait.wait);
		}
	} else {
		/* 重点: 这里关联了max_depth与scale */
		old >>= 1;
		/* 缩小一半 */
		iolat->rq_depth.max_depth = max(old, 1UL);
	}
	/* max_depth有什么用呢？
	 * 见rq_qos_wait(rqw, iolat, iolat_acquire_inflight, iolat_cleanup_cb)
	 */
}

/* Check our parent and see if the scale cookie has changed. */
static void check_scale_change(struct iolatency_grp *iolat)
{
	struct iolatency_grp *parent;
	struct child_latency_info *lat_info;
	unsigned int cur_cookie;
	/* 读取当前节点的scale_cookie */
	unsigned int our_cookie = atomic_read(&iolat->scale_cookie);
	u64 scale_lat;
	unsigned int old;
	int direction = 0;
	/* 如果已经到根了， 则直接返回 */
	if (lat_to_blkg(iolat)->parent == NULL)
		return;
	/* 找到parent的iolatency结构体 */
	parent = blkg_to_lat(lat_to_blkg(iolat)->parent);
	/* 如果parent没有就直接返回 */
	if (!parent)
		return;
	/* 找到父节点对应的子节点iolatency信息 */
	lat_info = &parent->child_lat;
	/* 读取父节点cookie */
	cur_cookie = atomic_read(&lat_info->scale_cookie);
	/* The latency that we missed. */
	scale_lat = READ_ONCE(lat_info->scale_lat);
	/* 这个cookie存的是什么？ */
	/* 与父节点的cookie对比，比较大小，看是否增加depth或者使用delay */
	/* 父节点cookie < 当前cgroup cookie*/
	if (cur_cookie < our_cookie)
		direction = -1;
	/* 父节点cookie > 当前cgroup cookie*/
	else if (cur_cookie > our_cookie)
		direction = 1;
	else
		return;
	/* 到这里，说明两者的cookie不相等 */
	/* 将父节点的child lat info中cookie放到当前iocgroup中 */
	/* 问题：如果都不能满足latency是怎么办呢？？*/
	old = atomic_cmpxchg(&iolat->scale_cookie, our_cookie, cur_cookie);

	/* Somebody beat us to the punch, just bail. */
	if (old != our_cookie)
		return;

	/* 当前值小了，并且设置了min_lat_nsec */
	if (direction < 0 && iolat->min_lat_nsec) {
		u64 samples_thresh;
		/* 重点：这里看当前cgroup是否延迟<=父节点管理的子cgroup们, 即需要解决的延迟目标cgroup不参与限流策略 */
		if (!scale_lat || iolat->min_lat_nsec <= scale_lat)
			return;

		/*
		 * Sometimes high priority groups are their own worst enemy, so
		 * instead of taking it out on some poor other group that did 5%
		 * or less of the IO's for the last summation just skip this
		 * scale down event.
		 */
		/* Total io's from all of our children for the last summation. */
		samples_thresh = lat_info->nr_samples * 5;
		samples_thresh = max(1ULL, div64_u64(samples_thresh, 100));
		if (iolat->nr_samples <= samples_thresh)
			return;
	}

	/* 如果depth已经低到1， 并且想要继续低，那么不可能了， 用delay方法 */
	/* We're as low as we can go. */
	if (iolat->rq_depth.max_depth == 1 && direction < 0) {
		/* 标记阻塞，在__cgroup_throttle_swaprate用到、禁止readahead */
		blkcg_use_delay(lat_to_blkg(iolat));
		return;
	}

	/* 如果回到DEFAULT_SCALE_COOKIE， 那么将rq_depth、delay都取消*/
	/* We're back to the default cookie, unthrottle all the things. */
	if (cur_cookie == DEFAULT_SCALE_COOKIE) {
		blkcg_clear_delay(lat_to_blkg(iolat));
		iolat->rq_depth.max_depth = UINT_MAX;
		/* 激活限流等待 */
		wake_up_all(&iolat->rq_wait.wait);
		return;
	}
	/* 这里是通过修改rq_depth.max_depth来限流了 */
	/* direction > 0 增加depth */
	/* direction < 0 减小depth */
	scale_change(iolat, direction > 0);
}

static void blkcg_iolatency_throttle(struct rq_qos *rqos, struct bio *bio)
{
	struct blk_iolatency *blkiolat = BLKIOLATENCY(rqos);
	struct blkcg_gq *blkg = bio->bi_blkg;
	bool issue_as_root = bio_issue_as_root_blkg(bio);

	if (!blk_iolatency_enabled(blkiolat))
		return;

	while (blkg && blkg->parent) {
		/* 找到对应cgroup的iolatency结构体 */
		struct iolatency_grp *iolat = blkg_to_lat(blkg);
		/* 如果当前cgroup没有iolatency结构体，就一直访问parent */
		if (!iolat) {
			blkg = blkg->parent;
			continue;
		}
		/* 待定做什么： 更新rq_depth.max_depth与delay是否启用 
		 * 好像只有这里更新了 */
		check_scale_change(iolat);
		__blkcg_iolatency_throttle(rqos, iolat, issue_as_root,
				     (bio->bi_opf & REQ_SWAP) == REQ_SWAP);
		blkg = blkg->parent;
	}

	/* 定时器，1s后调用 blkiolatency_timer_fn */
	if (!timer_pending(&blkiolat->timer))
		mod_timer(&blkiolat->timer, jiffies + HZ);
}

static void iolatency_record_time(struct iolatency_grp *iolat,
				  struct bio_issue *issue, u64 now,
				  bool issue_as_root)
{
	u64 start = bio_issue_time(issue);
	u64 req_time;

	/*
	 * Have to do this so we are truncated to the correct time that our
	 * issue is truncated to.
	 */
	now = __bio_issue_time(now);

	if (now <= start)
		return;

	req_time = now - start;

	/*
	 * We don't want to count issue_as_root bio's in the cgroups latency
	 * statistics as it could skew the numbers downwards.
	 */
	if (unlikely(issue_as_root && iolat->rq_depth.max_depth != UINT_MAX)) {
		u64 sub = iolat->min_lat_nsec;
		if (req_time < sub)
			blkcg_add_delay(lat_to_blkg(iolat), now, sub - req_time);
		return;
	}

	latency_stat_record_time(iolat, req_time);
}

#define BLKIOLATENCY_MIN_ADJUST_TIME (500 * NSEC_PER_MSEC)
#define BLKIOLATENCY_MIN_GOOD_SAMPLES 5

static void iolatency_check_latencies(struct iolatency_grp *iolat, u64 now)
{
	struct blkcg_gq *blkg = lat_to_blkg(iolat);
	struct iolatency_grp *parent;
	struct child_latency_info *lat_info;
	struct latency_stat stat;
	unsigned long flags;
	int cpu;

	latency_stat_init(iolat, &stat);
	preempt_disable();
	for_each_online_cpu(cpu) {
		struct latency_stat *s;
		/* 将当前block cgroup的per cpu统计值加到stat中 */
		s = per_cpu_ptr(iolat->stats, cpu);
		latency_stat_sum(iolat, &stat, s);
		latency_stat_init(iolat, s);
	}
	preempt_enable();

	parent = blkg_to_lat(blkg->parent);
	if (!parent)
		return;

	lat_info = &parent->child_lat;

	iolat_update_total_lat_avg(iolat, &stat);

	/* Everything is ok and we don't need to adjust the scale. */
	/* 不需要调整 */
	if (latency_sum_ok(iolat, &stat) &&
	    atomic_read(&lat_info->scale_cookie) == DEFAULT_SCALE_COOKIE)
		return;

	/* Somebody beat us to the punch, just bail. */
	spin_lock_irqsave(&lat_info->lock, flags);

	/* iolat->cur_stat记录当前cgroup本次调整周期的数据 */
	latency_stat_sum(iolat, &iolat->cur_stat, &stat);
	/* lat_info 记录cgroup所有children group的总io量
	 * iolat->nr_samples记录本group的io量
	 */
	lat_info->nr_samples -= iolat->nr_samples;
	lat_info->nr_samples += latency_stat_samples(iolat, &iolat->cur_stat);
	iolat->nr_samples = latency_stat_samples(iolat, &iolat->cur_stat);
	/* 已经被更新过了 或者没有到调整周期500ns，则直接跳过调整 */
	if ((lat_info->last_scale_event >= now ||
	    now - lat_info->last_scale_event < BLKIOLATENCY_MIN_ADJUST_TIME))
		goto out;
	/* miss io占总io超过1/10即触发 并且刚才这一段时间也是miss io超过1/10才触发 */
	if (latency_sum_ok(iolat, &iolat->cur_stat) &&
	    latency_sum_ok(iolat, &stat)) {
		/* 这次时间窗口的io数量样本不够 */
		if (latency_stat_samples(iolat, &iolat->cur_stat) <
		    BLKIOLATENCY_MIN_GOOD_SAMPLES)
			goto out;
		/* 这里表示，父节点限流了，并且出发限流的原因是当前cgroup
		 * 但是现在已经完成了限流任务，那么就需要增加当前父节点的cookie
		 */
		if (lat_info->scale_grp == iolat) {
			lat_info->last_scale_event = now;
			/* 后续父节点下的子cgroup都将根据这个cookie调整max_depth */
			scale_cookie_change(iolat->blkiolat, lat_info, true);
		}
	} else if (lat_info->scale_lat == 0 ||
		   lat_info->scale_lat >= iolat->min_lat_nsec) {
		lat_info->last_scale_event = now;
		/* 如果父节点所管理的延迟大于当前cgroup */
		if (!lat_info->scale_grp ||
		    lat_info->scale_lat > iolat->min_lat_nsec) {
			/* 将本cgroup的最小延迟更新到父亲节点 */
			WRITE_ONCE(lat_info->scale_lat, iolat->min_lat_nsec);
			/* 父节点标记上一次触发延迟目标的是当前cgroup */
			lat_info->scale_grp = iolat;
		}
		/* 开始对这个父节点进行限流，后续父节点下的子cgroup都将根据这个cookie调整max_depth */
		scale_cookie_change(iolat->blkiolat, lat_info, false);
	}
	latency_stat_init(iolat, &iolat->cur_stat);
out:
	spin_unlock_irqrestore(&lat_info->lock, flags);
}

static void blkcg_iolatency_done_bio(struct rq_qos *rqos, struct bio *bio)
{
	struct blkcg_gq *blkg;
	struct rq_wait *rqw;
	struct iolatency_grp *iolat;
	u64 window_start;
	u64 now;
	bool issue_as_root = bio_issue_as_root_blkg(bio);
	bool enabled = false;
	int inflight = 0;

	blkg = bio->bi_blkg;
	if (!blkg || !bio_flagged(bio, BIO_TRACKED))
		return;

	iolat = blkg_to_lat(bio->bi_blkg);
	if (!iolat)
		return;

	enabled = blk_iolatency_enabled(iolat->blkiolat);
	if (!enabled)
		return;

	now = ktime_to_ns(ktime_get());
	/* 由下而上，遍历所有的cgroup */
	while (blkg && blkg->parent) {
		iolat = blkg_to_lat(blkg);
		if (!iolat) {
			blkg = blkg->parent;
			continue;
		}
		rqw = &iolat->rq_wait;

		inflight = atomic_dec_return(&rqw->inflight);
		WARN_ON_ONCE(inflight < 0);
		/*
		 * If bi_status is BLK_STS_AGAIN, the bio wasn't actually
		 * submitted, so do not account for it.
		 */
		if (iolat->min_lat_nsec && bio->bi_status != BLK_STS_AGAIN) {
			/* 记录当前请求的延时情况 */
			iolatency_record_time(iolat, &bio->bi_issue, now,
					      issue_as_root);
			window_start = atomic64_read(&iolat->window_start);
			if (now > window_start &&
			    (now - window_start) >= iolat->cur_win_nsec) {
				/* 若超过窗口时间，开始进行限流控制 */
				if (atomic64_cmpxchg(&iolat->window_start,
					     window_start, now) == window_start)
					iolatency_check_latencies(iolat, now);
			}
		}
		/* 这里会激活所有被限流的线程 */
		wake_up(&rqw->wait);
		blkg = blkg->parent;
	}
}

static void blkcg_iolatency_exit(struct rq_qos *rqos)
{
	struct blk_iolatency *blkiolat = BLKIOLATENCY(rqos);

	del_timer_sync(&blkiolat->timer);
	blkcg_deactivate_policy(rqos->q, &blkcg_policy_iolatency);
	kfree(blkiolat);
}

static struct rq_qos_ops blkcg_iolatency_ops = {
	.throttle = blkcg_iolatency_throttle,
	.done_bio = blkcg_iolatency_done_bio,
	.exit = blkcg_iolatency_exit,
};

static void blkiolatency_timer_fn(struct timer_list *t)
{
	struct blk_iolatency *blkiolat = from_timer(blkiolat, t, timer);
	struct blkcg_gq *blkg;
	struct cgroup_subsys_state *pos_css;
	u64 now = ktime_to_ns(ktime_get());

	rcu_read_lock();
	blkg_for_each_descendant_pre(blkg, pos_css,
				     blkiolat->rqos.q->root_blkg) {
		struct iolatency_grp *iolat;
		struct child_latency_info *lat_info;
		unsigned long flags;
		u64 cookie;

		/*
		 * We could be exiting, don't access the pd unless we have a
		 * ref on the blkg.
		 */
		if (!blkg_tryget(blkg))
			continue;

		iolat = blkg_to_lat(blkg);
		if (!iolat)
			goto next;

		lat_info = &iolat->child_lat;
		/* 读取每个层级的child_lat->scale_cookie */
		cookie = atomic_read(&lat_info->scale_cookie);

		/* 如果cookie 大于 default则不处理 */
		if (cookie >= DEFAULT_SCALE_COOKIE)
			goto next;

		spin_lock_irqsave(&lat_info->lock, flags);
		/* 如果上次变更的时刻超过此时刻，则已经被更新过了，不处理 */
		if (lat_info->last_scale_event >= now)
			goto next_lock;

		/*
		 * We scaled down but don't have a scale_grp, scale up and carry
		 * on.
		 */
		/* 如果没有scale group， 则scale_cookie_change */
		if (lat_info->scale_grp == NULL) {
			scale_cookie_change(iolat->blkiolat, lat_info, true);
			goto next_lock;
		}

		/*
		 * It's been 5 seconds since our last scale event, clear the
		 * scale grp in case the group that needed the scale down isn't
		 * doing any IO currently.
		 */
		/* 上次sclae event已经过去5s，把scale group设置为NULL */
		if (now - lat_info->last_scale_event >=
		    ((u64)NSEC_PER_SEC * 5))
			lat_info->scale_grp = NULL;
next_lock:
		spin_unlock_irqrestore(&lat_info->lock, flags);
next:
		blkg_put(blkg);
	}
	rcu_read_unlock();
}

int blk_iolatency_init(struct request_queue *q)
{
	struct blk_iolatency *blkiolat;
	struct rq_qos *rqos;
	int ret;

	blkiolat = kzalloc(sizeof(*blkiolat), GFP_KERNEL);
	if (!blkiolat)
		return -ENOMEM;

	rqos = &blkiolat->rqos;
	rqos->id = RQ_QOS_LATENCY;
	rqos->ops = &blkcg_iolatency_ops;
	rqos->q = q;

	rq_qos_add(q, rqos);

	ret = blkcg_activate_policy(q, &blkcg_policy_iolatency);
	if (ret) {
		rq_qos_del(q, rqos);
		kfree(blkiolat);
		return ret;
	}

	timer_setup(&blkiolat->timer, blkiolatency_timer_fn, 0);

	return 0;
}

/*
 * return 1 for enabling iolatency, return -1 for disabling iolatency, otherwise
 * return 0.
 */
static int iolatency_set_min_lat_nsec(struct blkcg_gq *blkg, u64 val)
{
	struct iolatency_grp *iolat = blkg_to_lat(blkg);
	u64 oldval = iolat->min_lat_nsec;

	iolat->min_lat_nsec = val;
	iolat->cur_win_nsec = max_t(u64, val << 4, BLKIOLATENCY_MIN_WIN_SIZE);
	iolat->cur_win_nsec = min_t(u64, iolat->cur_win_nsec,
				    BLKIOLATENCY_MAX_WIN_SIZE);

	if (!oldval && val)
		return 1;
	if (oldval && !val) {
		blkcg_clear_delay(blkg);
		return -1;
	}
	return 0;
}

static void iolatency_clear_scaling(struct blkcg_gq *blkg)
{
	if (blkg->parent) {
		struct iolatency_grp *iolat = blkg_to_lat(blkg->parent);
		struct child_latency_info *lat_info;
		if (!iolat)
			return;

		lat_info = &iolat->child_lat;
		spin_lock(&lat_info->lock);
		atomic_set(&lat_info->scale_cookie, DEFAULT_SCALE_COOKIE);
		lat_info->last_scale_event = 0;
		lat_info->scale_grp = NULL;
		lat_info->scale_lat = 0;
		spin_unlock(&lat_info->lock);
	}
}

static ssize_t iolatency_set_limit(struct kernfs_open_file *of, char *buf,
			     size_t nbytes, loff_t off)
{
	struct blkcg *blkcg = css_to_blkcg(of_css(of));
	struct blkcg_gq *blkg;
	struct blkg_conf_ctx ctx;
	struct iolatency_grp *iolat;
	char *p, *tok;
	u64 lat_val = 0;
	u64 oldval;
	int ret;
	int enable = 0;

	ret = blkg_conf_prep(blkcg, &blkcg_policy_iolatency, buf, &ctx);
	if (ret)
		return ret;

	iolat = blkg_to_lat(ctx.blkg);
	p = ctx.body;

	ret = -EINVAL;
	while ((tok = strsep(&p, " "))) {
		char key[16];
		char val[21];	/* 18446744073709551616 */

		if (sscanf(tok, "%15[^=]=%20s", key, val) != 2)
			goto out;

		if (!strcmp(key, "target")) {
			u64 v;

			if (!strcmp(val, "max"))
				lat_val = 0;
			else if (sscanf(val, "%llu", &v) == 1)
				lat_val = v * NSEC_PER_USEC;
			else
				goto out;
		} else {
			goto out;
		}
	}

	/* Walk up the tree to see if our new val is lower than it should be. */
	blkg = ctx.blkg;
	oldval = iolat->min_lat_nsec;

	enable = iolatency_set_min_lat_nsec(blkg, lat_val);
	if (enable) {
		if (!blk_get_queue(blkg->q)) {
			ret = -ENODEV;
			goto out;
		}

		blkg_get(blkg);
	}

	if (oldval != iolat->min_lat_nsec) {
		iolatency_clear_scaling(blkg);
	}

	ret = 0;
out:
	blkg_conf_finish(&ctx);
	if (ret == 0 && enable) {
		struct iolatency_grp *tmp = blkg_to_lat(blkg);
		struct blk_iolatency *blkiolat = tmp->blkiolat;

		blk_mq_freeze_queue(blkg->q);

		if (enable == 1)
			atomic_inc(&blkiolat->enabled);
		else if (enable == -1)
			atomic_dec(&blkiolat->enabled);
		else
			WARN_ON_ONCE(1);

		blk_mq_unfreeze_queue(blkg->q);

		blkg_put(blkg);
		blk_put_queue(blkg->q);
	}
	return ret ?: nbytes;
}

static u64 iolatency_prfill_limit(struct seq_file *sf,
				  struct blkg_policy_data *pd, int off)
{
	struct iolatency_grp *iolat = pd_to_lat(pd);
	const char *dname = blkg_dev_name(pd->blkg);

	if (!dname || !iolat->min_lat_nsec)
		return 0;
	seq_printf(sf, "%s target=%llu\n",
		   dname, div_u64(iolat->min_lat_nsec, NSEC_PER_USEC));
	return 0;
}

static int iolatency_print_limit(struct seq_file *sf, void *v)
{
	blkcg_print_blkgs(sf, css_to_blkcg(seq_css(sf)),
			  iolatency_prfill_limit,
			  &blkcg_policy_iolatency, seq_cft(sf)->private, false);
	return 0;
}

static bool iolatency_ssd_stat(struct iolatency_grp *iolat, struct seq_file *s)
{
	struct latency_stat stat;
	int cpu;

	latency_stat_init(iolat, &stat);
	preempt_disable();
	for_each_online_cpu(cpu) {
		struct latency_stat *s;
		s = per_cpu_ptr(iolat->stats, cpu);
		latency_stat_sum(iolat, &stat, s);
	}
	preempt_enable();

	if (iolat->rq_depth.max_depth == UINT_MAX)
		seq_printf(s, " missed=%llu total=%llu depth=max",
			(unsigned long long)stat.ps.missed,
			(unsigned long long)stat.ps.total);
	else
		seq_printf(s, " missed=%llu total=%llu depth=%u",
			(unsigned long long)stat.ps.missed,
			(unsigned long long)stat.ps.total,
			iolat->rq_depth.max_depth);
	return true;
}

static bool iolatency_pd_stat(struct blkg_policy_data *pd, struct seq_file *s)
{
	struct iolatency_grp *iolat = pd_to_lat(pd);
	unsigned long long avg_lat;
	unsigned long long cur_win;

	if (!blkcg_debug_stats)
		return false;

	if (iolat->ssd)
		return iolatency_ssd_stat(iolat, s);

	avg_lat = div64_u64(iolat->lat_avg, NSEC_PER_USEC);
	cur_win = div64_u64(iolat->cur_win_nsec, NSEC_PER_MSEC);
	if (iolat->rq_depth.max_depth == UINT_MAX)
		seq_printf(s, " depth=max avg_lat=%llu win=%llu",
			avg_lat, cur_win);
	else
		seq_printf(s, " depth=%u avg_lat=%llu win=%llu",
			iolat->rq_depth.max_depth, avg_lat, cur_win);
	return true;
}

static struct blkg_policy_data *iolatency_pd_alloc(gfp_t gfp,
						   struct request_queue *q,
						   struct blkcg *blkcg)
{
	struct iolatency_grp *iolat;

	iolat = kzalloc_node(sizeof(*iolat), gfp, q->node);
	if (!iolat)
		return NULL;
	iolat->stats = __alloc_percpu_gfp(sizeof(struct latency_stat),
				       __alignof__(struct latency_stat), gfp);
	if (!iolat->stats) {
		kfree(iolat);
		return NULL;
	}
	return &iolat->pd;
}

static void iolatency_pd_init(struct blkg_policy_data *pd)
{
	struct iolatency_grp *iolat = pd_to_lat(pd);
	struct blkcg_gq *blkg = lat_to_blkg(iolat);
	struct rq_qos *rqos = blkcg_rq_qos(blkg->q);
	struct blk_iolatency *blkiolat = BLKIOLATENCY(rqos);
	u64 now = ktime_to_ns(ktime_get());
	int cpu;
	/* 判断非旋转设备，即固态和硬盘的区别 */
	if (blk_queue_nonrot(blkg->q))
		iolat->ssd = true;
	else
		iolat->ssd = false;

	for_each_possible_cpu(cpu) {
		struct latency_stat *stat;
		stat = per_cpu_ptr(iolat->stats, cpu);
		latency_stat_init(iolat, stat);
	}

	latency_stat_init(iolat, &iolat->cur_stat);
	rq_wait_init(&iolat->rq_wait);
	spin_lock_init(&iolat->child_lat.lock);
	iolat->rq_depth.queue_depth = blkg->q->nr_requests;
	iolat->rq_depth.max_depth = UINT_MAX;
	iolat->rq_depth.default_depth = iolat->rq_depth.queue_depth;
	iolat->blkiolat = blkiolat;
	iolat->cur_win_nsec = 100 * NSEC_PER_MSEC;
	atomic64_set(&iolat->window_start, now);

	/*
	 * We init things in list order, so the pd for the parent may not be
	 * init'ed yet for whatever reason.
	 */
	if (blkg->parent && blkg_to_pd(blkg->parent, &blkcg_policy_iolatency)) {
		struct iolatency_grp *parent = blkg_to_lat(blkg->parent);
		atomic_set(&iolat->scale_cookie,
			   atomic_read(&parent->child_lat.scale_cookie));
	} else {
		atomic_set(&iolat->scale_cookie, DEFAULT_SCALE_COOKIE);
	}

	atomic_set(&iolat->child_lat.scale_cookie, DEFAULT_SCALE_COOKIE);
}

static void iolatency_pd_offline(struct blkg_policy_data *pd)
{
	struct iolatency_grp *iolat = pd_to_lat(pd);
	struct blkcg_gq *blkg = lat_to_blkg(iolat);
	struct blk_iolatency *blkiolat = iolat->blkiolat;
	int ret;

	ret = iolatency_set_min_lat_nsec(blkg, 0);
	if (ret == 1)
		atomic_inc(&blkiolat->enabled);
	if (ret == -1)
		atomic_dec(&blkiolat->enabled);
	iolatency_clear_scaling(blkg);
}

static void iolatency_pd_free(struct blkg_policy_data *pd)
{
	struct iolatency_grp *iolat = pd_to_lat(pd);
	free_percpu(iolat->stats);
	kfree(iolat);
}

static struct cftype iolatency_files[] = {
	{
		.name = "latency",
		.flags = CFTYPE_NOT_ON_ROOT,
		.seq_show = iolatency_print_limit,
		.write = iolatency_set_limit,
	},
	{}
};

static struct blkcg_policy blkcg_policy_iolatency = {
	.dfl_cftypes	= iolatency_files,
	.pd_alloc_fn	= iolatency_pd_alloc,
	.pd_init_fn	= iolatency_pd_init,
	.pd_offline_fn	= iolatency_pd_offline,
	.pd_free_fn	= iolatency_pd_free,
	.pd_stat_fn	= iolatency_pd_stat,
};

static int __init iolatency_init(void)
{
	return blkcg_policy_register(&blkcg_policy_iolatency);
}

static void __exit iolatency_exit(void)
{
	blkcg_policy_unregister(&blkcg_policy_iolatency);
}

module_init(iolatency_init);
module_exit(iolatency_exit);
