/* drivers/misc/lowmemorykiller.c
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/list.h>

static int lowmem_shrink(int nr_to_scan, gfp_t gfp_mask);

static struct shrinker lowmem_shrinker = {
	.shrink = lowmem_shrink,
	.seeks = DEFAULT_SEEKS * 16
};
static uint32_t lowmem_debug_level = 2;
static int lowmem_adj[6] = {
	0,
	1,
	6,
	12,
};
static int lowmem_adj_size = 4;
static size_t lowmem_minfree[6] = {
	3*512, // 6MB
	2*1024, // 8MB
	4*1024, // 16MB
	16*1024, // 64MB
};
static int lowmem_minfree_size = 4;

#define lowmem_print(level, x...) do { if(lowmem_debug_level >= (level)) printk(x); } while(0)

module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
module_param_array_named(adj, lowmem_adj, int, &lowmem_adj_size, S_IRUGO | S_IWUSR);
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size, S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);

static LIST_HEAD(plist_head);
struct pri_pid {
	u32 pid;
	struct list_head list;
};

static DEFINE_MUTEX(pid_mutex);
static u32 pid = 0;
static int lowmem_add_pid(const char *val, struct kernel_param *kp);
static int lowmem_del_pid(const char *val, struct kernel_param *kp);
static int lowmem_get_pid(char *buffer, struct kernel_param *kp);
module_param_call(add_pid, lowmem_add_pid, lowmem_get_pid, &pid, 0664);
module_param_call(del_pid, lowmem_del_pid, lowmem_get_pid, &pid, 0664);

static int lowmem_get_pid(char *buffer, struct kernel_param *kp)
{
	struct pri_pid *next;

	mutex_lock(&pid_mutex);
	list_for_each_entry(next, &plist_head, list) {
		pr_info("lowmemkiller: pid=%lu\n", (unsigned long)(next->pid));
	}
	mutex_unlock(&pid_mutex);

	return 0;
}

static int lowmem_add_pid(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned long tmp;
	struct pri_pid *new_pid;
	struct pri_pid *next;

	ret = strict_strtoul(val, 10, &tmp);
	if (ret)
		goto out;

	mutex_lock(&pid_mutex);
	// Should nuke the list before start using it
	list_for_each_entry(next, &plist_head, list) {
		if (next->pid == tmp) {
			goto out;
		}
	}
	new_pid = kzalloc (sizeof(*new_pid), GFP_KERNEL);
	if (!new_pid) {
		pr_err("lowmemkiller: unable to allocate memory!\n");
		ret = -ENOMEM;
		goto out;
	}
	new_pid->pid = tmp;
	list_add(&(new_pid->list), &plist_head);
	pr_debug("lowmemkiller: add head=%p, pid=%lu\n", new_pid, tmp);
out:
	mutex_unlock(&pid_mutex);
	return ret;
}

static int lowmem_del_pid(const char *val, struct kernel_param *kp)
{
	int ret = 0;
	unsigned long tmp;
	struct pri_pid *next;

	ret = strict_strtoul(val, 10, &tmp);
	if (ret)
		goto out;

	mutex_lock(&pid_mutex);
	if (tmp == 0) {
		while (list_empty(&plist_head) != true) {
			next = (struct pri_pid *)list_entry(plist_head.next,struct pri_pid, list);
			list_del(&next->list);
			kfree(next);
		}
	} else {
		list_for_each_entry(next, &plist_head, list) {
			if (next->pid == tmp) {
				break;
			}
		}
		if (next->pid == tmp) {
			pr_debug("lowmemkiller: del head=%p, pid=%lu\n", next, tmp);
			list_del(&next->list);
			kfree(next);
		}
	}
out:
	mutex_unlock(&pid_mutex);
	return ret;
}

static int lowmem_shrink(int nr_to_scan, gfp_t gfp_mask)
{
	struct task_struct *p;
	struct task_struct *selected = NULL;
	int rem = 0;
	int tasksize;
	int i;
	int min_adj = OOM_ADJUST_MAX + 1;
	int selected_tasksize = 0;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES);
	int other_file = global_page_state(NR_FILE_PAGES);
	bool pri_previous;
	bool pri_current;
	struct pri_pid *next;

	if(lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if(lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for(i = 0; i < array_size; i++) {
		if (other_free < lowmem_minfree[i] &&
		    other_file < lowmem_minfree[i]) {
			min_adj = lowmem_adj[i];
			break;
		}
	}
	if(nr_to_scan > 0)
		lowmem_print(3, "lowmem_shrink %d, %x, ofree %d %d, ma %d\n", nr_to_scan, gfp_mask, other_free, other_file, min_adj);
	rem = global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
	if (nr_to_scan <= 0 || min_adj == OOM_ADJUST_MAX + 1) {
		lowmem_print(5, "lowmem_shrink %d, %x, return %d\n", nr_to_scan, gfp_mask, rem);
		return rem;
	}

	read_lock(&tasklist_lock);

	pri_previous = false;
	for_each_process(p) {
		if (p->oomkilladj < min_adj || !p->mm)
			continue;
		tasksize = get_mm_rss(p->mm);
		if (tasksize <= 0)
			continue;

		pri_current = false;
		list_for_each_entry(next, &plist_head, list) {
			if (next->pid == p->pid) {
				lowmem_print(1, "matched prioritized pid=%d\n", p->pid);
				pri_current = true;
				break;
			}
		}

		if (selected) {
			if (p->oomkilladj < selected->oomkilladj)
				continue;
			if (p->oomkilladj == selected->oomkilladj &&
			    tasksize <= selected_tasksize
			    && (!pri_previous || pri_current)) {
				continue;
			}
		}
		pri_previous = pri_current;
		selected = p;
		selected_tasksize = tasksize;
		lowmem_print(2, "select %d (%s), adj %d, size %d, to kill\n",
		             p->pid, p->comm, p->oomkilladj, tasksize);
	}
	if(selected != NULL) {
		if (fatal_signal_pending(selected)) {
			pr_warning("process %d is suffering a slow death\n",
				   selected->pid);
			read_unlock(&tasklist_lock);
			return rem;
		}
		lowmem_print(1, "send sigkill to %d (%s), adj %d, size %d\n",
		             selected->pid, selected->comm,
		             selected->oomkilladj, selected_tasksize);
		force_sig(SIGKILL, selected);
		rem -= selected_tasksize;
	}
	lowmem_print(4, "lowmem_shrink %d, %x, return %d\n", nr_to_scan, gfp_mask, rem);
	read_unlock(&tasklist_lock);
	return rem;
}

static int __init lowmem_init(void)
{
	register_shrinker(&lowmem_shrinker);
	return 0;
}

static void __exit lowmem_exit(void)
{
	unregister_shrinker(&lowmem_shrinker);
}

module_init(lowmem_init);
module_exit(lowmem_exit);

MODULE_LICENSE("GPL");

