// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kallsyms.h>
#include <linux/slab.h>
#include <linux/thread_info.h>
#include <soc/qcom/minidump.h>
#include <asm/sections.h>
#include <linux/mm.h>
#include <linux/sched/task.h>
#include <linux/vmalloc.h>
#include <linux/vmalloc.h>
#include <linux/sched/clock.h>

u64 curr_idx;
bool stop_logging;
struct md_region panic_backtrace_region;
extern int in_panic;

static void __init register_log_buf(void)
{
	char **log_bufp;
	uint32_t *log_buf_lenp;
	struct md_region md_entry;

	log_bufp = (char **)kallsyms_lookup_name("log_buf");
	log_buf_lenp = (uint32_t *)kallsyms_lookup_name("log_buf_len");
	if (!log_bufp || !log_buf_lenp) {
		pr_err("Unable to find log_buf by kallsyms!\n");
		return;
	}
	/*Register logbuf to minidump, first idx would be from bss section */
	strlcpy(md_entry.name, "KLOGBUF", sizeof(md_entry.name));
	md_entry.virt_addr = (uintptr_t) (*log_bufp);
	md_entry.phys_addr = virt_to_phys(*log_bufp);
	md_entry.size = *log_buf_lenp;
	md_entry.id = MINIDUMP_DEFAULT_ID;
	if (msm_minidump_add_region(&md_entry))
		pr_err("Failed to add logbuf in Minidump\n");
}

void panic_backtrace_stop(void)
{
	size_t size;
	char *vaddr;

	size = panic_backtrace_region.size;
	vaddr = (char *)panic_backtrace_region.virt_addr;

	printk("%s curr_idx = %d\n", __func__, curr_idx);
	if (in_panic == -1) {
		in_panic = raw_smp_processor_id();
		return;
	}
	if (vaddr == NULL || stop_logging)
		return;

	curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
			     "[<ffffffffffffffff>] 0xffffffffffffffff\n");
	curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
			     "$** *** *** *** *** *** *** *** Fatal *** *** *** *** *** *** *** **$\n");

	stop_logging = true;
	if (msm_minidump_add_region(&panic_backtrace_region))
		pr_err("Failed to add btrace in Minidump\n");
#if 0
	printk("%s the panic stack buffer we've got is like this %s\n", __func__, panic_backtrace_region.virt_addr);
#endif
}
EXPORT_SYMBOL(panic_backtrace_stop);

void panic_backtrace_log(int cpu, unsigned long where)
{
	size_t size;

	char *vaddr;
	static int skip_cnt;

	size = panic_backtrace_region.size;
	vaddr = (char *)panic_backtrace_region.virt_addr;

	if (vaddr == NULL || cpu != in_panic || stop_logging)
		return;
#if 0
	printk("%s curr_idx = %d\n", __func__, curr_idx);
	printk("%s cpu = %d where = %p %pS\n", __func__, cpu, where, where);
	printk("%s vaddr = %p in_panic = %d stop_logging = %d\n", __func__, vaddr, in_panic, stop_logging);
#endif
	if (curr_idx == 0) {
		u64 tv_kernel;
		unsigned long rem_nsec;

		tv_kernel = local_clock();
		rem_nsec = do_div(tv_kernel, 1000000000);
		curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
				     "$** *** *** *** *** *** *** *** Fatal *** *** *** *** *** *** *** **$\n");
		curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
				     "Exception Log Time:[Thu Jan 1 00:00:00 CST 2019][%lu.%06lu]\n",
										tv_kernel, rem_nsec / 1000);
		curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
				     "Exception Class: Kernel (KE)\n\n");
		curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
				     "PC is at [<ffffffffffffffff>] dummy+0x000/0x000\n");
		curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
				     "LR is at [<ffffffffffffffff>] dummy+0x000/0x000\n\n");
		curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
				     "Current Executing Process:\n[CPU, %d][%s, %d]\n\n",
					raw_smp_processor_id(), current->comm, current->pid);
		curr_idx += snprintf(vaddr + curr_idx, size - curr_idx, "Backtrace:\n");
	}

	if (skip_cnt > 0) {
		skip_cnt--;
		return;
	}
	curr_idx += snprintf(vaddr + curr_idx, size - curr_idx,
			     "[<%p>] %pS\n", (void *)where, (void *)where);
#if 0
	printk("%s the panic stack buffer we've got is like this %s\n", __func__, panic_backtrace_region.virt_addr);
#endif
}
EXPORT_SYMBOL(panic_backtrace_log);

void __init register_panic_backtrace(void)
{
	unsigned long vaddr;
	u64 paddr;
	size_t size = SZ_4K;

	vaddr = get_zeroed_page(GFP_KERNEL);;
	if (vaddr == 0) {
		pr_err("%s: fail to alloc memory\n", __func__);
		return;
	}
	paddr = (u64)virt_to_phys((const volatile char *)vaddr);

	memset((void *)vaddr, 0, size);
	pr_info("%s: alloc success paddr[0x%lx] vaddr[%p] size[%lx]\n",
				__func__, (unsigned long)paddr, vaddr, (unsigned long)size);
	strlcpy(panic_backtrace_region.name, "btrace", sizeof(panic_backtrace_region.name));
	panic_backtrace_region.virt_addr = (u64)vaddr;
	panic_backtrace_region.phys_addr = (u64)virt_to_phys((const volatile void *)vaddr);
	panic_backtrace_region.size = size;

}

static void register_stack_entry(struct md_region *ksp_entry, u64 sp, u64 size,
				 u32 cpu)
{
	struct page *sp_page;
	struct vm_struct *stack_vm_area = task_stack_vm_area(current);

	ksp_entry->id = MINIDUMP_DEFAULT_ID;
	ksp_entry->virt_addr = sp;
	ksp_entry->size = size;
	if (stack_vm_area) {
		sp_page = vmalloc_to_page((const void *) sp);
		ksp_entry->phys_addr = page_to_phys(sp_page);
	} else {
		ksp_entry->phys_addr = virt_to_phys((uintptr_t *)sp);
	}

	if (msm_minidump_add_region(ksp_entry))
		pr_err("Failed to add stack of cpu %d in Minidump\n", cpu);
}

static void __init register_kernel_sections(void)
{
	struct md_region ksec_entry;
	char *data_name = "KDATABSS";
	const size_t static_size = __per_cpu_end - __per_cpu_start;
	void __percpu *base = (void __percpu *)__per_cpu_start;
	unsigned int cpu;

	strlcpy(ksec_entry.name, data_name, sizeof(ksec_entry.name));
	ksec_entry.virt_addr = (uintptr_t)_sdata;
	ksec_entry.phys_addr = virt_to_phys(_sdata);
	ksec_entry.size = roundup((__bss_stop - _sdata), 4);
	ksec_entry.id = MINIDUMP_DEFAULT_ID;
	if (msm_minidump_add_region(&ksec_entry))
		pr_err("Failed to add data section in Minidump\n");

	/* Add percpu static sections */
	for_each_possible_cpu(cpu) {
		void *start = per_cpu_ptr(base, cpu);

		memset(&ksec_entry, 0, sizeof(ksec_entry));
		scnprintf(ksec_entry.name, sizeof(ksec_entry.name),
			"KSPERCPU%d", cpu);
		ksec_entry.virt_addr = (uintptr_t)start;
		ksec_entry.phys_addr = per_cpu_ptr_to_phys(start);
		ksec_entry.size = static_size;
		ksec_entry.id = MINIDUMP_DEFAULT_ID;
		if (msm_minidump_add_region(&ksec_entry))
			pr_err("Failed to add percpu sections in Minidump\n");
	}
}

static inline bool in_stack_range(u64 sp, u64 base_addr, unsigned int
				  stack_size)
{
	u64 min_addr = base_addr;
	u64 max_addr = base_addr + stack_size;

	return (min_addr <= sp && sp < max_addr);
}

static unsigned int calculate_copy_pages(u64 sp, struct vm_struct *stack_area)
{
	u64 tsk_stack_base = (u64) stack_area->addr;
	u64 offset;
	unsigned int stack_pages, copy_pages;

	if (in_stack_range(sp, tsk_stack_base, get_vm_area_size(stack_area))) {
		offset = sp - tsk_stack_base;
		stack_pages = get_vm_area_size(stack_area) / PAGE_SIZE;
		copy_pages = stack_pages - (offset / PAGE_SIZE);
	} else {
		copy_pages = 0;
	}
	return copy_pages;
}

void dump_stack_minidump(u64 sp)
{
	struct md_region ksp_entry, ktsk_entry;
	u32 cpu = smp_processor_id();
	struct vm_struct *stack_vm_area;
	unsigned int i, copy_pages;

	if (is_idle_task(current))
		return;

	if (sp < MODULES_END || sp > -256UL)
		sp = current_stack_pointer;

	/*
	 * Since stacks are now allocated with vmalloc, the translation to
	 * physical address is not a simple linear transformation like it is
	 * for kernel logical addresses, since vmalloc creates a virtual
	 * mapping. Thus, virt_to_phys() should not be used in this context;
	 * instead the page table must be walked to acquire the physical
	 * address of one page of the stack.
	 */
	stack_vm_area = task_stack_vm_area(current);
	if (stack_vm_area) {
		sp &= ~(PAGE_SIZE - 1);
		copy_pages = calculate_copy_pages(sp, stack_vm_area);
		for (i = 0; i < copy_pages; i++) {
			scnprintf(ksp_entry.name, sizeof(ksp_entry.name),
				  "KSTACK%d_%d", cpu, i);
			register_stack_entry(&ksp_entry, sp, PAGE_SIZE, cpu);
			sp += PAGE_SIZE;
		}
	} else {
		sp &= ~(THREAD_SIZE - 1);
		scnprintf(ksp_entry.name, sizeof(ksp_entry.name), "KSTACK%d",
			  cpu);
		register_stack_entry(&ksp_entry, sp, THREAD_SIZE, cpu);
	}

	scnprintf(ktsk_entry.name, sizeof(ktsk_entry.name), "KTASK%d", cpu);
	ktsk_entry.virt_addr = (u64)current;
	ktsk_entry.phys_addr = virt_to_phys((uintptr_t *)current);
	ktsk_entry.size = sizeof(struct task_struct);
	ktsk_entry.id = MINIDUMP_DEFAULT_ID;
	if (msm_minidump_add_region(&ktsk_entry))
		pr_err("Failed to add current task %d in Minidump\n", cpu);
}

static int __init msm_minidump_log_init(void)
{
	register_kernel_sections();
	register_log_buf();
	register_panic_backtrace();
	return 0;
}
late_initcall(msm_minidump_log_init);
