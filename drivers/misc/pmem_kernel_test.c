/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/android_pmem.h>
#include <linux/io.h>

#define MODULE_NAME "pmem_kernel_test"

static int read_write_test(void *kernel_addr, unsigned long size)
{
	int j, *p;

	printk(KERN_INFO MODULE_NAME ": read_write_test entry, "
		"kernel_addr %p, size %lu\n", kernel_addr, size);

	for (j = 0, p = kernel_addr;
			j < (size / sizeof(int));
			j++, p++)
		*p = j;

	for (j = 0, p = kernel_addr;
			j < (size / sizeof(int));
			j++, p++)
		if (*p != j) {
			printk(KERN_INFO MODULE_NAME ": read_write_test"
				" FAILS @ int offset %d!\n", j);
			return -1;
		}

	printk(KERN_INFO MODULE_NAME ": read_write_test success\n");
	return 0;
}

static void map_and_check(int32_t physaddr, unsigned long size)
{
	void *kernel_addr = ioremap((unsigned long)physaddr, size);

	printk(KERN_INFO MODULE_NAME
		": map_and_check entry, physaddr %#x, size %lu\n",
		physaddr, size);

	if (!kernel_addr) {
		printk(KERN_INFO MODULE_NAME
			": map_and_check FAILS ioremap!\n");
		return;
	} else {
		if (read_write_test(kernel_addr, size))
			printk(KERN_INFO MODULE_NAME ": map_and_check "
				" kernel_addr %p FAILS read_write_check!\n",
				kernel_addr);
		else
			printk(KERN_INFO MODULE_NAME ": map_and_check "
				" kernel_addr %p read_write_check success\n",
				kernel_addr);

		iounmap(kernel_addr);
	}

	printk(KERN_INFO MODULE_NAME ": map_and_check success\n");
}

static int test_pmem_device(size_t size, int32_t flags)
{
	int32_t ret;

	printk(KERN_INFO MODULE_NAME
		": test_pmem_device entry, size %d, flags %#x\n",
		size, flags);

	ret = pmem_kalloc(size, flags);
	if (ret < 0) {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device pmem_kalloc FAILS %d\n", ret);
		return (int)ret;
	} else {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device pmem_kalloc success\n");
	}

	map_and_check(ret, (unsigned long)size);

	ret = pmem_kfree(ret);
	if (ret < 0)
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device pmem_kfree FAILS %d\n", ret);
	else
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device pmem_kfree success\n");

	printk(KERN_INFO MODULE_NAME ": test_pmem_device success!\n");
	return (int)ret;
}

static int test_pmem_device_n1(int32_t flags)
{
	int ret;

	printk(KERN_INFO MODULE_NAME
		": test_pmem_device_n1 entry, flags %#x\n", flags);

	ret = test_pmem_device(0x100000, flags);
	if (ret < 0)
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_n1 FAILS %d\n", ret);
	else
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_n1 success\n");
	return ret;
}

static int test_pmem_device_n2(int32_t flags)
{
	int ret;

	printk(KERN_INFO MODULE_NAME
		": test_pmem_device_n2 entry, flags %#x\n", flags);

	ret = test_pmem_device(0x200000, flags);
	if (ret < 0)
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_n2 FAILS %d\n", ret);
	else
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_n2 success\n");
	return ret;
}

static int test_pmem_device_a1(int32_t flags)
{
	int32_t ret1, ret2;
	int ret = 0;
	void *kernel_addr1, *kernel_addr2;

	printk(KERN_INFO MODULE_NAME
		"test_pmem_device_a1 entry, flags %#x\n", flags);

	ret1 = pmem_kalloc(0x100000, flags);
	if (ret1 < 0) {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 pmem_kalloc 0x100000 "
			"FAILS %d\n", ret1);
		ret = ret1;
		goto leave1;
	} else {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 pmem_kalloc 0x100000 success\n");
	}

	kernel_addr1 = ioremap((unsigned long)ret1, 0x100000);
	if (!kernel_addr1) {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 ioremap for 0x100000 FAILS\n");
		ret = -1;
		goto leave2;
	} else {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 ioremap for 0x100000 success\n");
	}

	if (read_write_test(kernel_addr1, 0x100000)) {
		printk(KERN_INFO MODULE_NAME
			":test_pmem_device_a1 0x100000 read_write_test "
			"FAILS\n");
		ret = -1;
		goto leave3;
	} else {
		printk(KERN_INFO MODULE_NAME
			":test_pmem_device_a1 0x100000 read_write_test "
			"success\n");
	}

	ret2 = pmem_kalloc(0x10000, flags);
	if (ret2 < 0) {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 pmem_kalloc 0x10000 "
			"FAILS %d\n", ret2);
		ret = ret2;
		goto leave3;
	} else {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 pmem_kalloc 0x10000 "
			"success\n");
	}

	kernel_addr2 = ioremap((unsigned long)ret2, 0x10000);
	if (!kernel_addr2) {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 ioremap for 0x10000 FAILS\n");
		ret = -1;
		goto leave4;
	} else {
		printk(KERN_INFO MODULE_NAME
			": test_pmem_device_a1 ioremap for 0x10000 "
			"success\n");
	}

	if (read_write_test(kernel_addr2, 0x10000)) {
		printk(KERN_INFO MODULE_NAME
			":test_pmem_device_a1 0x10000 read_write_test "
			"FAILS\n");
		ret = -1;
		goto leave5;
	} else {
		printk(KERN_INFO MODULE_NAME
			":test_pmem_device_a1 0x10000 read_write_test "
			"success\n");
	}
	printk(KERN_INFO MODULE_NAME "test_pmem_device_a1 test PASSED!\n");

leave5:
	iounmap(kernel_addr2);
leave4:
	pmem_kfree(ret2);
leave3:
	iounmap(kernel_addr1);
leave2:
	pmem_kfree(ret1);
leave1:
	return ret;
}

static int nominal_test(void)
{
	int ret;

	/* test_pmem_device_n1, for all different alignments and memtypes */
	printk(KERN_INFO MODULE_NAME "nominal_test entry\n");

	/* make sure error case fails */
	ret = test_pmem_device_n1(0);
	if (!ret) { /* !! bad news, this should fail! */
		printk(KERN_INFO MODULE_NAME
			": unexpected nominal/n1 success for zero flags "
			"parameter!\n");
		return -ENODEV;
	} else {
		printk(KERN_INFO MODULE_NAME
			"expected nominal/n1 failure as test of error"
			" case %d\n", ret);
	}

	ret = test_pmem_device_n1(PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_4K);
	if (ret) { /* now, if we fail, bad news */
		printk(KERN_INFO MODULE_NAME
			": nominal/n1 4K alignment FAILS, ret %d\n", ret);
		return ret;
	} else {
		printk(KERN_INFO MODULE_NAME
			": nominal/n1 4K alignment success\n");
	}

	ret = test_pmem_device_n1(PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_1M);
	if (ret) { /* now, if we fail, bad news */
		printk(KERN_INFO MODULE_NAME
			": nominal/n1 failure of nominal/n1 1M alignment, "
			"ret %d\n", ret);
		return ret;
	} else {
		printk(KERN_INFO MODULE_NAME
			": nominal/n1 1M alignment success\n");
	}

	/* test_pmem_device_n2, for all different alignments and memtypes */

	/* make sure error case fails */
	ret = test_pmem_device_n2(0);
	if (!ret) { /* !! bad news, this should fail! */
		printk(KERN_INFO MODULE_NAME
			": unexpected nominal/n2 success for zero flags "
			"parameter!\n");
		return -ENODEV;
	} else {
		printk(KERN_INFO MODULE_NAME
			"expected nominal/n2 failure as test of error"
			" case %d\n", ret);
	}

	ret = test_pmem_device_n2(PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_4K);
	if (ret) { /* now, if we fail, bad news */
		printk(KERN_INFO MODULE_NAME
			": nominal/n2 4K alignment FAILS, ret %d\n", ret);
		return ret;
	} else {
		printk(KERN_INFO MODULE_NAME
			": nominal/n2 4K alignment success\n");
	}

	ret = test_pmem_device_n2(PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_1M);
	if (ret) { /* now, if we fail, bad news */
		printk(KERN_INFO MODULE_NAME
			": nominal/n2 failure of nominal/n1 1M alignment, "
			"ret %d\n", ret);
		return ret;
	} else {
		printk(KERN_INFO MODULE_NAME
			": nominal/n2 1M alignment success\n");
	}

	printk(KERN_INFO MODULE_NAME "Nominal test success\n");
	return 0;
}

static int adversarial_test(void)
{
	int ret;

	/* test_pmem_device_a1, for all different alignments and memtypes */
	printk(KERN_INFO MODULE_NAME "adversarial_test entry\n");

	/* make sure error case fails */
	ret = test_pmem_device_a1(0);
	if (!ret) { /* !! bad news, this should fail! */
		printk(KERN_INFO MODULE_NAME
			": unexpected adversarial/a1 success for zero flags "
			"parameter!\n");
		return -ENODEV;
	} else {
		printk(KERN_INFO MODULE_NAME
			"expected adversarial/a1 failure as test of error"
			" case %d\n", ret);
	}

	ret = test_pmem_device_a1(PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_4K);
	if (ret) { /* now, if we fail, bad news */
		printk(KERN_INFO MODULE_NAME
			": adversarial/a1 4K alignment FAILS, ret %d\n", ret);
		return ret;
	} else {
		printk(KERN_INFO MODULE_NAME
			": adversarial/a1 4K alignment success\n");
	}

	ret = test_pmem_device_a1(PMEM_MEMTYPE_EBI1 | PMEM_ALIGNMENT_1M);
	if (ret) { /* now, if we fail, bad news */
		printk(KERN_INFO MODULE_NAME
			": adversarial/a1 1M alignment FAILS, ret %d\n", ret);
		return ret;
	} else {
		printk(KERN_INFO MODULE_NAME
			": adversarial/a1 1M alignment success\n");
	}

	printk(KERN_INFO MODULE_NAME "adversarial_test success\n");
	return ret;
}

static int __init pmem_kernel_test_init(void)
{
	int ret;

	/* tests lifted directly from
	 * vendor/qcom-proprietary/kernel-tests/_pmem_test.c and
	 * ported to the kernel API.
	 * Full release test.
	 */
	ret = nominal_test();
	if (ret) {
		printk(KERN_INFO MODULE_NAME
			": nominal test FAILS, ret %d\n", ret);
		return ret;
	}

	ret = adversarial_test();
	if (ret) {
		printk(KERN_INFO MODULE_NAME
			": adversarial test FAILS, ret %d\n", ret);
		return ret;
	}

	printk(KERN_INFO MODULE_NAME ": All PMEM kernel API tests PASS!\n");
	return ret;
}

static void __exit pmem_kernel_test_exit(void)
{
}

device_initcall(pmem_kernel_test_init);
module_exit(pmem_kernel_test_exit);

MODULE_LICENSE("Dual BSD/GPL");
