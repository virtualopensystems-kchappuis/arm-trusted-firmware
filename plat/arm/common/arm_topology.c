/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <arch.h>
#include <psci.h>
#include <platform_def.h>

/*
 * Weak definitions use fixed topology. Strong definitions could make topology
 * configurable
 */
#pragma weak plat_get_pwr_domain_count
#pragma weak plat_get_pwr_domain_state
#pragma weak plat_arm_topology_setup


unsigned int plat_get_pwr_domain_count(unsigned int pwr_lvl,
						unsigned long mpidr)
{
	/* Report 1 (absent) power domain  at levels higher that the
	   cluster level */
	if (pwr_lvl > ARM_PWR_LVL1)
		return 1;

	if (pwr_lvl == ARM_PWR_LVL1)
		return ARM_CLUSTER_COUNT;

	return mpidr & 0x100 ? PLAT_ARM_CLUSTER1_CORE_COUNT :
				PLAT_ARM_CLUSTER0_CORE_COUNT;
}

unsigned int plat_get_pwr_domain_state(unsigned int pwr_lvl,
		unsigned long mpidr)
{
	return pwr_lvl <= ARM_PWR_LVL1 ? PSCI_PWR_DOMAIN_PRESENT :
						PSCI_PWR_DOMAIN_ABSENT;
}

void plat_arm_topology_setup(void)
{
}
