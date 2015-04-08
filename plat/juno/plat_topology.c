/*
 * Copyright (c) 2014-2015, ARM Limited and Contributors. All rights reserved.
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

#include <platform_def.h>
#include <psci.h>
#include "juno_private.h"

unsigned int plat_get_pwr_domain_count(unsigned int pwr_lvl,
						unsigned long mpidr)
{
	/* Report 1 (absent) power domain  at levels higher that the
	   cluster level */
	if (pwr_lvl > JUNO_PWR_LVL1)
		return 1;

	if (pwr_lvl == JUNO_PWR_LVL1)
		return 2; /* We have two clusters */

	return mpidr & 0x100 ? 4 : 2; /* 4 cpus in cluster 1, 2 in cluster 0 */
}

unsigned int plat_get_pwr_domain_state(unsigned int pwr_lvl,
		unsigned long mpidr)
{
	return pwr_lvl <= JUNO_PWR_LVL1 ? PSCI_PWR_DOMAIN_PRESENT :
						PSCI_PWR_DOMAIN_ABSENT;
}

int plat_setup_topology(void)
{
	/* Juno todo: Make topology configurable via SCC */
	return 0;
}

static unsigned int juno_get_core_count(unsigned int cluster)
{
	return (cluster == 1) ? JUNO_CLUSTER1_CORE_COUNT : JUNO_CLUSTER0_CORE_COUNT;
}

/*******************************************************************************
 * This function validates an MPIDR by checking whether it represents a CPU in
 * one of the two clusters present on the Juno platform. An error code (-1) is
 * returned if an incorrect mpidr is passed.
 ******************************************************************************/
unsigned int juno_check_mpidr(unsigned long mpidr)
{
	unsigned int cluster_id, cpu_id;

	mpidr &= MPIDR_AFFINITY_MASK;

	if (mpidr & ~(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK))
		return -1;

	cluster_id = (mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;
	cpu_id = (mpidr >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK;

	if (cluster_id >= JUNO_CLUSTER_COUNT)
		return -1;

	if (cpu_id >= juno_get_core_count(cluster_id))
		return -1;

	return 0;
}

/*******************************************************************************
 * This function implements a part of the critical interface between the psci
 * generic layer and the platform that allows the former to query the platform
 * to convert an MPIDR to a unique linear index. An error code (-1) is returned
 * in case the MPIDR is invalid.
 ******************************************************************************/
int platform_get_core_pos(unsigned long mpidr)
{
	int rc;

	rc = juno_check_mpidr(mpidr);
	if (rc >= 0)
		return juno_get_core_pos_common(mpidr);
	return rc;
}
