/*
 * Copyright (c) 2013-2015, ARM Limited and Contributors. All rights reserved.
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

#include <assert.h>
#include <platform_def.h>
#include "drivers/pwrc/fvp_pwrc.h"
#include "fvp_private.h"

/*
 * The FVP power domain tree does not have a single system level power domain
 * i.e. a single root node. The first entry in the power domain descriptor
 * specifies the number of power domains at the highest power level. For the FVP
 * this is 2 i.e. the number of cluster power domains.
 */
#define FVP_PWR_DOMAINS_AT_MAX_PWR_LVL	FVP_CLUSTER_COUNT

/* The FVP power domain tree descriptor */
const unsigned char fvp_power_domain_tree_desc[] = {
	/* No of root nodes */
	FVP_PWR_DOMAINS_AT_MAX_PWR_LVL,
	/* No of children for the first node */
	FVP_CLUSTER0_CORE_COUNT,
	/* No of children for the second node */
	FVP_CLUSTER1_CORE_COUNT
};

/*******************************************************************************
 * This function returns the FVP topology tree information.
 ******************************************************************************/
const unsigned char *platform_get_power_domain_tree_desc(void)
{
	return fvp_power_domain_tree_desc;
}

/*******************************************************************************
 * This function validates an MPIDR by checking whether it falls within the
 * acceptable bounds of an FVP variant. An FVP variant can have a maximum of 2
 * clusters and 4 CPUs per-cluster. The power controller is used to perform the
 * check. An error code (-1) is returned if an incorrect mpidr is passed.
 ******************************************************************************/
int fvp_check_mpidr(unsigned long mpidr)
{
	unsigned int cluster_id, cpu_id;

	mpidr &= MPIDR_AFFINITY_MASK;

	if (mpidr & ~(MPIDR_CLUSTER_MASK | MPIDR_CPU_MASK))
		return -1;

	cluster_id = (mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;
	cpu_id = (mpidr >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK;

	if (cluster_id >= FVP_CLUSTER_COUNT || cpu_id >= FVP_MAX_CPUS_PER_CLUSTER)
		return -1;

	if (fvp_pwrc_read_psysr(mpidr) == PSYSR_INVALID)
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

	rc = fvp_check_mpidr(mpidr);
	if (rc >= 0)
		return fvp_get_core_pos_common(mpidr);
	return rc;
}
