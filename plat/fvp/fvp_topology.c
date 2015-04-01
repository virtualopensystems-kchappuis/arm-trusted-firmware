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
/* TODO: Reusing psci error codes & state information. Get our own! */
#include <psci.h>
#include "drivers/pwrc/fvp_pwrc.h"

/* We treat '255' as an invalid power domain instance */
#define INVALID_PWR_DOMAIN	0xff

/*******************************************************************************
 * We support 3 flavours of the FVP: Foundation, Base AEM & Base Cortex. Each
 * flavour has a different topology. The common bit is that there can be a max.
 * of 2 clusters (power level 1) and 4 cpus (power level 0) per cluster. So we
 * define a tree like data structure which caters to these maximum bounds. It
 * simply marks the absent power domain instances as PSCI_PWR_DOMAIN_ABSENT
 * e.g. there is no cluster 1 on the Foundation FVP. The 'data' field is
 * currently unused.
 ******************************************************************************/
typedef struct pwr_domain {
	unsigned char sibling;
	unsigned char child;
	unsigned char state;
	unsigned int data;
} pwr_domain_t;

/*******************************************************************************
 * The following two data structures store the topology tree for the fvp. There
 * is a separate array for each power level i.e. cpus and clusters. The child
 * and sibling references allow traversal inside and in between the two arrays.
 ******************************************************************************/
static pwr_domain_t fvp_pwrlvl1_topology_map[FVP_CLUSTER_COUNT];
static pwr_domain_t fvp_pwrlvl0_topology_map[PLATFORM_CORE_COUNT];

/* Simple global variable to safeguard us from stupidity */
static unsigned int topology_setup_done;

/*******************************************************************************
 * This function implements a part of the critical interface between the psci
 * generic layer and the platform to allow the former to detect the platform
 * topology. psci queries the platform to determine how many power domains
 * are present at a particular level for a given mpidr e.g. consider a dual
 * cluster platform where each cluster has 4 cpus. A call to this function with
 * (0, 0x100) will return the number of cpus implemented under cluster 1 i.e. 4.
 * Similarly a call with (1, 0x100) will return 2 i.e. the number of clusters.
 * This is 'cause we are effectively asking how many power level 1 domains
 * are implemented under power domain 0 at level 2.
 ******************************************************************************/
unsigned int plat_get_pwr_domain_count(unsigned int pwr_lvl,
				unsigned long mpidr)
{
	unsigned int domain_count = 1, ctr;
	unsigned char parent_pwr_domain_id;

	assert(topology_setup_done == 1);

	switch (pwr_lvl) {
	case 3:
	case 2:
		/*
		 * Assert if the parent power domain instance is not 0.
		 * This also takes care of level 3 in an obfuscated way
		 */
		parent_pwr_domain_id = (mpidr >> MPIDR_AFF3_SHIFT)
						& MPIDR_AFFLVL_MASK;
		assert(parent_pwr_domain_id == 0);

		/*
		 * Report that we implement a single power domain
		 * instance at levels 2 & 3 which are PWR_DOMAIN_ABSENT
		 */
		break;
	case 1:
		/* Assert if the parent domain instance is not 0. */
		parent_pwr_domain_id = (mpidr >> MPIDR_AFF2_SHIFT)
						& MPIDR_AFFLVL_MASK;
		assert(parent_pwr_domain_id == 0);

		/* Fetch the starting index in the power level1 array */
		for (ctr = 0;
		     fvp_pwrlvl1_topology_map[ctr].sibling !=
						INVALID_PWR_DOMAIN;
		     ctr = fvp_pwrlvl1_topology_map[ctr].sibling) {
			domain_count++;
		}

		break;
	case 0:
		/* Assert if the cluster id is anything apart from 0 or 1 */
		parent_pwr_domain_id = (mpidr >> MPIDR_AFF1_SHIFT)
						& MPIDR_AFFLVL_MASK;
		assert(parent_pwr_domain_id < FVP_CLUSTER_COUNT);

		/* Fetch the starting index in the power level0 array */
		for (ctr =
		     fvp_pwrlvl1_topology_map[parent_pwr_domain_id].child;
		     fvp_pwrlvl0_topology_map[ctr].sibling !=
						INVALID_PWR_DOMAIN;
		     ctr = fvp_pwrlvl0_topology_map[ctr].sibling) {
			domain_count++;
		}

		break;
	default:
		assert(0);
	}

	return domain_count;
}

/*******************************************************************************
 * This function implements a part of the critical interface between the psci
 * generic layer and the platform to allow the former to detect the state of a
 * power domain in the platform topology. psci queries the platform to
 * determine whether an power domain is present or absent. This caters
 * for topologies where an intermediate power domain is missing e.g.
 * consider a platform which implements a single cluster with 4 cpus and there
 * is another cpu sitting directly on the interconnect along with the cluster.
 * The mpidrs of the cluster would range from 0x0-0x3. The mpidr of the single
 * cpu would be 0x100 to highlight that it does not belong to cluster 0. Cluster
 * 1 is however missing but needs to be accounted to reach this single cpu in
 * the topology tree. Hence it will be marked as PSCI_PWR_DOMAIN_ABSENT. This
 * is not applicable to the FVP but depicted as an example.
 ******************************************************************************/
unsigned int plat_get_pwr_domain_state(unsigned int pwr_lvl,
				unsigned long mpidr)
{
	unsigned int pwr_domain_state = PSCI_PWR_DOMAIN_ABSENT, idx;
	idx = (mpidr >> MPIDR_AFF1_SHIFT) & MPIDR_AFFLVL_MASK;

	assert(topology_setup_done == 1);

	switch (pwr_lvl) {
	case 3:
	case 2:
		/* Report power levels 2 & 3 as absent */
		break;
	case 1:
		pwr_domain_state = fvp_pwrlvl1_topology_map[idx].state;
		break;
	case 0:
		/*
		 * First get start index of the pwrlvl0 in its array & then add
		 * to it the instance id that we want the state of
		 */
		idx = fvp_pwrlvl1_topology_map[idx].child;
		idx += (mpidr >> MPIDR_AFF0_SHIFT) & MPIDR_AFFLVL_MASK;
		pwr_domain_state = fvp_pwrlvl0_topology_map[idx].state;
		break;
	default:
		assert(0);
	}

	return pwr_domain_state;
}

/*******************************************************************************
 * This function populates the FVP specific topology information depending upon
 * the FVP flavour its running on. We construct all the mpidrs we can handle
 * and rely on the PWRC.PSYSR to flag absent cpus when their status is queried.
 ******************************************************************************/
int fvp_setup_topology(void)
{
	unsigned char pwrlvl0, pwrlvl1, pwr_domain_state, pwrlvl0_offset = 0;
	unsigned long mpidr;

	topology_setup_done = 0;

	for (pwrlvl1 = 0; pwrlvl1 < FVP_CLUSTER_COUNT; pwrlvl1++) {

		fvp_pwrlvl1_topology_map[pwrlvl1].child = pwrlvl0_offset;
		fvp_pwrlvl1_topology_map[pwrlvl1].sibling = pwrlvl1 + 1;

		for (pwrlvl0 = 0; pwrlvl0 < FVP_MAX_CPUS_PER_CLUSTER;
							pwrlvl0++) {

			mpidr = pwrlvl1 << MPIDR_AFF1_SHIFT;
			mpidr |= pwrlvl0 << MPIDR_AFF0_SHIFT;

			if (fvp_pwrc_read_psysr(mpidr) != PSYSR_INVALID) {
				/*
				 * Presence of even a single pwrlvl0 indicates
				 * presence of parent pwrlvl1 on the FVP.
				 */
				pwr_domain_state = PSCI_PWR_DOMAIN_PRESENT;
				fvp_pwrlvl1_topology_map[pwrlvl1].state =
						PSCI_PWR_DOMAIN_PRESENT;
			} else {
				pwr_domain_state = PSCI_PWR_DOMAIN_ABSENT;
			}

			fvp_pwrlvl0_topology_map[pwrlvl0_offset].child =
							INVALID_PWR_DOMAIN;
			fvp_pwrlvl0_topology_map[pwrlvl0_offset].state =
							pwr_domain_state;
			fvp_pwrlvl0_topology_map[pwrlvl0_offset].sibling =
				pwrlvl0_offset + 1;

			/* Increment the absolute number of pwrlvl0s
			   traversed */
			pwrlvl0_offset++;
		}

		/* Tie-off the last pwrlvl0 sibling to -1 to avoid overflow */
		fvp_pwrlvl0_topology_map[pwrlvl0_offset - 1].sibling =
							INVALID_PWR_DOMAIN;
	}

	/* Tie-off the last pwrlvl1 sibling to INVALID_PWR_DOMAIN to avoid
	   overflow */
	fvp_pwrlvl1_topology_map[pwrlvl1 - 1].sibling = INVALID_PWR_DOMAIN;

	topology_setup_done = 1;
	return 0;
}
