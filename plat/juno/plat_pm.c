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
#include <arch_helpers.h>
#include <arm_gic.h>
#include <cci.h>
#include <debug.h>
#include <errno.h>
#include <platform.h>
#include <platform_def.h>
#include <psci.h>
#include "juno_def.h"
#include "juno_private.h"
#include "scpi.h"

/*******************************************************************************
 * Private Juno function to program the mailbox for a cpu before it is released
 * from reset.
 ******************************************************************************/
static void juno_program_mailbox(uint64_t mpidr, uint64_t address)
{
	uint64_t linear_id;
	uint64_t mbox;

	linear_id = platform_get_core_pos(mpidr);
	mbox = TRUSTED_MAILBOXES_BASE +	(linear_id << TRUSTED_MAILBOX_SHIFT);
	*((uint64_t *) mbox) = address;
	flush_dcache_range(mbox, sizeof(mbox));
}

/*******************************************************************************
 * Juno handler called to check the validity of the power state parameter.
 ******************************************************************************/
int32_t juno_validate_power_state(unsigned int power_state,
		psci_power_state_t *req_state)
{
	int pstate = psci_get_pstate_type(power_state);
	int pwr_lvl = psci_get_pstate_pwrlvl(power_state);
	int i;

	assert(req_state);

	if (pwr_lvl > PLAT_MAX_PWR_LVL)
		return PSCI_E_INVALID_PARAMS;

	/* Sanity check the requested state */
	if (pstate == PSTATE_TYPE_STANDBY) {
		/*
		 * It's possible to enter standby only at power level 0 i.e.
		 * a cpu on the Juno. Ignore any other power level.
		 */
		if (pwr_lvl != JUNO_PWR_LVL0)
			return PSCI_E_INVALID_PARAMS;

		req_state->pwr_domain_state[JUNO_PWR_LVL0] = JUNO_PM_RET;
	} else {
		for (i = JUNO_PWR_LVL0; i <= pwr_lvl; i++)
			req_state->pwr_domain_state[i] = JUNO_PM_OFF;
	}

	/*
	 * We expect the 'state id' to be zero.
	 */
	if (psci_get_pstate_id(power_state))
		return PSCI_E_INVALID_PARAMS;

	return PSCI_E_SUCCESS;
}


/*******************************************************************************
 * Juno handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 ******************************************************************************/
int32_t juno_pwr_domain_on(uint64_t mpidr,
			uint64_t sec_entrypoint)
{
	/*
	 * SCP takes care of powering up parent power domains so we
	 * only need to care about level 0
	 */

	/*
	 * Setup mailbox with address for CPU entrypoint when it next powers up
	 */
	juno_program_mailbox(mpidr, sec_entrypoint);

	scpi_set_css_power_state(mpidr, scpi_power_on, scpi_power_on,
				 scpi_power_on);

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * Juno handler called when a power level has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
void juno_pwr_domain_on_finish(psci_power_state_t *target_state)
{
	unsigned long mpidr;

	assert(target_state->pwr_domain_state[JUNO_PWR_LVL0] == JUNO_PM_OFF);

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();

	/*
	 * Perform the common cluster specific operations i.e enable coherency
	 * if this cluster was off.
	 */
	if (target_state->pwr_domain_state[JUNO_PWR_LVL1] == JUNO_PM_OFF)
		cci_enable_snoop_dvm_reqs(MPIDR_AFFLVL1_VAL(mpidr));

	/* Enable the gic cpu interface */
	arm_gic_cpuif_setup();

	/* Juno todo: Is this setup only needed after a cold boot? */
	arm_gic_pcpu_distif_setup();

	/* Clear the mailbox for this cpu. */
	juno_program_mailbox(mpidr, 0);
}

/*******************************************************************************
 * Common function called while turning a cpu off or suspending it. It is
 * called from juno_off() or juno_suspend() when these functions in turn are
 * called for power domain at the highest power level which will be powered
 * down. It performs the actions common to the OFF and SUSPEND calls.
 ******************************************************************************/
static void juno_power_down_common(psci_power_state_t *target_state)
{
	uint32_t cluster_state = scpi_power_on;

	/* Prevent interrupts from spuriously waking up this cpu */
	arm_gic_cpuif_deactivate();

	/* Cluster is to be turned off, so disable coherency */
	if (target_state->pwr_domain_state[JUNO_PWR_LVL1] == JUNO_PM_OFF) {
		cci_disable_snoop_dvm_reqs(MPIDR_AFFLVL1_VAL(read_mpidr()));
		cluster_state = scpi_power_off;
	}

	/*
	 * Ask the SCP to power down the appropriate components depending upon
	 * their state.
	 */
	scpi_set_css_power_state(read_mpidr_el1(),
				 scpi_power_off,
				 cluster_state,
				 scpi_power_on);
}

/*******************************************************************************
 * Handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
static void juno_pwr_domain_off(psci_power_state_t *target_state)
{
	assert(target_state->pwr_domain_state[JUNO_PWR_LVL0] == JUNO_PM_OFF);

	juno_power_down_common(target_state);
}

/*******************************************************************************
 * Handler called when a power domain is about to be suspended. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
static void juno_pwr_domain_suspend(uint64_t sec_entrypoint,
				    psci_power_state_t *target_state)
{
	/*
	 * Juno has retention only at cpu level. Just return
	 * as nothing is to be done for retention.
	 */
	if (target_state->pwr_domain_state[JUNO_PWR_LVL0] == JUNO_PM_RET)
		return;

	assert(target_state->pwr_domain_state[JUNO_PWR_LVL0] == JUNO_PM_OFF);

	/*
	 * Setup mailbox with address for CPU entrypoint when it next powers up.
	 */
	juno_program_mailbox(read_mpidr_el1(), sec_entrypoint);

	juno_power_down_common(target_state);
}

/*******************************************************************************
 * Juno handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
static void juno_pwr_domain_suspend_finish(psci_power_state_t *target_state)
{
	/*
	 * Return as nothing is to be done on waking up from retention.
	 */
	if (target_state->pwr_domain_state[JUNO_PWR_LVL0] == JUNO_PM_RET)
		return;

	juno_pwr_domain_on_finish(target_state);
}

/*******************************************************************************
 * Juno handlers to shutdown/reboot the system
 ******************************************************************************/
static void __dead2 juno_system_off(void)
{
	uint32_t response;

	/* Send the power down request to the SCP */
	response = scpi_sys_power_state(scpi_system_shutdown);

	if (response != SCP_OK) {
		ERROR("Juno System Off: SCP error %u.\n", response);
		panic();
	}
	wfi();
	ERROR("Juno System Off: operation not handled.\n");
	panic();
}

static void __dead2 juno_system_reset(void)
{
	uint32_t response;

	/* Send the system reset request to the SCP */
	response = scpi_sys_power_state(scpi_system_reboot);

	if (response != SCP_OK) {
		ERROR("Juno System Reset: SCP error %u.\n", response);
		panic();
	}
	wfi();
	ERROR("Juno System Reset: operation not handled.\n");
	panic();
}

/*******************************************************************************
 * Handler called when the CPU power domain is about to enter standby.
 ******************************************************************************/
void juno_cpu_standby(plat_local_state_t cpu_state)
{
	unsigned int scr;

	assert(cpu_state == JUNO_PM_RET);

	scr = read_scr_el3();
	/* Enable PhysicalIRQ bit for NS world to wake the CPU */
	write_scr_el3(scr | SCR_IRQ_BIT);
	isb();
	dsb();
	wfi();

	/*
	 * Restore SCR to the original value, synchronisation of scr_el3 is
	 * done by eret while el3_exit to save some execution cycles.
	 */
	write_scr_el3(scr);
}

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t juno_ops = {
	.pwr_domain_on		= juno_pwr_domain_on,
	.pwr_domain_on_finish	= juno_pwr_domain_on_finish,
	.pwr_domain_off		= juno_pwr_domain_off,
	.cpu_standby		= juno_cpu_standby,
	.pwr_domain_suspend	= juno_pwr_domain_suspend,
	.pwr_domain_suspend_finish	= juno_pwr_domain_suspend_finish,
	.system_off		= juno_system_off,
	.system_reset		= juno_system_reset,
	.validate_power_state	= juno_validate_power_state
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int32_t platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &juno_ops;
	return 0;
}
