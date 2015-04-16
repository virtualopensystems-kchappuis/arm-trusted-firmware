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

#include <arch_helpers.h>
#include <arm_gic.h>
#include <assert.h>
#include <bakery_lock.h>
#include <cci.h>
#include <debug.h>
#include <mmio.h>
#include <platform.h>
#include <plat_config.h>
#include <platform_def.h>
#include <psci.h>
#include <errno.h>
#include "drivers/pwrc/fvp_pwrc.h"
#include "fvp_def.h"
#include "fvp_private.h"

#if RECOM_STATE_ID_ENC
/*
 *  The table storing the valid idle power states. Ensure that the
 *  array entries are populated in ascending order of state-id to
 *  enable us to use binary search during power state validation.
 */
const uint32_t fvp_idle_states[] = {
	/* State-id - 0x01 */
	fvp_make_pwr_state(FVP_PM_RUN, FVP_PM_RET,
			FVP_PWR_LVL0, PSTATE_TYPE_STANDBY),
	/* State-id - 0x02 */
	fvp_make_pwr_state(FVP_PM_RUN, FVP_PM_OFF,
			FVP_PWR_LVL0, PSTATE_TYPE_POWERDOWN),
	/* State-id - 0x22 */
	fvp_make_pwr_state(FVP_PM_OFF, FVP_PM_OFF,
			FVP_PWR_LVL1, PSTATE_TYPE_POWERDOWN),
};
#endif /* __RECOM_STATE_ID_ENC__ */

/*******************************************************************************
 * Private FVP function to program the mailbox for a cpu before it is released
 * from reset.
 ******************************************************************************/
static void fvp_program_mailbox(uint64_t mpidr, uint64_t address)
{
	uint64_t linear_id;
	mailbox_t *fvp_mboxes;

	linear_id = platform_get_core_pos(mpidr);
	fvp_mboxes = (mailbox_t *)MBOX_BASE;
	fvp_mboxes[linear_id].value = address;
	flush_dcache_range((unsigned long) &fvp_mboxes[linear_id],
			   sizeof(unsigned long));
}

/*******************************************************************************
 * Function which implements the common FVP specific operations to power down a
 * cpu in response to a CPU_OFF or CPU_SUSPEND request.
 ******************************************************************************/
static void fvp_cpu_pwrdwn_common(void)
{
	/* Prevent interrupts from spuriously waking up this cpu */
	arm_gic_cpuif_deactivate();

	/* Program the power controller to power off this cpu. */
	fvp_pwrc_write_ppoffr(read_mpidr_el1());
}

/*******************************************************************************
 * Function which implements the common FVP specific operations to power down a
 * cluster in response to a CPU_OFF or CPU_SUSPEND request.
 ******************************************************************************/
static void fvp_cluster_pwrdwn_common(void)
{
	uint64_t mpidr = read_mpidr_el1();

	/* Disable coherency if this cluster is to be turned off */
	fvp_cci_disable();

	/* Program the power controller to turn the cluster off */
	fvp_pwrc_write_pcoffr(mpidr);
}

/*******************************************************************************
 * FVP handler called when a CPU is about to enter standby.
 ******************************************************************************/
void fvp_cpu_standby(plat_local_state_t cpu_state)
{

	assert(cpu_state == FVP_PM_RET);

	/*
	 * Enter standby state
	 * dsb is good practice before using wfi to enter low power states
	 */
	dsb();
	wfi();
}

/*******************************************************************************
 * FVP handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 ******************************************************************************/
int fvp_pwr_domain_on(unsigned long mpidr,
		   unsigned long sec_entrypoint)
{
	int rc = PSCI_E_SUCCESS;
	unsigned int psysr;

	/*
	 * Ensure that we do not cancel an inflight power off request
	 * for the target cpu. That would leave it in a zombie wfi.
	 * Wait for it to power off, program the jump address for the
	 * target cpu and then program the power controller to turn
	 * that cpu on
	 */
	do {
		psysr = fvp_pwrc_read_psysr(mpidr);
	} while (psysr & PSYSR_AFF_L0);

	fvp_program_mailbox(mpidr, sec_entrypoint);
	fvp_pwrc_write_pponr(mpidr);

	return rc;
}

/*******************************************************************************
 * FVP handler called when a power domain is about to be turned off. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void fvp_pwr_domain_off(psci_power_state_t *target_state)
{
	assert(target_state->pwr_domain_state[FVP_PWR_LVL0] == FVP_PM_OFF);

	/*
	 * If execution reaches this stage then this power domain will be
	 * suspended. Perform at least the cpu specific actions followed
	 * by the cluster specific operations if applicable.
	 */
	fvp_cpu_pwrdwn_common();

	if (target_state->pwr_domain_state[FVP_PWR_LVL1] == FVP_PM_OFF)
		fvp_cluster_pwrdwn_common();

}

/*******************************************************************************
 * FVP handler called when a power domain is about to be suspended. The
 * target_state encodes the power state that each level should transition to.
 ******************************************************************************/
void fvp_pwr_domain_suspend(unsigned long sec_entrypoint,
			psci_power_state_t *target_state)
{
	unsigned long mpidr;

	/*
	 * FVP has retention only at cpu level. Just return
	 * as nothing is to be done for retention.
	 */
	if (target_state->pwr_domain_state[FVP_PWR_LVL0] == FVP_PM_RET)
		return;

	assert(target_state->pwr_domain_state[FVP_PWR_LVL0] == FVP_PM_OFF);

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();

	/* Program the jump address for the this cpu */
	fvp_program_mailbox(mpidr, sec_entrypoint);

	/* Program the power controller to enable wakeup interrupts. */
	fvp_pwrc_set_wen(mpidr);

	/* Perform the common cpu specific operations */
	fvp_cpu_pwrdwn_common();

	/* Perform the common cluster specific operations */
	if (target_state->pwr_domain_state[FVP_PWR_LVL1] == FVP_PM_OFF)
		fvp_cluster_pwrdwn_common();
}

/*******************************************************************************
 * FVP handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 ******************************************************************************/
void fvp_pwr_domain_on_finish(psci_power_state_t *target_state)
{
	unsigned long mpidr;

	assert(target_state->pwr_domain_state[FVP_PWR_LVL0] == FVP_PM_OFF);

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();

	/* Perform the common cluster specific operations */
	if (target_state->pwr_domain_state[FVP_PWR_LVL1] == FVP_PM_OFF) {
		/*
		 * This CPU might have woken up whilst the cluster was
		 * attempting to power down. In this case the FVP power
		 * controller will have a pending cluster power off request
		 * which needs to be cleared by writing to the PPONR register.
		 * This prevents the power controller from interpreting a
		 * subsequent entry of this cpu into a simple wfi as a power
		 * down request.
		 */
		fvp_pwrc_write_pponr(mpidr);

		/* Enable coherency if this cluster was off */
		fvp_cci_enable();
	}

	/*
	 * Clear PWKUPR.WEN bit to ensure interrupts do not interfere
	 * with a cpu power down unless the bit is set again
	 */
	fvp_pwrc_clr_wen(mpidr);

	/* Zero the jump address in the mailbox for this cpu */
	fvp_program_mailbox(mpidr, 0);

	/* Enable the gic cpu interface */
	arm_gic_cpuif_setup();

	/* TODO: This setup is needed only after a cold boot */
	arm_gic_pcpu_distif_setup();
}

/*******************************************************************************
 * FVP handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 * TODO: At the moment we reuse the on finisher and reinitialize the secure
 * context. Need to implement a separate suspend finisher.
 ******************************************************************************/
void fvp_pwr_domain_suspend_finish(psci_power_state_t *target_state)
{
	/*
	 * Nothing to be done on waking up from retention from CPU level.
	 */
	if (target_state->pwr_domain_state[FVP_PWR_LVL0] == FVP_PM_RET)
		return;

	fvp_pwr_domain_on_finish(target_state);
}

/*******************************************************************************
 * FVP handlers to shutdown/reboot the system
 ******************************************************************************/
static void __dead2 fvp_system_off(void)
{
	/* Write the System Configuration Control Register */
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_SHUTDOWN));
	wfi();
	ERROR("FVP System Off: operation not handled.\n");
	panic();
}

static void __dead2 fvp_system_reset(void)
{
	/* Write the System Configuration Control Register */
	mmio_write_32(VE_SYSREGS_BASE + V2M_SYS_CFGCTRL,
		CFGCTRL_START | CFGCTRL_RW | CFGCTRL_FUNC(FUNC_REBOOT));
	wfi();
	ERROR("FVP System Reset: operation not handled.\n");
	panic();
}

#if !RECOM_STATE_ID_ENC
/*******************************************************************************
 * FVP handler called to check the validity of the power state parameter.
 ******************************************************************************/
int fvp_validate_power_state(unsigned int power_state,
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
		 * It's possible to enter standby only on power level 0
		 * i.e. a cpu on the fvp. Ignore any other power level.
		 */
		if (pwr_lvl != FVP_PWR_LVL0)
			return PSCI_E_INVALID_PARAMS;

		req_state->pwr_domain_state[FVP_PWR_LVL0] = FVP_PM_RET;
	} else {
		for (i = FVP_PWR_LVL0; i <= pwr_lvl; i++)
			req_state->pwr_domain_state[i] = FVP_PM_OFF;
	}

	/*
	 * We expect the 'state id' to be zero.
	 */
	if (psci_get_pstate_id(power_state))
		return PSCI_E_INVALID_PARAMS;

	return PSCI_E_SUCCESS;
}

#else
/*******************************************************************************
 * FVP handler called to check the validity of the power state parameter. The
 * power state parameter has to be a composite power state.
 ******************************************************************************/
int fvp_validate_power_state(unsigned int power_state,
				psci_power_state_t *req_state)
{
	unsigned int state_id;
	int i, idle_states_num =  sizeof(fvp_idle_states)/
					sizeof(fvp_idle_states[0]);

	assert(req_state);

	/*
	 *  Currently we are using a linear search for finding the matching
	 *  entry in the idle power state array. This can be made a binary
	 *  search if the number of entries justify the additional complexity.
	 */
	for (i = 0; i < idle_states_num; i++) {
		if (power_state == fvp_idle_states[i])
			break;
	}

	/* Return error if entry not found in the idle state array */
	if (i == idle_states_num)
		return PSCI_E_INVALID_PARAMS;

	i = 0;
	state_id = psci_get_pstate_id(power_state);

	/* Parse the State ID and populate the state info parameter */
	while (state_id) {
		req_state->pwr_domain_state[i++] = state_id &
						FVP_LOCAL_PSTATE_MASK;
		state_id >>= FVP_LOCAL_PSTATE_WIDTH;
	}

	return PSCI_E_SUCCESS;
}
#endif /* __RECOM_STATE_ID_ENC__ */

/*******************************************************************************
 * Export the platform handlers to enable psci to invoke them
 ******************************************************************************/
static const plat_pm_ops_t fvp_plat_pm_ops = {
	.cpu_standby = fvp_cpu_standby,
	.pwr_domain_on = fvp_pwr_domain_on,
	.pwr_domain_off = fvp_pwr_domain_off,
	.pwr_domain_suspend = fvp_pwr_domain_suspend,
	.pwr_domain_on_finish = fvp_pwr_domain_on_finish,
	.pwr_domain_suspend_finish = fvp_pwr_domain_suspend_finish,
	.system_off = fvp_system_off,
	.system_reset = fvp_system_reset,
	.validate_power_state = fvp_validate_power_state
};

/*******************************************************************************
 * Export the platform specific power ops & initialize the fvp power controller
 ******************************************************************************/
int platform_setup_pm(const plat_pm_ops_t **plat_ops)
{
	*plat_ops = &fvp_plat_pm_ops;
	return 0;
}
