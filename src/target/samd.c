/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2014  Richard Meadows <richardeoin>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements Atmel SAM D target specific functions for
 * detecting the device, providing the XML memory map and Flash memory
 * programming.
 *
 * Tested with
 * * SAMD20E17A (rev C)
 * * SAMD20J18A (rev B)
 * * SAMD21J18A (rev B)
 * *
 */
/* Refer to the SAM D20 Datasheet:
 * http://www.atmel.com/Images/Atmel-42129-SAM-D20_Datasheet.pdf
 * particularly Sections 12. DSU and 20. NVMCTRL
 */

#include "general.h"
#include "target.h"
#include "target_internal.h"
#include "cortexm.h"

#define UNUSED(v)									(void)(v)

static int samd_flash_erase(struct target_flash*,target_addr,size_t);
static int samd_flash_write(struct target_flash*,target_addr,const void*,size_t);

static int samd5_flash_erase(struct target_flash*,target_addr,size_t);
static int samd5_flash_write(struct target_flash*,target_addr,const void*,size_t);

static bool samd_cmd_erase_all(target *t);
static bool samd_cmd_lock_flash(target *t);
static bool samd_cmd_unlock_flash(target *t);
static bool samd_cmd_read_userrow(target *t);
static bool samd_cmd_serial(target *t);
static bool samd_cmd_mbist(target *t);
static bool samd_cmd_ssb(target *t);

const struct command_s samd_cmd_list[] = {
	{"erase_mass", (cmd_handler)samd_cmd_erase_all, "Erase entire flash memory"},
	{"lock_flash", (cmd_handler)samd_cmd_lock_flash, "Locks flash against spurious commands"},
	{"unlock_flash", (cmd_handler)samd_cmd_unlock_flash, "Unlocks flash"},
	{"user_row", (cmd_handler)samd_cmd_read_userrow, "Prints user row from flash"},
	{"serial", (cmd_handler)samd_cmd_serial, "Prints serial number"},
	{"mbist", (cmd_handler)samd_cmd_mbist, "Runs the built-in memory test"},
	{"set_security_bit", (cmd_handler)samd_cmd_ssb, "Sets the Security Bit"},
	{NULL, NULL, NULL}
};

static bool samd5_cmd_lock_flash(target *t);
static bool samd5_cmd_unlock_flash(target *t);
static bool samd5_cmd_serial(target *t);

const struct command_s samd5_cmd_list[] = {
	{"erase_mass", (cmd_handler)samd_cmd_erase_all, "Erase entire flash memory"},
	{"lock_flash", (cmd_handler)samd5_cmd_lock_flash, "Locks flash against spurious commands"},
	{"unlock_flash", (cmd_handler)samd5_cmd_unlock_flash, "Unlocks flash"},
	{"user_row", (cmd_handler)samd_cmd_read_userrow, "Prints user row from flash"},
	{"serial", (cmd_handler)samd5_cmd_serial, "Prints serial number"},
	{"mbist", (cmd_handler)samd_cmd_mbist, "Runs the built-in memory test"},
	{"set_security_bit", (cmd_handler)samd_cmd_ssb, "Sets the Security Bit"},
	{NULL, NULL, NULL}
};

/* Non-Volatile Memory Controller (NVMC) Parameters */
#define SAMD_ROW_SIZE				256
#define SAMD_PAGE_SIZE				64

#define SAMD5_FLASH_BLOCK_SIZE	8192	// smalest erasable chunk of flash
#define SAMD5_FLASH_PAGE_SIZE		512	// largest chunk writable in s 'single' operation


/* -------------------------------------------------------------------------- */
/* D1x, D2x Non-Volatile Memory Controller (NVMC) Registers */
/* -------------------------------------------------------------------------- */

#define SAMD_NVMC							0x41004000
#define SAMD_NVMC_CTRLA					(SAMD_NVMC + 0x0)
#define SAMD_NVMC_CTRLB					(SAMD_NVMC + 0x04)
#define SAMD_NVMC_PARAM					(SAMD_NVMC + 0x08)

#define SAMD_NVMC_INT_STATUS			(SAMD_NVMC + 0x14)

#define SAMD_NVMC_STATUS				(SAMD_NVMC + 0x18)
#define SAMD_NVMC_ADDRESS				(SAMD_NVMC + 0x1C)

/* Control A Register (CTRLA) */
#define SAMD_CTRLA_CMD_KEY				0xA500
#define SAMD_CTRLA_CMD_ERASEROW		0x0002
#define SAMD_CTRLA_CMD_WRITEPAGE		0x0004
#define SAMD_CTRLA_CMD_ERASEAUXROW	0x0005
#define SAMD_CTRLA_CMD_WRITEAUXPAGE	0x0006
#define SAMD_CTRLA_CMD_LOCK			0x0040
#define SAMD_CTRLA_CMD_UNLOCK			0x0041
#define SAMD_CTRLA_CMD_PAGEBUFFERCLEAR	0x0044
#define SAMD_CTRLA_CMD_SSB				0x0045
#define SAMD_CTRLA_CMD_INVALL			0x0046

/* Interrupt Flag Register (INTFLAG) */
#define SAMD_NVMC_READY			(1 << 0)


/* -------------------------------------------------------------------------- */
/* D51 Non-Volatile Memory Controller (NVMC) Registers */
/* -------------------------------------------------------------------------- */
#define SAMD5_NVMC						0x41004000

#define SAMD5_NVMC_CTRLA				(SAMD5_NVMC + 0x0)
#define SAMD5_NVMC_CTRLB				(SAMD5_NVMC + 0x04)
#define SAMD5_NVMC_PARAM				(SAMD5_NVMC + 0x08)
#define SAMD5_NVMC_INT_STATUS			(SAMD5_NVMC + 0x10)		// 16bit
#define SAMD5_NVMC_STATUS				(SAMD5_NVMC + 0x12)		// 16bit
#define SAMD5_NVMC_ADDRESS				(SAMD5_NVMC + 0x14)

#define SAMD5_NVMC_CTRLB_CMDEX_KEY	0xa500
#define SAMD5_NVMC_CTRLB_CMD_UR		0x12
#define SAMD5_NVMC_CTRLB_CMD_EB		0x01
#define SAMD5_NVMC_CTRLB_CMD_WP		0x03

#define SAMD5_NVMC_STATUS_READY		(1 << 0)


/* Non-Volatile Memory Calibration and Auxiliary Registers */
#define SAMD_NVM_USER_ROW_LOW		0x00804000
#define SAMD_NVM_USER_ROW_HIGH	0x00804004
#define SAMD_NVM_CALIBRATION		0x00806020
#define SAMD_NVM_SERIAL(n)			(0x0080A00C + (0x30 * ((n + 3) / 4)) + (0x4 * n))

#define SAMD5_NVM_SERIAL(n)		(0x008061FC + (0x4 * n))

/* -------------------------------------------------------------------------- */
/* Device Service Unit (DSU) Registers */
/* -------------------------------------------------------------------------- */

#define SAMD_DSU						0x41002000	// D5x OK
#define SAMD_DSU_EXT_ACCESS		(SAMD_DSU + 0x100)
#define SAMD_DSU_CTRLSTAT			(SAMD_DSU_EXT_ACCESS + 0x0)
#define SAMD_DSU_ADDRESS			(SAMD_DSU_EXT_ACCESS + 0x4)
#define SAMD_DSU_LENGTH				(SAMD_DSU_EXT_ACCESS + 0x8)
#define SAMD_DSU_DID					(SAMD_DSU_EXT_ACCESS + 0x018)
#define SAMD_DSU_PID(n)				(SAMD_DSU + 0x1FE0 + \
												(0x4 * (n % 4)) - (0x10 * (n / 4)))
#define SAMD_DSU_CID(n)				(SAMD_DSU + 0x1FF0 + \
												(0x4 * (n % 4)))

/* DSU Control and Status Register (CTRLSTAT)  D5x OK */
#define SAMD_CTRL_SWRST				(1 << 0)
#define SAMD_CTRL_CRC				(1 << 2)
#define SAMD_CTRL_MBIST				(1 << 3)
#define SAMD_CTRL_CHIP_ERASE		(1 << 4)
#define SAMD_STATUSA_DONE			(1 << 8)
#define SAMD_STATUSA_CRSTEXT		(1 << 9)
#define SAMD_STATUSA_BERR			(1 << 10)
#define SAMD_STATUSA_FAIL			(1 << 11)
#define SAMD_STATUSA_PERR			(1 << 12)
#define SAMD_STATUSB_PROT			(1 << 16)

/* Device Identification Register (DID) D5x OK */
#define SAMD_DID_MASK				0xFFBC0000
#define SAMD_DID_CONST_VALUE		0x10000000
#define SAMD_DID_DEVSEL_MASK		0xFF
#define SAMD_DID_DEVSEL_POS		0
#define SAMD_DID_REVISION_MASK	0x0F
#define SAMD_DID_REVISION_POS		8
#define SAMD_DID_SERIES_MASK		0x3F
#define SAMD_DID_SERIES_POS		16

#define SAMD_DID_FAMILY_MASK		0x1F
#define SAMD_DID_FAMILY_POS		23

#define SAMD_DID_PROCESSOR_MASK	0x0F
#define SAMD_DID_PROCESSOR_POS	28

/* Peripheral ID */
#define SAMD_PID_MASK				0x00F7FFFF
#define SAMD_PID_CONST_VALUE		0x0001FCD0

/* Component ID */
#define SAMD_CID_VALUE				0xB105100D

/**
 * Reads the SAM D20 Peripheral ID
 */
uint64_t samd_read_pid(target *t)
{
	uint64_t pid = 0;
	uint8_t i, j;

	/* Five PID registers to read LSB first */
	for (i = 0, j = 0; i < 5; i++, j += 8)
		pid |= (target_mem_read32(t, SAMD_DSU_PID(i)) & 0xFF) << j;

	return pid;
}
/**
 * Reads the SAM D20 Component ID
 */
uint32_t samd_read_cid(target *t)
{
	uint64_t cid = 0;
	uint8_t i, j;

	/* Four CID registers to read LSB first */
	for (i = 0, j = 0; i < 4; i++, j += 8)
		cid |= (target_mem_read32(t, SAMD_DSU_CID(i)) & 0xFF) << j;

	return cid;
}

/**
 * Overloads the default cortexm reset function with a version that
 * removes the target from extended reset where required.
 */
static void
samd_reset(target *t)
{
	/**
	 * SRST is not asserted here as it appears to reset the adiv5
	 * logic, meaning that subsequent adiv5_* calls PLATFORM_FATAL_ERROR.
	 *
	 * This is ok as normally you can just connect the debugger and go,
	 * but if that's not possible (protection or SWCLK being used for
	 * something else) then having SWCLK low on reset should get you
	 * debug access (cold-plugging). TODO: Confirm this
	 *
	 * See the SAM D20 datasheet §12.6 Debug Operation for more
	 * details.
	 *
	 * jtagtap_srst(true);
	 * jtagtap_srst(false);
	 */

	/* Read DHCSR here to clear S_RESET_ST bit before reset */
	target_mem_read32(t, CORTEXM_DHCSR);

	/* Request system reset from NVIC: SRST doesn't work correctly */
	/* This could be VECTRESET: 0x05FA0001 (reset only core)
	 *          or SYSRESETREQ: 0x05FA0004 (system reset)
	 */
	target_mem_write32(t, CORTEXM_AIRCR,
	                   CORTEXM_AIRCR_VECTKEY | CORTEXM_AIRCR_SYSRESETREQ);

	/* Exit extended reset */
	if (target_mem_read32(t, SAMD_DSU_CTRLSTAT) &
	    SAMD_STATUSA_CRSTEXT) {
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_CRSTEXT);
	}

	/* Poll for release from reset */
	while (target_mem_read32(t, CORTEXM_DHCSR) & CORTEXM_DHCSR_S_RESET_ST);

	/* Reset DFSR flags */
	target_mem_write32(t, CORTEXM_DFSR, CORTEXM_DFSR_RESETALL);

	/* Clear any target errors */
	target_check_error(t);
}

/**
 * Overloads the default cortexm detached function with a version that
 * removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void
samd20_revB_detach(target *t)
{
	cortexm_detach(t);

	/* ---- Additional ---- */
	/* Exit extended reset */
	if (target_mem_read32(t, SAMD_DSU_CTRLSTAT) &
	    SAMD_STATUSA_CRSTEXT) {
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMD_DSU_CTRLSTAT,
		                   SAMD_STATUSA_CRSTEXT);
	}
}

/**
 * Overloads the default cortexm halt_resume function with a version
 * that removes the target from extended reset where required.
 *
 * Only required for SAM D20 _Revision B_ Silicon
 */
static void
samd20_revB_halt_resume(target *t, bool step)
{
	cortexm_halt_resume(t, step);

	/* ---- Additional ---- */
	/* Exit extended reset */
	if (target_mem_read32(t, SAMD_DSU_CTRLSTAT) & SAMD_STATUSA_CRSTEXT) {
		/* Write bit to clear from extended reset */
		target_mem_write32(t, SAMD_DSU_CTRLSTAT,
		                   SAMD_STATUSA_CRSTEXT);
	}
}

/**
 * Overload the default cortexm attach for when the samd is protected.
 *
 * If the samd is protected then the default cortexm attach will
 * fail as the S_HALT bit in the DHCSR will never go high. This
 * function allows users to attach on a temporary basis so they can
 * rescue the device.
 */
static bool
samd_protected_attach(target *t)
{
	/**
	 * TODO: Notify the user that we're not really attached and
	 * they should issue the 'monitor erase_mass' command to
	 * regain access to the chip.
	 */

	/* Patch back in the normal cortexm attach for next time */
	t->attach = cortexm_attach;

	/* Allow attach this time */
	return true;
}


/**
 * Use the DSU Device Indentification Register to populate a struct
 * describing the SAM D device.
 */
struct samd_descr {
	uint8_t processor;
	uint8_t family;
	uint8_t series;
	char revision;
	char pin;
	uint8_t mem;
	char package[3];
};
struct samd_descr samd_parse_device_id(uint32_t did)
{
	struct samd_descr samd;
	memset(samd.package, 0, 3);

	uint8_t series = (did >> SAMD_DID_SERIES_POS) & SAMD_DID_SERIES_MASK;
	uint8_t revision = (did >> SAMD_DID_REVISION_POS) & SAMD_DID_REVISION_MASK;
	uint8_t devsel = (did >> SAMD_DID_DEVSEL_POS) & SAMD_DID_DEVSEL_MASK;

	samd.processor = (did >> SAMD_DID_PROCESSOR_POS) & SAMD_DID_PROCESSOR_MASK;
	samd.family = (did >> SAMD_DID_FAMILY_POS) & SAMD_DID_FAMILY_MASK;
	
	/* Series */
	switch (series) {
		case 0: samd.series = 20; break;
		case 1: samd.series = 21; break;
		case 2: samd.series = 10; break;
		case 3: samd.series = 11; break;
		case 6: samd.series = 51; break;
	}
	/* Revision */
	samd.revision = 'A' + revision;

	switch (samd.series) {
	case 20: /* SAM D20 */
	case 21: /* SAM D21/R21 */
		if (devsel < 0x18) {
			switch (devsel / 5) {
				case 0: samd.pin = 'J'; break;
				case 1: samd.pin = 'G'; break;
				case 2: samd.pin = 'E'; break;
				default: samd.pin = 'u'; break;
			}
			samd.mem = 18 - (devsel % 5);
		} else {
			//samd.device = 'R';
			samd.pin = 'R';
			switch (devsel) {
				default:
				case 0x18: samd.mem = 19; break;
				case 0x19: samd.mem = 18; break;
				case 0x1a: samd.mem = 17; break;
				case 0x1b: samd.mem = 16; break;
				case 0x1c: samd.mem = 18; break;
				case 0x1d: samd.mem = 17; break;
				case 0x1e: samd.mem = 16; break;
			}
		}
		break;
	case 10: /* SAM D10 */
	case 11: /* SAM D11 */
		switch (devsel / 3) {
			case 0: samd.package[0] = 'M'; break;
			case 1: samd.package[0] = 'S'; samd.package[1] = 'S'; break;
		}
		samd.pin = 'D';
		samd.mem = 14 - (devsel % 3);
		break;
	case 51: /* SAMD51 */
		switch (devsel) {
			// SAMD51G18
			// SAMD51G19
			// SAMD51J18

			// SAMD51J19
			case 7:
				samd.pin = 'J';
				samd.mem = 19;
			break;

			// SAMD51J20
			// SAMD51N19
			
			// SAMD51N20
			case 2:						//
				samd.pin = 'N';
				samd.mem = 20;
			break;
		}
		break;
	}
	
	return samd;
}

static void samd_add_flash(target *t, uint32_t addr, size_t length)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	f->start = addr;
	f->length = length;
	f->blocksize = SAMD_ROW_SIZE;
	f->erase = samd_flash_erase;
	f->write = samd_flash_write;
	f->buf_size = SAMD_PAGE_SIZE;
	target_add_flash(t, f);
}

static void samd5_add_flash(target *t, uint32_t addr, size_t length)
{
	struct target_flash *f = calloc(1, sizeof(*f));
	f->start = addr;
	f->length = length;
	f->blocksize = SAMD5_FLASH_BLOCK_SIZE;
	f->erase = samd5_flash_erase;
	f->write = samd5_flash_write;
	f->buf_size = SAMD5_FLASH_PAGE_SIZE;
	target_add_flash(t, f);
}


char variant_string[40];
bool samd_probe(target *t)
{
	uint32_t cid = samd_read_cid(t);
	uint32_t pid = samd_read_pid(t);
	uint32_t did = target_mem_read32(t, SAMD_DSU_DID);
	struct samd_descr samd = samd_parse_device_id(did);

	t->driver = variant_string;
	// d51N20:	0xb105100d 0x0009fcd0 0x060060002

	sprintf(variant_string,"xxx 0x%08x 0x%08x 0x%08x",(int)cid,(int)pid,(int)did);

	/* Check the ARM Coresight Component and Perhiperal IDs */
	if (
			(cid != SAMD_CID_VALUE)
		||	((pid & SAMD_PID_MASK) != SAMD_PID_CONST_VALUE)
	) {
		return false;
	}
	
	if (
		!(
				(samd.processor == 1)	// samd1x, samd2x
			||	(samd.processor == 6)	// samd51
		)
	) {
		return false;
	}
	
	uint32_t ctrlstat = target_mem_read32(t, SAMD_DSU_CTRLSTAT);

	/* Protected? */
	bool protected = (ctrlstat & SAMD_STATUSB_PROT);

	/* Part String */
	if (protected) {
		sprintf(variant_string,
		        "Atmel SAMD%d%c%dA%s (rev %c) (PROT=1)",
		        samd.series, samd.pin, samd.mem,
		        samd.package, samd.revision);
	} else {
		sprintf(variant_string,
		        "Atmel SAMD%d%c%dA%s (rev %c) 0x%08x",
		        samd.series, samd.pin, samd.mem,
		        samd.package, samd.revision,(int)did);
	}

	/* Setup Target */
	t->driver = variant_string;
	t->reset = samd_reset;

	if (samd.series == 20 && samd.revision == 'B') {
		/**
		 * These functions check for and
		 * extended reset. Appears to be
		 * related to Errata 35.4.1 ref 12015
		 */
		t->detach      = samd20_revB_detach;
		t->halt_resume = samd20_revB_halt_resume;
	}
	if (protected) {
		/**
		 * Overload the default cortexm attach
		 * for when the samd is protected.
		 * This function allows users to
		 * attach on a temporary basis so they
		 * can rescue the device.
		 */
		t->attach = samd_protected_attach;
	}

	#define KB(n)	((n) * 1024)

	switch (samd.series) {
		case 51:
			switch (samd.mem) {
				case 18:
					target_add_ram(t, 0x20000000, KB(128));
					samd5_add_flash(t, 0x00000000, KB(256));
				break;
				
				case 19:
					target_add_ram(t, 0x20000000, KB(192));
					samd5_add_flash(t, 0x00000000, KB(512));
				break;

				case 20:
					target_add_ram(t, 0x20000000, KB(256));
					samd5_add_flash(t, 0x00000000, KB(1024));
				break;
				
				default:
					target_add_ram(t, 0x20000000, KB(32));
					samd5_add_flash(t, 0x00000000, KB(256));
				break;
			}
			target_add_commands(t, samd5_cmd_list, "SAMD5");
		break;
		
		// D1x, D2x
		default:
			target_add_ram(t, 0x20000000, KB(32));
			samd_add_flash(t, 0x00000000, KB(256));
			target_add_commands(t, samd_cmd_list, "SAMD");
		break;
	}
	

	/* If we're not in reset here */
	if (!platform_srst_get_val()) {
		/* We'll have to release the target from
		 * extended reset to make attach possible */
		if (target_mem_read32(t, SAMD_DSU_CTRLSTAT) & SAMD_STATUSA_CRSTEXT) {

			/* Write bit to clear from extended reset */
			target_mem_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_CRSTEXT);
		}
	}

	return true;
}

/**
 * Temporary (until next reset) flash memory locking / unlocking
 */
static void samd_lock_current_address(target *t)
{
	/* Issue the lock command */
	target_mem_write32(t, SAMD_NVMC_CTRLA,
	                   SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_LOCK);
}
static void samd_unlock_current_address(target *t)
{
	/* Issue the unlock command */
	target_mem_write32(t, SAMD_NVMC_CTRLA,
	                   SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_UNLOCK);
}

/**
 * Erase flash row by row
 */
static int samd_flash_erase(struct target_flash *f, target_addr addr, size_t len)
{
	target *t = f->t;
	while (len) {
		/* Write address of first word in row to erase it */
		/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
		target_mem_write32(t, SAMD_NVMC_ADDRESS, addr >> 1);

		/* Unlock */
		samd_unlock_current_address(t);

		/* Issue the erase command */
		target_mem_write32(t, SAMD_NVMC_CTRLA,
		                   SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_ERASEROW);
		/* Poll for NVM Ready */
		while ((target_mem_read32(t, SAMD_NVMC_INT_STATUS) & SAMD_NVMC_READY) == 0)
			if (target_check_error(t))
				return -1;

		/* Lock */
		samd_lock_current_address(t);

		addr += f->blocksize;
		len -= f->blocksize;
	}

	return 0;
}

/**
 * Write flash page by page
 */
static int samd_flash_write(struct target_flash *f,
                            target_addr dest, const void *src, size_t len)
{
	target *t = f->t;

	// must be 16bit or 32bit writes

	/* Write within a single page. This may be part or all of the page */
	target_mem_write(t, dest, src, len);

	/* Unlock */
	samd_unlock_current_address(t);

	/* Issue the write page command */
	target_mem_write32(t, SAMD_NVMC_CTRLA,
	                   SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_WRITEPAGE);

	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMD_NVMC_INT_STATUS) & SAMD_NVMC_READY) == 0)
		if (target_check_error(t))
			return -1;

	/* Lock */
	samd_lock_current_address(t);

	return 0;
}

/**
 * Erase flash row by row
 */
static int samd5_flash_erase(struct target_flash *f, target_addr addr, size_t len)
{
	target *t = f->t;
	
	while (len) {
		/* Write address of first word in row to erase it */
		/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
		//target_mem_write32(t, SAMD_NVMC_ADDRESS, addr >> 1);

		while((target_mem_read32(t, SAMD5_NVMC_STATUS) & SAMD5_NVMC_STATUS_READY) == 0);

		/* Unlock */
		//samd_unlock_current_address(t);
		target_mem_write32 (t,SAMD5_NVMC_ADDRESS,addr);
		target_mem_write32 (
			t,
			SAMD5_NVMC_CTRLB,
			(
					SAMD5_NVMC_CTRLB_CMDEX_KEY
				|	SAMD5_NVMC_CTRLB_CMD_UR
			)
		);

		/* Issue the erase command */
		target_mem_write32 (t,SAMD5_NVMC_ADDRESS,addr);
		target_mem_write32 (
			t,
			SAMD5_NVMC_CTRLB,
			(
					SAMD5_NVMC_CTRLB_CMDEX_KEY
				|	SAMD5_NVMC_CTRLB_CMD_EB
			)
		);
	
		/* Poll for NVM Ready */
		while((target_mem_read32(t, SAMD5_NVMC_STATUS) & SAMD5_NVMC_STATUS_READY) == 0) {
			if (target_check_error(t))
				return -1;
		}

		/* Lock */
		//samd_lock_current_address(t);

		addr += f->blocksize;
		len -= f->blocksize;
	}

	return 0;
}

/**
 * Write flash page by page
 */
static int samd5_flash_write (
	struct target_flash *f,target_addr dest,const void *src, size_t len
) {
	target *t = f->t;
	
	
	/* Unlock */
	target_mem_write32 (t,SAMD5_NVMC_ADDRESS,dest);
	target_mem_write32 (
		t,
		SAMD5_NVMC_CTRLB,
		(
				SAMD5_NVMC_CTRLB_CMDEX_KEY
			|	SAMD5_NVMC_CTRLB_CMD_UR
		)
	);

	// must be 32bit writes
	
	/* Write within a single page. This may be part or all of the page */
	target_mem_write(t, dest, src, len);
	
	/* Issue the write page command */
	target_mem_write32 (
		t,
		SAMD5_NVMC_CTRLB,
		(
				SAMD5_NVMC_CTRLB_CMDEX_KEY
			|	SAMD5_NVMC_CTRLB_CMD_WP
		)
	);

	/* Poll for NVM Ready */
	while((target_mem_read32(t, SAMD5_NVMC_STATUS) & SAMD5_NVMC_STATUS_READY) == 0) {
		if (target_check_error(t))
			return -1;
	}

	return 0;
}

/**
 * Uses the Device Service Unit to erase the entire flash
 */
static bool samd_cmd_erase_all(target *t)
{
	/* Clear the DSU status bits */
	target_mem_write32(t, SAMD_DSU_CTRLSTAT,
	                   SAMD_STATUSA_DONE | SAMD_STATUSA_PERR |
	                   SAMD_STATUSA_FAIL);

	/* Erase all */
	target_mem_write32(t, SAMD_DSU_CTRLSTAT, SAMD_CTRL_CHIP_ERASE);

	/* Poll for DSU Ready */
	uint32_t status;
	while (((status = target_mem_read32(t, SAMD_DSU_CTRLSTAT)) &
		(SAMD_STATUSA_DONE | SAMD_STATUSA_PERR | SAMD_STATUSA_FAIL)) == 0)
		if (target_check_error(t))
			return false;

	/* Test the protection error bit in Status A */
	if (status & SAMD_STATUSA_PERR) {
		tc_printf(t, "Erase failed due to a protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	if (status & SAMD_STATUSA_FAIL) {
		tc_printf(t, "Erase failed.\n");
		return true;
	}

	tc_printf(t, "samd erase successful!\n");

	return true;
}

/**
 * Sets the NVM region lock bits in the User Row. This value is read
 * at startup as the default value for the lock bits, and hence does
 * not take effect until a reset.
 *
 * 0x0000 = Lock, 0xFFFF = Unlock (default)
 */
static bool samd_set_flashlock(target *t, uint16_t value)
{
	uint32_t high = target_mem_read32(t, SAMD_NVM_USER_ROW_HIGH);
	uint32_t low = target_mem_read32(t, SAMD_NVM_USER_ROW_LOW);

	/* Write address of a word in the row to erase it */
	/* Must be shifted right for 16-bit address, see Datasheet §20.8.8 Address */
	target_mem_write32(t, SAMD_NVMC_ADDRESS, SAMD_NVM_USER_ROW_LOW >> 1);

	/* Issue the erase command */
	target_mem_write32(t, SAMD_NVMC_CTRLA,
	                   SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_ERASEAUXROW);

	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMD_NVMC_INT_STATUS) & SAMD_NVMC_READY) == 0)
		if (target_check_error(t))
			return -1;

	/* Modify the high byte of the user row */
	high = (high & 0x0000FFFF) | ((value << 16) & 0xFFFF0000);

	/* Write back */
	target_mem_write32(t, SAMD_NVM_USER_ROW_LOW, low);
	target_mem_write32(t, SAMD_NVM_USER_ROW_HIGH, high);

	/* Issue the page write command */
	target_mem_write32(t, SAMD_NVMC_CTRLA,
	                   SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_WRITEAUXPAGE);

	return true;
}

static bool samd_cmd_lock_flash(target *t)
{
	return samd_set_flashlock(t, 0x0000);
}

static bool samd_cmd_unlock_flash(target *t)
{
	return samd_set_flashlock(t, 0xFFFF);
}

static bool samd_cmd_read_userrow(target *t)
{
	tc_printf(t, "User Row: 0x%08x%08x\n",
		target_mem_read32(t, SAMD_NVM_USER_ROW_HIGH),
		target_mem_read32(t, SAMD_NVM_USER_ROW_LOW));

	return true;
}

/**
 * Reads the 128-bit serial number from the NVM
 */
static bool samd_cmd_serial(target *t)
{
	tc_printf(t, "Serial Number: 0x");

	for (uint32_t i = 0; i < 4; i++) {
		tc_printf(t, "%08x", target_mem_read32(t, SAMD_NVM_SERIAL(i)));
	}

	tc_printf(t, "\n");

	return true;
}

static bool samd5_cmd_lock_flash(target *t)
{
	UNUSED(t);
	return false;
}

static bool samd5_cmd_unlock_flash(target *t)
{
	UNUSED(t);
	return false;
}

/**
 * Reads the 128-bit serial number from the NVM
 */
static bool samd5_cmd_serial(target *t)
{
	tc_printf(t, "Serial Number: 0x");

	for (uint32_t i = 0; i < 4; i++) {
		tc_printf(t, "%08x", target_mem_read32(t, SAMD5_NVM_SERIAL(i)));
	}

	tc_printf(t, "\n");

	return true;
}

/**
 * Returns the size (in bytes) of the current SAM D20's flash memory.
 */
static uint32_t samd_flash_size(target *t)
{
	/* Read the Device ID */
	uint32_t did = target_mem_read32(t, SAMD_DSU_DID);

	/* Mask off the device select bits */
	uint8_t devsel = did & SAMD_DID_DEVSEL_MASK;

	/* Shift the maximum flash size (256KB) down as appropriate */
	return (0x40000 >> (devsel % 5));
}

/**
 * Runs the Memory Built In Self Test (MBIST)
 */
static bool samd_cmd_mbist(target *t)
{
	/* Write the memory parameters to the DSU */
	target_mem_write32(t, SAMD_DSU_ADDRESS, 0);
	target_mem_write32(t, SAMD_DSU_LENGTH, samd_flash_size(t));

	/* Clear the fail bit */
	target_mem_write32(t, SAMD_DSU_CTRLSTAT, SAMD_STATUSA_FAIL);

	/* Write the MBIST command */
	target_mem_write32(t, SAMD_DSU_CTRLSTAT, SAMD_CTRL_MBIST);

	/* Poll for DSU Ready */
	uint32_t status;
	while (((status = target_mem_read32(t, SAMD_DSU_CTRLSTAT)) &
		(SAMD_STATUSA_DONE | SAMD_STATUSA_PERR | SAMD_STATUSA_FAIL)) == 0)
		if (target_check_error(t))
			return false;

	/* Test the protection error bit in Status A */
	if (status & SAMD_STATUSA_PERR) {
		tc_printf(t, "MBIST not run due to protection error.\n");
		return true;
	}

	/* Test the fail bit in Status A */
	if (status & SAMD_STATUSA_FAIL) {
		tc_printf(t, "MBIST Fail @ 0x%08x\n",
		          target_mem_read32(t, SAMD_DSU_ADDRESS));
	} else {
		tc_printf(t, "MBIST Passed!\n");
	}

	return true;
}
/**
 * Sets the security bit
 */
static bool samd_cmd_ssb(target *t)
{
	/* Issue the ssb command */
	target_mem_write32(t, SAMD_NVMC_CTRLA,
	                   SAMD_CTRLA_CMD_KEY | SAMD_CTRLA_CMD_SSB);

	/* Poll for NVM Ready */
	while ((target_mem_read32(t, SAMD_NVMC_INT_STATUS) & SAMD_NVMC_READY) == 0)
		if (target_check_error(t))
			return -1;

	tc_printf(t, "Set the security bit! "
		  "You will need to issue 'monitor erase_mass' to clear this.\n");

	return true;
}
