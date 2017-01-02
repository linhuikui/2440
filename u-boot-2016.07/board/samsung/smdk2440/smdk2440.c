/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002, 2010
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <netdev.h>
#include <asm/io.h>
#include <asm/arch/s3c24x0_cpu.h>

DECLARE_GLOBAL_DATA_PTR;

/* Fout = 400MHz */
#define M_MDIV	0x5C
#define M_PDIV	0x01
#define M_SDIV	0x01

#define U_M_MDIV	0x38
#define U_M_PDIV	0x02
#define U_M_SDIV	0x02

static inline void pll_delay(unsigned long loops)
{
	__asm__ volatile ("1:\n"
	  "subs %0, %1, #1\n"
	  "bne 1b" : "=r" (loops) : "0" (loops));
}

/*
 * Miscellaneous platform dependent initialisations
 */

int board_early_init_f(void)
{
	struct s3c24x0_clock_power * const clk_power =
					s3c24x0_get_base_clock_power();
	struct s3c24x0_gpio * const gpio = s3c24x0_get_base_gpio();

    writel(0x05, &clk_power->clkdivn);

	/* to reduce PLL lock time, adjust the LOCKTIME register */
	writel(0xFFFFFF, &clk_power->locktime);

	/* configure MPLL */
	writel((M_MDIV << 12) + (M_PDIV << 4) + M_SDIV,
	       &clk_power->mpllcon);

	/* some delay between MPLL and UPLL */
	pll_delay(4000);

	/* configure UPLL */
	writel((U_M_MDIV << 12) + (U_M_PDIV << 4) + U_M_SDIV,
	       &clk_power->upllcon);

	/* some delay between MPLL and UPLL */
	pll_delay(8000);

	/* set up the I/O ports */
	writel(0x007FFFFF, &gpio->gpacon);
	writel(0x00044555, &gpio->gpbcon);
	writel(0x000007FF, &gpio->gpbup);
	writel(0xAAAAAAAA, &gpio->gpccon);
	writel(0x0000FFFF, &gpio->gpcup);
	writel(0xAAAAAAAA, &gpio->gpdcon);
	writel(0x0000FFFF, &gpio->gpdup);
	writel(0xAAAAAAAA, &gpio->gpecon);
	writel(0x0000FFFF, &gpio->gpeup);
	writel(0x000055AA, &gpio->gpfcon);
	writel(0x000000FF, &gpio->gpfup);
	writel(0xFF95FFBA, &gpio->gpgcon);
	writel(0x0000FFFF, &gpio->gpgup);
	writel(0x002AFAAA, &gpio->gphcon);
	writel(0x000007FF, &gpio->gphup);

#ifdef CONFIG_DRIVER_DM9000
    unsigned int oldval_gpfcon = *(volatile unsigned int *)&gpio->gpfcon;
    unsigned int oldval_gpfup = *(volatile unsigned int *)&gpio->gpfup;
    unsigned int oldval_extint0 = *(volatile unsigned int *)&gpio->extint0;
    unsigned int oldval_eintmask = *(volatile unsigned int *)&gpio->eintmask;
    unsigned int oldval_bwscon = *(volatile unsigned int *)0x48000000;

     /* Set GPF7 as EINT7 */
    *((volatile unsigned int *)&gpio->gpfcon) = oldval_gpfcon & (~(3 << 14)) | (2 << 14);
    *((volatile unsigned int *)&gpio->gpfup) = oldval_gpfup | (1 << 7);
    /* EINT7 High level interrupt */
    *((volatile unsigned int *)&gpio->extint0) = (oldval_extint0 & (~(0x7 << 28))) | (0x1 << 28);
    /* Enable EINT7 */
    *((volatile unsigned int *)&gpio->eintmask) = oldval_eintmask & (~(1<<7));
    /* Set GPA15 as nGCS4 */
    *((volatile unsigned int *)&gpio->gpacon) |= 1 << 15;
    /* DM9000 width 16, wait enable */
    *((volatile unsigned int *)0x48000000) = oldval_bwscon & (~(0x7<<16)) | (0x5<<16);
    *((volatile unsigned int *)0x48000014) = (1<<13) | (1<<11) | (0x6<<8) | (1<<6) | (1<<4) | (0<<2) | (0);
#endif

	return 0;
}

int board_init(void)
{
	/* arch number of SMDK2410-Board */
	gd->bd->bi_arch_number = MACH_TYPE_S3C2440;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = 0x30000100;

	icache_enable();
	dcache_enable();

	return 0;
}

int dram_init(void)
{
	/* dram_init must store complete ramsize in gd->ram_size */
	gd->ram_size = PHYS_SDRAM_1_SIZE;
	return 0;
}

#ifdef CONFIG_CMD_NET
int board_eth_init(bd_t *bis)
{
	int rc = 0;
#ifdef CONFIG_DRIVER_DM9000
    rc = dm9000_initialize(bis);
#endif
	return rc;
}
#endif

/*
 * Hardcoded flash setup:
 * Flash 0 is a non-CFI AMD AM29LV800BB flash.
 */
ulong board_flash_get_legacy(ulong base, int banknum, flash_info_t *info)
{
	info->portwidth = FLASH_CFI_16BIT;
	info->chipwidth = FLASH_CFI_BY16;
	info->interface = FLASH_CFI_X16;
	return 1;
}
