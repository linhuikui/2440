/* linux/arch/arm/mach-s3c2440/mach-smdk2440.c
 *
 * Copyright (c) 2008 Ramax Lo <ramaxlo@gmail.com>
 *      Based on mach-anubis.c by Ben Dooks <ben@simtec.co.uk>
 *      and modifications by SBZ <sbz@spgui.org> and
 *      Weibing <http://weibing.blogbus.com> and
 *      Michel Pollet <buserror@gmail.com>
 *
 * For product information, visit http://code.google.com/p/smdk2440/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/serial_core.h>
#include <linux/serial_s3c.h>
#include <linux/dm9000.h>
#include <linux/platform_data/at24.h>
#include <linux/platform_device.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/mmc/host.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/hardware.h>
#include <mach/fb.h>
#include <asm/mach-types.h>

#include <mach/regs-gpio.h>
#include <linux/platform_data/leds-s3c24xx.h>
#include <mach/regs-lcd.h>
#include <mach/irqs.h>
#include <mach/gpio-samsung.h>
#include <linux/platform_data/mtd-nand-s3c2410.h>
#include <linux/platform_data/i2c-s3c2410.h>
#include <linux/platform_data/mmc-s3cmci.h>
#include <linux/platform_data/usb-s3c2410_udc.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/samsung-time.h>

#include <sound/s3c24xx_uda134x.h>

#include "common.h"

#define MACH_SMDK2440_DM9K_BASE (S3C2410_CS4 + 0x300)

static struct map_desc smdk2440_iodesc[] __initdata = {
	/* nothing to declare, move along */
};

#define UCON S3C2410_UCON_DEFAULT
#define ULCON S3C2410_LCON_CS8 | S3C2410_LCON_PNONE | S3C2410_LCON_STOPB
#define UFCON S3C2410_UFCON_RXTRIG8 | S3C2410_UFCON_FIFOMODE

static struct s3c2410_uartcfg smdk2440_uartcfgs[] __initdata = {
	[0] = {
		.hwport	     = 0,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[1] = {
		.hwport	     = 1,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
	[2] = {
		.hwport	     = 2,
		.flags	     = 0,
		.ucon	     = UCON,
		.ulcon	     = ULCON,
		.ufcon	     = UFCON,
	},
};

/* USB device UDC support */
static struct s3c2410_udc_mach_info smdk2440_udc_cfg __initdata = {
	.pullup_pin = S3C2410_GPC(5),
};

/* LCD driver info  480x272 */
static struct s3c2410fb_display smdk2440_lcd_cfg __initdata = {

	.lcdcon5	= S3C2410_LCDCON5_FRM565 |
			  S3C2410_LCDCON5_INVVLINE |
			  S3C2410_LCDCON5_INVVFRAME |
			  S3C2410_LCDCON5_PWREN |
			  S3C2410_LCDCON5_HWSWP,

	.type		= S3C2410_LCDCON1_TFT,

	.width		= 480,
	.height		= 272,

	.pixclock	= 166667, /* HCLK 60 MHz, divisor 10 */
	.xres		= 480,
	.yres		= 272,
	.bpp		= 16,
	.left_margin	= 20,
	.right_margin	= 8,
	.hsync_len	= 4,
	.upper_margin	= 8,
	.lower_margin	= 7,
	.vsync_len	= 4,
};
#if 0
static struct s3c2410fb_mach_info smdk2440_fb_info __initdata = {
	.displays	= &smdk2440_lcd_cfg,
	.num_displays	= 1,
	.default_display = 0,

	.gpccon		= 0xaaaaaaaa,
	.gpccon_mask	= 0xffffffff,
	.gpcup		= 0xffffffff,
	.gpcup_mask	= 0xffffffff,
	
	.gpdcon		= 0xaaaaaaaa,
	.gpdcon_mask	= 0xffffffff,
	.gpdup		= 0xffffffff,
	.gpdup_mask	= 0xffffffff,

	//.lpcsel		= ((0xCE6) & ~7) | 1<<4,
};
#else
#define S3C2410_GPCCON_MASK(x)	(3 << ((x) * 2))
#define S3C2410_GPDCON_MASK(x)	(3 << ((x) * 2))

static struct s3c2410fb_mach_info smdk2440_fb_info __initdata = {
	.displays	 = &smdk2440_lcd_cfg, /* not constant! see init */
	.num_displays	 = 1,
	.default_display = 0,

	/* Enable VD[2..7], VD[10..15], VD[18..23] and VCLK, syncs, VDEN
	 * and disable the pull down resistors on pins we are using for LCD
	 * data. */

	.gpcup		= (0xf << 1) | (0x3f << 10),

	.gpccon		= (S3C2410_GPC1_VCLK   | S3C2410_GPC2_VLINE |
			   S3C2410_GPC3_VFRAME | S3C2410_GPC4_VM |
			   S3C2410_GPC10_VD2   | S3C2410_GPC11_VD3 |
			   S3C2410_GPC12_VD4   | S3C2410_GPC13_VD5 |
			   S3C2410_GPC14_VD6   | S3C2410_GPC15_VD7),

	.gpccon_mask	= (S3C2410_GPCCON_MASK(1)  | S3C2410_GPCCON_MASK(2)  |
			   S3C2410_GPCCON_MASK(3)  | S3C2410_GPCCON_MASK(4)  |
			   S3C2410_GPCCON_MASK(10) | S3C2410_GPCCON_MASK(11) |
			   S3C2410_GPCCON_MASK(12) | S3C2410_GPCCON_MASK(13) |
			   S3C2410_GPCCON_MASK(14) | S3C2410_GPCCON_MASK(15)),

	.gpdup		= (0x3f << 2) | (0x3f << 10),

	.gpdcon		= (S3C2410_GPD2_VD10  | S3C2410_GPD3_VD11 |
			   S3C2410_GPD4_VD12  | S3C2410_GPD5_VD13 |
			   S3C2410_GPD6_VD14  | S3C2410_GPD7_VD15 |
			   S3C2410_GPD10_VD18 | S3C2410_GPD11_VD19 |
			   S3C2410_GPD12_VD20 | S3C2410_GPD13_VD21 |
			   S3C2410_GPD14_VD22 | S3C2410_GPD15_VD23),

	.gpdcon_mask	= (S3C2410_GPDCON_MASK(2)  | S3C2410_GPDCON_MASK(3) |
			   S3C2410_GPDCON_MASK(4)  | S3C2410_GPDCON_MASK(5) |
			   S3C2410_GPDCON_MASK(6)  | S3C2410_GPDCON_MASK(7) |
			   S3C2410_GPDCON_MASK(10) | S3C2410_GPDCON_MASK(11)|
			   S3C2410_GPDCON_MASK(12) | S3C2410_GPDCON_MASK(13)|
			   S3C2410_GPDCON_MASK(14) | S3C2410_GPDCON_MASK(15)),
};
#endif

/* MMC/SD  */
static struct s3c24xx_mci_pdata smdk2440_mmc_cfg __initdata = {
   .gpio_detect   = S3C2410_GPG(8),
   .gpio_wprotect = S3C2410_GPH(8),
   .set_power     = NULL,
   .ocr_avail     = MMC_VDD_32_33|MMC_VDD_33_34,
};

/* NAND Flash on SMDK2440 board */
static struct mtd_partition smdk2440_default_nand_part[] __initdata = {
	[0] = {
		.name	= "u-boot",
		.size	= SZ_256K,
		.offset	= 0,
	},
	[1] = {
		.name	= "u-boot-env",
		.size	= SZ_128K,
		.offset	= SZ_256K,
	},
	[2] = {
		.name	= "kernel",
		/* 5 megabytes, for a kernel with no modules
		 * or a uImage with a ramdisk attached */
		.size	= 0x00500000,
		.offset	= SZ_256K + SZ_128K,
	},
	[3] = {
		.name	= "root",
		.offset	= SZ_256K + SZ_128K + 0x00500000,
		.size	= MTDPART_SIZ_FULL,
	},
};

static struct s3c2410_nand_set smdk2440_nand_sets[] __initdata = {
	[0] = {
		.name		= "nand",
		.nr_chips	= 1,
		.nr_partitions	= ARRAY_SIZE(smdk2440_default_nand_part),
		.partitions	= smdk2440_default_nand_part,
		.flash_bbt 	= 1, /* we use u-boot to create a BBT */
	},
};

static struct s3c2410_platform_nand smdk2440_nand_info __initdata = {
	.tacls		= 0,
	.twrph0		= 25,
	.twrph1		= 15,
	.nr_sets	= ARRAY_SIZE(smdk2440_nand_sets),
	.sets		= smdk2440_nand_sets,
	.ignore_unset_ecc = 1,
};

/* DM9000AEP 10/100 ethernet controller */
static struct resource smdk2440_dm9k_resource[] = {
	[0] = DEFINE_RES_MEM(MACH_SMDK2440_DM9K_BASE, 4),
	[1] = DEFINE_RES_MEM(MACH_SMDK2440_DM9K_BASE + 4, 4),
	[2] = DEFINE_RES_NAMED(IRQ_EINT7, 1, NULL, IORESOURCE_IRQ \
						| IORESOURCE_IRQ_HIGHEDGE),
};

/*
 * The DM9000 has no eeprom, and it's MAC address is set by
 * the bootloader before starting the kernel.
 */
static struct dm9000_plat_data smdk2440_dm9k_pdata = {
	.flags		= (DM9000_PLATF_16BITONLY | DM9000_PLATF_NO_EEPROM),
};

static struct platform_device smdk2440_device_eth = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smdk2440_dm9k_resource),
	.resource	= smdk2440_dm9k_resource,
	.dev		= {
		.platform_data	= &smdk2440_dm9k_pdata,
	},
};

static struct gpio_keys_button smdk2440_buttons[] = {
	{
		.gpio		= S3C2410_GPF(0),		/* K1 EINT0*/
		.code		= KEY_F1,
		.desc		= "Button 1",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPF(2),		/* K2 EINT2*/
		.code		= KEY_F2,
		.desc		= "Button 2",
		.active_low	= 1,
	},
	{
		.gpio		= S3C2410_GPG(3),		/* K3 EINT11*/
		.code		= KEY_F3,
		.desc		= "Button 3",
		.active_low	= 1,
	},
#if 0
	/* this pin is also known as TCLK1 and seems to already
	 * marked as "in use" somehow in the kernel -- possibly wrongly */
	{
		.gpio		= S3C2410_GPG(11),	/* K4 EINT19*/
		.code		= KEY_F4,
		.desc		= "Button 4",
		.active_low	= 1,
	},
#endif
};

static struct gpio_keys_platform_data smdk2440_button_data = {
	.buttons	= smdk2440_buttons,
	.nbuttons	= ARRAY_SIZE(smdk2440_buttons),
};

static struct platform_device smdk2440_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.dev		= {
		.platform_data	= &smdk2440_button_data,
	}
};

/* LEDS */
static struct s3c24xx_led_platdata smdk2440_led1_pdata = {
	.name		= "led1",
	.gpio		= S3C2410_GPF(4),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "D10",
};

static struct s3c24xx_led_platdata smdk2440_led2_pdata = {
	.name		= "led2",
	.gpio		= S3C2410_GPF(5),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "D11",
};

static struct s3c24xx_led_platdata smdk2440_led3_pdata = {
	.name		= "led3",
	.gpio		= S3C2410_GPF(6),
	.flags		= S3C24XX_LEDF_ACTLOW | S3C24XX_LEDF_TRISTATE,
	.def_trigger	= "D12",
};

static struct s3c24xx_led_platdata smdk2440_led_backlight_pdata = {
	.name		= "backlight",
	.gpio		= S3C2410_GPB(0),
	.def_trigger	= "backlight",
};

static struct platform_device smdk2440_led1 = {
	.name		= "led",
	.id		= 1,
	.dev		= {
		.platform_data	= &smdk2440_led1_pdata,
	},
};

static struct platform_device smdk2440_led2 = {
	.name		= "led",
	.id		= 2,
	.dev		= {
		.platform_data	= &smdk2440_led2_pdata,
	},
};

static struct platform_device smdk2440_led3 = {
	.name		= "led",
	.id		= 3,
	.dev		= {
		.platform_data	= &smdk2440_led3_pdata,
	},
};

static struct platform_device smdk2440_led_backlight = {
	.name		= "backlight",
	.id		= 4,
	.dev		= {
		.platform_data	= &smdk2440_led_backlight_pdata,
	},
};

/* AUDIO */
static struct s3c24xx_uda134x_platform_data smdk2440_audio_pins = {
	.l3_clk = S3C2410_GPB(4),
	.l3_mode = S3C2410_GPB(2),
	.l3_data = S3C2410_GPB(3),
	.model = UDA134X_UDA1341
};

static struct platform_device smdk2440_audio = {
	.name		= "s3c24xx_wm8976",
	.id		= 0,
	.dev		= {
		.platform_data	= &smdk2440_audio_pins,
	},
};

static struct platform_device *smdk2440_devices[] __initdata = {
	&s3c_device_ohci,
	&s3c_device_wdt,
	&s3c_device_i2c0,
	&s3c_device_rtc,
	&s3c_device_usbgadget,
	&smdk2440_device_eth,
	&smdk2440_led1,
	&smdk2440_led2,
	&smdk2440_led3,
	&smdk2440_led_backlight,
	&smdk2440_button_device,
	&s3c_device_nand,
	&s3c_device_sdi,
	&s3c2440_device_dma,
	&s3c_device_iis,
	&s3c_device_lcd,
	/*&smdk2440_audio,*/
};

static void __init smdk2440_map_io(void)
{
	s3c24xx_init_io(smdk2440_iodesc, ARRAY_SIZE(smdk2440_iodesc));
	s3c24xx_init_uarts(smdk2440_uartcfgs, ARRAY_SIZE(smdk2440_uartcfgs));
	samsung_set_timer_source(SAMSUNG_PWM3, SAMSUNG_PWM4);
}

static void __init smdk2440_init_time(void)
{
	s3c2440_init_clocks(12000000);
	samsung_timer_init();
}

static void __init smdk2440_init(void)
{
	int i;

	printk(KERN_INFO "SMDK2440: initing smdk2440\n");

	/* turn LCD on */
	s3c_gpio_cfgpin(S3C2410_GPC(0), S3C2410_GPC0_LEND);

	/* Turn the backlight early on */
	WARN_ON(gpio_request_one(S3C2410_GPB(0), GPIOF_OUT_INIT_HIGH, NULL));
	gpio_free(S3C2410_GPB(0));

	/* remove pullup on optional PWM backlight -- unused on 3.5 and 7"s */
	gpio_request_one(S3C2410_GPB(1), GPIOF_IN, NULL);
	s3c_gpio_setpull(S3C2410_GPB(1), S3C_GPIO_PULL_UP);
	gpio_free(S3C2410_GPB(1));

	/* mark the key as input, without pullups (there is one on the board) */
	for (i = 0; i < ARRAY_SIZE(smdk2440_buttons); i++) {
		s3c_gpio_setpull(smdk2440_buttons[i].gpio, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(smdk2440_buttons[i].gpio, S3C2410_GPIO_INPUT);
	}

	s3c24xx_fb_set_platdata(&smdk2440_fb_info);

	s3c24xx_udc_set_platdata(&smdk2440_udc_cfg);
	s3c24xx_mci_set_platdata(&smdk2440_mmc_cfg);
	s3c_nand_set_platdata(&smdk2440_nand_info);
	s3c_i2c0_set_platdata(NULL);

	platform_add_devices(smdk2440_devices, ARRAY_SIZE(smdk2440_devices));
}


MACHINE_START(S3C2440, "SMDK2440")
	/* Maintainer: Michel Pollet <buserror@gmail.com> */
	.atag_offset	= 0x100,
	.map_io		= smdk2440_map_io,
	.init_machine	= smdk2440_init,
	.init_irq	= s3c2440_init_irq,
	.init_time	= smdk2440_init_time,
MACHINE_END

