/*
 * (C) Copyright 2007
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * Configuation settings for the LPC313X board from Embedded Artists.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define USE_NAND
#define CONFIG_ENV_IS_NOWHERE          1

/*
 * If we are developing, we might want to start u-boot from ram
 * so we MUST NOT initialize critical regs like mem-timing ...
 */
#undef CONFIG_SKIP_LOWLEVEL_INIT
#undef CONFIG_USE_IRQ			/* don't need them anymore */
#undef CONFIG_CMD_FLASH
#undef CFG_CLKS_IN_HZ			/* everything, incl board info, in Hz */

/*
 * SoC Configuration
 */
#define CONFIG_LPC313X		1		/* This is a ARM926EJS CPU */
#define CONFIG_ARM_THUMB	1		/* Use Thumb instruction set */
/*#define CONFIG_PLL_270		1		* Use 270MHz Clk Freq */
#define XTAL_IN			12000000	/* Crystal clk freq */
#define CONFIG_SYS_HZ		(XTAL_IN / 256)	/* decrementer freq in Hz */
#define CONFIG_SYS_NO_FLASH			/* No NOR flash */

/* LPC313x U-boot bootmode support.
 * The following defines effect which low-level boot mode
 * is supported to boot u-boot images.
 */
#if 0
#define CONFIG_CPU_NAND_BOOT
#define CONFIG_CPU_SPI_BOOT
#define CONFIG_CPU_USBDFU_BOOT
#endif
#define CONFIG_CPU_MMC_BOOT
#if 0
#define CONFIG_CPU_NOR_BOOT
#define CONFIG_CPU_UART_BOOT
#endif

#define CONFIG_LPC313x_BOOT_HEADER

/*
 * Memory Info
 */
#define CONFIG_SYS_TEXT_BASE		0x31200000


#define CONFIG_NR_DRAM_BANKS	1		/* we have 1 bank of DRAM */
#define PHYS_SDRAM		0x30000000	/* SDRAM Bank #1 */
#define PHYS_SDRAM_SIZE                0x02000000      /* 32 MB SDRAM */

#define CONFIG_SYS_SDRAM_BASE	PHYS_SDRAM
#define CONFIG_SYS_SDRAM_SIZE	PHYS_SDRAM_SIZE

#define CONFIG_SYS_INIT_RAM_ADDR	0x11029000
#define CONFIG_SYS_INIT_RAM_SIZE	0x00018000
#define CONFIG_SYS_GBL_DATA_OFFSET	(CONFIG_SYS_INIT_RAM_SIZE - \
						GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR	(CONFIG_SYS_INIT_RAM_ADDR + \
						CONFIG_SYS_GBL_DATA_OFFSET)

/* Defines for SPL */
#define CONFIG_SPL

#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_LIBCOMMON_SUPPORT

#if 0
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_LIBGENERIC_SUPPORT
#define CONFIG_SPL_LIBDISK_SUPPORT
#define CONFIG_SPL_NAND_SUPPORT
#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#endif
#define CONFIG_SPL_MMC_SUPPORT
#if 0
#define CONFIG_SPL_SERIAL_SUPPORT
#define CONFIG_SPL_USB_GADGET_SUPPORT
#endif
#define CONFIG_SPL_LDSCRIPT		"$(CPUDIR)/lpc313x/u-boot-spl.lds"

#define CONFIG_SPL_TEXT_BASE		0x11029000	/* CONFIG_SYS_INIT_RAM_ADDR */
#define CONFIG_SPL_MAX_SIZE		0x00014000	/* CONFIG_SYS_INIT_RAM_SIZE */
#define CONFIG_SPL_STACK		0x1103B000	/* CONFIG_SYS_INIT_SP_ADDR */

#define CONFIG_SPL_BSS_START_ADDR	0x30000000	/* PHYS_SDRAM */
#define CONFIG_SPL_BSS_MAX_SIZE		0x80000

#define CONFIG_BOARD_LATE_INIT

/*
 * U-Boot memory configuration
 */
#define CONFIG_STACKSIZE		(32 * 1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ		(4*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ		(4*1024)	/* FIQ stack */
#endif
#define CONFIG_SYS_MALLOC_LEN		(1 * 1024 * 1024)		/* No need for any malloc space, we steal from the end of the DRAM */
#define CONFIG_SYS_GBL_DATA_SIZE	128		/* size in bytes reserved for initial data */
#define CONFIG_SYS_MEMTEST_START	0x30000000	/* memtest works on */
#define CONFIG_SYS_MEMTEST_END		0x33FFFFFF	/* 64MB of DRAM */

/* 
 * Ethernet Driver 7 Network Configuratin.
 * */
#define CONFIG_CMD_NET
#define CONFIG_DRIVER_DM9000		1
#define CONFIG_DM9000_BASE		0x20020000
#define DM9000_IO			CONFIG_DM9000_BASE
#define DM9000_DATA			(CONFIG_DM9000_BASE + 0x10000)
#define CONFIG_NET_MULTI
#define CONFIG_CMD_DHCP			1
#define CONFIG_CMD_PING			1
#define CONFIG_NET_RETRY_COUNT		100
/*
 * Uncomment this below line if you want static IP
 * support. Don't forget to change "net_boot" command.
 * In "net_boot" command, use tftpboot instead of dhcp.
 */
#if 0
#define CONFIG_IPADDR			192.168.1.60
#define CONFIG_SERVERIP			192.168.1.45
#define CONFIG_GATEWAYIP		192.168.1.1
#define CONFIG_NETMASK			255.255.255.0
#endif

/* Linux boot using network */
#define CONFIG_BOOTCOMMAND		"run sdmmc_boot"

/*
 * Serial Driver Console
 */
#define CONFIG_SERIAL1			1	/* we use Serial line 1 */
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*
 * U-boot Boot configuration
 */
#define CONFIG_BOOTFILE			"uImage"
#define CONFIG_BOOTDELAY		1
#define CONFIG_EXTRA_ENV_SETTINGS	\
"loadaddr=0x31000000\0" \
"rd_addr=0x32000000\0" \
"usbtty=cdc_acm\0" \
"ramargs=setenv bootargs console=ttyS0,115200n8 root=/dev/mmcblk0p3 rootwait init=/etc/preinit rw loglevel=7\0" \
"serverip=192.168.1.48\0" \
"nfsargs=setenv bootargs console=ttyS0,115200n8 root=/dev/nfs init=/etc/preinit rw nfsroot=${serverip}:${rootpath} ip=dhcp loglevel=7\0" \
"rootpath=/tftpboot/arm\0" \
"bootfile=uImage\0" \
"ramfile=rootfs.ext2.gz.uboot\0" \
"net_boot=dhcp; run nfsargs; bootm $(loadaddr)\0" \
"spi_boot= sf probe 0 0 0; sf read $(loadaddr) 0x42000 0x200000; run nfsargs; bootm $(loadaddr)\0" \
"nand_boot= nand read $(loadaddr) 0x80000 0x200000; run ramargs; bootm $(loadaddr)\0" \
"sdmmc_boot= mmc init; fatload mmc 0 $(loadaddr) $(bootfile); run ramargs; bootm $(loadaddr)\0" \
"usbdfu_boot= usbpoll $(loadaddr); run nfsargs; bootm $(loadaddr)\0" \
"usb_boot= usb start; fatload usb 0 $(loadaddr) $(bootfile); run nfsargs; bootm $(loadaddr)\0" \
"uart_boot= loady; run nfsargs; bootm $(loadaddr)\0" \
"netram_boot=dhcp; tftp $(rd_addr) $(ramfile); run ramargs; bootm $(loadaddr) $(rd_addr)\0" \
"spiram_boot= sf probe 0 0 0; sf read $(loadaddr) 0x42000 0x200000; sf read $(rd_addr) 0x243000 0x400000; run ramargs; bootm $(loadaddr) $(rd_addr)\0" \
"nandram_boot= nand read $(loadaddr) 0x80000 0x200000; nand read $(rd_addr) 0x2a0000 0x400000; run ramargs; bootm $(loadaddr) $(rd_addr)\0" \
"sdmmcram_boot= mmc init; fatload mmc 0 $(loadaddr) $(bootfile); fatload mmc 0 $(rd_addr) $(ramfile); run ramargs; bootm $(loadaddr) $(rd_addr)\0" \
"usbdfuram_boot= usbpoll $(loadaddr); usbpoll $(rd_addr); run ramargs; bootm $(loadaddr) $(rd_addr)\0" \
"usbram_boot= usb start; fatload usb 0 $(loadaddr) $(bootfile); fatload usb 0 $(rd_addr) $(ramfile); run ramargs; bootm $(loadaddr) $(rd_addr)\0" \
"uartram_boot= loady; loady $(rd_addr); run ramargs; bootm $(loadaddr) $(rd_addr)\0" \
""

/*
 * Linux interfacing
 */
#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_SYS_MAXARGS		16			/* max number of command args	*/
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE	/* Boot Argument Buffer Size	*/

/*
 * U-boot geneal configurable
 */
#define CONFIG_SYS_LONGHELP				/* undef to save memory */
#define CONFIG_SYS_PROMPT		"Redwire Egg # "	/* Monitor Command Prompt */

#define CONFIG_SYS_CBSIZE		256		/* Console I/O Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) 
#define CONFIG_SYS_LOAD_ADDR		11029000
#define CONFIG_ENV_OVERWRITE		/* allow to overwrite serial and ethaddr */
#define CONFIG_AUTO_COMPLETE
#define CONFIG_CMDLINE_EDITING
#define CONFIG_PANIC_HANG

#include <config_cmd_default.h>
/*
 * U-boot Generic Command line configuration.
 */
#define CONFIG_CMD_AUTOSCRIPT	/* Autoscript Support */
#define CONFIG_CMD_BDI		/* bdinfo */
#define CONFIG_CMD_BOOTD	/* bootd */
#define CONFIG_CMD_CONSOLE	/* coninfo */
#define CONFIG_CMD_ECHO		/* echo arguments */
/*#define CONFIG_CMD_SAVEENV */        /* saveenv */ 
#define CONFIG_CMD_IMI		/* iminfo */
#define CONFIG_CMD_MISC		/* Misc functions like sleep etc*/
#define CONFIG_CMD_RUN		/* run command in env variable */
#define CONFIG_CMD_SETGETDCR	/* DCR support on 4xx */
#define CONFIG_CMD_XIMG		/* Load part of Multi Image */
#define CONFIG_CMD_MEMORY	/* Use for Memory*/
#define CONFIG_CMD_LOADB
#define CONFIG_MISC_INIT_R
/*
 * Flash (NAND,SPINOR) configuration
 */
/* Micron NAND Flash support */
#define CONFIG_CMD_NAND                        0
#define CONFIG_NAND_LPC313X            0
#define CONFIG_ENV_IS_IN_NAND  0
#define LPC313X_NAND_LP_SUPPORT                0
#define CONFIG_SYS_NAND_BASE		0x00000000
#define CONFIG_SYS_MAX_NAND_DEVICE     0               /* Max number of NAND devices */
#define NAND_MAX_CHIPS                 0

/* ATMEL SPI based NOR Flash support */
#define CONFIG_HARD_SPI                        0
#define CONFIG_LPC313X_SPI             0
#define CONFIG_SPI_FLASH               0
#define CONFIG_SPI_FLASH_ATMEL         0
#define CONFIG_CMD_SF                  0
#define CONFIG_ENV_IS_IN_SPI_FLASH     0

#ifdef CONFIG_ENV_IS_IN_NAND
#ifdef LPC313X_NAND_LP_SUPPORT
#define CONFIG_ENV_OFFSET		0x60000		/* Addr of Environment Sector */
#define CONFIG_ENV_SIZE			0x20000		/* Total Size of Environment Sector */
#else
#define CONFIG_ENV_OFFSET		0x20000		/* Addr of Environment Sector */
#define CONFIG_ENV_SIZE			0x4000		/* Total Size of Environment Sector */
#endif
#else

#ifdef CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SIZE			0x1000		/* Total Size of Environment Sector */
#define CONFIG_ENV_SECT_SIZE		0x10000
#define CONFIG_ENV_OFFSET		0x40000

#else
#define CONFIG_ENV_OFFSET		0x00		/* Addr of Environment Sector */
#define CONFIG_ENV_SIZE			0x800		/* Total Size of Environment Sector */
#define CONFIG_ENV_IS_NOWHERE		1

#endif
#endif

#define CONFIG_SYS_I2C_SPEED		100000

#define CONFIG_OF_LIBFDT

/* Uncomment if you want USB host support and
 * disable USB gadget supoprt. Please note if
 * you disable USB gadget support, USB DFU boot
 * mode will not work.
 */
/*
 * USB configuration as EHCI HOST
 */

#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI			/* Enable EHCI USB support */
#define CONFIG_USB_EHCI_LPC313X		/* on LPC313X platform */
#define CONFIG_EHCI_IS_TDI
#define CONFIG_USB_STORAGE
#define CONFIG_SUPPORT_VFAT
/*#undef CONFIG_CPU_USBDFU_BOOT*/


/*
 * USB configuration as GADGET
 */
#define CONFIG_USB_DEVICE		1
#define CONFIG_USB_TTY			1
#define CONFIG_SYS_CONSOLE_IS_IN_ENV	1
#define CONFIG_USBD_VENDORID		0x1457	/* Linux/NetChip */
#define CONFIG_USBD_PRODUCTID_CDCACM	0x5119	/* CDC ACM */
#define CONFIG_USBD_MANUFACTURER	"OpenMoko, Inc"
#define CONFIG_USBD_PRODUCT_NAME	"LPC313x Bootloader " U_BOOT_VERSION
#define CONFIG_USB_DFU			1
#define CONFIG_USBD_DFU_XFER_SIZE	4096	/* 0x1000 */
#define CONFIG_USBD_DFU_INTERFACE	2

/*
 * MMC Driver configuration
 */
#define CONFIG_CMD_MMC			1
#define CONFIG_MMC			1

#define CONFIG_CMD_FAT			1
#define CONFIG_DOS_PARTITION
#define CONFIG_ISO_PARTITION 



#endif /* __CONFIG_H */
