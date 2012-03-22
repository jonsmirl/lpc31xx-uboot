/*
 * (C) Copyright 2010 NXP Semiconductors
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

#include <common.h>
#include <nand.h>
#include <spi_flash.h>
#include <mmc.h>
#include <malloc.h>
#include <image.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>
#include <asm/arch/sysreg.h>
#include <asm/arch/ioconf.h>
#include <asm/arch/mci.h>
#include <asm/arch/timer.h>
#include <asm/arch/i2c.h>
#include <asm/arch/common.h>
#include "dos_partition.h"

typedef enum
{
	LPC313X_BOOT_MODE_NAND = 0,
	LPC313X_BOOT_MODE_SPI,
	LPC313X_BOOT_MODE_DFU_USB,
	LPC313X_BOOT_MODE_SD_MMC,
	LPC313X_BOOT_MODE_RESERVED,
	LPC313X_BOOT_MODE_NOR_FLASH,
	LPC313X_BOOT_MODE_UART,
	LPC313X_BOOT_MODE_TEST,
} LPC313X_BOOT_MODE_T;

/* LPC313x macros */
#define LPC313X_BOOTMODE_MASK		0x00000007
#define UART_DOWNLOAD_TIMEOUT_END_US	10000
#define UART_DOWNLOAD_TIMEOUT_START_US	10000000
#define READ_BLOCK_SIZE			(512)

#ifdef CONFIG_EA3152
#define AD_SLAVE_ADDR			(0x18)
#define AD_REG_OTGDCLIC_RW		0x0000
#define ANALOG_1V8_RAIL			2
/* 1v4 io voltage rail adjustments */
#define PSU_VOUT3_1_80			_BIT(16)
#endif

/* Memory macros */
#define LPC313X_ISRAM_BASE_ADD		0x11029000
#define LPC313X_SDRAM_TEMP_DATA_ADDR	0x32000000

/* SPI NOR flash macros */
#define LPC313X_SPI_NOR_UBOOT_SECTOR	0x0
#define LPC313X_SPI_NOR_SECTOR_SIZE	0x40000

/* SD/MMC macros */
#define LBA_STEP_MCI			32
/* based on LBA_STEP_MCI define we will search first 32MB of the partition only */
#define MAX_IMAGE_SEARCH_NUM_OF_BLOCKS	2048

/* USB macros */
#define EVT_usb_pll_bank		3
#define EVT_usb_atx_pll_lock_bit	_BIT(25)

//extern long __INITIAL_DATA_END;
extern long __initial_boot_image_size;

//int is_nand_init_done = 0;
//int is_spi_init_done = 0;

/* Define global data structure pointer to it*/
static gd_t gdata __attribute__ ((section(".data")));
static bd_t bdata __attribute__ ((section(".data")));

inline void hang(void)
{
	puts("### ERROR ### Please RESET the board ###\n");
	for (;;)
		;
}

void GPIOMuxSetup(void)
{
	gpio_set_outpin_high(IOCONF_GPIO, 2);
}

void board_init_f(ulong dummy)
{
	//GPIOMuxSetup();
	/*
	 * We call relocate_code() with relocation target same as the
	 * CONFIG_SYS_SPL_TEXT_BASE. This will result in relocation getting
	 * skipped. Instead, only .bss initialization will happen. That's
	 * all we need
	 */
	debug(">>board_init_f()\n");
	relocate_code(CONFIG_SYS_INIT_SP_ADDR, &gdata, CONFIG_SPL_TEXT_BASE);
}

#ifdef CONFIG_CPU_USBDFU_BOOT
extern unsigned char stop_polling;
#endif

DECLARE_GLOBAL_DATA_PTR;

typedef struct boot_header {
	int vector;
	int magic;
	long crc;
	unsigned char reserved[16];
	int imageType;
	int imageLength;
} boot_header_t;

#ifdef CONFIG_CPU_MMC_BOOT
int find_and_load_mci_image (volatile unsigned char * loadPointer,
		unsigned int startLBA, unsigned int sizeInLBA)
{
	unsigned int mci_block_index;
	unsigned int mci_block_limit;
	unsigned int image_length;
	boot_header_t *bhdr = (boot_header_t *)loadPointer;
	image_header_t *ihdr = (image_header_t *)loadPointer;

	mci_block_index = startLBA;
	mci_block_limit = startLBA;

	if (sizeInLBA > ((unsigned int) (MAX_IMAGE_SEARCH_NUM_OF_BLOCKS *
					LBA_STEP_MCI))) {
		mci_block_limit += ((unsigned int) (MAX_IMAGE_SEARCH_NUM_OF_BLOCKS *
					 LBA_STEP_MCI));
	}
	else {
		mci_block_limit += sizeInLBA;
	}

	while (mci_block_index < mci_block_limit) {
		/* Read the header */
		if (mci_read_blocks((int)mmc_get_dev(0), mci_block_index, 1, loadPointer) == 0) {
			/* This error while reading was not expected so return with failure */
			return (-1);
		}
		/* skip over the SPL boot image */
		if (bhdr->magic == 0x41676d69) {
			mci_block_index += bhdr->imageLength / READ_BLOCK_SIZE;
			continue;
		}
		if (!image_check_magic(ihdr)) {
			puts("Bad Magic Number\n");
			mci_block_index += (unsigned int) LBA_STEP_MCI;
			continue;
		}
		image_length = image_get_data_size(ihdr) + sizeof(*ihdr);

		/* Yes it makes sense, so try loading the rest of the image
		 * first test if the image fits the medium */
		if ((((image_length + READ_BLOCK_SIZE - 1)/READ_BLOCK_SIZE)
					+ mci_block_index) > (startLBA + sizeInLBA)) {
			mci_block_index += (unsigned int) LBA_STEP_MCI;
			continue;
		}
		/* Yes, the image fits the medium so load it */
		if (mci_read_blocks((int)mmc_get_dev(0),mci_block_index+1,
				((image_length + READ_BLOCK_SIZE - 1)/READ_BLOCK_SIZE),
				loadPointer + READ_BLOCK_SIZE) == 0)
			/* this error while reading was not expected so return with failure */
			return (-1);

		if (!image_check_hcrc(ihdr)) {
			puts("Bad Header Checksum\n");
			mci_block_index += (unsigned int) LBA_STEP_MCI;
			continue;
		}
		return 0;
	}

	return -1;
}
#endif

static int timer_expired(ulong timeStart, ulong timeDelta)
{
	ulong currTime = 0;

	currTime = get_timer(0);
	if (((currTime - timeStart) > timeDelta))
		return 1;

	return 0;
}

static int usb_copy_boot_image(ulong image_length, volatile unsigned char * loadPointer)
{
#ifdef CONFIG_CPU_USBDFU_BOOT
	u32 i = 500;

	while(!stop_polling) {
		usbtty_poll();
	}
	/* do usbtty_poll() for the DFU Class status request to complete */
	while(i) {
		usbtty_poll();
		i--;
	}
#else
	printf("Please enable USB Gadget supoprt before selecting USB DFU boot mode\n");
#endif
	return 0;
}

static int sdmmc_copy_boot_image(volatile unsigned char * loadPointer)
{
#ifdef CONFIG_CPU_MMC_BOOT
	unsigned int partition_number;
	partition_t partitionInfo;
	unsigned int max_num_sectors = mmc_get_card_size((int)mmc_get_dev(0));
	unsigned int ret = 0;

	/* Check if there is partition table on the device */
	if (0 == partition_table_probe()) {
		/* Yes there is, so try the partitions for the image. Search bootIt partition first
		 * so pass partition_number = 0 first. Usualy 0 is invalid partition number.
		 */
		for (partition_number = 0; partition_number <= MAX_PARTITION;
					 partition_number++) {
			if (0 == partition_get_info (&partitionInfo, partition_number)) {
				/* Check if the partition fits in the media (corrupt partition table check) */
				if ((partitionInfo.start_lba + partitionInfo.size_lba) <=
					max_num_sectors) {
					/* Found a valid partition, so try finding a image */
					ret = find_and_load_mci_image (loadPointer,
							partitionInfo.start_lba,
							partitionInfo.size_lba);
					if (ret == 0) {
						/* Good image found. */
						return 0;
					}
				}
			}
		}
	}
#else
	printf("Please enable MMC supoprt before selecting MMC boot mode\n");
#endif
	return 0;
}

static int flash_copy_boot_image(ulong image_length, volatile unsigned char * loadPointer)
{
	printf("Parallel flash boot mode is not supported\n");
	return 0;
}
static int nand_copy_boot_image(ulong image_length, volatile unsigned char * loadPointer)
{
#ifdef CONFIG_CPU_NAND_BOOT
	size_t len = 0;
	int ret = 0;

	len = image_length;
	printf("reading NAND..");
	ret = nand_read_skip_bad(&nand_info[0], 0x20000, &len,
			(u_char *)loadPointer);
	if(ret) {
		printf("Failed\n");
		return -1;
	}
	else {
		printf("Done\n");
	}
#else
	printf("Please enable NAND supoprt before selecting NAND boot mode\n");
#endif
	return 0;
}

static int spi_copy_boot_image(ulong image_length, volatile unsigned char * loadPointer)
{
#ifdef CONFIG_CPU_SPI_BOOT
	struct spi_flash *flash = NULL;
	int ret = 0;

	flash = spi_flash_probe(0, 0, 0, 0);
	if (!flash) {
		printf("Failed to initialize SPI flash\n");
		return 1;
	}

	printf("reading SPI NOR..");
	ret = spi_flash_read(flash,(LPC313X_SPI_NOR_UBOOT_SECTOR * LPC313X_SPI_NOR_SECTOR_SIZE),
			image_length, (void *)loadPointer);

	if (ret) {
		printf("Failed\n");
		return -1;
	}
	printf("Done\n");
#else
	printf("Please enable SPI supoprt before selecting SPI boot mode\n");
#endif
	return 0;
}

static int uart_copy_boot_image(volatile unsigned char *loadPointer)
{
	u32 byteCount = 0;
	ulong timeoutValue = 0;
	ulong timerexpire = 0;
	ulong timeStart = 0;
	image_header_t *hdr = (image_header_t *)loadPointer;
	uint32_t image_length = sizeof(image_header_t);

	/* Download the image until the maximal size is reached
	 * or a timeout on the byte reception happened.
	 */

	timeoutValue = UART_DOWNLOAD_TIMEOUT_START_US;
	printf("READY TO RECEIVE DATA IN BINARY MODE\n");


	while (1) {
		reset_timer();
		timeStart = get_timer(0);

		while (serial_getchar (loadPointer) == 0) {
			timerexpire = timer_expired(timeStart, timeoutValue);
			if(timerexpire == 1) {
				break;
			}
			if(byteCount >= image_length)
				break;
		}

		if(timerexpire == 1) {
			if (timeoutValue == UART_DOWNLOAD_TIMEOUT_END_US) {
				printf("Download finished\n" );
			}
			else {
				printf("Timeout!!!\n" );
			}
			break;
		}

		timeoutValue = UART_DOWNLOAD_TIMEOUT_END_US;

		loadPointer++;
		byteCount++;
		if(byteCount == sizeof(image_header_t)) {
			image_length += image_get_data_size(hdr);
		}
		if (byteCount >= image_length) {
			break;
		}
	}

	printf("%d bytes are transfered\n",byteCount);

	return 0;
}

#if defined(CONFIG_USB_EHCI_LPC313X) || defined(CONFIG_USB_DEVICE)
void usb_init_clocks(void)
{

	/* enable USB to AHB clock */
	cgu_clk_en_dis(CGU_SB_USB_OTG_AHB_CLK_ID, 1);
	/* enable clock to Event router */
	cgu_clk_en_dis(CGU_SB_EVENT_ROUTER_PCLK_ID, 1);

	/* reset USB block */
	cgu_soft_reset_module(USB_OTG_AHB_RST_N_SOFT);

	if(SYS_REGS->usb_atx_pll_pd_reg != 0) {
		/* enable USB OTG PLL */
		SYS_REGS->usb_atx_pll_pd_reg = 0x0;
		/* wait for PLL to lock */
		while (!(EVRT_RSR(EVT_usb_pll_bank) & EVT_usb_atx_pll_lock_bit));
	}

	return;
}
#endif

#ifdef CONFIG_EA3152
void psu_set_voltage(u32 rail, u32 volt)
{
	u32 reg_val;
	int bit_pos;

	/* read PSU register */
	i2c_read(I2C1_CTRL, AD_SLAVE_ADDR, AD_REG_OTGDCLIC_RW,
			2, &reg_val,4);

	/* check if this is to set VOUT3 rail */
	if (ANALOG_1V8_RAIL == rail) {
		reg_val |= (volt)?_BIT(16):0;
	} else {
		/* for 1v2 bitpos is 17 and for 3v3 bit pos 20 */
		bit_pos = (rail)? 17 : 20;

		if (volt > 0x7)
			volt = 0x7;

		/* zero the dcdc1 adjust bits */
		reg_val &= ~(0x7 << bit_pos);
		/* write the new adjust value */
		reg_val |= (volt << bit_pos);
	}

	i2c_write(I2C1_CTRL, AD_SLAVE_ADDR, AD_REG_OTGDCLIC_RW,
			2, reg_val,4);
	return;
}

void setup_sdram_voltage(void)
{
	unsigned int reg_val = 0;

	i2c_init(CGU_SB_I2C1_PCLK_ID,I2C1_CTRL);

	/* Set SDRAM voltage rail to 1.8V default is 1.4V */
	psu_set_voltage(ANALOG_1V8_RAIL, PSU_VOUT3_1_80);
}
#endif

static unsigned long load_address;

static void jump_to_image_no_args(void)
{
	typedef void (*image_entry_noargs_t)(void)__attribute__ ((noreturn));
	image_entry_noargs_t image_entry =
			(image_entry_noargs_t) load_address;

	debug("image entry point: 0x%X\n", load_address);
	image_entry();
}
void jump_to_image_no_args(void) __attribute__ ((noreturn));

void board_init_r(gd_t *id, ulong dummy)
{
	long uartid, image_length, os;
	int rc;
	ulong bootmode;
	volatile unsigned char * tmploadPointer =
		(volatile unsigned char *)(LPC313X_SDRAM_TEMP_DATA_ADDR);

	image_header_t *hdr = (image_header_t *)tmploadPointer;

	gd = id;

	/* Initialize Timer0 */
	timer_init1();

	/* Initialize UART0 */
	gd->baudrate = CONFIG_BAUDRATE;
	gd->flags = 0;
	uartid = serial_init();
	gd->have_console = 1;

	GPIOMuxSetup();

#if defined(CONFIG_USB_EHCI_LPC313X) || defined(CONFIG_USB_DEVICE)
	usb_init_clocks();
#endif

	/* Read Boot mode pins and copy u-boot image from
	 * selected BOOT device 
	 */
	IOCONF->block[IOCONF_GPIO].mode0_clear = LPC313X_BOOTMODE_MASK;
	IOCONF->block[IOCONF_GPIO].mode1_clear = LPC313X_BOOTMODE_MASK;

	udelay(1000);

	bootmode = IOCONF->block[IOCONF_GPIO].pins;
	bootmode &= LPC313X_BOOTMODE_MASK;

	bootmode = (((bootmode & 0x1) << 1) | ((bootmode & 0x2) << 1) |
			((bootmode & 0x4) >> 2));

//	image_length = UBOOT_FILESIZE;
	image_length = 0;

	switch(bootmode) {
#ifdef CONFIG_CPU_NAND_BOOT
	case LPC313X_BOOT_MODE_NAND:
		printf("BOOTMODE: NAND\n");
		nand_init();
		//is_nand_init_done = 1;
		rc = nand_copy_boot_image(image_length, tmploadPointer);
		break;
#endif
#ifdef CONFIG_CPU_SPI_BOOT
	case LPC313X_BOOT_MODE_SPI:
		printf("BOOTMODE: SPI\n");
		spi_init();
		//is_spi_init_done = 1;
		rc = spi_copy_boot_image(image_length, tmploadPointer);
		break;
#endif
#ifdef CONFIG_CPU_USBDFU_BOOT
	case LPC313X_BOOT_MODE_DFU_USB:
		printf("BOOTMODE: USB\n");
		drv_usbtty_init();
		rc = usb_copy_boot_image(image_length, tmploadPointer);
		break;
#endif
#ifdef CONFIG_CPU_MMC_BOOT
	case LPC313X_BOOT_MODE_SD_MMC:
		printf("BOOTMODE: SDMMC\n");
		mmc_legacy_init(0);
		rc = sdmmc_copy_boot_image(tmploadPointer);
		break;
#endif
#ifdef CONFIG_CPU_NOR_BOOT
	case LPC313X_BOOT_MODE_NOR_FLASH:
		printf("BOOTMODE: NOR\n");
		rc = flash_copy_boot_image(image_length, tmploadPointer);
		break;
#endif
#ifdef CONFIG_CPU_UART_BOOT
	case LPC313X_BOOT_MODE_UART:
		printf("BOOTMODE: UART\n");
		rc = uart_copy_boot_image(tmploadPointer);
		break;
#endif
	default:
		break;
	}

	if(rc == -1) {
		printf("valid image is not found\n");
		hang();
	}

	if (!image_check_magic(hdr)) {
		puts("Bad Magic Number\n");
		hang();
	}
	if (!image_check_hcrc(hdr)) {
		puts("Bad Header Checksum\n");
		hang();
	}
	image_print_contents(hdr);

	puts("   Verifying Checksum ... ");
	if (!image_check_dcrc(hdr)) {
		printf("Bad Data CRC\n");
		hang();
	}
	puts("OK\n");

	if (!image_check_target_arch(hdr)) {
		printf("Unsupported Architecture 0x%x\n", image_get_arch(hdr));
		hang();
	}

	os = image_get_os(hdr);
	load_address = image_get_load(hdr);

	memmove(load_address, tmploadPointer + sizeof(image_header_t), image_get_size(hdr));

	switch (os) {
	case IH_OS_U_BOOT:
		puts("Jumping to U-Boot\n");
		jump_to_image_no_args();
		break;
	default:
		printf("Unsupported OS image %d.. Jumping nevertheless..\n", os);
		jump_to_image_no_args();
	}
}


int setenv(const char *varname, const char *varvalue)
{
	return 0;
}

