/*
 * armboot - Startup Code for ARM926EJS CPU-core
 * Based on cpu/arm926ejs/start.S
 * 
 * Copyright (c) 2010 NXP Semiconductors
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

#include <config.h>
#include <common.h>
#include <version.h>
#include <asm/arch/hardware.h>
#include <asm/arch/clock.h>

/*
 * special lookup flags
 */
#define DO_MEM_DELAY	1
#define DO_MEM_READ	2

.globl lowlevel_init
.type lowlevel_init,function
lowlevel_init:
#ifndef CONFIG_SPL_BUILD
	mov	pc, lr		/* do nothing return */
#else
	/*
	* Go setup Memory and board specific bits prior to relocation.
	*/
	ldr	sp, =(0x1103B000)
	bic	sp, sp, #7 /* 8-byte alignment for ABI compliance */

	stmfd	sp!, {ip, lr}
	adr	r0, CGU_FDIV_REG_VALS
	adr	r1, CGU_FDIV_CLKS
	bl	init_clocks /* go setup pll,mux,memory */

#ifdef CONFIG_EA3152
	/*
	 * For LPC3152 board, set IO voltage for 
	 * SDRAM to work properly
	 */
	bl	setup_sdram_voltage
#endif
	/*
	 * Setup Memory Controller & SDRAM Timing.
	 */
	bl	mpmc_sdram_init /* go setup pll,mux,memory */

	/* Return to U-boot via saved link register */
	ldmfd	sp!, {ip, pc}

	/*
	 *************************************************************************
	 *
	 * Initialize MPMC and SDRAM Timing registers
	 *
	 *************************************************************************
	 */
mpmc_sdram_init:
	mov 	r9, lr
	/* memory control configuration */
	/* make r0 relative the current location so that it */
	/* reads INITMEM_DATA out of FLASH rather than memory ! */
	/*   r0 = current word pointer */
	/*   r1 = end word location, one word past last actual word */
	/*   r3 = address for writes, special lookup flags */
	/*   r4 = value for writes, delay constants, or read addresses */
	/*   r2 = location for mem reads */
	adr 	r0, INITMEM_DATA
	adr	r1, INITMEM_DATA_END

mem_loop:
	cmp	r1, r0
	moveq	pc, r9		@ Done

	ldr	r3, [r0], #4	@ Fetch Destination Register Address, or 1 for delay
	ldr	r4, [r0], #4	@ value

	cmp	r3, #DO_MEM_DELAY
	bleq	mem_delay
	beq	mem_loop
	cmp	r3, #DO_MEM_READ
	ldreq	r2, [r4]
	beq	mem_loop
	str	r4, [r3]	@ normal register/ram store
	b	mem_loop

mem_delay:
	ldr	r5, =TIMER0_BASE
	mov	r6, #0x88	    @ divide 6MHz clock by 256, enable
	str	r6, [r5, #8]
	ldr	r6, [r5, #4]    @load current counter
	sub r4, r6, r4
0:	ldr	r6, [r5, #4]	@ timer value
	cmp	r6, r4
	bhi	0b
	mov	r6, #0		@ disable timer
	str	r6, [r5, #8]
	mov	pc, lr

	.ltorg
/* the literal pools origin */

INITMEM_DATA:

	.word	0x13002884 /*SYS_REMAP_ADDR: */
	.word	0x11028000 
	.word	0x13002890 /*SYS_MUX_LCD_EBI:*/
	.word	0x00000001 
	/* Enable External Memory Ctrl */
	.word	0x17008000 /*MPMC_CTRL:*/
	.word	0x1 
	/* Force HCLK to MPMC_CLK to 1:1 ratio */
	.word	0x17008008 /*MPMC_CONFIG:*/
	.word	0x0 
	/* Set MPMC delay gates appropriately based on trace lengths between
	 SDRAM and the chip. Also based on the delay startergy used for SDRAM. */
	.word	0x13002868 /*SYS_MPMC_DELAY:*/
	.word	0x824 
	/* Initialize timings for LCD display, 16 bit */
	.word	0x17008200 /*MPMC_STCONFIG0:*/
	.word	0x81 
	.word	0x17008204 /*MPMC_STWTWEN0:*/
	.word	0x2 
	.word	0x17008208 /*MPMC_STWTOEN0:*/
	.word	0x1
	.word	0x1700820C /*MPMC_STWTRD0:*/
	.word	0x15
	.word	0x17008210 /*MPMC_STWTPG0:*/
	.word	0x2
	.word	0x17008214 /*MPMC_STWTWR0:*/
	.word	0x4
	.word	0x17008218 /*MPMC_STWTTURN0:*/
	.word	0x2

	/* Put Delay of 100us */
	.word	DO_MEM_DELAY
	.word	4
	/* Set command delay startergy */
	.word	0x17008028 /*MPMC_DYRDCFG:*/
	.word	0x1
	/* Configure "device config register" nSDCE0 for proper width
	 * SDRAM 
	 */
	.word	0x17008100 /*MPMC_DYCONFIG:*/
	.word	0x680
	.word	0x17008104 /*MPMC_DYRASCAS:*/
	.word	0x202
	/* Minimum 20ns program 1 so that atleast 2 HCLKs are used */
	.word	0x17008030 /*MPMC_DYTRP:*/
	.word	0x2
	.word	0x17008034 /*MPMC_DYTRAS:*/
	.word	0x4
	.word	0x17008038 /*MPMC_DYTSREX:*/
	.word	0x7
	.word	0x1700803C /*MPMC_DYTAPR:*/
	.word	0x4
	.word	0x17008040 /*MPMC_DYTDAL:*/
	.word	0x5
	.word	0x17008044 /*MPMC_DYTWR:*/
	.word	0x7
	.word	0x17008048 /*MPMC_DYTRC:*/
	.word	0x6
	.word	0x1700804C /*MPMC_DYTRFC:*/
	.word	0x6
	.word	0x17008050 /*MPMC_DYTXSR:*/
	.word	0x7
	.word	0x17008054 /*MPMC_DYTRRD:*/
	.word	0x1
	.word	0x17008058 /*MPMC_DYTMRD:*/
	.word	0x2
	/* Put delay 100us */
	.word	DO_MEM_DELAY
	.word	4
	/* Issue continuous NOP commands (INIT & MRS set) */
	.word	0x17008020 /*MPMC_DYNCTL:*/
	.word	0x183
	/* Put delay 200us */
	.word	DO_MEM_DELAY
	.word	6
	/* Issue a "pre-charge all" command */
	.word	0x17008020
	.word	0x103
	/*******************************************************************
	* Do at least 2 quick refresh cycles now.
	******************************************************************/
	.word	0x17008024 /*MPMC_DYNREF:*/
	.word	0x1
	/* Put delay 250us */
	.word	DO_MEM_DELAY
	.word	8
	/*******************************************************************
	* Set operational refersh rate.
	******************************************************************/
	.word	0x17008024 /*MPMC_DYNREF:*/
	.word	0x2b
	/* Select mode register update mode */
	.word	0x17008020 /*MPMC_DYNCTL*/
	.word	0x83
	/*******************************************************************
	* Program the SDRAM internal mode registers with CAS latency.
	******************************************************************/
	.word DO_MEM_READ
	.word 0x30023000

	/* Select normal operating mode */
	.word	0x17008020 /*MPMC_DYNCTL:*/
	.word	0x3

	/* Enable buffer */
	.word	0x17008100 /*MPMC_DYCONFIG:*/
	.word	0x80680
	.word	0x17008020 /*MPMC_DYNCTL: */
	.word	0x0

	/* program external refresh controller */
	.word   0x13002878  /* SYSCREG_MPMC_TESTMODE0 */
	.word   SYSREG_TEST0_VAL         
	.word   0x1300287c  /*SYSCREG_MPMC_TESTMODE1 */
	.word   SYSREG_TEST1_VAL        

INITMEM_DATA_END:
CGU_FDIV_REG_VALS:
	.word	CGU_DEF_FDIV0_VAL	 
	.word	CGU_DEF_FDIV1_VAL
	.word	CGU_DEF_FDIV2_VAL
	.word	CGU_DEF_FDIV3_VAL
	.word	CGU_DEF_FDIV4_VAL
	.word	CGU_DEF_FDIV5_VAL
	.word	CGU_DEF_FDIV6_VAL
	.word	CGU_DEF_FDIV7_VAL
	.word	CGU_DEF_FDIV8_VAL
	.word	CGU_DEF_FDIV9_VAL
	.word	CGU_DEF_FDIV10_VAL
	.word	CGU_DEF_FDIV11_VAL
	.word	CGU_DEF_FDIV12_VAL
	.word	CGU_DEF_FDIV13_VAL
	.word	CGU_DEF_FDIV14_VAL
	.word	CGU_DEF_FDIV15_VAL
	.word	CGU_DEF_FDIV16_VAL
	.word	CGU_DEF_FDIV17_VAL
	.word	CGU_DEF_FDIV18_VAL
	.word	CGU_DEF_FDIV19_VAL
	.word	CGU_DEF_FDIV20_VAL
	.word	CGU_DEF_FDIV21_VAL
	.word	CGU_DEF_FDIV22_VAL
	.word	CGU_DEF_FDIV23_VAL
	.word	CGU_DEF_DYNFDIV0_VAL
	.word	CGU_DEF_DYNFDIV1_VAL
	.word	CGU_DEF_DYNFDIV2_VAL
	.word	CGU_DEF_DYNFDIV3_VAL
	.word	CGU_DEF_DYNFDIV4_VAL
	.word	CGU_DEF_DYNFDIV5_VAL
	.word	CGU_DEF_DYNFDIV6_VAL

CGU_FDIV_CLKS:
	.word	CGU_DEF_DOMAIN0_DIV0	 
	.word	CGU_DEF_DOMAIN0_DIV1
	.word	CGU_DEF_DOMAIN0_DIV2
	.word	CGU_DEF_DOMAIN0_DIV3
	.word	CGU_DEF_DOMAIN0_DIV4
	.word	CGU_DEF_DOMAIN0_DIV5
	.word	CGU_DEF_DOMAIN0_DIV6
	.word	CGU_DEF_DOMAIN1_DIV7	 
	.word	CGU_DEF_DOMAIN1_DIV8
	.word	CGU_DEF_DOMAIN2_DIV9
	.word	CGU_DEF_DOMAIN2_DIV10
	.word	CGU_DEF_DOMAIN3_DIV11
	.word	CGU_DEF_DOMAIN3_DIV12
	.word	CGU_DEF_DOMAIN3_DIV13
	.word	CGU_DEF_DOMAIN4_DIV14	 
	.word	CGU_DEF_DOMAIN5_DIV15
	.word	CGU_DEF_DOMAIN6_DIV16
	.word	CGU_DEF_DOMAIN7_DIV17
	.word	CGU_DEF_DOMAIN7_DIV18
	.word	CGU_DEF_DOMAIN7_DIV19
	.word	CGU_DEF_DOMAIN7_DIV20
	.word	CGU_DEF_DOMAIN7_DIV21
	.word	CGU_DEF_DOMAIN7_DIV22
	.word	CGU_DEF_DOMAIN10_DIV23
	.word	CGU_DEF_DYNFDIV0_CFG
	.word	CGU_DEF_DYNFDIV1_CFG
	.word	CGU_DEF_DYNFDIV2_CFG
	.word	CGU_DEF_DYNFDIV3_CFG
	.word	CGU_DEF_DYNFDIV4_CFG
	.word	CGU_DEF_DYNFDIV5_CFG
	.word	CGU_DEF_DYNFDIV6_CFG
#endif

