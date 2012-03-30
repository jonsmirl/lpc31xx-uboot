#
# (C) Copyright 2000-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#

include $(TOPDIR)/config.mk

LIB	= $(obj)lib$(SOC).o

COBJS	= timer.o
COBJS	+= serial.o
COBJS	+= clock.o
COBJS	+= mci.o
COBJS	+= dos_partition.o
COBJS	+= spi.o
COBJS   += usbdfu.o
ifndef CONFIG_SPL_BUILD
COBJS   += usbpoll.o
COBJS	+= init.o
COBJS	+= i2c.o
COBJS	+= nand_params.o
COBJS	+= nand_format.o
COBJS	+= crc32.o
else
CFLAGS += -DUBOOT_FILESIZE=$(shell stat -c%s $(TOPDIR)/u-boot.img)
COBJS	+= spl.o
endif

SRCS	:= $(SOBJS:.o=.S) $(COBJS:.o=.c)
OBJS	:= $(addprefix $(obj),$(COBJS) $(SOBJS))

all:	$(obj).depend $(LIB)

$(LIB):	$(OBJS)
	$(call cmd_link_o_target, $(OBJS))


#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
