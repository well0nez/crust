#
# Copyright © 2017-2022 The Crust Firmware Authors.
# SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0-only
#

obj-y += msgbox.o
ifeq ($(CONFIG_PLATFORM_H713),y)
obj-y += sun50i-h713-msgbox.o
else
obj-y += sunxi-msgbox.o
endif
