/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 * This file by Marc-Etienne M.Léveillé <marc.etienne.ml@gmail.com>
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

#ifndef __FEATURES_H
#define __FEATURES_H

// To pick and choose the features and targets we want, we need to disable them
// all first with -DENABLE_SELECTED_FEATURES_ONLY and then enable the ones we
// want with -DENABLE_THINGS_WE_WANT. If nothing is defined, the default
// behavior will be the same as before: build with all features.
#ifndef ENABLE_SELECTED_FEATURES_ONLY
#define ENABLE_TARGET_PROTO_JTAG
#define ENABLE_TARGET_PROTO_SWDIO

#define ENABLE_TARGET_ARCH_CORTEX_M
#define ENABLE_TARGET_STM32F1
#define ENABLE_TARGET_STM32F4
#define ENABLE_TARGET_STM32L0
#define ENABLE_TARGET_STM32L4
#define ENABLE_TARGET_LPC11XX
#define ENABLE_TARGET_LPC15XX
#define ENABLE_TARGET_LPC43XX
#define ENABLE_TARGET_SAM3X
#define ENABLE_TARGET_SAM4L
#define ENABLE_TARGET_NRF51
#define ENABLE_TARGET_SAMD
#define ENABLE_TARGET_LMI
#define ENABLE_TARGET_KINETIS
#define ENABLE_TARGET_EFM32

#define ENABLE_TARGET_ARCH_CORTEX_A

#endif

#if defined(ENABLE_TARGET_STM32F1) || \
    defined(ENABLE_TARGET_STM32F4) || \
    defined(ENABLE_TARGET_STM32L0) || \
    defined(ENABLE_TARGET_STM32L4) || \
    defined(ENABLE_TARGET_LPC11XX) || \
    defined(ENABLE_TARGET_LPC15XX) || \
    defined(ENABLE_TARGET_LPC43XX) || \
    defined(ENABLE_TARGET_SAM3X) || \
    defined(ENABLE_TARGET_SAM4L) || \
    defined(ENABLE_TARGET_NRF51) || \
    defined(ENABLE_TARGET_SAMD) || \
    defined(ENABLE_TARGET_LMI) || \
    defined(ENABLE_TARGET_KINETIS) || \
    defined(ENABLE_TARGET_EFM32)
#define ENABLE_TARGET_ARCH_CORTEX_M
#endif

#if !(defined(ENABLE_TARGET_PROTO_JTAG) || defined(ENABLE_TARGET_PROTO_SWDIO))
#error "You need to enable at least one of ENABLE_TARGET_PROTO_JTAG or ENABLE_TARGET_PROTO_SWDIO."
#endif

#if !(defined(ENABLE_TARGET_ARCH_CORTEX_M) || defined(ENABLE_TARGET_ARCH_CORTEX_A))
#error "You need to enable support for a least one target (ENABLE_TRAGET_*)."
#endif

#endif
