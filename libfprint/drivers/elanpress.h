/*
 * Driver for ELAN press-type (touch) fingerprint sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * These sensors stream raw frames like the swipe sensors handled by the
 * elan driver, but the finger rests on the sensor instead of being swiped
 * across it. The imaged area is far too small for reliable minutiae
 * matching, so enrollment stores the images themselves and matching is
 * done by normalized cross-correlation, like the Windows driver does.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include "fpi-device.h"
#include "fpi-usb-transfer.h"

#define ELANPRESS_VEND_ID 0x04f3

#define ELANPRESS_EP_CMD_OUT (0x1 | FPI_USB_ENDPOINT_OUT)
#define ELANPRESS_EP_CMD_IN (0x3 | FPI_USB_ENDPOINT_IN)
#define ELANPRESS_EP_IMG_IN (0x2 | FPI_USB_ENDPOINT_IN)

#define ELANPRESS_CMD_LEN 2
#define ELANPRESS_CMD_TIMEOUT 5000
#define ELANPRESS_FRAME_TIMEOUT 2000

/* interval between finger presence polls */
#define ELANPRESS_POLL_INTERVAL_MS 30

/* frames captured per touch; the finger is static so these only serve to
 * average out noise */
#define ELANPRESS_MIN_FRAMES 3
#define ELANPRESS_MAX_FRAMES 10

/* number of touches stored during enrollment */
#define ELANPRESS_ENROLL_STAGES 8

/* version tag for the serialized print data */
#define ELANPRESS_PRINT_VERSION 1

G_DECLARE_FINAL_TYPE (FpiDeviceElanPress, fpi_device_elanpress, FPI,
                      DEVICE_ELANPRESS, FpDevice);

static const FpIdEntry elanpress_id_table[] = {
  {.vid = ELANPRESS_VEND_ID, .pid = 0x0c6e, },
  {.vid = 0, .pid = 0, },
};
