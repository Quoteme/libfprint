/*
 * Image processing and correlation matching for ELAN press-type sensors
 * Copyright (C) 2026 Filip Spanne
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

#include <glib.h>

/* frames dropped at the start (finger settling) and end (possible lift)
 * of a touch when enough frames are available */
#define ELANPRESS_SKIP_OLDEST 2
#define ELANPRESS_SKIP_NEWEST 1

/* matching parameters, validated against captures of the Windows driver:
 * genuine presses of the same finger region correlate at 0.59-0.91 while
 * impostor images stay below 0.47 */
#define ELANPRESS_NCC_THRESHOLD 0.55
#define ELANPRESS_NCC_MAX_DX 60
#define ELANPRESS_NCC_MAX_DY 20
#define ELANPRESS_NCC_MIN_OVERLAP_PX 1500

/* small placement-rotation tolerance: a press on a small unconstrained pad
 * doesn't land at the same angle each time */
#define ELANPRESS_NCC_MAX_ROT_DEG 12
#define ELANPRESS_NCC_ROT_STEP_DEG 4

void     elanpress_rotate_frame (const guint8 *raw, unsigned short *out,
                                 int w, int h);
guint8 * elanpress_process_frames (GSList *frames, int num_frames,
                                   const unsigned short *background,
                                   unsigned int size);
gdouble  elanpress_ncc_best (const guint8 *a, const guint8 *b,
                             int w, int h);
