/*
 * Image processing and correlation matching for ELAN press-type sensors
 * Copyright (C) 2026 Filip Spanne
 *
 * The imaged area of these sensors is far too small for reliable minutiae
 * matching, so prints store the enrolled images themselves and matching is
 * done by zero-mean normalized cross-correlation over translations, like
 * the Windows driver does.
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

#include "elanpress-match.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

static int
elanpress_cmp_short (const void *a, const void *b)
{
  return (int) (*(unsigned short *) a - *(unsigned short *) b);
}

/* the wire format is column-major with frame-height-tall columns; rotate
 * to row-major (same layout as the elan driver uses for these sensors) */
void
elanpress_rotate_frame (const guint8 *raw, unsigned short *out, int w, int h)
{
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      out[y * w + x] = ((const guint16 *) raw)[y + x * h];
}

/* Elantech's recommended 2-step non-linear normalization of the 14-bit
 * background-subtracted frame onto 8 bits (see the elan driver) */
static void
elanpress_normalize (const unsigned short *in, guint8 *out, unsigned int size)
{
  g_autofree unsigned short *sorted = g_malloc (size * sizeof (short));
  unsigned short lvl0, lvl1, lvl2, lvl3;
  unsigned short px;

  memcpy (sorted, in, size * sizeof (short));
  qsort (sorted, size, sizeof (short), elanpress_cmp_short);

  lvl0 = sorted[0];
  lvl1 = sorted[size * 3 / 10];
  lvl2 = sorted[size * 65 / 100];
  lvl3 = sorted[size - 1];

  for (unsigned int i = 0; i < size; i++)
    {
      px = in[i];
      if (px < lvl1)
        px = (px - lvl0) * 99 / MAX (lvl1 - lvl0, 1);
      else if (px < lvl2)
        px = 99 + ((px - lvl1) * 56 / MAX (lvl2 - lvl1, 1));
      else
        px = 155 + MIN ((px - lvl2) * 100 / MAX (lvl3 - lvl2, 1), 100);
      out[i] = (guint8) px;
    }
}

/* average the stable frames of a touch (newest-first list), subtract the
 * background and normalize; returns NULL if the finger wasn't properly on
 * the sensor */
guint8 *
elanpress_process_frames (GSList *frames, int num_frames,
                          const unsigned short *background,
                          unsigned int size)
{
  g_autofree unsigned int *sum = g_malloc0 (size * sizeof (unsigned int));
  g_autofree unsigned short *avg = g_malloc (size * sizeof (unsigned short));
  guint8 *out;
  int skip_oldest = 0, skip_newest = 0, used = 0, i;
  unsigned long total = 0;
  GSList *l;

  if (num_frames >= 6)
    {
      skip_oldest = ELANPRESS_SKIP_OLDEST;
      skip_newest = ELANPRESS_SKIP_NEWEST;
    }
  else if (num_frames >= 4)
    {
      skip_oldest = 1;
      skip_newest = 1;
    }

  for (l = frames, i = 0; l; l = l->next, i++)
    {
      const unsigned short *frame = l->data;

      if (i < skip_newest || i >= num_frames - skip_oldest)
        continue;
      for (unsigned int j = 0; j < size; j++)
        sum[j] += frame[j];
      used++;
    }

  if (used == 0)
    return NULL;

  for (unsigned int j = 0; j < size; j++)
    {
      unsigned int px = sum[j] / used;
      unsigned short bg = background ? background[j] : 0;

      avg[j] = px > bg ? px - bg : 0;
      total += avg[j];
    }

  if (total == 0)
    {
      g_debug ("touch darker than background, ignoring");
      return NULL;
    }

  out = g_malloc (size);
  elanpress_normalize (avg, out, size);
  return out;
}

/* sums how much brighter each pixel is than the background frame, to infer
 * a touch without relying on cmd_pre_scan (see elanpress.c) */
gboolean
elanpress_frame_has_touch (const unsigned short *frame,
                          const unsigned short *background,
                          unsigned int size)
{
  gint64 sum = 0;

  for (unsigned int i = 0; i < size; i++)
    {
      int d = (int) frame[i] - (int) background[i];

      if (d > 0)
        sum += d;
    }

  return sum > (gint64) size * ELANPRESS_TOUCH_MIN_MEAN_DELTA;
}

static gdouble
elanpress_ncc_at (const guint8 *a, const guint8 *b, int w, int h,
                  int dx, int dy)
{
  int x0 = MAX (0, dx), x1 = MIN (w, w + dx);
  int y0 = MAX (0, dy), y1 = MIN (h, h + dy);
  long n = (long) (x1 - x0) * (y1 - y0);
  gdouble sa = 0, sb = 0, ma, mb, num = 0, da = 0, db = 0;

  if (n < ELANPRESS_NCC_MIN_OVERLAP_PX)
    return -1.0;

  for (int y = y0; y < y1; y++)
    for (int x = x0; x < x1; x++)
      {
        sa += a[y * w + x];
        sb += b[(y - dy) * w + (x - dx)];
      }
  ma = sa / n;
  mb = sb / n;

  for (int y = y0; y < y1; y++)
    for (int x = x0; x < x1; x++)
      {
        gdouble va = a[y * w + x] - ma;
        gdouble vb = b[(y - dy) * w + (x - dx)] - mb;

        num += va * vb;
        da += va * va;
        db += vb * vb;
      }

  if (da <= 0 || db <= 0)
    return -1.0;

  return num / sqrt (da * db);
}

/* best zero-mean normalized cross-correlation over translations: coarse
 * grid first, then refinement around the best hit */
static gdouble
elanpress_ncc_best_translation (const guint8 *a, const guint8 *b, int w, int h)
{
  gdouble best = -1.0, c;
  int best_dx = 0, best_dy = 0;

  for (int dy = -ELANPRESS_NCC_MAX_DY; dy <= ELANPRESS_NCC_MAX_DY; dy += 2)
    for (int dx = -ELANPRESS_NCC_MAX_DX; dx <= ELANPRESS_NCC_MAX_DX; dx += 3)
      {
        c = elanpress_ncc_at (a, b, w, h, dx, dy);
        if (c > best)
          {
            best = c;
            best_dx = dx;
            best_dy = dy;
          }
      }

  for (int dy = best_dy - 2; dy <= best_dy + 2; dy++)
    for (int dx = best_dx - 3; dx <= best_dx + 3; dx++)
      {
        c = elanpress_ncc_at (a, b, w, h, dx, dy);
        if (c > best)
          best = c;
      }

  return best;
}

/* nearest-neighbour rotation of b around its centre by theta radians,
 * out-of-bounds samples read as 0 */
static void
elanpress_rotate_image (const guint8 *in, guint8 *out, int w, int h, gdouble theta)
{
  gdouble cx = w / 2.0, cy = h / 2.0, ct = cos (theta), st = sin (theta);

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        int sx = (int) round (cx + (x - cx) * ct + (y - cy) * st);
        int sy = (int) round (cy - (x - cx) * st + (y - cy) * ct);

        out[y * w + x] = (sx >= 0 && sx < w && sy >= 0 && sy < h) ?
                         in[sy * w + sx] : 0;
      }
}

/* best correlation over both translation and small rotations of b, so a
 * touch that landed at a slightly different angle than the enrolled image
 * still matches */
gdouble
elanpress_ncc_best (const guint8 *a, const guint8 *b, int w, int h)
{
  g_autofree guint8 *rotated = g_malloc (w * h);
  gdouble best = -1.0;

  for (gdouble deg = -ELANPRESS_NCC_MAX_ROT_DEG; deg <= ELANPRESS_NCC_MAX_ROT_DEG;
       deg += ELANPRESS_NCC_ROT_STEP_DEG)
    {
      elanpress_rotate_image (b, rotated, w, h, deg * G_PI / 180.0);
      best = MAX (best, elanpress_ncc_best_translation (a, rotated, w, h));
    }

  return best;
}
