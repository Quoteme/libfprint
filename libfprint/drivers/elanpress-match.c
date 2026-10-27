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

/* standard deviation of pixel values in a normalized touch image, as a
 * cheap proxy for how much ridge contrast it actually contains; flat,
 * low-contrast captures correlate poorly against everything (genuine or
 * not) so it is better to reject them at capture time than to store or
 * match against them */
gdouble
elanpress_image_quality (const guint8 *img, unsigned int size)
{
  gdouble sum = 0, sum_sq = 0, mean;

  for (unsigned int i = 0; i < size; i++)
    sum += img[i];
  mean = sum / size;

  for (unsigned int i = 0; i < size; i++)
    {
      gdouble d = img[i] - mean;

      sum_sq += d * d;
    }

  return sqrt (sum_sq / size);
}

/* mean positive and mean negative per-pixel delta of a frame against the
 * background, to infer a touch without relying on cmd_pre_scan (see
 * elanpress.c); a large negative delta means the background itself was
 * taken with a finger on the sensor */
void
elanpress_frame_delta (const unsigned short *frame,
                       const unsigned short *background,
                       unsigned int size,
                       gdouble *pos_mean, gdouble *neg_mean)
{
  gint64 pos = 0, neg = 0;

  for (unsigned int i = 0; i < size; i++)
    {
      int d = (int) frame[i] - (int) background[i];

      if (d > 0)
        pos += d;
      else
        neg -= d;
    }

  *pos_mean = (gdouble) pos / size;
  *neg_mean = (gdouble) neg / size;
}

/* summed-area tables of an image's pixel values and squared pixel values,
 * (w + 1) x (h + 1) with a zero first row/column, so the sum over any
 * overlap rectangle costs four lookups instead of a pass over it */
typedef struct
{
  gint64 *sum;
  gint64 *sum_sq;
} ElanpressIntegral;

static void
elanpress_integral_init (ElanpressIntegral *ii, int w, int h)
{
  ii->sum = g_malloc0 ((gsize) (w + 1) * (h + 1) * sizeof (gint64));
  ii->sum_sq = g_malloc0 ((gsize) (w + 1) * (h + 1) * sizeof (gint64));
}

static void
elanpress_integral_clear (ElanpressIntegral *ii)
{
  g_clear_pointer (&ii->sum, g_free);
  g_clear_pointer (&ii->sum_sq, g_free);
}

static void
elanpress_integral_fill (ElanpressIntegral *ii, const guint8 *img, int w, int h)
{
  int stride = w + 1;

  for (int y = 0; y < h; y++)
    {
      gint64 row = 0, row_sq = 0;

      for (int x = 0; x < w; x++)
        {
          int px = img[y * w + x];

          row += px;
          row_sq += px * px;
          ii->sum[(y + 1) * stride + x + 1] = ii->sum[y * stride + x + 1] + row;
          ii->sum_sq[(y + 1) * stride + x + 1] = ii->sum_sq[y * stride + x + 1] + row_sq;
        }
    }
}

static inline gint64
elanpress_integral_rect (const gint64 *t, int stride, int x0, int y0, int x1, int y1)
{
  return t[y1 * stride + x1] - t[y0 * stride + x1] - t[y1 * stride + x0] + t[y0 * stride + x0];
}

/* zero-mean NCC over the overlap of a and b shifted by (dx, dy). Written
 * in the n-scaled form (n*Sab - Sa*Sb) / sqrt((n*Saa - Sa^2)(n*Sbb - Sb^2)),
 * which equals the textbook mean-subtracted formula but needs only the
 * cross term Sab per offset, summed exactly in integers; the other sums
 * come from the precomputed integral images */
static gdouble
elanpress_ncc_at (const guint8 *a, const ElanpressIntegral *ia,
                  const guint8 *b, const ElanpressIntegral *ib,
                  int w, int h, int dx, int dy, gint64 min_overlap)
{
  int x0 = MAX (0, dx), x1 = MIN (w, w + dx);
  int y0 = MAX (0, dy), y1 = MIN (h, h + dy);
  int stride = w + 1, len = x1 - x0;
  gint64 n = (gint64) (x1 - x0) * (y1 - y0);
  gint64 sa, sb, saa, sbb, sab = 0, da, db;

  if (n < min_overlap)
    return -1.0;

  sa = elanpress_integral_rect (ia->sum, stride, x0, y0, x1, y1);
  saa = elanpress_integral_rect (ia->sum_sq, stride, x0, y0, x1, y1);
  sb = elanpress_integral_rect (ib->sum, stride, x0 - dx, y0 - dy, x1 - dx, y1 - dy);
  sbb = elanpress_integral_rect (ib->sum_sq, stride, x0 - dx, y0 - dy, x1 - dx, y1 - dy);

  da = n * saa - sa * sa;
  db = n * sbb - sb * sb;
  if (da <= 0 || db <= 0)
    return -1.0;

  for (int y = y0; y < y1; y++)
    {
      const guint8 *pa = a + y * w + x0;
      const guint8 *pb = b + (y - dy) * w + (x0 - dx);
      guint32 row = 0;

      for (int x = 0; x < len; x++)
        row += (guint32) pa[x] * pb[x];
      sab += row;
    }

  return (gdouble) (n * sab - sa * sb) / sqrt ((gdouble) da * (gdouble) db);
}

/* best zero-mean normalized cross-correlation over translations: coarse
 * grid first, then refinement around the best hit */
static gdouble
elanpress_ncc_best_translation (const guint8 *a, const ElanpressIntegral *ia,
                                const guint8 *b, const ElanpressIntegral *ib,
                                int w, int h)
{
  gdouble best = -1.0, c;
  int best_dx = 0, best_dy = 0;

  for (int dy = -ELANPRESS_NCC_MAX_DY; dy <= ELANPRESS_NCC_MAX_DY; dy += 2)
    for (int dx = -ELANPRESS_NCC_MAX_DX; dx <= ELANPRESS_NCC_MAX_DX; dx += 3)
      {
        c = elanpress_ncc_at (a, ia, b, ib, w, h, dx, dy,
                              ELANPRESS_NCC_MIN_OVERLAP_PX);
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
        c = elanpress_ncc_at (a, ia, b, ib, w, h, dx, dy,
                              ELANPRESS_NCC_MIN_OVERLAP_PX);
        if (c > best)
          best = c;
      }

  return best;
}

/* bilinear rotation of b around its centre by theta radians; out-of-bounds
 * corners contribute 0, so edges fade out instead of the hard cutoff a
 * nearest-neighbour sample would give, which correlates a little more
 * forgivingly and avoids the extra quantization noise nearest-neighbour
 * adds at non-multiple-of-90-degree angles */
static void
elanpress_rotate_image (const guint8 *in, guint8 *out, int w, int h, gdouble theta)
{
  gdouble cx = w / 2.0, cy = h / 2.0, ct = cos (theta), st = sin (theta);

  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      {
        gdouble sx = cx + (x - cx) * ct + (y - cy) * st;
        gdouble sy = cy - (x - cx) * st + (y - cy) * ct;
        int x0 = (int) floor (sx), y0 = (int) floor (sy);
        gdouble fx = sx - x0, fy = sy - y0;
        gdouble acc = 0;

        for (int dy = 0; dy <= 1; dy++)
          for (int dx = 0; dx <= 1; dx++)
            {
              int sxi = x0 + dx, syi = y0 + dy;
              gdouble wgt = (dx ? fx : 1 - fx) * (dy ? fy : 1 - fy);

              if (sxi >= 0 && sxi < w && syi >= 0 && syi < h)
                acc += wgt * in[syi * w + sxi];
            }
        out[y * w + x] = (guint8) CLAMP (lround (acc), 0, 255);
      }
}

/* exhaustive full-resolution reference search: best correlation over both
 * translation and small rotations of b, so a
 * touch that landed at a slightly different angle than the enrolled image
 * still matches. The angle grid is coarse (ELANPRESS_NCC_ROT_STEP_DEG), so
 * on top of it a parabola is fit through the best sample and its two
 * neighbours to estimate the true (sub-degree) peak, which is then
 * re-checked with one extra rotation+search - the same trick used for
 * sub-pixel peak estimation in image correlation/optical-flow literature,
 * applied here to the rotation axis instead of translation. */
gdouble
elanpress_ncc_best (const guint8 *a, const guint8 *b, int w, int h)
{
  g_autofree guint8 *rotated = g_malloc (w * h);
  int n = (int) (2 * ELANPRESS_NCC_MAX_ROT_DEG / ELANPRESS_NCC_ROT_STEP_DEG) + 1;
  g_autofree gdouble *scores = g_malloc (n * sizeof (gdouble));
  ElanpressIntegral ia, ir;
  gdouble best = -1.0, best_deg = 0;
  int best_i = 0;

  elanpress_integral_init (&ia, w, h);
  elanpress_integral_init (&ir, w, h);
  elanpress_integral_fill (&ia, a, w, h);

  for (int i = 0; i < n; i++)
    {
      gdouble deg = -ELANPRESS_NCC_MAX_ROT_DEG + i * ELANPRESS_NCC_ROT_STEP_DEG;

      elanpress_rotate_image (b, rotated, w, h, deg * G_PI / 180.0);
      elanpress_integral_fill (&ir, rotated, w, h);
      scores[i] = elanpress_ncc_best_translation (a, &ia, rotated, &ir, w, h);
      if (scores[i] > best)
        {
          best = scores[i];
          best_deg = deg;
          best_i = i;
        }
    }

  if (best_i > 0 && best_i < n - 1)
    {
      gdouble sl = scores[best_i - 1], sc = scores[best_i], sr = scores[best_i + 1];
      gdouble denom = sl - 2 * sc + sr;

      if (fabs (denom) > 1e-9)
        {
          gdouble offset = CLAMP (0.5 * (sl - sr) / denom, -1.0, 1.0);
          gdouble refined_deg = best_deg + offset * ELANPRESS_NCC_ROT_STEP_DEG;

          elanpress_rotate_image (b, rotated, w, h, refined_deg * G_PI / 180.0);
          elanpress_integral_fill (&ir, rotated, w, h);
          best = MAX (best, elanpress_ncc_best_translation (a, &ia, rotated, &ir, w, h));
        }
    }

  elanpress_integral_clear (&ia);
  elanpress_integral_clear (&ir);
  return best;
}

/* === coarse-to-fine, parallel matching === */

/* an image together with its integral image and a 2x box-downsampled copy
 * (plus that copy's integral image) for the coarse search level */
typedef struct
{
  const guint8      *img;
  ElanpressIntegral  ii;
  guint8            *half;
  ElanpressIntegral  hii;
  int                w, h, hw, hh;
} ElanpressLevels;

static void
elanpress_levels_init (ElanpressLevels *lv, const guint8 *img, int w, int h)
{
  lv->img = img;
  lv->w = w;
  lv->h = h;
  lv->hw = w / 2;
  lv->hh = h / 2;
  lv->half = g_malloc ((gsize) lv->hw * lv->hh);

  for (int y = 0; y < lv->hh; y++)
    for (int x = 0; x < lv->hw; x++)
      {
        const guint8 *p = img + 2 * y * w + 2 * x;

        lv->half[y * lv->hw + x] = (p[0] + p[1] + p[w] + p[w + 1] + 2) / 4;
      }

  elanpress_integral_init (&lv->ii, w, h);
  elanpress_integral_fill (&lv->ii, img, w, h);
  elanpress_integral_init (&lv->hii, lv->hw, lv->hh);
  elanpress_integral_fill (&lv->hii, lv->half, lv->hw, lv->hh);
}

static void
elanpress_levels_clear (ElanpressLevels *lv)
{
  g_clear_pointer (&lv->half, g_free);
  elanpress_integral_clear (&lv->ii);
  elanpress_integral_clear (&lv->hii);
}

/* full-resolution correlation around a half-resolution hit (hx, hy) */
static gdouble
elanpress_ncc_refine (const ElanpressLevels *a, const ElanpressLevels *b,
                      int hx, int hy)
{
  gdouble best = -1.0;

  for (int dy = 2 * hy - 2; dy <= 2 * hy + 2; dy++)
    for (int dx = 2 * hx - 2; dx <= 2 * hx + 2; dx++)
      best = MAX (best, elanpress_ncc_at (a->img, &a->ii, b->img, &b->ii,
                                          a->w, a->h, dx, dy,
                                          ELANPRESS_NCC_MIN_OVERLAP_PX));

  return best;
}

/* exhaustive translation search at half resolution (a quarter of the
 * pixels and a quarter of the offsets; the ridge period is only ~4 px at
 * that scale, so the peak is too narrow to skip offsets), then
 * full-resolution refinement around the best few local maxima: ridges are
 * periodic, so the
 * correlation surface has near-equal peaks one ridge period apart and the
 * coarse level alone can't reliably tell which is the true one. The
 * returned score is always a full-resolution NCC, comparable to
 * ELANPRESS_NCC_THRESHOLD. */
static gdouble
elanpress_ncc_pyramid_translation (const ElanpressLevels *a,
                                   const ElanpressLevels *b)
{
  int hdx = ELANPRESS_NCC_MAX_DX / 2, hdy = ELANPRESS_NCC_MAX_DY / 2;
  int nx = 2 * hdx + 1, ny = 2 * hdy + 1;
  g_autofree gdouble *coarse = g_malloc ((gsize) nx * ny * sizeof (gdouble));
  gdouble cand_score[ELANPRESS_NCC_CANDIDATES];
  int cand_x[ELANPRESS_NCC_CANDIDATES], cand_y[ELANPRESS_NCC_CANDIDATES];
  int n_cand = 0;
  gdouble best = -1.0;

  for (int iy = 0; iy < ny; iy++)
    for (int ix = 0; ix < nx; ix++)
      coarse[iy * nx + ix] =
        elanpress_ncc_at (a->half, &a->hii, b->half, &b->hii, a->hw, a->hh,
                          ix - hdx, iy - hdy,
                          ELANPRESS_NCC_MIN_OVERLAP_PX / 4);

  for (int iy = 0; iy < ny; iy++)
    for (int ix = 0; ix < nx; ix++)
      {
        gdouble c = coarse[iy * nx + ix];
        gboolean is_max = c > -1.0;
        int pos;

        for (int ny_ = MAX (iy - 1, 0); is_max && ny_ <= MIN (iy + 1, ny - 1); ny_++)
          for (int nx_ = MAX (ix - 1, 0); nx_ <= MIN (ix + 1, nx - 1); nx_++)
            if (coarse[ny_ * nx + nx_] > c)
              is_max = FALSE;
        if (!is_max)
          continue;

        /* insert into the descending top-N list */
        for (pos = n_cand; pos > 0 && cand_score[pos - 1] < c; pos--)
          if (pos < ELANPRESS_NCC_CANDIDATES)
            {
              cand_score[pos] = cand_score[pos - 1];
              cand_x[pos] = cand_x[pos - 1];
              cand_y[pos] = cand_y[pos - 1];
            }
        if (pos < ELANPRESS_NCC_CANDIDATES)
          {
            cand_score[pos] = c;
            cand_x[pos] = ix - hdx;
            cand_y[pos] = iy - hdy;
            n_cand = MIN (n_cand + 1, ELANPRESS_NCC_CANDIDATES);
          }
      }

  for (int i = 0; i < n_cand; i++)
    best = MAX (best, elanpress_ncc_refine (a, b, cand_x[i], cand_y[i]));

  return best;
}

static gdouble
elanpress_ncc_at_angle (const ElanpressLevels *a, const guint8 *b, gdouble deg)
{
  g_autofree guint8 *rotated = g_malloc ((gsize) a->w * a->h);
  ElanpressLevels lb;
  gdouble score;

  elanpress_rotate_image (b, rotated, a->w, a->h, deg * G_PI / 180.0);
  elanpress_levels_init (&lb, rotated, a->w, a->h);
  score = elanpress_ncc_pyramid_translation (a, &lb);
  elanpress_levels_clear (&lb);

  return score;
}

typedef struct
{
  ElanpressLevels    probe;
  guint8 *const     *images;
  guint              n_images;
  int                n_angles;
  /* per (image, angle) score; written by exactly one job each */
  gdouble           *scores;
  gdouble           *refined;
  gdouble            threshold;
  GCancellable      *cancellable;
  gint               found;
} ElanpressMatchCtx;

#define ELANPRESS_NOT_RUN -2.0

static gboolean
elanpress_match_should_stop (ElanpressMatchCtx *ctx)
{
  return g_atomic_int_get (&ctx->found) ||
         g_cancellable_is_cancelled (ctx->cancellable);
}

/* one job: one enrolled image at one grid angle */
static void
elanpress_angle_job (gpointer data, gpointer user_data)
{
  ElanpressMatchCtx *ctx = user_data;
  int job = GPOINTER_TO_INT (data) - 1;
  int img = job / ctx->n_angles, ai = job % ctx->n_angles;
  gdouble deg = -ELANPRESS_NCC_MAX_ROT_DEG + ai * ELANPRESS_NCC_ROT_STEP_DEG;
  gdouble s;

  if (elanpress_match_should_stop (ctx))
    return;

  s = elanpress_ncc_at_angle (&ctx->probe, ctx->images[img], deg);
  ctx->scores[job] = s;
  if (s >= ctx->threshold)
    g_atomic_int_set (&ctx->found, TRUE);
}

/* one job: sub-degree parabolic refinement of one enrolled image's angle
 * peak (see elanpress_ncc_best) */
static void
elanpress_refine_job (gpointer data, gpointer user_data)
{
  ElanpressMatchCtx *ctx = user_data;
  int img = GPOINTER_TO_INT (data) - 1;
  const gdouble *s = ctx->scores + img * ctx->n_angles;
  int bi = 0;
  gdouble sl, sc, sr, denom, offset;

  if (elanpress_match_should_stop (ctx))
    return;

  for (int i = 1; i < ctx->n_angles; i++)
    if (s[i] > s[bi])
      bi = i;
  if (bi == 0 || bi == ctx->n_angles - 1)
    return;

  sl = s[bi - 1];
  sc = s[bi];
  sr = s[bi + 1];
  denom = sl - 2 * sc + sr;
  if (sl <= -1.0 || sr <= -1.0 || fabs (denom) <= 1e-9)
    return;

  offset = CLAMP (0.5 * (sl - sr) / denom, -1.0, 1.0);
  ctx->refined[img] = elanpress_ncc_at_angle (&ctx->probe, ctx->images[img],
                                              -ELANPRESS_NCC_MAX_ROT_DEG +
                                              (bi + offset) * ELANPRESS_NCC_ROT_STEP_DEG);
  if (ctx->refined[img] >= ctx->threshold)
    g_atomic_int_set (&ctx->found, TRUE);
}

static void
elanpress_run_jobs (GFunc func, ElanpressMatchCtx *ctx,
                    const int *jobs, int n_jobs)
{
  GThreadPool *pool;
  int n_threads = CLAMP ((int) g_get_num_processors (), 1,
                         ELANPRESS_MATCH_MAX_THREADS);

  pool = g_thread_pool_new (func, ctx, MIN (n_threads, n_jobs), FALSE, NULL);
  if (!pool)
    {
      for (int i = 0; i < n_jobs; i++)
        func (GINT_TO_POINTER (jobs[i] + 1), ctx);
      return;
    }

  for (int i = 0; i < n_jobs; i++)
    g_thread_pool_push (pool, GINT_TO_POINTER (jobs[i] + 1), NULL);
  /* waits for every queued job to finish */
  g_thread_pool_free (pool, FALSE, TRUE);
}

/* best correlation of the probe against a set of enrolled images, over
 * translation and small rotations. Coarse-to-fine (see
 * elanpress_ncc_pyramid_translation), spread over a thread pool as one job
 * per (image, angle), angle 0 first since most presses land near it, and
 * stops as soon as any score reaches threshold since the match decision
 * can't change after that. Returns -1 if nothing could be compared. */
gdouble
elanpress_match_images (const guint8 *probe, guint8 *const *images,
                        guint n_images, int w, int h, gdouble threshold,
                        GCancellable *cancellable)
{
  ElanpressMatchCtx ctx = { 0 };
  int n_angles = (int) (2 * ELANPRESS_NCC_MAX_ROT_DEG / ELANPRESS_NCC_ROT_STEP_DEG) + 1;
  int n_jobs = n_images * n_angles, k = 0;
  g_autofree int *jobs = g_malloc ((gsize) MAX (n_jobs, 1) * sizeof (int));
  g_autofree gdouble *scores = g_malloc ((gsize) MAX (n_jobs, 1) * sizeof (gdouble));
  g_autofree gdouble *refined = g_malloc ((gsize) MAX (n_images, 1) * sizeof (gdouble));
  gdouble best = -1.0;

  if (n_images == 0)
    return -1.0;

  for (int i = 0; i < n_jobs; i++)
    scores[i] = ELANPRESS_NOT_RUN;
  for (guint i = 0; i < n_images; i++)
    refined[i] = ELANPRESS_NOT_RUN;

  elanpress_levels_init (&ctx.probe, probe, w, h);
  ctx.images = images;
  ctx.n_images = n_images;
  ctx.n_angles = n_angles;
  ctx.scores = scores;
  ctx.refined = refined;
  ctx.threshold = threshold;
  ctx.cancellable = cancellable;

  /* angle order 0, +step, -step, +2 step, ... for every image */
  for (int d = 0; d <= n_angles / 2; d++)
    for (int sign = 1; sign >= -1; sign -= 2)
      {
        int ai = n_angles / 2 + sign * d;

        if (d == 0 && sign < 0)
          continue;
        for (guint img = 0; img < n_images; img++)
          jobs[k++] = img * n_angles + ai;
      }
  elanpress_run_jobs (elanpress_angle_job, &ctx, jobs, k);

  if (!elanpress_match_should_stop (&ctx))
    {
      for (guint img = 0; img < n_images; img++)
        jobs[img] = img;
      elanpress_run_jobs (elanpress_refine_job, &ctx, jobs, n_images);
    }

  for (int i = 0; i < n_jobs; i++)
    best = MAX (best, scores[i]);
  for (guint i = 0; i < n_images; i++)
    best = MAX (best, refined[i]);

  elanpress_levels_clear (&ctx.probe);
  return best;
}
