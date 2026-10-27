/*
 * Unit tests for the elanpress correlation matcher
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

#include <math.h>
#include <stdio.h>

#include "drivers/elanpress-match.h"

#define MASTER_SIZE 320
#define N_ENROLL 8

static int img_w = 96;
static int img_h = 96;

/* synthetic finger: whorl-like ridges around a core, with a seed-dependent
 * core position, ridge period and smooth warp */
static guint8 *
make_master (guint32 seed)
{
  GRand *rand = g_rand_new_with_seed (seed);
  guint8 *m = g_malloc (MASTER_SIZE * MASTER_SIZE);
  gdouble cx = g_rand_double_range (rand, 120, 200);
  gdouble cy = g_rand_double_range (rand, 120, 200);
  gdouble period = g_rand_double_range (rand, 7.0, 10.0);
  gdouble ex = g_rand_double_range (rand, 0.7, 1.3);
  gdouble wa = g_rand_double_range (rand, 2.0, 6.0);
  gdouble wf = g_rand_double_range (rand, 0.01, 0.03);

  for (int y = 0; y < MASTER_SIZE; y++)
    for (int x = 0; x < MASTER_SIZE; x++)
      {
        gdouble dx = (x - cx) * ex, dy = y - cy;
        gdouble r = sqrt (dx * dx + dy * dy) + wa * sin (wf * x) * cos (wf * y);

        m[y * MASTER_SIZE + x] = (guint8) (128 + 100 * sin (2 * G_PI * r / period));
      }

  g_rand_free (rand);
  return m;
}

/* crop a w x h window of the master centred at (cx, cy), rotated by deg,
 * with additive noise, as a press would image it */
static guint8 *
take_press (const guint8 *m, gdouble cx, gdouble cy, gdouble deg, GRand *rand)
{
  guint8 *out = g_malloc (img_w * img_h);
  gdouble ct = cos (deg * G_PI / 180), st = sin (deg * G_PI / 180);

  for (int y = 0; y < img_h; y++)
    for (int x = 0; x < img_w; x++)
      {
        gdouble ox = x - img_w / 2.0, oy = y - img_h / 2.0;
        int sx = (int) lround (cx + ox * ct - oy * st);
        int sy = (int) lround (cy + ox * st + oy * ct);
        gdouble v = m[CLAMP (sy, 0, MASTER_SIZE - 1) * MASTER_SIZE +
                      CLAMP (sx, 0, MASTER_SIZE - 1)];

        v += g_rand_double_range (rand, -40, 40);
        out[y * img_w + x] = (guint8) CLAMP (v, 0, 255);
      }

  return out;
}

typedef struct
{
  guint8 *master;
  guint8 *enrolled[N_ENROLL];
} Finger;

static void
finger_init (Finger *f, guint32 seed, GRand *rand)
{
  f->master = make_master (seed);
  for (int i = 0; i < N_ENROLL; i++)
    f->enrolled[i] = take_press (f->master,
                                 160 + g_rand_double_range (rand, -12, 12),
                                 160 + g_rand_double_range (rand, -8, 8),
                                 g_rand_double_range (rand, -6, 6), rand);
}

static void
finger_clear (Finger *f)
{
  g_free (f->master);
  for (int i = 0; i < N_ENROLL; i++)
    g_free (f->enrolled[i]);
}

static gdouble
reference_score (const guint8 *probe, Finger *f)
{
  gdouble best = -1.0;

  for (int i = 0; i < N_ENROLL; i++)
    best = MAX (best, elanpress_ncc_best (probe, f->enrolled[i], img_w, img_h));
  return best;
}

/* the coarse-to-fine search must find (nearly) the same peak as the
 * exhaustive reference search, for genuine and impostor probes alike */
static void
test_matches_reference (void)
{
  g_autoptr(GRand) rand = g_rand_new_with_seed (1);
  gdouble t_ref = 0, t_new = 0, t_early = 0;

  for (guint32 seed = 1; seed <= 4; seed++)
    {
      Finger f, other;

      finger_init (&f, seed, rand);
      finger_init (&other, seed + 100, rand);

      for (int k = 0; k < 3; k++)
        {
          g_autofree guint8 *genuine = take_press (f.master,
                                                   160 + g_rand_double_range (rand, -15, 15),
                                                   160 + g_rand_double_range (rand, -10, 10),
                                                   g_rand_double_range (rand, -10, 10), rand);
          g_autofree guint8 *impostor = take_press (other.master,
                                                    160 + g_rand_double_range (rand, -15, 15),
                                                    160 + g_rand_double_range (rand, -10, 10),
                                                    g_rand_double_range (rand, -10, 10), rand);
          gint64 t0, t1, t2, t3;
          gdouble ref_g, new_g, early_g, ref_i, new_i;

          t0 = g_get_monotonic_time ();
          ref_g = reference_score (genuine, &f);
          t1 = g_get_monotonic_time ();
          /* threshold above 1 disables the early exit */
          new_g = elanpress_match_images (genuine, f.enrolled, N_ENROLL,
                                          img_w, img_h, 2.0, NULL);
          t2 = g_get_monotonic_time ();
          early_g = elanpress_match_images (genuine, f.enrolled, N_ENROLL,
                                            img_w, img_h,
                                            ELANPRESS_NCC_THRESHOLD, NULL);
          t3 = g_get_monotonic_time ();
          t_ref += t1 - t0;
          t_new += t2 - t1;
          t_early += t3 - t2;

          ref_i = reference_score (impostor, &f);
          new_i = elanpress_match_images (impostor, f.enrolled, N_ENROLL,
                                          img_w, img_h, 2.0, NULL);

          g_test_message ("seed %u: genuine ref %.3f new %.3f early %.3f, impostor ref %.3f new %.3f",
                          seed, ref_g, new_g, early_g, ref_i, new_i);

          g_assert_cmpfloat (ref_g, >=, ELANPRESS_NCC_THRESHOLD);
          g_assert_cmpfloat (new_g, >=, ref_g - 0.02);
          g_assert_cmpfloat (early_g, >=, ELANPRESS_NCC_THRESHOLD);
          g_assert_cmpfloat (ref_i, <, ELANPRESS_NCC_THRESHOLD);
          g_assert_cmpfloat (new_i, <, ELANPRESS_NCC_THRESHOLD);
        }

      finger_clear (&f);
      finger_clear (&other);
    }

  g_test_message ("%dx%d, 8 enrolled images, per genuine match: reference %.0f ms, new %.0f ms, new with early exit %.0f ms",
                  img_w, img_h, t_ref / 12000, t_new / 12000, t_early / 12000);
}

static void
test_cancelled (void)
{
  g_autoptr(GRand) rand = g_rand_new_with_seed (2);
  g_autoptr(GCancellable) cancellable = g_cancellable_new ();
  g_autofree guint8 *probe = NULL;
  Finger f;

  finger_init (&f, 7, rand);
  probe = take_press (f.master, 160, 160, 0, rand);
  g_cancellable_cancel (cancellable);
  g_assert_cmpfloat (elanpress_match_images (probe, f.enrolled, N_ENROLL,
                                             img_w, img_h, 2.0, cancellable),
                     <, 0);
  finger_clear (&f);
}

static void
test_frame_delta (void)
{
  unsigned short bg[4] = { 1000, 1000, 1000, 1000 };
  unsigned short frame[4] = { 3000, 1000, 600, 1000 };
  gdouble pos, neg;

  elanpress_frame_delta (frame, bg, 4, &pos, &neg);
  g_assert_cmpfloat_with_epsilon (pos, 500, 1e-9);
  g_assert_cmpfloat_with_epsilon (neg, 100, 1e-9);
}

int
main (int argc, char *argv[])
{
  const char *size = g_getenv ("ELANPRESS_TEST_SIZE");

  if (size)
    g_assert_cmpint (sscanf (size, "%dx%d", &img_w, &img_h), ==, 2);

  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/elanpress/match/reference", test_matches_reference);
  g_test_add_func ("/elanpress/match/cancelled", test_cancelled);
  g_test_add_func ("/elanpress/frame-delta", test_frame_delta);

  return g_test_run ();
}
