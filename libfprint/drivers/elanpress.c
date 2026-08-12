/*
 * Driver for ELAN press-type (touch) fingerprint sensors
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

#define FP_COMPONENT "elanpress"

#include "drivers_api.h"
#include "elanpress.h"
#include "elanpress-match.h"

static const guint8 cmd_led_on[ELANPRESS_CMD_LEN] = {0x40, 0x31};
static const guint8 cmd_get_image[ELANPRESS_CMD_LEN] = {0x00, 0x09};
static const guint8 cmd_stop[ELANPRESS_CMD_LEN] = {0x00, 0x0b};
static const guint8 cmd_get_sensor_dim[ELANPRESS_CMD_LEN] = {0x00, 0x0c};

struct _FpiDeviceElanPress
{
  FpDevice        parent;

  unsigned short  frame_width;
  unsigned short  frame_height;

  /* raw background frame (no finger), rotated to row-major */
  unsigned short *background;

  /* most recently captured frame, not yet classified as touch/no-touch */
  unsigned short *last_frame;

  /* rotated raw frames of the touch being captured */
  GSList         *frames;
  int             num_frames;

  /* processed images collected during enrollment */
  GPtrArray      *enroll_images;
  int             enroll_stage;
};

G_DEFINE_TYPE (FpiDeviceElanPress, fpi_device_elanpress, FP_TYPE_DEVICE);

/* === image processing === */

static unsigned int
elanpress_frame_size (FpiDeviceElanPress *self)
{
  return (unsigned int) self->frame_width * self->frame_height;
}

static guint8 *
elanpress_process_touch (FpiDeviceElanPress *self)
{
  return elanpress_process_frames (self->frames, self->num_frames,
                                   self->background,
                                   elanpress_frame_size (self));
}

/* === print (de)serialization === */

static void
elanpress_print_set_data (FpiDeviceElanPress *self, FpPrint *print,
                          GPtrArray *images)
{
  GVariantBuilder images_builder = G_VARIANT_BUILDER_INIT (G_VARIANT_TYPE ("aay"));
  GVariant *fpi_data;

  for (guint i = 0; i < images->len; i++)
    g_variant_builder_add_value (
      &images_builder,
      g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                 g_ptr_array_index (images, i),
                                 elanpress_frame_size (self),
                                 sizeof (guint8)));

  fpi_data = g_variant_new ("(qqqaay)",
                            (guint16) ELANPRESS_PRINT_VERSION,
                            (guint16) self->frame_width,
                            (guint16) self->frame_height,
                            &images_builder);

  fpi_print_set_type (print, FPI_PRINT_RAW);
  g_object_set (print, "fpi-data", fpi_data, NULL);
}

/* returns the stored images or NULL if the print is not usable */
static GPtrArray *
elanpress_print_get_images (FpiDeviceElanPress *self, FpPrint *print)
{
  g_autoptr(GVariant) fpi_data = NULL;
  g_autoptr(GVariant) images_v = NULL;
  GPtrArray *images;
  guint16 version = 0, width = 0, height = 0;
  gsize n_images;

  g_object_get (print, "fpi-data", &fpi_data, NULL);
  if (!fpi_data || !g_variant_check_format_string (fpi_data, "(qqq@aay)", FALSE))
    return NULL;

  g_variant_get (fpi_data, "(qqq@aay)", &version, &width, &height, &images_v);
  if (version != ELANPRESS_PRINT_VERSION ||
      width != self->frame_width || height != self->frame_height)
    return NULL;

  n_images = g_variant_n_children (images_v);
  if (n_images == 0)
    return NULL;

  images = g_ptr_array_new_full (n_images, g_free);
  for (gsize i = 0; i < n_images; i++)
    {
      g_autoptr(GVariant) img_v = g_variant_get_child_value (images_v, i);
      gsize len = 0;
      gconstpointer data = g_variant_get_fixed_array (img_v, &len, sizeof (guint8));

      if (len != elanpress_frame_size (self))
        {
          g_ptr_array_unref (images);
          return NULL;
        }
      g_ptr_array_add (images, g_memdup2 (data, len));
    }

  return images;
}

static gdouble
elanpress_match_print (FpiDeviceElanPress *self, const guint8 *probe,
                       FpPrint *print)
{
  g_autoptr(GPtrArray) images = elanpress_print_get_images (self, print);
  gdouble best = -1.0;

  if (!images)
    return -1.0;

  for (guint i = 0; i < images->len; i++)
    {
      gdouble c = elanpress_ncc_best (probe, g_ptr_array_index (images, i),
                                      self->frame_width, self->frame_height);
      best = MAX (best, c);
    }

  return best;
}

/* === USB helpers === */

static void
elanpress_reset_capture (FpiDeviceElanPress *self)
{
  g_slist_free_full (g_steal_pointer (&self->frames), g_free);
  self->num_frames = 0;
  g_clear_pointer (&self->last_frame, g_free);
}

static void
elanpress_send_cmd (FpiSsm *ssm, FpDevice *dev, const guint8 *cmd)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk_full (transfer, ELANPRESS_EP_CMD_OUT,
                                   (guint8 *) cmd, ELANPRESS_CMD_LEN, NULL);
  fpi_usb_transfer_submit (transfer, ELANPRESS_CMD_TIMEOUT,
                           fpi_device_get_cancellable (dev),
                           fpi_ssm_usb_transfer_cb, NULL);
}

static void
elanpress_read (FpiSsm *ssm, FpDevice *dev, guint8 ep, gsize len,
                guint timeout, FpiUsbTransferCallback callback)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk (transfer, ep, len);
  fpi_usb_transfer_submit (transfer, timeout,
                           fpi_device_get_cancellable (dev),
                           callback, NULL);
}

/* === stop helper: turn the sensor off after an action === */

static void
elanpress_send_stop (FpDevice *dev)
{
  g_autoptr(FpiUsbTransfer) transfer = fpi_usb_transfer_new (dev);
  g_autoptr(GError) error = NULL;

  fpi_usb_transfer_fill_bulk_full (transfer, ELANPRESS_EP_CMD_OUT,
                                   (guint8 *) cmd_stop, ELANPRESS_CMD_LEN,
                                   NULL);
  if (!fpi_usb_transfer_submit_sync (transfer, ELANPRESS_CMD_TIMEOUT, &error))
    fp_warn ("failed to send stop command: %s", error->message);
}

/* === touch capture state machine === */

/* cmd_pre_scan's status byte only ever answers once per power-up and then
 * wedges permanently, so finger presence is inferred from the image itself
 * (elanpress_frame_has_touch) rather than queried */

enum capture_states {
  CAPTURE_LED_ON,
  CAPTURE_BG_SEND,
  CAPTURE_BG_READ,
  CAPTURE_POLL_SEND,
  CAPTURE_POLL_READ,
  CAPTURE_NUM_STATES,
};

static void
elanpress_frame_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                    gpointer user_data, GError *error)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);
  unsigned short *frame;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  frame = g_malloc (elanpress_frame_size (self) * sizeof (short));
  elanpress_rotate_frame (transfer->buffer, frame,
                          self->frame_width, self->frame_height);

  if (fpi_ssm_get_cur_state (transfer->ssm) == CAPTURE_BG_READ)
    {
      g_free (self->background);
      self->background = frame;
      fp_dbg ("captured background frame");
    }
  else
    {
      g_free (self->last_frame);
      self->last_frame = frame;
    }

  fpi_ssm_next_state (transfer->ssm);
}

static void
capture_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);
  gsize frame_bytes = elanpress_frame_size (self) * 2;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CAPTURE_LED_ON:
      elanpress_send_cmd (ssm, dev, cmd_led_on);
      break;

    case CAPTURE_BG_SEND:
      if (self->background)
        {
          fpi_ssm_jump_to_state (ssm, CAPTURE_POLL_SEND);
          break;
        }
      elanpress_send_cmd (ssm, dev, cmd_get_image);
      break;

    case CAPTURE_BG_READ:
      elanpress_read (ssm, dev, ELANPRESS_EP_IMG_IN, frame_bytes,
                      ELANPRESS_FRAME_TIMEOUT, elanpress_frame_cb);
      break;

    case CAPTURE_POLL_SEND:
      fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NEEDED);
      elanpress_send_cmd (ssm, dev, cmd_get_image);
      break;

    case CAPTURE_POLL_READ:
      elanpress_read (ssm, dev, ELANPRESS_EP_IMG_IN, frame_bytes,
                      ELANPRESS_FRAME_TIMEOUT, elanpress_frame_cb);
      break;

    default:
      if (elanpress_frame_has_touch (self->last_frame, self->background,
                                     elanpress_frame_size (self)))
        {
          self->frames = g_slist_prepend (self->frames,
                                          g_steal_pointer (&self->last_frame));
          self->num_frames++;
          fpi_device_report_finger_status (dev,
                                           FP_FINGER_STATUS_NEEDED |
                                           FP_FINGER_STATUS_PRESENT);
          if (self->num_frames >= ELANPRESS_MAX_FRAMES)
            fpi_ssm_mark_completed (ssm);
          else
            fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_POLL_SEND,
                                           ELANPRESS_POLL_INTERVAL_MS);
          break;
        }

      g_clear_pointer (&self->last_frame, g_free);
      if (self->num_frames >= ELANPRESS_MIN_FRAMES)
        {
          /* finger lifted after enough frames: touch complete */
          fpi_ssm_mark_completed (ssm);
        }
      else if (self->num_frames > 0)
        {
          /* bounced touch, start over */
          fp_dbg ("finger lifted after only %d frames, retrying",
                  self->num_frames);
          elanpress_reset_capture (self);
          fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_POLL_SEND,
                                         ELANPRESS_POLL_INTERVAL_MS);
        }
      else
        {
          /* still waiting for the finger to touch down */
          fpi_ssm_jump_to_state_delayed (ssm, CAPTURE_POLL_SEND,
                                         ELANPRESS_POLL_INTERVAL_MS);
        }
      break;
    }
}

static void
elanpress_capture_touch (FpiDeviceElanPress *self, FpiSsmCompletedCallback done)
{
  FpiSsm *ssm;

  elanpress_reset_capture (self);
  /* one extra state acts as the post-frame decision point */
  ssm = fpi_ssm_new (FP_DEVICE (self), capture_run_state,
                     CAPTURE_NUM_STATES + 1);
  fpi_ssm_start (ssm, done);
}

/* === enroll === */

static void
elanpress_enroll_touch_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);
  FpPrint *print = NULL;
  guint8 *image;

  fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NONE);

  if (error)
    {
      elanpress_send_stop (dev);
      fpi_device_enroll_complete (dev, NULL, error);
      return;
    }

  image = elanpress_process_touch (self);
  elanpress_reset_capture (self);

  if (image && elanpress_image_quality (image, elanpress_frame_size (self)) <
      ELANPRESS_MIN_QUALITY_STDDEV)
    {
      fp_dbg ("enrolled touch too low-contrast to keep, retrying");
      g_clear_pointer (&image, g_free);
    }

  if (!image)
    {
      fpi_device_enroll_progress (dev, self->enroll_stage, NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
      elanpress_capture_touch (self, elanpress_enroll_touch_done);
      return;
    }

  g_ptr_array_add (self->enroll_images, image);
  self->enroll_stage++;
  fpi_device_enroll_progress (dev, self->enroll_stage, NULL, NULL);

  if (self->enroll_stage < ELANPRESS_ENROLL_STAGES)
    {
      elanpress_capture_touch (self, elanpress_enroll_touch_done);
      return;
    }

  elanpress_send_stop (dev);

  fpi_device_get_enroll_data (dev, &print);
  elanpress_print_set_data (self, print, self->enroll_images);
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);

  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

static void
elanpress_enroll (FpDevice *dev)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);

  self->enroll_stage = 0;
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
  self->enroll_images = g_ptr_array_new_with_free_func (g_free);

  elanpress_capture_touch (self, elanpress_enroll_touch_done);
}

/* === verify and identify === */

static void
elanpress_match_touch_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);
  FpiDeviceAction action = fpi_device_get_current_action (dev);
  g_autofree guint8 *probe = NULL;

  fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NONE);
  elanpress_send_stop (dev);

  if (error)
    {
      if (action == FPI_DEVICE_ACTION_IDENTIFY)
        fpi_device_identify_complete (dev, error);
      else
        fpi_device_verify_complete (dev, error);
      return;
    }

  probe = elanpress_process_touch (self);
  elanpress_reset_capture (self);

  if (probe && elanpress_image_quality (probe, elanpress_frame_size (self)) <
      ELANPRESS_MIN_QUALITY_STDDEV)
    {
      fp_dbg ("touch too low-contrast to match, treating as failed capture");
      g_clear_pointer (&probe, g_free);
    }

  if (!probe)
    {
      GError *retry = fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL);

      if (action == FPI_DEVICE_ACTION_IDENTIFY)
        {
          fpi_device_identify_report (dev, NULL, NULL, retry);
          fpi_device_identify_complete (dev, NULL);
        }
      else
        {
          fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, retry);
          fpi_device_verify_complete (dev, NULL);
        }
      return;
    }

  if (action == FPI_DEVICE_ACTION_IDENTIFY)
    {
      GPtrArray *gallery = NULL;
      FpPrint *best_print = NULL;
      gdouble best = -1.0;

      fpi_device_get_identify_data (dev, &gallery);
      for (guint i = 0; i < gallery->len; i++)
        {
          FpPrint *print = g_ptr_array_index (gallery, i);
          gdouble c = elanpress_match_print (self, probe, print);

          if (c > best)
            {
              best = c;
              best_print = print;
            }
        }

      fp_dbg ("best NCC %.3f (threshold %.2f)", best, ELANPRESS_NCC_THRESHOLD);
      if (best >= ELANPRESS_NCC_THRESHOLD)
        fpi_device_identify_report (dev, best_print, NULL, NULL);
      else
        fpi_device_identify_report (dev, NULL, NULL, NULL);
      fpi_device_identify_complete (dev, NULL);
    }
  else
    {
      FpPrint *print = NULL;
      gdouble c;

      fpi_device_get_verify_data (dev, &print);
      c = elanpress_match_print (self, probe, print);
      fp_dbg ("NCC %.3f (threshold %.2f)", c, ELANPRESS_NCC_THRESHOLD);

      if (c < 0)
        {
          fpi_device_verify_complete (dev,
                                      fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
          return;
        }

      fpi_device_verify_report (dev,
                                c >= ELANPRESS_NCC_THRESHOLD ?
                                FPI_MATCH_SUCCESS : FPI_MATCH_FAIL,
                                NULL, NULL);
      fpi_device_verify_complete (dev, NULL);
    }
}

static void
elanpress_identify_verify (FpDevice *dev)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);

  elanpress_capture_touch (self, elanpress_match_touch_done);
}

/* === open and close === */

enum open_states {
  OPEN_GET_DIM_SEND,
  OPEN_GET_DIM_READ,
  OPEN_NUM_STATES,
};

static void
elanpress_dim_cb (FpiUsbTransfer *transfer, FpDevice *dev,
                  gpointer user_data, GError *error)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  /* raw frames are column-major: byte 0 is the column height, byte 2 the
   * number of columns (see the elan driver) */
  self->frame_height = transfer->buffer[0];
  self->frame_width = transfer->buffer[2];

  /* work around sensors returning the sizes as zero-based index rather
   * than the number of pixels (same quirk as the elan driver) */
  if ((self->frame_width % 2 == 1) && (self->frame_height % 2 == 1))
    {
      self->frame_width++;
      self->frame_height++;
    }
  fp_dbg ("sensor dimensions, WxH: %dx%d", self->frame_width,
          self->frame_height);

  if (self->frame_width < 50 || self->frame_height < 20)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Unexpected sensor dimensions %dx%d",
                                                     self->frame_width,
                                                     self->frame_height));
      return;
    }

  fpi_ssm_next_state (transfer->ssm);
}

static void
open_run_state (FpiSsm *ssm, FpDevice *dev)
{
  switch (fpi_ssm_get_cur_state (ssm))
    {
    case OPEN_GET_DIM_SEND:
      elanpress_send_cmd (ssm, dev, cmd_get_sensor_dim);
      break;

    case OPEN_GET_DIM_READ:
      elanpress_read (ssm, dev, ELANPRESS_EP_CMD_IN, 4,
                      ELANPRESS_CMD_TIMEOUT, elanpress_dim_cb);
      break;
    }
}

static void
open_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  fpi_device_open_complete (dev, error);
}

static void
elanpress_open (FpDevice *dev)
{
  GError *error = NULL;

  if (!g_usb_device_reset (fpi_device_get_usb_device (dev), &error))
    {
      fpi_device_open_complete (dev, error);
      return;
    }

  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (dev),
                                     0, 0, &error))
    {
      fpi_device_open_complete (dev, error);
      return;
    }

  /* a prior crashed/killed session may have left the sensor mid-action; force it idle before use */
  elanpress_send_stop (dev);

  fpi_ssm_start (fpi_ssm_new (dev, open_run_state, OPEN_NUM_STATES),
                 open_complete);
}

static void
elanpress_close (FpDevice *dev)
{
  FpiDeviceElanPress *self = FPI_DEVICE_ELANPRESS (dev);
  GError *error = NULL;

  elanpress_reset_capture (self);
  g_clear_pointer (&self->background, g_free);
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);

  g_usb_device_release_interface (fpi_device_get_usb_device (dev),
                                  0, 0, &error);
  fpi_device_close_complete (dev, error);
}

static void
elanpress_cancel (FpDevice *dev)
{
  /* in-flight transfers are submitted with the action's cancellable, so
   * they fail on their own; nothing device-side to do here */
}

static void
fpi_device_elanpress_init (FpiDeviceElanPress *self)
{
}

static void
fpi_device_elanpress_class_init (FpiDeviceElanPressClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = "elanpress";
  dev_class->full_name = "ElanTech press-type fingerprint sensor";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = elanpress_id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->nr_enroll_stages = ELANPRESS_ENROLL_STAGES;
  /* the sensor spends lock-screen sessions waiting for a finger; it is a
   * tiny capacitive sensor designed to be always-on */
  dev_class->temp_hot_seconds = -1;

  dev_class->open = elanpress_open;
  dev_class->close = elanpress_close;
  dev_class->enroll = elanpress_enroll;
  dev_class->verify = elanpress_identify_verify;
  dev_class->identify = elanpress_identify_verify;
  dev_class->cancel = elanpress_cancel;

  fpi_device_class_auto_initialize_features (dev_class);
}
