/*
 * Standalone libusb probe for the ELAN press-type sensor (04f3:0c6e),
 * bypassing libfprint entirely. For protocol debugging only - not part
 * of the driver build.
 *
 * Build:
 *   gcc -O2 -Wall elanpress_probe.c -o elanpress_probe $(pkg-config --cflags
 * --libs libusb-1.0)
 *
 * Usage:
 *   sudo ./elanpress_probe dim
 *   sudo ./elanpress_probe status [count] [interval_ms]
 *   sudo ./elanpress_probe capture <out.pgm>
 *   sudo ./elanpress_probe led-on
 *   sudo ./elanpress_probe stop
 *
 * If claiming the interface fails as busy, fprintd is probably still
 * holding the device: `sudo systemctl stop fprintd` first.
 */

#include <libusb-1.0/libusb.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define VENDOR_ID 0x04f3
#define PRODUCT_ID 0x0c6e

#define EP_CMD_OUT 0x01
#define EP_CMD_IN 0x83
#define EP_IMG_IN 0x82

#define CMD_LEN 2
#define CMD_TIMEOUT_MS 5000
#define FRAME_TIMEOUT_MS 2000

static const uint8_t cmd_led_on[CMD_LEN] = {0x40, 0x31};
static const uint8_t cmd_pre_scan[CMD_LEN] = {0x40, 0x3f};
static const uint8_t cmd_get_image[CMD_LEN] = {0x00, 0x09};
static const uint8_t cmd_stop[CMD_LEN] = {0x00, 0x0b};
static const uint8_t cmd_get_sensor_dim[CMD_LEN] = {0x00, 0x0c};

static libusb_device_handle *dev;
static int iface_claimed;

/* Runs on every exit path (normal return, exit(), or SIGINT). A run that
 * dies mid pre_scan/status cycle otherwise leaves the sensor's firmware
 * state machine mid-transaction, which wedges it for the *next* invocation
 * (it stops answering entirely) until something sends cmd_stop again. */
static void cleanup(void) {
  if (!dev)
    return;

  if (iface_claimed) {
    int transferred = 0;

    libusb_bulk_transfer(dev, EP_CMD_OUT, (uint8_t *)cmd_stop, CMD_LEN,
                         &transferred, 1000);
    libusb_release_interface(dev, 0);
  }
  libusb_close(dev);
  dev = NULL;
}

static void handle_signal(int sig) {
  (void)sig;
  exit(1);
}

static void die(const char *msg, int rc) {
  fprintf(stderr, "%s: %s\n", msg, libusb_error_name(rc));
  exit(1);
}

static void send_cmd(const uint8_t *cmd) {
  int transferred = 0;
  int rc = libusb_bulk_transfer(dev, EP_CMD_OUT, (uint8_t *)cmd, CMD_LEN,
                                &transferred, CMD_TIMEOUT_MS);

  if (rc != 0 || transferred != CMD_LEN)
    die("send_cmd failed", rc);
}

static void read_bulk(uint8_t ep, uint8_t *buf, int len, int timeout_ms) {
  int transferred = 0;
  int rc = libusb_bulk_transfer(dev, ep, buf, len, &transferred, timeout_ms);

  if (rc != 0 || transferred != len) {
    fprintf(stderr, "read_bulk(ep=0x%02x, len=%d) failed: %s (got %d bytes)\n",
            ep, len, libusb_error_name(rc), transferred);
    exit(1);
  }
}

static uint8_t read_status(void) {
  uint8_t byte = 0;

  send_cmd(cmd_pre_scan);
  read_bulk(EP_CMD_IN, &byte, 1, CMD_TIMEOUT_MS);
  return byte;
}

static void get_dimensions(int *width, int *height) {
  uint8_t buf[4];

  send_cmd(cmd_get_sensor_dim);
  read_bulk(EP_CMD_IN, buf, 4, CMD_TIMEOUT_MS);

  *height = buf[0];
  *width = buf[2];

  /* same zero-based-index quirk workaround as the elanpress driver */
  if ((*width % 2 == 1) && (*height % 2 == 1)) {
    (*width)++;
    (*height)++;
  }
}

static double now_ms(void) {
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void cmd_dim(void) {
  int w, h;

  get_dimensions(&w, &h);
  printf("sensor dimensions: %dx%d (%d px, %d bytes/frame)\n", w, h, w * h,
         w * h * 2);
}

static void cmd_status(int count, int interval_ms) {
  double t0;

  /* the real driver always turns the LED on before polling status
   * (CAPTURE_LED_ON) - replicate that here rather than polling cold */
  send_cmd(cmd_led_on);
  printf("sent led-on, watching status...\n");

  t0 = now_ms();
  for (int i = 0; count <= 0 || i < count; i++) {
    uint8_t byte = read_status();

    printf("[%8.1fms] status = 0x%02x\n", now_ms() - t0, byte);
    if (interval_ms > 0)
      usleep(interval_ms * 1000);
  }
}

/* column-major raw -> row-major, matching elanpress_rotate_frame() */
static void rotate_frame(const uint16_t *raw, uint16_t *out, int w, int h) {
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++)
      out[y * w + x] = raw[y + x * h];
}

static void write_pgm(const char *path, const uint16_t *frame, int w, int h) {
  uint16_t lo = 0xffff, hi = 0;
  FILE *f;

  for (int i = 0; i < w * h; i++) {
    if (frame[i] < lo)
      lo = frame[i];
    if (frame[i] > hi)
      hi = frame[i];
  }
  if (hi == lo)
    hi = lo + 1;

  f = fopen(path, "wb");
  if (!f) {
    perror("fopen");
    exit(1);
  }

  fprintf(f, "P5\n%d %d\n255\n", w, h);
  for (int i = 0; i < w * h; i++) {
    int v = (frame[i] - lo) * 255 / (hi - lo);
    fputc((uint8_t)v, f);
  }
  fclose(f);
  printf("wrote %s (raw range %u-%u)\n", path, lo, hi);
}

/* unconditionally requests a frame - no finger-status gate at all */
static void cmd_capture(const char *outfile) {
  int w, h, frame_bytes;
  uint8_t *raw;
  uint16_t *rotated;

  get_dimensions(&w, &h);
  frame_bytes = w * h * 2;

  raw = malloc(frame_bytes);
  rotated = malloc(frame_bytes);

  send_cmd(cmd_get_image);
  read_bulk(EP_IMG_IN, raw, frame_bytes, FRAME_TIMEOUT_MS);

  rotate_frame((const uint16_t *)raw, rotated, w, h);
  write_pgm(outfile, rotated, w, h);

  free(raw);
  free(rotated);
}

int main(int argc, char **argv) {
  int rc;

  if (argc < 2) {
    fprintf(stderr,
            "usage: %s dim\n"
            "       %s status [count] [interval_ms]\n"
            "       %s capture <out.pgm>\n"
            "       %s led-on\n"
            "       %s stop\n",
            argv[0], argv[0], argv[0], argv[0], argv[0]);
    return 1;
  }

  rc = libusb_init(NULL);
  if (rc != 0)
    die("libusb_init", rc);

  dev = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
  if (!dev) {
    fprintf(stderr, "device %04x:%04x not found (run as root?)\n", VENDOR_ID,
            PRODUCT_ID);
    return 1;
  }

  atexit(cleanup);
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  libusb_set_auto_detach_kernel_driver(dev, 1);
  rc = libusb_claim_interface(dev, 0);
  if (rc != 0)
    die("libusb_claim_interface (is fprintd still holding it?)", rc);
  iface_claimed = 1;

  /* resync in case a previous run died mid-transaction and left the sensor
   * wedged (it was seen to answer once, then stop responding entirely) */
  {
    int transferred = 0;

    libusb_bulk_transfer(dev, EP_CMD_OUT, (uint8_t *)cmd_stop, CMD_LEN,
                         &transferred, 1000);
    usleep(50000);
  }

  if (strcmp(argv[1], "dim") == 0) {
    cmd_dim();
  } else if (strcmp(argv[1], "status") == 0) {
    int count = argc > 2 ? atoi(argv[2]) : 0;
    int interval_ms = argc > 3 ? atoi(argv[3]) : 30;

    cmd_status(count, interval_ms);
  } else if (strcmp(argv[1], "capture") == 0) {
    if (argc < 3) {
      fprintf(stderr, "capture needs an output path\n");
      return 1;
    }
    cmd_capture(argv[2]);
  } else if (strcmp(argv[1], "led-on") == 0) {
    send_cmd(cmd_led_on);
    printf("sent led-on\n");
  } else if (strcmp(argv[1], "stop") == 0) {
    send_cmd(cmd_stop);
    printf("sent stop\n");
  } else {
    fprintf(stderr, "unknown command: %s\n", argv[1]);
    return 1;
  }

  return 0;
}
