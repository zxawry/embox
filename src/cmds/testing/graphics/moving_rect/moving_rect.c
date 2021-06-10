/**
 * @file
 * @brief Draws red moving rectangle on the screen
 *
 * @date 10.06.2021
 * @author Alexander Kalmuk
 */

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <drivers/video/fb.h>

#define RECT_COLOR 0x0000ff /* red */
#define AREA_COLOR 0xffffff /* white */

#define RECT_MOVING_STEP 1

static uint32_t color_convert(struct fb_info *fb, uint32_t in) {
	uint32_t out;

	pix_fmt_convert(&in, &out, 1, RGB888, fb->var.fmt);

	return out;
}

static void moving_rect_draw(struct fb_info *fb) {
	struct fb_fillrect rect;

	rect.dx = 0;
	rect.dy = fb->var.yres / 3;
	rect.width = fb->var.xres / 8;
	rect.height = fb->var.yres / 3;
	rect.rop = ROP_COPY;

	while (1) {
		rect.color = color_convert(fb, AREA_COLOR);
		fb_fillrect(fb, &rect);

		rect.dx += RECT_MOVING_STEP;
		if (rect.dx + rect.width > fb->var.xres) {
			rect.dx = 0;
		}
		rect.color = color_convert(fb, RECT_COLOR);
		fb_fillrect(fb, &rect);

		usleep(10 * 1000);
	}
}

static struct fb_info *init_fb(void) {
	struct fb_fillrect rect;
	struct fb_info *fb;

	fb = fb_lookup(0);

	if (!fb) {
		fprintf(stderr, "Cannot open framebuffer\n");
		return NULL;
	}

	rect.dx = rect.dy = 0;
	rect.width = fb->var.xres;
	rect.height = fb->var.yres;
	rect.color = color_convert(fb, AREA_COLOR);
	rect.rop = ROP_COPY;

	fb_fillrect(fb, &rect);

	return fb;
}

static void print_usage(const char *cmd) {
	printf("Usage: %s [-h]\n", cmd);
}

int main(int argc, char **argv) {
	int opt;
	struct fb_info *fb;

	while (-1 != (opt = getopt(argc, argv, "h"))) {
		switch (opt) {
		case 'h':
			print_usage(argv[0]);
			/* FALLTHROUGH */
		default:
			return 0;
		}
	}

	fb = init_fb();
	if (!fb) {
		fprintf(stderr, "Framebuffer initialization error\n");

		return -1;
	}

	/* Testing for infinite time. */
	moving_rect_draw(fb);

	return 0;
}
