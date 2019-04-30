/**
 * @file
 * @brief Simple test to draw frame via embox/fb interface
 *
 * @date Jun 21, 2017
 * @author Anton Bondarev
 * @author Denis Deryugin
 */

#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <util/math.h>

#include <drivers/video/fb.h>
#include <kernel/time/ktime.h>
#include <lib/fps.h>
#include <mem/vmem.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))

void dcache_flush(const void *p, size_t size);
static void fill_gradient(void *base, const struct fb_info *fb_info) {
	int x, y, width, height;
	int bytes_per_pixel;
	long int idx = 0;

	/* Figure out the size of the screen in bytes */
	bytes_per_pixel = fb_info->var.bits_per_pixel / 8;

	width = fb_info->var.xres;
	height = fb_info->var.yres;
	idx = fb_info->var.xoffset + fb_info->var.xres * fb_info->var.yoffset;

	for (y = 0; y < height; y++) {
		for (x = 0; x < width; x++) {
			if (bytes_per_pixel == 4) {
				((uint32_t *)base)[idx + x] = (0 << 24) |                    /* No transparency */
							((200 - (y - 100) / 5) << 16) | /* A lot of red */
							((15 + (x - 100) / 2) << 8) |   /* A little green*/
							(100 << 0);                     /* Some blue */
			} else { /* assume RGB565 */
				int b = MIN(0x1F, (1 + x + y) / ((width + height) / 0x1F));
				int g = MIN(0x3F, (x + 1) / (width / 0x3F));
				int r = MIN(0x1F, (1 + height - y) / (height / 0x1F));
				uint16_t t = ((r & 0x1F) << 11) |
						((g & 0x3F) << 5) | (b & 0x1F);
				((uint16_t *)base)[idx + x] = t;
			}
		}

		idx += fb_info->var.xres;
	}
}

static int simple_test(struct fb_info *fb_info) {
	int scr_sz = fb_info->var.xres * fb_info->var.yres * fb_info->var.bits_per_pixel / 8;
	vmem_set_flags(vmem_current_context(),
			(mmu_vaddr_t) fb_info->screen_base,
			scr_sz,
			VMEM_PAGE_WRITABLE);
	fill_gradient(fb_info->screen_base, fb_info);
	fps_set_format("Embox FB simple test");
	fps_print(fb_info);
	return 0;
}

static int swap_test(struct fb_info *fb_info) {
	uint8_t *base[2];
	int scr_sz = fb_info->var.xres * fb_info->var.yres * fb_info->var.bits_per_pixel / 8;

	base[0] = malloc(scr_sz);
	base[1] = malloc(scr_sz);

	assert(base[0] && base[1]);

	vmem_set_flags(vmem_current_context(),
			(mmu_vaddr_t) base[0],
			scr_sz,
			VMEM_PAGE_WRITABLE);

	vmem_set_flags(vmem_current_context(),
			(mmu_vaddr_t) base[1],
			scr_sz,
			VMEM_PAGE_WRITABLE);

	fill_gradient(base[0], fb_info);
	fill_gradient(base[1], fb_info);

	for (int i = 0; i < scr_sz; i++) {
		base[1][i] ^= 0xff;
	}

	fps_set_format("Embox FB swap test\nFPS=%d");
	fps_set_base_frame(fb_info, base[0]);
	fps_set_back_frame(fb_info, base[1]);

	for (int i = 0; i < 100000; i++) {
		fps_print(fb_info);
		fps_swap(fb_info);
	}

	free(base[0]);
	free(base[1]);

	return 0;
}

static void print_help(void) {
	printf("Options: [-t] for tearing test\n");
}

int main(int argc, char **argv) {
	struct fb_info *fb_info;
	int opt;
	int (*test_fn)(struct fb_info *) = simple_test;

	while (-1 != (opt = getopt(argc, argv, "th"))) {
		switch (opt) {
		case 'h':
			print_help();
			return 0;
		case 't':
			test_fn = swap_test;
			break;
		default:
			printf("Uknown option\n");
			print_help();
			return 0;
		}
	}

	fb_info = fb_lookup(0);

	printf("%dx%d, %dbpp\n", fb_info->var.xres, fb_info->var.yres, fb_info->var.bits_per_pixel);

	test_fn(fb_info);

	return 0;
}
