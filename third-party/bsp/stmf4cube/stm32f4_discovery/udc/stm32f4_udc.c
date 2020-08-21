/**
 * @file
 * @brief
 *
 * @author  Erick Cafferata
 * @date    03.07.2020
 */

#include <assert.h>
#include <string.h>
#include <util/log.h>
#include <util/array.h>
#include <util/math.h>
#include <kernel/irq.h>
#include <embox/unit.h>
#include <drivers/usb/usb_defines.h>
#include <drivers/usb/gadget/udc.h>

EMBOX_UNIT_INIT(stm32f4_udc_init);

#define EP0_BUFFER_SIZE 1024
#define STM32F4_UDC_EPS_COUNT 7

#define STM32F4_UDC_IN_EP_MASK   ((1 << 1) | (1 << 3) | (1 << 5))
#define STM32F4_UDC_OUT_EP_MASK  ((1 << 2) | (1 << 4) | (1 << 6))

struct stm32f4_udc {
	struct usb_udc udc;
	struct usb_gadget_ep *eps[STM32F4_UDC_EPS_COUNT];
	struct usb_gadget_request *requests[STM32F4_UDC_EPS_COUNT];
	unsigned int setup_buf_pos;
};

static uint8_t ep0_buffer[EP0_BUFFER_SIZE];

static int stm32f4_udc_start(struct usb_udc *udc) {
	usb_stm32f4_init();

	return 0;
}

static int stm32f4_udc_ep_queue(struct usb_gadget_ep *ep,
	                        struct usb_gadget_request *req) {
	struct stm32f4_udc *u = (struct stm32f4_udc *) ep->udc;

	assert(ep && req);

	return 0;
}

static void stm32f4_udc_ep_enable(struct usb_gadget_ep *ep) {
	struct stm32f4_udc *u = (struct stm32f4_udc *) ep->udc;

	assert(ep);
}

static struct stm32f4_udc stm32f4_udc = {
	.udc = {
		.name = "stm32f4 udc",
		.udc_start = stm32f4_udc_start,
		.ep_queue = stm32f4_udc_ep_queue,
		.ep_enable = stm32f4_udc_ep_enable,

		.in_ep_mask = STM32F4_UDC_IN_EP_MASK,
		.out_ep_mask = STM32F4_UDC_OUT_EP_MASK,
	},
};

static int stm32f4_udc_init(void) {
	usb_gadget_register_udc(&stm32f4_udc.udc);

	return 0;
}
