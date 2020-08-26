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
#include <third_party/stmf4cube/usb_stm32f4.h>

/* FIX: add dependency */
#include <kernel/printk.h>

#include "stm32f4xx_hal.h"

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
	unsigned int tx_in_progress[STM32F4_UDC_EPS_COUNT];
};

extern PCD_HandleTypeDef hpcd;

static uint8_t ep0_buffer[EP0_BUFFER_SIZE];

static int stm32f4_udc_start(struct usb_udc *udc) {
	usb_stm32f4_init();

	return 0;
}

static int stm32f4_udc_ep_queue(struct usb_gadget_ep *ep,
	                        struct usb_gadget_request *req) {
	struct stm32f4_udc *u = (struct stm32f4_udc *) ep->udc;

	assert(ep && req);

	u->requests[ep->nr] = req;

	if (ep->nr == 0 || ep->dir == USB_DIR_IN) {
		/* It would be better to use queue here, put req in queue,
		 * then get next req from queue after current finished. */
		while (u->tx_in_progress[ep->nr]) {
		}
		u->tx_in_progress[ep->nr] = 1;
		HAL_PCD_EP_Transmit(&hpcd, ep->nr, req->buf, req->len);
	}

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

/* hardware-specific handlers */

static void stm32f4_ll_set_address(struct usb_control_header *req) {
	uint8_t  dev_addr;

	if ((req->w_index == 0U) && (req->w_length == 0U) && (req->w_value < 128U)) {
		dev_addr = (uint8_t)(req->w_value) & 0x7FU;

			HAL_PCD_SetAddress(&hpcd, dev_addr);
			//TODO: create function(CtlSendStatus)
			HAL_PCD_EP_Transmit(&hpcd, 0x00U, NULL, 0U);
	}
	else
	{
		HAL_PCD_EP_SetStall(&hpcd, 0x80U); //equivalent to CtlError()
		HAL_PCD_EP_SetStall(&hpcd, 0U);
	}
}

static void stm32f4_ll_set_configuration(struct usb_control_header *req) {
	int config = req->w_value & 0xff;

	log_debug("addr=0x%x", config);

	/*TODO: add check for config not found */
	usb_gadget_set_config(stm32f4_udc.udc.composite, config);

	HAL_PCD_EP_Transmit(&hpcd, 0x00U, NULL, 0U);
}

static void stm32f4_ll_get_status(struct usb_control_header *req) {
	uint16_t status = 0;

	switch (req->bm_request_type & USB_REQ_RECIP_MASK) {
	case USB_REQ_RECIP_DEVICE:
		/*TODO: add check for w_length != 2bytes */

		HAL_PCD_EP_Transmit(&hpcd, 0x00U, (uint8_t *) &status, 2U);

		break;
	/* TODO: add case for EPs recipient */
	/* case USB_REQ_RECIP_ENDP: */
	default:
		log_error("Unsupported RECIP 0x%x",
			req->bm_request_type & USB_REQ_RECIP_MASK);
		HAL_PCD_EP_SetStall(&hpcd, req->bm_request_type & 0x80U);
		break;
	}
}

/**
* @brief  stm32f4_ll_handle_standard_request
*         Handle USB device requests
* @param  req: USB request
* @retval status
*/
static void stm32f4_ll_handle_standard_request(struct usb_control_header *req) {
	int ret;

	switch (req->b_request) {
	case USB_REQ_SET_ADDRESS:
		stm32f4_ll_set_address(req);
		break;
	case USB_REQ_SET_CONFIG:
		stm32f4_ll_set_configuration(req);
		break;
	case USB_REQ_GET_CONFIG:
		//stm32f4_ll_get_configuration(req);
		log_debug("GET_CONFIGURATION");
		break;
	case USB_REQ_GET_STATUS:
		stm32f4_ll_get_status(req);
		break;
	case USB_REQ_SET_FEATURE:
		//stm32f4_ll_set_feature(req);
		log_debug("SET_FEATURE");
		break;
	case USB_REQ_CLEAR_FEATURE:
		//stm32f4_ll_clear_feature(req);
		log_debug("CLEAR_FEATURE");
		break;
	default:
		ret = usb_gadget_setup(stm32f4_udc.udc.composite,
			(const struct usb_control_header *) req, NULL);
		if (ret != 0) {
			log_error("Not implemented req 0x%x", req->b_request);
			HAL_PCD_EP_SetStall(&hpcd, req->bm_request_type & 0x80U);
		}
		break;
	}
}

/**
 * @brief  Setup stage callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
	//TODO: nice to have a gadget state struct to contain all the info
	struct usb_gadget_request *req;
	int ret;

	/* struct usb_control_header *ctrl = (struct usb_control_header *)hpcd->Setup; */
	struct usb_control_header ctrl;
	memcpy(&ctrl, hpcd->Setup, sizeof(struct usb_control_header));

	printk("usb: setupstage\n");
	printk("bmRequestType:0x%x\nbRequest:0x%x\nwValue:0x%x\nwIndex:0x%x\nwLength:0x%x\n",
			ctrl.bm_request_type, ctrl.b_request, ctrl.w_value, ctrl.w_index, ctrl.w_length);
			//ctrl->bm_request_type, ctrl->b_request, ctrl->w_value, ctrl->w_index, ctrl->w_length);

	switch (ctrl.bm_request_type & USB_REQ_TYPE_MASK) {
	case USB_REQ_TYPE_STANDARD:
		stm32f4_ll_handle_standard_request(&ctrl);
		break;
	default:
		ret = usb_gadget_setup(stm32f4_udc.udc.composite,
			(const struct usb_control_header *) &ctrl, ep0_buffer);
		if (ret != 0) {
			log_error("Setup failed, request=0x%x", ctrl.b_request);
			HAL_PCD_EP_SetStall(hpcd, ctrl.bm_request_type & 0x80U);
		}
		break;
	}
}
