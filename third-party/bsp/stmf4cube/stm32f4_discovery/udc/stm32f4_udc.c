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

#define USB_MAX_EP0_SIZE 64U

#define EP0_BUFFER_SIZE 1024
#define STM32F4_UDC_EPS_COUNT 8 /* IN and OUT */

#define STM32F4_UDC_IN_EP_MASK   ((1 << 5) | (1 << 6) | (1 << 7))
#define STM32F4_UDC_OUT_EP_MASK  ((1 << 1) | (1 << 2) | (1 << 3))

struct USBD_EndpointTypeDef {
//  uint32_t status;
  uint32_t total_length;
  uint32_t rem_length;
  uint32_t maxpacket;
  uint8_t is_used;
//  uint16_t bInterval;
};

struct stm32f4_udc {
	struct usb_udc udc;
	struct usb_gadget_ep *eps[STM32F4_UDC_EPS_COUNT];
	struct usb_gadget_request *requests[STM32F4_UDC_EPS_COUNT];
	//unsigned int setup_buf_pos;
	//unsigned int tx_in_progress[STM32F4_UDC_EPS_COUNT];
	struct USBD_EndpointTypeDef ep_info[STM32F4_UDC_EPS_COUNT];
	uint32_t ep0_data_len;
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
		while (u->ep_info[0x4 | ep->nr].is_used) {
		}
		/* shouldnt this be able to handle requests over EP max length? */
		//pdev->ep0_state = USBD_EP0_DATA_IN;
		u->ep_info[0x4 | ep->nr].is_used = 1;
		u->ep_info[0x4 | ep->nr].total_length = req->len;
		u->ep_info[0x4 | ep->nr].rem_length = req->len;

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
		log_debug("addr=0x%x", dev_addr);

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

	log_debug("conf=0x%x", config);

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
		printk("usb:if\n");
		ret = usb_gadget_setup(stm32f4_udc.udc.composite,
			(const struct usb_control_header *) req, NULL);
		if (ret != 0) {
			printk("if_error\n");
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

  stm32f4_udc.ep0_data_len = ctrl.w_length;

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

/**
  * @brief  Data In stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint Number
  * @retval None
  */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	printk("usb: dataINstage of 0x%x\n", epnum);
/*	struct usb_gadget_request *req;

	stm32f4_udc.tx_in_progress[epnum] = 0;

	if (epnum == 0) {
		//hw_usb_ep_rx_enable(0);
		hpcd->IN_ep[epnum].xfer_buff

		HAL_PCD_EP_Transmit(hpcd, 0x00U, hcpd->IN_ep[epnum].xfer_buff, pep->rem_length);
		HAL_PCD_EP_Receive(hpcd, 0U, NULL, 0U);
		return;
	} else {
		req = stm32f4_udc.requests[epnum];
		assert(req);

		stm32f4_udc.requests[epnum] = NULL;

		log_debug("req complete, ep=%d", epnum);

		if (req->complete) {
			req->complete(stm32f4_udc.eps[epnum], req);
		}
	}
}
*/
	//USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
	//void USBD_LL_DataInStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata) {
  struct USBD_EndpointTypeDef *pep;

	if (epnum == 0U) {
		//pep = &pdev->ep_in[0];
		pep = &stm32f4_udc.ep_info[0x4 | epnum];

		if (pep->rem_length > pep->maxpacket) { //this will transmit in more than this pkt
			pep->rem_length -= pep->maxpacket;

			//(void)USBD_CtlContinueSendData(pdev, pdata, pep->rem_length);
			HAL_PCD_EP_Transmit(hpcd, 0U, hpcd->IN_ep[epnum].xfer_buff, pep->rem_length);
			/* Prepare endpoint for premature end of transfer */
		 //(void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
			HAL_PCD_EP_Receive(hpcd, 0U, NULL, 0U);
		} else { //this will end in this pkt
			/* last packet is MPS multiple, so send ZLP packet */
			if ((pep->maxpacket == pep->rem_length) &&
					(pep->total_length >= pep->maxpacket) &&
					(pep->total_length < stm32f4_udc.ep0_data_len)) {
				//(void)USBD_CtlContinueSendData(pdev, NULL, 0U);
				HAL_PCD_EP_Transmit(hpcd, 0U, NULL, 0U);
				stm32f4_udc.ep0_data_len = 0U;
				/* Prepare endpoint for premature end of transfer */
				//(void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
				HAL_PCD_EP_Receive(hpcd, 0U, NULL, 0U);
			} else {
				printk("datain:?\n");
				//if ((pdev->pClass->EP0_TxSent != NULL) &&
				//		(pdev->dev_state == USBD_STATE_CONFIGURED)) {
				//	pdev->pClass->EP0_TxSent(pdev);
				//}
				//(void)USBD_LL_StallEP(pdev, 0x80U);
				stm32f4_udc.ep_info[0x4 | epnum].is_used = 0;
				HAL_PCD_EP_SetStall(hpcd, 0x80U);
				//(void)USBD_CtlReceiveStatus(pdev);
					/* Set EP0 State */
					//pdev->ep0_state = USBD_EP0_STATUS_OUT;
					/* Start the transfer */
					//(void)USBD_LL_PrepareReceive(pdev, 0U, NULL, 0U);
					HAL_PCD_EP_Receive(hpcd, 0U, NULL, 0U);
			}
		}
	}
	/*  uncomment for EP!=0 later */
//  else if ((pdev->pClass->DataIn != NULL) &&
//           (pdev->dev_state == USBD_STATE_CONFIGURED))
//  {
//    (USBD_StatusTypeDef)pdev->pClass->DataIn(pdev, epnum);
//  }
//  else
//  {
//    /* should never be in this condition */
//    /* maybe add a log instead of return */
//  }
}

/**
 * @brief  Data Out stage callback.
 * @param  hpcd: PCD handle
 * @param  epnum: Endpoint Number
 * @retval None
 */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	//USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
	printk("usb: dataOUTstage\n");

//	if (epnum == 0U)
//	{
//		pep = &pdev->ep_out[0];
//
////		if (pdev->ep0_state == USBD_EP0_DATA_OUT)
////		{
//		if (pep->rem_length > pep->maxpacket)
//		{
//			pep->rem_length -= pep->maxpacket;
//
//			//(void)USBD_CtlContinueRx(pdev, pdata, MIN(pep->rem_length, pep->maxpacket));
//			//(void)USBD_LL_PrepareReceive(pdev, 0U, pbuf, len);
//			HAL_PCD_EP_Receive(hpcd, ep_addr, pbuf, size);
//		}
//		else
//		{
//			if ((pdev->pClass->EP0_RxReady != NULL) &&
//					(pdev->dev_state == USBD_STATE_CONFIGURED))
//			{
//				pdev->pClass->EP0_RxReady(pdev);
//			}
//			//(void)USBD_CtlSendStatus(pdev);
//			HAL_PCD_EP_Transmit(hpcd, 0x00U, NULL, 0U);
//		}
////		}
//	}
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
	/* Reset Device */
	printk("usb: reset\n");
/*	printk("ep0[OUT]_info:\n"
			"num:%d,"
			"in/out:%d,"
			"stall:%d,"
			"type:%d,"
			"pid:%d,"
			"tx_fifo:%d,"
			"maxpacket:%d,"
			"xfer_len:%d,"
			"xfer_cnt:%d\n",
			hpcd->OUT_ep[0].num,
			hpcd->OUT_ep[0].is_in,
			hpcd->OUT_ep[0].is_stall,
			hpcd->OUT_ep[0].type,
			hpcd->OUT_ep[0].data_pid_start,
			hpcd->OUT_ep[0].tx_fifo_num,
			hpcd->OUT_ep[0].maxpacket,
			hpcd->OUT_ep[0].xfer_len,
			hpcd->OUT_ep[0].xfer_count);

	printk("ep0[IN]_info:\n"
			"num:%d,"
			"in/out:%d,"
			"stall:%d,"
			"type:%d,"
			"pid:%d,"
			"tx_fifo:%d,"
			"maxpacket:%d,"
			"xfer_len:%d,"
			"xfer_cnt:%d\n",
			hpcd->IN_ep[0].num,
			hpcd->IN_ep[0].is_in,
			hpcd->IN_ep[0].is_stall,
			hpcd->IN_ep[0].type,
			hpcd->IN_ep[0].data_pid_start,
			hpcd->IN_ep[0].tx_fifo_num,
			hpcd->IN_ep[0].maxpacket,
			hpcd->IN_ep[0].xfer_len,
			hpcd->IN_ep[0].xfer_count);
			*/

	/*TODO: DeInit some stuff */

	/* Open EP0 OUT */
  //(void)USBD_LL_OpenEP(pdev, 0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  HAL_PCD_EP_Open(hpcd, 0x00U, USB_MAX_EP0_SIZE, EP_TYPE_CTRL); /* EP0_MAX_SIZE */
  //stm32f4_udc.ep_info[0x00U | 0x00U].is_used = 1U;

  stm32f4_udc.ep_info[0x00U | 0x00U].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  //(void)USBD_LL_OpenEP(pdev, 0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  HAL_PCD_EP_Open(hpcd, 0x80U, USB_MAX_EP0_SIZE, EP_TYPE_CTRL); /* EP0_MAX_SIZE */
  //stm32f4_udc.ep_info[0x04U | 0x00U].is_used = 1U;

  stm32f4_udc.ep_info[0x04U | 0x00U].maxpacket = USB_MAX_EP0_SIZE;
}
