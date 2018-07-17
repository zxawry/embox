/**
 * @file imx-hdmi.c
 * @brief HDMI video driver for IWAVE RAINBOW G15S board
 * @author Denis Deryugin <deryugin.denis@gmail.com>
 * @version 0.1
 * @date 29.03.2017
 */

#include <drivers/common/memory.h>
#include <embox/unit.h>
#include <hal/reg.h>
#include <kernel/printk.h>
#include <util/log.h>

EMBOX_UNIT_INIT(imx_hdmi_init);

#define BASE_ADDR	OPTION_GET(NUMBER,base_addr)

#define HDMI_DESIGN_ID         (BASE_ADDR + 0x0000)
#define HDMI_REVISION_ID       (BASE_ADDR + 0x0001)
#define HDMI_PRODUCT_ID0       (BASE_ADDR + 0x0002)
#define HDMI_PRODUCT_ID1       (BASE_ADDR + 0x0003)
#define HDMI_CONFIG0_ID        (BASE_ADDR + 0x0004)
#define HDMI_CONFIG1_ID        (BASE_ADDR + 0x0005)
#define HDMI_CONFIG2_ID        (BASE_ADDR + 0x0006)

static int imx_hdmi_init(void) {
	uint8_t tmp;
	printk("\n\tInitialize i.MX6 HDMI controller...\n"
			"\t\tDesignID=%0x;RevisionID=%0x\n",
			REG8_LOAD(HDMI_DESIGN_ID),
			REG8_LOAD(HDMI_REVISION_ID));

	if (REG8_LOAD(HDMI_PRODUCT_ID0) != 0xA0) {
		log_error("Wrong HDMI_PRODUCT_ID0 value");
		return -1;
	}

	tmp = REG8_LOAD(HDMI_PRODUCT_ID1);
	if (tmp != 0x01 && tmp != 0xC1) {
		log_error("Wrong HDMI_PRODUCT_ID1 value\n");
	}

	tmp = REG8_LOAD(HDMI_CONFIG0_ID);
	printk("\tCheck supported features:\n");
	printk("\t\tInternal pixel repetition:    %s\n",
			tmp & (1 << 7) ? "yes" : "no");
	printk("\t\tHBR interface:                %s\n",
			tmp & (1 << 6) ? "yes" : "no");
	printk("\t\tSPDIF interface:              %s\n",
			tmp & (1 << 5) ? "yes" : "no");
	printk("\t\tI2S interface:                %s\n",
			tmp & (1 << 4) ? "yes" : "no");
	printk("\t\tHDMI 1.4:                     %s\n",
			tmp & (1 << 3) ? "yes" : "no");
	printk("\t\tColor Space Conversion block: %s\n",
			tmp & (1 << 2) ? "yes" : "no");
	printk("\t\tCEC:                          %s\n",
			tmp & (1 << 1) ? "yes" : "no");
	printk("\t\tHDCP:                         %s\n",
			tmp & (1 << 0) ? "yes" : "no");

	tmp = REG8_LOAD(HDMI_CONFIG1_ID);
	printk("\tConfiguration interface is %s\n",
		tmp & (1 << 0) ? "AHB" :
		tmp & (1 << 1) ? "APB" :
		tmp & (1 << 2) ? "OCP" :
		tmp & (1 << 3) ? "I2C" :
		tmp & (1 << 4) ? "SFR" : "unknown");

	tmp = REG8_LOAD(HDMI_CONFIG2_ID);
	printk("\tPHY interface type: %s\n",
			tmp == 0x00 ? "legacy PHY" :
			tmp == 0xF2 ? "HDMI 3D TX PHY" :
			tmp == 0xE2 ? "HDMI 3D TX PHY + HEAC PHY" : "unknown");

	return 0;
}
