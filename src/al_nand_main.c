/*
 * Annapurna Labs Nand driver.
 *
 * Copyright (C) 2013 Annapurna Labs Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * TODO:
 * - add sysfs statistics
 * - use dma for reading writing
 * - get config parameters from device tree instead of config registers
 * - use correct ECC size and not entire OOB
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>

#include "al_hal_nand.h"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Annapurna Labs");

#define WAIT_EMPTY_CMD_FIFO_TIME_OUT 1000000
#define AL_NAND_NAME "al-nand"
#define AL_NAND_MAX_ONFI_TIMING_MODE 1

#define AL_NAND_MAX_BIT_FLIPS 4
#define AL_NAND_MAX_OOB_SIZE SZ_1K

#define NAND_SET_FEATURES_ADDR 0xfa

#define ONFI_COL_ADDR_CYCLE_MASK 0xf0
#define ONFI_COL_ADDR_CYCLE_POS  4
#define ONFI_ROW_ADDR_CYCLE_MASK 0x0f
#define ONFI_ROW_ADDR_CYCLE_POS  0

#define AL_NAND_MAX_CHIPS 1

struct nand_data {
	struct nand_chip chip;
	struct platform_device *pdev;

	struct al_nand_ctrl_obj nand_obj;
	uint8_t word_cache[4];
	int cache_pos;
	struct al_nand_ecc_config ecc_config;
	struct al_nand_dev_properties dev_props;
	struct al_nand_extra_dev_properties dev_ext_props;

	/*** interrupts ***/
	struct completion complete;
	uint32_t irq_status;
	int irq;

	uint8_t oob[AL_NAND_MAX_OOB_SIZE];
};

/*
 * Addressing RMN: 2903
 *
 * RMN description:
 * NAND timing parameters that are used in the non-manual mode are wrong and
 * reduce performance.
 * Replacing with the manual parameters to increase speed
 */
static unsigned long clk_freq_khz = 375000;
#define NAND_CLK_CYCLES(nsec) ((nsec) * clk_freq_khz / 1000000)

static void al_nand_timing_params_set(struct al_nand_device_timing *timing, uint8_t mode)
{
	switch (mode) {
	case 0:
		timing->tSETUP = NAND_CLK_CYCLES(14);
		timing->tHOLD = NAND_CLK_CYCLES(22);
		timing->tWRP = NAND_CLK_CYCLES(54);
		timing->tRR = NAND_CLK_CYCLES(43);
		timing->tWB = NAND_CLK_CYCLES(206);
		timing->tWH = NAND_CLK_CYCLES(32);
		timing->tINTCMD = NAND_CLK_CYCLES(86);
		timing->readDelay = NAND_CLK_CYCLES(3);
		break;
	case 1:
		timing->tSETUP = NAND_CLK_CYCLES(14);
		timing->tHOLD = NAND_CLK_CYCLES(14);
		timing->tWRP = NAND_CLK_CYCLES(27);
		timing->tRR = NAND_CLK_CYCLES(22);
		timing->tWB = NAND_CLK_CYCLES(104);
		timing->tWH = NAND_CLK_CYCLES(19);
		timing->tINTCMD = NAND_CLK_CYCLES(43);
		timing->readDelay = NAND_CLK_CYCLES(3);
		break;
	case 2:
		timing->tSETUP = NAND_CLK_CYCLES(14);
		timing->tHOLD = NAND_CLK_CYCLES(14);
		timing->tWRP = NAND_CLK_CYCLES(19);
		timing->tRR = NAND_CLK_CYCLES(22);
		timing->tWB = NAND_CLK_CYCLES(104);
		timing->tWH = NAND_CLK_CYCLES(16);
		timing->tINTCMD = NAND_CLK_CYCLES(43);
		timing->readDelay = NAND_CLK_CYCLES(3);
		break;
	case 3:
		timing->tSETUP = NAND_CLK_CYCLES(14);
		timing->tHOLD = NAND_CLK_CYCLES(14);
		timing->tWRP = NAND_CLK_CYCLES(16);
		timing->tRR = NAND_CLK_CYCLES(22);
		timing->tWB = NAND_CLK_CYCLES(104);
		timing->tWH = NAND_CLK_CYCLES(11);
		timing->tINTCMD = NAND_CLK_CYCLES(27);
		timing->readDelay = NAND_CLK_CYCLES(3);
		break;
	case 4:
		timing->tSETUP = NAND_CLK_CYCLES(14);
		timing->tHOLD = NAND_CLK_CYCLES(14);
		timing->tWRP = NAND_CLK_CYCLES(14);
		timing->tRR = NAND_CLK_CYCLES(22);
		timing->tWB = NAND_CLK_CYCLES(104);
		timing->tWH = NAND_CLK_CYCLES(11);
		timing->tINTCMD = NAND_CLK_CYCLES(27);
		timing->readDelay = NAND_CLK_CYCLES(3);
		break;
	case 5:
		timing->tSETUP = NAND_CLK_CYCLES(14);
		timing->tHOLD = NAND_CLK_CYCLES(14);
		timing->tWRP = NAND_CLK_CYCLES(14);
		timing->tRR = NAND_CLK_CYCLES(22);
		timing->tWB = NAND_CLK_CYCLES(104);
		timing->tWH = NAND_CLK_CYCLES(11);
		timing->tINTCMD = NAND_CLK_CYCLES(27);
		timing->readDelay = NAND_CLK_CYCLES(3);
		break;
	default:
		pr_err("%s: Invalid timing mode %d", __func__, mode);
		BUG();
	}
}

static uint32_t wait_for_irq(struct nand_data *nand, uint32_t irq_mask);

static void nand_cw_size_get(
			int		num_bytes,
			uint32_t	*cw_size,
			uint32_t	*cw_count)
{
	num_bytes = AL_ALIGN_UP(num_bytes, 4);

	if (num_bytes < *cw_size)
		*cw_size = num_bytes;

	if (0 != (num_bytes % *cw_size))
		*cw_size = num_bytes / 4;

	BUG_ON(num_bytes % *cw_size);

	*cw_count = num_bytes / *cw_size;
}

static void nand_send_byte_count_command(
			struct al_nand_ctrl_obj		*nand_obj,
			enum al_nand_command_type	cmd_id,
			uint16_t			len)
{
	uint32_t cmd;

	cmd = AL_NAND_CMD_SEQ_ENTRY(
			cmd_id,
			(len & 0xff));

	al_nand_cmd_single_execute(nand_obj, cmd);

	cmd = AL_NAND_CMD_SEQ_ENTRY(
			cmd_id,
			((len & 0xff00) >> 8));

	al_nand_cmd_single_execute(nand_obj, cmd);
}

static void nand_wait_cmd_fifo_empty(
			struct nand_data	*nand)
{
	int cmd_buff_empty;
	uint32_t i = WAIT_EMPTY_CMD_FIFO_TIME_OUT;

	while (i > 0) {
		cmd_buff_empty = al_nand_cmd_buff_is_empty(&nand->nand_obj);
		if (cmd_buff_empty)
			break;

		udelay(1);
		i--;
	}

	if (i == 0)
		pr_err("Wait for empty cmd fifo for more than a sec!\n");
}

void nand_cmd_ctrl(struct nand_chip *chip, int dat, unsigned int ctrl)
{
	uint32_t cmd;
	enum al_nand_command_type type;
	struct nand_data *nand = nand_get_controller_data(chip);

	if ((ctrl & (NAND_CLE | NAND_ALE)) == 0)
		return;

	nand->cache_pos = -1;

	type = ((ctrl & NAND_CTRL_CLE) == NAND_CTRL_CLE) ?
					AL_NAND_COMMAND_TYPE_CMD :
					AL_NAND_COMMAND_TYPE_ADDRESS;

	cmd = AL_NAND_CMD_SEQ_ENTRY(type, (dat & 0xff));
	dev_dbg(&nand->pdev->dev, "nand_cmd_ctrl: dat=0x%x, ctrl=0x%x, cmd=0x%x\n",
							dat, ctrl, cmd);

	al_nand_cmd_single_execute(&nand->nand_obj, cmd);

	nand_wait_cmd_fifo_empty(nand);

	if ((dat == NAND_CMD_PAGEPROG) && (ctrl & NAND_CLE)) {
		cmd = AL_NAND_CMD_SEQ_ENTRY(
				AL_NAND_COMMAND_TYPE_WAIT_FOR_READY,
				0);

		dev_dbg(&nand->pdev->dev, "%s: pagepro. send cmd = 0x%x\n",
				__func__, cmd);
		al_nand_cmd_single_execute(&nand->nand_obj, cmd);

		nand_wait_cmd_fifo_empty(nand);

		al_nand_wp_set_enable(&nand->nand_obj, 1);
		al_nand_tx_set_enable(&nand->nand_obj, 0);
	}
}

void nand_dev_select(struct nand_chip *chip, int chipnr)
{
	struct nand_data *nand = nand_get_controller_data(chip);

	if (chipnr < 0)
		return;

	al_nand_dev_select(&nand->nand_obj, chipnr);

	dev_dbg(&nand->pdev->dev, "nand_dev_select: chipnr = %d\n", chipnr);
}

int nand_dev_ready(struct nand_chip *chip)
{
	struct nand_data *nand = nand_get_controller_data(chip);
	int is_ready = 0;

	is_ready = al_nand_dev_is_ready(&nand->nand_obj);

	dev_dbg(&nand->pdev->dev, "nand_dev_ready: ready = %d\n", is_ready);

	return is_ready;
}

/*
 * read len bytes from the nand device.
 */
void nand_read_buff(struct nand_chip *chip, uint8_t *buf, int len)
{
	struct nand_data *nand = nand_get_controller_data(chip);
	uint32_t cw_size;
	uint32_t cw_count;
	uint32_t intr_status;
	void __iomem *data_buff;

	dev_dbg(&nand->pdev->dev, "nand_read_buff: read len = %d\n", len);

	cw_size = chip->ecc.size;

	BUG_ON(len & 3);
	BUG_ON(nand->cache_pos != -1);

	nand_cw_size_get(len, &cw_size, &cw_count);

	al_nand_cw_config(
			&nand->nand_obj,
			cw_size,
			cw_count);

	while (cw_count--)
		nand_send_byte_count_command(&nand->nand_obj,
				AL_NAND_COMMAND_TYPE_DATA_READ_COUNT,
				cw_size);
	while (len > 0) {
		intr_status = wait_for_irq(nand, AL_NAND_INTR_STATUS_BUF_RDRDY);

		data_buff = al_nand_data_buff_base_get(&nand->nand_obj);
		memcpy(buf, data_buff, cw_size);
		buf += cw_size;
		len -= cw_size;
	}
}

/*
 * read byte from the device.
 * read byte is not supported by the controller so this function reads
 * 4 bytes as a cache and use it in the next calls.
 */
uint8_t nand_read_byte_from_fifo(struct nand_chip *chip)
{
	uint8_t ret_val;
	struct nand_data *nand = nand_get_controller_data(chip);

	dev_dbg(&nand->pdev->dev,
			"%s: cache_pos = %d", __func__, nand->cache_pos);

	if (nand->cache_pos == -1) {
		nand_read_buff(chip, nand->word_cache, 4);
		nand->cache_pos = 0;
	}

	ret_val = nand->word_cache[nand->cache_pos];
	nand->cache_pos++;
	if (nand->cache_pos == 4)
		nand->cache_pos = -1;

	dev_dbg(&nand->pdev->dev, "%s: return = 0x%x\n", __func__, ret_val);
	return ret_val;
}
/*
 * writing buffer to the nand device.
 * this func will wait for the write to be complete
 */
void nand_write_buff(struct nand_chip *chip, const uint8_t *buf, int len)
{
	uint32_t cw_size = chip->ecc.size;
	uint32_t cw_count;
	struct nand_data *nand = nand_get_controller_data(chip);
	void __iomem *data_buff;

	dev_dbg(&nand->pdev->dev, "nand_write_buff: len = %d start: 0x%x%x%x\n",
			len, buf[0], buf[1], buf[2]);

	al_nand_tx_set_enable(&nand->nand_obj, 1);
	al_nand_wp_set_enable(&nand->nand_obj, 0);

	nand_cw_size_get(len, &cw_size, &cw_count);

	al_nand_cw_config(
			&nand->nand_obj,
			cw_size,
			cw_count);

	while (cw_count--)
		nand_send_byte_count_command(&nand->nand_obj,
				AL_NAND_COMMAND_TYPE_DATA_WRITE_COUNT,
				cw_size);


	while (len > 0) {
		wait_for_irq(nand, AL_NAND_INTR_STATUS_BUF_WRRDY);

		data_buff = al_nand_data_buff_base_get(&nand->nand_obj);
		memcpy(data_buff, buf, cw_size);

		buf += cw_size;
		len -= cw_size;
	}

	/* enable wp and disable tx will be executed after commands
	 * NAND_CMD_PAGEPROG and AL_NAND_COMMAND_TYPE_WAIT_FOR_READY will be
	 * sent to make sure all data were written.
	 */
}

/******************************************************************************/
/**************************** ecc functions ***********************************/
/******************************************************************************/
static inline int is_empty_oob(uint8_t *oob, int len)
{
	int flips = 0;
	int i;
	int j;

	for (i = 0; i < len; i++) {
		if (oob[i] == 0xff)
			continue;

		for (j = 0; j < 8; j++) {
			if ((oob[i] & BIT(j)) == 0) {
				flips++;
				if (flips >= AL_NAND_MAX_BIT_FLIPS)
					break;
			}
		}
	}

	if (flips < AL_NAND_MAX_BIT_FLIPS)
		return 1;

	return 0;
}
/*
 * read page with HW ecc support (corrected and uncorrected stat will be
 * updated).
 */
int ecc_read_page(struct nand_chip *chip,
		  uint8_t *buf, int oob_required, int page)
{
	struct nand_data *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct mtd_oob_region ecc_region;
	int uncorr_err_count = 0;
	int corr_err_count = 0;

	dev_dbg(&nand->pdev->dev, "ecc_read_page: read page %d\n", page);

	mtd_ooblayout_ecc(mtd, 0, &ecc_region);

	/* Clear TX/RX ECC state machine */
	al_nand_tx_set_enable(&nand->nand_obj, 1);
	al_nand_tx_set_enable(&nand->nand_obj, 0);

	al_nand_uncorr_err_clear(&nand->nand_obj);
	al_nand_corr_err_clear(&nand->nand_obj);

	al_nand_ecc_set_enabled(&nand->nand_obj, 1);

	BUG_ON(oob_required);

	/* First need to read the OOB to the controller to calc the ecc */
	chip->legacy.cmdfunc(chip, NAND_CMD_READOOB,
			     ecc_region.offset, page);

	nand_send_byte_count_command(&nand->nand_obj,
				AL_NAND_COMMAND_TYPE_SPARE_READ_COUNT,
				ecc_region.length);

	/* move to the start of the page to read the data */
	chip->legacy.cmdfunc(chip, NAND_CMD_RNDOUT, 0x00, -1);

	/* read the buffer (after ecc correction) */
	chip->legacy.read_buf(chip, buf, mtd->writesize);

	uncorr_err_count = al_nand_uncorr_err_get(&nand->nand_obj);
	corr_err_count = al_nand_corr_err_get(&nand->nand_obj);

	al_nand_ecc_set_enabled(&nand->nand_obj, 0);

	/* update statistics*/
	if (0 != uncorr_err_count) {
		bool uncorr_err = true;
		if (nand->ecc_config.algorithm == AL_NAND_ECC_ALGORITHM_BCH) {
			/* the ECC in BCH algorithm will find an uncorrected
			 * errors while trying to read an empty page.
			 * to avoid error messages and failures in the upper
			 * layer, don't update the statistics in this case */
			chip->legacy.read_buf(chip, nand->oob, mtd->oobsize);

			if (is_empty_oob(nand->oob, mtd->oobsize)){
				uncorr_err = false;
				memset(buf, 0xff, mtd->writesize);
			}
		}

		if (uncorr_err) {
			mtd->ecc_stats.failed++;
			pr_err("uncorrected errors found in page %d! (increased to %d)\n",
				page, mtd->ecc_stats.failed);
		}
	}

	if (0 != corr_err_count) {
		mtd->ecc_stats.corrected++;
		dev_dbg(&nand->pdev->dev, "ecc_read_page: corrected increased to %d\n",
						mtd->ecc_stats.corrected);
	}

	dev_dbg(&nand->pdev->dev, "ecc_read_page: corrected = %d\n",
					mtd->ecc_stats.corrected);

	return 0;
}

int ecc_read_subpage(struct nand_chip *chip,
		     uint32_t offs, uint32_t len, uint8_t *buf, int page)
{
	pr_err("ERROR: read subpage not supported!\n");
	return -1;
}
/*
 * program page with HW ecc support.
 * this function is called after the commands and adderess for this page sent.
 */
int ecc_write_page(struct nand_chip *chip,
		   const uint8_t *buf, int oob_required, int page)
{
	struct nand_data *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct mtd_oob_region ecc_region;
	uint32_t cmd;

	dev_dbg(&nand->pdev->dev, "ecc_write_page\n");

	BUG_ON(oob_required);

	mtd_ooblayout_ecc(mtd, 0, &ecc_region);

	nand_prog_page_begin_op(chip, page, 0, NULL, 0);

	al_nand_ecc_set_enabled(&nand->nand_obj, 1);

	nand_write_buff(chip, buf, mtd->writesize);

	chip->legacy.cmdfunc(chip, NAND_CMD_RNDIN,
			     mtd->writesize + ecc_region.offset, -1);

	cmd = AL_NAND_CMD_SEQ_ENTRY(
			AL_NAND_COMMAND_TYPE_WAIT_CYCLE_COUNT,
			0);

	al_nand_tx_set_enable(&nand->nand_obj, 1);
	al_nand_wp_set_enable(&nand->nand_obj, 0);

	al_nand_cmd_single_execute(&nand->nand_obj, cmd);

	dev_dbg(&nand->pdev->dev, "%s: spare bytes: %d\n", __func__,
		ecc_region.length);
	nand_send_byte_count_command(&nand->nand_obj,
				AL_NAND_COMMAND_TYPE_SPARE_WRITE_COUNT,
				ecc_region.length);

	nand_wait_cmd_fifo_empty(nand);

	al_nand_wp_set_enable(&nand->nand_obj, 1);
	al_nand_tx_set_enable(&nand->nand_obj, 0);

	al_nand_ecc_set_enabled(&nand->nand_obj, 0);

	return nand_prog_page_end_op(chip);
}

/******************************************************************************/
/****************************** interrupts ************************************/
/******************************************************************************/
static irqreturn_t al_nand_isr(int irq, void *dev_id);

static void nand_interrupt_init(struct nand_data *nand)
{
	int ret;

	init_completion(&nand->complete);
	nand->irq_status = 0;
	al_nand_int_disable(&nand->nand_obj, 0xffff);

	ret = devm_request_irq(&nand->pdev->dev, nand->irq, al_nand_isr,
			       IRQF_SHARED, AL_NAND_NAME, nand);
	if (ret)
		pr_info("failed to request irq. rc %d\n", ret);
}
/*
 * ISR for nand interrupts - save the interrupt status, disable this interrupts
 * and nodify the waiting proccess.
 */
static irqreturn_t al_nand_isr(int irq, void *dev_id)
{
	struct nand_data *nand = dev_id;
	uint32_t irq_status = 0x0;

	irq_status = al_nand_int_status_get(&nand->nand_obj);

	al_nand_int_disable(&nand->nand_obj, irq_status);
	complete(&nand->complete);

	return IRQ_HANDLED;

}

/*
 * Waiting for interrupt with status in irq_mask to occur.
 * return 0 when reach timeout of 1 sec.
 */
static uint32_t wait_for_irq(struct nand_data *nand, uint32_t irq_mask)
{
	unsigned long comp_res = 0;
	unsigned long timeout = msecs_to_jiffies(1000);

	al_nand_int_enable(&nand->nand_obj, irq_mask);
	comp_res = wait_for_completion_timeout(&nand->complete, timeout);

	if (comp_res == 0) {
		/* timeout */
		pr_err("timeout occurred, mask = 0x%x\n", irq_mask);

		return -EINVAL;
	}
	return 0;

}

/******************************************************************************/
/**************************** configuration ***********************************/
/******************************************************************************/
static enum al_nand_ecc_bch_num_corr_bits bch_num_bits_convert(
							unsigned int bits)
{
	switch (bits) {
	case 4:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_4;
	case 8:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_8;
	case 12:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_12;
	case 16:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_16;
	case 20:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_20;
	case 24:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_24;
	case 28:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_28;
	case 32:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_32;
	case 36:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_36;
	case 40:
		return AL_NAND_ECC_BCH_NUM_CORR_BITS_40;
	default:
		BUG();
	}

	return AL_NAND_ECC_BCH_NUM_CORR_BITS_8;
}

static enum al_nand_device_page_size page_size_bytes_convert(
							unsigned int bytes)
{
	switch (bytes) {
	case 2048:
		return AL_NAND_DEVICE_PAGE_SIZE_2K;
	case 4096:
		return AL_NAND_DEVICE_PAGE_SIZE_4K;
	case 8192:
		return AL_NAND_DEVICE_PAGE_SIZE_8K;
	case 16384:
		return AL_NAND_DEVICE_PAGE_SIZE_16K;
	default:
		BUG();
	}

	return AL_NAND_DEVICE_PAGE_SIZE_4K;
}

static void nand_set_timing_mode(
			struct nand_data *nand,
			enum al_nand_device_timing_mode timing)
{
	uint32_t cmds[] = {
		AL_NAND_CMD_SEQ_ENTRY(
			AL_NAND_COMMAND_TYPE_CMD, NAND_CMD_SET_FEATURES),
		AL_NAND_CMD_SEQ_ENTRY(
			AL_NAND_COMMAND_TYPE_ADDRESS, NAND_SET_FEATURES_ADDR),
		AL_NAND_CMD_SEQ_ENTRY(
			AL_NAND_COMMAND_TYPE_STATUS_WRITE, timing),
		AL_NAND_CMD_SEQ_ENTRY(
			AL_NAND_COMMAND_TYPE_STATUS_WRITE, 0x00),
		AL_NAND_CMD_SEQ_ENTRY(
			AL_NAND_COMMAND_TYPE_STATUS_WRITE, 0x00),
		AL_NAND_CMD_SEQ_ENTRY(
			AL_NAND_COMMAND_TYPE_STATUS_WRITE, 0x00)};

	al_nand_cmd_seq_execute(&nand->nand_obj, cmds, ARRAY_SIZE(cmds));

	nand_wait_cmd_fifo_empty(nand);
}

static int nand_resources_get_and_map(
				struct platform_device *pdev,
				void __iomem **nand_base,
				void __iomem **pbs_base)
{
	struct device_node *np;

	np = of_find_compatible_node(
			NULL, NULL, "al,alpine-nand");

	*nand_base = of_iomap(np, 0);
	if (!(*nand_base)) {
		pr_err("%s: failed to map nand memory\n", __func__);
		return -ENOMEM;
	}

	np = of_find_compatible_node(
			NULL, NULL, "al,alpine-pbs");

	*pbs_base = of_iomap(np, 0);
	if (!(*pbs_base)) {
		pr_err("%s: pbs_base map failed\n", __func__);
		return -ENOMEM;
	}

	return 0;

}

static void nand_onfi_config_set(
		struct nand_chip *chip,
		struct al_nand_dev_properties *device_properties,
		struct al_nand_ecc_config *ecc_config)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct onfi_params *onfi = chip->parameters.onfi;
	enum al_nand_device_page_size onfi_page_size;
	int i;
	uint16_t async_timing_mode = onfi->async_timing_mode;

	/* Addressing RMN: 2903 */
	device_properties->timingMode = AL_NAND_DEVICE_TIMING_MODE_MANUAL;

	/*
	 * If the device contains async timing mode information, use it to find the max timing mode
	 * supported by both the device and AL_NAND_MAX_ONFI_TIMING_MODE. Otherwise (non ONFI mode)
	 * use AL_NAND_MAX_ONFI_TIMING_MODE.
	 */
	for (i = AL_NAND_MAX_ONFI_TIMING_MODE;
		async_timing_mode && i && (!((1 << i) & async_timing_mode)); i--)
		;
	al_nand_timing_params_set(&device_properties->timing, i);

	nand_set_timing_mode(chip->priv, device_properties->timingMode);

	device_properties->num_col_cyc = chip->page_shift;
	device_properties->num_row_cyc = chip->chip_shift;
	onfi_page_size = page_size_bytes_convert(mtd->writesize);
	device_properties->pageSize = onfi_page_size;

	if (chip->ecc.strength == 1) {
		ecc_config->algorithm = AL_NAND_ECC_ALGORITHM_HAMMING;
	} else if (chip->ecc.strength > 1) {
		ecc_config->algorithm = AL_NAND_ECC_ALGORITHM_BCH;
		ecc_config->num_corr_bits =
			bch_num_bits_convert(chip->ecc.strength);
	}
}

static int al_nand_ooblayout_ecc(struct mtd_info *mtd, int section,
				 struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_data *nand = nand_get_controller_data(chip);
	int ecc_loc;

	if (section > 0)
		return -ERANGE;

	ecc_loc = nand->ecc_config.spareAreaOffset -
		nand->dev_ext_props.pageSize;

	oobregion->offset = ecc_loc;
	oobregion->length = mtd->oobsize - ecc_loc;

	return 0;
}

static int al_nand_ooblayout_free(struct mtd_info *mtd, int section,
				  struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_data *nand = nand_get_controller_data(chip);
	int ecc_loc;

	if (section > 0)
		return -ERANGE;

	ecc_loc = nand->ecc_config.spareAreaOffset -
		nand->dev_ext_props.pageSize;

	oobregion->offset = 2;
	oobregion->length = ecc_loc - 2;

	return 0;
}

static const struct mtd_ooblayout_ops al_nand_ooblayout_ops = {
	.ecc = al_nand_ooblayout_ecc,
	.free = al_nand_ooblayout_free,
};

static int al_nand_attach_chip(struct nand_chip *chip)
{
	struct nand_data *nand = nand_get_controller_data(chip);
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	const struct nand_ecc_props *requirements =
		nanddev_get_ecc_requirements(&chip->base);

	if (nand->dev_ext_props.eccIsEnabled != 0) {
		ecc->engine_type = NAND_ECC_ENGINE_TYPE_ON_HOST;
		ecc->placement = NAND_ECC_PLACEMENT_INTERLEAVED;
		ecc->size = mtd->writesize;
		ecc->strength = requirements->strength;
		ecc->bytes = mtd->oobsize;

		mtd_set_ooblayout(mtd, &al_nand_ooblayout_ops);

		ecc->read_page = ecc_read_page;
		ecc->read_subpage = ecc_read_subpage;
		ecc->write_page = ecc_write_page;
	} else {
		ecc->engine_type = NAND_ECC_ENGINE_TYPE_NONE;
		ecc->size = 512 << nand->ecc_config.messageSize;
	}

	nand_onfi_config_set(chip, &nand->dev_props, &nand->ecc_config);

	return 0;
}

static const struct nand_controller_ops al_nand_controller_ops = {
	.attach_chip = al_nand_attach_chip,
};

static int al_nand_probe(struct platform_device *pdev)
{
	struct mtd_info *mtd;
	struct nand_chip *nand;
	struct nand_data *nand_dat;
	int ret = 0;
	void __iomem *nand_base;
	void __iomem *pbs_base;
	struct clk *clk;

	nand_dat = kzalloc(sizeof(struct nand_data), GFP_KERNEL);
	if (nand_dat == NULL) {
		pr_err("Failed to allocate nand_data!\n");
		return -1;
	}

	pr_info("%s: AnnapurnaLabs nand driver\n", __func__);

	nand = &nand_dat->chip;
	mtd = nand_to_mtd(nand);

	nand_set_controller_data(nand, nand_dat);
	nand_set_flash_node(nand, pdev->dev.of_node);
	mtd->dev.parent = &pdev->dev;

	nand_dat->cache_pos = -1;
	nand_dat->pdev = pdev;

	dev_set_drvdata(&pdev->dev, nand_dat);

	ret = nand_resources_get_and_map(pdev, &nand_base, &pbs_base);
	if (ret != 0) {
		pr_err("%s: nand_resources_get_and_map failed\n", __func__);
		goto err;
	}

	ret = al_nand_init(&nand_dat->nand_obj,	nand_base);
	if (ret != 0) {
		pr_err("nand init failed\n");
		goto err;
	}

	if (0 != al_nand_dev_config_basic(&nand_dat->nand_obj)) {
		pr_err("dev_config_basic failed\n");
		ret = -EIO;
		goto err;
	}

	nand_dat->irq = platform_get_irq(pdev, 0);
	if (nand_dat->irq < 0) {
		pr_err("%s: no irq defined\n", __func__);
		return -ENXIO;
	}
	nand_interrupt_init(nand_dat);

	clk = clk_get(&pdev->dev, NULL);

	if (IS_ERR(clk))
		dev_dbg(&pdev->dev, "%s: clock doesn't appear in the device tree."
				    "setting to default %ld", __func__, clk_freq_khz);
	else
		clk_freq_khz = clk_get_rate(clk) / 1000;

	nand->options = NAND_NO_SUBPAGE_WRITE;

	nand->legacy.cmd_ctrl = nand_cmd_ctrl;
	nand->legacy.read_byte = nand_read_byte_from_fifo;
	nand->legacy.read_buf = nand_read_buff;
	nand->legacy.dev_ready = nand_dev_ready;
	nand->legacy.write_buf = nand_write_buff;
	nand->legacy.select_chip = nand_dev_select;

	if (0 != al_nand_properties_decode(
					pbs_base,
					&nand_dat->dev_props,
					&nand_dat->ecc_config,
					&nand_dat->dev_ext_props)) {
		pr_err("%s: nand_properties_decode failed\n", __func__);
		ret = -EIO;
		goto err;
	}

	nand->legacy.dummy_controller.ops = &al_nand_controller_ops;
	ret = nand_scan(nand, 1);
	if (ret) {
		pr_err("%s: nand_scan failed\n", __func__);
		goto err;
	}

	BUG_ON(mtd->oobsize > AL_NAND_MAX_OOB_SIZE);

	if (0 != al_nand_dev_config(
				&nand_dat->nand_obj,
				&nand_dat->dev_props,
				&nand_dat->ecc_config)) {
		pr_err("dev_config failed\n");
		ret = -EIO;
		goto err;
	}

	mtd->name = AL_NAND_NAME;

	mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);

	return 0;

err:
	kfree(nand_dat);
	return ret;
}

static int al_nand_remove(struct platform_device *pdev)
{
	struct nand_data *nand_dat = dev_get_drvdata(&pdev->dev);

	dev_dbg(&nand_dat->pdev->dev, "%s: nand driver removed\n", __func__);

	mtd_device_unregister(nand_to_mtd(&nand_dat->chip));
	nand_cleanup(&nand_dat->chip);

	kfree(nand_dat);

	return 0;
}

static const struct of_device_id al_nand_match[] = {
	{ .compatible = "al,alpine-nand", },
	{}
};

static struct platform_driver al_nand_driver = {
	.driver = {
		.name = "al,alpine-nand",
		.owner = THIS_MODULE,
		.of_match_table = al_nand_match,
	},
	.probe = al_nand_probe,
	.remove = al_nand_remove,
};

static int __init nand_init(void)
{
	return platform_driver_register(&al_nand_driver);
}

static void __exit nand_exit(void)
{
	platform_driver_unregister(&al_nand_driver);
}

module_init(nand_init);
module_exit(nand_exit);

