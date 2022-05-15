/*
 * Copyright (C) 2010 - 2017 Novatek, Inc.
 *
 * $Revision$
 * $Date$
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/uaccess.h>

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio.h>
//#include <linux/vivo_ts_function.h>

#include "nt36xxx.h"
#include "firmware/PD1901_no_flash_fw.h"
#include "firmware/PD1901_no_flash_fw_mp.h"
#include "firmware/PD1901_no_flash_fw_lc_boe.h"
#include "firmware/PD1901_no_flash_fw_lc_boe_mp.h"
#include "firmware/PD1901_no_flash_fw_inx.h"
#include "firmware/PD1901_no_flash_fw_inx_mp.h"
#include "firmware/PD1901_no_flash_fw_36525b_boe.h"
#include "firmware/PD1901_no_flash_fw_36525b_boe_mp.h"

#if BOOT_UPDATE_FIRMWARE
#define VTI NVT_LOG 
#define VTD NVT_LOG

#define FW_BIN_SIZE_116KB		(118784)
#define FW_BIN_SIZE FW_BIN_SIZE_116KB
#define FW_BIN_VER_OFFSET		(0x1A000)
#define FW_BIN_VER_BAR_OFFSET	(0x1A001)
#define FW_BIN_TYPE_OFFSET		(0x1A00D)

#define NVT_DUMP_PARTITION      (0)
#define NVT_DUMP_PARTITION_LEN  (1024)
#define NVT_DUMP_PARTITION_PATH "/data/local/tmp"

struct timeval start, end;
const struct firmware *fw_entry_Spi = NULL;
static uint8_t *fwbuf = NULL;
static struct firmware array_info;

struct nvt_ts_bin_map {
	char name[12];
	uint32_t BIN_addr;
	uint32_t SRAM_addr;
	uint32_t size;
	uint32_t crc;
	NVT_PARTITION_TYPE type;
};

static struct nvt_ts_bin_map *bin_map;

/*******************************************************
Description:
	Novatek touchscreen init variable and allocate buffer
for download firmware function.

return:
	n.a.
*******************************************************/
static int32_t nvt_download_init(void)
{
	/* allocate buffer for transfer firmware */
	//NVT_LOG("NVT_TANSFER_LEN = %ld\n", NVT_TANSFER_LEN);

	if (fwbuf == NULL) {
		fwbuf = (uint8_t *)kzalloc((NVT_TANSFER_LEN+1), GFP_KERNEL);
		if(fwbuf == NULL) {
			NVT_ERR("kzalloc for fwbuf failed!\n");
			return -ENOMEM;
		}
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen checksum function. Calculate bin
file checksum for comparison.

return:
	n.a.
*******************************************************/
static uint32_t CheckSum(const u8 *data, uint32_t len)
{
	uint32_t i = 0;
	uint32_t checksum = 0;

	for (i = 0 ; i < len+1 ; i++)
		checksum += data[i];

	checksum += len;
	checksum = ~checksum +1;

	return checksum;
}

static uint32_t byte_to_word(const uint8_t *data)
{
	return data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);
}

/*******************************************************
Description:
	Novatek touchscreen parsing bin header function.

return:
	n.a.
*******************************************************/
static uint32_t partition = 0;
static uint8_t ilm_dlm_num = 2;
static int32_t nvt_bin_header_parser(const u8 *fwdata, size_t fwsize)
{
	uint32_t list = 0;
	uint32_t pos = 0x00;
	uint32_t end = 0x00;
	uint8_t info_sec_num = 0;
	uint8_t ovly_sec_num = 0;
	uint8_t ovly_info = 0;

	/* Find the header size */
	end = fwdata[0] + (fwdata[1] << 8) + (fwdata[2] << 16) + (fwdata[3] << 24);
	pos = 0x30;	// info section start at 0x30 offset
	while (pos < end) {
		info_sec_num ++;
		pos += 0x10;	/* each header info is 16 bytes */
	}

	/*
	 * Find the DLM OVLY section
	 * [0:3] Overlay Section Number
	 * [4]   Overlay Info
	 * [5]   WakeUp Gesture Info
	 */
	ts_spi->wkg_info = (fwdata[0x28] & 0x20) >> 5;
	ovly_info = (fwdata[0x28] & 0x10) >> 4;
	ovly_sec_num = (ovly_info) ? (fwdata[0x28] & 0x0F) : 0;

	/*
	 * calculate all partition number
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	partition = ilm_dlm_num + ovly_sec_num + info_sec_num;
	NVT_LOG("wkg_info = %d, ovly_info = %d, "
			"ilm_dlm_num = %d, ovly_sec_num = %d, info_sec_num = %d, partition = %d\n",
			ts_spi->wkg_info, ovly_info, ilm_dlm_num, ovly_sec_num, info_sec_num, partition);

	/* allocated memory for header info */
	bin_map = (struct nvt_ts_bin_map *)kzalloc((partition+1) * sizeof(struct nvt_ts_bin_map), GFP_KERNEL);
	if(bin_map == NULL) {
		NVT_ERR("kzalloc for bin_map failed!\n");
		return -ENOMEM;
	}

	for (list = 0; list < partition; list++) {
		/*
		 * [1] parsing ILM & DLM header info
		 * BIN_addr : SRAM_addr : size (12-bytes)
		 * crc located at 0x18 & 0x1C
		 */
		if (list < ilm_dlm_num) {
			bin_map[list].BIN_addr = byte_to_word(&fwdata[0 + list*12]);
			bin_map[list].SRAM_addr = byte_to_word(&fwdata[4 + list*12]);
			bin_map[list].size = byte_to_word(&fwdata[8 + list*12]);
			if (ts_spi->hw_crc)
				bin_map[list].crc = byte_to_word(&fwdata[0x18 + list*4]);
			else {
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			} //ts->hw_crc
			if (list == 0) {
				sprintf(bin_map[list].name, "ILM");
				bin_map[list].type = NVTILM;
			}
			else if (list == 1) {
				sprintf(bin_map[list].name, "DLM");
				bin_map[list].type = NVTDLM;
			}
		}

		/*
		 * [2] parsing others header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if ((list >= ilm_dlm_num) && (list < (ilm_dlm_num + info_sec_num))) {
			/* others partition located at 0x30 offset */
			pos = 0x30 + (0x10 * (list - ilm_dlm_num));

			bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
			bin_map[list].size = byte_to_word(&fwdata[pos+4]);
			bin_map[list].BIN_addr = byte_to_word(&fwdata[pos+8]);
			if (ts_spi->hw_crc)
				bin_map[list].crc = byte_to_word(&fwdata[pos+12]);
			else {
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			} //ts->hw_crc
			/* detect header end to protect parser function */
			if ((bin_map[list].BIN_addr == 0) && (bin_map[list].size != 0)) {
				sprintf(bin_map[list].name, "Header");
				bin_map[list].type = NVTHEADER;
			} else {
				sprintf(bin_map[list].name, "Info-%d", (list - ilm_dlm_num));
				bin_map[list].type = NVTINFO;
			}
		}

		/*
		 * [3] parsing overlay section header info
		 * SRAM_addr : size : BIN_addr : crc (16-bytes)
		 */
		if (list >= (ilm_dlm_num + info_sec_num)) {
			/* overlay info located at DLM (list = 1) start addr */
			pos = bin_map[1].BIN_addr + (0x10 * (list- ilm_dlm_num - info_sec_num));

			bin_map[list].SRAM_addr = byte_to_word(&fwdata[pos]);
			bin_map[list].size = byte_to_word(&fwdata[pos+4]);
			bin_map[list].BIN_addr = byte_to_word(&fwdata[pos+8]);
			if (ts_spi->hw_crc)
				bin_map[list].crc = byte_to_word(&fwdata[pos+12]);
			else {
				bin_map[list].crc = CheckSum(&fwdata[bin_map[list].BIN_addr], bin_map[list].size);
			} //ts->hw_crc
			sprintf(bin_map[list].name, "Overlay-%d", (list- ilm_dlm_num - info_sec_num));
			/* if wkg_info flag enable, overlay-0 is gesture function */
			if ((list == (ilm_dlm_num + info_sec_num)) && (ts_spi->wkg_info)) {
				bin_map[list].type = NVTWKG;
			} else {
				bin_map[list].type = NVTOVLAY;
			}
		}

		/* BIN size error detect */
		if ((bin_map[list].BIN_addr + bin_map[list].size) > fwsize) {
			NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					bin_map[list].BIN_addr, bin_map[list].BIN_addr + bin_map[list].size);
			return -EINVAL;
		}

//		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X), CRC (0x%08X)\n",
//				list, bin_map[list].name,
//				bin_map[list].SRAM_addr, bin_map[list].size,  bin_map[list].BIN_addr, bin_map[list].crc);
	}

	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen release update firmware function.

return:
	n.a.
*******************************************************/
static void update_firmware_release(void)
{
	if ((fw_entry_Spi != NULL) && (fw_entry_Spi != &array_info)) {
		release_firmware(fw_entry_Spi);
	}

	fw_entry_Spi = NULL;
}

/*******************************************************
Description:
	Novatek touchscreen request update firmware function.

return:
	Executive outcomes. 0---succeed. -1,-22---failed.
*******************************************************/
static int32_t update_firmware_request(char *filename, int fwtype, int fwRequest)
{
	uint8_t retry = 0;
	int32_t ret = 0;

	if (NULL == filename) {
		return -1;
	}	

	while (1) {	
		VTI("VIVO_TS loading the firmware\n");
		
		if (fwtype == FWTYPE_Normal) {
			VTI("VTS_FW_TYPE_FW= %d fwtype = %d", 1, fwtype);
			array_info.data = PD1901_no_flash_fw_36525b_boe;//vivoTsGetFw(VTS_FW_TYPE_FW, &fw_size);
			array_info.size = sizeof(PD1901_no_flash_fw_36525b_boe);//fw_size;
			update_firmware_release();
			fw_entry_Spi = &array_info;
		} else { //FWTYPE_MP
			VTI("VTS_FW_TYPE_MP = %d fwtype = %d", 0, fwtype);
			array_info.data = PD1901_no_flash_fw_36525b_boe_mp;
			array_info.size = sizeof(PD1901_no_flash_fw_36525b_boe_mp);
			update_firmware_release();
			fw_entry_Spi = &array_info;
		}
			/*goto request_fail;*/
		

		// check bin file size (116kb)
		if (fw_entry_Spi->size != FW_BIN_SIZE) {
			NVT_ERR("bin file size not match. (%zu)\n", fw_entry_Spi->size);
			ret = -1;
			goto invalid;
		}

		// check if FW version add FW version bar equals 0xFF
		if (*(fw_entry_Spi->data + FW_BIN_VER_OFFSET) + *(fw_entry_Spi->data + FW_BIN_VER_BAR_OFFSET) != 0xFF) {
			NVT_ERR("bin file FW_VER + FW_VER_BAR should be 0xFF!\n");
			NVT_ERR("FW_VER=0x%02X, FW_VER_BAR=0x%02X\n", *(fw_entry_Spi->data+FW_BIN_VER_OFFSET), *(fw_entry_Spi->data+FW_BIN_VER_BAR_OFFSET));
			ret = -1;
			goto invalid;
		}

		NVT_LOG("FW type is 0x%02X\n", *(fw_entry_Spi->data + FW_BIN_TYPE_OFFSET));

		/* BIN Header Parser */
		ret = nvt_bin_header_parser(fw_entry_Spi->data, fw_entry_Spi->size);
		if (ret) {
			NVT_ERR("bin header parser failed\n");
			goto invalid;
		} else {
			break;
		}

invalid:
		update_firmware_release();

/*request_fail:*/
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	return ret;
}

#if NVT_DUMP_PARTITION
/*******************************************************
Description:
	Novatek touchscreen dump flash partition function.

return:
	n.a.
*******************************************************/
loff_t file_offset = 0;
static int32_t nvt_read_ram_and_save_file(uint32_t addr, uint16_t len, char *name)
{
	char file[256] = "";
	uint8_t *fbufp = NULL;
	int32_t ret = 0;
	struct file *fp = NULL;
	mm_segment_t org_fs;

	sprintf(file, "%s/dump_%s.bin", NVT_DUMP_PARTITION_PATH, name);
	NVT_LOG("Dump [%s] from 0x%08X to 0x%08X\n", file, addr, addr+len);

	fbufp = (uint8_t *)kzalloc(len+1, GFP_KERNEL);
	if(fbufp == NULL) {
		NVT_ERR("kzalloc for fbufp failed!\n");
		ret = -ENOMEM;
		goto alloc_buf_fail;
	}

	org_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(file, O_RDWR | O_CREAT, 0644);
	if (fp == NULL || IS_ERR(fp)) {
		ret = -ENOMEM;
		NVT_ERR("open file failed\n");
		goto open_file_fail;
	}

	/* SPI read */
	//---set xdata index to addr---
	nvt_set_page(addr);

	fbufp[0] = addr & 0x7F;	//offset
	CTP_SPI_READ(ts_spi->client, fbufp, len+1);

	/* Write to file */
	ret = vfs_write(fp, (char __user *)fbufp+1, len, &file_offset);
	if (ret != len) {
		NVT_ERR("write file failed\n");
		goto open_file_fail;
	} else {
		ret = 0;
	}

open_file_fail:
	set_fs(org_fs);
	if (!IS_ERR_OR_NULL(fp)) {
		filp_close(fp, NULL);
		fp = NULL;
	}

	if (!IS_ERR_OR_NULL(fbufp)) {
		kfree(fbufp);
		fbufp = NULL;
	}
alloc_buf_fail:

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen nvt_dump_partition function to dump
 each partition for debug.

return:
	n.a.
*******************************************************/
static int32_t nvt_dump_partition(void)
{
	uint32_t list = 0;
	char *name;
	uint32_t SRAM_addr, size;
	uint32_t i = 0;
	uint16_t len = 0;
	int32_t count = 0;
	int32_t ret = 0;

	if (NVT_DUMP_PARTITION_LEN >= sizeof(ts->rbuf)) {
		NVT_ERR("dump len %d is larger than buffer size %ld\n",
				NVT_DUMP_PARTITION_LEN, sizeof(ts->rbuf));
		return -EINVAL;
	} else if (NVT_DUMP_PARTITION_LEN >= NVT_TANSFER_LEN) {
		NVT_ERR("dump len %d is larger than NVT_TANSFER_LEN\n", NVT_DUMP_PARTITION_LEN);
		return -EINVAL;
	}

	if (bin_map == NULL) {
		NVT_ERR("bin_map is NULL\n");
		return -ENOMEM;
	}

	memset(fwbuf, 0, (NVT_DUMP_PARTITION_LEN+1));

	for (list = 0; list < partition; list++) {
		/* initialize variable */
		SRAM_addr = bin_map[list].SRAM_addr;
		size = bin_map[list].size;
		name = bin_map[list].name;

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!size)
			continue;
		else
			size = size +1;

		/* write data to SRAM */
		if (size % NVT_DUMP_PARTITION_LEN)
			count = (size / NVT_DUMP_PARTITION_LEN) + 1;
		else
			count = (size / NVT_DUMP_PARTITION_LEN);

		for (i = 0 ; i < count ; i++) {
			len = (size < NVT_DUMP_PARTITION_LEN) ? size : NVT_DUMP_PARTITION_LEN;

			/* dump for debug download firmware */
			ret = nvt_read_ram_and_save_file(SRAM_addr, len, name);
			if (ret < 0) {
				NVT_ERR("nvt_read_ram_and_save_file failed, ret = %d\n", ret);
				goto out;
			}

			SRAM_addr += NVT_DUMP_PARTITION_LEN;
			size -= NVT_DUMP_PARTITION_LEN;
		}

		file_offset = 0;
	}

out:
	return ret;
}
#endif /* NVT_DUMP_PARTITION */

/*******************************************************
Description:
	Novatek touchscreen write data to sram function.

- fwdata   : The buffer is written
- SRAM_addr: The sram destination address
- size     : Number of data bytes in @fwdata being written
- BIN_addr : The transferred data offset of @fwdata

return:
	Executive outcomes. 0---succeed. else---fail.
*******************************************************/
static int32_t nvt_write_sram(const u8 *fwdata,
		uint32_t SRAM_addr, uint32_t size, uint32_t BIN_addr)
{
	int32_t ret = 0;
	uint32_t i = 0;
	uint32_t len = 0;
	int32_t count = 0;

	if (size % NVT_TANSFER_LEN)
		count = (size / NVT_TANSFER_LEN) + 1;
	else
		count = (size / NVT_TANSFER_LEN);

	for (i = 0 ; i < count ; i++) {
		len = (size < NVT_TANSFER_LEN) ? size : NVT_TANSFER_LEN;

		//---set xdata index to start address of SRAM---
		ret = nvt_set_page(SRAM_addr);
		if (ret) {
			NVT_ERR("set page failed, ret = %d\n", ret);
			return ret;
		}

		//---write data into SRAM---
		fwbuf[0] = SRAM_addr & 0x7F;	//offset
		memcpy(fwbuf+1, &fwdata[BIN_addr], len);	//payload
		ret = CTP_SPI_WRITE(ts_spi->client, fwbuf, len+1);
		if (ret) {
			NVT_ERR("write to sram failed, ret = %d\n", ret);
			return ret;
		}

		SRAM_addr += NVT_TANSFER_LEN;
		BIN_addr += NVT_TANSFER_LEN;
		size -= NVT_TANSFER_LEN;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen nvt_write_firmware function to write
firmware into each partition.

return:
	n.a.
*******************************************************/
static int32_t nvt_write_firmware(const u8 *fwdata, size_t fwsize)
{
	uint32_t list = 0;
	char *name;
	uint32_t BIN_addr, SRAM_addr, size;	
	int32_t ret = 0;

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	for (list = 0; list < partition; list++) {
		/* ignore WKG for normally boot code */
		if (bin_map[list].type == NVTWKG) {
			continue;
		}

		/* initialize variable */
		SRAM_addr = bin_map[list].SRAM_addr;
		size = bin_map[list].size;
		BIN_addr = bin_map[list].BIN_addr;
		name = bin_map[list].name;

//		NVT_LOG("[%d][%s] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X)\n",
//				list, name, SRAM_addr, size, BIN_addr);

		/* Check data size */
		if ((BIN_addr + size) > fwsize) {
			NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
					BIN_addr, BIN_addr + size);
			ret = -EINVAL;
			goto out;
		}

		/* ignore reserved partition (Reserved Partition size is zero) */
		if (!size)
			continue;
		else
			size = size +1;

		/* write data to SRAM */
		ret = nvt_write_sram(fwdata, SRAM_addr, size, BIN_addr);
		if (ret) {
			NVT_ERR("sram program failed, ret = %d\n", ret);
			goto out;
		}
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check checksum function.
This function will compare file checksum and fw checksum.

return:
	n.a.
*******************************************************/
static int32_t nvt_check_fw_checksum(void)
{
	uint32_t fw_checksum = 0;
	uint32_t len = partition*4;
	uint32_t list = 0;
	int32_t ret = 0;

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	//---set xdata index to checksum---
	nvt_set_page(ts_spi->mmap->R_ILM_CHECKSUM_ADDR);

	/* read checksum */
	fwbuf[0] = (ts_spi->mmap->R_ILM_CHECKSUM_ADDR) & 0x7F;
	ret = CTP_SPI_READ(ts_spi->client, fwbuf, len+1);
	if (ret) {
		NVT_ERR("Read fw checksum failed\n");
		return ret;
	}

	/*
	 * Compare each checksum from fw
	 * ILM + DLM + Overlay + Info
	 * ilm_dlm_num (ILM & DLM) + ovly_sec_num + info_sec_num
	 */
	for (list = 0; list < partition; list++) {
		/* ignore WKG for normally boot code */
		if (bin_map[list].type == NVTWKG) {
			continue;
		}
		fw_checksum = byte_to_word(&fwbuf[1+list*4]);

		/* ignore reserved partition (Reserved Partition size is zero) */
		if(!bin_map[list].size)
			continue;

		if (bin_map[list].crc != fw_checksum) {
			NVT_ERR("[%d] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
					list, bin_map[list].crc, fw_checksum);
			ret = -EIO;
		}
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set bootload crc reg bank function.
This function will set hw crc reg before enable crc function.

return:
	n.a.
*******************************************************/
static void nvt_set_bld_crc_bank(uint32_t DES_ADDR, uint32_t SRAM_ADDR,
		uint32_t LENGTH_ADDR, uint32_t size,
		uint32_t G_CHECKSUM_ADDR, uint32_t crc)
{
	/* write destination address */
	nvt_set_page(DES_ADDR);
	fwbuf[0] = DES_ADDR & 0x7F;
	fwbuf[1] = (SRAM_ADDR) & 0xFF;
	fwbuf[2] = (SRAM_ADDR >> 8) & 0xFF;
	fwbuf[3] = (SRAM_ADDR >> 16) & 0xFF;
	CTP_SPI_WRITE(ts_spi->client, fwbuf, 4);

	/* write length */
	//nvt_set_page(LENGTH_ADDR);
	fwbuf[0] = LENGTH_ADDR & 0x7F;
	fwbuf[1] = (size) & 0xFF;
	fwbuf[2] = (size >> 8) & 0xFF;
	CTP_SPI_WRITE(ts_spi->client, fwbuf, 3);

	/* write golden dlm checksum */
	//nvt_set_page(G_CHECKSUM_ADDR);
	fwbuf[0] = G_CHECKSUM_ADDR & 0x7F;
	fwbuf[1] = (crc) & 0xFF;
	fwbuf[2] = (crc >> 8) & 0xFF;
	fwbuf[3] = (crc >> 16) & 0xFF;
	fwbuf[4] = (crc >> 24) & 0xFF;
	CTP_SPI_WRITE(ts_spi->client, fwbuf, 5);

	return;
}

/*******************************************************
Description:
	Novatek touchscreen check DMA hw crc function.
This function will check hw crc result is pass or not.

return:
	n.a.
*******************************************************/
static void nvt_set_bld_hw_crc(void)
{
	/* [0] ILM */
	/* write register bank */
	nvt_set_bld_crc_bank(ts_spi->mmap->ILM_DES_ADDR, bin_map[0].SRAM_addr,
			ts_spi->mmap->ILM_LENGTH_ADDR, bin_map[0].size,
			ts_spi->mmap->G_ILM_CHECKSUM_ADDR, bin_map[0].crc);

	/* [1] DLM */
	/* write register bank */
	nvt_set_bld_crc_bank(ts_spi->mmap->DLM_DES_ADDR, bin_map[1].SRAM_addr,
			ts_spi->mmap->DLM_LENGTH_ADDR, bin_map[1].size,
			ts_spi->mmap->G_DLM_CHECKSUM_ADDR, bin_map[1].crc);
}

/*******************************************************
Description:
	Novatek touchscreen Download_Firmware with HW CRC
function. It's complete download firmware flow.

return:
	n.a.
*******************************************************/
static int32_t nvt_download_firmware_hw_crc(void)
{
	uint8_t retry = 0;
	int32_t ret = 0;

	do_gettimeofday(&start);

	while (1) {
		/* bootloader reset to reset MCU */
		nvt_bootloader_reset_Spi();

		/* Start to write firmware process */
		ret = nvt_write_firmware(fw_entry_Spi->data, fw_entry_Spi->size);
		if (ret) {
			NVT_ERR("Write_Firmware failed. (%d)\n", ret);
			goto fail;
		}

#if NVT_DUMP_PARTITION
		ret = nvt_dump_partition();
		if (ret) {
			NVT_ERR("nvt_dump_partition failed, ret = %d\n", ret);
		}
#endif

		/* set ilm & dlm reg bank */
		nvt_set_bld_hw_crc();

		/* enable hw bld crc function */
		nvt_bld_crc_enable();

		/* clear fw reset status & enable fw crc check */
		nvt_fw_crc_enable();

		/* Set Boot Ready Bit */
		nvt_boot_ready();

		ret = nvt_check_fw_reset_state_Spi(RESET_STATE_INIT);
		if (ret) {
			NVT_ERR("nvt_check_fw_reset_state_Spi failed. (%d)\n", ret);
			goto fail;
		} else {
			break;
		}
fail:
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	do_gettimeofday(&end);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen Download_Firmware function. It's
complete download firmware flow.

return:
	n.a.
*******************************************************/
static int32_t nvt_download_firmware(void)
{
	uint8_t retry = 0;
	int32_t ret = 0;

	do_gettimeofday(&start);

	while (1) {
		/*
		 * Send eng reset cmd before download FW
		 * Keep TP_RESX low when send eng reset cmd
		 */
#if NVT_TOUCH_SUPPORT_HW_RST
		gpio_set_value(ts_spi->reset_gpio, 0);
		mdelay(1);	//wait 1ms
#endif
		nvt_eng_reset();
#if NVT_TOUCH_SUPPORT_HW_RST
		gpio_set_value(ts_spi->reset_gpio, 1);
		mdelay(10);	//wait tRT2BRST after TP_RST
#endif
		nvt_bootloader_reset_Spi();

		/* clear fw reset status */
		nvt_write_addr(ts_spi->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE, 0x00);

		/* Start to write firmware process */
		ret = nvt_write_firmware(fw_entry_Spi->data, fw_entry_Spi->size);
		if (ret) {
			NVT_ERR("Write_Firmware failed. (%d)\n", ret);
			goto fail;
		}

#if NVT_DUMP_PARTITION
		ret = nvt_dump_partition();
		if (ret) {
			NVT_ERR("nvt_dump_partition failed, ret = %d\n", ret);
		}
#endif

		/* Set Boot Ready Bit */
		nvt_boot_ready();

		ret = nvt_check_fw_reset_state_Spi(RESET_STATE_INIT);
		if (ret) {
			NVT_ERR("nvt_check_fw_reset_state failed. (%d)\n", ret);
			goto fail;
		}

		/* check fw checksum result */
		ret = nvt_check_fw_checksum();
		if (ret) {
			NVT_ERR("firmware checksum not match, retry=%d\n", retry);
			goto fail;
		} else {
			break;
		}

fail:
		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("error, retry=%d\n", retry);
			break;
		}
	}

	do_gettimeofday(&end);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware main function.

return:
	n.a.
*******************************************************/
void nvt_update_firmware(char *firmware_name, int fwtype, int fwRequest)
{
	int8_t ret = 0;

	// request bin file in "/etc/firmware"
	
	ret = update_firmware_request(firmware_name, fwtype, fwRequest);
	if (ret) {
		NVT_ERR("update_firmware_request failed. (%d)\n", ret);
		goto request_firmware_fail;
	}

	/* initial buffer and variable */
	ret = nvt_download_init();
	if (ret) {
		NVT_ERR("Download Init failed. (%d)\n", ret);
		goto download_fail;
	}

	/* download firmware process */
	if (ts_spi->hw_crc)
		ret = nvt_download_firmware_hw_crc();
	else
		ret = nvt_download_firmware();
	if (ret) {
		NVT_ERR("Download Firmware failed. (%d)\n", ret);
		goto download_fail;
	}

	NVT_LOG("Update firmware success! <%ld.%06ld>\n",
			(end.tv_sec - start.tv_sec), (end.tv_usec - start.tv_usec));

	/* Get FW Info */
	ret = nvt_get_fw_info_Spi();
	if (ret) {
		NVT_ERR("nvt_get_fw_info_Spi failed. (%d)\n", ret);
		goto download_fail;
	}

download_fail:
	if (!IS_ERR_OR_NULL(bin_map)) {
		kfree(bin_map);
		bin_map = NULL;
	}

	update_firmware_release();
request_firmware_fail:

	return;
}

/*******************************************************
Description:
	Novatek touchscreen update firmware when booting
	function.

return:
	n.a.
*******************************************************/
void Boot_Update_Firmware_Spi(struct work_struct *work)
{
	mutex_lock(&ts_spi->lock);
	nvt_update_firmware(BOOT_UPDATE_FIRMWARE_NAME, FWTYPE_Normal, FWTYPE_REQUEST_NO);
	mutex_unlock(&ts_spi->lock);
}
static int32_t nvt_get_gesture_info(uint32_t *SRAM_addr, uint32_t *size,
		uint32_t *BIN_addr, uint32_t *crc)
{
	uint32_t list = 0;
	int32_t ret = -1;

	for (list = 0; list < partition; list++) {
		/* get WKG info */
		if (bin_map[list].type == NVTWKG) {
			/* initialize variable */
			*SRAM_addr = bin_map[list].SRAM_addr;
			*size = bin_map[list].size;
			*BIN_addr = bin_map[list].BIN_addr;
			*crc = bin_map[list].crc;

			//NVT_LOG("[WKG] SRAM (0x%08X), SIZE (0x%08X), BIN (0x%08X), CRC (0x%08X)\n",
			//		*SRAM_addr, *size,  *BIN_addr, *crc);

			ret = 0;
			break;
		} else {
			continue;
		}
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen download gesture code function.

- fwdata   : fw data
- fwsize   : fw size
- SRAM_addr: The sram destination address
- size     : Number of data bytes in @fwdata being written
- BIN_addr : The transferred data offset of @fwdata
- crc      : The crc result of transferred data

return:
	Executive outcomes. 0---succeed. else---fail.
*******************************************************/
static int32_t nvt_write_gesture_firmware(const u8 *fwdata, size_t fwsize,
		uint32_t SRAM_addr, uint32_t size, uint32_t BIN_addr, uint32_t crc)
{
	int32_t ret = 0;

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	/* Check data size */
	if ((BIN_addr + size) > fwsize) {
		NVT_ERR("access range (0x%08X to 0x%08X) is larger than bin size!\n",
				BIN_addr, BIN_addr + size);
		ret = -EINVAL;
		goto out;
	}

	size = size +1;

	/* write data to SRAM */
	ret = nvt_write_sram(fwdata, SRAM_addr, size, BIN_addr);
	if (ret) {
		NVT_ERR("sram program failed, ret = %d\n", ret);
		goto out;
	}

out:
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen check gesture checksum function.
This function will compare file checksum and fw checksum.

return:
	n.a.
*******************************************************/
static int32_t nvt_check_gesture_checksum(uint32_t SRAM_addr, uint32_t size, uint32_t crc)
{
	uint32_t fw_checksum = 0;
	uint32_t wkg_size = size;
	uint32_t i = 0;
	uint32_t len = 0;
	uint32_t count = 0;
	uint32_t nvt_read_max = 1024;	/* max read 1k data */
	uint32_t offset = 0;
	uint8_t data[1026];
	int32_t ret = 0;

	if (nvt_read_max >= sizeof(ts_spi->rbuf)) {
		NVT_ERR("read len %d is larger than buffer size %ld\n",
				nvt_read_max, sizeof(ts_spi->rbuf));
		return -EINVAL;
	} else if (nvt_read_max >= NVT_TANSFER_LEN) {
		NVT_ERR("read len %d is larger than NVT_TANSFER_LEN\n", nvt_read_max);
		return -EINVAL;
	}

	memset(fwbuf, 0, (NVT_TANSFER_LEN+1));

	size = size + 1;

	/* read data from SRAM */
	if (size % nvt_read_max)
		count = (size / nvt_read_max) + 1;
	else
		count = (size / nvt_read_max);

	for (i = 0 ; i < count ; i++) {
		len = (size < nvt_read_max) ? size : nvt_read_max;

		//---set xdata index---
		nvt_set_page(SRAM_addr);

		data[0] = (SRAM_addr) & 0x7F;
		ret = CTP_SPI_READ(ts_spi->client, data, len+1);
		if (ret) {
			NVT_ERR("Read fw checksum failed\n");
			return ret;
		}

		memcpy((fwbuf+offset), (data+1), (len));
		SRAM_addr += nvt_read_max;
		size -= nvt_read_max;
		offset += nvt_read_max;
	}

	/* calculate checksum from readback data and compare it! */
	fw_checksum = CheckSum(&fwbuf[0], wkg_size);

	if (crc != fw_checksum) {
		NVT_ERR("[WKG] BIN_checksum=0x%08X, FW_checksum=0x%08X\n",
				crc, fw_checksum);
		ret = -EIO;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen complete flow for download
gesture firmware code function.

return:
	n/a.
*******************************************************/
void nvt_update_gesture_firmware(char *firmware_name, int fwtype, int fwRequest)
{
	uint32_t BIN_addr, SRAM_addr, size, crc;
	uint32_t i = 0;
	uint32_t retry = 0;
	uint8_t buf[8] = {0};
	int32_t ret = 0;

	// request bin file in "/etc/firmware"
	ret = update_firmware_request(firmware_name, fwtype, fwRequest);
	if (ret) {
		NVT_ERR("update_firmware_request failed. (%d)\n", ret);
		goto request_firmware_fail;
	}

	ret = nvt_get_gesture_info(&SRAM_addr, &size, &BIN_addr, &crc);
	if (ret) {
		NVT_ERR("get gesture info failed. (%d)\n", ret);
		goto download_fail;
	}

	/* initial buffer and variable */
	ret = nvt_download_init();
	if (ret) {
		NVT_ERR("Download Init failed. (%d)\n", ret);
		goto download_fail;
	}

	do_gettimeofday(&start);

	/* Step 1: Host Send CMD 0x13, 0xFF, 0xFF, 0x00 to EVENT_MAP_HOST_CMD */
	//---write spi command to enter "wakeup gesture mode"---
	buf[0] = EVENT_MAP_HOST_CMD;
	buf[1] = 0x13;
	buf[2] = 0xFF;
	buf[3] = 0xFF;
	buf[4] = 0x00;
	CTP_SPI_WRITE(ts_spi->client, buf, 5);

	/* Step 2: Host Polling 0xA5 from EVENT_MAP_HOST_CMD */
	for (i=0 ; i<=20 ; i++) {
		usleep_range(10000, 10000);

		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0xFF;
		buf[2] = 0xFF;
		buf[3] = 0xFF;
		buf[4] = 0xFF;
		CTP_SPI_READ(ts_spi->client, buf, 5);

		if (buf[4] == 0xA5) {
			break;
		}

		if (i == 20) {
			NVT_LOG("Polling 0x%02X 0x%02X 0x%02X 0x%02X failed, retry %d\n",
					buf[1], buf[2], buf[3], buf[4], i);
			goto download_fail;
		}
	}

	while (1) {
		/* Step 3: Load gesture code */
		ret = nvt_write_gesture_firmware(fw_entry_Spi->data, fw_entry_Spi->size,
				SRAM_addr, size, BIN_addr, crc);
		if (ret) {
			NVT_ERR("write gesture firmware failed. (%d)\n", ret);
			goto download_fail;
		}

		/* Step 4: Check FW Checksum */
		ret = nvt_check_gesture_checksum(SRAM_addr, size, crc);
		if (!ret) {
			break;
		}

		retry++;
		if(unlikely(retry > 2)) {
			NVT_ERR("load gesture code failed, crc = 0x%02X, retry = %d\n", crc, retry);
			goto download_fail;
		}
	}

	/* Step 5: Host Send CMD 0x00 to EVENT_BUF_ADDR offset 0x53 */
	nvt_write_addr((ts_spi->mmap->EVENT_BUF_ADDR | 0x53), 0x00);
	do_gettimeofday(&end);

	NVT_LOG("Update gesture firmware success! <%ld us>\n",
			(end.tv_sec - start.tv_sec)*1000000L + (end.tv_usec - start.tv_usec));

download_fail:
	if (!IS_ERR_OR_NULL(bin_map)) {
		kfree(bin_map);
		bin_map = NULL;
	}

	update_firmware_release();
request_firmware_fail:
	return;
}
#endif /* BOOT_UPDATE_FIRMWARE */
