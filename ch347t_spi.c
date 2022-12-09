/*
 * This file is part of the flashrom project.
 *
 * Copyright (C) 2022 qianfan Zhao <qianfanguijin@163.com>
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
 */

#include <string.h>
#include <libusb.h>
#include "flash.h"
#include "platform.h"
#include "programmer.h"

#define USB_TIMEOUT			1000	/* 1000 ms is plenty and we have no backup strategy anyway. */
#define WRITE_EP			0x06
#define READ_EP				0x86

#define CH347_MAX_CS			2
#define CH347_USB_MPS			512
#define CH347_MAX_DATA_WRITE		512
#define CH347_MAX_DATA_READ		4096

enum {
	CH347_CMD_SPI_INIT		= 0xC0,
	CH347_CMD_SPI_SET_CS,
	CH347_CMD_SPI_RW,		/* read and write */
	CH347_CMD_SPI_RO,		/* read only */
	CH347_CMD_SPI_WO,		/* write only */
	CH347_CMD_SPI_INFO_RD		= 0xCA
};

enum spi_prescaler {
	SPI_BAUDRATEPRESCALER_2		= 0x00,
	SPI_BAUDRATEPRESCALER_4		= 0x08,
	SPI_BAUDRATEPRESCALER_8		= 0x10,
	SPI_BAUDRATEPRESCALER_16	= 0x18,
	SPI_BAUDRATEPRESCALER_32	= 0x20,
	SPI_BAUDRATEPRESCALER_64	= 0x28,
	SPI_BAUDRATEPRESCALER_128	= 0x30,
	SPI_BAUDRATEPRESCALER_256	= 0x38
};

enum spi_dir {
	SPI_DIRECTION_2LINES_FULLDUPLEX	= 0x0000,
	SPI_DIRECTION_2LINES_RXONLY	= 0x0400,
	SPI_DIRECTION_1LINE_RX		= 0x8000,
	SPI_DIRECTION_1LINE_TX		= 0xc000
};

enum spi_mode {
	SPI_MODE_SLAVE			= 0x0000,
	SPI_MODE_MASTER			= 0x0104
};

enum spi_datasize {
	SPI_DATASIZE_16B		= 0x0800,
	SPI_DATASIZE_8B			= 0x0000
};

enum spi_cpol {
	SPI_CPOL_LOW			= 0,
	SPI_CPOL_HIGH			= 2,
};

enum spi_cpha {
	SPI_CPHA_1EDGE			= 0,
	SPI_CPHA_2EDGE			= 1
};

enum spi_firstbit {
	SPI_FIRSTBIT_LSB		= 0x0080,
	SPI_FIRSTBIT_MSB		= 0x0000
};

enum spi_nss {
	SPI_NSS_SOFT			= 0x0200,
	SPI_NSS_HARD			= 0x0000
};

#define SPI_MISCCFG_CS0_ACTIVE_HIGH	0x80
#define SPI_MISCCFG_CS1_ACTIVE_HIGH	0x40

typedef uint8_t __le8;
typedef uint16_t __le16;
typedef uint32_t __le32;

struct ch347_spi_param {
	__le16				spi_direction;
	__le16				spi_mode;
	__le16				spi_datasize;
	__le16				spi_cpol;
	__le16				spi_cpha;
	__le16				spi_nss;
	__le16				spi_prescaler;
	__le16				spi_firstbit;
	__le16				spi_crcpoly;
	__le16				spi_rwdelay_us; /* SpiOutInInterTime(us) */
	__le8				spi_rx_outb; /* data send on mosi when recving */
	__le8				spi_misccfg;
	__le8				reserved[4];
} __attribute__((packed));

static void ch347_spi_param_set_default(struct ch347_spi_param *p)
{
	p->spi_direction = cpu_to_le16(SPI_DIRECTION_2LINES_FULLDUPLEX);
	p->spi_mode = cpu_to_le16(SPI_MODE_MASTER);
	p->spi_datasize = cpu_to_le16(SPI_DATASIZE_8B);
	p->spi_cpol = cpu_to_le16(SPI_CPOL_HIGH); /* SPI MODE3 */
	p->spi_cpha = cpu_to_le16(SPI_CPHA_2EDGE);
	p->spi_nss = cpu_to_le16(SPI_NSS_SOFT);
	p->spi_prescaler = cpu_to_le16(SPI_BAUDRATEPRESCALER_16); /* 120 / 16 = 7.5M */
	p->spi_firstbit = cpu_to_le16(SPI_FIRSTBIT_MSB);
	p->spi_rx_outb = cpu_to_le8(0xff);
}

struct ch347_spi_set_cs_param {
	#define CH347_SPICS_ACTIVE	0x80
	#define CH347_SPICS_DEACTIVE	0xC0
	uint8_t				active_flag;
	uint16_t			active_delay_us;
	uint16_t			deactive_delay_us;
};

#define buf_set_le8(p, n) do {		\
	*(p) = (n);			\
	(p)++;				\
} while (0)

#define buf_set_le16(p, n) do {		\
	__le16 tmp = cpu_to_le16(n);	\
	memcpy(p, &tmp, sizeof(tmp));	\
	(p) += sizeof(tmp);		\
} while (0)

static size_t ch347_spi_set_cs_param_to_buf(struct ch347_spi_set_cs_param *cs0,
					    struct ch347_spi_set_cs_param *cs1,
					    uint8_t *buf, size_t bufsz)
{
	#define CH347_SPI_SET_CS_PARAM_SZ 10
	uint8_t *p0 = buf, *p1 = buf + (CH347_SPI_SET_CS_PARAM_SZ / 2);
	size_t length = 0;

	if (bufsz >= CH347_SPI_SET_CS_PARAM_SZ) {
		length = CH347_SPI_SET_CS_PARAM_SZ;
		memset(buf, 0, length);

		if (cs0) {
			buf_set_le8( p0, cs0->active_flag);
			buf_set_le16(p0, cs0->active_delay_us);
			buf_set_le16(p0, cs0->deactive_delay_us);
		}

		if (cs1) {
			buf_set_le8( p1, cs1->active_flag);
			buf_set_le16(p1, cs1->active_delay_us);
			buf_set_le16(p1, cs1->deactive_delay_us);
		}
	}

	return length;
}

struct ch347_device {
	struct libusb_device_handle	*handle;

	int				channel;
	struct ch347_spi_param		param;
};

static void msg_pspew_hexdump(const void *data, size_t sz)
{
	const size_t bytes_per_line = 64;
	const uint8_t *p = data;

	for (size_t i = 0; i < sz; i++) {
		msg_pspew("%02x", p[i]);
		if (((i + 1) % bytes_per_line) == 0)
			msg_pspew("\n");
	}

	if (sz % bytes_per_line)
		msg_pspew("\n");
}

/* return negative when failed, 0 on successful */
static int ch347_write(struct ch347_device *dev, uint8_t cmd, const void *data, size_t sz)
{
	uint8_t buf[3 + CH347_MAX_DATA_WRITE];
	int err = 0;

	while (sz > 0) {
		size_t n = min(sz, CH347_MAX_DATA_WRITE);
		int transferred = 0;

		buf[0] = cmd;
		buf[1] = (n >> 0) & 0xff; /* lsb */
		buf[2] = (n >> 8) & 0xff; /* msb */
		memcpy(&buf[3], data, n);

		msg_pspew("TX (%03zu):\n", n + 3);
		msg_pspew_hexdump(buf, n + 3);
		err = libusb_bulk_transfer(dev->handle, WRITE_EP, buf, n + 3,
					   &transferred, USB_TIMEOUT);
		if (err < 0) {
			msg_perr("Sending cmd %02x with %zu bytes data failed(%d %s)\n",
				cmd, n, err, libusb_error_name(err));
			break;
		}

		transferred -= 3;
		data += transferred;
		sz -= transferred;
	}

	return err;
}

/* return number of bytes read on successful */
static int ch347_read(struct ch347_device *dev, uint8_t cmd, void *rx, size_t rxsz)
{
	uint8_t buf[CH347_USB_MPS];
	size_t n = 0;

	do {
		size_t remain = rxsz - n;
		int err, transferred;
		size_t length;

		err = libusb_bulk_transfer(dev->handle, READ_EP, buf,
					   sizeof(buf), &transferred, USB_TIMEOUT);
		if (err < 0) {
			msg_perr("Read response of cmd %02x failed(%d, %s)\n",
				 cmd, err, libusb_error_name(err));
			return err;
		}

		msg_pspew("RX (%03d):\n", transferred);
		msg_pspew_hexdump(buf, transferred);

		if (buf[0] != cmd) {
			msg_perr("Read response of cmd %02x failed(bad code %02x)\n",
				 cmd, buf[0]);
			return -1;
		} else if (transferred < 3) {
			msg_perr("Read response of cmd %02x failed(imcompleted %d)\n",
				 cmd, transferred);
			return -1;
		}

		transferred -= 3;
		length = buf[1] | (buf[2] << 8);
		if (length != (size_t)transferred) {
			msg_perr("Read length doesn't match(%zu != %d)\n",
				 length, transferred);
			return -1;
		}

		if (length > remain)
			length = remain;
		memcpy(rx + n, buf + 3, length);
		n += length;
	} while (n < rxsz);

	return n;
}

static int ch347_transfer(struct ch347_device *dev, uint8_t cmd, const void *tx,
			  size_t txsz, void *rx, size_t rxsz)
{
	int ret;

	ret = ch347_write(dev, cmd, tx, txsz);
	if (ret < 0)
		return ret;

	return ch347_read(dev, cmd, rx, rxsz);
}

static int ch347_spi_set_param(struct ch347_device *dev, struct ch347_spi_param *p)
{
	__le8 ack = 0xff;
	int ret;

	ret = ch347_transfer(dev, CH347_CMD_SPI_INIT, p, sizeof(*p),
			     &ack, sizeof(ack));
	if (ret < 0)
		return ret;

	if (le_to_cpu8(ack) != 0x00) {
		msg_perr("set param failed(%02x)\n", le_to_cpu8(ack));
		return -1;
	}

	return 0;
}

static int ch347_spi_set_cs(struct ch347_device *dev, int isactive)
{
	struct ch347_spi_set_cs_param cs = { 0 };
	uint8_t buf[32];
	size_t len;

	cs.active_flag = isactive ? CH347_SPICS_ACTIVE : CH347_SPICS_DEACTIVE;

	if (dev->channel == 0)
		len = ch347_spi_set_cs_param_to_buf(&cs, NULL, buf, sizeof(buf));
	else
		len = ch347_spi_set_cs_param_to_buf(NULL, &cs, buf, sizeof(buf));

	return ch347_write(dev, CH347_CMD_SPI_SET_CS, buf, len);
}

static int ch347_spi_write_only(struct ch347_device *dev, const void *tx, size_t sz)
{
	__le8 ack = 0xff;
	int ret;

	ret = ch347_transfer(dev, CH347_CMD_SPI_WO, tx, sz, &ack, sizeof(ack));
	if (ret < 0)
		return ret;

	if (le_to_cpu8(ack) != 0x00) {
		msg_perr("write %zu bytes data failed(%02x)\n",
			 sz, le_to_cpu8(ack));
		return -1;
	}

	return 0;
}

static int ch347_spi_read_only(struct ch347_device *dev, void *rx, size_t sz)
{
	__le32 rxsz = cpu_to_le32(sz);
	int ret;

	ret = ch347_transfer(dev, CH347_CMD_SPI_RO, &rxsz, sizeof(rxsz), rx, sz);
	if (ret < 0)
		return ret;

	if (ret != (int)sz) {
		msg_perr("read incompleted(%zu != %d)\n",
			 sz, ret);
		return -1;
	}

	return 0;
}

static struct ch347_device ch347_device = {
	.handle = NULL,
};

static const struct dev_entry devs_ch347t_spi[] = {
	{0x1A86, 0x55DB, OK, "Winchiphead (WCH)", "CH347T"},
	{0},
};

static int ch347t_spi_send_command(const struct flashctx *flash,
				   unsigned int writecnt, unsigned int readcnt,
				   const unsigned char *writearr,
				   unsigned char *readarr)
{
	struct ch347_device *dev = &ch347_device;
	int ret;

	if (!dev->handle)
		return -1;

	ret = ch347_spi_set_cs(dev, 1);
	if (ret < 0)
		return ret;

	if (writecnt) {
		ret = ch347_spi_write_only(dev, writearr, writecnt);
		if (ret < 0)
			goto release;
	}

	if (readcnt) {
		ret = ch347_spi_read_only(dev, readarr, readcnt);
		if (ret < 0)
			goto release;
	}

release:
	ch347_spi_set_cs(dev, 0);
	return ret;
}

/*
 * Suupport vendor protocol mode only which usb id is 1a86:55db.
 * SPI is running at 7.5M and fixed CS0.
 *
 * USB interfaces:
 *     interface[0:1]: CDC ACM
 *     interface2:     Vendor Specific Class(for spi)
 */

#define CH347_VCP_INTERFACE		2

static int ch347t_spi_shutdown(void *data)
{
	struct ch347_device *dev = &ch347_device;
	struct libusb_device_handle *handle = dev->handle;

	if (!handle)
		return -1;

	libusb_release_interface(handle, CH347_VCP_INTERFACE);
	libusb_attach_kernel_driver(handle, CH347_VCP_INTERFACE);
	libusb_close(handle);
	libusb_exit(NULL);
	dev->handle = NULL;
	return 0;
}

static const struct spi_master spi_master_ch347t_spi = {
	.features	= SPI_MASTER_4BA,
	.max_data_read	= CH347_MAX_DATA_READ,
	.max_data_write	= CH347_MAX_DATA_WRITE,
	.command	= ch347t_spi_send_command,
	.multicommand	= default_spi_send_multicommand,
	.read		= default_spi_read,
	.write_256	= default_spi_write_256,
	.write_aai	= default_spi_write_aai,
	.shutdown	= ch347t_spi_shutdown,
	.probe_opcode	= default_spi_probe_opcode,
};

static int ch347t_spi_init(const struct programmer_cfg *cfg)
{
	struct ch347_device *dev = &ch347_device;
	struct libusb_device_handle *handle;
	int ret;

	if (dev->handle != NULL) {
		msg_cerr("%s: handle already set! Please report a bug at flashrom@flashrom.org\n", __func__);
		return -1;
	}

	ret = libusb_init(NULL);
	if (ret < 0) {
		msg_perr("Couldn't initialize libusb!\n");
		return -1;
	}

	/* Enable information, warning, and error messages (only). */
#if LIBUSB_API_VERSION < 0x01000106
	libusb_set_debug(NULL, 3);
#else
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_INFO);
#endif

	uint16_t vid = devs_ch347t_spi[0].vendor_id;
	uint16_t pid = devs_ch347t_spi[0].device_id;
	handle = libusb_open_device_with_vid_pid(NULL, vid, pid);
	if (handle == NULL) {
		msg_perr("Couldn't open device %04x:%04x.\n", vid, pid);
		return -1;
	}

	ret = libusb_detach_kernel_driver(handle, CH347_VCP_INTERFACE);
	if (ret != 0 && ret != LIBUSB_ERROR_NOT_FOUND)
		msg_pwarn("Cannot detach the existing USB driver. Claiming the interface may fail. %s\n",
			libusb_error_name(ret));

	ret = libusb_claim_interface(handle, CH347_VCP_INTERFACE);
	if (ret != 0) {
		msg_perr("Failed to claim interface 0: '%s'\n", libusb_error_name(ret));
		goto close_handle;
	}

	dev->handle = handle;
	ch347_spi_param_set_default(&dev->param);

	ret = ch347_spi_set_param(dev, &dev->param);
	if (ret < 0) {
		msg_perr("Failed to init spi.\n");
		goto release_interface;
	}

	return register_spi_master(&spi_master_ch347t_spi, NULL);

release_interface:
	libusb_release_interface(handle, CH347_VCP_INTERFACE);
close_handle:
	libusb_attach_kernel_driver(handle, CH347_VCP_INTERFACE);
	libusb_close(handle);
	handle = NULL;
	return -1;
}

const struct programmer_entry programmer_ch347t_spi = {
	.name			= "ch347t_spi",
	.type			= USB,
	.devs.dev		= devs_ch347t_spi,
	.init			= ch347t_spi_init,
};
