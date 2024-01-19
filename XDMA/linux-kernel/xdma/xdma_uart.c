// SPDX-License-Identifier: GPL-2.0
/*
 * uartlite.c: Serial driver for Xilinx uartlite serial controller over PCIe
 *
 * Copyright (C) 2006 Peter Korsgaard <jacmet@sunsite.dk>
 * Copyright (C) 2007 Secret Lab Technologies Ltd.
 * Copyright (C) 2023 Institute of Computing Technology, Chinese Academy of Sciences.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/bitfield.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/pm_runtime.h>

#include "xdma_mod.h"
#include "libxdma.h"
#include "libxdma_api.h"

#define ULITE_NAME		"ttyUL"
#define ULITE_MAJOR		204
#define ULITE_MINOR		187
#define ULITE_NR_XDMA_INSTS	16
#define ULITE_NR_UART_PER_INST	1

#define ULITE_BAR_OFFSET_DFLT	0x11000

/* ---------------------------------------------------------------------
 * Register definitions
 *
 * For register details see datasheet:
 * https://www.xilinx.com/support/documentation/ip_documentation/opb_uartlite.pdf
 */

#define ULITE_RX		0x00
#define ULITE_TX		0x04
#define ULITE_STATUS		0x08
#define ULITE_CONTROL		0x0c

#define ULITE_REGION		16

#define ULITE_STATUS_RXVALID	0x01
#define ULITE_STATUS_RXFULL	0x02
#define ULITE_STATUS_TXEMPTY	0x04
#define ULITE_STATUS_TXFULL	0x08
#define ULITE_STATUS_IE		0x10
#define ULITE_STATUS_OVERRUN	0x20
#define ULITE_STATUS_FRAME	0x40
#define ULITE_STATUS_PARITY	0x80

#define ULITE_CONTROL_RST_TX	0x01
#define ULITE_CONTROL_RST_RX	0x02
#define ULITE_CONTROL_IE	0x10
#define UART_AUTOSUSPEND_TIMEOUT	3000	/* ms */

/**
 * struct uartlite_data: Driver private data
 * reg_ops: Functions to read/write registers
 * baud: The baud rate configured when this device was synthesized
 * cflags: The cflags for parity and data bits
 * xpdev: pointer to structure xdma_pci_dev
 */
struct uartlite_data {
	const struct uartlite_reg_ops *reg_ops;
	unsigned int baud;
	tcflag_t cflags;
	struct xdma_pci_dev *xpdev;
};

struct uartlite_reg_ops {
	u32 (*in)(void __iomem *addr);
	void (*out)(u32 val, void __iomem *addr);
};

static u32 uartlite_inbe32(void __iomem *addr)
{
	return ioread32be(addr);
}

static void uartlite_outbe32(u32 val, void __iomem *addr)
{
	iowrite32be(val, addr);
}

static const struct uartlite_reg_ops uartlite_be = {
	.in = uartlite_inbe32,
	.out = uartlite_outbe32,
};

static u32 uartlite_inle32(void __iomem *addr)
{
	return ioread32(addr);
}

static void uartlite_outle32(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static const struct uartlite_reg_ops uartlite_le = {
	.in = uartlite_inle32,
	.out = uartlite_outle32,
};

static inline u32 uart_in32(u32 offset, struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;

	return pdata->reg_ops->in(port->membase + offset);
}

static inline void uart_out32(u32 val, u32 offset, struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;

	pdata->reg_ops->out(val, port->membase + offset);
}

static struct uart_port ulite_ports[ULITE_NR_XDMA_INSTS][ULITE_NR_UART_PER_INST];

/* ---------------------------------------------------------------------
 * Core UART driver operations
 */

static int ulite_receive(struct uart_port *port, int stat)
{
	struct tty_port *tport = &port->state->port;
	unsigned char ch = 0;
	char flag = TTY_NORMAL;

	if ((stat & (ULITE_STATUS_RXVALID | ULITE_STATUS_OVERRUN
		     | ULITE_STATUS_FRAME)) == 0)
		return 0;

	/* stats */
	if (stat & ULITE_STATUS_RXVALID) {
		port->icount.rx++;
		ch = uart_in32(ULITE_RX, port);

		if (stat & ULITE_STATUS_PARITY)
			port->icount.parity++;
	}

	if (stat & ULITE_STATUS_OVERRUN)
		port->icount.overrun++;

	if (stat & ULITE_STATUS_FRAME)
		port->icount.frame++;


	/* drop byte with parity error if IGNPAR specificed */
	if (stat & port->ignore_status_mask & ULITE_STATUS_PARITY)
		stat &= ~ULITE_STATUS_RXVALID;

	stat &= port->read_status_mask;

	if (stat & ULITE_STATUS_PARITY)
		flag = TTY_PARITY;


	stat &= ~port->ignore_status_mask;

	if (stat & ULITE_STATUS_RXVALID)
		tty_insert_flip_char(tport, ch, flag);

	if (stat & ULITE_STATUS_FRAME)
		tty_insert_flip_char(tport, 0, TTY_FRAME);

	if (stat & ULITE_STATUS_OVERRUN)
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);

	return 1;
}

static int ulite_transmit(struct uart_port *port, int stat)
{
	struct circ_buf *xmit  = &port->state->xmit;

	if (stat & ULITE_STATUS_TXFULL)
		return 0;

	if (port->x_char) {
		uart_out32(port->x_char, ULITE_TX, port);
		port->x_char = 0;
		port->icount.tx++;
		return 1;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		return 0;

	uart_out32(xmit->buf[xmit->tail], ULITE_TX, port);
	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
	port->icount.tx++;

	/* wake up */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	return 1;
}

static irqreturn_t ulite_isr(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	int stat, busy, n = 0;
	unsigned long flags;

	do {
		spin_lock_irqsave(&port->lock, flags);
		stat = uart_in32(ULITE_STATUS, port);
		busy  = ulite_receive(port, stat);
		busy |= ulite_transmit(port, stat);
		spin_unlock_irqrestore(&port->lock, flags);
		n++;
	} while (busy);

	/* work done? */
	if (n > 1) {
		tty_flip_buffer_push(&port->state->port);
		return IRQ_HANDLED;
	} else {
		return IRQ_NONE;
	}
}

static unsigned int ulite_tx_empty(struct uart_port *port)
{
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&port->lock, flags);
	ret = uart_in32(ULITE_STATUS, port);
	spin_unlock_irqrestore(&port->lock, flags);

	return ret & ULITE_STATUS_TXEMPTY ? TIOCSER_TEMT : 0;
}

static unsigned int ulite_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void ulite_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* N/A */
}

static void ulite_stop_tx(struct uart_port *port)
{
	/* N/A */
}

static void ulite_start_tx(struct uart_port *port)
{
	ulite_transmit(port, uart_in32(ULITE_STATUS, port));
}

static void ulite_stop_rx(struct uart_port *port)
{
	/* don't forward any more data (like !CREAD) */
	port->ignore_status_mask = ULITE_STATUS_RXVALID | ULITE_STATUS_PARITY
		| ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;
}

static void ulite_break_ctl(struct uart_port *port, int ctl)
{
	/* N/A */
}

static int ulite_startup(struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;
	struct xdma_pci_dev *xpdev = pdata->xpdev;
	struct xdma_dev *xdev = xpdev->xdev;
	int ret;

	/*FIXME*/
	ret = xdma_user_isr_register(xdev, (1 << port->line), ulite_isr, port);
	if (ret)
		return ret;

	uart_out32(ULITE_CONTROL_RST_RX | ULITE_CONTROL_RST_TX,
		ULITE_CONTROL, port);
	uart_out32(ULITE_CONTROL_IE, ULITE_CONTROL, port);

	return 0;
}

static void ulite_shutdown(struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;
	struct xdma_pci_dev *xpdev = pdata->xpdev;
	struct xdma_dev *xdev = xpdev->xdev;
	int ret;

	uart_out32(0, ULITE_CONTROL, port);
	uart_in32(ULITE_CONTROL, port); /* dummy */
	/*FIXME*/
	ret = xdma_user_isr_register(xdev, (1 << port->line), NULL, NULL);
	if (ret)
		return;
}

static void ulite_set_termios(struct uart_port *port,
			      struct ktermios *termios,
			      struct ktermios *old)
{
	unsigned long flags;
	struct uartlite_data *pdata = port->private_data;

	/* Set termios to what the hardware supports */
	termios->c_iflag &= ~BRKINT;
	termios->c_cflag &= ~(CSTOPB | PARENB | PARODD | CSIZE);
	termios->c_cflag |= pdata->cflags & (PARENB | PARODD | CSIZE);
	tty_termios_encode_baud_rate(termios, pdata->baud, pdata->baud);

	spin_lock_irqsave(&port->lock, flags);

	port->read_status_mask = ULITE_STATUS_RXVALID | ULITE_STATUS_OVERRUN
		| ULITE_STATUS_TXFULL;

	if (termios->c_iflag & INPCK)
		port->read_status_mask |=
			ULITE_STATUS_PARITY | ULITE_STATUS_FRAME;

	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= ULITE_STATUS_PARITY
			| ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |=
			ULITE_STATUS_RXVALID | ULITE_STATUS_PARITY
			| ULITE_STATUS_FRAME | ULITE_STATUS_OVERRUN;

	/* update timeout */
	uart_update_timeout(port, termios->c_cflag, pdata->baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *ulite_type(struct uart_port *port)
{
	return port->type == PORT_UARTLITE ? "uartlite" : NULL;
}

static void ulite_release_port(struct uart_port *port)
{
	port->membase = NULL;
}

static int ulite_request_port(struct uart_port *port)
{
	struct uartlite_data *pdata = port->private_data;
	struct xdma_pci_dev *xpdev = pdata->xpdev;
	struct xdma_dev *xdev = xpdev->xdev;
	struct xdma_uart_device *xuart_dev = &xpdev->uart_dev;
	int ret;

	pr_debug("ulite console: port=%p; port->mapbase=%llx\n",
		 port, (unsigned long long) port->mapbase);

	port->membase = xdev->bar[xuart_dev->bar] + port->mapbase;
	if (!port->membase) {
		dev_err(port->dev, "Unable to map registers\n");
		return -EBUSY;
	}

	pdata->reg_ops = &uartlite_be;
	ret = uart_in32(ULITE_CONTROL, port);
	uart_out32(ULITE_CONTROL_RST_TX, ULITE_CONTROL, port);
	ret = uart_in32(ULITE_STATUS, port);
	/* Endianess detection */
	if ((ret & ULITE_STATUS_TXEMPTY) != ULITE_STATUS_TXEMPTY)
		pdata->reg_ops = &uartlite_le;

	return 0;
}

static void ulite_config_port(struct uart_port *port, int flags)
{
	if (!ulite_request_port(port))
		port->type = PORT_UARTLITE;
}

static int ulite_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

#ifdef CONFIG_CONSOLE_POLL
static int ulite_get_poll_char(struct uart_port *port)
{
	if (!(uart_in32(ULITE_STATUS, port) & ULITE_STATUS_RXVALID))
		return NO_POLL_CHAR;

	return uart_in32(ULITE_RX, port);
}

static void ulite_put_poll_char(struct uart_port *port, unsigned char ch)
{
	while (uart_in32(ULITE_STATUS, port) & ULITE_STATUS_TXFULL)
		cpu_relax();

	/* write char to device */
	uart_out32(ch, ULITE_TX, port);
}
#endif

static const struct uart_ops ulite_ops = {
	.tx_empty	= ulite_tx_empty,
	.set_mctrl	= ulite_set_mctrl,
	.get_mctrl	= ulite_get_mctrl,
	.stop_tx	= ulite_stop_tx,
	.start_tx	= ulite_start_tx,
	.stop_rx	= ulite_stop_rx,
	.break_ctl	= ulite_break_ctl,
	.startup	= ulite_startup,
	.shutdown	= ulite_shutdown,
	.set_termios	= ulite_set_termios,
	.type		= ulite_type,
	.release_port	= ulite_release_port,
	.request_port	= ulite_request_port,
	.config_port	= ulite_config_port,
	.verify_port	= ulite_verify_port,
	//.pm		= ulite_pm,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= ulite_get_poll_char,
	.poll_put_char	= ulite_put_poll_char,
#endif
};

/* ---------------------------------------------------------------------
 * Port assignment functions (mapping devices to uart_port structures)
 */

/** ulite_assign: register a uartlite device with the driver
 *
 * @xuart_dev: pointer to XDMA UART device structure
 * @id: requested id number.  Pass -1 for automatic port assignment
 * @pdata: private data for uartlite
 *
 * Returns: 0 on success, <0 otherwise
 */
static int ulite_assign(struct xdma_uart_device *xuart_dev, int id, struct uartlite_data *pdata)
{
	struct uart_port *port = NULL;
	struct xdma_pci_dev *xpdev = xuart_dev->xpdev;
	struct xdma_dev *xdev = xpdev->xdev;
	int rc;
	int xdma_idx = xdev->idx;

	/* if id = -1; then scan for a free id and use that */
	if (id < 0) {
		for (id = 0; id < ULITE_NR_UART_PER_INST; id++)
			if (ulite_ports[xdma_idx][id].mapbase == 0)
				break;
	}
	if (id < 0 || id >= ULITE_NR_UART_PER_INST) {
		dev_err(&xpdev->pdev->dev, "xdma%d %s%i too large\n", xdma_idx, ULITE_NAME, id);
		return -EINVAL;
	}

	if (ulite_ports[xdma_idx][id].mapbase == ULITE_BAR_OFFSET_DFLT) {
		dev_err(&xpdev->pdev->dev, "cannot assign to xdma%d %s%i; it is already in use\n",
			xdma_idx, ULITE_NAME, id);
		return -EBUSY;
	}

	port = &ulite_ports[xdma_idx][id];

	spin_lock_init(&port->lock);
	port->fifosize = 16;
	port->regshift = 2;
	port->iotype = UPIO_MEM;
	port->iobase = 1; /* mark port in use */
	/*FIXME*/
	port->mapbase = ULITE_BAR_OFFSET_DFLT;
	port->membase = NULL; 
	port->ops = &ulite_ops;
	/*FIXME*/
	port->irq = id;
	port->flags = UPF_BOOT_AUTOCONF;
	/*FIXME*/
	port->dev = &xpdev->pdev->dev;
	port->type = PORT_UNKNOWN;
	/*FIXME*/
	port->line = id;
	port->private_data = pdata;

	/* Register the port */
	rc = uart_add_one_port(&xuart_dev->uart_drv, port);
	if (rc) {
		dev_err(&xpdev->pdev->dev, "uart_add_one_port() failed; err=%i\n", rc);
		port->mapbase = 0;
		port->iobase = 0; /* mark port unused */
		return rc;
	}
	xuart_dev->uart_port = port;

	return 0;
}

/** ulite_release: register a uartlite device with the driver
 *
 * @xuart_dev: pointer to XDMA UART private structure
 */
static int ulite_release(struct xdma_uart_device *xuart_dev)
{
	struct uart_port *port = xuart_dev->uart_port;
	int rc = 0;

	if (port) {
		rc = uart_remove_one_port(&xuart_dev->uart_drv, port);
		if (rc < 0)
			return rc;
		port->mapbase = 0;
		port->iobase = 0; /* mark port unused */
		xuart_dev->uart_port = NULL;
	}

	return 0;
}

/* create_uart_device() -- create a UART device driver */
static int create_uart_device(struct xdma_pci_dev *xpdev, struct xdma_uart_device *xuart_dev, int bar)
{
	int rv;
	struct xdma_dev *xdev = xpdev->xdev;
	struct uartlite_data *pdata;
	char name[16];

	spin_lock_init(&xuart_dev->lock);
	/*
	 * do not register yet, create kobjects and name them,
	 */
	xuart_dev->magic = MAGIC_UART;
	xuart_dev->xpdev = xpdev;
	xuart_dev->xdev = xdev;
	xuart_dev->bar = bar;

	sprintf(name, "xdma%d_%s", xdev->idx, ULITE_NAME);

	xuart_dev->uart_drv.owner = THIS_MODULE;
	xuart_dev->uart_drv.driver_name	= "xdma-uartlite";
	xuart_dev->uart_drv.dev_name = name;
	xuart_dev->uart_drv.major = ULITE_MAJOR + xdev->idx;
	xuart_dev->uart_drv.minor = ULITE_MINOR;
	xuart_dev->uart_drv.nr = ULITE_NR_UART_PER_INST;

	/* allocate private data of uart lite device */
	pdata = devm_kzalloc(&xpdev->pdev->dev, sizeof(struct uartlite_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	/* setup default uart parameters */
	pdata->baud = 115200;
	pdata->cflags = CS8;
	pdata->xpdev = xpdev;

	if (!xuart_dev->uart_drv.state) {
		dev_dbg(&xpdev->pdev->dev, "uartlite: calling uart_register_driver()\n");
		rv = uart_register_driver(&xuart_dev->uart_drv);
		if (rv < 0) {
			dev_err(&xpdev->pdev->dev, "Failed to register uart driver\n");
			return rv;
		}
	}

	rv = ulite_assign(xuart_dev, -1, pdata);

	return rv;
}

static int destroy_uart_device(struct xdma_uart_device *xuart_dev)
{
	return ulite_release(xuart_dev);
}

void xpdev_destroy_uart(struct xdma_pci_dev *xpdev)
{
	struct xdma_dev *xdev = xpdev->xdev;
	struct xdma_uart_device *xuart_dev = &xpdev->uart_dev;
	struct uart_driver *uart_drv = &xuart_dev->uart_drv;
	int rv;

	/* remove UART device */
	if (xdev->user_bar_idx >= 0) {
		rv = destroy_uart_device(xuart_dev);
		if (rv < 0)
			pr_err("Failed to destroy uart device driver (ttyUL) error 0x%x\n", rv);
	}

	/* unregister UART driver */
	if(uart_drv->state)
		uart_unregister_driver(uart_drv);
}

int xpdev_create_uart(struct xdma_pci_dev *xpdev)
{
	struct xdma_dev *xdev = xpdev->xdev;
	int rv = 0;

	/* initialize UART device driver */
	if (xdev->user_bar_idx >= 0) {
		rv = create_uart_device(xpdev, &xpdev->uart_dev, xdev->user_bar_idx);
		if (rv < 0) {
			pr_err("create uart device driver (ttyUL) failed\n");
			goto fail;
		}
	}

	return 0;

fail:
	rv = -1;
	xpdev_destroy_uart(xpdev);
	return rv;
}
