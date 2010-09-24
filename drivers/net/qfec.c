/*
 * qfec.c - qualcomm fast Ethernet driver
 *
 * Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/io.h>

# include <linux/platform_device.h>

# include <linux/types.h>        /* size_t */
# include <linux/interrupt.h>    /* mark_bh */

# include <linux/netdevice.h>   /* struct device, and other headers */
# include <linux/etherdevice.h> /* eth_type_trans */
# include <linux/skbuff.h>

# include <linux/proc_fs.h>
# include <linux/timer.h>
# include <linux/mii.h>

#include "qfec.h"

#define QFEC_NAME       "qfec"

/* -------------------------------------------------------------------------
 */

#define ETH_BUF_SIZE    0x600
#define MAX_N_BD        50
#define MAC_ADDR_SIZE	6

/* -----------------------------------------------------
 * logging macros
 */
#define QFEC_LOG_PR    1
#define QFEC_LOG_DBG   2
#define QFEC_LOG_DBG2  4

static int qfec_debug = QFEC_LOG_PR | QFEC_LOG_DBG;
module_param(qfec_debug, int, 0400);

#define QFEC_DEBUG
#ifdef QFEC_DEBUG
# define QFEC_LOG(flag, ...)                    \
	do {                                    \
		if (flag & qfec_debug)        \
			pr_devel(__VA_ARGS__);   \
	} while (0)
#else
# define QFEC_LOG(flag, ...)
#endif

#define QFEC_LOG_ERR(...) pr_err(__VA_ARGS__)

/* -------------------------------------------------------------------------
 * driver buffer-descriptor
 *   contains the 4 word HW descriptor plus an additional 4-words.
 *   (See the DSL bits in the BUS-Mode register).
 */
#define BD_FLAG_LAST_BD     1

struct buf_desc {
	struct qfec_buf_desc    desc;       /* must be first */
	struct sk_buff         *skb;
	void                   *buf_virt_addr;
	void                   *buf_phys_addr;
	uint32_t                last_bd_flag;
};

/*
 *inline functions accessing non-struct qfec_buf_desc elements
 */

/* skb */
static inline struct sk_buff *qfec_bd_skbuf_get(struct buf_desc *p_bd)
{
	return p_bd->skb;
};

static inline void qfec_bd_skbuf_set(struct buf_desc *p_bd, struct sk_buff *p)
{
	p_bd->skb   = p;
};

/* virtual addr  */
static inline void qfec_bd_virt_set(struct buf_desc *p_bd, void *addr)
{
	p_bd->buf_virt_addr = addr;
};

static inline void *qfec_bd_virt_get(struct buf_desc *p_bd)
{
	return p_bd->buf_virt_addr;
};

/* physical addr  */
static inline void qfec_bd_phys_set(struct buf_desc *p_bd, void *addr)
{
	p_bd->buf_phys_addr = addr;
};

static inline void *qfec_bd_phys_get(struct buf_desc *p_bd)
{
	return p_bd->buf_phys_addr;
};

/* last_bd_flag */
static inline uint32_t qfec_bd_last_bd(struct buf_desc *p_bd)
{
	return (p_bd->last_bd_flag != 0);
};

static inline void qfec_bd_last_bd_set(struct buf_desc *p_bd)
{
	p_bd->last_bd_flag = BD_FLAG_LAST_BD;
};

/*
 *inline functions accessing struct qfec_buf_desc elements
 */

/* ownership bit */
static inline uint32_t qfec_bd_own(struct buf_desc *p_bd)
{
	return p_bd->desc.status & BUF_OWN;
};

static inline void qfec_bd_own_set(struct buf_desc *p_bd)
{
	p_bd->desc.status |= BUF_OWN ;
};

static inline void qfec_bd_own_clr(struct buf_desc *p_bd)
{
	p_bd->desc.status &= ~(BUF_OWN);
};

static inline uint32_t qfec_bd_status_get(struct buf_desc *p_bd)
{
	return p_bd->desc.status;
};

static inline uint32_t qfec_bd_status_len(struct buf_desc *p_bd)
{
	return BUF_RX_FL_GET(p_bd->desc);
};

/* control register */
static inline void qfec_bd_ctl_reset(struct buf_desc *p_bd)
{
	p_bd->desc.ctl  = 0;
};

static inline uint32_t qfec_bd_ctl_get(struct buf_desc *p_bd)
{
	return p_bd->desc.ctl;
};

static inline void qfec_bd_ctl_set(struct buf_desc *p_bd, uint32_t val)
{
	p_bd->desc.ctl |= val;
};

static inline void qfec_bd_ctl_wr(struct buf_desc *p_bd, uint32_t val)
{
	p_bd->desc.ctl = val;
};

/* pbuf register  */
static inline void *qfec_bd_pbuf_get(struct buf_desc *p_bd)
{
	return p_bd->desc.p_buf;
}

static inline void qfec_bd_pbuf_set(struct buf_desc *p_bd, void *p)
{
	p_bd->desc.p_buf = p;
}

/* next register */
static inline void *qfec_bd_next_get(struct buf_desc *p_bd)
{
	return p_bd->desc.next;
};

/* -----------------------------------------------------
 * initialize an RX BD w/ a new buf
 */
static int qfec_rbd_init(struct net_device *dev, struct buf_desc *p_bd)
{
	struct sk_buff     *skb;
	uint32_t            ctrl;
	void               *p;
	void               *v;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %p bd\n", __func__, p_bd);

	/* allocate and record ptrs for sk buff */
	skb   = dev_alloc_skb(ETH_BUF_SIZE);
	if (!skb)
		goto err;

	qfec_bd_skbuf_set(p_bd, skb);
	QFEC_LOG(QFEC_LOG_DBG2, "%s: %p skb\n", __func__, skb);

	v = skb_put(skb, ETH_BUF_SIZE);
	qfec_bd_virt_set(p_bd, v);

	p = (void *) dma_map_single(&dev->dev,
		(void *)skb->data, ETH_BUF_SIZE, DMA_FROM_DEVICE);
	qfec_bd_pbuf_set(p_bd, p);
	qfec_bd_phys_set(p_bd, p);

	QFEC_LOG(QFEC_LOG_DBG2,
		"%s: %p p_bd, %p data, %p skb_put, %p virt, %p p_buf, %p p\n",
		__func__, (void *)p_bd,
		(void *)skb->data, v, /*(void *)skb_put(skb, ETH_BUF_SIZE), */
		(void *)qfec_bd_virt_get(p_bd), (void *)qfec_bd_pbuf_get(p_bd),
		(void *)p);

	/* populate control register */
	ctrl  = ETH_BUF_SIZE;
	qfec_bd_ctl_reset(p_bd);
	qfec_bd_ctl_set(p_bd, ctrl);

	/* mark the last BD and set end-of-ring bit */
	if (qfec_bd_last_bd(p_bd))
		qfec_bd_ctl_set(p_bd, BUF_RX_RER);

	qfec_bd_own_set(p_bd);
	return 0;

err:
	return -ENOMEM;
};

/* -------------------------------------------------------------------------
 * ring structure used to maintain indices of buffer-descriptor (BD) usage
 *
 *   The RX BDs are normally all pre-allocated with buffers available to be
 *   DMA'd into with received frames.  The head indicates the first BD/buffer
 *   containing a received frame, and the tail indicates the oldest BD/buffer
 *   that needs to be restored for use.   Head and tail are both initialized
 *   to zero, and n_free is initialized to zero, since all BD are initialized.
 *
 *   The TX BDs are normally available for use, only being initialized as
 *   TX frames are requested for transmission.   The head indicates the
 *   first available BD, and the tail indicate the oldest BD that has
 *   not been acknowledged as transmitted.    Head and tail are both initialized
 *   to zero, and n_free is initialized to len, since all are available for use.
 */
struct ring {
	int     head;
	int     tail;
	int     n_free;
	int     len;
};

/* accessory in line functions for struct ring */
static inline void qfec_ring_init(struct ring *p_ring, int size, int free)
{
	p_ring->head  = p_ring->tail = 0;
	p_ring->len   = size;
	p_ring->n_free = free;
}

static inline int qfec_ring_full(struct ring *p_ring)
{
	return (p_ring->n_free == 0);
};

static inline int qfec_ring_empty(struct ring *p_ring)
{
	return (p_ring->n_free == p_ring->len);
}

static inline void qfec_ring_head_adv(struct ring *p_ring)
{
	p_ring->head = ++p_ring->head % p_ring->len;
	p_ring->n_free--;
};

static inline void qfec_ring_tail_adv(struct ring *p_ring)
{
	p_ring->tail = ++p_ring->tail % p_ring->len;
	p_ring->n_free++;
};

static inline int qfec_ring_head(struct ring *p_ring)
{

	return p_ring->head;
};

static inline int qfec_ring_tail(struct ring *p_ring)
{
	return p_ring->tail;
};

static inline int qfec_ring_room(struct ring *p_ring)
{
	return p_ring->n_free;
};

/* -------------------------------------------------------------------------
 * counters track normal and abnormal driver events and activity
 */
enum cntr {
	isr                  =  0,
	fatal_bus,

	early_tx,
	tx_no_resource,
	tx_proc_stopped,
	tx_jabber_tmout,

	xmit,
	tx_int,
	tx_isr,
	tx_owned,
	tx_underflow,

	tx_replenish,
	tx_skb_null,
	tx_timeout,
	tx_too_large,

	gmac_isr,

	/* half */
	norm_int,
	abnorm_int,

	early_rx,
	rx_buf_unavail,
	rx_proc_stopped,
	rx_watchdog,

	netif_rx_cntr,
	rx_int,
	rx_isr,
	rx_owned,
	rx_overflow,

	rx_dropped,
	rx_skb_null,
	queue_start,
	queue_stop,

	rx_paddr_nok,
	cntr_last,
};

static char *cntr_name[]  = {
	"isr",
	"fatal_bus",

	"early_tx",
	"tx_no_resource",
	"tx_proc_stopped",
	"tx_jabber_tmout",

	"xmit",
	"tx_int",
	"tx_isr",
	"tx_owned",
	"tx_underflow",

	"tx_replenish",
	"tx_skb_null",
	"tx_timeout",
	"tx_too_large",

	"gmac_isr",

	/* half */
	"norm_int",
	"abnorm_int",

	"early_rx",
	"rx_buf_unavail",
	"rx_proc_stopped",
	"rx_watchdog",

	"netif_rx",
	"rx_int",
	"rx_isr",
	"rx_owned",
	"rx_overflow",

	"rx_dropped",
	"rx_skb_null",
	"queue_start",
	"queue_stop",

	"rx_paddr_nok",

	""
};

/* -------------------------------------------------------------------------
 * private data
 */

static struct net_device  *qfec_dev;

enum qfec_state {
	queue_started = 0x01,
	queue_stopped = 0x02,
};

struct qfec_priv {
	struct net_device      *net_dev;
	struct net_device_stats stats;            /* req statistics */

	struct device           dev;

	spinlock_t              hw_lock;

	unsigned int            state;            /* driver state */

	void                   *bd_base;          /* addr buf-desc */
	dma_addr_t              tbd_dma;          /* dma/phy-addr buf-desc */
	dma_addr_t              rbd_dma;          /* dma/phy-addr buf-desc */

	struct resource        *mac_res;
	void                   *mac_base;         /* mac (virt) base address */

	struct resource        *clk_res;
	void                   *clk_base;         /* clk (virt) base address */

	struct resource        *fuse_res;
	void                   *fuse_base;        /* mac addr fuses */

	unsigned int            n_tbd;            /* # of TX buf-desc */
	struct ring             ring_tbd;         /* TX ring */
	struct buf_desc        *p_tbd;            /* * TX buf-desc */

	unsigned int            n_rbd;            /* # of RX buf-desc */
	struct ring             ring_rbd;         /* RX ring */
	struct buf_desc        *p_rbd;            /* * RX buf-desc */

	unsigned long           cntr[cntr_last];  /* activity counters */

	struct mii_if_info      mii;              /* used by mii lib */

	int                     mdio_clk;         /* phy mdio clock rate */
	int                     phy_id;           /* default PHY addr (0) */
	struct timer_list       phy_tmr;          /* monitor PHY state */
};

/* ----------------------------------------------------------------------------
 * cntrs display
 */

static int qfec_cntrs_show(char *buf, char **start, off_t offset,
	int count, int *eof, void *data)
{
	struct net_device       *dev  = (struct net_device *) data;
	struct qfec_priv        *priv = netdev_priv(dev);
	int                      h    = (cntr_last + 1) / 2;
	int                      l;
	int                      n;

	QFEC_LOG(QFEC_LOG_DBG2, "%s:\n", __func__);

	l = sprintf(&buf[0], "%s:\n", __func__);
	for (n = 0; n < h; n++)  {
		l += sprintf(&buf[l], "      %12lu  %-16s %12lu  %s\n",
			priv->cntr[n],   cntr_name[n],
			priv->cntr[n+h], cntr_name[n+h]);
	}

	*eof = 1;
	return l;
}

# define CNTR_INC(priv, name)  (priv->cntr[name]++)

/* -------------------------------------------------------------------------
 * functions that manage state
 */
static inline void qfec_queue_start(struct net_device *dev)
{
	struct qfec_priv  *priv = netdev_priv(dev);

	if (priv->state & queue_stopped)  {
		netif_wake_queue(dev);
		priv->state &= ~queue_stopped;
		CNTR_INC(priv, queue_start);
	}
};

static inline void qfec_queue_stop(struct net_device *dev)
{
	struct qfec_priv  *priv = netdev_priv(dev);

	netif_stop_queue(dev);
	priv->state |= queue_stopped;
	CNTR_INC(priv, queue_stop);
};

/* -------------------------------------------------------------------------
 * functions to access and initialize the MAC registers
 */
static inline uint32_t qfec_reg_read(struct qfec_priv *priv, uint32_t reg)
{
	return ioread32((void *) (priv->mac_base + reg));
}

/* ----------------------------------------------------- */
static void qfec_reg_write(struct qfec_priv *priv, uint32_t reg, uint32_t val)
{
	uint32_t    addr = (uint32_t)priv->mac_base + reg;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %08x <- %08x\n", __func__, addr, val);
	iowrite32(val, (void *)addr);
}

/* --------------------------------------------------------------------
 * retrieve some frame statistics from controller
 */
static struct qfec_stat {
	int32_t     tx_reg;
	int32_t     rx_reg;
	char       *label;
} qfec_stats[] = {
	{  69,       97,  "good/bad Bytes"   },
	{  89,       98,  "Bytes"   },

	{  70,       96,  "good/bad Frames"   },
	{  71,       99,  "Bcast Frames"   },
	{  72,      100,  "Mcast Frames"   },
	{  90,      113,  "Unicast Frames"   },
	{  -1,      106,  "Jumbo Frames"   },

	{  92,      116,  "Pause Frames"   },
	{  93,      118,  "Vlan Frames"   },

	{  73,      107,  "Frames 64"  },
	{  74,      108,  "Frames 65-127"  },
	{  75,      109,  "Frames 128-255"  },
	{  76,      110,  "Frames 256-511"  },
	{  77,      111,  "Frames 512-1023"  },
	{  78,      112,  "Frames 1024+"  },

	{  -1,      101,  "Crc error Frames"   },
	{  -1,      117,  "Drop due to FIFO Ovrfl"   },
	{  -1,      114,  "Length error Frames"   },
	{  -1,      115,  "Length invalid Frames"   },
	{  -1,      105,  "Runt Frames"   },
};

#define QFEC_REG_OFFSET(reg)    (reg * sizeof(uint32_t))

/* ------------------------------------------------ */
static int qfec_stats_show(char *buf, char **start, off_t offset,
	int count, int *eof, void *data)
{
	struct net_device       *dev  = (struct net_device *) data;
	struct qfec_priv        *priv = netdev_priv(dev);
	struct qfec_stat        *p    = qfec_stats;
	int                      l    = 0;
	int                      n;

	QFEC_LOG(QFEC_LOG_DBG2, "%s:\n", __func__);

	l += sprintf(&buf[l], "%s:\n", __func__);
	l += sprintf(&buf[l], "      %12s  %12s\n", "TX", "RX");

	for (n = ARRAY_SIZE(qfec_stats); n > 0; n--, p++) {
		if ((p->tx_reg > 0) && (p->rx_reg > 0)) {
			l += sprintf(&buf[l], "      %12u  %12u    %s\n",
				qfec_reg_read(priv, QFEC_REG_OFFSET(p->tx_reg)),
				qfec_reg_read(priv, QFEC_REG_OFFSET(p->rx_reg)),
				p->label);
		} else if (p->rx_reg) {
			l += sprintf(&buf[l], "      %12s  %12u    %s\n",
				"",
				qfec_reg_read(priv, QFEC_REG_OFFSET(p->rx_reg)),
				p->label);
		} else {
			l += sprintf(&buf[l], "      %12u  %12s    %s\n",
				qfec_reg_read(priv, QFEC_REG_OFFSET(p->tx_reg)),
				"",
				p->label);
		}
	}

	*eof = 1;
	return l;
}

/* -------------------------------------------------------------------------
 * table and functions to initialize controller registers
 */

struct reg_entry {
	unsigned int  rdonly;
	unsigned int  addr;
	char         *label;
	unsigned int  val;
};

static struct reg_entry  qfec_reg_tbl[] = {
	{ 0, BUS_MODE_REG,           "BUS_MODE_REG",     BUS_MODE_REG_DEFAULT },
	{ 0, AXI_BUS_MODE_REG,       "AXI_BUS_MODE_REG", AXI_BUS_MODE_DEFAULT },
	{ 0, AXI_STATUS_REG,         "AXI_STATUS_REG",     0 },

	{ 0, MAC_ADR_0_HIGH_REG,     "MAC_ADR_0_HIGH_REG", 0x00000302 },
	{ 0, MAC_ADR_0_LOW_REG,      "MAC_ADR_0_LOW_REG",  0x01350702 },

	{ 1, RX_DES_LST_ADR_REG,     "RX_DES_LST_ADR_REG", 0 },
	{ 1, TX_DES_LST_ADR_REG,     "TX_DES_LST_ADR_REG", 0 },
	{ 1, STATUS_REG,             "STATUS_REG",         0 },
	{ 1, DEBUG_REG,              "DEBUG_REG",          0 },

	{ 0, INTRP_EN_REG,           "INTRP_EN_REG",       QFEC_INTRP_SETUP},

	{ 1, CUR_HOST_TX_DES_REG,    "CUR_HOST_TX_DES_REG",    0 },
	{ 1, CUR_HOST_RX_DES_REG,    "CUR_HOST_RX_DES_REG",    0 },
	{ 1, CUR_HOST_TX_BU_ADR_REG, "CUR_HOST_TX_BU_ADR_REG", 0 },
	{ 1, CUR_HOST_RX_BU_ADR_REG, "CUR_HOST_RX_BU_ADR_REG", 0 },

	{ 1, MAC_FR_FILTER_REG,      "MAC_FR_FILTER_REG",      0 },

	{ 0, MAC_CONFIG_REG,         "MAC_CONFIG_REG",    MAC_CONFIG_REG_SPD_1G
							| MAC_CONFIG_REG_DM
							| MAC_CONFIG_REG_TE
							| MAC_CONFIG_REG_RE },

	/* ------------------ */
	{ 1, INTRP_STATUS_REG,       "INTRP_STATUS_REG",   0 },
	{ 1, INTRP_MASK_REG,         "INTRP_MASK_REG",     0 },

	{ 0, OPER_MODE_REG,          "OPER_MODE_REG",  OPER_MODE_REG_DEFAULT },

	{ 1, GMII_ADR_REG,           "GMII_ADR_REG",           0 },
	{ 1, GMII_DATA_REG,          "GMII_DATA_REG",          0 },

	{ 0, MMC_INTR_MASK_RX_REG,   "MMC_INTR_MASK_RX_REG",   0xFFFFFFFF },
	{ 0, MMC_INTR_MASK_TX_REG,   "MMC_INTR_MASK_TX_REG",   0xFFFFFFFF },

	/* ------------------ */
	{ 1, TS_HIGH_REG,            "TS_HIGH_REG",            0 },
	{ 1, TS_LOW_REG,             "TS_LOW_REG",             0 },

	{ 1, TS_HI_UPDT_REG,         "TS_HI_UPDATE_REG",       0 },
	{ 1, TS_LO_UPDT_REG,         "TS_LO_UPDATE_REG",       0 },
	{ 0, TS_SUB_SEC_INCR_REG,    "TS_SUB_SEC_INCR_REG",    112 },

	{ 0, TS_CTL_REG,             "TS_CTL_REG",        TS_CTL_TSENALL
							| TS_CTL_TSCTRLSSR
							| TS_CTL_TSINIT
							| TS_CTL_TSENA },
};

/* ------------------------------------------------ */
static void qfec_reg_init(struct qfec_priv *priv)
{
	struct reg_entry *p = qfec_reg_tbl;
	int         n = ARRAY_SIZE(qfec_reg_tbl);

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	for  (; n--; p++) {
		if (!p->rdonly)
			qfec_reg_write(priv, p->addr, p->val);
	}
}

/*
 * display registers thru proc-fs
 */
static int qfec_reg_show(char *buf, char **start, off_t offset,
	int count, int *eof, void *data)
{
	struct qfec_priv   *priv = netdev_priv((struct net_device *)data);
	struct reg_entry   *p = qfec_reg_tbl;
	int                 n = ARRAY_SIZE(qfec_reg_tbl);
	int                 l = 0;

	QFEC_LOG(QFEC_LOG_DBG2, "%s:\n", __func__);

	for (; n--; p++) {
		l += sprintf(&buf[l], "    %8p   %04x %08x  %s\n",
			(void *)priv->mac_base + p->addr, p->addr,
			qfec_reg_read(priv, p->addr), p->label);
	}

	*eof = 1;
	return  l;
}

/* -------------------------------------------------------------------------
 * set the MAC-0 address
 */
static void qfec_set_adr_regs(struct qfec_priv *priv, uint8_t *addr)
{
	uint32_t        h = 0;
	uint32_t        l = 0;

	h = h << 8 | addr[5];
	h = h << 8 | addr[4];

	l = l << 8 | addr[3];
	l = l << 8 | addr[2];
	l = l << 8 | addr[1];
	l = l << 8 | addr[0];

	qfec_reg_write(priv, MAC_ADR_0_HIGH_REG, h);
	qfec_reg_write(priv, MAC_ADR_0_LOW_REG,  l);

	QFEC_LOG(QFEC_LOG_DBG, "%s: %08x %08x\n", __func__, h, l);
}

/* ------------------------------------------------
 * reset the controller
 */

#define QFEC_RESET_TIMEOUT   10000
	/* reset should always clear but did not w/o test/delay
	 * in RgMii mode.  there is no spec'd max timeout
	 */

static int qfec_hw_reset(struct qfec_priv *priv)
{
	int             timeout = QFEC_RESET_TIMEOUT;

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	qfec_reg_write(priv, BUS_MODE_REG, BUS_MODE_SWR);

	while (qfec_reg_read(priv, BUS_MODE_REG) & BUS_MODE_SWR) {
		if (timeout-- == 0) {
			QFEC_LOG_ERR("%s: timeout\n", __func__);
			return -ETIME;
		}

		/* there were problems resetting the controller
		 * in RGMII mode when there wasn't sufficient
		 * delay between register reads
		 */
		usleep_range(100, 200);
	}

	return 0;
}

/* ------------------------------------------------
 * initialize controller
 */
static int qfec_hw_init(struct qfec_priv *priv)
{
	int  res = 0;

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	res = qfec_hw_reset(priv);
	if (res)
		return res;

	qfec_reg_init(priv);

	/* config buf-desc locations */
	qfec_reg_write(priv, TX_DES_LST_ADR_REG, priv->tbd_dma);
	qfec_reg_write(priv, RX_DES_LST_ADR_REG, priv->rbd_dma);

	/* clear interrupts */
	qfec_reg_write(priv, STATUS_REG, INTRP_EN_REG_NIE | INTRP_EN_REG_RIE
		| INTRP_EN_REG_TIE | INTRP_EN_REG_TUE | INTRP_EN_REG_ETE);

	return res;
}

/* ------------------------------------------------
 * en/disable controller
 */
static void qfec_hw_enable(struct qfec_priv *priv)
{
	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	qfec_reg_write(priv, OPER_MODE_REG,
	qfec_reg_read(priv, OPER_MODE_REG)
		| OPER_MODE_REG_ST | OPER_MODE_REG_SR);
}

static void qfec_hw_disable(struct qfec_priv *priv)
{
	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	qfec_reg_write(priv, OPER_MODE_REG,
	qfec_reg_read(priv, OPER_MODE_REG)
		& ~(OPER_MODE_REG_ST | OPER_MODE_REG_SR));
}

/* -------------------------------------------------------------------------
 * interface selection
 */
struct intf_config  {
	uint32_t     intf_sel;
	uint32_t     emac_ns;
	uint32_t     eth_x_en_ns;
	uint32_t     clkmux_sel;
};

#define ETH_X_EN_NS_REVMII      (ETH_X_EN_NS_DEFAULT | ETH_TX_CLK_INV)
#define CLKMUX_REVMII           (EMAC_CLKMUX_SEL_0 | EMAC_CLKMUX_SEL_1)

static struct intf_config intf_config_tbl[] = {
	{ EMAC_PHY_INTF_SEL_MII,    EMAC_NS_DEFAULT, ETH_X_EN_NS_DEFAULT, 0 },
	{ EMAC_PHY_INTF_SEL_RGMII,  EMAC_NS_DEFAULT, ETH_X_EN_NS_DEFAULT, 0 },
	{ EMAC_PHY_INTF_SEL_REVMII, EMAC_NS_DEFAULT, ETH_X_EN_NS_REVMII,
								CLKMUX_REVMII }
};

/*
 * emac clk register read and write functions
 */
static inline uint32_t qfec_clkreg_read(struct qfec_priv *priv, uint32_t reg)
{
	return ioread32((void *) (priv->clk_base + reg));
}

static inline void qfec_clkreg_write(struct qfec_priv *priv,
	uint32_t reg, uint32_t val)
{
	uint32_t   addr = (uint32_t)priv->clk_base + reg;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %08x <- %08x\n", __func__, addr, val);
	iowrite32(val, (void *)addr);
}

/* -----------------------------------------------
 * configure the PHY interface and clock routing and signal bits
 */
enum phy_intfc  {
	intfc_mii     = 0,
	intfc_rgmii   = 1,
	intfc_revmii  = 2,
};

static int qfec_intf_sel(struct qfec_priv *priv, unsigned int intfc)
{
	struct intf_config   *p;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %d\n", __func__, intfc);

	if (intfc > intfc_revmii)  {
		QFEC_LOG_ERR("%s: range\n", __func__);
		return -ENXIO;
	}

	p = &intf_config_tbl[intfc];

	qfec_clkreg_write(priv, EMAC_PHY_INTF_SEL_REG, p->intf_sel);
	qfec_clkreg_write(priv, EMAC_NS_REG,           p->emac_ns);
	qfec_clkreg_write(priv, ETH_X_EN_NS_REG,       p->eth_x_en_ns);
	qfec_clkreg_write(priv, EMAC_CLKMUX_SEL_REG,   p->clkmux_sel);

	return 0;
}

/* ------------------------------------------------
 * display registers thru proc-fs
 */
static struct qfec_clk_reg {
	uint32_t        offset;
	char           *label;
} qfec_clk_regs[] = {
	{ ETH_MD_REG,                  "ETH_MD_REG"  },
	{ ETH_NS_REG,                  "ETH_NS_REG"  },
	{ ETH_X_EN_NS_REG,             "ETH_X_EN_NS_REG"  },
	{ EMAC_PTP_MD_REG,             "EMAC_PTP_MD_REG"  },
	{ EMAC_PTP_NS_REG,             "EMAC_PTP_NS_REG"  },
	{ EMAC_NS_REG,                 "EMAC_NS_REG"  },
	{ EMAC_TX_FS_REG,              "EMAC_TX_FS_REG"  },
	{ EMAC_RX_FS_REG,              "EMAC_RX_FS_REG"  },
	{ EMAC_PHY_INTF_SEL_REG,       "EMAC_PHY_INTF_SEL_REG"  },
	{ EMAC_PHY_ADDR_REG,           "EMAC_PHY_ADDR_REG"  },
	{ EMAC_REVMII_PHY_ADDR_REG,    "EMAC_REVMII_PHY_ADDR_REG"  },
	{ EMAC_CLKMUX_SEL_REG,         "EMAC_CLKMUX_SEL_REG"  },
};

static int qfec_clk_reg_show(char *buf, char **start, off_t offset,
	int count, int *eof, void *data)
{
	struct qfec_priv        *priv = netdev_priv((struct net_device *)data);
	struct qfec_clk_reg     *p = qfec_clk_regs;
	int                      n = ARRAY_SIZE(qfec_clk_regs);
	int                      l = 0;

	QFEC_LOG(QFEC_LOG_DBG2, "%s:\n", __func__);

	for (; n--; p++) {
		l += sprintf(&buf[l], "    %8p  %8x  %08x  %s\n",
			(void *)priv->clk_base + p->offset, p->offset,
			qfec_clkreg_read(priv, p->offset), p->label);
	}

	*eof = 1;
	return  l;
}

/* -------------------------------------------------------------------------
 * speed selection
 */

struct qfec_pll_cfg {
	uint32_t    spd;
	uint32_t    eth_md;     /* M [31:16], NOT 2*D [15:0] */
	uint32_t    eth_ns;     /* NOT(M-N) [31:16], ctl bits [11:0]  */
};

static struct qfec_pll_cfg qfec_pll_cfg_tbl[] = {
	/* 2.5 MHz */
	{ MAC_CONFIG_REG_SPD_10,   ETH_MD_M(1)  | ETH_MD_2D_N(100),
						  ETH_NS_NM(100-1)
						| ETH_NS_MCNTR_EN
						| ETH_NS_MCNTR_MODE_DUAL
						| ETH_NS_PRE_DIV(0)
						| CLK_SRC_PLL_EMAC },
	/* 25 MHz */
	{ MAC_CONFIG_REG_SPD_100,  ETH_MD_M(1)  | ETH_MD_2D_N(10),
						  ETH_NS_NM(10-1)
						| ETH_NS_MCNTR_EN
						| ETH_NS_MCNTR_MODE_DUAL
						| ETH_NS_PRE_DIV(0)
						| CLK_SRC_PLL_EMAC },
	/* 125 MHz */
	{MAC_CONFIG_REG_SPD_1G,    0,             ETH_NS_PRE_DIV(1)
						| CLK_SRC_PLL_EMAC },
};

enum speed  {
	spd_10   = 0,
	spd_100  = 1,
	spd_1000 = 2,
};

/*
 * configure the PHY interface and clock routing and signal bits
 */
static int qfec_speed_cfg(struct net_device *dev, unsigned int spd,
	unsigned int dplx)
{
	struct qfec_priv       *priv = netdev_priv(dev);
	struct qfec_pll_cfg    *p;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %d spd, %d dplx\n", __func__, spd, dplx);

	if (spd > spd_1000)  {
		QFEC_LOG_ERR("%s: range\n", __func__);
		return -ENODEV;
	}

	p = &qfec_pll_cfg_tbl[spd];

	/* set the MAC speed bits */
	qfec_reg_write(priv, MAC_CONFIG_REG,
	(qfec_reg_read(priv, MAC_CONFIG_REG)
		& ~(MAC_CONFIG_REG_SPD | MAC_CONFIG_REG_DM))
			| p->spd | (dplx ? MAC_CONFIG_REG_DM : 0));

	qfec_clkreg_write(priv, ETH_MD_REG, p->eth_md);
	qfec_clkreg_write(priv, ETH_NS_REG, p->eth_ns);

	return 0;
}

/* -------------------------------------------------------------------------
 * MDIO operations
 * ------------------------------------------------------------------------- */

/*
 * wait reasonable amount of time for MDIO operation to complete, not busy
 */
static int qfec_mdio_busy(struct net_device *dev)
{
	int     i;

	for (i = 100; i > 0; i--)  {
		if (!(qfec_reg_read(
			netdev_priv(dev), GMII_ADR_REG) & GMII_ADR_REG_GB))  {
			return 0;
		}
		udelay(1);
	}

	return -ETIME;
}

/* -----------------------------------------------------
 * initiate either a read or write MDIO operation
 */

static int qfec_mdio_oper(struct net_device *dev, int phy_id, int reg, int wr)
{
	struct qfec_priv   *priv = netdev_priv(dev);
	unsigned long       flags;
	int                 res = 0;

	spin_lock_irqsave(&priv->hw_lock, flags);

	/* insure phy not busy */
	res = qfec_mdio_busy(dev);
	if (res)  {
		QFEC_LOG_ERR("%s: busy\n", __func__);
		goto done;
	}

	/* initiate operation */
	qfec_reg_write(priv, GMII_ADR_REG,
		GMII_ADR_REG_ADR_SET(phy_id)
		| GMII_ADR_REG_REG_SET(reg)
		| GMII_ADR_REG_CSR_SET(priv->mdio_clk)
		| (wr ? GMII_ADR_REG_GW : 0)
		| GMII_ADR_REG_GB);

	/* wait for operation to complete */
	res = qfec_mdio_busy(dev);
	if (res)  {
		QFEC_LOG_ERR("%s: timeout\n", __func__);
		goto done;
	}

done:
	spin_unlock_irqrestore(&priv->hw_lock, flags);
	return res;
}

/* -----------------------------------------------------
 * read MDIO register
 */
static int qfec_mdio_read(struct net_device *dev, int phy_id, int reg)
{
	int     val;
	int	res = 0;

	res = qfec_mdio_oper(dev, phy_id, reg, 0);
	if (res)  {
		QFEC_LOG_ERR("%s: oper\n", __func__);
		return res;
	}

	val = qfec_reg_read(netdev_priv(dev), GMII_DATA_REG);
	QFEC_LOG(QFEC_LOG_DBG2, "%s: %2d reg, 0x%04x val\n",
		__func__, reg, val);

	return val;
}

/* -----------------------------------------------------
 * write MDIO register
 */
static void qfec_mdio_write(struct net_device *dev, int phy_id, int reg,
	int val)
{
	struct qfec_priv   *priv = netdev_priv(dev);

	QFEC_LOG(QFEC_LOG_DBG2, "%s:\n", __func__);

	qfec_reg_write(priv, GMII_DATA_REG, val);

	if (qfec_mdio_oper(dev, phy_id, reg, 1))
		QFEC_LOG_ERR("%s: oper\n", __func__);
}

/* -------------------------------------------------------------------------
 * get auto-negotiation results
 */

#define BMSR_100        (BMSR_100HALF2 | BMSR_100FULL2 | BMSR_100HALF  \
			| BMSR_100FULL  | BMSR_100BASE4)

#define BMSR_100_FD     (BMSR_100FULL2 | BMSR_100FULL | BMSR_100BASE4)

#define BMSR_10         (BMSR_10HALF | BMSR_10FULL)

static void qfec_get_an(struct net_device *dev, uint32_t *spd, uint32_t *dplx)
{
	struct qfec_priv   *priv = netdev_priv(dev);
	uint32_t            status;

	status = qfec_mdio_read(dev, priv->phy_id, MII_BMSR);

	/* todo: check extended status register for 1G abilities */

	if (status & BMSR_100)  {
		*spd  = spd_100;
		*dplx = status & BMSR_100_FD ? 1 : 0;
	}

	else if (status & BMSR_10)  {
		*spd  = spd_10;
		*dplx = status & BMSR_10FULL ? 1 : 0;
	}
}

/*
 * monitor phy status, and process auto-neg results when changed
 */

static void qfec_phy_monitor(unsigned long data)
{
	struct net_device  *dev  = (struct net_device *) data;
	struct qfec_priv   *priv = netdev_priv(dev);
	unsigned int        spd  = 0;
	unsigned int        dplx = 1;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %08x link, %08x carrier\n", __func__,
		mii_link_ok(&priv->mii), netif_carrier_ok(priv->net_dev));

	mod_timer(&priv->phy_tmr, jiffies + HZ);

	if (mii_link_ok(&priv->mii) && !netif_carrier_ok(priv->net_dev))  {
		qfec_get_an(dev, &spd, &dplx);
		qfec_speed_cfg(dev, spd, dplx);
		QFEC_LOG(QFEC_LOG_DBG, "%s: link up, %d spd, %d dplx\n",
			__func__, spd, dplx);

		netif_carrier_on(dev);
	}

	else if (!mii_link_ok(&priv->mii) && netif_carrier_ok(priv->net_dev))  {
		QFEC_LOG(QFEC_LOG_DBG, "%s: link down\n", __func__);
		netif_carrier_off(dev);
	}
}

/* -------------------------------------------------------------------------
 * dealloc buffer descriptor memory
 */

static void qfec_mem_dealloc(struct net_device *dev)
{
	struct qfec_priv   *priv = netdev_priv(dev);
	unsigned int        size = PAGE_SIZE;

	dma_free_coherent(&dev->dev, size, priv->bd_base, priv->tbd_dma);
	priv->bd_base = 0;
}

/* -------------------------------------------------------------------------
 * allocate shared device memory for TX/RX buf-desc (and buffers)
 */
#define RX_TX_BD_RATIO      2
#define MIN_NUM_TX_BD       10

static int qfec_mem_alloc(struct net_device *dev)
{
	struct qfec_priv   *priv = netdev_priv(dev);
	unsigned int        size = PAGE_SIZE;
	unsigned int        unit;

	QFEC_LOG(QFEC_LOG_DBG, "%s: %p dev\n", __func__, dev);

	/* alloc mem for buf-desc, if not already alloc'd */
	if (!priv->bd_base)  {
		priv->bd_base = dma_alloc_coherent(&dev->dev,
			size, &priv->tbd_dma,
			GFP_KERNEL | __GFP_DMA);
	}

	if (!priv->bd_base)  {
		QFEC_LOG_ERR("%s: dma_alloc_coherent failed\n", __func__);
		return -ENOMEM;
	}

	/* determine the space requires per TX BD knowing the
	  * ratio of RX to TX BDs desired, and the BD size
	  */
	unit = (sizeof(struct buf_desc)
	     + (RX_TX_BD_RATIO * sizeof(struct buf_desc)));

	/* determine the number of TX BDs from the available memory and the
	  * unit size calc'd above.  the apply some minimums, compromising
	  * on the RX/TX ratio. report error if minimums not met.
	  */
	priv->n_tbd = size / unit;
	priv->n_tbd = priv->n_tbd < MIN_NUM_TX_BD ? MIN_NUM_TX_BD : priv->n_tbd;

	priv->n_rbd = priv->n_tbd * RX_TX_BD_RATIO;

	QFEC_LOG(QFEC_LOG_DBG,
		" %s: 0x%08x size, %d unit, %d n_tbd, %d n_rbd\n",
		__func__, size, unit, priv->n_tbd, priv->n_rbd);


	if (priv->n_rbd < MIN_NUM_TX_BD) {
		QFEC_LOG_ERR("%s: insufficient memory, %d unit\n",
			__func__, unit);
		return -ENOMEM;
	}

	/* allocate the space */
	priv->p_tbd     = (struct buf_desc *) priv->bd_base;
	QFEC_LOG(QFEC_LOG_DBG2, " %s: %p p_tbd\n", __func__, priv->p_tbd);

	priv->p_rbd     = (struct buf_desc *) &priv->p_tbd[priv->n_tbd];
	QFEC_LOG(QFEC_LOG_DBG2, " %s: %p p_rbd\n", __func__, priv->p_rbd);

	priv->rbd_dma   = priv->tbd_dma
			+ (priv->n_tbd * sizeof(struct buf_desc));

	return 0;
}

/* -------------------------------------------------------------------------
 * display buffer descriptors
 */

static int qfec_bd_fmt(char *buf, struct buf_desc *p_bd)
{
	return sprintf(buf,
		"%8p: %08x %08x %8p %8p  %8p %8p %8p",
		p_bd,                     qfec_bd_status_get(p_bd),
		qfec_bd_ctl_get(p_bd),    qfec_bd_pbuf_get(p_bd),
		qfec_bd_next_get(p_bd),   qfec_bd_skbuf_get(p_bd),
		qfec_bd_virt_get(p_bd),   qfec_bd_phys_get(p_bd));
}

static int qfec_bd_show(char *buf, struct buf_desc *p_bd, int n_bd,
	struct ring *p_ring, char *label)
{
	int     l = 0;
	int     n;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %s\n", __func__, label);

	l += sprintf(&buf[l], "%s: %s\n", __func__, label);
	if (!p_bd)
		return l;

	n_bd = n_bd > MAX_N_BD ? MAX_N_BD : n_bd;

	for (n = 0; n < n_bd; n++, p_bd++) {
		l += qfec_bd_fmt(&buf[l], p_bd);
		l += sprintf(&buf[l], "%s%s\n",
			(qfec_ring_head(p_ring) == n ? " < h" : ""),
			(qfec_ring_tail(p_ring) == n ? " < t" : ""));
	}

	return l;
}

/*
 * display TX BDs
 */
static int qfec_bd_tx_show(char *buf, char **start, off_t offset,
	int count, int *eof, void *data)
{
	struct qfec_priv  *priv = netdev_priv((struct net_device *)data);

	*eof = 1;
	return qfec_bd_show(buf,
	priv->p_tbd, priv->n_tbd, &priv->ring_tbd, "TX");
}

/*
 * display RX BDs
 */
static int qfec_bd_rx_show(char *buf, char **start, off_t offset,
	int count, int *eof, void *data)
{
	struct qfec_priv  *priv = netdev_priv((struct net_device *)data);

	*eof = 1;
	return  qfec_bd_show(buf,
		priv->p_rbd, priv->n_rbd, &priv->ring_rbd, "RX");
}

/* -------------------------------------------------------------------------
 * free transmitted skbufs from buffer-descriptor no owned by HW
 */
static int qfec_tx_replenish(struct net_device *dev)
{
	struct qfec_priv   *priv   = netdev_priv(dev);
	struct ring        *p_ring = &priv->ring_tbd;
	struct buf_desc    *p_bd   = &priv->p_tbd[qfec_ring_tail(p_ring)];
	struct sk_buff     *skb;

	QFEC_LOG(QFEC_LOG_DBG2, "%s:\n", __func__);

	while (!qfec_ring_empty(p_ring))  {
		if (qfec_bd_own(p_bd))
			break;          /* done for now */

		skb = qfec_bd_skbuf_get(p_bd);
		if (skb == NULL)  {
			QFEC_LOG_ERR("%s: null sk_buff\n", __func__);
			CNTR_INC(priv, tx_skb_null);
			return -ENOMEM;
		}

		/* update statistics before freeing skb */
		priv->stats.tx_packets++;
		priv->stats.tx_bytes  += skb->len;

		QFEC_LOG(QFEC_LOG_DBG2, " %s: unmap buf %08x\n", __func__,
						(int) qfec_bd_pbuf_get(p_bd));

		dma_unmap_single(&dev->dev, (int) qfec_bd_pbuf_get(p_bd),
				ETH_BUF_SIZE, DMA_TO_DEVICE);

		QFEC_LOG(QFEC_LOG_DBG2, " %s: free skb %p\n", __func__, skb);
		dev_kfree_skb_any(skb);
		qfec_bd_skbuf_set(p_bd, NULL);


		qfec_ring_tail_adv(p_ring);
		p_bd   = &priv->p_tbd[qfec_ring_tail(p_ring)];
	}

	qfec_queue_start(dev);

	return 0;
}

/* -------------------------------------------------------------------------
 * clear ownership bits of all TX buf-desc and release the sk-bufs
 */
static void qfec_tx_timeout(struct net_device *dev)
{
	struct qfec_priv   *priv   = netdev_priv(dev);
	struct buf_desc    *bd     = priv->p_tbd;
	int                 n;

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);
	CNTR_INC(priv, tx_timeout);

	for (n = 0; n < priv->n_tbd; n++, bd++)
		qfec_bd_own_clr(bd);

	qfec_tx_replenish(dev);
}

/* -------------------------------------------------------------------------
 * rx() - process a received frame
 */
static void qfec_rx_int(struct net_device *dev)
{
	struct qfec_priv   *priv   = netdev_priv(dev);
	struct ring        *p_ring = &priv->ring_rbd;
	struct buf_desc    *p_bd   = &priv->p_rbd[qfec_ring_head(p_ring)];
	struct sk_buff     *skb;

	QFEC_LOG(QFEC_LOG_DBG2, "%s:\n", __func__);
	CNTR_INC(priv, rx_int);

	spin_lock(&priv->hw_lock);

	/* check that valid interrupt occurred */
	if (qfec_bd_own(p_bd))  {
		QFEC_LOG_ERR("%s: owned by DMA\n", __func__);
		CNTR_INC(priv, rx_owned);
		goto done;
	}

	/* process all unowned frames */
	while ((!qfec_bd_own(p_bd)) && (!qfec_ring_full(p_ring)))  {
		skb = qfec_bd_skbuf_get(p_bd);

		if (skb == NULL)  {
			QFEC_LOG_ERR("%s: null sk_buff\n", __func__);
			CNTR_INC(priv, rx_skb_null);
			break;
		}

		else  {
			skb->len = qfec_bd_status_len(p_bd);

			/* update statistics before freeing skb */
			priv->stats.rx_packets++;
			priv->stats.rx_bytes  += skb->len;

			skb->dev        = dev;
			skb->protocol   = eth_type_trans(skb, dev);
			skb->ip_summed  = CHECKSUM_UNNECESSARY;

			dma_unmap_single(&dev->dev,
				(int) qfec_bd_phys_get(p_bd),
				ETH_BUF_SIZE, DMA_FROM_DEVICE);

			if (NET_RX_DROP == netif_rx(skb))  {
				priv->stats.rx_dropped++;
				QFEC_LOG_ERR("%s: dropped\n", __func__);
				CNTR_INC(priv, rx_dropped);
			}
			CNTR_INC(priv, netif_rx_cntr);
		}

		qfec_bd_skbuf_set(p_bd, NULL);

		qfec_ring_head_adv(p_ring);
		p_bd   = &priv->p_rbd[qfec_ring_head(p_ring)];
	}

	/* replenish bufs */
	while (!qfec_ring_empty(p_ring))  {
		if (qfec_rbd_init(dev, &priv->p_rbd[qfec_ring_tail(p_ring)]))
			break;
		qfec_ring_tail_adv(p_ring);
	}

done:
	spin_unlock(&priv->hw_lock);
}

/* -------------------------------------------------------------------------
 * isr() - interrupt service routine
 *          determine cause of interrupt and invoke/schedule appropriate
 *          processing or error handling
 */
#define ISR_ERR_CHK(priv, status, interrupt, cntr) \
	if (status & interrupt) \
		CNTR_INC(priv, cntr)

static irqreturn_t qfec_int(int irq, void *dev_id)
{
	struct net_device  *dev      = dev_id;
	struct qfec_priv   *priv     = netdev_priv(dev);
	uint32_t            status   = qfec_reg_read(priv, STATUS_REG);
	uint32_t            int_bits = STATUS_REG_NIS | STATUS_REG_AIS;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: %s\n", __func__, dev->name);

	/* abnormal interrupt */
	if (status & STATUS_REG_AIS)  {
		QFEC_LOG(QFEC_LOG_DBG, "%s: abnormal status 0x%08x\n",
			__func__, status);

		ISR_ERR_CHK(priv, status, STATUS_REG_RU,  rx_buf_unavail);
		ISR_ERR_CHK(priv, status, STATUS_REG_FBI, fatal_bus);

		ISR_ERR_CHK(priv, status, STATUS_REG_RWT, rx_watchdog);
		ISR_ERR_CHK(priv, status, STATUS_REG_RPS, rx_proc_stopped);
		ISR_ERR_CHK(priv, status, STATUS_REG_UNF, tx_underflow);

		ISR_ERR_CHK(priv, status, STATUS_REG_OVF, rx_overflow);
		ISR_ERR_CHK(priv, status, STATUS_REG_TJT, tx_jabber_tmout);
		ISR_ERR_CHK(priv, status, STATUS_REG_TPS, tx_proc_stopped);

		int_bits |= STATUS_REG_AIS_BITS;
		CNTR_INC(priv, abnorm_int);
	}

	if (status & STATUS_REG_NIS)
		CNTR_INC(priv, norm_int);

	/* receive interrupt */
	if (status & STATUS_REG_RI)  {
		CNTR_INC(priv, rx_isr);
		qfec_rx_int(dev);
		int_bits |= STATUS_REG_ERI | STATUS_REG_RI;
	}

	/* transmit interrupt */
	if (status & STATUS_REG_TI)  {
		int_bits |= STATUS_REG_ETI | STATUS_REG_TU | STATUS_REG_TI;
		CNTR_INC(priv, tx_isr);
	}

	/* gmac interrupt */
	if (status & (STATUS_REG_GPI | STATUS_REG_GMI | STATUS_REG_GLI))  {
		CNTR_INC(priv, gmac_isr);
		int_bits |= STATUS_REG_GMI;
	}

	/* clear interrupts */
	qfec_reg_write(priv, STATUS_REG, int_bits);
	CNTR_INC(priv, isr);

	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------
 * open () - register system resources (IRQ, DMA, ...)
 *   turn on HW, perform device setup.
 */
static int qfec_open(struct net_device *dev)
{
	struct qfec_priv   *priv = netdev_priv(dev);
	struct buf_desc    *p_bd;
	struct ring        *p_ring;
	int                 n;
	int                 res = 0;

	QFEC_LOG(QFEC_LOG_DBG, "%s: %p dev\n", __func__, dev);

	if (!dev)  {
		res = -EINVAL;
		goto err;
	}

	/* allocate TX/RX buffer-descriptors and buffers */

	res = qfec_mem_alloc(dev);
	if (res)
		goto err;

	/* initialize TX */
	for (n = 0, p_bd = priv->p_tbd; n < priv->n_tbd; n++, p_bd++) {
		if (n == (priv->n_tbd - 1))
			qfec_bd_last_bd_set(p_bd);

		qfec_bd_own_clr(p_bd);      /* clear ownership */
	}

	qfec_ring_init(&priv->ring_tbd, priv->n_tbd, priv->n_tbd);

	/* initialize RX buffer descriptors and allocate sk_bufs */
	p_ring = &priv->ring_rbd;
	qfec_ring_init(p_ring, priv->n_rbd, 0);
	qfec_bd_last_bd_set(&priv->p_rbd[priv->n_rbd - 1]);

	for (n = 0, p_bd = priv->p_rbd; n < priv->n_rbd; n++, p_bd++) {
		if (qfec_rbd_init(dev, p_bd))
			break;
		qfec_ring_tail_adv(p_ring);
	}

	/* initialize controller after BDs allocated */
	res = qfec_hw_init(priv);
	if (res)
		goto err1;

	/* get/set (primary) MAC address */
	qfec_set_adr_regs(priv, dev->dev_addr);

	/* configure PHY */
	priv->phy_id = 0;

	qfec_intf_sel(priv, intfc_mii);

	/* start phy monitor */
	QFEC_LOG(QFEC_LOG_DBG, " %s: start timer\n", __func__);
	netif_carrier_off(priv->net_dev);
	setup_timer(&priv->phy_tmr, qfec_phy_monitor, (unsigned long)dev);
	mod_timer(&priv->phy_tmr, jiffies + HZ);

	/* initialize interrupts */
	QFEC_LOG(QFEC_LOG_DBG, " %s: request irq %d\n", __func__, dev->irq);
	res = request_irq(dev->irq, qfec_int, 0, dev->name, dev);
	if (res)
		goto err1;

	/* enable controller */
	qfec_hw_enable(priv);

	QFEC_LOG(QFEC_LOG_DBG, "%s: %08x link, %08x carrier\n", __func__,
		mii_link_ok(&priv->mii), netif_carrier_ok(priv->net_dev));

	QFEC_LOG(QFEC_LOG_DBG, " %s: done\n", __func__);
	return 0;

err1:
	qfec_mem_dealloc(dev);
err:
	QFEC_LOG_ERR("%s: error - %d\n", __func__, res);
	return res;
}

/* -------------------------------------------------------------------------
 * stop() - "reverse operations performed at open time"
 */
static int qfec_stop(struct net_device *dev)
{
	struct qfec_priv   *priv = netdev_priv(dev);
	struct buf_desc    *p_bd;
	struct sk_buff     *skb;
	int                 n;

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	del_timer_sync(&priv->phy_tmr);

	qfec_hw_disable(priv);
	qfec_queue_stop(dev);
	free_irq(dev->irq, dev);

	/* free all pending sk_bufs */
	for (n = priv->n_rbd, p_bd = priv->p_rbd; n > 0; n--, p_bd++) {
		skb = qfec_bd_skbuf_get(p_bd);
		if (skb)
			dev_kfree_skb(skb);
	}

	for (n = priv->n_tbd, p_bd = priv->p_tbd; n > 0; n--, p_bd++) {
		skb = qfec_bd_skbuf_get(p_bd);
		if (skb)
			dev_kfree_skb(skb);
	}

	qfec_mem_dealloc(dev);

	priv->p_tbd = priv->p_rbd = NULL;

	QFEC_LOG(QFEC_LOG_DBG, " %s: done\n", __func__);

	return 0;
}

/* -------------------------------------------------------------------------
 *
 */
static int qfec_set_config(struct net_device *dev, struct ifmap *map)
{
	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);
	return 0;
}

/* -------------------------------------------------------------------------
 * pass data from skbuf to buf-desc
 */
static int qfec_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct qfec_priv   *priv   = netdev_priv(dev);
	struct ring        *p_ring = &priv->ring_tbd;
	struct buf_desc    *p_bd;
	uint32_t            ctrl   = 0;
	int                 ret    = NETDEV_TX_OK;
	unsigned long       flags;

	QFEC_LOG(QFEC_LOG_DBG2, "%s: skb %p\n", __func__, skb);

	CNTR_INC(priv, xmit);

	spin_lock_irqsave(&priv->hw_lock, flags);
	qfec_tx_replenish(dev);

	/* stop queuing if no resources available */
	if (qfec_ring_room(p_ring) == 0)  {
		QFEC_LOG_ERR("%s: no free buf-desc\n", __func__);
		qfec_queue_stop(dev);
		CNTR_INC(priv, tx_no_resource);

		ret = NETDEV_TX_BUSY;
		goto done;
	}

	/* locate and save *sk_buff */
	p_bd = &priv->p_tbd[qfec_ring_head(p_ring)];
	qfec_bd_skbuf_set(p_bd, skb);

	/* set DMA ptr to sk_buff data and write cache to memory */
	qfec_bd_pbuf_set(p_bd, (void *)
	dma_map_single(&dev->dev,
		(void *)skb->data, ETH_BUF_SIZE, DMA_TO_DEVICE));

	ctrl  = skb->len;
	ctrl |= BUF_TX_IC;              /* interrupt on complete */

	if (qfec_bd_last_bd(p_bd))
		ctrl |= BUF_RX_RER;

	/* no gather, no multi buf frames */
	ctrl |= BUF_TX_FS | BUF_TX_LS;  /* 1st and last segment */

	qfec_bd_ctl_wr(p_bd, ctrl);
	qfec_bd_own_set(p_bd);

	qfec_ring_head_adv(p_ring);
	qfec_reg_write(priv, TX_POLL_DEM_REG, 1);      /* poll */

done:
	spin_unlock_irqrestore(&priv->hw_lock, flags);
	return ret;
}

/* -------------------------------------------------------------------------
 *
 */
static int qfec_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct qfec_priv   *priv = netdev_priv(dev);

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	return generic_mii_ioctl(&priv->mii, if_mii(ifr), cmd, NULL);
}

/* -------------------------------------------------------------------------
 *
 */
static struct net_device_stats *qfec_get_stats(struct net_device *dev)
{
	struct qfec_priv   *priv = netdev_priv(dev);

	QFEC_LOG(QFEC_LOG_DBG2, "qfec_stats:\n");

	return &priv->stats;
}

/* -------------------------------------------------------------------------
 * accept new mac address
 */
static int qfec_set_mac_address(struct net_device *dev, void *p)
{
	struct qfec_priv   *priv = netdev_priv(dev);
	struct sockaddr    *addr = p;

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	memcpy(dev->dev_addr, addr->sa_data, dev->addr_len);

	qfec_set_adr_regs(priv, dev->dev_addr);

	return 0;
}

/* -------------------------------------------------------------------------
 * -------------------------------------------------------------------------
 * support creation/removal of qfec proc-fs entries
 */

static struct qfec_proc_entry {
	char           *name;
	read_proc_t    *func;
} qfec_proc_entries[] = {
	{ "bd_tx",   qfec_bd_tx_show      },
	{ "bd_rx",   qfec_bd_rx_show      },
	{ "clk_reg", qfec_clk_reg_show    },
	{ "cntrs",   qfec_cntrs_show      },
	{ "reg",     qfec_reg_show        },
	{ "stats",   qfec_stats_show      },
};

#define QFEC_PROC_ENTRIES  \
	(sizeof(qfec_proc_entries) / sizeof(struct qfec_proc_entry))

static void qfec_proc_fs_create(struct net_device *dev)
{
	struct qfec_proc_entry  *p = qfec_proc_entries;
	int                      n;
	struct proc_dir_entry   *proc_dir = proc_mkdir(dev->name, NULL);

	for (n = QFEC_PROC_ENTRIES; n > 0; n--, p++) {
		remove_proc_entry(p->name, proc_dir);
		create_proc_read_entry(p->name,
					0,          /* default mode */
					proc_dir,   /* parent directory */
					p->func,
					dev);
	}
}

static void qfec_proc_fs_remove(struct net_device *dev)
{
	struct qfec_proc_entry  *p = qfec_proc_entries;
	int                      n;
	char                     s[40];

	for (n = QFEC_PROC_ENTRIES; n > 0; n--, p++) {
		sprintf(s, "%s_%s", dev->name, p->name);
		remove_proc_entry(s, NULL);
	}
}

/* -------------------------------------------------------------------------
 *  read discontinuous MAC address from corrected fuse memory region
 */

static void qfec_get_mac_address(char *buf, char *mac_base, int nBytes)
{
	static int offset[] = { 0, 1, 2, 3, 4, 8 };
	int        n;

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	for (n = 0; n < nBytes; n++)
		*buf++ = ioread8(mac_base + offset[n]);
}

/* -------------------------------------------------------------------------
 * static definition of driver functions
 */
static const struct net_device_ops qfec_netdev_ops = {
	.ndo_open               = qfec_open,
	.ndo_stop               = qfec_stop,
	.ndo_start_xmit         = qfec_xmit,

	.ndo_do_ioctl           = qfec_do_ioctl,
	.ndo_tx_timeout         = qfec_tx_timeout,
	.ndo_set_mac_address    = qfec_set_mac_address,

	.ndo_change_mtu         = eth_change_mtu,
	.ndo_validate_addr      = eth_validate_addr,

	.ndo_get_stats          = qfec_get_stats,
	.ndo_set_config         = qfec_set_config,
};

/*
 * map a specified resource
 */
static int qfec_map_resource(struct platform_device *plat, int resource,
	struct resource **priv_res,
	void                   **addr)
{
	struct resource         *res;

	QFEC_LOG(QFEC_LOG_DBG, "%s: 0x%x resource\n", __func__, resource);

	/* allocate region to access controller registers */
	*priv_res = res = platform_get_resource(plat, resource, 0);
	if (!res) {
		QFEC_LOG_ERR("%s: platform_get_resource failed\n", __func__);
		return -ENODEV;
	}

	res = request_mem_region(res->start, res->end - res->start, QFEC_NAME);
	if (!res) {
		QFEC_LOG_ERR("%s: request_mem_region failed, %08x %08x\n",
			__func__, res->start, res->end - res->start);
		return -EBUSY;
	}

	*addr = ioremap(res->start, res->end - res->start);
	if (!*addr)
		return -ENOMEM;

	QFEC_LOG(QFEC_LOG_DBG, " %s: io mapped from %p to %p\n",
		__func__, (void *)res->start, *addr);

	return 0;
};

/*
 * free allocated io regions
 */
static void qfec_free_res(struct resource *res, void *base)
{

	if (res)  {
		if (base)
			iounmap((void __iomem *)base);

		release_mem_region(res->start, res->end - res->start);
	}
};

/*
 * probe function that obtain configuration info and allocate net_device
 */
static int __devinit qfec_probe(struct platform_device *plat)
{
	struct net_device  *dev;
	struct qfec_priv   *priv;
	int                 ret = 0;

	/* allocate device */
	dev = alloc_etherdev(sizeof(struct qfec_priv));
	if (!dev) {
		QFEC_LOG_ERR("%s: alloc_etherdev failed\n", __func__);
		ret = -ENOMEM;
		goto err;
	}

	QFEC_LOG(QFEC_LOG_DBG, "%s: %08x dev\n",      __func__, (int)dev);

	qfec_dev = dev;
	SET_NETDEV_DEV(dev, &plat->dev);

	dev->netdev_ops      = &qfec_netdev_ops;
	dev->watchdog_timeo  = 2 * HZ;
	dev->irq             = platform_get_irq(plat, 0);

	dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	/* initialize private data */
	priv = (struct qfec_priv *)netdev_priv(dev);
	memset((void *)priv, 0, sizeof(priv));

	priv->net_dev   = dev;
	platform_set_drvdata(plat, dev);

	/* initialize phy structure */
	priv->mii.phy_id_mask   = 0x1F;
	priv->mii.reg_num_mask  = 0x1F;
	priv->mii.dev           = dev;
	priv->mii.mdio_read     = qfec_mdio_read;
	priv->mii.mdio_write    = qfec_mdio_write;

	/* map register regions */
	ret = qfec_map_resource(
		plat, IORESOURCE_MEM, &priv->mac_res, &priv->mac_base);
	if (ret)  {
		QFEC_LOG_ERR("%s: IORESOURCE_MEM mac failed\n", __func__);
		goto err1;
	}

	ret = qfec_map_resource(
		plat, IORESOURCE_IO, &priv->clk_res, &priv->clk_base);
	if (ret)  {
		QFEC_LOG_ERR("%s: IORESOURCE_IO clk failed\n", __func__);
		goto err2;
	}

	ret = qfec_map_resource(
		plat, IORESOURCE_DMA, &priv->fuse_res, &priv->fuse_base);
	if (ret)  {
		QFEC_LOG_ERR("%s: IORESOURCE_DMA fuse failed\n", __func__);
		goto err3;
	}

	/* initialize MAC addr */
	qfec_get_mac_address(dev->dev_addr, priv->fuse_base, MAC_ADDR_SIZE);

	QFEC_LOG(QFEC_LOG_DBG, "%s: mac  %02x:%02x:%02x:%02x:%02x:%02x\n",
		__func__,
		dev->dev_addr[0], dev->dev_addr[1],
		dev->dev_addr[2], dev->dev_addr[3],
		dev->dev_addr[4], dev->dev_addr[5]);

	ret = register_netdev(dev);
	if (ret)  {
		QFEC_LOG_ERR("%s: register_netdev failed\n", __func__);
		goto err4;
	}

	spin_lock_init(&priv->hw_lock);
	qfec_proc_fs_create(dev);

	return 0;

	/* error handling */
err4:
	qfec_free_res(priv->fuse_res, priv->fuse_base);
err3:
	qfec_free_res(priv->clk_res, priv->clk_base);
err2:
	qfec_free_res(priv->mac_res, priv->mac_base);
err1:
	free_netdev(dev);
err:
	QFEC_LOG_ERR("%s: err\n", __func__);
	return ret;
}

/* -------------------------------------------------------------------------
 * module remove
 */
static int __devexit qfec_remove(struct platform_device *plat)
{
	struct net_device  *dev  = platform_get_drvdata(plat);
	struct qfec_priv   *priv = netdev_priv(dev);

	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	platform_set_drvdata(plat, NULL);

	qfec_free_res(priv->fuse_res, priv->fuse_base);
	qfec_free_res(priv->clk_res, priv->clk_base);
	qfec_free_res(priv->mac_res, priv->mac_base);

	unregister_netdev(dev);
	free_netdev(dev);

	qfec_proc_fs_remove(dev);

	return 0;
}

/* -------------------------------------------------------------------------
 * module support
 *     the FSM9xxx is not a mobile device does not support power management
 */

static struct platform_driver qfec_driver = {
	.probe  = qfec_probe,
	.remove = __devexit_p(qfec_remove),
	.driver = {
		.name   = QFEC_NAME,
		.owner  = THIS_MODULE,
	},
};

/* ----------------------------
 * module init
 */
static int __init qfec_init_module(void)
{
	int  res;

	QFEC_LOG(QFEC_LOG_DBG, "%s: %s\n", __func__, qfec_driver.driver.name);

	res = platform_driver_register(&qfec_driver);

	QFEC_LOG(QFEC_LOG_DBG, "%s: %d - platform_driver_register\n",
		__func__, res);

	return  res;
}

/* ----------------------------
 * module exit
 */
static void __exit qfec_exit_module(void)
{
	QFEC_LOG(QFEC_LOG_DBG, "%s:\n", __func__);

	platform_driver_unregister(&qfec_driver);
}

MODULE_DESCRIPTION("FSM Network Driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Rick Adams <rgadams@codeaurora.org>");
MODULE_VERSION("1.0");

module_init(qfec_init_module);
module_exit(qfec_exit_module);
