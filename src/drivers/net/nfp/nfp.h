#ifndef _NFP_H
#define _NFP_H

#include <ipxe/iobuf.h>
#include <ipxe/if_ether.h>

#include "nfp_net_ctrl.h"

/** @file
 *
 * NFP network driver
 *
 */

//FILE_LICENCE ( );

/** NFP BAR size */
#define NFP_BAR_SIZE 128

/* Vendor ID */
#define NETRONOME_VENDOR_ID 0x19EE

/* Device ID */
#define NETRONOME_DEVICE_ID_6000 0x6000
#define NETRONOME_DEVICE_ID_5000 0x5000
#define NETRONOME_DEVICE_ID_4000 0x4000
#define NETRONOME_DEVICE_ID_3800 0x3800

/* Bit fields for the PCI config command. */
//#define PCI_COMMAND_MASTER  0x04
//#define PCI_COMMAND_IO      0x01
//#define PCI_COMMAND         0x04
//#define PCI_LATENCY_TIMER   0x0D

/* Generate a hex value with specified bit set. */
#define BIT(nr)			(1UL << (nr))

/* Gnerate a mask based on a range between specified low and high bits. */
#define GENMASK(h, l) \
	((~0ULL << (l)) & (~0ULL >> (64 - (h) - 1)))

/* Maximum number of interfaces supported by this driver */
#define MAX_INTERFACES   8
#define MAX_NSP_ETH_TABLE_ENTRIES   24

/* Indicating how many vNICs are supported. */
#define MAX_SUPPORT_VNICS   MAX_INTERFACES

/* Indicating the number of PCIe islands supported. */
#define MAX_SUPPORT_PCIE_ISLANDS   4

/* Device BAR exposed */
#define NETRONOME_DEV_BAR_0   0
#define NETRONOME_DEV_BAR_1   1
#define NETRONOME_DEV_BAR_2   2

/* Each BAR is subdivided into 8 slices. */
#define NETRONOME_SLICES_PER_BAR 8

/* Defines the expansion BAR number, 0 to 7. There is an expansion BAR per */
/* physical BAR, so each permutation of expansion BAR number with physical BAR */
/* is unique. */
#define NETRONOME_EXP_BAR_0      0
#define NETRONOME_EXP_BAR_1      1
#define NETRONOME_EXP_BAR_2      2
#define NETRONOME_EXP_BAR_3      3
#define NETRONOME_EXP_BAR_4      4
#define NETRONOME_EXP_BAR_7      7

/* Each BAR is subdivided into 1/8 of the BAR size. This is the offset required */
/* to reach the next section on the respective BAR */
#define NETRONOME_DEV_BAR_0_BITSIZE   (27 - 3)
#define NETRONOME_DEV_BAR_1_BITSIZE   (26 - 3)
#define NETRONOME_DEV_BAR_2_BITSIZE   (24 - 3)

#define NETRONOME_DEV_BAR_0_SLICE     (1 << NETRONOME_DEV_BAR_0_BITSIZE)
#define NETRONOME_DEV_BAR_1_SLICE     (1 << NETRONOME_DEV_BAR_1_BITSIZE)
#define NETRONOME_DEV_BAR_2_SLICE     (1 << NETRONOME_DEV_BAR_2_BITSIZE)

/* Here we define the BAR mappings used for the UNDI driver. This is per PCIe, */
/* so it doesn't matter what other drivers are handling other PCIe devices. */
/* We leave BAR 0.0 as is, since that should be initialized to give us access */
/* to CSRs etc. needed to program the other BARs. */
/* ExpBAR is PCIInternal Target starting at 0x30000 */

#define NETRONOME_DEV_BAR_0_1_MAP         0x68000000

/* Set BAR_0.3 to i48.ctm (StubROM only) */
#define NETRONOME_DEV_BAR_0_3_MAP         0x2b960000

/* Set BAR 1.0 to access the resource table at NFP_RESOURCE_TBL_BASE for an */
/* atomic read instruction. */
#define NETRONOME_DEV_BAR_1_0_MAP          0x3838100

/* Set BAR 1.1 to access the resource table at NFP_RESOURCE_TBL_BASE for a */
/* test_set_imm instruction. */
#define NETRONOME_DEV_BAR_1_1_MAP          0x3e58100

/* Set BAR 1.2 to access the resource table at NFP_RESOURCE_TBL_BASE for an */
/* atomic write instruction */
#define NETRONOME_DEV_BAR_1_2_MAP          0x3848100

/* Set BAR_2.0 to i4.ctm (StubROM only) */
#define NETRONOME_DEV_BAR_2_0_MAP         0x27820000

/* Set BAR_2.1 to access QCP (UNDI only) */
#define NETRONOME_DEV_BAR_2_1_MAP_UNDI    0x20000000

/* Number of IO Reads/Writes */
#define READ_WIDTH_ONCE        1
#define WRITE_WIDTH_ONCE       1
#define READ_WIDTH_TWICE       2
#define WRITE_WIDTH_TWICE      2
#define READ_WIDTH_FOUR_TIMES  4
#define WRITE_WIDTH_FOUR_TIMES 4

/* PF vNIC MAC address HwInfo key parameters */
#define VNIC_MAC_ADDR_KEY_MAXSIZE   32
#define VNIC_MAC_ADDR_KEY           "eth%u.mac.pf%u"

/* Common queue controller peripheral ring structure. */
struct qcp_ring {
	/* Pointer to base of the QCP queue */
	u32 qptr;

	/* Size in number of descriptors */
	u16 count;

	/* Local queue write pointer. */
	u32 writeptr;

	/* Local queue read pointer. */
	u32 readptr;

	/* Device DMA address for the ring. */
	physaddr_t ringdmaaddr;

	/* DMA mapping handle, needed to unmap again. */
	struct dma_mapping *map;

	/* Host buffer for ring */
	char *ringdmahostbuffer;

	/* Host buffer size in pages. Needed for free */
	u64 hostbuffersize;

};

struct buffer {
	/* Stash host buffer addresses. Need to give this back to the PXE layer. */
	u64 hostbufferaddr;
	struct io_buffer *iobuf;

	/* Device DMA address for the buffer. */
	u64 dmaaddr;

	/* DMA mapping handle, needed to unmap again. */
	struct dma_mapping *map;

	/* Keep track of DMA mapping state to free correct entries */
	int ismapped;

};

/* Transmit ring structure */
struct tx_ring {
	/* QCP ring structure for transmitting packets. */
	struct qcp_ring ring;

	/* List of buffers used by device. Required to hand buffers back to PXE layer. */
	struct buffer *freelist;

	/* Size of the Tx buffer list in pages. Needed for free. */
	u64 freelistsize;
};

/* Receive ring structure */
struct rx_ring {
	/* QCP ring structure for receiving packets. */
	struct qcp_ring ring;

	/* List of buffers used by device. */
	struct buffer *freelist;

	/* Size of the free list in pages. Needed for free. */
	u64 freelistsize;

	/* Store the datapath RX offset. */
	u32 rxoffset;

};

struct config_queue {
	/* Pointer to base of the QCP queue */
	u32 qptr;

};


/* NFD version number field. */
struct nfd_ver {
	/* Minor NFD version number */
	char verminor;

	/* Major NFD version number */
	char vermajor;

	/* NFD version class */
	char verclass;

	/* Reserved field */
	char verreserved;

};

/* Device Statistics Information. */
struct Statistics {
	/* TX Packet Statistics. */
	u64 txpackets;

	/* TX Octet Statistics. */
	u64 txoctets;

	/* RX Packet Statistics. */
	u64 rxpackets;

	/* RX Octet Statistics. */
	u64 rxoctets;
};

/* Device Information. */
struct dev_info {
	/* Interface Number. */
	u32 interfacenum;

	/* Interface Index. */
	u32 interfaceindex;

	/* LInk Speed Identifier. */
	u32 linkspeed;

	/* MTU Size. */
	u32 mtusize;

	/* Current Link Status. */
	u32 linkstatus;

	/* Human Readable Link Speed. */
	u32 linkspeedvalue;

	/* State of the interface */
	u32 adminstate;
};
/**
 * struct nfp_bar - describes BAR configuration and usage
 * @nfp:        backlink to owner
 * @barcfg:     cached contents of BAR config CSR
 * @base:       the BAR's base CPP offset
 * @mask:       mask for the BAR aperture (read only)
 * @bitsize:    bitsize of BAR aperture (read only)
 * @index:      index of the BAR
 * @refcnt:     number of current users
 * @iomem:      mapped IO memory
 * @resource:   iomem resource window
 */
struct nfp_bar {
	u32 barcfg;
	u64 base;          /* CPP address base */
	u64 mask;          /* Bit mask of the bar */
	u32 bitsize;       /* Bit size of the bar */
	int index;

	void *iomem;
};

/* Netronome NIC Info structure containing all required information */
/* for a single interface. */

/** NFP network card */
struct nfp_adapter {

	struct net_device *netdev;
	struct pci_device *pdev;
	struct dma_device *dma;

	struct nfp_bar bar[3 * 8];

	u8 hw_addr[6];
	uint32_t ioaddr;

	/* The current PXE state */
	u64  state;

	/* Store the PCIe island number */
	char pcie_island;

	/* Device Identifiers. */
	u16 vendor_id;
	u16 device_id;
	u16 sub_vendor_id;
	u16 sub_system_id;
	u8 rev_id;

	/* PCI Config Information */
	void *config;

#ifndef NETRONOME_STUBROM

	/* Device Location Information. */
	u64  bus;
	u64  device;
	u64  function;

	/* Interface Info */
	struct dev_info device_info;

	/* Device Firmware Version Info */
	struct nfd_ver version;

	/* The number of supported ports/vNICs, i.e. nfd_cfg_pX_num_ports */
	u32 vniccount;

	/* Offset of vNIC ctrl BAR structure, i.e. _pfX_net_bar0 */
	u32 ctrlbar[MAX_SUPPORT_VNICS];

	/* Reconfiguration ring */
	struct config_queue cfg_queue;

	/* Device TX ring info per vNIC. We only support a single ring per vNIC. */
	struct tx_ring tx_ring[MAX_SUPPORT_VNICS];

	/* Device RX ring info per vNIC. We only support a single ring per vNIC. */
	struct rx_ring rx_ring[MAX_SUPPORT_VNICS];

	/* Memory required for RX buffers */
	u32 RxMemRequired;

	/* Device Statistics Location */
	u32 stats_offset;

	/* Current Statistics State */
	struct Statistics stats;

	/* Previous Statistics State */
	struct Statistics stats_state;

	/* MAC address fields required by PXE layer */
	char PermNodeAddress[ETH_ALEN];
	char CurrentNodeAddress[ETH_ALEN];
	char BroadcastNodeAddress[ETH_ALEN];

	/* ID Returned from PXE layer */
	u64 UniqueID;
#endif
} __attribute__ ((aligned));

/* Link speed options */
#define DEV_SUPP_SPEED_NOT_VALID 0
#define DEV_SUPP_SPEED_1G        1
#define DEV_SUPP_SPEED_10G       10
#define DEV_SUPP_SPEED_25G       25
#define DEV_SUPP_SPEED_40G       40
#define DEV_SUPP_SPEED_50G       50
#define DEV_SUPP_SPEED_100G      100

/* Link speed values */
#define DEV_LINK_SPEED_NOT_VALID 0
#define DEV_LINK_SPEED_1G        1
#define DEV_LINK_SPEED_10G       10
#define DEV_LINK_SPEED_25G       25
#define DEV_LINK_SPEED_40G       40
#define DEV_LINK_SPEED_50G       50
#define DEV_LINK_SPEED_100G      100
#define PXE_LINK_SPEED_1G        (DEV_LINK_SPEED_1G   * 1000)
#define PXE_LINK_SPEED_10G       (DEV_LINK_SPEED_10G  * 1000)
#define PXE_LINK_SPEED_25G       (DEV_LINK_SPEED_25G  * 1000)
#define PXE_LINK_SPEED_40G       (DEV_LINK_SPEED_40G  * 1000)
#define PXE_LINK_SPEED_50G       (DEV_LINK_SPEED_50G  * 1000)
#define PXE_LINK_SPEED_100G      (DEV_LINK_SPEED_100G * 1000)

/* Split addresses between low and high bits */
#define UPPER_8_BITS(Value) (((Value) >> 32) & 0xff)
#define LOWER_32_BITS(Value) ((Value) & 0xffffffff)

// Converts the relative config offset into the real BAR offset for a specified
// vNIC.
#define VNIC_CTRL_ADDR(Dev, VnicIndex, Offset) \
	(u64)((Dev)->ctrlbar[(VnicIndex)] + (Offset))

// Converts the relative QCP offset into the real BAR offset. Note that the QCP
// is accessible on BAR2.1.
#define QCP(Ring, Offset) ((Ring)->qptr + (Offset) + NETRONOME_DEV_BAR_2_SLICE)

/* Total octets in header */
#define ETH_HLEN	14

/* The MTU Size. */
#define MTU_SIZE         1500

/* TX total buffer size. We don't support VLAN. */
#define TX_BUF_SIZE      (NFP_NET_MAX_PREPEND + ETH_HLEN + MTU_SIZE)

/* RX total buffer size. We don't support VLAN. */
#define RX_BUF_SIZE      (NFP_NET_MAX_PREPEND + ETH_HLEN + MTU_SIZE)

/* NFD Meta data mask field */
#define NFP_NET_META_FIELD_MASK GENMASK(NFP_NET_META_FIELD_SIZE - 1, 0)

/* Checksum Status Flags. */
#define L3_CSUM_IPV4_ERROR  0x02
#define L4_CSUM_TCP_ERROR   0x03
#define L4_CSUM_UDP_ERROR   0x05

/* Shift required to read L3 checksum status. */
#define L3_STATUS_SHIFT 4

/* Mask required to read L3 checksum status. */
#define L3_STATUS_MASK  0x3

/* Read the L3 checksum status. */
#define CSUM_L3_STATUS(DeviceHeader) (((DeviceHeader) >> L3_STATUS_SHIFT) & L3_STATUS_MASK)

/* Shift required to read L4 checksum status. */
#define L4_STATUS_SHIFT 5

/* Mask required to read L4 checksum status. */
#define L4_STATUS_MASK  0x7

/* Read the L4 checksum status. */
#define CSUM_L4_STATUS(DeviceHeader) (((DeviceHeader) >> L4_STATUS_SHIFT) & L4_STATUS_MASK)

/* Get the buffer address for a defined index. */
#define GET_RX_BUF_ADDRESS(RxBufBase, Index) ((RxBufBase) + ((Index) * RX_BUF_SIZE))

/* NFD version field mask. Each field is only 8 bits. */
#define NFD_VERSION_MASK 0xFF

/* NFD minor version shift. */
#define NFD_VERSION_MINOR_SHIFT 0

/* NFD major version shift. */
#define NFD_VERSION_MAJOR_SHIFT 8

/* NFD version class shift. */
#define NFD_VERSION_CLASS_SHIFT 16

/* NFD version reserved shift. */
#define NFD_VERSION_RESV_SHIFT 24

/* Read NFD minor version. */
#define NFD_VERSION_MINOR(VersionNumber) \
	(((VersionNumber) >> NFD_VERSION_MINOR_SHIFT) & NFD_VERSION_MASK)

/* Read NFD minor version. */
#define NFD_VERSION_MAJOR(VersionNumber) \
	(((VersionNumber) >> NFD_VERSION_MAJOR_SHIFT) & NFD_VERSION_MASK)

/* Read NFD minor version. */
#define NFD_VERSION_CLASS(VersionNumber) \
	(((VersionNumber) >> NFD_VERSION_CLASS_SHIFT) & NFD_VERSION_MASK)

/* Read NFD version reserved. */
#define NFD_VERSION_RESV(VersionNumber) \
	(((VersionNumber) >> NFD_VERSION_RESV_SHIFT) & NFD_VERSION_MASK)

/* Device information start byte shift. */
#define DEVINFO_START_BYTE_SHIFT 0

/* Device information start byte mask. */
#define DEVINFO_START_BYTE_MASK 0xFF

/* Device information interface num shift. */
#define DEVINFO_INT_NUM_SHIFT 8

/* Device information interface num mask. */
#define DEVINFO_INT_NUM_MASK 0x1F

/* Device information link speed shift. */
#define DEVINFO_LINK_SPEED_SHIFT 13

/* Device information link speed mask. */
#define DEVINFO_LINK_SPEED_MASK 0x7

/* Device information MTU size shift. */
#define DEVINFO_MTU_SIZE_SHIFT 16

/* Device information MTU size mask. */
#define DEVINFO_MTU_SIZE_MASK 0x7FFF

/* Device information link status shift. */
#define DEVINFO_LINK_STATUS_SHIFT 31

/* Device information link status mask. */
#define DEVINFO_LINK_STATUS_MASK 0x1

/* Read device information start byte. */
#define DEVINFO_START_BYTE(dev_info) (((dev_info) >> DEVINFO_START_BYTE_SHIFT) & DEVINFO_START_BYTE_MASK)

/* Read device information interface number. */
#define DEVINFO_INT_NUM(dev_info) (((dev_info) >> DEVINFO_INT_NUM_SHIFT) & DEVINFO_INT_NUM_MASK)

/* Read device information link speed. */
#define DEVINFO_LINK_SPEED(dev_info) (((dev_info) >> DEVINFO_LINK_SPEED_SHIFT) & DEVINFO_LINK_SPEED_MASK)

/* Read device information MTU size. */
#define DEVINFO_MTU_SIZE(dev_info) (((dev_info) >> DEVINFO_MTU_SIZE_SHIFT) & DEVINFO_MTU_SIZE_MASK)

/* Read device information link status. */
#define DEVINFO_LINK_STATUS(dev_info) (((dev_info) >> DEVINFO_LINK_STATUS_SHIFT) & DEVINFO_LINK_STATUS_MASK)

/* Convenience macro for wrapping descriptor index on ring size */
#define D_IDX(ring, idx)	((idx) & ((ring)->count - 1))

/* Max time to wait for NFP to respond on updates (loop cycles) */
#define NFP_NET_POLL_TIMEOUT	1000

/* Maximum number of bytes prepended to a packet */
#define NFP_NET_MAX_PREPEND		64

/* Queue/Ring definitions */
#define NFP_NET_TX_DESCS_UNDI    128
#define NFP_NET_RX_DESCS_UNDI    128

/* TX descriptor format */
#define PCIE_DESC_TX_EOP		BIT(7)
#define PCIE_DESC_TX_OFFSET_MASK	GENMASK(6, 0)
#define PCIE_DESC_TX_MSS_MASK		GENMASK(13, 0)

/* Flags in the host TX descriptor */
#define PCIE_DESC_TX_CSUM		BIT(7)
#define PCIE_DESC_TX_IP4_CSUM		BIT(6)
#define PCIE_DESC_TX_TCP_CSUM		BIT(5)
#define PCIE_DESC_TX_UDP_CSUM		BIT(4)
#define PCIE_DESC_TX_VLAN		BIT(3)
#define PCIE_DESC_TX_LSO		BIT(2)
#define PCIE_DESC_TX_ENCAP		BIT(1)
#define PCIE_DESC_TX_O_IP4_CSUM	BIT(0)

/* RX and freelist descriptor format */
#define PCIE_DESC_RX_DD			BIT(7)
#define PCIE_DESC_RX_META_LEN_MASK	GENMASK(6, 0)

/* Flags in the RX descriptor */
#define PCIE_DESC_RX_RSS		cpu_to_le16(BIT(15))
#define PCIE_DESC_RX_I_IP4_CSUM		cpu_to_le16(BIT(14))
#define PCIE_DESC_RX_I_IP4_CSUM_OK	cpu_to_le16(BIT(13))
#define PCIE_DESC_RX_I_TCP_CSUM		cpu_to_le16(BIT(12))
#define PCIE_DESC_RX_I_TCP_CSUM_OK	cpu_to_le16(BIT(11))
#define PCIE_DESC_RX_I_UDP_CSUM		cpu_to_le16(BIT(10))
#define PCIE_DESC_RX_I_UDP_CSUM_OK	cpu_to_le16(BIT(9))
#define PCIE_DESC_RX_BPF		cpu_to_le16(BIT(8))
#define PCIE_DESC_RX_EOP		cpu_to_le16(BIT(7))
#define PCIE_DESC_RX_IP4_CSUM		cpu_to_le16(BIT(6))
#define PCIE_DESC_RX_IP4_CSUM_OK	cpu_to_le16(BIT(5))
#define PCIE_DESC_RX_TCP_CSUM		cpu_to_le16(BIT(4))
#define PCIE_DESC_RX_TCP_CSUM_OK	cpu_to_le16(BIT(3))
#define PCIE_DESC_RX_UDP_CSUM		cpu_to_le16(BIT(2))
#define PCIE_DESC_RX_UDP_CSUM_OK	cpu_to_le16(BIT(1))
#define PCIE_DESC_RX_VLAN		cpu_to_le16(BIT(0))

#define PCIE_DESC_RX_CSUM_ALL		(PCIE_DESC_RX_IP4_CSUM |	\
		PCIE_DESC_RX_TCP_CSUM |	\
		PCIE_DESC_RX_UDP_CSUM |	\
		PCIE_DESC_RX_I_IP4_CSUM |	\
		PCIE_DESC_RX_I_TCP_CSUM |	\
		PCIE_DESC_RX_I_UDP_CSUM)
#define PCIE_DESC_RX_CSUM_OK_SHIFT	1
#define __PCIE_DESC_RX_CSUM_ALL		le16_to_cpu(PCIE_DESC_RX_CSUM_ALL)
#define __PCIE_DESC_RX_CSUM_ALL_OK	(__PCIE_DESC_RX_CSUM_ALL >>	\
		PCIE_DESC_RX_CSUM_OK_SHIFT)

/* Queue Controller Peripheral access functions and definitions. */
/* Some of the BARs of the NFP are mapped to portions of the Queue */
/* Controller Peripheral (QCP) address space on the NFP.  A QCP queue */
/* has a read and a write pointer (as well as a size and flags, */
/* indicating overflow etc).  The QCP offers a number of different */
/* operation on queue pointers, but here we only offer function to */
/* either add to a pointer or to read the pointer value. */
#define NFP_QCP_QUEUE_ADDR_SZ			0x800
#define NFP_QCP_QUEUE_AREA_SZ			0x80000
#define NFP_QCP_QUEUE_OFF(_x)			((_x) * NFP_QCP_QUEUE_ADDR_SZ)
#define NFP_QCP_QUEUE_ADD_RPTR			0x0000
#define NFP_QCP_QUEUE_ADD_WPTR			0x0004
#define NFP_QCP_QUEUE_STS_LO			0x0008
#define NFP_QCP_QUEUE_STS_LO_READPTR_mask	0x3ffff
#define NFP_QCP_QUEUE_STS_HI			0x000c
#define NFP_QCP_QUEUE_STS_HI_WRITEPTR_mask	0x3ffff

/* The offset of a QCP queues in the PCIe Target */
#define NFP_PCIE_QUEUE(_q) (0x80000 + (NFP_QCP_QUEUE_ADDR_SZ * ((_q) & 0xff)))

/* Read or Read Pointer of a queue */
#define NFP_QCP_READ_PTR 0

/* Read or Write Pointer of a queue */
#define NFP_QCP_WRITE_PTR 1

/* Slice size for each vNIC of the NFD BAR */
#define NFP_PF_CSR_SLICE_SIZE	(32 * 1024)

#pragma pack(1)

/* TX descriptor structure. */
struct tx_desc {
	/* High bits of host buf address */
	u8 DmaAddrHi;

	/* Length to DMA for this desc */
	u16 DmaLen;

	/* Offset in buf where pkt starts highest bit is eop flag. */
	u8 OffsetEop;

	/* Low 32bit of host buf addr */
	u32 DmaAddrLo;

	/* MSS to be used for LSO */
	u16 Mss;

	/* LSO, TCP payload offset */
	u8 LsoHdrlen;

	/* TX Flags, see @PCIE_DESC_TX_* */
	u8 Flags;

	/* VLAN tag to add if indicated */
	u16 Vlan;

	/* Length of frame + meta data */
	u16 DataLen;

};

struct fl_desc {
	/* High bits of the buf address */
	u8 DmaAddrHi;

	/* Must be zero */
	u16 Reserved;

	/* Must be zero */
	u8 MetaLenDd;

	/* Low bits of the buffer address */
	u32 DmaAddrLo;

};

struct rx_desc {
	/* Length of the frame + meta data */
	u16 DataLen;

	/* Reserved */
	u8 Reserved;

	/* Length of meta data prepended + descriptor done flag. */
	u8 MetaLenDd;

	/* RX flags. See @PCIE_DESC_RX_* */
	u16 Flags;

	/* VLAN if stripped */
	u16 Vlan;

};

#define NFP_RESOURCE_ENTRY_NAME_SZ      8
struct nsp_rtsym_response {
	// Target memory type
	int type;

	// CPP target number
	int target;

	// Domain ID (island)
	int domain;

	// CPP address offset
	u64 addr;

	// Size of the symbol
	u64 size;
};

struct nfp_resource_entry {
	struct nfp_resource_entry_mutex {
		u32 owner;
		u32 key;
	} mutex;
	struct nfp_resource_entry_region {
		u8  name[NFP_RESOURCE_ENTRY_NAME_SZ];
		u8  reserved[5];
		u8  cpp_action;
		u8  cpp_token;
		u8  cpp_target;
		u32 page_offset;
		u32 page_size;
	} region;
};

#define NFP_CPP_ACTION_RW               32

#define NFP_COMPUTE_BAR_WIDTH_32        4
#define NFP_COMPUTE_BAR_WIDTH_64        8

#define NFP_RESOURCE_TBL_SIZE           4096
#define NFP_RESOURCE_TBL_ENTRIES        (NFP_RESOURCE_TBL_SIZE /        \
		sizeof(struct nfp_resource_entry))
#pragma pack(0)

#define NSP_CSR_CPP_BASE      0x0000004000
#define NSP_CSR_LENGTH        (sizeof(u64) * 4)

/// Resouce table parameters
#define NFP_RESOURCE_TBL_TARGET		NFP_CPP_TARGET_MU
#define NFP_RESOURCE_TBL_BASE		0x8100000000ULL

/// The nfp.sp resource name
#define RESOURCE_NFPSP_NAME "nfp.sp"

/// This is the crc32 of the resouce name, i.e. nfp.sp
#define RESOURCE_NFPSP_KEY 0xdf97c080

// Define the addresses based on the BAR mapping configuration
#define NFP_RESOURCE_MUR_ADDR(Offset) (Offset)
#define NFP_RESOURCE_MUW_ADDR(Offset) ((Offset) + 2 * NETRONOME_DEV_BAR_1_SLICE)
#define NFP_RESOURCE_MUS_ADDR(Offset) ((Offset) + NETRONOME_DEV_BAR_1_SLICE)

/// The lock word comprises of the owner ID, which is generally 0x<Island>ff
/// where Island is the PCIe number, along with the 0xf if locked.

#define MUTEX_LOCK(Island) \
	(((u32)(((Island) << 8) | 0xff) << 16) | 0x000f)

#define MUTEX_UNLOCK(Island) \
	(((u32)(((Island) << 8) | 0xff) << 16) | 0x0000)

#define MUTEX_IS_LOCKED(Owner) \
	(((Owner) & 0xffff) == 0x000f)

#define MUTEX_IS_UNLOCKED(Owner) \
	(((Owner) & 0xffff) == 0x0000)

#define MUTEX_OWNER(val) \
	(((val) >>16) & 0xffff)

/// Internal temporary rtsym lookup buffer
#define RTSYM_LOOKUP_BUFFER_SIZE 128

/// The following defines have been taken from the kernel driver in order to
/// translate the rtsym location into appropriate CPP address/target fields.
#define NFP_CPP_TARGET_MU               7
#define NFP_RTSYM_TARGET_EMU_CACHE     -7
#define NFP_RTSYM_TYPE_OBJECT           1

#define NFP_ISLAND_EMU_MIN              24
#define NFP_ISLAND_EMU_MAX              27

#define NFP_MU_ISLAND_LSB               32
#define NFP_MU_ISLAND_DIRECT_LSB        35
#define NFP_MU_LOCALITY_LSB             38
#define NFP_MU_ADDR_ACCESS_TYPE_MASK	3ULL
#define NFP_MU_ADDR_ACCESS_TYPE_DIRECT	2ULL

#define NFP_MU_ADDR_ISLAND_INDEX0       (0ULL << 37)
#define NFP_MU_ADDR_ISLAND_INDEX1       (1ULL << 37)

///
/// Rtsym location as returned by the NSP
///
typedef struct NspRtsymResponse {
	/// Target memory type
	u64 Type;

	/// CPP target number
	u64 Target;

	/// Domain ID (island)
	u64 Domain;

	/// CPP address offset
	u64 Addr;

	/// Size of the symbol
	u64 Size;

} NspRtsymLocation;

///
/// Offsets relative to the CSR base
///
#define NSP_STATUS            0x00
#define   NSP_STATUS_MAGIC_of(x)      (((x) >> 48) & 0xffff)
#define   NSP_STATUS_MAGIC(x)         (((x) & 0xffffULL) << 48)
#define   NSP_STATUS_MAJOR_of(x)      (((x) >> 44) & 0xf)
#define   NSP_STATUS_MAJOR(x)         (((x) & 0xfULL) << 44)
#define   NSP_STATUS_MINOR_of(x)      (((x) >> 32) & 0xfff)
#define   NSP_STATUS_MINOR(x)         (((x) & 0xfffULL) << 32)
#define   NSP_STATUS_CODE_of(x)       (((x) >> 16) & 0xffff)
#define   NSP_STATUS_CODE(x)          (((x) & 0xffffULL) << 16)
#define   NSP_STATUS_RESULT_of(x)     (((x) >>  8) & 0xff)
#define   NSP_STATUS_RESULT(x)        (((x) & 0xffULL) << 8)
#define   NSP_STATUS_BUSY             (1ULL << 0)

#define NSP_COMMAND           0x08
#define   NSP_COMMAND_OPTION(x)       ((u64)((x) & 0xffffffff) <<32)
#define   NSP_COMMAND_OPTION_of(x)    (((x) >> 32) & 0xffffffff)
#define   NSP_COMMAND_CODE(x)         ((u32)((x) & 0xffff) <<16)
#define   NSP_COMMAND_CODE_of(x)      (((x) >> 16) & 0xffff)
#define   NSP_COMMAND_START           (1ULL << 0)
#define   NSP_COMMAND_OPTION_RESP_of(x)      ((NSP_COMMAND_OPTION_of((x)) >> 8) & 0xff)
#define   NSP_COMMAND_OPTION_REASON_of(x)    ((NSP_COMMAND_OPTION_of((x)) >> 16) & 0xff)

///
/// CPP address to retrieve the data from
///
#define NSP_BUFFER            0x10
#define   NSP_BUFFER_CPP(x)           ((u64)(((x) >> 8) & 0xffffff) << 40)
#define   NSP_BUFFER_CPP_of(x)        ((((x) >> 40) & 0xffffff) << 8)
#define   NSP_BUFFER_ADDRESS(x)       (((x) & ((1ULL << 40) - 1)) << 0)
#define   NSP_BUFFER_ADDRESS_of(x)    (((x) >> 0) & ((1ULL << 40) - 1))

///
/// CPP address for the NSP default buffer
///
#define NSP_DFLT_BUFFER            0x18
#define   NSP_DFLT_BUFFER_CPP(x)           ((u64)(((x) >> 8) & 0xffffff) << 40)
#define   NSP_DFLT_BUFFER_CPP_of(x)        ((((x) >> 40) & 0xffffff) << 8)
#define   NSP_DFLT_BUFFER_CPP_TARGET_of(x) (((x) >> 56) & 0xff)
#define   NSP_DFLT_BUFFER_CPP_TOKEN_of(x)  (((x) >> 48) & 0xff)
#define   NSP_DFLT_BUFFER_CPP_ACTION_of(x) (((x) >> 40) & 0xff)
#define   NSP_DFLT_BUFFER_ADDRESS(x)       (((x) & ((1ULL << 40) - 1)) << 0)
#define   NSP_DFLT_BUFFER_ADDRESS_of(x)    (((x) >> 0) & ((1ULL << 40) - 1))

/* Command Options to load UNDI rom from flash to buffer:
 * UNDI_LOAD_OPT_AUTO   : NSP determines correct UNDI rom to load (based on
 *			  autoload.fw hwinfo)
 * UNDI_LOAD_OPT_LEGACY : Load legacy UNDI rom (undi.rom fis partition)
 * UNDI_LOAD_OPT_NG     : Load next gen UNDI rom (undi_ng.rom fis partition)
 */
#define UNDI_LOAD_OPT_AUTO	0
#define UNDI_LOAD_OPT_LEGACY	1
#define UNDI_LOAD_OPT_NG	2

///
/// Get the nth entry in an eth table buffer
///
#define NSP_ETH_ENTRY(x)        ((x) * 0x20)

///
/// Port geometry
///
#define NSP_ETH_PORT            0x00
#define   NSP_ETH_PORT_FEC_RS(x)        ((uint64_t)((x) & 0x1) << 61)
#define   NSP_ETH_PORT_FEC_RS_of(x)     (((x) >> 61) & 0x1)
#define   NSP_ETH_PORT_FEC_BASER(x)     ((uint64_t)((x) & 0x1) << 60)
#define   NSP_ETH_PORT_FEC_BASER_of(x)  (((x) >> 60) & 0x1)
#define   NSP_ETH_PORT_PHYLABEL(x)      ((uint64_t)((x) & 0x3f) << 54)
#define   NSP_ETH_PORT_PHYLABEL_of(x)   (((x) >> 54) & 0x3f)
#define   NSP_ETH_PORT_LABEL(x)         ((uint64_t)((x) & 0x3f) << 48)
#define   NSP_ETH_PORT_LABEL_of(x)      (((x) >> 48) & 0x3f)
#define   NSP_ETH_PORT_ECHANNELS(x)     ((uint64_t)((x) & 0xf) << 40)
#define   NSP_ETH_PORT_ECHANNELS_of(x)  (((x) >> 40) & 0xf)
#define   NSP_ETH_PORT_ECHANNEL(x)      ((uint64_t)((x) & 0x7f) << 32)
#define   NSP_ETH_PORT_ECHANNEL_of(x)   (((x) >> 32) & 0x7f)
#define   NSP_ETH_PORT_ICHANNELS(x)     (((x) & 0xf) << 24)
#define   NSP_ETH_PORT_ICHANNELS_of(x)  (((x) >> 24) & 0xf)
#define   NSP_ETH_PORT_ICHANNEL(x)      (((x) & 0x7f) << 16)
#define   NSP_ETH_PORT_ICHANNEL_of(x)   (((x) >> 16) & 0x7f)
#define   NSP_ETH_PORT_INDEX(x)         (((x) & 0xff) << 8)
#define   NSP_ETH_PORT_INDEX_of(x)      (((x) >>  8) & 0xff)
#define   NSP_ETH_PORT_LANES(x)         (((x) & 0xf) >> 0)
#define   NSP_ETH_PORT_LANES_of(x)      (((x) >>  0) & 0xf)
#define NSP_ETH_STATE           0x08
#define   NSP_ETH_STATE_FEC(x)          (((x) & 3) << 26)
#define   NSP_ETH_STATE_FEC_of(x)       (((x) >> 26) & 0x3)
#define      NSP_ETH_FEC_AUTO 0
#define      NSP_ETH_FEC_FC   1
#define      NSP_ETH_FEC_RS   2
#define      NSP_ETH_FEC_NONE 3
#define   NSP_ETH_STATE_ANEG(x)         (((x) & 7) << 23)
#define   NSP_ETH_STATE_ANEG_of(x)      (((x) >> 23) & 0x7)
#define      NSP_ETH_ANEG_AUTO   0
#define      NSP_ETH_ANEG_SEARCH 1
#define      NSP_ETH_ANEG_CONS   2
#define      NSP_ETH_ANEG_IEEE   3
#define      NSP_ETH_ANEG_FORCED 4
#define   NSP_ETH_STATE_MCHG(x)         (((x) & 1) << 22)
#define   NSP_ETH_STATE_MCHG_of(x)      (((x) >> 22) & 0x1)
#define   NSP_ETH_STATE_MEDIA(x)        (((x) & 3) << 20)
#define   NSP_ETH_STATE_MEDIA_of(x)     (((x) >> 20) & 0x3)
#define      NSP_ETH_MEDIA_COPPER_PASSIVE 0
#define      NSP_ETH_MEDIA_COPPER_ACTIVE  1
#define      NSP_ETH_MEDIA_OPTICAL        2
#define   NSP_ETH_STATE_INTERFACE(x)    (((x) & 0xff) << 12)
#define   NSP_ETH_STATE_INTERFACE_of(x) (((x) >> 12) & 0xff)
#define      NSP_ETH_INTERFACE_UNKNOWN  0
#define      NSP_ETH_INTERFACE_SFP      1
#define      NSP_ETH_INTERFACE_SFP2     2
#define      NSP_ETH_INTERFACE_SFP5     5
#define      NSP_ETH_INTERFACE_SFP_PLUS 10
#define      NSP_ETH_INTERFACE_SFP28    28
#define      NSP_ETH_INTERFACE_QSFP     40
#define      NSP_ETH_INTERFACE_RJ45     45
#define      NSP_ETH_INTERFACE_CXP      100
#define      NSP_ETH_INTERFACE_QSFP28   112
#define   NSP_ETH_STATE_RATE(x)         (((x) & 0xf) << 8)
#define   NSP_ETH_STATE_RATE_of(x)      (((x) >> 8) & 0xf)
#define      NSP_ETH_RATE_10M           1
#define      NSP_ETH_RATE_100M          2
#define      NSP_ETH_RATE_1G            3
#define      NSP_ETH_RATE_10G           4
#define      NSP_ETH_RATE_25G           5
#define      NSP_ETH_RATE_2P5G          6
#define      NSP_ETH_RATE_5G            7
#define   NSP_ETH_STATE_BOOTABLE        (1 << 6)
#define   NSP_ETH_STATE_ACTIVE          (1 << 5)
#define   NSP_ETH_STATE_LINK            (1 << 4)
#define   NSP_ETH_STATE_ENABLED_RX      (1 << 3)
#define   NSP_ETH_STATE_ENABLED_TX      (1 << 2)
#define   NSP_ETH_STATE_ENABLED         (1 << 1)
#define   NSP_ETH_STATE_CONFIGURED      (1 << 0)

///
/// MAC address, 0-padded in the upper 16 bits
///
#define NSP_ETH_MAC             0x10

#define NSP_ETH_CONTROL         0x18
#define   NSP_ETH_CONTROL_ENABLE_RX     (1 << 3)
#define   NSP_ETH_CONTROL_ENABLE_TX     (1 << 2)
#define   NSP_ETH_CONTROL_ENABLE        (1 << 1)
#define   NSP_ETH_CONTROL_CONFIGURE     (1 << 0)

struct nsp_eth_tbl {
	u64 port;
	u64 state;
	u64 mac;
	u64 ctrl;
};

#define NSP_ETH_TABLE_SIZE_BYTES (MAX_NSP_ETH_TABLE_ENTRIES * sizeof(struct nsp_eth_tbl))

#define NSP_MAGIC             0xab10
#define NSP_MAJOR             0

#define NSP_CODE_MAJOR_of(code)	(((code) >> 12) & 0xf)
#define NSP_CODE_MINOR_of(code)	(((code) >>  0) & 0xfff)

#define NSP_CODE_NOOP             0       /* No operation */
#define NSP_CODE_SOFT_RESET       1       /* Soft reset the NFP */
#define NSP_CODE_FW_DEFAULT       2       /* Load default (UNDI) FW to buffer */
#define NSP_CODE_PHY_INIT         3       /* Initialize the PHY */
#define NSP_CODE_MAC_INIT         4       /* Initialize the MAC */
#define NSP_CODE_PHY_RXADAPT      5       /* Re-run PHY RX Adaptation */
#define NSP_CODE_FW_LOAD          6       /* Load firmware from buffer, len in option */
#define NSP_CODE_ETH_RESCAN       7       /* Rescan ETHs, update ETH_TABLE */
#define NSP_CODE_ETH_CONTROL      8       /* Perform ETH control action */
#define NSP_CODE_MDIO_PROXY       9       /* NSP MDIO proxy */
#define NSP_CODE_RPC              10      /* NSP RPC */
#define NSP_CODE_ARM_BIN_LOAD     11      /* Load bianry to arm */
#define NSP_CODE_SENSORS          12      /* Retrieve sensor state */
#define NSP_CODE_BSP_VERSION      13      /* Retrieve BSP version */
#define NSP_CODE_RTSYM_LOOKUP     14      /* Look up real time symbol*/
#define NSP_CODE_MAC_CONFIG       15      /* MAC configuration with supplied parameter overrides */
#define NSP_CODE_FW_STORED        16      /* Load app firmware */
#define NSP_CODE_HWINFO_LOOKUP    17      /* Look up HWinfo key including the overrides. */
#define NSP_CODE_HWINFO_SET       18      /* Set hwinfo key overrides*/
#define NSP_CODE_FW_LOADED        19      /* Perform Firmware Load check*/
#define NSP_CODE_LOAD_UNDI        20      /* Load UNDI driver from flash to default buffer */
#define NSP_CODE_FW_VERSIONS      21      /* Perform Firmware Load check*/
#define NSP_CODE_SFF_I2C          22      /* Read SFP EEPROM*/

#define NSP_CODE_TERMINATE        0xffff  /* Stop the NSP */
#endif /* _NFP_H */
