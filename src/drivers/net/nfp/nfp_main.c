/**************************************************************************
 *
 * Device driver for Netronome flow processor devices (6xxx/5xxx/4xxx)
 *
 * Written by Sridhar Pinnapa <sridhar.pinnapa@corigine.com>
 ***************************************************************************/

//FILE_LICENCE ( );

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <byteswap.h>

#include <ipxe/errfile.h>
#include <ipxe/netdevice.h>
#include <ipxe/ethernet.h>
#include <ipxe/if_ether.h>
#include <ipxe/iobuf.h>
#include <ipxe/malloc.h>
#include <ipxe/dma.h>
#include <ipxe/pci.h>
#include <ipxe/io.h>

#include "nfp.h"

#define TRUE	1
#define FALSE	0

static u64 NspDefaultBuffer = 0;
static u64 NspDefaultBufferOffset = 0;
static u64 NspCsrOffset = 0;

/*
 * The following defines are needed for BAR mapping calculation. These were
 * obtained from the relevant portions of nfp_cpp.h and nfp6000_pcie.c
 */

#define NFP_PCIE_CFG_BAR_PCIETOCPPEXPANSIONBAR(bar, slot) \
	(0x400 + ((bar) * 8 + (slot)) * 4)

#define NFP_PCIE_CPP_BAR_PCIETOCPPEXPANSIONBAR(bar, slot) \
	(((bar) * 8 + (slot)) * 4)

#define NFP_PCIE_P2C_BULK_SIZE(__bitsize)               (1 << (__bitsize))
#define NFP_PCIE_BULK_MAP_BITSIZE                       (40 - 21)

#define NFP_CPP_NUM_TARGETS                             16

#define   NFP_PCIE_BAR_PCIE2CPP_LengthSelect(_x)        (((_x) & 0x3) << 27)
#define   NFP_PCIE_BAR_PCIE2CPP_LengthSelect_of(_x)     (((_x) >> 27) & 0x3)
#define     NFP_PCIE_BAR_PCIE2CPP_LengthSelect_32BIT    0
#define     NFP_PCIE_BAR_PCIE2CPP_LengthSelect_64BIT    1
#define     NFP_PCIE_BAR_PCIE2CPP_LengthSelect_0BYTE    3

#define   NFP_PCIE_BAR_PCIE2CPP_MapType(_x)             (((_x) & 0x7) << 29)
#define   NFP_PCIE_BAR_PCIE2CPP_MapType_of(_x)          (((_x) >> 29) & 0x7)
#define     NFP_PCIE_BAR_PCIE2CPP_MapType_BULK          1

#define   NFP_PCIE_BAR_PCIE2CPP_Target_BaseAddress(_x)  (((_x) & 0xf) << 23)
#define   NFP_PCIE_BAR_PCIE2CPP_Target_BaseAddress_of(_x) (((_x) >> 23) & 0xf)

#define   NFP_PCIE_BAR_PCIE2CPP_Token_BaseAddress(_x)   (((_x) & 0x3) << 21)
#define   NFP_PCIE_BAR_PCIE2CPP_Token_BaseAddress_of(_x) (((_x) >> 21) & 0x3)

static inline uint8_t log2(uint64_t n)
{
	uint8_t val;
	for (val = 0; n > 1; val++, n >>= 1);

	return val;
}

static char tohex(char digit)
{
	char value;

	// We assume that the MAC address delivered from the NSP would be in the
	// correct form, i.e. ab:cd:ef:01:23:45
	if (digit >= 'a') {
		value = digit - 'a' + 10;
	} else if (digit >= 'A') {
		value = digit - 'A' + 10;
	} else {
		value = digit - '0';
	}

	return value;
}

/**
  Function computes the BAR mapping and offset for a given CPP transaction.

  @param Target        CPP target
  @param Action        CPP action
  @param Token         CPP token
  @param Addr          CPP address field
  @param Size          Size of the mapping requirement (only used for BAR size check)
  @param Width         Width of access, e.g. 0, 4 or 8
  @param BarBitsize    Bit size of the BAR slice targetted for configuration
  @param BarMapping    Output of the BAR mapping needed to access this location
  @param Offset         Output of the offset needed access the CPP location

  @retval 0   The operation completed successfully.
  @retval Others        An unexpected error occurred.

 **/
	int
nfp_compute_bar(u32 Target, u32 Action, u32 Token, u64 Addr,
		u32 Size, u8 Width, u8 BarBitsize,
		u32 *BarMapping, u64 *Offset)
{
	u64 LowerAddr;
	u64 BarCfg;
	u64 Mask;

	//
	// We only support bulk mappings at the moment
	//
	if (Action != NFP_CPP_ACTION_RW) {
		DBG2("%s: Action 0x%x not supported\n",
				__func__, Action);
		return -EINVAL;
	}

	if (BarBitsize < NFP_PCIE_BULK_MAP_BITSIZE) {
		DBG2("%s: BAR too small, bitsize %d < %d\n",
				__func__, BarBitsize, NFP_PCIE_BULK_MAP_BITSIZE);
		return -EINVAL;
	}

	if (Target >= NFP_CPP_NUM_TARGETS) {
		DBG2("%s: Target 0x%x not supported\n",
				__func__, Target);
		return -EINVAL;
	}

	switch (Width) {
		case 8:
			BarCfg = NFP_PCIE_BAR_PCIE2CPP_LengthSelect(
					NFP_PCIE_BAR_PCIE2CPP_LengthSelect_64BIT);
			break;
		case 4:
			BarCfg = NFP_PCIE_BAR_PCIE2CPP_LengthSelect(
					NFP_PCIE_BAR_PCIE2CPP_LengthSelect_32BIT);
			break;
		case 0:
			BarCfg = NFP_PCIE_BAR_PCIE2CPP_LengthSelect(
					NFP_PCIE_BAR_PCIE2CPP_LengthSelect_0BYTE);
			break;
		default:
			return -EINVAL;
	}

	Mask = ~(NFP_PCIE_P2C_BULK_SIZE(BarBitsize) - 1);
	BarCfg |= NFP_PCIE_BAR_PCIE2CPP_MapType(NFP_PCIE_BAR_PCIE2CPP_MapType_BULK);
	BarCfg |= NFP_PCIE_BAR_PCIE2CPP_Target_BaseAddress(Target);
	BarCfg |= NFP_PCIE_BAR_PCIE2CPP_Token_BaseAddress(Token);

	if ((Addr & Mask) != ((Addr + Size - 1) & Mask)) {
		DBG2("Won't use for bulk mapping <%#llx,%#llx>, target=0x%x, token=0x%x. BAR too small (%#llx) - (%#llx != %#llx).\n",
				Addr, Addr + Size, Target, Token, Mask, Addr & Mask,
				(Addr + Size - 1) & Mask);
		return -EINVAL;
	}
	LowerAddr = Addr & Mask;
	BarCfg |= LowerAddr >> NFP_PCIE_BULK_MAP_BITSIZE;
	*BarMapping = BarCfg;
	*Offset = Addr - LowerAddr;

	DBG2("Created bulk mapping %d:%d:%d:0x%llx-0x%llx: 0x%llx with offset 0x%llx\n",
			Target, Action, Token, LowerAddr, LowerAddr + ~Mask, BarCfg,
			*Offset);

	return 0;
}

	static int
__nfp_dev_data_bar_readb(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u8);
	char *read = (char *)value;
	for(i = 0; i < len; i++) {
		*read = readb((bar->iomem + offset + (i * width)));
		read += i;
	}
	return 0;
}

	static int
__nfp_dev_data_bar_readw(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u16);
	u16 *read = (u16 *)value;
	for(i = 0; i < len; i++) {
		*read = readw((bar->iomem + offset + (i * width)));
		read += i;
	}
	return 0;
}

	static int
__nfp_dev_data_bar_readl(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u32);
	u32 *read = (u32 *)value;
	for(i = 0; i < len; i++) {
		*read = readl((bar->iomem + offset + (i * width)));
		read += i;
	}
	return 0;
}

	static int
__nfp_dev_data_bar_readq(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u64);
	u64 *read = (u64 *)value;
	for(i = 0; i < len; i++) {
		*read = readq((bar->iomem + offset + (i * width)));
		read += i;
	}
	return 0;
}

	static int
nfp_dev_bar_write(struct nfp_adapter *adapter, struct nfp_bar *bar, u64 offset, u32 newcfg)
{
	int base, slot;
	int xbar;

	base = bar->index >> 3;
	slot = bar->index & 7;

	xbar = NFP_PCIE_CPP_BAR_PCIETOCPPEXPANSIONBAR(base, slot);
	pci_write_config_dword(adapter->pdev, (xbar + offset), newcfg);

	bar->barcfg = newcfg;

	DBG2("BAR%d: updated to 0x%08x\n", bar->index, newcfg);

	return 0;
}

	static int
__nfp_dev_data_bar_writeb(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u8);
	for(i = 0; i < len; i++) {
		writeb(*((u8 *)value + i), (bar->iomem + offset + (i * width)));
	}
	return 0;
}

	static int
__nfp_dev_data_bar_writew(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u16);
	for(i = 0; i < len; i++) {
		writew(*((u16 *)value + i), (bar->iomem + offset + (i * width)));
	}
	return 0;
}

	static int
__nfp_dev_data_bar_writel(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u32);
	for(i = 0; i < len; i++) {
		writel(*((u32 *)value + i), (bar->iomem + offset + (i * width)));
	}
	return 0;
}

	static int
__nfp_dev_data_bar_writeq(struct nfp_adapter *adapter __unused, struct nfp_bar *bar, u64 offset,
		u16 len, void *value)
{
	int i;
	int width = sizeof(u64);
	for(i = 0; i < len; i++) {
		writeq(*((u64 *)value + i), (bar->iomem + offset + (i * width)));
	}
	return 0;
}

	static int
nfp_dev_data_bar_write(struct nfp_adapter *adapter, struct nfp_bar *bar, u64 offset, 
		u8 width, u16 len, void *value)
{
	int ret;
	switch(width) {
		case 1: /* byte (1B/8-bit) */
			ret = __nfp_dev_data_bar_writeb(adapter, bar, offset, len, value);
			break;
		case 2: /* word (2B/16-bit) */
			ret = __nfp_dev_data_bar_writew(adapter, bar, offset, len, value);
			break;
		case 4: /* dword (4B/32-bit) */
			ret = __nfp_dev_data_bar_writel(adapter, bar, offset, len, value);
			break;
		case 8: /* qword (8B/64-bit) */
			ret = __nfp_dev_data_bar_writeq(adapter, bar, offset, len, value);
			break;
		default:
			return -ENOTSUP;
	}
	return ret;
}

/**
  Function configures the BAR mapping for the local PCIe island.

  @param adapter PCI IO functions including read and write.
  @param BarNumber      Bar slot number, 0, 1 or 2.
  @param BarSection     Expansion BAR section, 0 - 7.
  @param BarMapping     Mapping value to program into the register.

  @retval 0   The operation completed successfully.
  @retval Others        An unexpected error occurred.

 **/
	int
nfp_cfg_expbar(struct nfp_adapter *adapter, struct nfp_bar *bar, u32 BarMapping)
{

	return nfp_dev_bar_write(adapter, bar, 0, BarMapping);
}

static int nfp_dev_bar2_writeb(struct nfp_adapter *adapter, u64 offset, u32 value)
{
	u8 width = sizeof(u8);
	u16 len = 1;
	return nfp_dev_data_bar_write(adapter, &adapter->bar[16], offset, width, len, &value);
}

static int nfp_dev_bar2_writew(struct nfp_adapter *adapter, u64 offset, u32 value)
{
	u8 width = sizeof(u16);
	u16 len = 1;
	return nfp_dev_data_bar_write(adapter, &adapter->bar[16], offset, width, len, &value);
}

static int nfp_dev_bar2_writel(struct nfp_adapter *adapter, u64 offset, u32 value)
{
	u8 width = sizeof(u32);
	u16 len = 1;
	return nfp_dev_data_bar_write(adapter, &adapter->bar[16], offset, width, len, &value);
}

static int nfp_dev_bar2_writeq(struct nfp_adapter *adapter, u64 offset, u64 value)
{
	u8 width = sizeof(u64);
	u16 len = 1;
	return nfp_dev_data_bar_write(adapter, &adapter->bar[16], offset, width, len, &value);
}

int nfp_dev_bar_read(struct nfp_adapter *adapter, struct nfp_bar *bar, u64 offset, u32 *value)
{
	int base, slot;
	int xbar;

	base = bar->index >> 3;
	slot = bar->index & 7;

	xbar = NFP_PCIE_CPP_BAR_PCIETOCPPEXPANSIONBAR(base, slot);
	pci_read_config_dword(adapter->pdev, (xbar + offset), value);

	return 0;
}

int nfp_dev_data_bar_read(struct nfp_adapter *adapter, struct nfp_bar *bar, u64 offset,
		u8 width, u16 len, void *value)
{
	int ret;
	switch(width) {
		case 1: /* byte (1B/8-bit) */
			ret = __nfp_dev_data_bar_readb(adapter, bar, offset, len, value);
			break;
		case 2: /* word (2B/16-bit) */
			ret = __nfp_dev_data_bar_readw(adapter, bar, offset, len, value);
			break;
		case 4: /* dword (4B/32-bit) */
			ret = __nfp_dev_data_bar_readl(adapter, bar, offset, len, value);
			break;
		case 8: /* qword (8B/64-bit) */
			ret = __nfp_dev_data_bar_readq(adapter, bar, offset, len, value);
			break;
		default:
			return -ENOTSUP;
	}
	return ret;
}

static int nfp_dev_bar2_readl(struct nfp_adapter *adapter, u64 addr, u32 *value)
{
	u8 width = sizeof(u32);
	u16 len = 1;
	return nfp_dev_data_bar_read(adapter, &adapter->bar[16], addr, width, len, value);
}

int nfp_nsp_cmd(struct nfp_adapter *adapter, u16 Code, u32 Option)
{
	u64 NspCommand = 0;
	u64 NspStatus = 0;
	int Status;
	u32 Timeout;

	for (Timeout = 20; Timeout > 0; Timeout--) {
		Status = nfp_dev_data_bar_read(adapter, &adapter->bar[8],
				NspCsrOffset + NSP_STATUS, sizeof(u64), 1, &NspStatus);
		if (Status < 0) {
			return Status;
		}

		DBG2("NSP_STATUS: %llx\n", NspStatus);
		if ((NSP_STATUS_MAGIC_of(NspStatus) == NSP_MAGIC) &&
				!(NspStatus & NSP_STATUS_BUSY)) {
			break;
		}

		DBG2("t%3d:c%2d:o%3d|", Timeout, Code, Option);
		usleep(1000);
		DBG2("\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
	}

	if (Timeout == 0) {
		if (NspStatus & NSP_STATUS_BUSY) {
			DBG2("NSP: Busy\n");
			return -ETIMEDOUT;
		} else {
			DBG2("NSP: Not detected\n\r");
			return -1;
		}
	}

	//
	// Setup the NSP default buffer for usage
	//
	Status = nfp_dev_data_bar_write(adapter, &adapter->bar[8],
			NspCsrOffset + NSP_BUFFER, sizeof(u64), 1,
			&NspDefaultBuffer);
	if (Status < 0) {
		return Status;
	}

	NspCommand = (NSP_COMMAND_CODE(Code) | NSP_COMMAND_START) | ((u64) Option << 32);
	DBG2("command=0x%llx [code=0x%x, option=0x%x]\n", NspCommand, Code, Option);

	Status = nfp_dev_data_bar_write(adapter, &adapter->bar[8],
			NspCsrOffset + NSP_COMMAND, sizeof(u64), 1,
			&NspCommand);
	if (Status < 0) {
		return Status;
	}

	//
	// Wait for 'timeout' seconds until the NSP command execution started
	//
	for (Timeout = 25; Timeout > 0; Timeout--) {
		Status = nfp_dev_data_bar_read(adapter, &adapter->bar[8],
				NspCsrOffset + NSP_COMMAND, sizeof(u64), 1,
				&NspCommand);
		if (Status < 0) {
			return Status;
		}

		DBG2("NSP_COMMAND_START(%d): %llx\n", Code, NspCommand);
		if (!(NspCommand & NSP_COMMAND_START)) {
			break;
		}

		DBG2("t%3d:c%2d:o%3d|", Timeout, Code, Option);
		usleep(1000);
		DBG2("\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
	}

	if (Timeout == 0) {
		DBG2("NSP: Timeout waiting for code 0x%04x to start\n", Code);
		return -ETIMEDOUT;
	}

	//
	// Wait for NSP_STATUS_BUSY to go to 0
	// Wait for 'timeout' seconds until the NSP command is finished
	//
	for (Timeout = 30; Timeout > 0; Timeout--) {
		Status = nfp_dev_data_bar_read(adapter, &adapter->bar[8],
				NspCsrOffset + NSP_STATUS, sizeof(u64), 1,
				&NspStatus);
		if (Status < 0) {
			return Status;
		}

		DBG2("NSP_STATUS_BUSY(%d): %llx\n", Code, NspStatus);
		if (!(NspStatus & NSP_STATUS_BUSY)) {
			break;
		}

		DBG2("t%3d:c%2d:o%3d|", Timeout, Code, Option);
		usleep(1000);
		DBG2("\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
	}

	if (Timeout == 0) {
		DBG2("NSP: Timeout waiting for code 0x%04x to complete\n", Code);
		return -ETIMEDOUT;
	}

	//
	// Read the additional responses from the NSP contained in the NspCommand
	// field.
	//
	Status = nfp_dev_data_bar_read(adapter, &adapter->bar[8],
			NspCsrOffset + NSP_COMMAND, sizeof(u64), 1,
			&NspCommand);
	if (Status < 0) {
		return Status;
	}

	if (NSP_STATUS_RESULT_of(NspStatus)) {
		DBG2("NSP_RESULT(%d): %llx\n", Code, NspStatus);
		if (Code == NSP_CODE_FW_STORED) {
			DBG2("NSP_COMMAND_OPTION_RESP(%d): %x\n", Code,
					(u32)NSP_COMMAND_OPTION_RESP_of(NspCommand));
			DBG2("NSP_COMMAND_OPTION_REASON(%d): %x\n", Code,
					(u32)NSP_COMMAND_OPTION_REASON_of(NspCommand));

			if (NSP_COMMAND_OPTION_RESP_of(NspCommand) == 1) {
				return 0;
			}
		}
		return -ETIMEDOUT;
	}
	if (Code == NSP_CODE_FW_LOADED) {
		if (NSP_COMMAND_OPTION_of(NspCommand) == 1) {
			return 0;
		}
		return -ETIMEDOUT;
	}
	return 0;
}

void nfp_get_nsp_def_buf(u64 *NspDefBuf, u64 *NspDefBufOffset)
{
	*NspDefBuf = NspDefaultBuffer;
	*NspDefBufOffset = NspDefaultBufferOffset;
}

int nfp_read_nsp_buf(struct nfp_adapter *adapter,
		void *HostBuffer, u32 Bytes)
{

	DBG2("%s: read @ %d: 0x%llx for %d words and %d Bytes\n",
			__func__, NETRONOME_DEV_BAR_1, NspDefaultBufferOffset,
			Bytes / 4, Bytes);

	return nfp_dev_data_bar_read(adapter, &adapter->bar[8],
			NspDefaultBufferOffset, sizeof(u32), Bytes / 4, HostBuffer);
}

int nfp_write_nsp_buf(struct nfp_adapter *adapter,
		void *HostBuffer, u32 Bytes)
{

	if (Bytes % sizeof(u32)) {
		DBG2("%s: Only supports whole 32bit writes!\n", __func__);
		return -1;
	}

	DBG2("%s: write @ %d: 0x%llx for %d words and %d Bytes\n",
			__func__, NETRONOME_DEV_BAR_1, NspDefaultBufferOffset,
			Bytes / 4, Bytes);

	return nfp_dev_data_bar_write(adapter, &adapter->bar[8],
			NspDefaultBufferOffset, sizeof(u32), Bytes / 4, HostBuffer);
}

int nfp_nsp_rtsym_to_cpp(struct nfp_adapter *adapter, char *Symbol,
		u32 SymbolNameLength, int *CppTarget,
		u64 *CppAddr, u32 *Size)
{
	char Buffer[RTSYM_LOOKUP_BUFFER_SIZE];
	struct nsp_rtsym_response Rtsym;
	int Status;

	//
	// Pad the symbol name by clearing out our own internal buffer and only
	// copying the symbol data. We copy the entire internal buffer to the NSP
	// buffer, but we only specify the actual symbol size in the command.
	//
	memset(Buffer, 0, RTSYM_LOOKUP_BUFFER_SIZE);
	memcpy(Buffer, Symbol, SymbolNameLength);

	Status = nfp_write_nsp_buf(adapter, Symbol, RTSYM_LOOKUP_BUFFER_SIZE);
	if (Status < 0) {
		DBG2("NSP write to buffer error. Status = %x!\n", Status);
		goto errUnlockNsp;
	}

	Status = nfp_nsp_cmd(adapter, NSP_CODE_RTSYM_LOOKUP, SymbolNameLength);
	if (Status < 0) {
		DBG2("NSP RTSYM lookup failed. Status = %x!\n", Status);
		goto errUnlockNsp;
	}

	Status = nfp_read_nsp_buf(adapter, (void *)&Rtsym, sizeof(Rtsym));
	if (Status < 0) {
		DBG2("NSP read from buffer error. Status = %x!\n", Status);
		goto errUnlockNsp;
	}


	// Constrain some of the items we don't support. Assume 40bit addressing
	// is used.
	if (Rtsym.type != NFP_RTSYM_TYPE_OBJECT) {
		DBG2("RTsym '%s', unsupported type %x\n", Buffer, Rtsym.type);
		return -1;
	}
	if (!(Rtsym.target == NFP_CPP_TARGET_MU
				|| Rtsym.target == NFP_RTSYM_TARGET_EMU_CACHE)) {
		DBG2("RTsym '%s', unsupported target %d\n", Buffer, Rtsym.target);
		return -1;
	}
	// We don't support NBI or Crypto access
	if ((Rtsym.domain >= 8 && Rtsym.domain < NFP_ISLAND_EMU_MIN)) {
		DBG2("RTsym '%s', unsupported domain %d\n", Buffer, Rtsym.domain);
		return -1;
	}

	*Size = Rtsym.size;
	*CppTarget = Rtsym.target;
	if (Rtsym.domain >= NFP_ISLAND_EMU_MIN && Rtsym.domain <= NFP_ISLAND_EMU_MAX
			&& Rtsym.target == NFP_CPP_TARGET_MU
			&& (Rtsym.addr >> NFP_MU_LOCALITY_LSB) != NFP_MU_ADDR_ACCESS_TYPE_DIRECT) {
		*CppAddr = Rtsym.addr;
		*CppAddr |= (u64)(Rtsym.domain & 0x3) << NFP_MU_ISLAND_DIRECT_LSB;
		*CppAddr |= NFP_MU_ADDR_ISLAND_INDEX1;
	} else {
		*CppAddr = Rtsym.addr;
		*CppAddr |= (u64)(Rtsym.domain & 0x3f) << NFP_MU_ISLAND_LSB;

		// Use direct access mode for non-EMU island memory access
		*CppAddr &= ~(NFP_MU_ADDR_ACCESS_TYPE_MASK << NFP_MU_LOCALITY_LSB);
		*CppAddr |= NFP_MU_ADDR_ACCESS_TYPE_DIRECT << NFP_MU_LOCALITY_LSB;

		if (*CppTarget == NFP_RTSYM_TARGET_EMU_CACHE) {
			*CppTarget = NFP_CPP_TARGET_MU;
		}
	}

	DBG2("Found symbol: %s[%d]\n", Symbol, SymbolNameLength);
	DBG2("  Type: 0x%x\n", Rtsym.type);
	DBG2("  Domain: 0x%x\n", Rtsym.domain);
	DBG2("  Target: 0x%x\n", Rtsym.target);
	DBG2("  Addr: 0x%llx\n", Rtsym.addr);
	DBG2("  Size: 0x%llx\n", Rtsym.size);
	DBG2("  CPP Target: 0x%x\n", *CppTarget);
	DBG2("  CPP Addr: 0x%llx\n", *CppAddr);

	return 0;

errUnlockNsp:
	return Status;
}

int nfp_nsp_eth_rescan(struct nfp_adapter *adapter, void *HostBuffer,
		u32 BufferSize)
{
	int Status;

	memset((char *) HostBuffer, 0, BufferSize);

	Status = nfp_nsp_cmd(adapter, NSP_CODE_ETH_RESCAN,
			NSP_ETH_TABLE_SIZE_BYTES);
	if (Status < 0) {
		DBG2("NSP ETH Scan Timeout. Status = %x!\n", Status);
		goto outUnlockNsp;
	}

	Status = nfp_read_nsp_buf(adapter, HostBuffer,
			NSP_ETH_TABLE_SIZE_BYTES);
	if (Status < 0) {
		DBG2("NSP read from buffer error. Status = %x!\n", Status);
		goto outUnlockNsp;
	}

	Status = 0;

outUnlockNsp:
	return Status;
}

int nfp_nsp_eth_cfg(struct nfp_adapter *adapter, void *HostBuffer,
		u32 Bytes)
{
	int Status;

	Status = nfp_write_nsp_buf(adapter, HostBuffer, Bytes);
	if (Status < 0) {
		DBG2("NSP write to buffer error. Status = %x!\n", Status);
		goto outUnlockNsp;
	}

	Status = nfp_nsp_cmd(adapter, NSP_CODE_ETH_CONTROL, Bytes);
	if (Status < 0) {
		DBG2("NSP ETH Control Failure. Status = %x!\n", Status);
		goto outUnlockNsp;
	}

	Status = 0;

outUnlockNsp:
	return Status;
}

/**
  Increment the value of the write pointer of a queue by 1.

  @param adapter PCI I/O protocol functions for reading/writing to the device.
  @param Ring     QCP ring structure reference.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
	static int
nfp_qcp_incr_wrptr(struct nfp_adapter *adapter, struct qcp_ring *Ring)
{
	u64 Address = QCP(Ring, NFP_QCP_QUEUE_ADD_WPTR);
	return nfp_dev_bar2_writel(adapter, Address, 1);
}

/**
  Read the value of the read pointer of a queue.

  @param PciIoFunctions PCI I/O protocol functions for reading/writing to the device.
  @param Ring     QCP ring structure reference.
  @param Value        Return value of the queue pointer.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
	static int
nfp_get_qcp_rdptr(struct nfp_adapter *adapter, struct qcp_ring *Ring,
		u32 *Value)
{
	u64 Address;
	u32 ReadPtr;
	int Status;

	Address = QCP(Ring, NFP_QCP_QUEUE_STS_LO);

	Status = nfp_dev_bar2_readl(adapter, Address, &ReadPtr);
	if (Status < 0) {
		return Status;
	}

	*Value = ReadPtr & NFP_QCP_QUEUE_STS_LO_READPTR_mask;

	return 0;
}

//
// The 'nfp_get_qcp_wrptr' function is not currently used, however, it
// is valuable during debugging. So let's keep it around for that purpose.
//
#if DEBUG
/**
  Read the value of the write pointer of a queue.

  @param PciIoFunctions PCI I/O protocol functions for reading/writing to the device.
  @param Ring     QCP ring structure reference.
  @param Value        Return value of the queue pointer.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
	static int
nfp_get_qcp_wrptr(struct nfp_adapter *adapter, struct qcp_ring *Ring,
		u32 *Value)
{
	u64 Address;
	u32 ReadPtr;
	int Status;

	Address = QCP(Ring, NFP_QCP_QUEUE_STS_HI);

	Status = nfp_dev_bar2_readl(adapter, Address, &ReadPtr);
	if (Status < 0) {
		return Status;
	}

	*Value = ReadPtr & NFP_QCP_QUEUE_STS_HI_WRITEPTR_mask;

	return 0;
}
#endif

/**
  Function reads the vNICs' BAR.

  @param adapter      Structure containing interface specific information.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
int nfp_cfg_vnic_bar(struct nfp_adapter *adapter)
{
	u32 BarMapping;
	char Buffer[128];
	int CppTarget;
	u64 CppAddr;
	u32 TxRings;
	u32 RxRings;
	int Status;
	u64 Offset;
	u16 Bytes;
	u32 Size;
	u32 Iter;

	//XXX: We can actually support multiple vNICs, we just need to read the
	//     max from firmware.
	adapter->vniccount = 1;

	Bytes = snprintf(Buffer, 128, "_pf%d_net_bar0", adapter->pcie_island);
	Status = nfp_nsp_rtsym_to_cpp(adapter, Buffer, Bytes, &CppTarget,
			&CppAddr, &Size);
	if (Status < 0) {
		DBG2("Failed to find symbol: %s[%d]\n", Buffer, Bytes);
		return Status;
	}

	Status = nfp_compute_bar(CppTarget, NFP_CPP_ACTION_RW, 0, CppAddr,
			Size, 4, NETRONOME_DEV_BAR_2_BITSIZE,
			&BarMapping, &Offset);
	if (Status < 0) {
		DBG2("Failed to compute BAR: %x\n", Status);
		return Status;
	}

	// Set BAR to access NFD BAR config space
	Status = nfp_cfg_expbar(adapter, &adapter->bar[16], BarMapping);
	if (Status < 0) {
		return Status;
	}

	// Set BAR to access QCP
	Status = nfp_cfg_expbar(adapter, &adapter->bar[17], NETRONOME_DEV_BAR_2_1_MAP_UNDI);
	if (Status < 0) {
		return Status;
	}

	//
	// We only support a single TX and RX queue for UNDI, but make sure the
	// firmware does support that.
	//
	for (Iter = 0; Iter < adapter->vniccount; Iter++) {
		adapter->ctrlbar[Iter] = Offset + (Iter * NFP_PF_CSR_SLICE_SIZE);

		Status = nfp_dev_bar2_readl(adapter, VNIC_CTRL_ADDR(adapter, Iter, NFP_NET_CFG_MAX_TXRINGS),
				&TxRings);
		if (Status < 0) {
			return Status;
		}

		Status = nfp_dev_bar2_readl(adapter, VNIC_CTRL_ADDR(adapter, Iter, NFP_NET_CFG_MAX_RXRINGS),
				&RxRings);
		if (Status < 0) {
			return Status;
		}

		if ((TxRings == 0) || (RxRings ==  0)) {
			DBG2("Firmware doesn't support Tx(%d) or Rx(%d) Rings!\n",
					TxRings, RxRings);
			return -ENOTSUP;
		}
	}

	return 0;
}

/**
  Function reads and prints device firmware version information.

  @param adapter      Structure containing interface specific information.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
int nfp_read_ver_info(struct nfp_adapter *adapter)
{
	int Status;
	u32 VersionNumber;

	//
	// Read the firmware version number. We only need to read this for the
	// first vNIC since NFD will enforce the same version on all vNICs.
	//
	Status = nfp_dev_bar2_readl(adapter, VNIC_CTRL_ADDR(adapter, 0, NFP_NET_CFG_VERSION),
			&VersionNumber);
	if (Status < 0) {
		return Status;
	}

	//
	// Save the version information in the device structure.
	//
	adapter->version.verclass = NFD_VERSION_CLASS(VersionNumber);
	adapter->version.vermajor = NFD_VERSION_MAJOR(VersionNumber);
	adapter->version.verminor = NFD_VERSION_MINOR(VersionNumber);
	adapter->version.verreserved = NFD_VERSION_RESV(VersionNumber);

	//
	// Note that the NFD version would dictate the Queue stride parameter. But
	// since we only use a single queue, we don't really care.
	//

	return 0;
}

/**
  Calculate the memory required by the driver for RX buffers.

  @param adapter      Structure containing interface specific information.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
int nfp_calc_rxbuf_mem(struct nfp_adapter *adapter)
{
	adapter->RxMemRequired = NFP_NET_RX_DESCS_UNDI * RX_BUF_SIZE;

	return 0;
}

/**
  Request a firmware reconfiguration

  @param adapter      Structure containing interface specific information.
  @param VnicIdx      vNIC index to reconfigure.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static int nfp_reconfig(struct nfp_adapter *adapter, u8 VnicIdx)
{
	char ReconfigDone;
	u32 UpdateCfg;
	u64 Address;
	int Retry;
	int Status;

	//
	// Ping hardware and check for ack
	//
	Address = QCP(&adapter->cfg_queue, NFP_QCP_QUEUE_ADD_WPTR);
	Status = nfp_dev_bar2_writel(adapter, Address, 1);
	if (Status < 0) {
		return Status;
	}

	Retry = 0;
	ReconfigDone = FALSE;

	while (ReconfigDone == FALSE) {
		Status = nfp_dev_bar2_readl(adapter,
				VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_UPDATE),
				&UpdateCfg);
		if (Status < 0) {
			return Status;
		}

		if (UpdateCfg & NFP_NET_CFG_UPDATE_ERR) {
			DBG2("Reconfig error: 0x%08x\n", UpdateCfg);
			return -EIO;
		}
		if (Retry >= NFP_NET_POLL_TIMEOUT) {
			DBG2("Reconfig timeout: 0x%08x\n", UpdateCfg);
			return -EIO;
		}

		if (UpdateCfg == 0) {
			DBG2("Reconfigured vNIC #%d! (Retry=%d)\n", VnicIdx, Retry);
			ReconfigDone = TRUE;
		}
		else {
			Retry++;
		}
	}

	return 0;
}

/**
  Calculate the memory required by the driver for RX buffers.

  @param adapter      Structure containing interface specific information.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static int nfp_give_fl_entry(struct nfp_adapter *adapter,
		struct qcp_ring *RxRing)
{
	struct fl_desc *Desc;
	u32 WriteIndex;
	struct buffer *RxBuffer;
	struct io_buffer *iobuf;
	int Status;

	//
	// We shouldn't ever overlap read and write pointers since the firmware is
	// never expected to DMA more packets than buffers we provided
	//
	if (RxRing->readptr > RxRing->writeptr) {
		DBG2("RX Ring corruption! Read:%d Write:%d\n",
				RxRing->readptr, RxRing->writeptr);
		return -EIO;
	}

	WriteIndex = D_IDX(RxRing, RxRing->writeptr);

	Desc = (struct fl_desc *) &RxRing->ringdmahostbuffer[WriteIndex * sizeof(struct fl_desc)];
	RxBuffer = &adapter->rx_ring[0].freelist[WriteIndex];

	//
	// Only do the DMA mapping as soon as we hand the buffer to the NFP. This
	// simplifies things since we must unmap before reading the packet data.
	//
	/* Allocate I/O buffer */
	iobuf = alloc_rx_iob (RX_BUF_SIZE, adapter->dma);
	if ( ! iobuf ) {
		return -ENOMEM;
	}
	RxBuffer->iobuf = iobuf;
	RxBuffer->dmaaddr = cpu_to_le64 ( iob_dma ( iobuf ) );

	RxBuffer->ismapped = TRUE;

	Desc->DmaAddrLo = LOWER_32_BITS(RxBuffer->dmaaddr);
	Desc->DmaAddrHi = UPPER_8_BITS(RxBuffer->dmaaddr);
	Desc->Reserved = 0;
	Desc->MetaLenDd = 0;

	Status = nfp_qcp_incr_wrptr(adapter, RxRing);
	if (Status < 0) {
		DBG2("Failed to increment write pointer\n");
		return Status;
	}
	RxRing->writeptr++;

	return 0;
}

/**
  Calculate the memory required by the driver for RX buffers.

  @param adapter      Structure containing interface specific information.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static int nfp_refill_freelist(struct nfp_adapter *adapter,
		u8 VnicIdx)
{
	struct qcp_ring *RxRing;
	u16 Index;
	int Status;

	RxRing = &adapter->rx_ring[VnicIdx].ring;

	for (Index = 0; Index < RxRing->count; Index++) {
		Status = nfp_give_fl_entry(adapter, RxRing);
		if (Status < 0) {
			return Status;
		}
	}

	return 0;
}

/**
  Clear vNIC configuration and disable

  @param adapter      Structure containing interface specific information.
  @param VnicIdx      vNIC index to disable

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static void nfp_dev_clr_cfg_and_dis(struct nfp_adapter *adapter,
		u8 VnicIdx)
{

	DBG2("Device #%d: Disabling\n", VnicIdx);

	//
	// No harm in running this function even if the vNIC was already disabled.
	//
	adapter->device_info.adminstate = 0;

	//
	// Clear out configuration
	//
	nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_TXR_ADDR(0)),
			0);
	nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_RXR_ADDR(0)),
			0);
	nfp_dev_bar2_writeb(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_TXR_SZ(0)),
			0);
	nfp_dev_bar2_writeb(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_RXR_SZ(0)),
			0);
	nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_TXRS_ENABLE),
			0);
	nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_RXRS_ENABLE),
			0);
	nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_MACADDR),
			0);
	nfp_dev_bar2_writew(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx,
				NFP_NET_CFG_MACADDR + 6),
			0);

	//
	// Disable device
	//
	nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_CTRL),
			0);
	nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_UPDATE),
			NFP_NET_CFG_UPDATE_GEN | NFP_NET_CFG_UPDATE_RING);

	nfp_reconfig(adapter, VnicIdx);

	//
	// Reset the ring pointers
	//
	adapter->rx_ring[VnicIdx].ring.readptr = 0;
	adapter->rx_ring[VnicIdx].ring.writeptr = 0;

	adapter->tx_ring[VnicIdx].ring.readptr = 0;
	adapter->tx_ring[VnicIdx].ring.writeptr = 0;
}

/**
  Configure and enable a vNIC. This involves configuring the ring parameters,
  device parameters and finally requesting the firmware to reconfigure the
  device.

  @param adapter      Structure containing interface specific information.
  @param VnicIdx      vNIC index to configure and enable.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
	static int
nfp_dev_conf_and_en(struct nfp_adapter *adapter, u8 VnicIdx)
{
	u32 MacAddr32;
	u16 MacAddr16;
	struct qcp_ring *TxRing;
	struct qcp_ring *RxRing;
	int Status;

	if (adapter->device_info.adminstate) {
		return 0;
	}

	DBG2("Device #%d: Enabling\n", VnicIdx);

	TxRing = &adapter->tx_ring[VnicIdx].ring;
	RxRing = &adapter->rx_ring[VnicIdx].ring;

	Status = nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_TXR_ADDR(0)),
			TxRing->ringdmaaddr);
	if (Status < 0) {
		return Status;
	}

	Status = nfp_dev_bar2_writeb(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_TXR_SZ(0)),
			log2(TxRing->count));
	if (Status < 0) {
		return Status;
	}

	Status = nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_RXR_ADDR(0)),
			RxRing->ringdmaaddr);
	if (Status < 0) {
		return Status;
	}
	Status = nfp_dev_bar2_writeb(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_RXR_SZ(0)),
			log2(TxRing->count));
	if (Status < 0) {
		return Status;
	}

	//
	// Enable ring 0 only for both RX and TX
	//
	Status = nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_TXRS_ENABLE),
			1);
	if (Status < 0) {
		DBG2("Failed to enable TxRing: 0x%x\n", Status);
		return Status;
	}
	Status = nfp_dev_bar2_writeq(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_RXRS_ENABLE),
			1);
	if (Status < 0) {
		DBG2("Failed to enable RxRing: 0x%x\n", Status);
		goto errDisableDevice;
	}

	Status = nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_MTU),
			MTU_SIZE);
	if (Status < 0) {
		DBG2("Failed to set MTU: 0x%x\n", Status);
		goto errDisableDevice;
	}

	Status = nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_FLBUFSZ),
			RX_BUF_SIZE);
	if (Status < 0) {
		DBG2("Failed to set FL buffer size: 0x%x\n", Status);
		goto errDisableDevice;
	}

	//
	// Finally, update the MAC address in the BAR
	//
	MacAddr32 = cpu_to_be32(*(u32 *)&adapter->CurrentNodeAddress[0]);
	MacAddr16 = cpu_to_be16(*(u32 *)&adapter->CurrentNodeAddress[4]);

	Status = nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_MACADDR),
			MacAddr32);
	if (Status < 0) {
		goto errDisableDevice;
	}
	Status = nfp_dev_bar2_writew(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_MACADDR + 6),
			MacAddr16);
	if (Status < 0) {
		goto errDisableDevice;
	}

	Status = nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_CTRL),
			NFP_NET_CFG_CTRL_ENABLE);
	if (Status < 0) {
		DBG2("Failed to enable vNIC: 0x%x\n", Status);
		goto errDisableDevice;
	}

	Status = nfp_dev_bar2_writel(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, NFP_NET_CFG_UPDATE),
			NFP_NET_CFG_UPDATE_GEN | NFP_NET_CFG_UPDATE_RING);
	if (Status < 0) {
		DBG2("Failed to update vNIC configuration: 0x%x\n",
				Status);
		goto errDisableDevice;
	}

	Status = nfp_reconfig(adapter, VnicIdx);
	if (Status < 0) {
		goto errDisableDevice;
	}

	//
	// Only populate the free list after the device has been reconfigured.
	//
	Status = nfp_refill_freelist(adapter, VnicIdx);
	if (Status < 0) {
		goto errDisableDevice;
	}

	adapter->device_info.adminstate = TRUE;

	return 0;

errDisableDevice:
	nfp_dev_clr_cfg_and_dis(adapter, VnicIdx);
	return Status;
}

/**
  Function initialises a ring structure for descriptors including allocating a
  buffer and setting the DMA mapping for the ring descriptors.

  nfp_ring_cleanup must be called to undo the allocation.

  @param adapter      Structure containing interface specific information.
  @param Ring         Ring structure to allocate data for
  @param QBaseOffset  Queue base offset in the config BAR
  @param VnicIdx      vNIC index for the ring.
  @param QIndex       Queue index for the ring.
  @param DescCount    The number of descriptors in the ring.
  @param DescSize     Byte size of the descriptor stored on the ring

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static int nfp_ring_alloc(struct nfp_adapter *adapter,
		struct qcp_ring *Ring, u32 QBaseOffset,
		u8 VnicIdx, u8 QIndex,
		u16 DescCount, u16 DescSize)
{
	u32 QBase;
	int RequestedBytes;
	int Status;
	struct dma_mapping map;

	//
	// Read the firmware Queue base address
	//
	Status = nfp_dev_bar2_readl(adapter,
			VNIC_CTRL_ADDR(adapter, VnicIdx, QBaseOffset),
			&QBase);
	if (Status < 0) {
		return Status;
	}
	Ring->qptr = NFP_PCIE_QUEUE(QIndex) + QBase * NFP_QCP_QUEUE_ADDR_SZ;
	Ring->count = DescCount;

	Ring->readptr = 0;
	Ring->writeptr = 0;

	//
	// Allocate a host ring buffer and map it for DMA. Stash the DMA device
	// address to pass to the NFP later on.
	//
	RequestedBytes = DescSize * Ring->count;
	Ring->ringdmahostbuffer = dma_alloc(adapter->dma, &map, RequestedBytes, RequestedBytes);

	if (! Ring->ringdmahostbuffer) {
		DBG2("Failed to allocate memory for NFP_TX_DESC ring!\n");
		return -ENOMEM;
	}

	Ring->map = &map;
	Ring->hostbuffersize = RequestedBytes;
	memset(Ring->ringdmahostbuffer, 0, RequestedBytes);

	Ring->ringdmaaddr = dma(&map,  Ring->ringdmahostbuffer);

	return 0;

}

/**
  Reverse the actions of RingAlloc by unmapping the DMA range and freeing the
  DMA buffer for a ring.

  nfp_ring_cleanup must be called to undo the allocation.

  @param adapter      Structure containing interface specific information.
  @param Ring         Ring structure to free data for

 **/
static void nfp_ring_cleanup(struct nfp_adapter *adapter __unused, struct qcp_ring * Ring)
{

	/* Unmap and free DMA-coherent buffer */
	dma_free(Ring->map, Ring->ringdmahostbuffer, Ring->hostbuffersize);
	Ring->writeptr = 0;
	Ring->readptr = 0;
}

/**
  Function initialises the configuration queue.

  We don't need to undo this action.

  @param adapter      Structure containing interface specific information.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static int nfp_init_cfg_queue(struct nfp_adapter *adapter)
{
	u32 QBase;
	int Status;

	//
	// The configuration queue is based on the first odd numbered queue in the
	// TX queue space
	//
	Status = nfp_dev_bar2_readl(adapter,
			VNIC_CTRL_ADDR(adapter, 0,
				NFP_NET_CFG_START_TXQ),
			&QBase);
	if (Status < 0) {
		return Status;
	}
	adapter->cfg_queue.qptr = NFP_PCIE_QUEUE(1) + QBase * NFP_QCP_QUEUE_ADDR_SZ;

	return 0;
}

/**
  Function initialises TX ring and allocates the TX buffer list.

  @param adapter      Structure containing interface specific information.
  @param VnicIdx      vNIC index.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
	static int
nfp_vnic_init_tx(struct nfp_adapter *adapter, u8 VnicIdx)
{
	struct qcp_ring *Ring;
	struct dma_mapping map;
	int Status;
	int bytes;

	Ring = &adapter->tx_ring[VnicIdx].ring;

	Status = nfp_ring_alloc(adapter, Ring,
			NFP_NET_CFG_START_TXQ, VnicIdx, 0,
			NFP_NET_TX_DESCS_UNDI, sizeof(struct tx_desc));
	if (Status < 0) {
		return Status;
	}

	//
	// Allocate the TX buffer list if it hasn't already been allocated.
	//
	bytes = ((int)(NFP_NET_TX_DESCS_UNDI * sizeof(struct buffer)));
	adapter->tx_ring[VnicIdx].freelistsize = bytes;

	adapter->tx_ring[VnicIdx].freelist = dma_alloc(adapter->dma, &map, bytes, bytes);
	if (! adapter->tx_ring[VnicIdx].freelist) {
		Status = -ENOMEM;
		goto errRingCleanup;
	}
	adapter->tx_ring[VnicIdx].freelist->map = &map;

	memset((char *) adapter->tx_ring[VnicIdx].freelist, 0, bytes);

	return 0;

errRingCleanup:
	nfp_ring_cleanup(adapter, Ring);
	return Status;
}

/**
  Function undoes the initialization that 'nfp_vnic_cleanup_tx' performed.

  @param adapter      Structure containing interface specific information.
  @param VnicIdx      vNIC index.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static void nfp_vnic_cleanup_tx(struct nfp_adapter *adapter, u8 VnicIdx)
{
	dma_free(adapter->tx_ring[VnicIdx].freelist->map, 
			(void *)&adapter->tx_ring[VnicIdx].freelist->hostbufferaddr,
			adapter->tx_ring[VnicIdx].freelistsize);
	adapter->tx_ring[VnicIdx].freelist = NULL;
	nfp_ring_cleanup(adapter, &adapter->tx_ring[VnicIdx].ring);
}

/**
  Function initialises the RX ring and allocates the RX free list buffer.

  @param adapter      Structure containing interface specific information.
  @param VnicIdx      vNIC index.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
	static int
nfp_vnic_init_rx(struct nfp_adapter *adapter, u8 VnicIdx)
{
	int Status;
	int bytes;
	struct dma_mapping map;

	Status = nfp_ring_alloc(adapter, &adapter->rx_ring[VnicIdx].ring,
			NFP_NET_CFG_START_RXQ, VnicIdx, 0,
			NFP_NET_RX_DESCS_UNDI, sizeof(struct rx_desc));

	//
	// Set the RX offset as per the NFD version requirements
	//
	if (adapter->version.vermajor >= 2) {
		Status = nfp_dev_bar2_readl(adapter,
				VNIC_CTRL_ADDR(adapter, VnicIdx,
					NFP_NET_CFG_RX_OFFSET),
				&adapter->rx_ring[VnicIdx].rxoffset);
		if (Status < 0) {
			goto errRingCleanup;
		}

		if (adapter->rx_ring[VnicIdx].rxoffset > NFP_NET_MAX_PREPEND) {
			DBG2("Invalid RX offset %d\n",
					adapter->rx_ring[VnicIdx].rxoffset);
			Status = -EINVAL;
			goto errRingCleanup;
		}
	} else {
		adapter->rx_ring[VnicIdx].rxoffset = NFP_NET_RX_OFFSET;
	}

	bytes = ((int)(NFP_NET_RX_DESCS_UNDI * sizeof(struct buffer)));
	adapter->rx_ring[VnicIdx].freelistsize = bytes;

	adapter->rx_ring[VnicIdx].freelist = dma_alloc(adapter->dma, &map, bytes, bytes);

	if (! adapter->rx_ring[VnicIdx].freelist) {
		Status = -ENOMEM;
		goto errRingCleanup;
	}
	adapter->rx_ring[VnicIdx].freelist->map = &map;
	memset((char *) adapter->rx_ring[VnicIdx].freelist, 0, bytes);

	return 0;

errRingCleanup:
	nfp_ring_cleanup(adapter, &adapter->rx_ring[VnicIdx].ring);
	return Status;
}

/**
  Function undoes the initialization that 'nfp_vnic_init_rx' performed.

  @param adapter      Structure containing interface specific information.
  @param VnicIdx      vNIC index.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
static void nfp_vnic_cleanup_rx(struct nfp_adapter *adapter, u8 VnicIdx)
{
	struct buffer *RxBuffer;
	u16 Index;

	//
	// Assumes that the device is stopped and buffers are in [0, Ring.Count - 1]
	//
	for (Index = 0; Index < adapter->rx_ring[VnicIdx].ring.count; Index++) {
		RxBuffer = &adapter->rx_ring[VnicIdx].freelist[Index];

		//
		// The DmaMapping field will only be non-zero if the mapping has been
		// done.
		//
		if (!RxBuffer->ismapped)
			continue;

		/* Free Rx I/O Buffer */
		free_rx_iob(RxBuffer->iobuf);

		/* Unmap the descriptor */
		dma_unmap(RxBuffer->map);
		memset((char *) RxBuffer, 0, sizeof(struct buffer));
	}

	adapter->rx_ring[VnicIdx].freelist = NULL;
	nfp_ring_cleanup(adapter, &adapter->rx_ring[VnicIdx].ring);
}

static int nfp_set_vnic_mac_addr(struct nfp_adapter * adapter)
{
	u8 HwAddr[ETH_ALEN];
	u8 HwAddrIndex;
	struct dma_mapping map;
	void *HostBuffer;
	int Status;
	int Bytes;
	char temp;
	u8 Index;

	Bytes = VNIC_MAC_ADDR_KEY_MAXSIZE;
	HostBuffer = dma_alloc(adapter->dma, &map, Bytes, Bytes);

	if (! HostBuffer) {
		DBG2("Failed to allocate memory for vNIC MAC lookup!\n");
		return -ENOMEM;
	}

	Bytes = snprintf(HostBuffer, Bytes, VNIC_MAC_ADDR_KEY,
			adapter->device_info.interfaceindex,
			adapter->pcie_island);
	DBG2("HWinfo lookup: %s [%d]\n", (char *)HostBuffer, Bytes);

	Status = nfp_write_nsp_buf(adapter, HostBuffer, Bytes);
	if (Status < 0) {
		DBG2("NSP write to buffer error. Status = %x!\n", Status);
		goto outUnlockNsp;
	}

	Status = nfp_nsp_cmd(adapter, NSP_CODE_HWINFO_LOOKUP, VNIC_MAC_ADDR_KEY_MAXSIZE);
	if (Status < 0) {
		// Note, this is a verbose warning since not all systems are expected
		// to have this key
		DBG2("HWinfo lookup failed. Status = %x!\n", Status);
		goto outUnlockNsp;
	}

	Status = nfp_read_nsp_buf(adapter, HostBuffer, VNIC_MAC_ADDR_KEY_MAXSIZE);
	if (Status < 0) {
		DBG2("NSP write to buffer error. Status = %x!\n", Status);
		goto outUnlockNsp;
	}

	DBG2("HWinfo lookup result: %s\n", (char *)HostBuffer);

	HwAddrIndex = 0;
	Index = 0;
	while (HwAddrIndex < ETH_ALEN &&
			Index < (VNIC_MAC_ADDR_KEY_MAXSIZE-1)) {
		temp = ((char *) HostBuffer)[Index];
		if (temp == ':') {
			Index++;
			continue;
		}
		if (temp == 0) {
			break;
		}

		HwAddr[HwAddrIndex] = tohex(temp) << 4;
		temp = ((char *) HostBuffer)[Index + 1];
		HwAddr[HwAddrIndex] |= tohex(temp);
		Index+=2;
		HwAddrIndex++;
	}

	for (Index = 0; Index < ETH_ALEN; Index++) {
		adapter->PermNodeAddress[Index] = HwAddr[Index];
		adapter->CurrentNodeAddress[Index] = HwAddr[Index];
		adapter->BroadcastNodeAddress[Index] = 0xff;
	}

outUnlockNsp:
	dma_free(&map, HostBuffer, VNIC_MAC_ADDR_KEY_MAXSIZE);
	return Status;
}

	static void
nfp_set_phys_mac_addr(struct nfp_adapter * adapter, u8 *PhysPortMac)
{
	u8 Index;

	for (Index = 0; Index < ETH_ALEN; Index++) {
		adapter->PermNodeAddress[Index] =
			PhysPortMac[ETH_ALEN - 1 - Index];
		adapter->CurrentNodeAddress[Index] =
			PhysPortMac[ETH_ALEN - 1 - Index];
		adapter->BroadcastNodeAddress[Index] = 0xff;
	}
}

	static void
nfp_set_mac_addr(struct nfp_adapter * adapter, u8 *PhysPortMac)
{
	int Status;
	u8 Index;

	for (Index = ETH_ALEN; Index < ETH_ALEN; Index++) {
		adapter->PermNodeAddress[Index] = 0;
		adapter->CurrentNodeAddress[Index] = 0;
		adapter->BroadcastNodeAddress[Index] = 0;
	}

	Status = nfp_set_vnic_mac_addr(adapter);
	if (Status < 0) {
		DBG2("Using Physical Function vNIC MAC\n");
		return;
	}

	nfp_set_phys_mac_addr(adapter, PhysPortMac);
	DBG2("Using Physical Port MAC\n");
}

static int nfp_init_iface(struct nfp_adapter * adapter)
{
	struct nsp_eth_tbl *NspPortTable_first_bootable_port = NULL;
	u16 link_speed_first_port = 0;
	u8 BootableDevFirstPort = 0;
	u8 BootableDev = 0;
	struct dev_info *DeviceInfo;
	struct nsp_eth_tbl *NspPortTable;
	u16 link_speed = 0;
	void *NspPortBuffer;
	u8 FirstPort = 0;
	int Status;
	int Bytes;
	int NextPort = 0;
	u8 entry;
	struct dma_mapping map;

	Bytes = NSP_ETH_TABLE_SIZE_BYTES;
	NspPortBuffer = dma_alloc(adapter->dma, &map, Bytes, Bytes);

	if (! NspPortBuffer) {
		DBG2("Failed to allocate memory for NSP_ETH_TABLE. Status!\n");
		return -ENOMEM;
	}

	Status = nfp_nsp_eth_rescan(adapter, NspPortBuffer, Bytes);
	if (Status < 0) {
		goto outFreeBuffer;
	}

	//
	// Look for an enabled and bootable interface
	//
	for (entry = NextPort; entry < MAX_NSP_ETH_TABLE_ENTRIES; entry++) {
		NspPortTable = (struct nsp_eth_tbl *) NspPortBuffer + entry;

		DBG2("entry=%d, lanes=0x%llx, state=0x%llx\n",
				entry, NSP_ETH_PORT_LANES(NspPortTable->port),
				NspPortTable->state);

		if (!NSP_ETH_PORT_LANES(NspPortTable->port))
			continue;
		if (!(NspPortTable->state & NSP_ETH_STATE_ENABLED))
			continue;
		if (!(NspPortTable->state & NSP_ETH_STATE_BOOTABLE))
			continue;

		NextPort = entry + 1;
		BootableDev = entry;
		NspPortTable = (struct nsp_eth_tbl *) NspPortBuffer + BootableDev;

		//
		// EnableRx for selected BootableDev
		//
		if (!(NspPortTable->state & NSP_ETH_STATE_ENABLED_RX)) {

			NspPortTable->ctrl =
				(NspPortTable->state & 0xf) | NSP_ETH_STATE_ENABLED_RX;

			Status = nfp_nsp_eth_cfg(adapter, NspPortBuffer, Bytes);
			if (Status < 0) {
				goto outFreeBuffer;
			}
		}

		usleep(1000);

		//
		// Re-scan to get updated link status
		//
		Status = nfp_nsp_eth_rescan(adapter, NspPortBuffer, Bytes);
		if (Status < 0) {
			goto outFreeBuffer;
		}

		switch (NSP_ETH_STATE_RATE_of(NspPortTable->state)) {
			case 0:
				link_speed = 0;
				break;
			case 1:
				link_speed = 0;
				break;
			case 2:
				link_speed = 0;
				break;
			case 3:
				link_speed = NSP_ETH_PORT_LANES(NspPortTable->port) * 1;
				break;
			case 4:
				link_speed = NSP_ETH_PORT_LANES(NspPortTable->port) * 10;
				break;
			case 5:
				link_speed = NSP_ETH_PORT_LANES(NspPortTable->port) * 25;
				break;
		}

		if (!(FirstPort)) {
			NspPortTable_first_bootable_port = NspPortTable;
			BootableDevFirstPort = BootableDev;
			link_speed_first_port = link_speed;
			FirstPort++;
		}

		if ((NspPortTable->state & NSP_ETH_STATE_ENABLED)
				&& (NspPortTable->state & NSP_ETH_STATE_LINK)) {
			DBG2("Link UP    : %llx\n",
					(NspPortTable->state >> 4) & 1);
		} else {
			//
			// If no link on this port, check the other ports for link.
			//
			continue;
		}

		DBG2("Port Entry : %d\n", BootableDev);
		DBG2("Link Speed : %d Gbps\n", link_speed);
		break;
	}

	if ((entry > (MAX_NSP_ETH_TABLE_ENTRIES - 1)) && (!(FirstPort))) {
		DBG2("No bootable media detected!\n");
		Status = -ENODEV;
		goto outFreeBuffer;
	}

	if (entry > (MAX_NSP_ETH_TABLE_ENTRIES - 1)) {
		DBG2("No link on all enabled ports!. Defaulting to port %d\n",
				BootableDevFirstPort);
		NspPortTable = NspPortTable_first_bootable_port;
		BootableDev = BootableDevFirstPort;
		link_speed = link_speed_first_port;
	} else {
		NspPortTable = (struct nsp_eth_tbl *) NspPortBuffer + entry;
	}

	DeviceInfo = (struct dev_info *) &adapter->device_info;
	DeviceInfo->interfacenum = BootableDev;
	DeviceInfo->interfaceindex = 0;
	DeviceInfo->linkspeed = link_speed;
	DeviceInfo->mtusize = MTU_SIZE;
	DeviceInfo->linkstatus = ((NspPortTable->state >> 4) & 1);
	DeviceInfo->adminstate = 0;

	//
	// The link speed depends on the value read.
	//
	switch (DeviceInfo->linkspeed) {
		case DEV_SUPP_SPEED_NOT_VALID:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_NOT_VALID;

			DBG2(" Link Speed Not Valid\n");
			Status = -ENODEV;
			goto outFreeBuffer;
		case DEV_SUPP_SPEED_1G:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_1G;
			break;
		case DEV_SUPP_SPEED_10G:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_10G;
			break;
		case DEV_SUPP_SPEED_25G:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_25G;
			break;
		case DEV_SUPP_SPEED_40G:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_40G;
			break;
		case DEV_SUPP_SPEED_50G:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_50G;
			break;
		case DEV_SUPP_SPEED_100G:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_100G;
			break;
		default:
			DeviceInfo->linkspeedvalue = DEV_LINK_SPEED_NOT_VALID;

			DBG2(" Link Speed Not Valid\n");
			Status = -EIO;
			goto outFreeBuffer;
	}

	//
	// Update MAC addresses to be used by the PXE layer later.
	//
	nfp_set_mac_addr(adapter, (u8 *)&NspPortTable->mac);

	Status = 0;

outFreeBuffer:
	dma_free(&map, NspPortBuffer, ((int) Bytes));
	return Status;
}

/**
  Function initialises the device, specifically the active vNIC and
  configuration queue.

  @param adapter      Structure containing interface specific information.

  @retval 0 The operation completed successfully.
  @retval Others      An unexpected error occurred.

 **/
int nfp_dev_init(struct nfp_adapter *adapter)
{
	u32 Interface = adapter->device_info.interfaceindex;
	struct pci_device *pdev = adapter->pdev;
	int Status;

	// Read the relevant device information from the PCI configuration header obtained
	// on driver start.
	adapter->vendor_id = pdev->vendor;
	adapter->device_id = pdev->device;
	pci_read_config_byte(pdev, PCI_REVISION, &adapter->rev_id);
	pci_read_config_word(pdev, PCI_SUBSYSTEM_VENDOR_ID, &adapter->sub_vendor_id);
	pci_read_config_word(pdev, PCI_SUBSYSTEM_ID, &adapter->sub_system_id);

	Status = nfp_vnic_init_tx(adapter, Interface);
	if (Status < 0) {
		return Status;
	}
	Status = nfp_vnic_init_rx(adapter, Interface);
	if (Status < 0) {
		goto errCleanTxVnic;
	}
	Status = nfp_init_cfg_queue(adapter);
	if (Status < 0) {
		goto errCleanRxVnic;
	}

	nfp_dev_conf_and_en(adapter, Interface);

	if (Status < 0) {
		goto errCleanRxVnic;
	}

	return 0;

errCleanRxVnic:
	nfp_vnic_cleanup_rx(adapter, Interface);
errCleanTxVnic:
	nfp_vnic_cleanup_tx(adapter, Interface);
	return Status;
}

	static int
nfp_get_nsp_resource(struct nfp_adapter *adapter, u64 *Address,
		struct nfp_resource_entry *Resource)
{
	u64 TableAddr;
	int Status;
	u32 Iter;

	//
	// BAR mapping is configured such that we can start at offset 0.
	//
	TableAddr = 0;

	for (Iter = 0; Iter < NFP_RESOURCE_TBL_ENTRIES; Iter++) {

		Status = nfp_dev_data_bar_read(adapter, &adapter->bar[8],
				NFP_RESOURCE_MUR_ADDR(TableAddr), sizeof(u32),
				sizeof(struct nfp_resource_entry) / 4, Resource);
		if (Status) {
			DBG2("%s: NspMutex: PCI IO Read Error\n",
					__func__);
			return Status;
		}

		if (Resource->mutex.key != RESOURCE_NFPSP_KEY) {
			TableAddr += sizeof(struct nfp_resource_entry);
			continue;
		}
		*Address = TableAddr;

		DBG2("MUTEX: Found key 0x%x at 0x%llx\n",
				Resource->mutex.key, TableAddr);
		DBG2("  MutexOwner=0x%x\n", Resource->mutex.owner);
		DBG2("  MutexKey=0x%x\n", Resource->mutex.key);

		return 0;
	}

	// We rely on the NSP, if we can't find the nfp.sp resource, UNDI is not
	// supported.
	return -1;
}

/** @file
 *
 * NFP PXE network driver
 *
 */


/**
 * nfp_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 *
 * Free all transmit software resources
 **/
void nfp_free_tx_resources ( struct nfp_adapter *adapter )
{
	DBG ( "nfp_free_tx_resources\n" );

	nfp_vnic_cleanup_tx(adapter, 0);
}

/**
 * nfp_free_rx_resources - Free Rx Resources
 * @adapter: board private structure
 *
 * Free all receive software resources
 **/
void nfp_free_rx_resources ( struct nfp_adapter *adapter )
{
	//int i;

	DBG ( "nfp_free_rx_resources\n" );

	nfp_vnic_cleanup_rx(adapter, 0);

	/*
	   for ( i = 0; i < NUM_RX_DESC; i++ ) {
	   free_iob ( adapter->rx_iobuf[i] );
	   }
	   */
}

/**
 * nfp_process_tx_packets - process transmitted packets
 *
 * @v netdev    network interface device structure
 **/
static void nfp_process_tx_packets ( struct net_device *netdev )
{
	struct nfp_adapter *adapter = netdev_priv ( netdev );
	//uint32_t i;
	uint32_t nfp_rdptr;
	struct qcp_ring *ring;
	//struct tx_desc *tx_curr_desc;

	/* Check status of transmitted packets
	*/

	ring = &adapter->tx_ring[0].ring;
	/* Get NFP read ptr (how many pkts NFP has processed) */
	if (nfp_get_qcp_rdptr(adapter, ring, &nfp_rdptr) < 0)
		return;

	DBGP("process_tx_packets: tx_head = %d, tx_tail = %d\n", ring->writeptr, ring->readptr);

	/* Advance our read ptr until NPF's read ptr */
	while ( ring->readptr != nfp_rdptr) {

		//tx_curr_desc = (struct tx_desc *)adapter->tx_ring[VnicIdx].ring.ringdmahostbuffer[i * sizeof(struct tx_desc)];
		/* Complete TX descriptor */
		netdev_tx_complete_next(netdev);
		adapter->tx_ring[0].ring.readptr++;
	}
}

/**
 * nfp_process_rx_packets - process received packets
 *
 * @v netdev    network interface device structure
 **/
static void nfp_process_rx_packets ( struct net_device *netdev )
{
	struct nfp_adapter *adapter = netdev_priv ( netdev );
	struct qcp_ring *rx_ring;
	uint32_t wrptr;
	uint32_t rdptr;
	struct rx_desc *rx_curr_desc;
	struct buffer *rxbuffer;
	struct io_buffer *iobuf;
	uint16_t MetaLen;
	uint16_t PacketLen;
	uint16_t PacketOffset __unused;
	//uint16_t MetaOffset;

	DBGP ( "nfp_process_rx_packets\n" );

	rx_ring = &adapter->rx_ring[0].ring;

	wrptr = rx_ring->writeptr;
	rdptr = rx_ring->readptr;

	DBGP ("process_rx_packets: rx_head = %d, rx_tail = %d\n", wrptr, rdptr);

	/* Process received packets
	*/
	while (rdptr != wrptr) {
		rx_curr_desc = (struct rx_desc *)&rx_ring->ringdmahostbuffer[rdptr * sizeof(struct rx_desc)];

		rmb();

		// Do we have any packets waiting
		if (!(rx_curr_desc->MetaLenDd & PCIE_DESC_RX_DD)) {
			break;
		}

		rxbuffer = &adapter->rx_ring[0].freelist[rdptr];

		if (rx_curr_desc->DataLen == 0) {
			DBGP("Packet RXed without data! Discarding\n");
			continue;
		}

		MetaLen = rx_curr_desc->MetaLenDd & PCIE_DESC_RX_META_LEN_MASK;
		PacketLen = rx_curr_desc->DataLen - MetaLen;

		if (adapter->rx_ring[0].rxoffset == NFP_NET_CFG_RX_OFFSET_DYNAMIC)
			PacketOffset = MetaLen;
		else
			PacketOffset = adapter->rx_ring[0].rxoffset;

		//MetaOffset = PacketOffset - MetaLen;

		if (PacketLen > RX_BUF_SIZE)
			PacketLen = RX_BUF_SIZE;

		iobuf = rxbuffer->iobuf;
		iob_put(iobuf, PacketLen);
		rx_ring->readptr++;
		rdptr = rx_ring->readptr;
		netdev_rx (netdev, iobuf);
	}
}

/**
 * nfp_transmit - Transmit a packet
 *
 * @v netdev    Network device
 * @v iobuf     I/O buffer
 *
 * @ret rc       Returns 0 on success, negative on failure
 */
static int nfp_transmit ( struct net_device *netdev, struct io_buffer *iobuf __unused )
{
	struct nfp_adapter *adapter = netdev_priv ( netdev );
	//struct tx_desc *tx_curr_desc;

	DBGP ("nfp_transmit: 0x%p\n", adapter);

	return 0;
}

static int nfp_nsp_init ( struct nfp_adapter *adapter )
{

	int err = 0;
	struct nfp_resource_entry resource;
	u64 temp;
	u32 BarMapping;
	struct nfp_bar *bar;

	bar = &adapter->bar[(NETRONOME_DEV_BAR_1 * 8) + NETRONOME_EXP_BAR_0];
	err = nfp_cfg_expbar(adapter, bar, NETRONOME_DEV_BAR_1_0_MAP);
	if (err < 0)
		return -1;

	bar = &adapter->bar[(NETRONOME_DEV_BAR_1 * 8) + NETRONOME_EXP_BAR_1];
	err = nfp_cfg_expbar(adapter, bar, NETRONOME_DEV_BAR_1_1_MAP);
	if (err < 0)
		return -1;

	bar = &adapter->bar[(NETRONOME_DEV_BAR_1 * 8) + NETRONOME_EXP_BAR_2];
	err = nfp_cfg_expbar(adapter, bar, NETRONOME_DEV_BAR_1_2_MAP);
	if (err < 0)
		return -1;

	err = nfp_get_nsp_resource(adapter, &temp, &resource);
	if (err < 0)
		return -1;

	DBGP("Resource nfp.sp:\n");
	DBG2("   RegionCppTarget=0x%x\n", resource.region.cpp_target);
	DBG2("   RegionCppAction=0x%x\n", resource.region.cpp_action);
	DBG2("   RegionCppToken=0x%x\n", resource.region.cpp_token);
	DBG2("   RegionPageOffset=0x%x ~ 0x%x\n",
			resource.region.page_offset, resource.region.page_offset << 8);
	DBG2("   RegionPageSize=0x%x ~ 0x%x\n",
			resource.region.page_size, resource.region.page_size << 8);


	err = nfp_compute_bar(resource.region.cpp_target,
			resource.region.cpp_action,
			resource.region.cpp_token,
			(u64)resource.region.page_offset << 8,
			resource.region.page_size << 8,
			NFP_COMPUTE_BAR_WIDTH_64,
			NETRONOME_DEV_BAR_1_BITSIZE,
			&BarMapping, &NspCsrOffset);
	if (err) {
		DBG2("Failed to compute BAR: %x\n", err);
		return err;
	}

	bar = &adapter->bar[(NETRONOME_DEV_BAR_1 * 8) + NETRONOME_EXP_BAR_3];
	err = nfp_cfg_expbar(adapter, bar, BarMapping);
	if (err)
		return err;

	// Since we use subsection #3, we modify the offset appropriately.
	NspCsrOffset += 3 * NETRONOME_DEV_BAR_1_SLICE;

	DBG2("NSP CSR Base: Mapping=0x%x Offset=0x%llx\n",
			BarMapping, NspCsrOffset);

	// We read and store the NSP default buffer location. This cannot change
	// during run time.
	err = nfp_dev_data_bar_read(adapter, &adapter->bar[8],
			NspCsrOffset + NSP_DFLT_BUFFER, sizeof(u64), 1,
			&NspDefaultBuffer);
	if (err) {
		return err;
	}

	DBG2("NSP Default Buffer:\n");
	DBG2("   RAW=0x%llx\n", NspDefaultBuffer);
	DBG2("   Target=0x%x\n",
			(int)NSP_DFLT_BUFFER_CPP_TARGET_of(NspDefaultBuffer));
	DBG2("   Action=0x%x\n",
			(int)NSP_DFLT_BUFFER_CPP_ACTION_of(NspDefaultBuffer));
	DBG2("   Token=0x%x\n",
			(int)NSP_DFLT_BUFFER_CPP_TOKEN_of(NspDefaultBuffer));
	DBG2("   Address=0x%llx\n",
			NSP_DFLT_BUFFER_ADDRESS_of(NspDefaultBuffer));

	// Note that the size specified here is only used to vet the mapping. We
	// specify the minimum size of the NSP buffer, which is 1Mb. We won't
	// use that much from the UNDI driver in any case.
	err = nfp_compute_bar(NSP_DFLT_BUFFER_CPP_TARGET_of(NspDefaultBuffer),
			NSP_DFLT_BUFFER_CPP_ACTION_of(NspDefaultBuffer),
			NSP_DFLT_BUFFER_CPP_TOKEN_of(NspDefaultBuffer),
			NSP_DFLT_BUFFER_ADDRESS_of(NspDefaultBuffer),
			0x100000, NFP_COMPUTE_BAR_WIDTH_32,
			NETRONOME_DEV_BAR_1_BITSIZE,
			&BarMapping, &NspDefaultBufferOffset);
	if (err) {
		DBG2("Failed to compute BAR: %x\n", err);
		return err;
	}

	bar = &adapter->bar[(NETRONOME_DEV_BAR_1 * 8) + NETRONOME_EXP_BAR_4];
	err = nfp_cfg_expbar(adapter, bar, BarMapping);
	if (err) {
		return err;
	}

	// Since we use subsection #4, we modify the offset appropriately.
	NspDefaultBufferOffset += 4 * NETRONOME_DEV_BAR_1_SLICE;

	DBG2("NSP buffer: Mapping=0x%x Offset=0x%llx\n", BarMapping, NspDefaultBufferOffset);

	return 0;
}

/******************************************************************************
 *
 * Device reset
 *
 ******************************************************************************
 */

/**
 * Reset hardware
 *
 * @v adapter		NFP device
 * @ret rc		Return status code
 */
static int nfp_reset ( struct nfp_adapter *adapter ) {

	DBGC ( adapter, "NFP %p does not yet support reset\n", adapter );
	return -ENOTSUP;
}

/**
 * NFP SW init
 *
 * @v adapter		NFP device
 * @ret rc		Return status code
 */
static int nfp_sw_init ( struct nfp_adapter *adapter ) {

	int err;
	DBGC ( adapter, "NFP %p does not yet support sw init/reset\n", adapter );
	err = nfp_init_iface(adapter);
	if (err)
		return -1;

	err = nfp_cfg_vnic_bar(adapter);
	if (err)
		return -1;

	err = nfp_read_ver_info(adapter);
	if (err)
		return -1;

	return 0;
}
/******************************************************************************
 *
 * Link state
 *
 ******************************************************************************
 */

/**
 * Check link state
 *
 * @v netdev		Network device
 */
static void nfp_check_link ( struct net_device *netdev ) {
	struct nfp_adapter *adapter = netdev->priv;

	DBGC ( adapter, "NFP %p does not yet support link state\n", adapter );
	netdev_link_err ( netdev, -ENOTSUP );
}

/******************************************************************************
 *
 * Network device interface
 *
 ******************************************************************************
 */

/**
 * Open network device
 *
 * @v netdev		Network device
 * @ret rc		Return status code
 */
static int nfp_open ( struct net_device *netdev ) {
	struct nfp_adapter *adapter = netdev->priv;

	DBG ("nfp_open\n");
	memcpy ( adapter->hw_addr, netdev->ll_addr, ETH_ALEN );

	return nfp_dev_init(adapter);
}

/**
 * Close network device
 *
 * @v netdev		Network device
 */
static void nfp_close ( struct net_device *netdev ) {
	struct nfp_adapter *adapter = netdev->priv;

	DBGC ( adapter, "NFP %p close\n", adapter );

	/* disable receive */

	mdelay(30);

	/* Free Tx resources */
	nfp_free_tx_resources(adapter);

	/* Free Rx resources */
	nfp_free_rx_resources(adapter);
}

/**
 * Poll for completed and received packets
 *
 * @v netdev		Network device
 */
static void nfp_poll ( struct net_device *netdev ) {
	struct nfp_adapter *adapter = netdev->priv;

	DBGP("nfp_poll\n");

	nfp_process_tx_packets(netdev);

	nfp_process_rx_packets(netdev);

	nfp_refill_freelist(adapter, 0);

}

/**
 * Enable or disable interrupts
 *
 * @v netdev		Network device
 * @v enable		Interrupts should be enabled
 */
static void nfp_irq ( struct net_device *netdev, int enable ) {
	struct nfp_adapter *adapter = netdev->priv;

	DBG ("NFP %p does not yet support interrupts\n", adapter );
	( void ) enable;
}

/** NFP network device operations */
static struct net_device_operations nfp_operations = {
	.open		= nfp_open,
	.close		= nfp_close,
	.transmit	= nfp_transmit,
	.poll		= nfp_poll,
	.irq		= nfp_irq,
};

/******************************************************************************
 *
 * PCI interface
 *
 ******************************************************************************
 */

/**
 * Probe PCI device
 *
 * @v pdev		PCI device
 * @ret rc		Return status code
 */
static int nfp_probe ( struct pci_device *pdev ) {
	struct net_device *netdev;
	struct nfp_adapter *adapter;
	struct nfp_bar *bar;
	unsigned long mmio_start, mmio_len;
	int rc = -EINVAL;

	printf ( "adapter_probe\n" );

	/* Allocate and initialise net device */
	netdev = alloc_etherdev ( sizeof ( *adapter ) );
	if ( ! netdev ) {
		rc = -ENOMEM;
		goto err_alloc;
	}

	/* Associate nfp specific network operations with
	 * generic network device layer */
	netdev_init ( netdev, &nfp_operations );

	/* Associate this network device with given PCI device */	
	pci_set_drvdata ( pdev, netdev );
	netdev->dev = &pdev->dev;

	/* Initialize driver private storage */
	adapter = netdev->priv;
	memset ( adapter, 0, sizeof ( *adapter ) );

	adapter->pdev = pdev;
	adapter->dma  = &pdev->dma;
	dma_set_mask_64bit(adapter->dma);
	netdev->dma = &pdev->dma;

	adapter->ioaddr = pdev->ioaddr;

	/* Fix up PCI device - Set device to be a busmaster */
	adjust_pci_device ( pdev );

	/* Map registers */
	adapter->config = pci_ioremap ( pdev, pdev->membase, NFP_BAR_SIZE );
	if ( ! adapter->config ) {
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* pci_bar_start = pci_resource_start */
	mmio_start = pci_bar_start ( pdev, PCI_BASE_ADDRESS_0 );
	/* pci_bar_size = pci_resource_len */
	mmio_len   = pci_bar_size  ( pdev, PCI_BASE_ADDRESS_0 );

	DBG ( "mmio_start 0: %#08lx\n", mmio_start );
	DBG ( "mmio_len 0: %#08lx\n", mmio_len );

	/* Base Address for BAR_0.0 to use in writel()/readl() */
	adapter->bar[0].iomem = pci_ioremap ( pdev, mmio_start, mmio_len );
	DBG ( "adapter->bar[0].iomem: %p\n", adapter->bar[0].iomem );

	if ( ! adapter->bar[0].iomem ) {
		DBG ( "err_ioremap\n" );
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* pci_bar_start = pci_resource_start */
	mmio_start = pci_bar_start ( pdev, PCI_BASE_ADDRESS_2 );
	/* pci_bar_size = pci_resource_len */
	mmio_len   = pci_bar_size  ( pdev, PCI_BASE_ADDRESS_2 );

	DBG ( "mmio_start 2: %#08lx\n", mmio_start );
	DBG ( "mmio_len 2: %#08lx\n", mmio_len );

	/* Base Address for BAR_1.0 to use in writel()/readl() */
	adapter->bar[8].iomem = pci_ioremap ( pdev, mmio_start, mmio_len );
	DBG ( "adapter->bar[8].iomem: %p\n", adapter->bar[8].iomem );

	if ( ! adapter->bar[8].iomem ) {
		DBG ( "err_ioremap\n" );
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* pci_bar_start = pci_resource_start */
	mmio_start = pci_bar_start ( pdev, PCI_BASE_ADDRESS_4 );
	/* pci_bar_size = pci_resource_len */
	mmio_len   = pci_bar_size  ( pdev, PCI_BASE_ADDRESS_4 );

	DBG ( "mmio_start 4: %#08lx\n", mmio_start );
	DBG ( "mmio_len 4: %#08lx\n", mmio_len );

	/* Base Address for BAR_2.0 to use in writel()/readl() */
	adapter->bar[16].iomem = pci_ioremap ( pdev, mmio_start, mmio_len );
	DBG ( "adapter->bar[16].iomem: %p\n", adapter->bar[16].iomem );

	if ( ! adapter->bar[16].iomem ) {
		DBG ( "err_ioremap\n" );
		rc = -ENODEV;
		goto err_ioremap;
	}

	/* Setup required NFP BAR's and get NSP resource */
	rc = nfp_nsp_init( adapter );
	if (rc) {
		DBG ( "err_nsp_init\n" );
		goto err_nsp_init;
	}

	bar = &adapter->bar[(NETRONOME_DEV_BAR_0 * 8) + NETRONOME_EXP_BAR_1];
	rc = nfp_cfg_expbar(adapter, bar, NETRONOME_DEV_BAR_0_1_MAP);
	if (rc < 0)
		goto err_sw_init;

	/* setup adapter struct */
	rc = nfp_sw_init ( adapter );
	if (rc) {
		DBG ( "err_sw_init\n" );
		goto err_sw_init;
	}

	/* Register network device */
	if ( ( rc = register_netdev ( netdev ) ) != 0 )
		goto err_register_netdev;

	/* Set initial link state */
	nfp_check_link ( netdev );

	return 0;

	unregister_netdev ( netdev );
err_register_netdev:
err_sw_init:
	nfp_reset ( adapter );
err_nsp_init:
	iounmap ( adapter->config );
err_ioremap:
	netdev_nullify ( netdev );
	netdev_put ( netdev );
err_alloc:
	return rc;
}

/**
 * Remove PCI device
 *
 * @v pci		PCI device
 */
static void nfp_remove ( struct pci_device *pdev ) {
	struct net_device *netdev = pci_get_drvdata ( pdev );
	struct nfp_adapter *adapter = netdev->priv;

	DBG ( "nfp_remove\n" );

	/* Unregister network device */
	unregister_netdev ( netdev );

	/* Reset card */
	nfp_reset ( adapter );

	/* Free network device */
	iounmap ( adapter->config );
	netdev_nullify ( netdev );
	netdev_put ( netdev );
}

/** NFP PCI device IDs */
static struct pci_device_id nfp_adapters[] = {
	PCI_ROM ( 0x19ee, 0x6000, "nfp",	"NFP6000", 0 ),
	PCI_ROM ( 0x19ee, 0x5000, "nfp",	"NFP5000", 0 ),
	PCI_ROM ( 0x19ee, 0x4000, "nfp",	"NFP4000", 0 ),
};

/** NFP PCI driver */
struct pci_driver nfp_driver __pci_driver = {
	.ids = nfp_adapters,
	.id_count = ( sizeof ( nfp_adapters ) / sizeof ( nfp_adapters[0] ) ),
	.probe = nfp_probe,
	.remove = nfp_remove,
};
