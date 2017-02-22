/*
 * IOMMU API for ARM architected SMMU implementations.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * Copyright (C) 2013 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 *
 * This driver currently supports:
 *	- SMMUv1 and v2 implementations
 *	- Stream-matching and stream-indexing
 *	- v7/v8 long-descriptor format
 *	- Non-secure access to the SMMU
 *	- Context fault reporting
 */

#define pr_fmt(fmt) "arm-smmu: " fmt

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <soc/qcom/secure_buffer.h>
#include <linux/of_platform.h>
#include <linux/msm-bus.h>
#include <dt-bindings/msm/msm-bus-ids.h>

#include <linux/amba/bus.h>

#include "io-pgtable.h"

/* Maximum number of stream IDs assigned to a single device */
#define MAX_MASTER_STREAMIDS		128

/* Maximum number of context banks per SMMU */
#define ARM_SMMU_MAX_CBS		128

/* SMMU global address space */
#define ARM_SMMU_GR0(smmu)		((smmu)->base)
#define ARM_SMMU_GR1(smmu)		((smmu)->base + (1 << (smmu)->pgshift))

/*
 * SMMU global address space with conditional offset to access secure
 * aliases of non-secure registers (e.g. nsCR0: 0x400, nsGFSR: 0x448,
 * nsGFSYNR0: 0x450)
 */
#define ARM_SMMU_GR0_NS(smmu)						\
	((smmu)->base +							\
		((smmu->options & ARM_SMMU_OPT_SECURE_CFG_ACCESS)	\
			? 0x400 : 0))

/*
 * Some 64-bit registers only make sense to write atomically, but in such
 * cases all the data relevant to AArch32 formats lies within the lower word,
 * therefore this actually makes more sense than it might first appear.
 */
#ifdef CONFIG_64BIT
#define smmu_write_atomic_lq		writeq_relaxed
#else
#define smmu_write_atomic_lq		writel_relaxed
#endif

/* Configuration registers */
#define ARM_SMMU_GR0_sCR0		0x0
#define sCR0_CLIENTPD			(1 << 0)
#define sCR0_GFRE			(1 << 1)
#define sCR0_GFIE			(1 << 2)
#define sCR0_GCFGFRE			(1 << 4)
#define sCR0_GCFGFIE			(1 << 5)
#define sCR0_USFCFG			(1 << 10)
#define sCR0_VMIDPNE			(1 << 11)
#define sCR0_PTM			(1 << 12)
#define sCR0_FB				(1 << 13)
#define sCR0_VMID16EN			(1 << 31)
#define sCR0_BSU_SHIFT			14
#define sCR0_BSU_MASK			0x3

/* Auxiliary Configuration register */
#define ARM_SMMU_GR0_sACR		0x10

/* Identification registers */
#define ARM_SMMU_GR0_ID0		0x20
#define ARM_SMMU_GR0_ID1		0x24
#define ARM_SMMU_GR0_ID2		0x28
#define ARM_SMMU_GR0_ID3		0x2c
#define ARM_SMMU_GR0_ID4		0x30
#define ARM_SMMU_GR0_ID5		0x34
#define ARM_SMMU_GR0_ID6		0x38
#define ARM_SMMU_GR0_ID7		0x3c
#define ARM_SMMU_GR0_sGFSR		0x48
#define ARM_SMMU_GR0_sGFSYNR0		0x50
#define ARM_SMMU_GR0_sGFSYNR1		0x54
#define ARM_SMMU_GR0_sGFSYNR2		0x58

#define ID0_S1TS			(1 << 30)
#define ID0_S2TS			(1 << 29)
#define ID0_NTS				(1 << 28)
#define ID0_SMS				(1 << 27)
#define ID0_ATOSNS			(1 << 26)
#define ID0_PTFS_NO_AARCH32		(1 << 25)
#define ID0_PTFS_NO_AARCH32S		(1 << 24)
#define ID0_CTTW			(1 << 14)
#define ID0_NUMIRPT_SHIFT		16
#define ID0_NUMIRPT_MASK		0xff
#define ID0_NUMSIDB_SHIFT		9
#define ID0_NUMSIDB_MASK		0xf
#define ID0_NUMSMRG_SHIFT		0
#define ID0_NUMSMRG_MASK		0xff

#define ID1_PAGESIZE			(1 << 31)
#define ID1_NUMPAGENDXB_SHIFT		28
#define ID1_NUMPAGENDXB_MASK		7
#define ID1_NUMS2CB_SHIFT		16
#define ID1_NUMS2CB_MASK		0xff
#define ID1_NUMCB_SHIFT			0
#define ID1_NUMCB_MASK			0xff

#define ID2_OAS_SHIFT			4
#define ID2_OAS_MASK			0xf
#define ID2_IAS_SHIFT			0
#define ID2_IAS_MASK			0xf
#define ID2_UBS_SHIFT			8
#define ID2_UBS_MASK			0xf
#define ID2_PTFS_4K			(1 << 12)
#define ID2_PTFS_16K			(1 << 13)
#define ID2_PTFS_64K			(1 << 14)
#define ID2_VMID16			(1 << 15)

#define ID7_MAJOR_SHIFT			4
#define ID7_MAJOR_MASK			0xf

/* Global TLB invalidation */
#define ARM_SMMU_GR0_TLBIVMID		0x64
#define ARM_SMMU_GR0_TLBIALLNSNH	0x68
#define ARM_SMMU_GR0_TLBIALLH		0x6c
#define ARM_SMMU_GR0_sTLBGSYNC		0x70
#define ARM_SMMU_GR0_sTLBGSTATUS	0x74
#define sTLBGSTATUS_GSACTIVE		(1 << 0)
#define TLB_LOOP_TIMEOUT		500000	/* 500ms */

/* Stream mapping registers */
#define ARM_SMMU_GR0_SMR(n)		(0x800 + ((n) << 2))
#define SMR_VALID			(1 << 31)
#define SMR_MASK_SHIFT			16
#define SMR_ID_SHIFT			0

#define ARM_SMMU_GR0_S2CR(n)		(0xc00 + ((n) << 2))
#define S2CR_CBNDX_SHIFT		0
#define S2CR_CBNDX_MASK			0xff
#define S2CR_TYPE_SHIFT			16
#define S2CR_TYPE_MASK			0x3
enum arm_smmu_s2cr_type {
	S2CR_TYPE_TRANS,
	S2CR_TYPE_BYPASS,
	S2CR_TYPE_FAULT,
};

#define S2CR_PRIVCFG_SHIFT		24
#define S2CR_PRIVCFG_MASK		0x3
enum arm_smmu_s2cr_privcfg {
	S2CR_PRIVCFG_DEFAULT,
	S2CR_PRIVCFG_DIPAN,
	S2CR_PRIVCFG_UNPRIV,
	S2CR_PRIVCFG_PRIV,
};

/* Context bank attribute registers */
#define ARM_SMMU_GR1_CBAR(n)		(0x0 + ((n) << 2))
#define CBAR_VMID_SHIFT			0
#define CBAR_VMID_MASK			0xff
#define CBAR_S1_BPSHCFG_SHIFT		8
#define CBAR_S1_BPSHCFG_MASK		3
#define CBAR_S1_BPSHCFG_NSH		3
#define CBAR_S1_MEMATTR_SHIFT		12
#define CBAR_S1_MEMATTR_MASK		0xf
#define CBAR_S1_MEMATTR_WB		0xf
#define CBAR_TYPE_SHIFT			16
#define CBAR_TYPE_MASK			0x3
#define CBAR_TYPE_S2_TRANS		(0 << CBAR_TYPE_SHIFT)
#define CBAR_TYPE_S1_TRANS_S2_BYPASS	(1 << CBAR_TYPE_SHIFT)
#define CBAR_TYPE_S1_TRANS_S2_FAULT	(2 << CBAR_TYPE_SHIFT)
#define CBAR_TYPE_S1_TRANS_S2_TRANS	(3 << CBAR_TYPE_SHIFT)
#define CBAR_IRPTNDX_SHIFT		24
#define CBAR_IRPTNDX_MASK		0xff

#define ARM_SMMU_GR1_CBFRSYNRA(n)	(0x400 + ((n) << 2))
#define CBFRSYNRA_SID_MASK		(0xffff)

#define ARM_SMMU_GR1_CBA2R(n)		(0x800 + ((n) << 2))
#define CBA2R_RW64_32BIT		(0 << 0)
#define CBA2R_RW64_64BIT		(1 << 0)
#define CBA2R_VMID_SHIFT		16
#define CBA2R_VMID_MASK			0xffff

/* Translation context bank */
#define ARM_SMMU_CB_BASE(smmu)		((smmu)->base + ((smmu)->size >> 1))
#define ARM_SMMU_CB(smmu, n)		((n) * (1 << (smmu)->pgshift))

#define ARM_SMMU_CB_SCTLR		0x0
#define ARM_SMMU_CB_ACTLR		0x4
#define ARM_SMMU_CB_RESUME		0x8
#define ARM_SMMU_CB_TTBCR2		0x10
#define ARM_SMMU_CB_TTBR0		0x20
#define ARM_SMMU_CB_TTBR1		0x28
#define ARM_SMMU_CB_TTBCR		0x30
#define ARM_SMMU_CB_CONTEXTIDR		0x34
#define ARM_SMMU_CB_S1_MAIR0		0x38
#define ARM_SMMU_CB_S1_MAIR1		0x3c
#define ARM_SMMU_CB_PAR			0x50
#define ARM_SMMU_CB_FSR			0x58
#define ARM_SMMU_CB_FSRRESTORE		0x5c
#define ARM_SMMU_CB_FAR			0x60
#define ARM_SMMU_CB_FSYNR0		0x68
#define ARM_SMMU_CB_S1_TLBIVA		0x600
#define ARM_SMMU_CB_S1_TLBIASID		0x610
#define ARM_SMMU_CB_S1_TLBIVAL		0x620
#define ARM_SMMU_CB_S2_TLBIIPAS2	0x630
#define ARM_SMMU_CB_S2_TLBIIPAS2L	0x638
#define ARM_SMMU_CB_TLBSYNC		0x7f0
#define ARM_SMMU_CB_TLBSTATUS		0x7f4
#define TLBSTATUS_SACTIVE		(1 << 0)
#define ARM_SMMU_CB_ATS1PR		0x800
#define ARM_SMMU_CB_ATSR		0x8f0

#define SCTLR_S1_ASIDPNE		(1 << 12)
#define SCTLR_CFCFG			(1 << 7)
#define SCTLR_CFIE			(1 << 6)
#define SCTLR_CFRE			(1 << 5)
#define SCTLR_E				(1 << 4)
#define SCTLR_AFE			(1 << 2)
#define SCTLR_TRE			(1 << 1)
#define SCTLR_M				(1 << 0)

#define ARM_MMU500_ACTLR_CPRE		(1 << 1)

#define ARM_MMU500_ACR_CACHE_LOCK	(1 << 26)

#define ARM_SMMU_IMPL_DEF0(smmu) \
	((smmu)->base + (2 * (1 << (smmu)->pgshift)))
#define ARM_SMMU_IMPL_DEF1(smmu) \
	((smmu)->base + (6 * (1 << (smmu)->pgshift)))
#define CB_PAR_F			(1 << 0)

#define ATSR_ACTIVE			(1 << 0)

#define RESUME_RETRY			(0 << 0)
#define RESUME_TERMINATE		(1 << 0)

#define TTBCR2_SEP_SHIFT		15
#define TTBCR2_SEP_UPSTREAM		(0x7 << TTBCR2_SEP_SHIFT)

#define TTBRn_ASID_SHIFT		48

#define FSR_MULTI			(1 << 31)
#define FSR_SS				(1 << 30)
#define FSR_UUT				(1 << 8)
#define FSR_ASF				(1 << 7)
#define FSR_TLBLKF			(1 << 6)
#define FSR_TLBMCF			(1 << 5)
#define FSR_EF				(1 << 4)
#define FSR_PF				(1 << 3)
#define FSR_AFF				(1 << 2)
#define FSR_TF				(1 << 1)

#define FSR_IGN				(FSR_AFF | FSR_ASF | \
					 FSR_TLBMCF | FSR_TLBLKF)
#define FSR_FAULT			(FSR_MULTI | FSR_SS | FSR_UUT | \
					 FSR_EF | FSR_PF | FSR_TF | FSR_IGN)

#define FSYNR0_WNR			(1 << 4)

static int force_stage;
module_param(force_stage, int, S_IRUGO);
MODULE_PARM_DESC(force_stage,
	"Force SMMU mappings to be installed at a particular stage of translation. A value of '1' or '2' forces the corresponding stage. All other values are ignored (i.e. no stage is forced). Note that selecting a specific stage will disable support for nested translation.");
static bool disable_bypass;
module_param(disable_bypass, bool, S_IRUGO);
MODULE_PARM_DESC(disable_bypass,
	"Disable bypass streams such that incoming transactions from devices that are not attached to an iommu domain will report an abort back to the device and will not be allowed to pass through the SMMU.");

enum arm_smmu_arch_version {
	ARM_SMMU_V1,
	ARM_SMMU_V1_64K,
	ARM_SMMU_V2,
};

enum arm_smmu_implementation {
	GENERIC_SMMU,
	ARM_MMU500,
	CAVIUM_SMMUV2,
	QCOM_SMMUV2,
	QCOM_SMMUV500,
};

struct arm_smmu_device;
struct arm_smmu_arch_ops {
	int (*init)(struct arm_smmu_device *smmu);
	void (*device_reset)(struct arm_smmu_device *smmu);
	phys_addr_t (*iova_to_phys_hard)(struct iommu_domain *domain,
					 dma_addr_t iova);
	void (*iova_to_phys_fault)(struct iommu_domain *domain,
				dma_addr_t iova, phys_addr_t *phys1,
				phys_addr_t *phys_post_tlbiall);
};

struct arm_smmu_impl_def_reg {
	u32 offset;
	u32 value;
};

struct arm_smmu_s2cr {
	struct iommu_group		*group;
	int				count;
	enum arm_smmu_s2cr_type		type;
	enum arm_smmu_s2cr_privcfg	privcfg;
	u8				cbndx;
};

#define s2cr_init_val (struct arm_smmu_s2cr){				\
	.type = disable_bypass ? S2CR_TYPE_FAULT : S2CR_TYPE_BYPASS,	\
}

struct arm_smmu_smr {
	u16				mask;
	u16				id;
	bool				valid;
};

struct arm_smmu_master_cfg {
	struct arm_smmu_device		*smmu;
	int				num_streamids;
	u16				streamids[MAX_MASTER_STREAMIDS];
	s16				smendx[MAX_MASTER_STREAMIDS];
};
#define INVALID_SMENDX			-1
#define for_each_cfg_sme(cfg, i, idx) \
	for (i = 0; idx = cfg->smendx[i], i < cfg->num_streamids; ++i)

/*
 * Describes resources required for on/off power operation.
 * Separate reference count is provided for atomic/nonatomic
 * operations.
 */
struct arm_smmu_power_resources {
	struct platform_device		*pdev;
	struct device			*dev;

	struct clk			**clocks;
	int				num_clocks;

	struct regulator_bulk_data	*gdscs;
	int				num_gdscs;

	uint32_t			bus_client;
	struct msm_bus_scale_pdata	*bus_dt_data;

	/* Protects power_count */
	struct mutex			power_lock;
	int				power_count;

	/* Protects clock_refs_count */
	spinlock_t			clock_refs_lock;
	int				clock_refs_count;
};

struct arm_smmu_device {
	struct device			*dev;

	void __iomem			*base;
	unsigned long			size;
	unsigned long			pgshift;

#define ARM_SMMU_FEAT_COHERENT_WALK	(1 << 0)
#define ARM_SMMU_FEAT_STREAM_MATCH	(1 << 1)
#define ARM_SMMU_FEAT_TRANS_S1		(1 << 2)
#define ARM_SMMU_FEAT_TRANS_S2		(1 << 3)
#define ARM_SMMU_FEAT_TRANS_NESTED	(1 << 4)
#define ARM_SMMU_FEAT_TRANS_OPS		(1 << 5)
#define ARM_SMMU_FEAT_VMID16		(1 << 6)
#define ARM_SMMU_FEAT_FMT_AARCH64_4K	(1 << 7)
#define ARM_SMMU_FEAT_FMT_AARCH64_16K	(1 << 8)
#define ARM_SMMU_FEAT_FMT_AARCH64_64K	(1 << 9)
#define ARM_SMMU_FEAT_FMT_AARCH32_L	(1 << 10)
#define ARM_SMMU_FEAT_FMT_AARCH32_S	(1 << 11)
	u32				features;

#define ARM_SMMU_OPT_SECURE_CFG_ACCESS (1 << 0)
#define ARM_SMMU_OPT_FATAL_ASF		(1 << 1)
#define ARM_SMMU_OPT_SKIP_INIT		(1 << 2)
#define ARM_SMMU_OPT_DYNAMIC		(1 << 3)
	u32				options;
	enum arm_smmu_arch_version	version;
	enum arm_smmu_implementation	model;

	u32				num_context_banks;
	u32				num_s2_context_banks;
	DECLARE_BITMAP(context_map, ARM_SMMU_MAX_CBS);
	atomic_t			irptndx;

	u32				num_mapping_groups;
	u16				streamid_mask;
	u16				smr_mask_mask;
	struct arm_smmu_smr		*smrs;
	struct arm_smmu_s2cr		*s2crs;
	struct mutex			stream_map_mutex;

	unsigned long			va_size;
	unsigned long			ipa_size;
	unsigned long			pa_size;
	unsigned long			pgsize_bitmap;

	u32				num_global_irqs;
	u32				num_context_irqs;
	unsigned int			*irqs;

	u32				cavium_id_base; /* Specific to Cavium */
	/* Specific to QCOM */
	struct arm_smmu_impl_def_reg	*impl_def_attach_registers;
	unsigned int			num_impl_def_attach_registers;

	struct arm_smmu_power_resources *pwr;

	spinlock_t			atos_lock;

	/* protects idr */
	struct mutex			idr_mutex;
	struct idr			asid_idr;

	struct arm_smmu_arch_ops	*arch_ops;
	void				*archdata;
};

enum arm_smmu_context_fmt {
	ARM_SMMU_CTX_FMT_NONE,
	ARM_SMMU_CTX_FMT_AARCH64,
	ARM_SMMU_CTX_FMT_AARCH32_L,
	ARM_SMMU_CTX_FMT_AARCH32_S,
};

struct arm_smmu_cfg {
	u8				cbndx;
	u8				irptndx;
	u32				cbar;
	u32				procid;
	u16				asid;
	enum arm_smmu_context_fmt	fmt;
};
#define INVALID_IRPTNDX			0xff
#define INVALID_CBNDX			0xff
#define INVALID_ASID			0xffff
/*
 * In V7L and V8L with TTBCR2.AS == 0, ASID is 8 bits.
 * V8L 16 with TTBCR2.AS == 1 (16 bit ASID) isn't supported yet.
 */
#define MAX_ASID			0xff

#define ARM_SMMU_CB_ASID(smmu, cfg)		((cfg)->asid)
#define ARM_SMMU_CB_VMID(smmu, cfg) ((u16)(smmu)->cavium_id_base + (cfg)->cbndx + 1)

enum arm_smmu_domain_stage {
	ARM_SMMU_DOMAIN_S1 = 0,
	ARM_SMMU_DOMAIN_S2,
	ARM_SMMU_DOMAIN_NESTED,
};

struct arm_smmu_pte_info {
	void *virt_addr;
	size_t size;
	struct list_head entry;
};

struct arm_smmu_domain {
	struct arm_smmu_device		*smmu;
	struct io_pgtable_ops		*pgtbl_ops;
	struct io_pgtable_cfg		pgtbl_cfg;
	spinlock_t			pgtbl_lock;
	struct arm_smmu_cfg		cfg;
	enum arm_smmu_domain_stage	stage;
	struct mutex			init_mutex; /* Protects smmu pointer */
	u32 attributes;
	u32				secure_vmid;
	struct list_head		pte_info_list;
	struct list_head		unassign_list;
	struct mutex			assign_lock;
	struct list_head		secure_pool_list;
	struct iommu_domain		domain;
};

struct arm_smmu_option_prop {
	u32 opt;
	const char *prop;
};

static atomic_t cavium_smmu_context_count = ATOMIC_INIT(0);

static struct arm_smmu_option_prop arm_smmu_options[] = {
	{ ARM_SMMU_OPT_SECURE_CFG_ACCESS, "calxeda,smmu-secure-config-access" },
	{ ARM_SMMU_OPT_FATAL_ASF, "qcom,fatal-asf" },
	{ ARM_SMMU_OPT_SKIP_INIT, "qcom,skip-init" },
	{ ARM_SMMU_OPT_DYNAMIC, "qcom,dynamic" },
	{ 0, NULL},
};

static phys_addr_t arm_smmu_iova_to_phys(struct iommu_domain *domain,
					dma_addr_t iova);
static phys_addr_t arm_smmu_iova_to_phys_hard(struct iommu_domain *domain,
					      dma_addr_t iova);
static void arm_smmu_destroy_domain_context(struct iommu_domain *domain);

static int arm_smmu_prepare_pgtable(void *addr, void *cookie);
static void arm_smmu_unprepare_pgtable(void *cookie, void *addr, size_t size);
static int arm_smmu_assign_table(struct arm_smmu_domain *smmu_domain);
static void arm_smmu_unassign_table(struct arm_smmu_domain *smmu_domain);

static int arm_smmu_arch_init(struct arm_smmu_device *smmu);
static void arm_smmu_arch_device_reset(struct arm_smmu_device *smmu);

static struct arm_smmu_domain *to_smmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct arm_smmu_domain, domain);
}

static void parse_driver_options(struct arm_smmu_device *smmu)
{
	int i = 0;

	do {
		if (of_property_read_bool(smmu->dev->of_node,
						arm_smmu_options[i].prop)) {
			smmu->options |= arm_smmu_options[i].opt;
			dev_dbg(smmu->dev, "option %s\n",
				arm_smmu_options[i].prop);
		}
	} while (arm_smmu_options[++i].opt);
}

static bool is_dynamic_domain(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	return !!(smmu_domain->attributes & (1 << DOMAIN_ATTR_DYNAMIC));
}

static bool arm_smmu_is_domain_secure(struct arm_smmu_domain *smmu_domain)
{
	return (smmu_domain->secure_vmid != VMID_INVAL);
}

static void arm_smmu_secure_domain_lock(struct arm_smmu_domain *smmu_domain)
{
	if (arm_smmu_is_domain_secure(smmu_domain))
		mutex_lock(&smmu_domain->assign_lock);
}

static void arm_smmu_secure_domain_unlock(struct arm_smmu_domain *smmu_domain)
{
	if (arm_smmu_is_domain_secure(smmu_domain))
		mutex_unlock(&smmu_domain->assign_lock);
}

static struct device_node *dev_get_dev_node(struct device *dev)
{
	if (dev_is_pci(dev)) {
		struct pci_bus *bus = to_pci_dev(dev)->bus;

		while (!pci_is_root_bus(bus))
			bus = bus->parent;
		return of_node_get(bus->bridge->parent->of_node);
	}

	return of_node_get(dev->of_node);
}

static int __arm_smmu_get_pci_sid(struct pci_dev *pdev, u16 alias, void *data)
{
	*((__be32 *)data) = cpu_to_be32(alias);
	return 0; /* Continue walking */
}

static int __find_legacy_master_phandle(struct device *dev, void *data)
{
	struct of_phandle_iterator *it = *(void **)data;
	struct device_node *np = it->node;
	int err;

	of_for_each_phandle(it, err, dev->of_node, "mmu-masters",
			    "#stream-id-cells", 0)
		if (it->node == np) {
			*(void **)data = dev;
			return 1;
		}
	it->node = np;
	return err == -ENOENT ? 0 : err;
}

static struct platform_driver arm_smmu_driver;

static int arm_smmu_register_legacy_master(struct device *dev)
{
	struct arm_smmu_device *smmu;
	struct arm_smmu_master_cfg *cfg;
	struct device_node *np;
	struct of_phandle_iterator it;
	void *data = &it;
	__be32 pci_sid;
	int err = 0;

	memset(&it, sizeof(it), 0);
	np = dev_get_dev_node(dev);
	if (!np || !of_find_property(np, "#stream-id-cells", NULL)) {
		of_node_put(np);
		return -ENODEV;
	}

	it.node = np;
	err = driver_for_each_device(&arm_smmu_driver.driver, NULL, &data,
				     __find_legacy_master_phandle);
	of_node_put(np);
	if (err == 0)
		return -ENODEV;
	if (err < 0)
		return err;

	smmu = dev_get_drvdata(data);

	if (it.cur_count > MAX_MASTER_STREAMIDS) {
		dev_err(smmu->dev,
			"reached maximum number (%d) of stream IDs for master device %s\n",
			MAX_MASTER_STREAMIDS, dev_name(dev));
		return -ENOSPC;
	}
	if (dev_is_pci(dev)) {
		/* "mmu-masters" assumes Stream ID == Requester ID */
		pci_for_each_dma_alias(to_pci_dev(dev), __arm_smmu_get_pci_sid,
				       &pci_sid);
		it.cur = &pci_sid;
		it.cur_count = 1;
	}

	cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;

	cfg->smmu = smmu;
	dev->archdata.iommu = cfg;

	while (it.cur_count--)
		cfg->streamids[cfg->num_streamids++] = be32_to_cpup(it.cur++);

	return 0;
}

static int __arm_smmu_alloc_bitmap(unsigned long *map, int start, int end)
{
	int idx;

	do {
		idx = find_next_zero_bit(map, end, start);
		if (idx == end)
			return -ENOSPC;
	} while (test_and_set_bit(idx, map));

	return idx;
}

static void __arm_smmu_free_bitmap(unsigned long *map, int idx)
{
	clear_bit(idx, map);
}

static int arm_smmu_prepare_clocks(struct arm_smmu_power_resources *pwr)
{
	int i, ret = 0;

	for (i = 0; i < pwr->num_clocks; ++i) {
		ret = clk_prepare(pwr->clocks[i]);
		if (ret) {
			dev_err(pwr->dev, "Couldn't prepare clock #%d\n", i);
			while (i--)
				clk_unprepare(pwr->clocks[i]);
			break;
		}
	}
	return ret;
}

static void arm_smmu_unprepare_clocks(struct arm_smmu_power_resources *pwr)
{
	int i;

	for (i = pwr->num_clocks; i; --i)
		clk_unprepare(pwr->clocks[i - 1]);
}

static int arm_smmu_enable_clocks(struct arm_smmu_power_resources *pwr)
{
	int i, ret = 0;

	for (i = 0; i < pwr->num_clocks; ++i) {
		ret = clk_enable(pwr->clocks[i]);
		if (ret) {
			dev_err(pwr->dev, "Couldn't enable clock #%d\n", i);
			while (i--)
				clk_disable(pwr->clocks[i]);
			break;
		}
	}

	return ret;
}

static void arm_smmu_disable_clocks(struct arm_smmu_power_resources *pwr)
{
	int i;

	for (i = pwr->num_clocks; i; --i)
		clk_disable(pwr->clocks[i - 1]);
}

static int arm_smmu_request_bus(struct arm_smmu_power_resources *pwr)
{
	if (!pwr->bus_client)
		return 0;
	return msm_bus_scale_client_update_request(pwr->bus_client, 1);
}

static void arm_smmu_unrequest_bus(struct arm_smmu_power_resources *pwr)
{
	if (!pwr->bus_client)
		return;
	WARN_ON(msm_bus_scale_client_update_request(pwr->bus_client, 0));
}

/* Clocks must be prepared before this (arm_smmu_prepare_clocks) */
static int arm_smmu_power_on_atomic(struct arm_smmu_power_resources *pwr)
{
	int ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&pwr->clock_refs_lock, flags);
	if (pwr->clock_refs_count > 0) {
		pwr->clock_refs_count++;
		spin_unlock_irqrestore(&pwr->clock_refs_lock, flags);
		return 0;
	}

	ret = arm_smmu_enable_clocks(pwr);
	if (!ret)
		pwr->clock_refs_count = 1;

	spin_unlock_irqrestore(&pwr->clock_refs_lock, flags);
	return ret;
}

/* Clocks should be unprepared after this (arm_smmu_unprepare_clocks) */
static void arm_smmu_power_off_atomic(struct arm_smmu_power_resources *pwr)
{
	unsigned long flags;

	spin_lock_irqsave(&pwr->clock_refs_lock, flags);
	if (pwr->clock_refs_count == 0) {
		WARN(1, "%s: bad clock_ref_count\n", dev_name(pwr->dev));
		spin_unlock_irqrestore(&pwr->clock_refs_lock, flags);
		return;

	} else if (pwr->clock_refs_count > 1) {
		pwr->clock_refs_count--;
		spin_unlock_irqrestore(&pwr->clock_refs_lock, flags);
		return;
	}

	arm_smmu_disable_clocks(pwr);

	pwr->clock_refs_count = 0;
	spin_unlock_irqrestore(&pwr->clock_refs_lock, flags);
}

static int arm_smmu_power_on_slow(struct arm_smmu_power_resources *pwr)
{
	int ret;

	mutex_lock(&pwr->power_lock);
	if (pwr->power_count > 0) {
		pwr->power_count += 1;
		mutex_unlock(&pwr->power_lock);
		return 0;
	}

	ret = regulator_bulk_enable(pwr->num_gdscs, pwr->gdscs);
	if (ret)
		goto out_unlock;

	ret = arm_smmu_request_bus(pwr);
	if (ret)
		goto out_disable_regulators;

	ret = arm_smmu_prepare_clocks(pwr);
	if (ret)
		goto out_disable_bus;

	pwr->power_count = 1;
	mutex_unlock(&pwr->power_lock);
	return 0;

out_disable_bus:
	arm_smmu_unrequest_bus(pwr);
out_disable_regulators:
	regulator_bulk_disable(pwr->num_gdscs, pwr->gdscs);
out_unlock:
	mutex_unlock(&pwr->power_lock);
	return ret;
}

static void arm_smmu_power_off_slow(struct arm_smmu_power_resources *pwr)
{
	mutex_lock(&pwr->power_lock);
	if (pwr->power_count == 0) {
		WARN(1, "%s: Bad power count\n", dev_name(pwr->dev));
		mutex_unlock(&pwr->power_lock);
		return;

	} else if (pwr->power_count > 1) {
		pwr->power_count--;
		mutex_unlock(&pwr->power_lock);
		return;
	}

	arm_smmu_unprepare_clocks(pwr);
	arm_smmu_unrequest_bus(pwr);
	regulator_bulk_disable(pwr->num_gdscs, pwr->gdscs);

	mutex_unlock(&pwr->power_lock);
}

static int arm_smmu_power_on(struct arm_smmu_power_resources *pwr)
{
	int ret;

	ret = arm_smmu_power_on_slow(pwr);
	if (ret)
		return ret;

	ret = arm_smmu_power_on_atomic(pwr);
	if (ret)
		goto out_disable;

	return 0;

out_disable:
	arm_smmu_power_off_slow(pwr);
	return ret;
}

static void arm_smmu_power_off(struct arm_smmu_power_resources *pwr)
{
	arm_smmu_power_off_atomic(pwr);
	arm_smmu_power_off_slow(pwr);
}

/*
 * Must be used instead of arm_smmu_power_on if it may be called from
 * atomic context
 */
static int arm_smmu_domain_power_on(struct iommu_domain *domain,
				struct arm_smmu_device *smmu)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	int atomic_domain = smmu_domain->attributes & (1 << DOMAIN_ATTR_ATOMIC);

	if (atomic_domain)
		return arm_smmu_power_on_atomic(smmu->pwr);

	return arm_smmu_power_on(smmu->pwr);
}

/*
 * Must be used instead of arm_smmu_power_on if it may be called from
 * atomic context
 */
static void arm_smmu_domain_power_off(struct iommu_domain *domain,
				struct arm_smmu_device *smmu)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	int atomic_domain = smmu_domain->attributes & (1 << DOMAIN_ATTR_ATOMIC);

	if (atomic_domain) {
		arm_smmu_power_off_atomic(smmu->pwr);
		return;
	}

	arm_smmu_power_off(smmu->pwr);
}

/* Wait for any pending TLB invalidations to complete */
static void arm_smmu_tlb_sync_cb(struct arm_smmu_device *smmu,
				int cbndx)
{
	void __iomem *base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cbndx);
	u32 val;

	writel_relaxed(0, base + ARM_SMMU_CB_TLBSYNC);
	if (readl_poll_timeout_atomic(base + ARM_SMMU_CB_TLBSTATUS, val,
				      !(val & TLBSTATUS_SACTIVE),
				      0, TLB_LOOP_TIMEOUT))
		dev_err(smmu->dev, "TLBSYNC timeout!\n");
}

static void __arm_smmu_tlb_sync(struct arm_smmu_device *smmu)
{
	int count = 0;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);

	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_sTLBGSYNC);
	while (readl_relaxed(gr0_base + ARM_SMMU_GR0_sTLBGSTATUS)
	       & sTLBGSTATUS_GSACTIVE) {
		cpu_relax();
		if (++count == TLB_LOOP_TIMEOUT) {
			dev_err_ratelimited(smmu->dev,
			"TLB sync timed out -- SMMU may be deadlocked\n");
			return;
		}
		udelay(1);
	}
}

static void arm_smmu_tlb_sync(void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	arm_smmu_tlb_sync_cb(smmu_domain->smmu, smmu_domain->cfg.cbndx);
}

/* Must be called with clocks/regulators enabled */
static void arm_smmu_tlb_inv_context(void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	bool stage1 = cfg->cbar != CBAR_TYPE_S2_TRANS;
	void __iomem *base;

	if (stage1) {
		base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
		writel_relaxed(ARM_SMMU_CB_ASID(smmu, cfg),
			       base + ARM_SMMU_CB_S1_TLBIASID);
		arm_smmu_tlb_sync_cb(smmu, cfg->cbndx);
	} else {
		base = ARM_SMMU_GR0(smmu);
		writel_relaxed(ARM_SMMU_CB_VMID(smmu, cfg),
			       base + ARM_SMMU_GR0_TLBIVMID);
		__arm_smmu_tlb_sync(smmu);
	}
}

static void arm_smmu_tlb_inv_range_nosync(unsigned long iova, size_t size,
					  size_t granule, bool leaf, void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	bool stage1 = cfg->cbar != CBAR_TYPE_S2_TRANS;
	void __iomem *reg;

	if (stage1) {
		reg = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
		reg += leaf ? ARM_SMMU_CB_S1_TLBIVAL : ARM_SMMU_CB_S1_TLBIVA;

		if (cfg->fmt != ARM_SMMU_CTX_FMT_AARCH64) {
			iova &= ~12UL;
			iova |= ARM_SMMU_CB_ASID(smmu, cfg);
			do {
				writel_relaxed(iova, reg);
				iova += granule;
			} while (size -= granule);
		} else {
			iova >>= 12;
			iova |= (u64)ARM_SMMU_CB_ASID(smmu, cfg) << 48;
			do {
				writeq_relaxed(iova, reg);
				iova += granule >> 12;
			} while (size -= granule);
		}
	} else if (smmu->version == ARM_SMMU_V2) {
		reg = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
		reg += leaf ? ARM_SMMU_CB_S2_TLBIIPAS2L :
			      ARM_SMMU_CB_S2_TLBIIPAS2;
		iova >>= 12;
		do {
			smmu_write_atomic_lq(iova, reg);
			iova += granule >> 12;
		} while (size -= granule);
	} else {
		reg = ARM_SMMU_GR0(smmu) + ARM_SMMU_GR0_TLBIVMID;
		writel_relaxed(ARM_SMMU_CB_VMID(smmu, cfg), reg);
	}
}

struct arm_smmu_secure_pool_chunk {
	void *addr;
	size_t size;
	struct list_head list;
};

static void *arm_smmu_secure_pool_remove(struct arm_smmu_domain *smmu_domain,
					size_t size)
{
	struct arm_smmu_secure_pool_chunk *it;

	list_for_each_entry(it, &smmu_domain->secure_pool_list, list) {
		if (it->size == size) {
			void *addr = it->addr;

			list_del(&it->list);
			kfree(it);
			return addr;
		}
	}

	return NULL;
}

static int arm_smmu_secure_pool_add(struct arm_smmu_domain *smmu_domain,
				     void *addr, size_t size)
{
	struct arm_smmu_secure_pool_chunk *chunk;

	chunk = kmalloc(sizeof(*chunk), GFP_ATOMIC);
	if (!chunk)
		return -ENOMEM;

	chunk->addr = addr;
	chunk->size = size;
	memset(addr, 0, size);
	list_add(&chunk->list, &smmu_domain->secure_pool_list);

	return 0;
}

static void arm_smmu_secure_pool_destroy(struct arm_smmu_domain *smmu_domain)
{
	struct arm_smmu_secure_pool_chunk *it, *i;

	list_for_each_entry_safe(it, i, &smmu_domain->secure_pool_list, list) {
		arm_smmu_unprepare_pgtable(smmu_domain, it->addr, it->size);
		/* pages will be freed later (after being unassigned) */
		kfree(it);
	}
}

static void *arm_smmu_alloc_pages_exact(void *cookie,
					size_t size, gfp_t gfp_mask)
{
	int ret;
	void *page;
	struct arm_smmu_domain *smmu_domain = cookie;

	if (!arm_smmu_is_domain_secure(smmu_domain))
		return alloc_pages_exact(size, gfp_mask);

	page = arm_smmu_secure_pool_remove(smmu_domain, size);
	if (page)
		return page;

	page = alloc_pages_exact(size, gfp_mask);
	if (page) {
		ret = arm_smmu_prepare_pgtable(page, cookie);
		if (ret) {
			free_pages_exact(page, size);
			return NULL;
		}
	}

	return page;
}

static void arm_smmu_free_pages_exact(void *cookie, void *virt, size_t size)
{
	struct arm_smmu_domain *smmu_domain = cookie;

	if (!arm_smmu_is_domain_secure(smmu_domain)) {
		free_pages_exact(virt, size);
		return;
	}

	if (arm_smmu_secure_pool_add(smmu_domain, virt, size))
		arm_smmu_unprepare_pgtable(smmu_domain, virt, size);
}

static struct iommu_gather_ops arm_smmu_gather_ops = {
	.tlb_flush_all	= arm_smmu_tlb_inv_context,
	.tlb_add_flush	= arm_smmu_tlb_inv_range_nosync,
	.tlb_sync	= arm_smmu_tlb_sync,
	.alloc_pages_exact = arm_smmu_alloc_pages_exact,
	.free_pages_exact = arm_smmu_free_pages_exact,
};

static phys_addr_t arm_smmu_verify_fault(struct iommu_domain *domain,
					 dma_addr_t iova, u32 fsr)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu;
	phys_addr_t phys;
	phys_addr_t phys_post_tlbiall;

	smmu = smmu_domain->smmu;

	if (smmu->arch_ops && smmu->arch_ops->iova_to_phys_fault) {
		smmu->arch_ops->iova_to_phys_fault(domain, iova, &phys,
		&phys_post_tlbiall);
	} else {
		phys = arm_smmu_iova_to_phys_hard(domain, iova);
		arm_smmu_tlb_inv_context(smmu_domain);
		phys_post_tlbiall = arm_smmu_iova_to_phys_hard(domain, iova);
	}

	if (phys != phys_post_tlbiall) {
		dev_err(smmu->dev,
			"ATOS results differed across TLBIALL...\n"
			"Before: %pa After: %pa\n", &phys, &phys_post_tlbiall);
	}
	if (!phys_post_tlbiall) {
		dev_err(smmu->dev,
			"ATOS still failed. If the page tables look good (check the software table walk) then hardware might be misbehaving.\n");
	}

	return phys_post_tlbiall;
}

static irqreturn_t arm_smmu_context_fault(int irq, void *dev)
{
	int flags, ret, tmp;
	u32 fsr, fsynr, resume;
	unsigned long iova;
	struct iommu_domain *domain = dev;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *cb_base;
	void __iomem *gr1_base;
	bool fatal_asf = smmu->options & ARM_SMMU_OPT_FATAL_ASF;
	phys_addr_t phys_soft;
	u32 frsynra;
	bool non_fatal_fault = !!(smmu_domain->attributes &
					DOMAIN_ATTR_NON_FATAL_FAULTS);

	static DEFINE_RATELIMIT_STATE(_rs,
				      DEFAULT_RATELIMIT_INTERVAL,
				      DEFAULT_RATELIMIT_BURST);

	ret = arm_smmu_power_on(smmu->pwr);
	if (ret)
		return IRQ_NONE;

	gr1_base = ARM_SMMU_GR1(smmu);
	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
	fsr = readl_relaxed(cb_base + ARM_SMMU_CB_FSR);

	if (!(fsr & FSR_FAULT)) {
		ret = IRQ_NONE;
		goto out_power_off;
	}

	if (fatal_asf && (fsr & FSR_ASF)) {
		dev_err(smmu->dev,
			"Took an address size fault.  Refusing to recover.\n");
		BUG();
	}

	fsynr = readl_relaxed(cb_base + ARM_SMMU_CB_FSYNR0);
	flags = fsynr & FSYNR0_WNR ? IOMMU_FAULT_WRITE : IOMMU_FAULT_READ;
	if (fsr & FSR_TF)
		flags |= IOMMU_FAULT_TRANSLATION;
	if (fsr & FSR_PF)
		flags |= IOMMU_FAULT_PERMISSION;
	if (fsr & FSR_EF)
		flags |= IOMMU_FAULT_EXTERNAL;
	if (fsr & FSR_SS)
		flags |= IOMMU_FAULT_TRANSACTION_STALLED;

	iova = readq_relaxed(cb_base + ARM_SMMU_CB_FAR);
	phys_soft = arm_smmu_iova_to_phys(domain, iova);
	frsynra = readl_relaxed(gr1_base + ARM_SMMU_GR1_CBFRSYNRA(cfg->cbndx));
	frsynra &= CBFRSYNRA_SID_MASK;
	tmp = report_iommu_fault(domain, smmu->dev, iova, flags);
	if (!tmp || (tmp == -EBUSY)) {
		dev_dbg(smmu->dev,
			"Context fault handled by client: iova=0x%08lx, fsr=0x%x, fsynr=0x%x, cb=%d\n",
			iova, fsr, fsynr, cfg->cbndx);
		dev_dbg(smmu->dev,
			"soft iova-to-phys=%pa\n", &phys_soft);
		ret = IRQ_HANDLED;
		resume = RESUME_TERMINATE;
	} else {
		phys_addr_t phys_atos = arm_smmu_verify_fault(domain, iova,
							      fsr);
		if (__ratelimit(&_rs)) {
			dev_err(smmu->dev,
				"Unhandled context fault: iova=0x%08lx, fsr=0x%x, fsynr=0x%x, cb=%d\n",
				iova, fsr, fsynr, cfg->cbndx);
			dev_err(smmu->dev, "FAR    = %016lx\n",
				(unsigned long)iova);
			dev_err(smmu->dev,
				"FSR    = %08x [%s%s%s%s%s%s%s%s%s]\n",
				fsr,
				(fsr & 0x02) ? "TF " : "",
				(fsr & 0x04) ? "AFF " : "",
				(fsr & 0x08) ? "PF " : "",
				(fsr & 0x10) ? "EF " : "",
				(fsr & 0x20) ? "TLBMCF " : "",
				(fsr & 0x40) ? "TLBLKF " : "",
				(fsr & 0x80) ? "MHF " : "",
				(fsr & 0x40000000) ? "SS " : "",
				(fsr & 0x80000000) ? "MULTI " : "");
			dev_err(smmu->dev,
				"soft iova-to-phys=%pa\n", &phys_soft);
			if (!phys_soft)
				dev_err(smmu->dev,
					"SOFTWARE TABLE WALK FAILED! Looks like %s accessed an unmapped address!\n",
					dev_name(smmu->dev));
			dev_err(smmu->dev,
				"hard iova-to-phys (ATOS)=%pa\n", &phys_atos);
			dev_err(smmu->dev, "SID=0x%x\n", frsynra);
		}
		ret = IRQ_NONE;
		resume = RESUME_TERMINATE;
		if (!non_fatal_fault) {
			dev_err(smmu->dev,
				"Unhandled arm-smmu context fault!\n");
			BUG();
		}
	}

	/*
	 * If the client returns -EBUSY, do not clear FSR and do not RESUME
	 * if stalled. This is required to keep the IOMMU client stalled on
	 * the outstanding fault. This gives the client a chance to take any
	 * debug action and then terminate the stalled transaction.
	 * So, the sequence in case of stall on fault should be:
	 * 1) Do not clear FSR or write to RESUME here
	 * 2) Client takes any debug action
	 * 3) Client terminates the stalled transaction and resumes the IOMMU
	 * 4) Client clears FSR. The FSR should only be cleared after 3) and
	 *    not before so that the fault remains outstanding. This ensures
	 *    SCTLR.HUPCF has the desired effect if subsequent transactions also
	 *    need to be terminated.
	 */
	if (tmp != -EBUSY) {
		/* Clear the faulting FSR */
		writel_relaxed(fsr, cb_base + ARM_SMMU_CB_FSR);

		/*
		 * Barrier required to ensure that the FSR is cleared
		 * before resuming SMMU operation
		 */
		wmb();

		/* Retry or terminate any stalled transactions */
		if (fsr & FSR_SS)
			writel_relaxed(resume, cb_base + ARM_SMMU_CB_RESUME);
	}

out_power_off:
	arm_smmu_power_off(smmu->pwr);

	return ret;
}

static irqreturn_t arm_smmu_global_fault(int irq, void *dev)
{
	u32 gfsr, gfsynr0, gfsynr1, gfsynr2;
	struct arm_smmu_device *smmu = dev;
	void __iomem *gr0_base = ARM_SMMU_GR0_NS(smmu);

	if (arm_smmu_power_on(smmu->pwr))
		return IRQ_NONE;

	gfsr = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSR);
	gfsynr0 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR0);
	gfsynr1 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR1);
	gfsynr2 = readl_relaxed(gr0_base + ARM_SMMU_GR0_sGFSYNR2);

	if (!gfsr) {
		arm_smmu_power_off(smmu->pwr);
		return IRQ_NONE;
	}

	dev_err_ratelimited(smmu->dev,
		"Unexpected global fault, this could be serious\n");
	dev_err_ratelimited(smmu->dev,
		"\tGFSR 0x%08x, GFSYNR0 0x%08x, GFSYNR1 0x%08x, GFSYNR2 0x%08x\n",
		gfsr, gfsynr0, gfsynr1, gfsynr2);

	writel(gfsr, gr0_base + ARM_SMMU_GR0_sGFSR);
	arm_smmu_power_off(smmu->pwr);
	return IRQ_HANDLED;
}

static void arm_smmu_init_context_bank(struct arm_smmu_domain *smmu_domain,
				       struct io_pgtable_cfg *pgtbl_cfg)
{
	u32 reg, reg2;
	u64 reg64;
	bool stage1;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	void __iomem *cb_base, *gr1_base;

	gr1_base = ARM_SMMU_GR1(smmu);
	stage1 = cfg->cbar != CBAR_TYPE_S2_TRANS;
	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);

	if (smmu->version > ARM_SMMU_V1) {
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH64)
			reg = CBA2R_RW64_64BIT;
		else
			reg = CBA2R_RW64_32BIT;
		/* 16-bit VMIDs live in CBA2R */
		if (smmu->features & ARM_SMMU_FEAT_VMID16)
			reg |= ARM_SMMU_CB_VMID(smmu, cfg) << CBA2R_VMID_SHIFT;

		writel_relaxed(reg, gr1_base + ARM_SMMU_GR1_CBA2R(cfg->cbndx));
	}

	/* CBAR */
	reg = cfg->cbar;
	if (smmu->version < ARM_SMMU_V2)
		reg |= cfg->irptndx << CBAR_IRPTNDX_SHIFT;

	/*
	 * Use the weakest shareability/memory types, so they are
	 * overridden by the ttbcr/pte.
	 */
	if (stage1) {
		reg |= (CBAR_S1_BPSHCFG_NSH << CBAR_S1_BPSHCFG_SHIFT) |
			(CBAR_S1_MEMATTR_WB << CBAR_S1_MEMATTR_SHIFT);
	} else if (!(smmu->features & ARM_SMMU_FEAT_VMID16)) {
		/* 8-bit VMIDs live in CBAR */
		reg |= ARM_SMMU_CB_VMID(smmu, cfg) << CBAR_VMID_SHIFT;
	}
	writel_relaxed(reg, gr1_base + ARM_SMMU_GR1_CBAR(cfg->cbndx));

	/* TTBRs */
	if (stage1) {
		u16 asid = ARM_SMMU_CB_ASID(smmu, cfg);

		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_S) {
			reg = pgtbl_cfg->arm_v7s_cfg.ttbr[0];
			writel_relaxed(reg, cb_base + ARM_SMMU_CB_TTBR0);
			reg = pgtbl_cfg->arm_v7s_cfg.ttbr[1];
			writel_relaxed(reg, cb_base + ARM_SMMU_CB_TTBR1);
			writel_relaxed(asid, cb_base + ARM_SMMU_CB_CONTEXTIDR);
		} else {
			reg64 = pgtbl_cfg->arm_lpae_s1_cfg.ttbr[0];
			reg64 |= (u64)asid << TTBRn_ASID_SHIFT;
			writeq_relaxed(reg64, cb_base + ARM_SMMU_CB_TTBR0);
			reg64 = pgtbl_cfg->arm_lpae_s1_cfg.ttbr[1];
			reg64 |= (u64)asid << TTBRn_ASID_SHIFT;
			writeq_relaxed(reg64, cb_base + ARM_SMMU_CB_TTBR1);
		}
	} else {
		reg64 = pgtbl_cfg->arm_lpae_s2_cfg.vttbr;
		writeq_relaxed(reg64, cb_base + ARM_SMMU_CB_TTBR0);
	}

	/* TTBCR */
	if (stage1) {
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_S) {
			reg = pgtbl_cfg->arm_v7s_cfg.tcr;
			reg2 = 0;
		} else {
			reg = pgtbl_cfg->arm_lpae_s1_cfg.tcr;
			reg2 = pgtbl_cfg->arm_lpae_s1_cfg.tcr >> 32;
			reg2 |= TTBCR2_SEP_UPSTREAM;
		}
		if (smmu->version > ARM_SMMU_V1)
			writel_relaxed(reg2, cb_base + ARM_SMMU_CB_TTBCR2);
	} else {
		reg = pgtbl_cfg->arm_lpae_s2_cfg.vtcr;
	}
	writel_relaxed(reg, cb_base + ARM_SMMU_CB_TTBCR);

	/* MAIRs (stage-1 only) */
	if (stage1) {
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_S) {
			reg = pgtbl_cfg->arm_v7s_cfg.prrr;
			reg2 = pgtbl_cfg->arm_v7s_cfg.nmrr;
		} else {
			reg = pgtbl_cfg->arm_lpae_s1_cfg.mair[0];
			reg2 = pgtbl_cfg->arm_lpae_s1_cfg.mair[1];
		}
		writel_relaxed(reg, cb_base + ARM_SMMU_CB_S1_MAIR0);
		writel_relaxed(reg2, cb_base + ARM_SMMU_CB_S1_MAIR1);
	}

	/* SCTLR */
	reg = SCTLR_CFCFG | SCTLR_CFIE | SCTLR_CFRE | SCTLR_AFE | SCTLR_TRE;
	if (!(smmu_domain->attributes & (1 << DOMAIN_ATTR_S1_BYPASS)) ||
	    !stage1)
		reg |= SCTLR_M;
	if (stage1)
		reg |= SCTLR_S1_ASIDPNE;
#ifdef __BIG_ENDIAN
	reg |= SCTLR_E;
#endif
	writel_relaxed(reg, cb_base + ARM_SMMU_CB_SCTLR);
}

static int arm_smmu_init_asid(struct iommu_domain *domain,
				struct arm_smmu_device *smmu)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	bool dynamic = is_dynamic_domain(domain);
	int ret;

	if (!dynamic) {
		cfg->asid = cfg->cbndx + 1;
	} else {
		mutex_lock(&smmu->idr_mutex);
		ret = idr_alloc_cyclic(&smmu->asid_idr, domain,
				smmu->num_context_banks + 2,
				MAX_ASID + 1, GFP_KERNEL);

		mutex_unlock(&smmu->idr_mutex);
		if (ret < 0) {
			dev_err(smmu->dev, "dynamic ASID allocation failed: %d\n",
				ret);
			return ret;
		}
		cfg->asid = ret;
	}
	return 0;
}

static void arm_smmu_free_asid(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	bool dynamic = is_dynamic_domain(domain);

	if (cfg->asid == INVALID_ASID || !dynamic)
		return;

	mutex_lock(&smmu->idr_mutex);
	idr_remove(&smmu->asid_idr, cfg->asid);
	mutex_unlock(&smmu->idr_mutex);
}

static int arm_smmu_init_domain_context(struct iommu_domain *domain,
					struct arm_smmu_device *smmu)
{
	int irq, start, ret = 0;
	unsigned long ias, oas;
	struct io_pgtable_ops *pgtbl_ops;
	enum io_pgtable_fmt fmt;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	bool is_fast = smmu_domain->attributes & (1 << DOMAIN_ATTR_FAST);
	unsigned long quirks = 0;
	bool dynamic;

	mutex_lock(&smmu_domain->init_mutex);
	if (smmu_domain->smmu)
		goto out_unlock;

	smmu_domain->cfg.irptndx = INVALID_IRPTNDX;
	smmu_domain->cfg.asid = INVALID_ASID;

	/* We're bypassing these SIDs, so don't allocate an actual context */
	if (domain->type == IOMMU_DOMAIN_DMA) {
		smmu_domain->smmu = smmu;
		goto out_unlock;
	}

	dynamic = is_dynamic_domain(domain);
	if (dynamic && !(smmu->options & ARM_SMMU_OPT_DYNAMIC)) {
		dev_err(smmu->dev, "dynamic domains not supported\n");
		ret = -EPERM;
		goto out_unlock;
	}

	/*
	 * Mapping the requested stage onto what we support is surprisingly
	 * complicated, mainly because the spec allows S1+S2 SMMUs without
	 * support for nested translation. That means we end up with the
	 * following table:
	 *
	 * Requested        Supported        Actual
	 *     S1               N              S1
	 *     S1             S1+S2            S1
	 *     S1               S2             S2
	 *     S1               S1             S1
	 *     N                N              N
	 *     N              S1+S2            S2
	 *     N                S2             S2
	 *     N                S1             S1
	 *
	 * Note that you can't actually request stage-2 mappings.
	 */
	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S1))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S2;
	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S2))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S1;

	/*
	 * Choosing a suitable context format is even more fiddly. Until we
	 * grow some way for the caller to express a preference, and/or move
	 * the decision into the io-pgtable code where it arguably belongs,
	 * just aim for the closest thing to the rest of the system, and hope
	 * that the hardware isn't esoteric enough that we can't assume AArch64
	 * support to be a superset of AArch32 support...
	 */
	if (smmu->features & ARM_SMMU_FEAT_FMT_AARCH32_L)
		cfg->fmt = ARM_SMMU_CTX_FMT_AARCH32_L;
	if (IS_ENABLED(CONFIG_IOMMU_IO_PGTABLE_ARMV7S) &&
	    !IS_ENABLED(CONFIG_64BIT) && !IS_ENABLED(CONFIG_ARM_LPAE) &&
	    (smmu->features & ARM_SMMU_FEAT_FMT_AARCH32_S) &&
	    (smmu_domain->stage == ARM_SMMU_DOMAIN_S1))
		cfg->fmt = ARM_SMMU_CTX_FMT_AARCH32_S;
	if ((IS_ENABLED(CONFIG_64BIT) || cfg->fmt == ARM_SMMU_CTX_FMT_NONE) &&
	    (smmu->features & (ARM_SMMU_FEAT_FMT_AARCH64_64K |
			       ARM_SMMU_FEAT_FMT_AARCH64_16K |
			       ARM_SMMU_FEAT_FMT_AARCH64_4K)))
		cfg->fmt = ARM_SMMU_CTX_FMT_AARCH64;

	if (cfg->fmt == ARM_SMMU_CTX_FMT_NONE) {
		ret = -EINVAL;
		goto out_unlock;
	}

	switch (smmu_domain->stage) {
	case ARM_SMMU_DOMAIN_S1:
		cfg->cbar = CBAR_TYPE_S1_TRANS_S2_BYPASS;
		start = smmu->num_s2_context_banks;
		ias = smmu->va_size;
		oas = smmu->ipa_size;
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH64) {
			fmt = ARM_64_LPAE_S1;
		} else if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH32_L) {
			fmt = ARM_32_LPAE_S1;
			ias = min(ias, 32UL);
			oas = min(oas, 40UL);
		} else {
			fmt = ARM_V7S;
			ias = min(ias, 32UL);
			oas = min(oas, 32UL);
		}
		break;
	case ARM_SMMU_DOMAIN_NESTED:
		/*
		 * We will likely want to change this if/when KVM gets
		 * involved.
		 */
	case ARM_SMMU_DOMAIN_S2:
		cfg->cbar = CBAR_TYPE_S2_TRANS;
		start = 0;
		ias = smmu->ipa_size;
		oas = smmu->pa_size;
		if (cfg->fmt == ARM_SMMU_CTX_FMT_AARCH64) {
			fmt = ARM_64_LPAE_S2;
		} else {
			fmt = ARM_32_LPAE_S2;
			ias = min(ias, 40UL);
			oas = min(oas, 40UL);
		}
		break;
	default:
		ret = -EINVAL;
		goto out_unlock;
	}

	if (is_fast)
		fmt = ARM_V8L_FAST;

	if (smmu_domain->attributes & (1 << DOMAIN_ATTR_USE_UPSTREAM_HINT))
		quirks |= IO_PGTABLE_QUIRK_QCOM_USE_UPSTREAM_HINT;

	/* Dynamic domains must set cbndx through domain attribute */
	if (!dynamic) {
		ret = __arm_smmu_alloc_bitmap(smmu->context_map, start,
				      smmu->num_context_banks);
		if (ret < 0)
			goto out_unlock;
		cfg->cbndx = ret;
	}
	if (smmu->version < ARM_SMMU_V2) {
		cfg->irptndx = atomic_inc_return(&smmu->irptndx);
		cfg->irptndx %= smmu->num_context_irqs;
	} else {
		cfg->irptndx = cfg->cbndx;
	}

	smmu_domain->pgtbl_cfg = (struct io_pgtable_cfg) {
		.quirks		= quirks,
		.pgsize_bitmap	= smmu->pgsize_bitmap,
		.ias		= ias,
		.oas		= oas,
		.tlb		= &arm_smmu_gather_ops,
		.iommu_dev	= smmu->dev,
	};

	smmu_domain->smmu = smmu;
	pgtbl_ops = alloc_io_pgtable_ops(fmt, &smmu_domain->pgtbl_cfg,
					smmu_domain);
	if (!pgtbl_ops) {
		ret = -ENOMEM;
		goto out_clear_smmu;
	}

	/*
	 * assign any page table memory that might have been allocated
	 * during alloc_io_pgtable_ops
	 */
	arm_smmu_secure_domain_lock(smmu_domain);
	arm_smmu_assign_table(smmu_domain);
	arm_smmu_secure_domain_unlock(smmu_domain);

	/* Update the domain's page sizes to reflect the page table format */
	domain->pgsize_bitmap = smmu_domain->pgtbl_cfg.pgsize_bitmap;

	/* Assign an asid */
	ret = arm_smmu_init_asid(domain, smmu);
	if (ret)
		goto out_clear_smmu;

	if (!dynamic) {
		/* Initialise the context bank with our page table cfg */
		arm_smmu_init_context_bank(smmu_domain,
						&smmu_domain->pgtbl_cfg);

		/*
		 * Request context fault interrupt. Do this last to avoid the
		 * handler seeing a half-initialised domain state.
		 */
		irq = smmu->irqs[smmu->num_global_irqs + cfg->irptndx];
		ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
			arm_smmu_context_fault, IRQF_ONESHOT | IRQF_SHARED,
			"arm-smmu-context-fault", domain);
		if (ret < 0) {
			dev_err(smmu->dev, "failed to request context IRQ %d (%u)\n",
				cfg->irptndx, irq);
			cfg->irptndx = INVALID_IRPTNDX;
			goto out_clear_smmu;
		}
	} else {
		cfg->irptndx = INVALID_IRPTNDX;
	}
	mutex_unlock(&smmu_domain->init_mutex);

	/* Publish page table ops for map/unmap */
	smmu_domain->pgtbl_ops = pgtbl_ops;
	return 0;

out_clear_smmu:
	arm_smmu_destroy_domain_context(domain);
	smmu_domain->smmu = NULL;
out_unlock:
	mutex_unlock(&smmu_domain->init_mutex);
	return ret;
}

static void arm_smmu_domain_reinit(struct arm_smmu_domain *smmu_domain)
{
	smmu_domain->cfg.irptndx = INVALID_IRPTNDX;
	smmu_domain->cfg.cbndx = INVALID_CBNDX;
	smmu_domain->secure_vmid = VMID_INVAL;
}

static void arm_smmu_destroy_domain_context(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	void __iomem *cb_base;
	int irq;
	bool dynamic;
	int ret;

	if (!smmu || domain->type == IOMMU_DOMAIN_DMA)
		return;

	ret = arm_smmu_power_on(smmu->pwr);
	if (ret) {
		WARN_ONCE(ret, "Woops, powering on smmu %p failed. Leaking context bank\n",
				smmu);
		return;
	}

	dynamic = is_dynamic_domain(domain);
	if (dynamic) {
		arm_smmu_free_asid(domain);
		free_io_pgtable_ops(smmu_domain->pgtbl_ops);
		arm_smmu_power_off(smmu->pwr);
		arm_smmu_secure_domain_lock(smmu_domain);
		arm_smmu_secure_pool_destroy(smmu_domain);
		arm_smmu_unassign_table(smmu_domain);
		arm_smmu_secure_domain_unlock(smmu_domain);
		arm_smmu_domain_reinit(smmu_domain);
		return;
	}

	/*
	 * Disable the context bank and free the page tables before freeing
	 * it.
	 */
	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
	writel_relaxed(0, cb_base + ARM_SMMU_CB_SCTLR);

	if (cfg->irptndx != INVALID_IRPTNDX) {
		irq = smmu->irqs[smmu->num_global_irqs + cfg->irptndx];
		devm_free_irq(smmu->dev, irq, domain);
	}

	free_io_pgtable_ops(smmu_domain->pgtbl_ops);
	arm_smmu_secure_domain_lock(smmu_domain);
	arm_smmu_secure_pool_destroy(smmu_domain);
	arm_smmu_unassign_table(smmu_domain);
	arm_smmu_secure_domain_unlock(smmu_domain);
	__arm_smmu_free_bitmap(smmu->context_map, cfg->cbndx);

	arm_smmu_power_off(smmu->pwr);
	arm_smmu_domain_reinit(smmu_domain);
}

static struct iommu_domain *arm_smmu_domain_alloc(unsigned type)
{
	struct arm_smmu_domain *smmu_domain;

	/* Do not support DOMAIN_DMA for now */
	if (type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;
	/*
	 * Allocate the domain and initialise some of its data structures.
	 * We can't really do anything meaningful until we've added a
	 * master.
	 */
	smmu_domain = kzalloc(sizeof(*smmu_domain), GFP_KERNEL);
	if (!smmu_domain)
		return NULL;

	if (type == IOMMU_DOMAIN_DMA &&
	    iommu_get_dma_cookie(&smmu_domain->domain)) {
		kfree(smmu_domain);
		return NULL;
	}

	mutex_init(&smmu_domain->init_mutex);
	spin_lock_init(&smmu_domain->pgtbl_lock);
	INIT_LIST_HEAD(&smmu_domain->pte_info_list);
	INIT_LIST_HEAD(&smmu_domain->unassign_list);
	mutex_init(&smmu_domain->assign_lock);
	INIT_LIST_HEAD(&smmu_domain->secure_pool_list);
	arm_smmu_domain_reinit(smmu_domain);

	return &smmu_domain->domain;
}

static void arm_smmu_domain_free(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	/*
	 * Free the domain resources. We assume that all devices have
	 * already been detached.
	 */
	iommu_put_dma_cookie(domain);
	arm_smmu_destroy_domain_context(domain);
	kfree(smmu_domain);
}

static void arm_smmu_write_smr(struct arm_smmu_device *smmu, int idx)
{
	struct arm_smmu_smr *smr = smmu->smrs + idx;
	u32 reg = smr->id << SMR_ID_SHIFT | smr->mask << SMR_MASK_SHIFT;

	if (smr->valid)
		reg |= SMR_VALID;
	writel_relaxed(reg, ARM_SMMU_GR0(smmu) + ARM_SMMU_GR0_SMR(idx));
}

static void arm_smmu_write_s2cr(struct arm_smmu_device *smmu, int idx)
{
	struct arm_smmu_s2cr *s2cr = smmu->s2crs + idx;
	u32 reg = (s2cr->type & S2CR_TYPE_MASK) << S2CR_TYPE_SHIFT |
		  (s2cr->cbndx & S2CR_CBNDX_MASK) << S2CR_CBNDX_SHIFT |
		  (s2cr->privcfg & S2CR_PRIVCFG_MASK) << S2CR_PRIVCFG_SHIFT;

	writel_relaxed(reg, ARM_SMMU_GR0(smmu) + ARM_SMMU_GR0_S2CR(idx));
}

static void arm_smmu_write_sme(struct arm_smmu_device *smmu, int idx)
{
	arm_smmu_write_s2cr(smmu, idx);
	if (smmu->smrs)
		arm_smmu_write_smr(smmu, idx);
}

static int arm_smmu_find_sme(struct arm_smmu_device *smmu, u16 id, u16 mask)
{
	struct arm_smmu_smr *smrs = smmu->smrs;
	int i, free_idx = -ENOSPC;

	/* Stream indexing is blissfully easy */
	if (!smrs)
		return id;

	/* Validating SMRs is... less so */
	for (i = 0; i < smmu->num_mapping_groups; ++i) {
		if (!smrs[i].valid) {
			/*
			 * Note the first free entry we come across, which
			 * we'll claim in the end if nothing else matches.
			 */
			if (free_idx < 0)
				free_idx = i;
			continue;
		}
		/*
		 * If the new entry is _entirely_ matched by an existing entry,
		 * then reuse that, with the guarantee that there also cannot
		 * be any subsequent conflicting entries. In normal use we'd
		 * expect simply identical entries for this case, but there's
		 * no harm in accommodating the generalisation.
		 */
		if ((mask & smrs[i].mask) == mask &&
		    !((id ^ smrs[i].id) & ~smrs[i].mask))
			return i;
		/*
		 * If the new entry has any other overlap with an existing one,
		 * though, then there always exists at least one stream ID
		 * which would cause a conflict, and we can't allow that risk.
		 */
		if (!((id ^ smrs[i].id) & ~(smrs[i].mask | mask)))
			return -EINVAL;
	}

	return free_idx;
}

static bool arm_smmu_free_sme(struct arm_smmu_device *smmu, int idx)
{
	if (--smmu->s2crs[idx].count)
		return false;

	smmu->s2crs[idx] = s2cr_init_val;
	if (smmu->smrs)
		smmu->smrs[idx].valid = false;

	return true;
}

static int arm_smmu_master_alloc_smes(struct device *dev)
{
	struct arm_smmu_master_cfg *cfg = dev->archdata.iommu;
	struct arm_smmu_device *smmu = cfg->smmu;
	struct arm_smmu_smr *smrs = smmu->smrs;
	struct iommu_group *group;
	int i, idx, ret;

	mutex_lock(&smmu->stream_map_mutex);
	/* Figure out a viable stream map entry allocation */
	for_each_cfg_sme(cfg, i, idx) {
		if (idx != INVALID_SMENDX) {
			ret = -EEXIST;
			goto out_err;
		}

		ret = arm_smmu_find_sme(smmu, cfg->streamids[i], 0);
		if (ret < 0)
			goto out_err;

		idx = ret;
		if (smrs && smmu->s2crs[idx].count == 0) {
			smrs[idx].id = cfg->streamids[i];
			smrs[idx].mask = 0; /* We don't currently share SMRs */
			smrs[idx].valid = true;
		}
		smmu->s2crs[idx].count++;
		cfg->smendx[i] = (s16)idx;
	}

	group = iommu_group_get_for_dev(dev);
	if (!group)
		group = ERR_PTR(-ENOMEM);
	if (IS_ERR(group)) {
		ret = PTR_ERR(group);
		goto out_err;
	}
	iommu_group_put(group);

	/* It worked! Now, poke the actual hardware */
	for_each_cfg_sme(cfg, i, idx) {
		arm_smmu_write_sme(smmu, idx);
		smmu->s2crs[idx].group = group;
	}

	mutex_unlock(&smmu->stream_map_mutex);
	return 0;

out_err:
	while (i--) {
		arm_smmu_free_sme(smmu, cfg->smendx[i]);
		cfg->smendx[i] = INVALID_SMENDX;
	}
	mutex_unlock(&smmu->stream_map_mutex);
	return ret;
}

static void arm_smmu_master_free_smes(struct arm_smmu_master_cfg *cfg)
{
	struct arm_smmu_device *smmu = cfg->smmu;
	int i, idx;

	mutex_lock(&smmu->stream_map_mutex);
	for_each_cfg_sme(cfg, i, idx) {
		if (arm_smmu_free_sme(smmu, idx))
			arm_smmu_write_sme(smmu, idx);
		cfg->smendx[i] = INVALID_SMENDX;
	}
	mutex_unlock(&smmu->stream_map_mutex);
}

static int arm_smmu_domain_add_master(struct arm_smmu_domain *smmu_domain,
				      struct arm_smmu_master_cfg *cfg)
{
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_s2cr *s2cr = smmu->s2crs;
	enum arm_smmu_s2cr_type type = S2CR_TYPE_TRANS;
	u8 cbndx = smmu_domain->cfg.cbndx;
	int i, idx;

	/*
	 * FIXME: This won't be needed once we have IOMMU-backed DMA ops
	 * for all devices behind the SMMU. Note that we need to take
	 * care configuring SMRs for devices both a platform_device and
	 * and a PCI device (i.e. a PCI host controller)
	 */
	if (smmu_domain->domain.type == IOMMU_DOMAIN_DMA)
		type = S2CR_TYPE_BYPASS;

	for_each_cfg_sme(cfg, i, idx) {
		if (type == s2cr[idx].type && cbndx == s2cr[idx].cbndx)
			continue;

		s2cr[idx].type = type;
		s2cr[idx].privcfg = S2CR_PRIVCFG_DEFAULT;
		s2cr[idx].cbndx = cbndx;
		arm_smmu_write_s2cr(smmu, idx);
	}

	return 0;
}

static void arm_smmu_detach_dev(struct iommu_domain *domain,
				struct device *dev)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	int dynamic = smmu_domain->attributes & (1 << DOMAIN_ATTR_DYNAMIC);
	int atomic_domain = smmu_domain->attributes & (1 << DOMAIN_ATTR_ATOMIC);

	if (dynamic)
		return;

	if (!smmu) {
		dev_err(dev, "Domain not attached; cannot detach!\n");
		return;
	}

	dev->archdata.iommu = NULL;

	/* Remove additional vote for atomic power */
	if (atomic_domain) {
		WARN_ON(arm_smmu_power_on_atomic(smmu->pwr));
		arm_smmu_power_off(smmu->pwr);
	}
}

static int arm_smmu_assign_table(struct arm_smmu_domain *smmu_domain)
{
	int ret = 0;
	int dest_vmids[2] = {VMID_HLOS, smmu_domain->secure_vmid};
	int dest_perms[2] = {PERM_READ | PERM_WRITE, PERM_READ};
	int source_vmid = VMID_HLOS;
	struct arm_smmu_pte_info *pte_info, *temp;

	if (!arm_smmu_is_domain_secure(smmu_domain))
		return ret;

	list_for_each_entry(pte_info, &smmu_domain->pte_info_list, entry) {
		ret = hyp_assign_phys(virt_to_phys(pte_info->virt_addr),
				      PAGE_SIZE, &source_vmid, 1,
				      dest_vmids, dest_perms, 2);
		if (WARN_ON(ret))
			break;
	}

	list_for_each_entry_safe(pte_info, temp, &smmu_domain->pte_info_list,
								entry) {
		list_del(&pte_info->entry);
		kfree(pte_info);
	}
	return ret;
}

static void arm_smmu_unassign_table(struct arm_smmu_domain *smmu_domain)
{
	int ret;
	int dest_vmids = VMID_HLOS;
	int dest_perms = PERM_READ | PERM_WRITE | PERM_EXEC;
	int source_vmlist[2] = {VMID_HLOS, smmu_domain->secure_vmid};
	struct arm_smmu_pte_info *pte_info, *temp;

	if (!arm_smmu_is_domain_secure(smmu_domain))
		return;

	list_for_each_entry(pte_info, &smmu_domain->unassign_list, entry) {
		ret = hyp_assign_phys(virt_to_phys(pte_info->virt_addr),
				      PAGE_SIZE, source_vmlist, 2,
				      &dest_vmids, &dest_perms, 1);
		if (WARN_ON(ret))
			break;
		free_pages_exact(pte_info->virt_addr, pte_info->size);
	}

	list_for_each_entry_safe(pte_info, temp, &smmu_domain->unassign_list,
				 entry) {
		list_del(&pte_info->entry);
		kfree(pte_info);
	}
}

static void arm_smmu_unprepare_pgtable(void *cookie, void *addr, size_t size)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_pte_info *pte_info;

	BUG_ON(!arm_smmu_is_domain_secure(smmu_domain));

	pte_info = kzalloc(sizeof(struct arm_smmu_pte_info), GFP_ATOMIC);
	if (!pte_info)
		return;

	pte_info->virt_addr = addr;
	pte_info->size = size;
	list_add_tail(&pte_info->entry, &smmu_domain->unassign_list);
}

static int arm_smmu_prepare_pgtable(void *addr, void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_pte_info *pte_info;

	BUG_ON(!arm_smmu_is_domain_secure(smmu_domain));

	pte_info = kzalloc(sizeof(struct arm_smmu_pte_info), GFP_ATOMIC);
	if (!pte_info)
		return -ENOMEM;
	pte_info->virt_addr = addr;
	list_add_tail(&pte_info->entry, &smmu_domain->pte_info_list);
	return 0;
}

static int arm_smmu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	int ret;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_master_cfg *cfg = dev->archdata.iommu;
	struct arm_smmu_device *smmu;
	int atomic_domain = smmu_domain->attributes & (1 << DOMAIN_ATTR_ATOMIC);

	if (!cfg) {
		dev_err(dev, "cannot attach to SMMU, is it on the same bus?\n");
		return -ENXIO;
	}
	smmu = cfg->smmu;

	/* Enable Clocks and Power */
	ret = arm_smmu_power_on(smmu->pwr);
	if (ret)
		return ret;

	/* Ensure that the domain is finalised */
	ret = arm_smmu_init_domain_context(domain, cfg->smmu);
	if (ret < 0)
		goto out_power_off;

	/* Do not modify the SIDs, HW is still running */
	if (is_dynamic_domain(domain)) {
		ret = 0;
		goto out_power_off;
	}

	/*
	 * Sanity check the domain. We don't support domains across
	 * different SMMUs.
	 */
	if (smmu_domain->smmu != cfg->smmu) {
		dev_err(dev,
			"cannot attach to SMMU %s whilst already attached to domain on SMMU %s\n",
			dev_name(smmu_domain->smmu->dev),
				 dev_name(cfg->smmu->dev));
		ret = -EINVAL;
		goto out_power_off;
	}

	/* Looks ok, so add the device to the domain */
	ret = arm_smmu_domain_add_master(smmu_domain, cfg);

out_power_off:
	/*
	 * Keep an additional vote for non-atomic power until domain is
	 * detached
	 */
	if (!ret && atomic_domain) {
		WARN_ON(arm_smmu_power_on(smmu->pwr));
		arm_smmu_power_off_atomic(smmu->pwr);
	}

	arm_smmu_power_off(smmu->pwr);

	return ret;
}

static int arm_smmu_map(struct iommu_domain *domain, unsigned long iova,
			phys_addr_t paddr, size_t size, int prot)
{
	int ret;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops= smmu_domain->pgtbl_ops;

	if (!ops)
		return -ENODEV;

	arm_smmu_secure_domain_lock(smmu_domain);

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	ret = ops->map(ops, iova, paddr, size, prot);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	arm_smmu_assign_table(smmu_domain);
	arm_smmu_secure_domain_unlock(smmu_domain);

	return ret;
}

static size_t arm_smmu_unmap(struct iommu_domain *domain, unsigned long iova,
			     size_t size)
{
	size_t ret;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops= smmu_domain->pgtbl_ops;

	if (!ops)
		return 0;

	ret = arm_smmu_domain_power_on(domain, smmu_domain->smmu);
	if (ret)
		return ret;

	arm_smmu_secure_domain_lock(smmu_domain);

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	ret = ops->unmap(ops, iova, size);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	arm_smmu_domain_power_off(domain, smmu_domain->smmu);
	/*
	 * While splitting up block mappings, we might allocate page table
	 * memory during unmap, so the vmids needs to be assigned to the
	 * memory here as well.
	 */
	arm_smmu_assign_table(smmu_domain);
	/* Also unassign any pages that were free'd during unmap */
	arm_smmu_unassign_table(smmu_domain);
	arm_smmu_secure_domain_unlock(smmu_domain);
	return ret;
}

static size_t arm_smmu_map_sg(struct iommu_domain *domain, unsigned long iova,
			   struct scatterlist *sg, unsigned int nents, int prot)
{
	int ret;
	size_t size;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops = smmu_domain->pgtbl_ops;

	if (!ops)
		return -ENODEV;

	ret = arm_smmu_domain_power_on(domain, smmu_domain->smmu);
	if (ret)
		return ret;

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	ret = ops->map_sg(ops, iova, sg, nents, prot, &size);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	if (!ret)
		arm_smmu_unmap(domain, iova, size);

	arm_smmu_domain_power_off(domain, smmu_domain->smmu);
	arm_smmu_assign_table(smmu_domain);

	return ret;
}

static phys_addr_t __arm_smmu_iova_to_phys_hard(struct iommu_domain *domain,
					      dma_addr_t iova)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct io_pgtable_ops *ops= smmu_domain->pgtbl_ops;
	struct device *dev = smmu->dev;
	void __iomem *cb_base;
	u32 tmp;
	u64 phys;
	unsigned long va;

	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);

	/* ATS1 registers can only be written atomically */
	va = iova & ~0xfffUL;
	if (smmu->version == ARM_SMMU_V2)
		smmu_write_atomic_lq(va, cb_base + ARM_SMMU_CB_ATS1PR);
	else /* Register is only 32-bit in v1 */
		writel_relaxed(va, cb_base + ARM_SMMU_CB_ATS1PR);

	if (readl_poll_timeout_atomic(cb_base + ARM_SMMU_CB_ATSR, tmp,
				      !(tmp & ATSR_ACTIVE), 5, 50)) {
		phys = ops->iova_to_phys(ops, iova);
		dev_err(dev,
			"iova to phys timed out on %pad. software table walk result=%pa.\n",
			&iova, &phys);
		phys = 0;
		return phys;
	}

	phys = readq_relaxed(cb_base + ARM_SMMU_CB_PAR);
	if (phys & CB_PAR_F) {
		dev_err(dev, "translation fault!\n");
		dev_err(dev, "PAR = 0x%llx\n", phys);
		phys = 0;
	} else {
		phys = (phys & (PHYS_MASK & ~0xfffULL)) | (iova & 0xfff);
	}

	return phys;
}

static phys_addr_t arm_smmu_iova_to_phys(struct iommu_domain *domain,
					dma_addr_t iova)
{
	phys_addr_t ret;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops= smmu_domain->pgtbl_ops;

	if (!ops)
		return 0;

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	ret = ops->iova_to_phys(ops, iova);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	return ret;
}

/*
 * This function can sleep, and cannot be called from atomic context. Will
 * power on register block if required. This restriction does not apply to the
 * original iova_to_phys() op.
 */
static phys_addr_t arm_smmu_iova_to_phys_hard(struct iommu_domain *domain,
					dma_addr_t iova)
{
	phys_addr_t ret = 0;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	if (smmu_domain->smmu->arch_ops &&
	    smmu_domain->smmu->arch_ops->iova_to_phys_hard)
		return smmu_domain->smmu->arch_ops->iova_to_phys_hard(
						domain, iova);

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	if (smmu_domain->smmu->features & ARM_SMMU_FEAT_TRANS_OPS &&
			smmu_domain->stage == ARM_SMMU_DOMAIN_S1)
		ret = __arm_smmu_iova_to_phys_hard(domain, iova);

	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	return ret;
}

static bool arm_smmu_capable(enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		/*
		 * Return true here as the SMMU can always send out coherent
		 * requests.
		 */
		return true;
	case IOMMU_CAP_INTR_REMAP:
		return true; /* MSIs are just memory writes */
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}

static int arm_smmu_add_device(struct device *dev)
{
	struct arm_smmu_master_cfg *cfg;
	int i, ret;

	ret = arm_smmu_register_legacy_master(dev);
	cfg = dev->archdata.iommu;
	if (ret)
		goto out_free;

	ret = -EINVAL;
	for (i = 0; i < cfg->num_streamids; i++) {
		u16 sid = cfg->streamids[i];

		if (sid & ~cfg->smmu->streamid_mask) {
			dev_err(dev, "stream ID 0x%x out of range for SMMU (0x%x)\n",
				sid, cfg->smmu->streamid_mask);
			goto out_free;
		}
		cfg->smendx[i] = INVALID_SMENDX;
	}

	ret = arm_smmu_master_alloc_smes(dev);
	if (!ret)
		return ret;

out_free:
	kfree(cfg);
	dev->archdata.iommu = NULL;
	return ret;
}

static void arm_smmu_remove_device(struct device *dev)
{
	struct arm_smmu_master_cfg *cfg = dev->archdata.iommu;

	if (!cfg)
		return;

	arm_smmu_master_free_smes(cfg);
	iommu_group_remove_device(dev);
	kfree(cfg);
	dev->archdata.iommu = NULL;
}

static struct iommu_group *arm_smmu_device_group(struct device *dev)
{
	struct arm_smmu_master_cfg *cfg = dev->archdata.iommu;
	struct arm_smmu_device *smmu = cfg->smmu;
	struct iommu_group *group = NULL;
	int i, idx;

	for_each_cfg_sme(cfg, i, idx) {
		if (group && smmu->s2crs[idx].group &&
		    group != smmu->s2crs[idx].group)
			return ERR_PTR(-EINVAL);

		group = smmu->s2crs[idx].group;
	}

	if (group)
		return group;

	if (dev_is_pci(dev))
		group = pci_device_group(dev);
	else
		group = generic_device_group(dev);

	return group;
}

static int arm_smmu_domain_get_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	int ret = 0;

	switch (attr) {
	case DOMAIN_ATTR_NESTING:
		*(int *)data = (smmu_domain->stage == ARM_SMMU_DOMAIN_NESTED);
		return 0;
	case DOMAIN_ATTR_PT_BASE_ADDR:
		*((phys_addr_t *)data) =
			smmu_domain->pgtbl_cfg.arm_lpae_s1_cfg.ttbr[0];
		return 0;
	case DOMAIN_ATTR_CONTEXT_BANK:
		/* context bank index isn't valid until we are attached */
		if (smmu_domain->smmu == NULL)
			return -ENODEV;

		*((unsigned int *) data) = smmu_domain->cfg.cbndx;
		ret = 0;
		break;
	case DOMAIN_ATTR_TTBR0: {
		u64 val;
		struct arm_smmu_device *smmu = smmu_domain->smmu;
		/* not valid until we are attached */
		if (smmu == NULL)
			return -ENODEV;

		val = smmu_domain->pgtbl_cfg.arm_lpae_s1_cfg.ttbr[0];
		if (smmu_domain->cfg.cbar != CBAR_TYPE_S2_TRANS)
			val |= (u64)ARM_SMMU_CB_ASID(smmu, &smmu_domain->cfg)
					<< (TTBRn_ASID_SHIFT);
		*((u64 *)data) = val;
		ret = 0;
		break;
	}
	case DOMAIN_ATTR_CONTEXTIDR:
		/* not valid until attached */
		if (smmu_domain->smmu == NULL)
			return -ENODEV;
		*((u32 *)data) = smmu_domain->cfg.procid;
		ret = 0;
		break;
	case DOMAIN_ATTR_PROCID:
		*((u32 *)data) = smmu_domain->cfg.procid;
		ret = 0;
		break;
	case DOMAIN_ATTR_DYNAMIC:
		*((int *)data) = !!(smmu_domain->attributes
					& (1 << DOMAIN_ATTR_DYNAMIC));
		ret = 0;
		break;
	case DOMAIN_ATTR_NON_FATAL_FAULTS:
		*((int *)data) = !!(smmu_domain->attributes
				    & (1 << DOMAIN_ATTR_NON_FATAL_FAULTS));
		ret = 0;
		break;
	case DOMAIN_ATTR_S1_BYPASS:
		*((int *)data) = !!(smmu_domain->attributes
				    & (1 << DOMAIN_ATTR_S1_BYPASS));
		ret = 0;
		break;
	case DOMAIN_ATTR_SECURE_VMID:
		*((int *)data) = smmu_domain->secure_vmid;
		ret = 0;
		break;
	case DOMAIN_ATTR_PGTBL_INFO: {
		struct iommu_pgtbl_info *info = data;

		if (!(smmu_domain->attributes & (1 << DOMAIN_ATTR_FAST))) {
			ret = -ENODEV;
			break;
		}
		info->pmds = smmu_domain->pgtbl_cfg.av8l_fast_cfg.pmds;
		ret = 0;
		break;
	}
	case DOMAIN_ATTR_FAST:
		*((int *)data) = !!(smmu_domain->attributes
					& (1 << DOMAIN_ATTR_FAST));
		ret = 0;
		break;
	case DOMAIN_ATTR_USE_UPSTREAM_HINT:
		*((int *)data) = !!(smmu_domain->attributes &
				   (1 << DOMAIN_ATTR_USE_UPSTREAM_HINT));
		ret = 0;
		break;
	default:
		return -ENODEV;
	}
	return ret;
}

static int arm_smmu_domain_set_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	int ret = 0;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	mutex_lock(&smmu_domain->init_mutex);

	switch (attr) {
	case DOMAIN_ATTR_NESTING:
		if (smmu_domain->smmu) {
			ret = -EPERM;
			goto out_unlock;
		}

		if (*(int *)data)
			smmu_domain->stage = ARM_SMMU_DOMAIN_NESTED;
		else
			smmu_domain->stage = ARM_SMMU_DOMAIN_S1;

		break;
	case DOMAIN_ATTR_PROCID:
		if (smmu_domain->smmu != NULL) {
			dev_err(smmu_domain->smmu->dev,
			  "cannot change procid attribute while attached\n");
			ret = -EBUSY;
			break;
		}
		smmu_domain->cfg.procid = *((u32 *)data);
		ret = 0;
		break;
	case DOMAIN_ATTR_DYNAMIC: {
		int dynamic = *((int *)data);

		if (smmu_domain->smmu != NULL) {
			dev_err(smmu_domain->smmu->dev,
			  "cannot change dynamic attribute while attached\n");
			ret = -EBUSY;
			break;
		}

		if (dynamic)
			smmu_domain->attributes |= 1 << DOMAIN_ATTR_DYNAMIC;
		else
			smmu_domain->attributes &= ~(1 << DOMAIN_ATTR_DYNAMIC);
		ret = 0;
		break;
	}
	case DOMAIN_ATTR_CONTEXT_BANK:
		/* context bank can't be set while attached */
		if (smmu_domain->smmu != NULL) {
			ret = -EBUSY;
			break;
		}
		/* ... and it can only be set for dynamic contexts. */
		if (!(smmu_domain->attributes & (1 << DOMAIN_ATTR_DYNAMIC))) {
			ret = -EINVAL;
			break;
		}

		/* this will be validated during attach */
		smmu_domain->cfg.cbndx = *((unsigned int *)data);
		ret = 0;
		break;
	case DOMAIN_ATTR_NON_FATAL_FAULTS: {
		u32 non_fatal_faults = *((int *)data);

		if (non_fatal_faults)
			smmu_domain->attributes |=
					1 << DOMAIN_ATTR_NON_FATAL_FAULTS;
		else
			smmu_domain->attributes &=
					~(1 << DOMAIN_ATTR_NON_FATAL_FAULTS);
		ret = 0;
		break;
	}
	case DOMAIN_ATTR_S1_BYPASS: {
		int bypass = *((int *)data);

		/* bypass can't be changed while attached */
		if (smmu_domain->smmu != NULL) {
			ret = -EBUSY;
			break;
		}
		if (bypass)
			smmu_domain->attributes |= 1 << DOMAIN_ATTR_S1_BYPASS;
		else
			smmu_domain->attributes &=
					~(1 << DOMAIN_ATTR_S1_BYPASS);

		ret = 0;
		break;
	}
	case DOMAIN_ATTR_ATOMIC:
	{
		int atomic_ctx = *((int *)data);

		/* can't be changed while attached */
		if (smmu_domain->smmu != NULL) {
			ret = -EBUSY;
			break;
		}
		if (atomic_ctx)
			smmu_domain->attributes |= (1 << DOMAIN_ATTR_ATOMIC);
		else
			smmu_domain->attributes &= ~(1 << DOMAIN_ATTR_ATOMIC);
		break;
	}
	case DOMAIN_ATTR_SECURE_VMID:
		if (smmu_domain->secure_vmid != VMID_INVAL) {
			ret = -ENODEV;
			WARN(1, "secure vmid already set!");
			break;
		}
		smmu_domain->secure_vmid = *((int *)data);
		break;
	case DOMAIN_ATTR_FAST:
		if (*((int *)data))
			smmu_domain->attributes |= 1 << DOMAIN_ATTR_FAST;
		ret = 0;
		break;
	case DOMAIN_ATTR_USE_UPSTREAM_HINT:
		/* can't be changed while attached */
		if (smmu_domain->smmu != NULL) {
			ret = -EBUSY;
			break;
		}
		if (*((int *)data))
			smmu_domain->attributes |=
				1 << DOMAIN_ATTR_USE_UPSTREAM_HINT;
		ret = 0;
		break;
	default:
		ret = -ENODEV;
	}

out_unlock:
	mutex_unlock(&smmu_domain->init_mutex);
	return ret;
}

static void arm_smmu_trigger_fault(struct iommu_domain *domain,
					unsigned long flags)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu;
	void __iomem *cb_base;

	if (!smmu_domain->smmu) {
		pr_err("Can't trigger faults on non-attached domains\n");
		return;
	}

	smmu = smmu_domain->smmu;
	if (arm_smmu_power_on(smmu->pwr))
		return;

	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
	dev_err(smmu->dev, "Writing 0x%lx to FSRRESTORE on cb %d\n",
		flags, cfg->cbndx);
	writel_relaxed(flags, cb_base + ARM_SMMU_CB_FSRRESTORE);
	/* give the interrupt time to fire... */
	msleep(1000);

	arm_smmu_power_off(smmu->pwr);
}

static unsigned long arm_smmu_reg_read(struct iommu_domain *domain,
				       unsigned long offset)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	void __iomem *cb_base;
	unsigned long val;

	if (offset >= SZ_4K) {
		pr_err("Invalid offset: 0x%lx\n", offset);
		return 0;
	}

	smmu = smmu_domain->smmu;
	if (!smmu) {
		WARN(1, "Can't read registers of a detached domain\n");
		val = 0;
		return val;
	}

	if (arm_smmu_power_on(smmu->pwr))
		return 0;

	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
	val = readl_relaxed(cb_base + offset);

	arm_smmu_power_off(smmu->pwr);
	return val;
}

static void arm_smmu_reg_write(struct iommu_domain *domain,
			       unsigned long offset, unsigned long val)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu;
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	void __iomem *cb_base;

	if (offset >= SZ_4K) {
		pr_err("Invalid offset: 0x%lx\n", offset);
		return;
	}

	smmu = smmu_domain->smmu;
	if (!smmu) {
		WARN(1, "Can't read registers of a detached domain\n");
		return;
	}

	if (arm_smmu_power_on(smmu->pwr))
		return;

	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);
	writel_relaxed(val, cb_base + offset);

	arm_smmu_power_off(smmu->pwr);
}

static void arm_smmu_tlbi_domain(struct iommu_domain *domain)
{
	arm_smmu_tlb_inv_context(to_smmu_domain(domain));
}

static int arm_smmu_enable_config_clocks(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	return arm_smmu_power_on(smmu_domain->smmu->pwr);
}

static void arm_smmu_disable_config_clocks(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	arm_smmu_power_off(smmu_domain->smmu->pwr);
}

static struct iommu_ops arm_smmu_ops = {
	.capable		= arm_smmu_capable,
	.domain_alloc		= arm_smmu_domain_alloc,
	.domain_free		= arm_smmu_domain_free,
	.attach_dev		= arm_smmu_attach_dev,
	.detach_dev		= arm_smmu_detach_dev,
	.map			= arm_smmu_map,
	.unmap			= arm_smmu_unmap,
	.map_sg			= arm_smmu_map_sg,
	.iova_to_phys		= arm_smmu_iova_to_phys,
	.iova_to_phys_hard	= arm_smmu_iova_to_phys_hard,
	.add_device		= arm_smmu_add_device,
	.remove_device		= arm_smmu_remove_device,
	.device_group		= arm_smmu_device_group,
	.domain_get_attr	= arm_smmu_domain_get_attr,
	.domain_set_attr	= arm_smmu_domain_set_attr,
	.pgsize_bitmap		= -1UL, /* Restricted during device attach */
	.trigger_fault		= arm_smmu_trigger_fault,
	.reg_read		= arm_smmu_reg_read,
	.reg_write		= arm_smmu_reg_write,
	.tlbi_domain		= arm_smmu_tlbi_domain,
	.enable_config_clocks	= arm_smmu_enable_config_clocks,
	.disable_config_clocks	= arm_smmu_disable_config_clocks,
};

#define IMPL_DEF1_MICRO_MMU_CTRL	0
#define MICRO_MMU_CTRL_LOCAL_HALT_REQ	(1 << 2)
#define MICRO_MMU_CTRL_IDLE		(1 << 3)

/* Definitions for implementation-defined registers */
#define ACTLR_QCOM_OSH_SHIFT		28
#define ACTLR_QCOM_OSH			1

#define ACTLR_QCOM_ISH_SHIFT		29
#define ACTLR_QCOM_ISH			1

#define ACTLR_QCOM_NSH_SHIFT		30
#define ACTLR_QCOM_NSH			1

static int qsmmuv2_wait_for_halt(struct arm_smmu_device *smmu)
{
	void __iomem *impl_def1_base = ARM_SMMU_IMPL_DEF1(smmu);
	u32 tmp;

	if (readl_poll_timeout_atomic(impl_def1_base + IMPL_DEF1_MICRO_MMU_CTRL,
					tmp, (tmp & MICRO_MMU_CTRL_IDLE),
					0, 30000)) {
		dev_err(smmu->dev, "Couldn't halt SMMU!\n");
		return -EBUSY;
	}

	return 0;
}

static int __qsmmuv2_halt(struct arm_smmu_device *smmu, bool wait)
{
	void __iomem *impl_def1_base = ARM_SMMU_IMPL_DEF1(smmu);
	u32 reg;

	reg = readl_relaxed(impl_def1_base + IMPL_DEF1_MICRO_MMU_CTRL);
	reg |= MICRO_MMU_CTRL_LOCAL_HALT_REQ;
	writel_relaxed(reg, impl_def1_base + IMPL_DEF1_MICRO_MMU_CTRL);

	return wait ? qsmmuv2_wait_for_halt(smmu) : 0;
}

static int qsmmuv2_halt(struct arm_smmu_device *smmu)
{
	return __qsmmuv2_halt(smmu, true);
}

static int qsmmuv2_halt_nowait(struct arm_smmu_device *smmu)
{
	return __qsmmuv2_halt(smmu, false);
}

static void qsmmuv2_resume(struct arm_smmu_device *smmu)
{
	void __iomem *impl_def1_base = ARM_SMMU_IMPL_DEF1(smmu);
	u32 reg;

	reg = readl_relaxed(impl_def1_base + IMPL_DEF1_MICRO_MMU_CTRL);
	reg &= ~MICRO_MMU_CTRL_LOCAL_HALT_REQ;
	writel_relaxed(reg, impl_def1_base + IMPL_DEF1_MICRO_MMU_CTRL);
}

static void qsmmuv2_device_reset(struct arm_smmu_device *smmu)
{
	int i;
	u32 val;
	struct arm_smmu_impl_def_reg *regs = smmu->impl_def_attach_registers;
	void __iomem *cb_base;

	/*
	 * SCTLR.M must be disabled here per ARM SMMUv2 spec
	 * to prevent table walks with an inconsistent state.
	 */
	for (i = 0; i < smmu->num_context_banks; ++i) {
		cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, i);
		val = ACTLR_QCOM_ISH << ACTLR_QCOM_ISH_SHIFT |
		ACTLR_QCOM_OSH << ACTLR_QCOM_OSH_SHIFT |
		ACTLR_QCOM_NSH << ACTLR_QCOM_NSH_SHIFT;
		writel_relaxed(val, cb_base + ARM_SMMU_CB_ACTLR);
	}

	/* Program implementation defined registers */
	qsmmuv2_halt(smmu);
	for (i = 0; i < smmu->num_impl_def_attach_registers; ++i)
		writel_relaxed(regs[i].value,
			ARM_SMMU_GR0(smmu) + regs[i].offset);
	qsmmuv2_resume(smmu);
}

static phys_addr_t __qsmmuv2_iova_to_phys_hard(struct iommu_domain *domain,
					      dma_addr_t iova, bool halt)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	int ret;
	phys_addr_t phys = 0;
	unsigned long flags;

	ret = arm_smmu_power_on(smmu_domain->smmu->pwr);
	if (ret)
		return 0;

	if (halt) {
		ret = qsmmuv2_halt(smmu);
		if (ret)
			goto out_power_off;
	}

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	spin_lock(&smmu->atos_lock);
	phys = __arm_smmu_iova_to_phys_hard(domain, iova);
	spin_unlock(&smmu->atos_lock);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	if (halt)
		qsmmuv2_resume(smmu);

out_power_off:
	arm_smmu_power_off(smmu_domain->smmu->pwr);
	return phys;
}

static phys_addr_t qsmmuv2_iova_to_phys_hard(struct iommu_domain *domain,
					      dma_addr_t iova)
{
	return __qsmmuv2_iova_to_phys_hard(domain, iova, true);
}

static void qsmmuv2_iova_to_phys_fault(
				struct iommu_domain *domain,
				dma_addr_t iova, phys_addr_t *phys,
				phys_addr_t *phys_post_tlbiall)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_cfg *cfg = &smmu_domain->cfg;
	struct arm_smmu_device *smmu;
	void __iomem *cb_base;
	u64 sctlr, sctlr_orig;
	u32 fsr;

	smmu = smmu_domain->smmu;
	cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, cfg->cbndx);

	qsmmuv2_halt_nowait(smmu);

	writel_relaxed(RESUME_TERMINATE, cb_base + ARM_SMMU_CB_RESUME);

	qsmmuv2_wait_for_halt(smmu);

	/* clear FSR to allow ATOS to log any faults */
	fsr = readl_relaxed(cb_base + ARM_SMMU_CB_FSR);
	writel_relaxed(fsr, cb_base + ARM_SMMU_CB_FSR);

	/* disable stall mode momentarily */
	sctlr_orig = readl_relaxed(cb_base + ARM_SMMU_CB_SCTLR);
	sctlr = sctlr_orig & ~SCTLR_CFCFG;
	writel_relaxed(sctlr, cb_base + ARM_SMMU_CB_SCTLR);

	*phys = __qsmmuv2_iova_to_phys_hard(domain, iova, false);
	arm_smmu_tlb_inv_context(smmu_domain);
	*phys_post_tlbiall = __qsmmuv2_iova_to_phys_hard(domain, iova, false);

	/* restore SCTLR */
	writel_relaxed(sctlr_orig, cb_base + ARM_SMMU_CB_SCTLR);

	qsmmuv2_resume(smmu);
}

struct arm_smmu_arch_ops qsmmuv2_arch_ops = {
	.device_reset = qsmmuv2_device_reset,
	.iova_to_phys_hard = qsmmuv2_iova_to_phys_hard,
	.iova_to_phys_fault = qsmmuv2_iova_to_phys_fault,
};

static void arm_smmu_context_bank_reset(struct arm_smmu_device *smmu)
{
	int i;
	u32 reg, major;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	void __iomem *cb_base;

	/*
	 * Before clearing ARM_MMU500_ACTLR_CPRE, need to
	 * clear CACHE_LOCK bit of ACR first. And, CACHE_LOCK
	 * bit is only present in MMU-500r2 onwards.
	 */
	reg = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID7);
	major = (reg >> ID7_MAJOR_SHIFT) & ID7_MAJOR_MASK;
	if ((smmu->model == ARM_MMU500) && (major >= 2)) {
		reg = readl_relaxed(gr0_base + ARM_SMMU_GR0_sACR);
		reg &= ~ARM_MMU500_ACR_CACHE_LOCK;
		writel_relaxed(reg, gr0_base + ARM_SMMU_GR0_sACR);
	}

	/* Make sure all context banks are disabled and clear CB_FSR  */
	for (i = 0; i < smmu->num_context_banks; ++i) {
		cb_base = ARM_SMMU_CB_BASE(smmu) + ARM_SMMU_CB(smmu, i);
		writel_relaxed(0, cb_base + ARM_SMMU_CB_SCTLR);
		writel_relaxed(FSR_FAULT, cb_base + ARM_SMMU_CB_FSR);
		/*
		 * Disable MMU-500's not-particularly-beneficial next-page
		 * prefetcher for the sake of errata #841119 and #826419.
		 */
		if (smmu->model == ARM_MMU500) {
			reg = readl_relaxed(cb_base + ARM_SMMU_CB_ACTLR);
			reg &= ~ARM_MMU500_ACTLR_CPRE;
			writel_relaxed(reg, cb_base + ARM_SMMU_CB_ACTLR);
		}
	}
}

static void arm_smmu_device_reset(struct arm_smmu_device *smmu)
{
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	int i;
	u32 reg;

	/* clear global FSR */
	reg = readl_relaxed(ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sGFSR);
	writel_relaxed(reg, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sGFSR);

	/*
	 * Reset stream mapping groups: Initial values mark all SMRn as
	 * invalid and all S2CRn as bypass unless overridden.
	 */
	if (!(smmu->options & ARM_SMMU_OPT_SKIP_INIT)) {
		for (i = 0; i < smmu->num_mapping_groups; ++i)
			arm_smmu_write_sme(smmu, i);

		arm_smmu_context_bank_reset(smmu);
	}

	/* Invalidate the TLB, just in case */
	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_TLBIALLH);
	writel_relaxed(0, gr0_base + ARM_SMMU_GR0_TLBIALLNSNH);

	reg = readl_relaxed(ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);

	/* Enable fault reporting */
	reg |= (sCR0_GFRE | sCR0_GFIE | sCR0_GCFGFRE | sCR0_GCFGFIE);

	/* Disable TLB broadcasting. */
	reg |= (sCR0_VMIDPNE | sCR0_PTM);

	/* Enable client access, handling unmatched streams as appropriate */
	reg &= ~sCR0_CLIENTPD;
	if (disable_bypass)
		reg |= sCR0_USFCFG;
	else
		reg &= ~sCR0_USFCFG;

	/* Disable forced broadcasting */
	reg &= ~sCR0_FB;

	/* Don't upgrade barriers */
	reg &= ~(sCR0_BSU_MASK << sCR0_BSU_SHIFT);

	if (smmu->features & ARM_SMMU_FEAT_VMID16)
		reg |= sCR0_VMID16EN;

	/* Push the button */
	__arm_smmu_tlb_sync(smmu);
	writel(reg, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);

	/* Manage any implementation defined features */
	arm_smmu_arch_device_reset(smmu);
}

static int arm_smmu_id_size_to_bits(int size)
{
	switch (size) {
	case 0:
		return 32;
	case 1:
		return 36;
	case 2:
		return 40;
	case 3:
		return 42;
	case 4:
		return 44;
	case 5:
	default:
		return 48;
	}
}

static int arm_smmu_parse_impl_def_registers(struct arm_smmu_device *smmu)
{
	struct device *dev = smmu->dev;
	int i, ntuples, ret;
	u32 *tuples;
	struct arm_smmu_impl_def_reg *regs, *regit;

	if (!of_find_property(dev->of_node, "attach-impl-defs", &ntuples))
		return 0;

	ntuples /= sizeof(u32);
	if (ntuples % 2) {
		dev_err(dev,
			"Invalid number of attach-impl-defs registers: %d\n",
			ntuples);
		return -EINVAL;
	}

	regs = devm_kmalloc(
		dev, sizeof(*smmu->impl_def_attach_registers) * ntuples,
		GFP_KERNEL);
	if (!regs)
		return -ENOMEM;

	tuples = devm_kmalloc(dev, sizeof(u32) * ntuples * 2, GFP_KERNEL);
	if (!tuples)
		return -ENOMEM;

	ret = of_property_read_u32_array(dev->of_node, "attach-impl-defs",
					tuples, ntuples);
	if (ret)
		return ret;

	for (i = 0, regit = regs; i < ntuples; i += 2, ++regit) {
		regit->offset = tuples[i];
		regit->value = tuples[i + 1];
	}

	devm_kfree(dev, tuples);

	smmu->impl_def_attach_registers = regs;
	smmu->num_impl_def_attach_registers = ntuples / 2;

	return 0;
}


static int arm_smmu_init_clocks(struct arm_smmu_power_resources *pwr)
{
	const char *cname;
	struct property *prop;
	int i;
	struct device *dev = pwr->dev;

	pwr->num_clocks =
		of_property_count_strings(dev->of_node, "clock-names");

	if (pwr->num_clocks < 1) {
		pwr->num_clocks = 0;
		return 0;
	}

	pwr->clocks = devm_kzalloc(
		dev, sizeof(*pwr->clocks) * pwr->num_clocks,
		GFP_KERNEL);

	if (!pwr->clocks)
		return -ENOMEM;

	i = 0;
	of_property_for_each_string(dev->of_node, "clock-names",
				prop, cname) {
		struct clk *c = devm_clk_get(dev, cname);

		if (IS_ERR(c)) {
			dev_err(dev, "Couldn't get clock: %s",
				cname);
			return PTR_ERR(c);
		}

		if (clk_get_rate(c) == 0) {
			long rate = clk_round_rate(c, 1000);

			clk_set_rate(c, rate);
		}

		pwr->clocks[i] = c;

		++i;
	}
	return 0;
}

static int arm_smmu_init_regulators(struct arm_smmu_power_resources *pwr)
{
	const char *cname;
	struct property *prop;
	int i, ret = 0;
	struct device *dev = pwr->dev;

	pwr->num_gdscs =
		of_property_count_strings(dev->of_node, "qcom,regulator-names");

	if (pwr->num_gdscs < 1) {
		pwr->num_gdscs = 0;
		return 0;
	}

	pwr->gdscs = devm_kzalloc(
			dev, sizeof(*pwr->gdscs) * pwr->num_gdscs, GFP_KERNEL);

	if (!pwr->gdscs)
		return -ENOMEM;

	i = 0;
	of_property_for_each_string(dev->of_node, "qcom,regulator-names",
				prop, cname)
		pwr->gdscs[i].supply = cname;

	ret = devm_regulator_bulk_get(dev, pwr->num_gdscs, pwr->gdscs);
	return ret;
}

static int arm_smmu_init_bus_scaling(struct arm_smmu_power_resources *pwr)
{
	struct device *dev = pwr->dev;

	/* We don't want the bus APIs to print an error message */
	if (!of_find_property(dev->of_node, "qcom,msm-bus,name", NULL)) {
		dev_dbg(dev, "No bus scaling info\n");
		return 0;
	}

	pwr->bus_dt_data = msm_bus_cl_get_pdata(pwr->pdev);
	if (!pwr->bus_dt_data) {
		dev_err(dev, "Unable to read bus-scaling from devicetree\n");
		return -EINVAL;
	}

	pwr->bus_client = msm_bus_scale_register_client(pwr->bus_dt_data);
	if (!pwr->bus_client) {
		dev_err(dev, "Bus client registration failed\n");
		msm_bus_cl_clear_pdata(pwr->bus_dt_data);
		return -EINVAL;
	}

	return 0;
}

/*
 * Cleanup done by devm. Any non-devm resources must clean up themselves.
 */
static struct arm_smmu_power_resources *arm_smmu_init_power_resources(
						struct platform_device *pdev)
{
	struct arm_smmu_power_resources *pwr;
	int ret;

	pwr = devm_kzalloc(&pdev->dev, sizeof(*pwr), GFP_KERNEL);
	if (!pwr)
		return ERR_PTR(-ENOMEM);

	pwr->dev = &pdev->dev;
	pwr->pdev = pdev;
	mutex_init(&pwr->power_lock);
	spin_lock_init(&pwr->clock_refs_lock);

	ret = arm_smmu_init_clocks(pwr);
	if (ret)
		return ERR_PTR(ret);

	ret = arm_smmu_init_regulators(pwr);
	if (ret)
		return ERR_PTR(ret);

	ret = arm_smmu_init_bus_scaling(pwr);
	if (ret)
		return ERR_PTR(ret);

	return pwr;
}

/*
 * Bus APIs are not devm-safe.
 */
static void arm_smmu_exit_power_resources(struct arm_smmu_power_resources *pwr)
{
	msm_bus_scale_unregister_client(pwr->bus_client);
	msm_bus_cl_clear_pdata(pwr->bus_dt_data);
}

static int arm_smmu_device_cfg_probe(struct arm_smmu_device *smmu)
{
	unsigned long size;
	void __iomem *gr0_base = ARM_SMMU_GR0(smmu);
	u32 id;
	bool cttw_dt, cttw_reg;
	int i;

	dev_dbg(smmu->dev, "probing hardware configuration...\n");
	dev_dbg(smmu->dev, "SMMUv%d with:\n",
			smmu->version == ARM_SMMU_V2 ? 2 : 1);

	/* ID0 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID0);

	/* Restrict available stages based on module parameter */
	if (force_stage == 1)
		id &= ~(ID0_S2TS | ID0_NTS);
	else if (force_stage == 2)
		id &= ~(ID0_S1TS | ID0_NTS);

	if (id & ID0_S1TS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_S1;
		dev_dbg(smmu->dev, "\tstage 1 translation\n");
	}

	if (id & ID0_S2TS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_S2;
		dev_dbg(smmu->dev, "\tstage 2 translation\n");
	}

	if (id & ID0_NTS) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_NESTED;
		dev_dbg(smmu->dev, "\tnested translation\n");
	}

	if (!(smmu->features &
		(ARM_SMMU_FEAT_TRANS_S1 | ARM_SMMU_FEAT_TRANS_S2))) {
		dev_err(smmu->dev, "\tno translation support!\n");
		return -ENODEV;
	}

	if ((id & ID0_S1TS) &&
		((smmu->version < ARM_SMMU_V2) || !(id & ID0_ATOSNS))) {
		smmu->features |= ARM_SMMU_FEAT_TRANS_OPS;
		dev_dbg(smmu->dev, "\taddress translation ops\n");
	}

	/*
	 * In order for DMA API calls to work properly, we must defer to what
	 * the DT says about coherency, regardless of what the hardware claims.
	 * Fortunately, this also opens up a workaround for systems where the
	 * ID register value has ended up configured incorrectly.
	 */
	cttw_dt = of_dma_is_coherent(smmu->dev->of_node);
	cttw_reg = !!(id & ID0_CTTW);
	if (cttw_dt)
		smmu->features |= ARM_SMMU_FEAT_COHERENT_WALK;
	if (cttw_dt || cttw_reg)
		dev_dbg(smmu->dev, "\t%scoherent table walk\n",
			   cttw_dt ? "" : "non-");
	if (cttw_dt != cttw_reg)
		dev_notice(smmu->dev,
			   "\t(IDR0.CTTW overridden by dma-coherent property)\n");

	/* Max. number of entries we have for stream matching/indexing */
	size = 1 << ((id >> ID0_NUMSIDB_SHIFT) & ID0_NUMSIDB_MASK);
	smmu->streamid_mask = size - 1;
	if (id & ID0_SMS) {
		u32 smr;

		smmu->features |= ARM_SMMU_FEAT_STREAM_MATCH;
		size = (id >> ID0_NUMSMRG_SHIFT) & ID0_NUMSMRG_MASK;
		if (size == 0) {
			dev_err(smmu->dev,
				"stream-matching supported, but no SMRs present!\n");
			return -ENODEV;
		}

		/*
		 * SMR.ID bits may not be preserved if the corresponding MASK
		 * bits are set, so check each one separately. We can reject
		 * masters later if they try to claim IDs outside these masks.
		 */
		smr = smmu->streamid_mask << SMR_ID_SHIFT;
		writel_relaxed(smr, gr0_base + ARM_SMMU_GR0_SMR(0));
		smr = readl_relaxed(gr0_base + ARM_SMMU_GR0_SMR(0));
		smmu->streamid_mask = smr >> SMR_ID_SHIFT;

		smr = smmu->streamid_mask << SMR_MASK_SHIFT;
		writel_relaxed(smr, gr0_base + ARM_SMMU_GR0_SMR(0));
		smr = readl_relaxed(gr0_base + ARM_SMMU_GR0_SMR(0));
		smmu->smr_mask_mask = smr >> SMR_MASK_SHIFT;

		/* Zero-initialised to mark as invalid */
		smmu->smrs = devm_kcalloc(smmu->dev, size, sizeof(*smmu->smrs),
					  GFP_KERNEL);
		if (!smmu->smrs)
			return -ENOMEM;

		dev_notice(smmu->dev,
			   "\tstream matching with %lu register groups, mask 0x%x",
			   size, smmu->smr_mask_mask);
	}
	/* s2cr->type == 0 means translation, so initialise explicitly */
	smmu->s2crs = devm_kmalloc_array(smmu->dev, size, sizeof(*smmu->s2crs),
					 GFP_KERNEL);
	if (!smmu->s2crs)
		return -ENOMEM;
	for (i = 0; i < size; i++)
		smmu->s2crs[i] = s2cr_init_val;

	smmu->num_mapping_groups = size;
	mutex_init(&smmu->stream_map_mutex);

	if (smmu->version < ARM_SMMU_V2 || !(id & ID0_PTFS_NO_AARCH32)) {
		smmu->features |= ARM_SMMU_FEAT_FMT_AARCH32_L;
		if (!(id & ID0_PTFS_NO_AARCH32S))
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH32_S;
	}

	/* ID1 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID1);
	smmu->pgshift = (id & ID1_PAGESIZE) ? 16 : 12;

	/* Check for size mismatch of SMMU address space from mapped region */
	size = 1 << (((id >> ID1_NUMPAGENDXB_SHIFT) & ID1_NUMPAGENDXB_MASK) + 1);
	size *= 2 << smmu->pgshift;
	if (smmu->size != size)
		dev_warn(smmu->dev,
			"SMMU address space size (0x%lx) differs from mapped region size (0x%lx)!\n",
			size, smmu->size);

	smmu->num_s2_context_banks = (id >> ID1_NUMS2CB_SHIFT) & ID1_NUMS2CB_MASK;
	smmu->num_context_banks = (id >> ID1_NUMCB_SHIFT) & ID1_NUMCB_MASK;
	if (smmu->num_s2_context_banks > smmu->num_context_banks) {
		dev_err(smmu->dev, "impossible number of S2 context banks!\n");
		return -ENODEV;
	}
	dev_dbg(smmu->dev, "\t%u context banks (%u stage-2 only)\n",
		   smmu->num_context_banks, smmu->num_s2_context_banks);
	/*
	 * Cavium CN88xx erratum #27704.
	 * Ensure ASID and VMID allocation is unique across all SMMUs in
	 * the system.
	 */
	if (smmu->model == CAVIUM_SMMUV2) {
		smmu->cavium_id_base =
			atomic_add_return(smmu->num_context_banks,
					  &cavium_smmu_context_count);
		smmu->cavium_id_base -= smmu->num_context_banks;
	}

	/* ID2 */
	id = readl_relaxed(gr0_base + ARM_SMMU_GR0_ID2);
	size = arm_smmu_id_size_to_bits((id >> ID2_IAS_SHIFT) & ID2_IAS_MASK);
	smmu->ipa_size = size;

	/* The output mask is also applied for bypass */
	size = arm_smmu_id_size_to_bits((id >> ID2_OAS_SHIFT) & ID2_OAS_MASK);
	smmu->pa_size = size;

	if (id & ID2_VMID16)
		smmu->features |= ARM_SMMU_FEAT_VMID16;

	/*
	 * What the page table walker can address actually depends on which
	 * descriptor format is in use, but since a) we don't know that yet,
	 * and b) it can vary per context bank, this will have to do...
	 */
	if (dma_set_mask_and_coherent(smmu->dev, DMA_BIT_MASK(size)))
		dev_warn(smmu->dev,
			 "failed to set DMA mask for table walker\n");

	if (smmu->version < ARM_SMMU_V2) {
		smmu->va_size = smmu->ipa_size;
		if (smmu->version == ARM_SMMU_V1_64K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_64K;
	} else {
		size = (id >> ID2_UBS_SHIFT) & ID2_UBS_MASK;
		smmu->va_size = arm_smmu_id_size_to_bits(size);
		if (id & ID2_PTFS_4K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_4K;
		if (id & ID2_PTFS_16K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_16K;
		if (id & ID2_PTFS_64K)
			smmu->features |= ARM_SMMU_FEAT_FMT_AARCH64_64K;
	}

	/* Now we've corralled the various formats, what'll it do? */
	if (smmu->features & ARM_SMMU_FEAT_FMT_AARCH32_S)
		smmu->pgsize_bitmap |= SZ_4K | SZ_64K | SZ_1M | SZ_16M;
	if (smmu->features &
	    (ARM_SMMU_FEAT_FMT_AARCH32_L | ARM_SMMU_FEAT_FMT_AARCH64_4K))
		smmu->pgsize_bitmap |= SZ_4K | SZ_2M | SZ_1G;
	if (smmu->features & ARM_SMMU_FEAT_FMT_AARCH64_16K)
		smmu->pgsize_bitmap |= SZ_16K | SZ_32M;
	if (smmu->features & ARM_SMMU_FEAT_FMT_AARCH64_64K)
		smmu->pgsize_bitmap |= SZ_64K | SZ_512M;

	if (arm_smmu_ops.pgsize_bitmap == -1UL)
		arm_smmu_ops.pgsize_bitmap = smmu->pgsize_bitmap;
	else
		arm_smmu_ops.pgsize_bitmap |= smmu->pgsize_bitmap;
	dev_dbg(smmu->dev, "\tSupported page sizes: 0x%08lx\n",
		   smmu->pgsize_bitmap);


	if (smmu->features & ARM_SMMU_FEAT_TRANS_S1)
		dev_dbg(smmu->dev, "\tStage-1: %lu-bit VA -> %lu-bit IPA\n",
			smmu->va_size, smmu->ipa_size);

	if (smmu->features & ARM_SMMU_FEAT_TRANS_S2)
		dev_dbg(smmu->dev, "\tStage-2: %lu-bit IPA -> %lu-bit PA\n",
			smmu->ipa_size, smmu->pa_size);

	return 0;
}

static int arm_smmu_arch_init(struct arm_smmu_device *smmu)
{
	if (!smmu->arch_ops)
		return 0;
	if (!smmu->arch_ops->init)
		return 0;
	return smmu->arch_ops->init(smmu);
}

static void arm_smmu_arch_device_reset(struct arm_smmu_device *smmu)
{
	if (!smmu->arch_ops)
		return;
	if (!smmu->arch_ops->device_reset)
		return;
	return smmu->arch_ops->device_reset(smmu);
}

struct arm_smmu_match_data {
	enum arm_smmu_arch_version version;
	enum arm_smmu_implementation model;
	struct arm_smmu_arch_ops *arch_ops;
};

#define ARM_SMMU_MATCH_DATA(name, ver, imp, ops)	\
static struct arm_smmu_match_data name = {		\
.version = ver,						\
.model = imp,						\
.arch_ops = ops,					\
}							\

struct arm_smmu_arch_ops qsmmuv500_arch_ops;

ARM_SMMU_MATCH_DATA(smmu_generic_v1, ARM_SMMU_V1, GENERIC_SMMU, NULL);
ARM_SMMU_MATCH_DATA(smmu_generic_v2, ARM_SMMU_V2, GENERIC_SMMU, NULL);
ARM_SMMU_MATCH_DATA(arm_mmu401, ARM_SMMU_V1_64K, GENERIC_SMMU, NULL);
ARM_SMMU_MATCH_DATA(arm_mmu500, ARM_SMMU_V2, ARM_MMU500, NULL);
ARM_SMMU_MATCH_DATA(cavium_smmuv2, ARM_SMMU_V2, CAVIUM_SMMUV2, NULL);
ARM_SMMU_MATCH_DATA(qcom_smmuv2, ARM_SMMU_V2, QCOM_SMMUV2, &qsmmuv2_arch_ops);
ARM_SMMU_MATCH_DATA(qcom_smmuv500, ARM_SMMU_V2, QCOM_SMMUV500,
		    &qsmmuv500_arch_ops);

static const struct of_device_id arm_smmu_of_match[] = {
	{ .compatible = "arm,smmu-v1", .data = &smmu_generic_v1 },
	{ .compatible = "arm,smmu-v2", .data = &smmu_generic_v2 },
	{ .compatible = "arm,mmu-400", .data = &smmu_generic_v1 },
	{ .compatible = "arm,mmu-401", .data = &arm_mmu401 },
	{ .compatible = "arm,mmu-500", .data = &arm_mmu500 },
	{ .compatible = "cavium,smmu-v2", .data = &cavium_smmuv2 },
	{ .compatible = "qcom,smmu-v2", .data = &qcom_smmuv2 },
	{ .compatible = "qcom,qsmmu-v500", .data = &qcom_smmuv500 },
	{ },
};
MODULE_DEVICE_TABLE(of, arm_smmu_of_match);

static int qsmmuv500_tbu_register(struct device *dev, void *data);
static int arm_smmu_device_dt_probe(struct platform_device *pdev)
{
	const struct arm_smmu_match_data *data;
	struct resource *res;
	struct arm_smmu_device *smmu;
	struct device *dev = &pdev->dev;
	int num_irqs, i, err;

	smmu = devm_kzalloc(dev, sizeof(*smmu), GFP_KERNEL);
	if (!smmu) {
		dev_err(dev, "failed to allocate arm_smmu_device\n");
		return -ENOMEM;
	}
	smmu->dev = dev;
	spin_lock_init(&smmu->atos_lock);
	idr_init(&smmu->asid_idr);
	mutex_init(&smmu->idr_mutex);

	data = of_device_get_match_data(dev);
	smmu->version = data->version;
	smmu->model = data->model;
	smmu->arch_ops = data->arch_ops;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	smmu->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(smmu->base))
		return PTR_ERR(smmu->base);
	smmu->size = resource_size(res);

	if (of_property_read_u32(dev->of_node, "#global-interrupts",
				 &smmu->num_global_irqs)) {
		dev_err(dev, "missing #global-interrupts property\n");
		return -ENODEV;
	}

	num_irqs = 0;
	while ((res = platform_get_resource(pdev, IORESOURCE_IRQ, num_irqs))) {
		num_irqs++;
		if (num_irqs > smmu->num_global_irqs)
			smmu->num_context_irqs++;
	}

	if (!smmu->num_context_irqs) {
		dev_err(dev, "found %d interrupts but expected at least %d\n",
			num_irqs, smmu->num_global_irqs + 1);
		return -ENODEV;
	}

	smmu->irqs = devm_kzalloc(dev, sizeof(*smmu->irqs) * num_irqs,
				  GFP_KERNEL);
	if (!smmu->irqs) {
		dev_err(dev, "failed to allocate %d irqs\n", num_irqs);
		return -ENOMEM;
	}

	for (i = 0; i < num_irqs; ++i) {
		int irq = platform_get_irq(pdev, i);

		if (irq < 0) {
			dev_err(dev, "failed to get irq index %d\n", i);
			return -ENODEV;
		}
		smmu->irqs[i] = irq;
	}

	parse_driver_options(smmu);

	smmu->pwr = arm_smmu_init_power_resources(pdev);
	if (IS_ERR(smmu->pwr))
		return PTR_ERR(smmu->pwr);

	err = arm_smmu_power_on(smmu->pwr);
	if (err)
		goto out_exit_power_resources;

	err = arm_smmu_device_cfg_probe(smmu);
	if (err)
		goto out_power_off;

	err = arm_smmu_parse_impl_def_registers(smmu);
	if (err)
		goto out_power_off;

	if (smmu->version == ARM_SMMU_V2 &&
	    smmu->num_context_banks != smmu->num_context_irqs) {
		dev_err(dev,
			"found %d context interrupt(s) but have %d context banks. assuming %d context interrupts.\n",
			smmu->num_context_irqs, smmu->num_context_banks,
			smmu->num_context_banks);
		smmu->num_context_irqs = smmu->num_context_banks;
	}

	for (i = 0; i < smmu->num_global_irqs; ++i) {
		err = devm_request_threaded_irq(smmu->dev, smmu->irqs[i],
					NULL, arm_smmu_global_fault,
					IRQF_ONESHOT | IRQF_SHARED,
					"arm-smmu global fault", smmu);
		if (err) {
			dev_err(dev, "failed to request global IRQ %d (%u)\n",
				i, smmu->irqs[i]);
			goto out_power_off;
		}
	}

	err = arm_smmu_arch_init(smmu);
	if (err)
		goto out_power_off;

	platform_set_drvdata(pdev, smmu);
	arm_smmu_device_reset(smmu);
	arm_smmu_power_off(smmu->pwr);

	return 0;

out_power_off:
	arm_smmu_power_off(smmu->pwr);

out_exit_power_resources:
	arm_smmu_exit_power_resources(smmu->pwr);

	return err;
}

static int arm_smmu_device_remove(struct platform_device *pdev)
{
	struct arm_smmu_device *smmu = platform_get_drvdata(pdev);

	if (!smmu)
		return -ENODEV;

	if (arm_smmu_power_on(smmu->pwr))
		return -EINVAL;

	if (!bitmap_empty(smmu->context_map, ARM_SMMU_MAX_CBS))
		dev_err(&pdev->dev, "removing device with active domains!\n");

	idr_destroy(&smmu->asid_idr);

	/* Turn the thing off */
	writel(sCR0_CLIENTPD, ARM_SMMU_GR0_NS(smmu) + ARM_SMMU_GR0_sCR0);
	arm_smmu_power_off(smmu->pwr);

	arm_smmu_exit_power_resources(smmu->pwr);

	return 0;
}

static struct platform_driver arm_smmu_driver = {
	.driver	= {
		.name		= "arm-smmu",
		.of_match_table	= of_match_ptr(arm_smmu_of_match),
	},
	.probe	= arm_smmu_device_dt_probe,
	.remove	= arm_smmu_device_remove,
};

static int __init arm_smmu_init(void)
{
	struct device_node *np;
	int ret;

	/*
	 * Play nice with systems that don't have an ARM SMMU by checking that
	 * an ARM SMMU exists in the system before proceeding with the driver
	 * and IOMMU bus operation registration.
	 */
	np = of_find_matching_node(NULL, arm_smmu_of_match);
	if (!np)
		return 0;

	of_node_put(np);

	ret = platform_driver_register(&arm_smmu_driver);
	if (ret)
		return ret;

	/* Oh, for a proper bus abstraction */
	if (!iommu_present(&platform_bus_type))
		bus_set_iommu(&platform_bus_type, &arm_smmu_ops);

#ifdef CONFIG_ARM_AMBA
	if (!iommu_present(&amba_bustype))
		bus_set_iommu(&amba_bustype, &arm_smmu_ops);
#endif

#ifdef CONFIG_PCI
	if (!iommu_present(&pci_bus_type)) {
		pci_request_acs();
		bus_set_iommu(&pci_bus_type, &arm_smmu_ops);
	}
#endif

	return 0;
}

static void __exit arm_smmu_exit(void)
{
	return platform_driver_unregister(&arm_smmu_driver);
}

subsys_initcall(arm_smmu_init);
module_exit(arm_smmu_exit);

#define DEBUG_SID_HALT_REG		0x0
#define DEBUG_SID_HALT_VAL		(0x1 << 16)

#define DEBUG_SR_HALT_ACK_REG		0x20
#define DEBUG_SR_HALT_ACK_VAL		(0x1 << 1)

#define TBU_DBG_TIMEOUT_US		30000

struct qsmmuv500_tbu_device {
	struct list_head		list;
	struct device			*dev;
	struct arm_smmu_device		*smmu;
	void __iomem			*base;
	void __iomem			*status_reg;

	struct arm_smmu_power_resources *pwr;

	/* Protects halt count */
	spinlock_t			halt_lock;
	u32				halt_count;
};

static int qsmmuv500_tbu_power_on_all(struct arm_smmu_device *smmu)
{
	struct qsmmuv500_tbu_device *tbu;
	struct list_head *list = smmu->archdata;
	int ret = 0;

	list_for_each_entry(tbu, list, list) {
		ret = arm_smmu_power_on(tbu->pwr);
		if (ret)
			break;
	}
	if (!ret)
		return 0;

	list_for_each_entry_continue_reverse(tbu, list, list) {
		arm_smmu_power_off(tbu->pwr);
	}
	return ret;
}

static void qsmmuv500_tbu_power_off_all(struct arm_smmu_device *smmu)
{
	struct qsmmuv500_tbu_device *tbu;
	struct list_head *list = smmu->archdata;

	list_for_each_entry_reverse(tbu, list, list) {
		arm_smmu_power_off(tbu->pwr);
	}
}

static int qsmmuv500_tbu_halt(struct qsmmuv500_tbu_device *tbu)
{
	unsigned long flags;
	u32 val;
	void __iomem *base;

	spin_lock_irqsave(&tbu->halt_lock, flags);
	if (tbu->halt_count) {
		tbu->halt_count++;
		spin_unlock_irqrestore(&tbu->halt_lock, flags);
		return 0;
	}

	base = tbu->base;
	val = readl_relaxed(base + DEBUG_SID_HALT_REG);
	val |= DEBUG_SID_HALT_VAL;
	writel_relaxed(val, base + DEBUG_SID_HALT_REG);

	if (readl_poll_timeout_atomic(base + DEBUG_SR_HALT_ACK_REG,
					val, (val & DEBUG_SR_HALT_ACK_VAL),
					0, TBU_DBG_TIMEOUT_US)) {
		dev_err(tbu->dev, "Couldn't halt TBU!\n");
		spin_unlock_irqrestore(&tbu->halt_lock, flags);
		return -ETIMEDOUT;
	}

	tbu->halt_count = 1;
	spin_unlock_irqrestore(&tbu->halt_lock, flags);
	return 0;
}

static void qsmmuv500_tbu_resume(struct qsmmuv500_tbu_device *tbu)
{
	unsigned long flags;
	u32 val;
	void __iomem *base;

	spin_lock_irqsave(&tbu->halt_lock, flags);
	if (!tbu->halt_count) {
		WARN(1, "%s: bad tbu->halt_count", dev_name(tbu->dev));
		spin_unlock_irqrestore(&tbu->halt_lock, flags);
		return;

	} else if (tbu->halt_count > 1) {
		tbu->halt_count--;
		spin_unlock_irqrestore(&tbu->halt_lock, flags);
		return;
	}

	base = tbu->base;
	val = readl_relaxed(base + DEBUG_SID_HALT_REG);
	val &= ~DEBUG_SID_HALT_VAL;
	writel_relaxed(val, base + DEBUG_SID_HALT_REG);

	tbu->halt_count = 0;
	spin_unlock_irqrestore(&tbu->halt_lock, flags);
}

static int qsmmuv500_halt_all(struct arm_smmu_device *smmu)
{
	struct qsmmuv500_tbu_device *tbu;
	struct list_head *list = smmu->archdata;
	int ret = 0;

	list_for_each_entry(tbu, list, list) {
		ret = qsmmuv500_tbu_halt(tbu);
		if (ret)
			break;
	}

	if (!ret)
		return 0;

	list_for_each_entry_continue_reverse(tbu, list, list) {
		qsmmuv500_tbu_resume(tbu);
	}
	return ret;
}

static void qsmmuv500_resume_all(struct arm_smmu_device *smmu)
{
	struct qsmmuv500_tbu_device *tbu;
	struct list_head *list = smmu->archdata;

	list_for_each_entry(tbu, list, list) {
		qsmmuv500_tbu_resume(tbu);
	}
}

static void qsmmuv500_device_reset(struct arm_smmu_device *smmu)
{
	int i, ret;
	struct arm_smmu_impl_def_reg *regs = smmu->impl_def_attach_registers;

	ret = qsmmuv500_tbu_power_on_all(smmu);
	if (ret)
		return;

	/* Program implementation defined registers */
	qsmmuv500_halt_all(smmu);
	for (i = 0; i < smmu->num_impl_def_attach_registers; ++i)
		writel_relaxed(regs[i].value,
			ARM_SMMU_GR0(smmu) + regs[i].offset);
	qsmmuv500_resume_all(smmu);
	qsmmuv500_tbu_power_off_all(smmu);
}

static int qsmmuv500_tbu_register(struct device *dev, void *data)
{
	struct arm_smmu_device *smmu = data;
	struct qsmmuv500_tbu_device *tbu;
	struct list_head *list = smmu->archdata;

	if (!dev->driver) {
		dev_err(dev, "TBU failed probe, QSMMUV500 cannot continue!\n");
		return -EINVAL;
	}

	tbu = dev_get_drvdata(dev);

	INIT_LIST_HEAD(&tbu->list);
	tbu->smmu = smmu;
	list_add(&tbu->list, list);
	return 0;
}

static int qsmmuv500_arch_init(struct arm_smmu_device *smmu)
{
	struct device *dev = smmu->dev;
	struct list_head *list;
	int ret;

	list = devm_kzalloc(dev, sizeof(*list), GFP_KERNEL);
	if (!list)
		return -ENOMEM;

	INIT_LIST_HEAD(list);
	smmu->archdata = list;

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret)
		return ret;

	/* Attempt to register child devices */
	ret = device_for_each_child(dev, smmu, qsmmuv500_tbu_register);
	if (ret)
		return -EINVAL;

	return 0;
}

struct arm_smmu_arch_ops qsmmuv500_arch_ops = {
	.init = qsmmuv500_arch_init,
	.device_reset = qsmmuv500_device_reset,
};

static const struct of_device_id qsmmuv500_tbu_of_match[] = {
	{.compatible = "qcom,qsmmuv500-tbu"},
	{}
};

static int qsmmuv500_tbu_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct qsmmuv500_tbu_device *tbu;

	tbu = devm_kzalloc(dev, sizeof(*tbu), GFP_KERNEL);
	if (!tbu)
		return -ENOMEM;

	INIT_LIST_HEAD(&tbu->list);
	tbu->dev = dev;
	spin_lock_init(&tbu->halt_lock);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	tbu->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(tbu->base))
		return PTR_ERR(tbu->base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "status-reg");
	tbu->status_reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(tbu->status_reg))
		return PTR_ERR(tbu->status_reg);

	tbu->pwr = arm_smmu_init_power_resources(pdev);
	if (IS_ERR(tbu->pwr))
		return PTR_ERR(tbu->pwr);

	dev_set_drvdata(dev, tbu);
	return 0;
}

static struct platform_driver qsmmuv500_tbu_driver = {
	.driver	= {
		.name		= "qsmmuv500-tbu",
		.of_match_table	= of_match_ptr(qsmmuv500_tbu_of_match),
	},
	.probe	= qsmmuv500_tbu_probe,
};

static int __init qsmmuv500_tbu_init(void)
{
	return platform_driver_register(&qsmmuv500_tbu_driver);
}
subsys_initcall(qsmmuv500_tbu_init);

MODULE_DESCRIPTION("IOMMU API for ARM architected SMMU implementations");
MODULE_AUTHOR("Will Deacon <will.deacon@arm.com>");
MODULE_LICENSE("GPL v2");
