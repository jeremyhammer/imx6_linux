/*
 * CAAM/SEC 4.x driver backend
 * Private/internal definitions between modules
 *
 * Copyright (C) 2008-2012 Freescale Semiconductor, Inc.
 *
 */

#ifndef INTERN_H
#define INTERN_H

#define JOBR_UNASSIGNED 0
#define JOBR_ASSIGNED 1

/* Currently comes from Kconfig param as a ^2 (driver-required) */
#define JOBR_DEPTH (1 << CONFIG_CRYPTO_DEV_FSL_CAAM_RINGSIZE)

/* Kconfig params for interrupt coalescing if selected (else zero) */
#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_INTC
#define JOBR_INTC JRCFG_ICEN
#define JOBR_INTC_TIME_THLD CONFIG_CRYPTO_DEV_FSL_CAAM_INTC_TIME_THLD
#define JOBR_INTC_COUNT_THLD CONFIG_CRYPTO_DEV_FSL_CAAM_INTC_COUNT_THLD
#else
#define JOBR_INTC 0
#define JOBR_INTC_TIME_THLD 0
#define JOBR_INTC_COUNT_THLD 0
#endif

#ifndef CONFIG_OF
#define JR_IRQRES_NAME_ROOT "irq_jr"
#define JR_MEMRES_NAME_ROOT "offset_jr"
#endif

#ifdef CONFIG_ARM
/*
 * FIXME: ARM tree doesn't seem to provide this, ergo it seems to be
 * in "platform limbo". Find a better place, perhaps.
 */
static inline void irq_dispose_mapping(unsigned int virq)
{
	return;
}
#endif


/*
 * Storage for tracking each in-process entry moving across a ring
 * Each entry on an output ring needs one of these
 */
struct caam_jrentry_info {
	void (*callbk)(struct device *dev, u32 *desc, u32 status, void *arg);
	void *cbkarg;	/* Argument per ring entry */
	u32 *desc_addr_virt;	/* Stored virt addr for postprocessing */
	dma_addr_t desc_addr_dma;	/* Stored bus addr for done matching */
	u32 desc_size;	/* Stored size for postprocessing, header derived */
};

/* Private sub-storage for a single JobR */
struct caam_drv_private_jr {
	struct device *parentdev;	/* points back to controller dev */
	int ridx;
	struct caam_job_ring __iomem *rregs;	/* JobR's register space */
	struct tasklet_struct irqtask[NR_CPUS];
	int irq;			/* One per queue */
	int assign;			/* busy/free */

	/* Job ring info */
	int ringsize;	/* Size of rings (assume input = output) */
	struct caam_jrentry_info *entinfo;	/* Alloc'ed 1 per ring entry */
	spinlock_t inplock ____cacheline_aligned; /* Input ring index lock */
	int inp_ring_write_index;	/* Input index "tail" */
	int head;			/* entinfo (s/w ring) head index */
	dma_addr_t *inpring;	/* Base of input ring, alloc DMA-safe */
	spinlock_t outlock ____cacheline_aligned; /* Output ring index lock */
	int out_ring_read_index;	/* Output index "tail" */
	int tail;			/* entinfo (s/w ring) tail index */
	struct jr_outentry *outring;	/* Base of output ring, DMA-safe */
};

/*
 * Driver-private storage for a single CAAM block instance
 */
struct caam_drv_private {

	struct device *dev;
	struct device **jrdev; /* Alloc'ed array per sub-device */
	spinlock_t jr_alloc_lock;
	struct platform_device *pdev;

	/* Physical-presence section */
	struct caam_ctrl *ctrl; /* controller region */
	struct caam_deco **deco; /* DECO/CCB views */
	struct caam_assurance *ac;
	struct caam_queue_if *qi; /* QI control region */

	/*
	 * Detected geometry block. Filled in from device tree if powerpc,
	 * or from register-based version detection code
	 */
	u8 total_jobrs;		/* Total Job Rings in device */
	u8 qi_present;		/* Nonzero if QI present in device */
	int secvio_irq;		/* Security violation interrupt number */

	/* which jr allocated to scatterlist crypto */
	atomic_t tfm_count ____cacheline_aligned;
	int num_jrs_for_algapi;
	struct device **algapi_jr;
	/* list of registered crypto algorithms (mk generic context handle?) */
	struct list_head alg_list;
	/* list of registered hash algorithms (mk generic context handle?) */
	struct list_head hash_list;

#ifdef CONFIG_ARM
	struct clk *caam_clk;
#endif

	/*
	 * debugfs entries for developer view into driver/device
	 * variables at runtime.
	 */
#ifdef CONFIG_DEBUG_FS
	struct dentry *dfs_root;
	struct dentry *ctl; /* controller dir */
	struct dentry *ctl_rq_dequeued, *ctl_ob_enc_req, *ctl_ib_dec_req;
	struct dentry *ctl_ob_enc_bytes, *ctl_ob_prot_bytes;
	struct dentry *ctl_ib_dec_bytes, *ctl_ib_valid_bytes;
	struct dentry *ctl_faultaddr, *ctl_faultdetail, *ctl_faultstatus;

	struct debugfs_blob_wrapper ctl_kek_wrap, ctl_tkek_wrap, ctl_tdsk_wrap;
	struct dentry *ctl_kek, *ctl_tkek, *ctl_tdsk;
#endif
};

/*
 * These startup/shutdown functions exist to enable API startup/shutdown
 * outside of the OF device detection framework. It's necessary for ARM
 * kernels as presently delivered.
 *
 * Once ARM kernels are shipping with OF support, these functions can
 * be re-integrated into the normal probe startup/exit functions,
 * and these prototypes can then be removed.
 */
#ifndef CONFIG_OF
void caam_algapi_shutdown(struct platform_device *pdev);
int caam_algapi_startup(struct platform_device *pdev);

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_AHASH_API
int caam_algapi_hash_startup(struct platform_device *pdev);
void caam_algapi_hash_shutdown(struct platform_device *pdev);
#endif

#ifdef CONFIG_CRYPTO_DEV_FSL_CAAM_RNG_API
int caam_rng_startup(struct platform_device *pdev);
void caam_rng_shutdown(void);
#endif
#endif /* CONFIG_OF */

#endif /* INTERN_H */
