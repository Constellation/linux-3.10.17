#ifndef __NOUVEAU_PARAVIRT_H__
#define __NOUVEAU_PARAVIRT_H__

#include <core/subdev.h>
#include <core/device.h>
#include <subdev/fb.h>

#define NOUVEAU_PV_REG_BAR 4
#define NOUVEAU_PV_SLOT_SIZE 0x1000ULL
#define NOUVEAU_PV_SLOT_NUM 64ULL
#define NOUVEAU_PV_SLOT_TOTAL (NOUVEAU_PV_SLOT_SIZE * NOUVEAU_PV_SLOT_NUM)
#define NOUVEAU_PV_BATCH_SIZE 128ULL

/* PV OPS */
enum {
  NOUVEAU_PV_OP_SET_PGD,
  NOUVEAU_PV_OP_MAP_PGT,
  NOUVEAU_PV_OP_MAP,
  NOUVEAU_PV_OP_MAP_BATCH,
  NOUVEAU_PV_OP_MAP_SG_BATCH,
  NOUVEAU_PV_OP_UNMAP_BATCH,
  NOUVEAU_PV_OP_VM_FLUSH,
  NOUVEAU_PV_OP_MEM_ALLOC,
  NOUVEAU_PV_OP_MEM_FREE,
  NOUVEAU_PV_OP_BAR3_PGT,
};

struct nouveau_paravirt {
	struct nouveau_subdev base;
	spinlock_t lock;
	spinlock_t slot_lock;
	u8 __iomem *slot;
	u8 __iomem *mmio;
	u64 used_slot;
	struct semaphore sema;
};

static inline struct nouveau_paravirt *
nouveau_paravirt(void *obj)
{
	return (void *)nv_device(obj)->subdev[NVDEV_SUBDEV_PARAVIRT];
}

extern struct nouveau_oclass nvc0_paravirt_oclass;

struct nouveau_paravirt_slot {
	union {
		u8   u8[NOUVEAU_PV_SLOT_SIZE / sizeof(u8) ];
		u16 u16[NOUVEAU_PV_SLOT_SIZE / sizeof(u16)];
		u32 u32[NOUVEAU_PV_SLOT_SIZE / sizeof(u32)];
		u64 u64[NOUVEAU_PV_SLOT_SIZE / sizeof(u64)];
	};
};

struct nouveau_paravirt_mem {
	struct nouveau_paravirt *dev;
	struct kref refcount;
	u32 id;
	u32 size;
};

struct nouveau_channel;
struct nouveau_vma;
struct nouveau_mem;

int nouveau_paravirt_enabled(struct nouveau_paravirt *);

struct nouveau_paravirt_slot* nouveau_paravirt_alloc_slot(struct nouveau_paravirt *);
void nouveau_paravirt_free_slot(struct nouveau_paravirt *, struct nouveau_paravirt_slot *);
int nouveau_paravirt_call(struct nouveau_paravirt *, struct nouveau_paravirt_slot *);

int nouveau_paravirt_mem_new(struct nouveau_paravirt *, u32 size, struct nouveau_paravirt_mem **);
void nouveau_paravirt_mem_ref(struct nouveau_paravirt_mem *, struct nouveau_paravirt_mem **);

int nouveau_paravirt_set_pgd(struct nouveau_paravirt *, struct nouveau_channel*, struct nouveau_paravirt_mem*, int id);
int nouvaeu_paravirt_map_pgt(struct nouveau_paravirt *, struct nouveau_paravirt_mem *pgd, u32 index, struct nouveau_paravirt_mem *pgt[2]);
int nouveau_paravirt_map(struct nouveau_paravirt *, struct nouveau_paravirt_mem *pgt, u32 index, u64 phys);
int nouveau_paravirt_map_batch(struct nouveau_paravirt *, struct nouveau_paravirt_mem *pgt, u32 index, u64 phys, u32 next, u32 count);
int nouveau_paravirt_unmap_batch(struct nouveau_paravirt *, struct nouveau_paravirt_mem *pgt, u32 index, u32 count);
int nouveau_paravirt_map_sg_batch(struct nouveau_paravirt *, struct nouveau_paravirt_mem *pgt, u32 index, struct nouveau_vma *vma, struct nouveau_mem *mem, dma_addr_t *list, u32 count);
int nouveau_paravirt_vm_flush(struct nouveau_paravirt *, struct nouveau_paravirt_mem* pgd, u32 engine);
int nouveau_paravirt_bar3_pgt(struct nouveau_paravirt *, struct nouveau_paravirt_mem *pgt);

#define nouveau_paravirt_create(p,e,o,d)                                          \
	nouveau_subdev_create((p), (e), (o), 0, "PARAVIRT", "paravirt", d)
#define nouveau_paravirt_destroy(p)                                               \
	nouveau_subdev_destroy(&(p)->base)
#define nouveau_paravirt_init(p)                                                  \
	nouveau_subdev_init(&(p)->base)
#define nouveau_paravirt_fini(p,s)                                                \
	nouveau_subdev_fini(&(p)->base, (s))

#define _nouveau_paravirt_init _nouveau_subdev_init
#define _nouveau_paravirt_fini _nouveau_subdev_fini

#endif
