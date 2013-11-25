/*
 * Copyright 2013 Keio University.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: Yusuke Suzuki
 */

#include <core/object.h>
#include <core/client.h>
#include <core/handle.h>
#include <core/option.h>
#include <subdev/vm.h>
#include <subdev/fb.h>

#include <subdev/paravirt.h>

static inline u64
nvc0_vm_addr(struct nouveau_vma *vma, u64 phys, u32 memtype, u32 target)
{
	phys >>= 8;

	phys |= 0x00000001; /* present */
	if (vma->access & NV_MEM_ACCESS_SYS)
		phys |= 0x00000002;

	phys |= ((u64)target  << 32);
	phys |= ((u64)memtype << 36);

	return phys;
}

static inline u32 nvpv_rd32(struct nouveau_paravirt *paravirt, unsigned reg)
{
	return ioread32_native(paravirt->mmio + reg);
}

static inline void nvpv_wr32(struct nouveau_paravirt *paravirt, unsigned reg, u32 val)
{
	iowrite32_native(val, paravirt->mmio + reg);
}

static inline u32 slot_pos(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_slot *slot)
{
	return (slot->u8 - paravirt->slot) / NOUVEAU_PV_SLOT_SIZE;
}

struct nouveau_paravirt_slot* nouveau_paravirt_alloc_slot(struct nouveau_paravirt *paravirt)
{
	u32 pos;
	unsigned long flags;
	u8* ret = NULL;

	down(&paravirt->sema);
	spin_lock_irqsave(&paravirt->slot_lock, flags);

	pos = fls64(paravirt->used_slot) - 1;
	paravirt->used_slot &= ~((0x1ULL) << pos);
	ret = paravirt->slot + pos * NOUVEAU_PV_SLOT_SIZE;

	spin_unlock_irqrestore(&paravirt->slot_lock, flags);

	// NV_INFO(dev, "alloc virt space 0x%llx pos %u\n", (u64)(ret), pos);
	return (struct nouveau_paravirt_slot*)ret;
}

void nouveau_paravirt_free_slot(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_slot *slot)
{
	unsigned long flags;
	u32 pos = slot_pos(paravirt, slot);

	spin_lock_irqsave(&paravirt->slot_lock, flags);
	paravirt->used_slot |= ((0x1ULL) << pos);
	spin_unlock_irqrestore(&paravirt->slot_lock, flags);
	up(&paravirt->sema);
	// NV_INFO(dev, "free virt space 0x%llx pos %u\n", (u64)(slot), pos);
}

int nouveau_paravirt_call(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_slot *slot)
{
	u32 pos = slot_pos(paravirt, slot);
	unsigned long flags;

	spin_lock_irqsave(&paravirt->lock, flags);
	nvpv_wr32(paravirt, 0xc, pos);  // invoke A3 call
	spin_unlock_irqrestore(&paravirt->lock, flags);

	return 0;
}

#if 0
int nouveau_paravirt_mem_new(struct nouveau_paravirt *paravirt, u32 size, struct nouveau_paravirt_mem **ret)
{
	struct nouveau_paravirt_mem* obj;

	obj = kzalloc(sizeof(*obj), GFP_KERNEL);
	if (!obj)
		return -ENOMEM;

	obj->dev = dev;
	kref_init(&obj->refcount);
	obj->size = size;

	{
		int ret;
		struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(dev);
		slot->u8[0] = NOUVEAU_PV_OP_MEM_ALLOC;
		slot->u32[1] = size;
		nouveau_paravirt_call(dev, slot);

		ret = slot->u32[0];
		obj->id = slot->u32[1];

		nouveau_paravirt_free_slot(dev, slot);

		if (ret) {
			nouveau_paravirt_mem_ref(NULL, &obj);
			return ret;
		}
	}

	*ret = obj;
	return 0;
}

static void nouveau_paravirt_mem_del(struct kref *ref)
{
	struct nouveau_paravirt_mem *obj = container_of(ref, struct nouveau_paravirt_mem, refcount);
	struct drm_device* dev = obj->dev;
	{
		struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(dev);
		slot->u8[0] = NOUVEAU_PV_OP_MEM_FREE;
		slot->u32[1] = obj->id;
		nouveau_paravirt_call(dev, slot);
		nouveau_paravirt_free_slot(dev, slot);
	}
	kfree(obj);
}

void nouveau_paravirt_mem_ref(struct nouveau_paravirt_mem *ref, struct nouveau_paravirt_mem **ptr)
{
	if (ref)
		kref_get(&ref->refcount);

	if (*ptr)
		kref_put(&(*ptr)->refcount, nouveau_paravirt_mem_del);

	*ptr = ref;
}
#endif

int nouveau_paravirt_set_pgd(struct nouveau_paravirt* paravirt, struct nouveau_channel* chan, struct nouveau_paravirt_mem* pgd, int id)
{
	int ret;
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_SET_PGD;
	slot->u32[1] = pgd->id;
	slot->u32[2] = id;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];

	nouveau_paravirt_free_slot(paravirt, slot);

	return ret;
}

int nouvaeu_paravirt_map_pgt(struct nouveau_paravirt* paravirt, struct nouveau_paravirt_mem *pgd, u32 index, struct nouveau_paravirt_mem *pgt[2])
{
	int ret;
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_MAP_PGT;
	slot->u32[1] = pgd->id;
	slot->u32[2] = (pgt[0]) ? pgt[0]->id : 0;
	slot->u32[3] = (pgt[1]) ? pgt[1]->id : 0;
	slot->u32[4] = index;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];

	nouveau_paravirt_free_slot(paravirt, slot);

	return ret;
}

int nouveau_paravirt_map(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_mem *pgt, u32 index, u64 phys)
{
	int ret;
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_MAP;
	slot->u32[1] = pgt->id;
	slot->u32[2] = index;
	slot->u64[2] = phys;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];

	nouveau_paravirt_free_slot(paravirt, slot);

	return ret;
}

int nouveau_paravirt_map_batch(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_mem *pgt, u32 index, u64 phys, u32 next, u32 count)
{
	int ret;
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_MAP_BATCH;
	slot->u32[1] = pgt->id;
	slot->u32[2] = index;
	slot->u32[3] = next;
	slot->u32[4] = count;
	slot->u64[3] = phys;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];

	nouveau_paravirt_free_slot(paravirt, slot);
	return ret;
}

int nouveau_paravirt_map_sg_batch(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_mem *pgt, u32 index, struct nouveau_vma *vma, struct nouveau_mem *mem, dma_addr_t *list, u32 count)
{
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	const u32 filled = count / NOUVEAU_PV_BATCH_SIZE;
	const u32 rest = count % NOUVEAU_PV_BATCH_SIZE;
	const u32 target = (vma->access & NV_MEM_ACCESS_NOSNOOP) ? 7 : 5;

	if (filled) {
		int ret;
		u32 i, j;
		for (i = 0; i < filled; ++i) {
			slot->u8[0] = NOUVEAU_PV_OP_MAP_SG_BATCH;
			slot->u32[1] = pgt->id;
			slot->u32[2] = index + NOUVEAU_PV_BATCH_SIZE * i;
			slot->u32[3] = NOUVEAU_PV_BATCH_SIZE;
			for (j = 0; j < NOUVEAU_PV_BATCH_SIZE; ++j) {
				slot->u64[2 + j] = nvc0_vm_addr(vma, *list++, mem->memtype, target);
			}
			nouveau_paravirt_call(paravirt, slot);
			ret = slot->u32[0];
			if (ret) {
				nouveau_paravirt_free_slot(paravirt, slot);
				return ret;
			}
		}
	}

	if (rest) {
		int ret;
		u32 i;
		slot->u8[0] = NOUVEAU_PV_OP_MAP_SG_BATCH;
		slot->u32[1] = pgt->id;
		slot->u32[2] = index + NOUVEAU_PV_BATCH_SIZE * filled;
		slot->u32[3] = rest;
		for (i = 0; i < rest; ++i) {
			slot->u64[2 + i] = nvc0_vm_addr(vma, *list++, mem->memtype, target);
		}
		nouveau_paravirt_call(paravirt, slot);
		ret = slot->u32[0];
		if (ret) {
			nouveau_paravirt_free_slot(paravirt, slot);
			return ret;
		}
	}

	nouveau_paravirt_free_slot(paravirt, slot);
	return 0;
}

int nouveau_paravirt_unmap_batch(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_mem *pgt, u32 index, u32 count)
{
	int ret;
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_UNMAP_BATCH;
	slot->u32[1] = pgt->id;
	slot->u32[2] = index;
	slot->u32[3] = count;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];

	nouveau_paravirt_free_slot(paravirt, slot);
	return ret;
}

int nouveau_paravirt_vm_flush(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_mem *pgd, u32 engine)
{
	int ret;
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_VM_FLUSH;
	slot->u32[1] = pgd->id;
	slot->u32[2] = engine;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];

	nouveau_paravirt_free_slot(paravirt, slot);

	return ret;
}

int nouveau_paravirt_bar3_pgt(struct nouveau_paravirt *paravirt, struct nouveau_paravirt_mem *pgt)
{
	int ret;
	struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_BAR3_PGT;
	slot->u32[1] = pgt->id;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];

	nouveau_paravirt_free_slot(paravirt, slot);

	return ret;
}

static int
nvc0_paravirt_ctor(struct nouveau_object *parent, struct nouveau_object *engine,
		   struct nouveau_oclass *oclass, void *data, u32 size,
		   struct nouveau_object **pobject)
{
	struct nouveau_device *device = nv_device(parent);
	struct pci_dev *pdev = device->pdev;
	struct nouveau_paravirt *paravirt;
	u64 address;
	int ret;

	nv_warn(parent, "[%s]\n", __PRETTY_FUNCTION__);

	ret = nouveau_paravirt_create(parent, engine, oclass, &paravirt);
	*pobject = nv_object(paravirt);
	if (ret)
		return ret;

	paravirt->used_slot = ~(0ULL);
	spin_lock_init(&paravirt->slot_lock);
	spin_lock_init(&paravirt->lock);
	sema_init(&paravirt->sema, NOUVEAU_PV_SLOT_NUM);

	if (!(paravirt->slot = kzalloc(NOUVEAU_PV_SLOT_TOTAL, GFP_KERNEL)))
		return -ENOMEM;

	((u32*)paravirt->slot)[0] = 0xdeadbeefUL;

	/* map BAR4 */
	paravirt->mmio = (u8 __iomem *)ioremap(pci_resource_start(pdev, NOUVEAU_PV_REG_BAR), pci_resource_len(pdev, NOUVEAU_PV_REG_BAR));
	if (!paravirt->mmio)
		return -ENODEV;

	/* notify this physical address to A3 */
	address = __pa(paravirt->slot);  /* convert kmalloc-ed virt to phys */
	nvpv_wr32(paravirt, 0x4, lower_32_bits(address));
	nvpv_wr32(paravirt, 0x8, upper_32_bits(address));
	if (nvpv_rd32(paravirt, 0x0) != 0x0) {
		iounmap(paravirt->mmio);
		paravirt->mmio = NULL;
		return -ENODEV;
	}

	return 0;
}

static void
nvc0_paravirt_dtor(struct nouveau_object *object)
{
	struct nouveau_paravirt *paravirt = (void *)object;

	if (paravirt->slot)
		kfree(paravirt->slot);

	if (paravirt->mmio)
		iounmap(paravirt->mmio);
	nouveau_paravirt_destroy(paravirt);
}

struct nouveau_oclass
nvc0_paravirt_oclass = {
	.handle = NV_SUBDEV(PARAVIRT, 0xc0),
	.ofuncs = &(struct nouveau_ofuncs) {
		.ctor = nvc0_paravirt_ctor,
		.dtor = nvc0_paravirt_dtor,
		.init = _nouveau_paravirt_init,
		.fini = _nouveau_paravirt_fini,
	},
};
