/*
 * Copyright 2012 Red Hat Inc.
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
 * Authors: Ben Skeggs
 */

#include <subdev/fb.h>
#include <subdev/ltcg.h>
#include <subdev/bios.h>

struct nvc0_fb_priv {
	struct nouveau_fb base;
	struct page *r100c10_page;
	dma_addr_t r100c10;
};

extern const u8 nvc0_pte_storage_type_map[256];


static bool
nvc0_fb_memtype_valid(struct nouveau_fb *pfb, u32 tile_flags)
{
	u8 memtype = (tile_flags & 0x0000ff00) >> 8;
	return likely((nvc0_pte_storage_type_map[memtype] != 0xff));
}

static int
nvc0_fb_vram_init(struct nouveau_fb *pfb)
{
	pfb->ram.type = NV_MEM_TYPE_GDDR5;
	pfb->ram.ranks = 2;  // ???
    pfb->ram.size = (0x2ULL << 30);
    return nouveau_mm_init(&pfb->vram, 0, pfb->ram.size >> 12, 1);
}

static int
nvc0_fb_vram_new(struct nouveau_fb *pfb, u64 size, u32 align, u32 ncmin,
		 u32 memtype, struct nouveau_mem **pmem)
{
	struct nouveau_mm *mm = &pfb->vram;
	struct nouveau_mm_node *r;
	struct nouveau_mem *mem;
	int type = (memtype & 0x0ff);
	int back = (memtype & 0x800);
	int ret;
	const bool comp = nvc0_pte_storage_type_map[type] != type;

	size  >>= 12;
	align >>= 12;
	ncmin >>= 12;
	if (!ncmin)
		ncmin = size;

	mem = kzalloc(sizeof(*mem), GFP_KERNEL);
	if (!mem)
		return -ENOMEM;

	INIT_LIST_HEAD(&mem->regions);
	mem->size = size;

	mutex_lock(&pfb->base.mutex);
	if (comp) {
		struct nouveau_ltcg *ltcg = nouveau_ltcg(pfb->base.base.parent);

		/* compression only works with lpages */
		if (align == (1 << (17 - 12))) {
			int n = size >> 5;
			ltcg->tags_alloc(ltcg, n, &mem->tag);
		}
		if (unlikely(!mem->tag))
			type = nvc0_pte_storage_type_map[type];
	}
	mem->memtype = type;

	do {
		if (back)
			ret = nouveau_mm_tail(mm, 1, size, ncmin, align, &r);
		else
			ret = nouveau_mm_head(mm, 1, size, ncmin, align, &r);
		if (ret) {
			mutex_unlock(&pfb->base.mutex);
			pfb->ram.put(pfb, &mem);
			return ret;
		}

		list_add_tail(&r->rl_entry, &mem->regions);
		size -= r->length;
	} while (size);
	mutex_unlock(&pfb->base.mutex);

	r = list_first_entry(&mem->regions, struct nouveau_mm_node, rl_entry);
	mem->offset = (u64)r->offset << 12;
	*pmem = mem;
	return 0;
}

static void
nvc0_fb_vram_del(struct nouveau_fb *pfb, struct nouveau_mem **pmem)
{
	struct nouveau_ltcg *ltcg = nouveau_ltcg(pfb->base.base.parent);

	if ((*pmem)->tag)
		ltcg->tags_free(ltcg, &(*pmem)->tag);

	nv50_fb_vram_del(pfb, pmem);
}

static int
nvc0_fb_init(struct nouveau_object *object)
{
	struct nvc0_fb_priv *priv = (void *)object;
	int ret;

	ret = nouveau_fb_init(&priv->base);
	if (ret)
		return ret;
	return 0;
}

static void
nvc0_fb_dtor(struct nouveau_object *object)
{
	struct nouveau_device *device = nv_device(object);
	struct nvc0_fb_priv *priv = (void *)object;
	nouveau_fb_destroy(&priv->base);
}

static int
nvc0_fb_ctor(struct nouveau_object *parent, struct nouveau_object *engine,
	     struct nouveau_oclass *oclass, void *data, u32 size,
	     struct nouveau_object **pobject)
{
	struct nouveau_device *device = nv_device(parent);
	struct nvc0_fb_priv *priv;
	int ret;

	ret = nouveau_fb_create(parent, engine, oclass, &priv);
	*pobject = nv_object(priv);
	if (ret)
		return ret;

	priv->base.memtype_valid = nvc0_fb_memtype_valid;
	priv->base.ram.init = nvc0_fb_vram_init;
	priv->base.ram.get = nvc0_fb_vram_new;
	priv->base.ram.put = nvc0_fb_vram_del;
	return nouveau_fb_preinit(&priv->base);
}


struct nouveau_oclass
nvc0_fb_oclass = {
	.handle = NV_SUBDEV(FB, 0xc0),
	.ofuncs = &(struct nouveau_ofuncs) {
		.ctor = nvc0_fb_ctor,
		.dtor = nvc0_fb_dtor,
		.init = nvc0_fb_init,
		.fini = _nouveau_fb_fini,
	},
};
