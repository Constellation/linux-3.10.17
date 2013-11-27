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
#include <core/gpuobj.h>

#include <subdev/instmem.h>
#include <subdev/bar.h>
#include <subdev/vm.h>

#include <subdev/paravirt.h>

struct nouveau_paravirt_gpuobj_class {
	u64 size;
	u32 align;
	u32 flags;
};

int
nouveau_paravirt_gpuobj_create_(struct nouveau_object *parent,
				struct nouveau_object *engine,
				struct nouveau_oclass *oclass, u32 pclass,
				u32 size, u32 align, u32 flags,
				int length, void **pobject)
{
	struct nouveau_paravirt_gpuobj *object;
	int ret;

	ret = nouveau_object_create_(parent, engine, oclass, pclass |
				     NV_PARAVIRT_CLASS, length, pobject);
	object = *pobject;
	if (ret)
		return ret;

	object->base.parent = parent;
	object->base.flags = flags;
	object->base.addr = 0xdeadbeef;  /* dummy */
	object->base.size = size;
	object->paravirt_id = 0xdeadbeef;  /* dummy */

	return 0;
}

static int
_nouveau_paravirt_gpuobj_ctor(struct nouveau_object *parent,
			      struct nouveau_object *engine,
			      struct nouveau_oclass *oclass, void *data, u32 size,
			      struct nouveau_object **pobject)
{
	struct nouveau_paravirt_gpuobj_class *args = data;
	struct nouveau_paravirt_gpuobj *object;
	struct nouveau_paravirt_slot *slot;
	struct nouveau_paravirt *paravirt;
	int ret;

	nv_warn(parent, "[%s]\n", __PRETTY_FUNCTION__);

	ret = nouveau_paravirt_gpuobj_create(parent, engine, oclass, 0,
				             args->size, args->align, args->flags,
				             &object);
	*pobject = nv_object(object);
	if (ret)
		return ret;

	paravirt = nouveau_paravirt(parent);
	slot = nouveau_paravirt_alloc_slot(paravirt);
	slot->u8[0] = NOUVEAU_PV_OP_MEM_ALLOC;
	slot->u32[1] = args->size;
	nouveau_paravirt_call(paravirt, slot);

	ret = slot->u32[0];
	object->paravirt_id = slot->u32[1];

	nouveau_paravirt_free_slot(paravirt, slot);

	BUG_ON(object->paravirt_id == 0);

	if (ret) {
		nouveau_object_dec(nv_object(object), false);
		return ret;
	}

	return 0;
}

void
nouveau_paravirt_gpuobj_destroy(struct nouveau_paravirt_gpuobj *object)
{
	struct nouveau_paravirt *paravirt = nouveau_paravirt(object);
	if (object->paravirt_id) {
		struct nouveau_paravirt_slot* slot = nouveau_paravirt_alloc_slot(paravirt);
		slot->u8[0] = NOUVEAU_PV_OP_MEM_FREE;
		slot->u32[1] = object->paravirt_id;
		nouveau_paravirt_call(paravirt, slot);
		nouveau_paravirt_free_slot(paravirt, slot);
	}
	nouveau_object_destroy(&(object->base.base));
}

void
_nouveau_paravirt_gpuobj_dtor(struct nouveau_object *object)
{
	nouveau_paravirt_gpuobj_destroy(nv_paravirt_gpuobj(object));
}

int
_nouveau_paravirt_gpuobj_init(struct nouveau_object *object)
{
	return nouveau_object_init(&(nv_paravirt_gpuobj(object)->base.base));
}

int
_nouveau_paravirt_gpuobj_fini(struct nouveau_object *object, bool suspend)
{
	return nouveau_object_fini(&(nv_paravirt_gpuobj(object)->base.base), suspend);
}

static struct nouveau_oclass
_nouveau_paravirt_gpuobj_oclass = {
	.handle = NV_PARAVIRT_CLASS,
	.ofuncs = &(struct nouveau_ofuncs) {
		.ctor = _nouveau_paravirt_gpuobj_ctor,
		.dtor = _nouveau_paravirt_gpuobj_dtor,
		.init = _nouveau_paravirt_gpuobj_init,
		.fini = _nouveau_paravirt_gpuobj_fini,
	},
};

int
nouveau_paravirt_gpuobj_new(struct nouveau_object *parent,
			    u32 size, u32 align, u32 flags,
			    struct nouveau_gpuobj **pgpuobj)
{
	struct nouveau_object *engine = parent;
	struct nouveau_paravirt_gpuobj_class args = {
		.size = size,
		.align = align,
		.flags = flags,
	};

	if (!nv_iclass(engine, NV_SUBDEV_CLASS))
		engine = engine->engine;
	BUG_ON(engine == NULL);

	return nouveau_object_ctor(parent, engine, &_nouveau_paravirt_gpuobj_oclass,
				   &args, sizeof(args),
				   (struct nouveau_object **)pgpuobj);
}



