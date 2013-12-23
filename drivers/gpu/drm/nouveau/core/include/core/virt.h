#ifndef __NOUVEAU_VIRT_H__
#define __NOUVEAU_VIRT_H__

#include <core/device.h>
#include <core/gpuobj.h>

#define NOUVEAU_VIRT_REG_BAR 4
#define NOUVEAU_VIRT_SLOT_SIZE 0x1000ULL
#define NOUVEAU_VIRT_SLOT_NUM 64ULL
#define NOUVEAU_VIRT_SLOT_TOTAL (NOUVEAU_VIRT_SLOT_SIZE * NOUVEAU_VIRT_SLOT_NUM)
#define NOUVEAU_VIRT_BATCH_SIZE 128ULL

#define nouveau_virt_hypercall(slot) \
	for ((slot) = nouveau_virt_acquire(); \
	     (slot); \
	     (slot) = nouveau_virt_release((slot)))

struct nouveau_virt_slot {
	union {
		u8   u8[NOUVEAU_VIRT_SLOT_SIZE / sizeof(u8) ];
		u16 u16[NOUVEAU_VIRT_SLOT_SIZE / sizeof(u16)];
		u32 u32[NOUVEAU_VIRT_SLOT_SIZE / sizeof(u32)];
		u64 u64[NOUVEAU_VIRT_SLOT_SIZE / sizeof(u64)];
	};
};

#endif
