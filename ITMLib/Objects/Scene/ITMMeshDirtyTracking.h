// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM
#pragma once

#include "ITMRepresentationAccess.h"

// A cube uses its origin and the seven corners in the positive directions.
// A changed sample can therefore affect negative neighboring owners only on
// the corresponding zero-coordinate face, edge, or corner of its block.
_CPU_AND_GPU_CODE_ inline unsigned int meshDirtyOwnersForVoxel(int x, int y, int z)
{
	unsigned int mask = 1;
	if (x == 0) mask |= mask << 1;
	if (y == 0) mask |= mask << 2;
	if (z == 0) mask |= mask << 4;
	return mask;
}

_CPU_AND_GPU_CODE_ inline bool meshOwnerNeedsUpdate(const ITMHashEntry &owner, const ITMHashEntry *hashTable)
{
	if (owner.mesh_dirty_mask & 1u) return true;
	for (int direction = 1; direction < 8; ++direction) {
		const Vector3i neighbor = owner.pos.toInt() +
			Vector3i(direction & 1, (direction >> 1) & 1, (direction >> 2) & 1);
		int index = hashIndex(neighbor);
		while (true) {
			const ITMHashEntry &entry = hashTable[index];
			if (entry.pos.toInt() == neighbor && entry.ptr >= 0) {
				if (entry.mesh_dirty_mask & (1u << direction)) return true;
				break;
			}
			if (entry.offset < 1) break;
			index = SDF_BUCKET_NUM + entry.offset - 1;
		}
	}
	return false;
}
