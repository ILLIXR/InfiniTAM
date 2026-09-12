#pragma once

#include "ITMLib/Objects/Meshing/ITMMesh.h"

#include <array>
#include <cstdint>
#include <draco_illixr/mesh/mesh.h>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>

namespace ILLIXR {

struct voxel_block_dictionary_hash {
    size_t operator()(const std::array<int32_t, 3>& block) const {
        return (uint32_t(block[0]) * 73856093u) ^ (uint32_t(block[1]) * 19349669u) ^ (uint32_t(block[2]) * 83492791u);
    }
};

// Build each extraction chunk independently. Deduplication stays on the
// compression worker, after the chunk has been published.
inline std::unique_ptr<draco_illixr::Mesh> make_draco_mesh(const ITMLib::ITMMesh::Triangle* triangles, unsigned first_triangle,
                                                           unsigned num_faces) {
    using namespace draco_illixr;
    if (num_faces > std::numeric_limits<PointIndex::ValueType>::max() / 3) {
        return nullptr;
    }
    const unsigned num_vertices = num_faces * 3;
    auto           mesh         = std::make_unique<Mesh>();
    mesh->SetNumFaces(num_faces);
    mesh->set_num_points(num_vertices);

    GeometryAttribute position;
    position.Init(GeometryAttribute::POSITION, nullptr, 3, DT_FLOAT32, false, sizeof(float) * 3, 0);
    const int position_id = mesh->AddAttribute(position, true, num_vertices);
    if (position_id < 0) {
        return nullptr;
    }
    auto* positions    = mesh->attribute(position_id);
    auto  voxel_blocks = std::make_unique<PointAttribute>();
    voxel_blocks->Init(GeometryAttribute::GENERIC, 1, DT_INT32, false, num_faces);
    std::unordered_map<std::array<int32_t, 3>, int32_t, voxel_block_dictionary_hash> block_ids;
    std::vector<int32_t>                                                             dictionary;
    block_ids.reserve(256);
    dictionary.reserve(256 * 3);

    for (unsigned face = 0; face < num_faces; ++face) {
        const auto& triangle = triangles[first_triangle + face];
        const float p0[]     = {triangle.p0.x, triangle.p0.y, triangle.p0.z};
        const float p1[]     = {triangle.p1.x, triangle.p1.y, triangle.p1.z};
        const float p2[]     = {triangle.p2.x, triangle.p2.y, triangle.p2.z};
        positions->SetAttributeValue(AttributeValueIndex(face * 3), p0);
        positions->SetAttributeValue(AttributeValueIndex(face * 3 + 1), p1);
        positions->SetAttributeValue(AttributeValueIndex(face * 3 + 2), p2);
        // Preserve the winding used by the PLY handoff and ITMMesh::WriteOBJ.
        mesh->SetFace(FaceIndex(face), {PointIndex(face * 3 + 2), PointIndex(face * 3 + 1), PointIndex(face * 3)});
        const std::array<int32_t, 3> block{triangle.vb_info.x, triangle.vb_info.y, triangle.vb_info.z};
        const auto                   next_id  = static_cast<int32_t>(dictionary.size() / 3);
        const auto                   inserted = block_ids.try_emplace(block, next_id);
        if (inserted.second) {
            dictionary.insert(dictionary.end(), block.begin(), block.end());
        }
        voxel_blocks->SetAttributeValue(AttributeValueIndex(face), &inserted.first->second);
    }

    const int block_id = mesh->AddPerFaceAttribute(std::move(voxel_blocks));
    if (block_id < 0) {
        return nullptr;
    }
    auto metadata = std::make_unique<AttributeMetadata>();
    // IDs and exact coordinates belong to this chunk. Keep the dictionary in
    // its Draco payload so decoding never depends on a separate message.
    metadata->AddEntryString("attribute_name", "_VOXELBLOCK_ID");
    metadata->AddEntryInt("ada_block_dictionary_version", 1);
    if (!dictionary.empty()) {
        metadata->AddEntryIntArray("ada_block_dictionary", dictionary);
    }
    mesh->AddAttributeMetadata(block_id, std::move(metadata));
    return mesh;
}

} // namespace ILLIXR
