#pragma once

#include "ITMLib/Objects/Meshing/ITMMesh.h"

#include <cstdint>
#include <draco_illixr/mesh/mesh.h>
#include <limits>
#include <memory>

namespace ILLIXR {

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
    voxel_blocks->Init(GeometryAttribute::GENERIC, 3, DT_INT32, false, num_faces);

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
        const int32_t block[] = {triangle.vb_info.x, triangle.vb_info.y, triangle.vb_info.z};
        voxel_blocks->SetAttributeValue(AttributeValueIndex(face), block);
    }

    const int block_id = mesh->AddPerFaceAttribute(std::move(voxel_blocks));
    if (block_id < 0) {
        return nullptr;
    }
    auto metadata = std::make_unique<AttributeMetadata>();
    metadata->AddEntryString("attribute_name", "_VOXELBLOCK_INFO");
    mesh->AddAttributeMetadata(block_id, std::move(metadata));
    return mesh;
}

} // namespace ILLIXR
