// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Asher Elmquist
// =============================================================================
//
// Class to manage adding, changing, and removing godot scene nodes
//
// =============================================================================

// #include <iostream>
// #include <memory>

// godot includes
#include <core/math/quat.h>
#include <core/vector.h>
//#include <scene/3d/mesh_instance.h>
//#include <scene/3d/spatial.h>

// ChGodotIncludes
#include "chrono_godot/nodes/ChGdBody.h"

namespace chrono {
namespace gd {

ChGdBody::ChGdBody(std::shared_ptr<ChGdAssetManager> assetManager) {
    m_assetManager = assetManager;
    m_bodyNode = memnew(Spatial);
    m_bodyNode->set_name("BodyNode");
}
ChGdBody::~ChGdBody() {
    // for (auto obj : m_resourceList) {
    //     memdelete(obj);
    // }
    // m_resourceList.clear();
    //
    // memdelete(m_bodyNode);
}

void ChGdBody::CreateFromChrono(Spatial* parent_node, std::shared_ptr<ChBody> body) {
    parent_node->add_child(m_bodyNode);
    m_bodyRef = body;
    if (body->GetAssets().size() == 0)
        return;

    // position of the body
    const ChVector<> pos = body->GetFrame_REF_to_abs().GetPos();
    // rotation of the body
    ChQuaternion<> rot = body->GetFrame_REF_to_abs().GetRot();

    // iterate through the body's asset list
    for (auto asset : body->GetAssets()) {
        // check if the asset is a ChVisualization
        if (std::shared_ptr<ChVisualization> visual_asset = std::dynamic_pointer_cast<ChVisualization>(asset)) {
            ChVector<> center = visual_asset->Pos;
            // rotate asset pos into global frame
            center = rot.Rotate(center);
            // Get the local rotation of the asset
            ChQuaternion<> local_rot = visual_asset->Rot.Get_A_quaternion();
            // compute the final position of the geometry
            ChVector<> pos_final = pos + center;

            if (ChSphereShape* sphere_shape = dynamic_cast<ChSphereShape*>(asset.get())) {
                // save the radius of the sphere for object scaling
                double radius = sphere_shape->GetSphereGeometry().rad;

                // add a new sphere given by the asset manager - this will only
                // create one instance of the sphere mesh
                SphereMesh* mesh = m_assetManager->GetSphereMesh();
                Ref<SpatialMaterial> material{m_assetManager->GetColorMaterial({1.0, 0, 0})};

                // create the mesh node to attach the mesh and material
                MeshInstance* mesh_node = memnew(MeshInstance);
                m_resourceList.push_back(mesh_node);
                m_bodyNode->add_child(mesh_node);
                mesh_node->set_name("SphereMesh");
                mesh_node->set_mesh(mesh);
                mesh_node->set_surface_material(0, material);

                // set the transformation on the mesh node TODO: set transforms rather than rotations
                mesh_node->set_translation(GDVector(pos_final));
                mesh_node->set_rotation(GDQuat(rot * local_rot).get_euler_yxz());
                mesh_node->set_scale(Vector3(radius, radius, radius));

                // save the mesh and chgeometry pair for updates later
                m_geometries.push_back({mesh_node, sphere_shape});

            } else if (ChEllipsoidShape* ellipsoid_shape = dynamic_cast<ChEllipsoidShape*>(asset.get())) {
            } else if (ChBoxShape* box_shape = dynamic_cast<ChBoxShape*>(asset.get())) {
                ChVector<> size = box_shape->GetBoxGeometry().GetLengths();

                // add a new box given by the asset manager - this will only
                // create one instance of the box mesh
                CubeMesh* mesh = m_assetManager->GetCubeMesh();
                Ref<SpatialMaterial> material{m_assetManager->GetColorMaterial({0.0, 0.3, 0.3})};

                // create the mesh node to attach the mesh and material
                MeshInstance* mesh_node = memnew(MeshInstance);
                m_resourceList.push_back(mesh_node);
                m_bodyNode->add_child(mesh_node);
                mesh_node->set_name("CubeMesh");
                mesh_node->set_mesh(mesh);
                mesh_node->set_surface_material(0, material);

                // set the transformation on the mesh node TODO: set transforms rather than rotations
                mesh_node->set_translation(GDVector(pos_final));
                mesh_node->set_rotation(GDQuat(rot * local_rot).get_euler_yxz());
                mesh_node->set_scale(GDVector(size));

                // Transform t = GDTransform(rot * local_rot).translated(GDVector(pos_final));
                // mesh_node->set_transform(t);
                // mesh_node->set_scale(GDVector(size));

                // save the mesh and chgeometry pair for updates later
                m_geometries.push_back({mesh_node, box_shape});

            } else if (ChCylinderShape* cylinder_shape = dynamic_cast<ChCylinderShape*>(asset.get())) {
                double radius = cylinder_shape->GetCylinderGeometry().rad;
                double height =
                    (cylinder_shape->GetCylinderGeometry().p2 - cylinder_shape->GetCylinderGeometry().p1).Length();

                // add a new cylinder given by the asset manager - this will only
                // create one instance of the cylinder mesh
                CylinderMesh* mesh = m_assetManager->GetCylinderMesh();
                Ref<SpatialMaterial> material{m_assetManager->GetColorMaterial({0, 0, 0.8})};

                // create the mesh node to attach the mesh and material
                MeshInstance* mesh_node = memnew(MeshInstance);
                m_resourceList.push_back(mesh_node);
                m_bodyNode->add_child(mesh_node);
                mesh_node->set_name("CylinderMesh");
                mesh_node->set_mesh(mesh);
                mesh_node->set_surface_material(0, material);

                // set the transformation on the mesh node TODO: set transforms rather than rotations
                mesh_node->set_translation(GDVector(pos_final));
                mesh_node->set_rotation(GDQuat(rot * local_rot).get_euler_yxz());
                mesh_node->set_scale(Vector3(radius, height, radius));

                // save the mesh and chgeometry pair for updates later
                m_geometries.push_back({mesh_node, cylinder_shape});

            } else if (ChConeShape* cone_shape = dynamic_cast<ChConeShape*>(asset.get())) {
            } else if (ChRoundedBoxShape* shape = dynamic_cast<ChRoundedBoxShape*>(asset.get())) {
            } else if (ChCapsuleShape* capsule_shape = dynamic_cast<ChCapsuleShape*>(asset.get())) {
            } else if (ChTriangleMeshShape* trimesh_shape = dynamic_cast<ChTriangleMeshShape*>(asset.get())) {
            } else if (ChPathShape* path_shape = dynamic_cast<ChPathShape*>(asset.get())) {
            } else if (ChLineShape* line_shape = dynamic_cast<ChLineShape*>(asset.get())) {
            }
        }
        // check if the asset is a ChColorAsset
        else if (std::shared_ptr<ChColorAsset> visual_asset = std::dynamic_pointer_cast<ChColorAsset>(asset)) {
            // std::cout << "Asset was color\n";
        }

        // check if the asset is a ChTexture
        else if (std::shared_ptr<ChTexture> visual_asset = std::dynamic_pointer_cast<ChTexture>(asset)) {
            // std::cout << "Asset was texture\n";
        }
    }
}
void ChGdBody::UpdatePose() {
    const ChVector<> pos = m_bodyRef->GetFrame_REF_to_abs().GetPos();
    // rotation of the body
    ChQuaternion<> rot = m_bodyRef->GetFrame_REF_to_abs().GetRot();

    for (auto geometry : m_geometries) {
        ChVector<> center = geometry.ch_geo->Pos;
        // rotate asset pos into global frame
        center = rot.Rotate(center);
        // Get the local rotation of the asset
        ChQuaternion<> local_rot = geometry.ch_geo->Rot.Get_A_quaternion();
        ChVector<> pos_final = pos + center;

        // set the tranformation of the mesh node to be same as chrono TODO: set transforms rather than rotations
        geometry.godot_mesh->set_translation(GDVector(pos_final));
        geometry.godot_mesh->set_rotation(GDQuat(rot * local_rot).get_euler_yxz());
    }
}

}  // namespace gd
}  // namespace chrono
