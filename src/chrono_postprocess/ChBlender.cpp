// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2023 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChModelFileShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChLineShape.h"
#include "chrono/assets/ChPathShape.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChGlyphs.h"
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChParticleCloud.h"

#include "chrono_postprocess/ChBlender.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace postprocess {

using namespace geometry;

ChBlender::ChBlender(ChSystem* system) : ChPostProcessBase(system) {
    base_path = "";
    pic_path = "anim";
    out_path = "output";
    pic_filename = "picture";
    out_script_filename = "exported";
    out_data_filename = "state";
    framenumber = 0;
    camera_add_default = false;
    camera_location = ChVector<>(0, 1.5, -2);
    camera_aim = ChVector<>(0, 0, 0);
    camera_up = ChVector<>(0, 1, 0);
    camera_angle = 30;
    camera_orthographic = false;
    camera_found_in_assets = false;
    def_light_location = ChVector<>(2, 3, -1);
    def_light_color = ChColor(1, 1, 1);
    def_light_cast_shadows = true;
    background = ChColor(1, 1, 1);
    picture_height = 768;
    picture_width = 1024;
    COGs_show = false;
    COGs_size = 0.04;
    frames_item_show = false;
    frames_item_size = 0.05;
    frames_asset_show = false;
    frames_asset_size = 0.03;
    frames_links_show = false;
    frames_links_size = 0.04;
    contacts_show = ContactSymbolType::VECTOR;
    contacts_maxsize = 0.1;
    contacts_vector_length_type = ContactSymbolVectorLength::PROPERTY;
    contacts_vector_scalelenght = 0.1;
    contacts_vector_length_prop = "norm";
    contacts_vector_width_type = ContactSymbolVectorWidth::CONSTANT;
    contacts_vector_scalewidth = 0.001;
    contacts_vector_width_prop = "";
    contacts_sphere_size_type = ContactSymbolSphereSize::CONSTANT;
    contacts_sphere_scalesize = 0.01;
    contacts_vector_width_prop = "";
    contacts_color_type = ContactSymbolColor::CONSTANT;
    contacts_color_constant = ChColor(1, 0, 0);
    contacts_color_prop = "";
    contacts_colormap_startscale = 0;
    contacts_colormap_endscale = 10;
    contacts_vector_tip = true;
    wireframe_thickness = 0.001;
    single_asset_file = true;

    SetBlenderUp_is_ChronoY();
}

void ChBlender::Add(std::shared_ptr<ChPhysicsItem> item) {
    m_items.insert(item);
}

void ChBlender::Remove(std::shared_ptr<ChPhysicsItem> item) {
    m_items.erase(item);
}

void ChBlender::AddAll() {
    for (const auto& body : mSystem->Get_bodylist()) {
        Add(body);
    }
    for (const auto& mesh : mSystem->Get_meshlist()) {
        Add(mesh);
    }
    for (const auto& ph : mSystem->Get_otherphysicslist()) {
        Add(ph);
    }
    for (const auto& link : mSystem->Get_linklist()) {
        Add(link);
    }
}

void ChBlender::RemoveAll() {
    for (auto& body : mSystem->Get_bodylist()) {
        Remove(body);
    }
    for (auto& mesh : mSystem->Get_meshlist()) {
        Remove(mesh);
    }
    for (auto& ph : mSystem->Get_otherphysicslist()) {
        Remove(ph);
    }
    for (auto& link : mSystem->Get_linklist()) {
        Remove(link);
    }
}

void ChBlender::SetCustomCommands(std::shared_ptr<ChPhysicsItem> item, const std::string& commands) {
    m_custom_commands[(size_t)item.get()] = commands;
}

void ChBlender::UpdateRenderList() {
    // Remove physics items from the render list if the only reference to them is in this list.
    for (auto item = m_items.begin(); item != m_items.end();) {
        if (item->use_count() == 1)
            item = m_items.erase(item);
        else
            ++item;
    }
}

std::string bl_replaceOnce(std::string result, const std::string& replaceWhat, const std::string& replaceWithWhat) {
    const std::string::size_type pos = result.find(replaceWhat);
    if (pos == std::string::npos)
        return result;
    result.replace(pos, replaceWhat.size(), replaceWithWhat);
    return result;
}

std::string bl_replaceAll(std::string result, const std::string& replaceWhat, const std::string& replaceWithWhat) {
    while (1) {
        const std::string::size_type pos = result.find(replaceWhat);
        if (pos == std::string::npos)
            break;
        result.replace(pos, replaceWhat.size(), replaceWithWhat);
    }
    return result;
}

void ChBlender::SetCamera(ChVector<> location, ChVector<> aim, double angle, bool ortho) {
    camera_add_default = true;
    camera_location = location;
    camera_aim = aim;
    camera_angle = angle;
    camera_orthographic = ortho;
}

void ChBlender::SetLight(ChVector<> location, ChColor color, bool cast_shadow) {
    def_light_location = location;
    def_light_color = color;
    def_light_cast_shadows = cast_shadow;
}

void ChBlender::SetShowCOGs(bool show, double msize) {
    COGs_show = show;
    if (show)
        COGs_size = msize;
}
void ChBlender::SetShowItemsFrames(bool show, double msize) {
    frames_item_show = show;
    if (show)
        frames_item_size = msize;
}
void ChBlender::SetShowAssetsFrames(bool show, double msize) {
    frames_asset_show = show;
    if (show)
        frames_asset_show = msize;
}
void ChBlender::SetShowLinksFrames(bool show, double msize) {
    frames_links_show = show;
    if (show)
        frames_links_size = msize;
}
void ChBlender::SetShowContactsOff() {
    this->contacts_show = ContactSymbolType::NONE;
}

void ChBlender::SetShowContactsVectors(
    ContactSymbolVectorLength length_type,
    double scale_length,  // if ContactSymbolVectorLength::CONSTANT means abs.length, otherwise is scaling factor for
                          // property
    const std::string scale_prop,  // needed if ContactSymbolVectorLength::PROPERTY, options: 'norm'. Otherwise ""
    ContactSymbolVectorWidth width_type,
    double scale_width,  // if ContactSymbolVectorWidth::CONSTANT means abs.width, otherwise is scaling factor for norm
                         // or property
    const std::string width_prop,  // needed if ContactSymbolVectorWidth::PROPERTY, options: "norm". Otherwise ""
    ContactSymbolColor color_type,
    ChColor const_color,           // if ContactSymbolColor::CONSTANT, otherwise not used
    const std::string color_prop,  // needed if ContactSymbolColor::PROPERTY, options: "norm". Otherwise ""
    double colormap_start,         // falsecolor start value, if not  ContactSymbolColor::CONSTANT,
    double colormap_end,           // falsecolor start value, if not  ContactSymbolColor::CONSTANT
    bool do_vector_tip) {
    contacts_show = ContactSymbolType::VECTOR;

    this->contacts_vector_length_type = length_type;
    this->contacts_vector_scalelenght = scale_length;
    this->contacts_vector_length_prop = scale_prop;
    this->contacts_vector_width_type = width_type;
    this->contacts_vector_scalewidth = scale_width;
    this->contacts_vector_width_prop = width_prop;
    this->contacts_vector_tip = do_vector_tip;
    this->contacts_color_type = color_type;
    this->contacts_color_prop = color_prop;
    this->contacts_color_constant = const_color;
    this->contacts_colormap_startscale = colormap_start;
    this->contacts_colormap_endscale = colormap_end;
}

void ChBlender::SetShowContactsSpheres(
    ContactSymbolSphereSize size_type,
    double scale_size,  // if ContactSymbolSphereSize::CONSTANT means abs.size, otherwise is scaling factor for norm or
                        // property
    ContactSymbolColor color_type,
    ChColor const_color,          // if ContactSymbolColor::CONSTANT, otherwise not used
    double colormap_start,        // falsecolor start value, if not  ContactSymbolColor::CONSTANT,
    double colormap_end,          // falsecolor start value, if not  ContactSymbolColor::CONSTANT
    const std::string size_prop,  // needed if ContactSymbolSphereSize::PROPERTY
    const std::string color_prop  // needed if ContactSymbolColor::PROPERTY)
) {
    contacts_show = ContactSymbolType::SPHERE;

    this->contacts_sphere_size_type = size_type;
    this->contacts_sphere_scalesize = scale_size;
    this->contacts_color_type = color_type;
    this->contacts_color_constant = const_color;
    this->contacts_colormap_startscale = colormap_start;
    this->contacts_colormap_endscale = colormap_end;
}

void ChBlender::ExportScript(const std::string& filename) {
    // Regenerate the list of objects that need Blender rendering
    UpdateRenderList();

    out_script_filename = filename;

    // Reset the maps that will be used to avoid saving multiple times a shared Chrono asset
    m_blender_shapes.clear();
    m_blender_materials.clear();
    m_blender_frame_shapes.clear();
    m_blender_frame_materials.clear();
    m_blender_cameras.clear();

    // Create directories
    if (base_path != "") {
        if (!filesystem::create_directory(filesystem::path(base_path))) {
            std::cout << "Error creating base directory \"" << base_path << "\" for the Blender files." << std::endl;
            return;
        }
        base_path = base_path + "/";
    }
    filesystem::create_directory(filesystem::path(base_path + pic_path));
    filesystem::create_directory(filesystem::path(base_path + out_path));

    // Generate the xxx.assets.py script (initial assets, it will be populated later by
    // appending assets as they enter the exporter, only once if shared, using ExportAssets() )

    std::string assets_filename = out_script_filename + ".assets.py";
    {
        ChStreamOutAsciiFile assets_file((base_path + assets_filename).c_str());
        assets_file
            << "# File containing meshes and objects for rendering Blender scenes, shared through all frames.\n";
        assets_file << "# This file must be imported in Blender using File/Import/chrono import menu, \n";
        assets_file << "# that is available in Blender if you installed the chrono_import.py add-on.\n\n";

        // Write Blender custom code
        if (custom_script.size() > 0) {
            assets_file << "# Custom user-added script: \n\n";
            assets_file << custom_script;
            assets_file << "\n\n";
        }

        // write global settings for symbols
        assets_file << "chrono_view_asset_csys =  " << (this->frames_asset_show ? "True" : "False") << "\n"
                    << "chrono_view_asset_csys_size = " << this->frames_asset_size << "\n"
                    << "chrono_view_item_csys =  " << (this->frames_item_show ? "True" : "False") << "\n"
                    << "chrono_view_item_csys_size = " << this->frames_item_size << "\n"
                    << "chrono_view_link_csys =  " << (this->frames_links_show ? "True" : "False") << "\n"
                    << "chrono_view_link_csys_size = " << this->frames_links_size << "\n"
                    << "\n";
        std::string abspath_pic_output =
            filesystem::path(base_path + pic_path + "/" + pic_filename + "_######").make_absolute().str();
        std::replace(abspath_pic_output.begin(), abspath_pic_output.end(), '\\', '/');
        assets_file << "bpy.context.scene.render.filepath = '" << abspath_pic_output << "'\n"
                    << "bpy.context.scene.render.resolution_x = " << this->picture_width << "\n"
                    << "bpy.context.scene.render.resolution_y = " << this->picture_height << "\n"
                    << "\n\n";

        // write default camera, if defined
        if (camera_add_default) {
            std::string cameraname("default_camera");
            assets_file << "bpy.ops.object.camera_add(enter_editmode=False, location=(0, 0, 0), scale=(1, 1, 1)) \n"
                        << "new_object = bpy.context.object \n"
                        << "new_object.name= '" << cameraname << "' \n"
                        << "new_object.data.lens_unit='FOV' \n";
            if (this->camera_orthographic) {
                assets_file << "new_object.data.type='ORTHO'\n";
                assets_file << "new_object.data.ortho_scale="
                            << double((camera_location - camera_aim).Length() *
                                      tan(0.5 * camera_angle * chrono::CH_C_DEG_TO_RAD))
                            << "\n";
            } else {
                assets_file << "new_object.data.type='PERSP'\n";
                assets_file << "new_object.data.angle=" << camera_angle * chrono::CH_C_DEG_TO_RAD << "\n";
            }
            assets_file << "chrono_cameras.objects.link(new_object) \n"
                        << "bpy.context.scene.collection.objects.unlink(new_object) \n\n";
            ChVector<> cdirzm = camera_aim - camera_location;
            ChVector<> cdirx = Vcross(cdirzm, VECT_Y);
            ChVector<> cdiry = Vcross(cdirx, cdirzm);
            ChMatrix33<> cmrot;
            cmrot.Set_A_axis(cdirx.GetNormalized(), cdiry.GetNormalized(), -cdirzm.GetNormalized());
            ChFrame<> cframeloc(camera_location, cmrot);
            ChFrame<> cframeabs = cframeloc >> blender_frame;
            assets_file << "update_camera_coordinates(";
            assets_file << "'" << cameraname << "',";
            assets_file << "(" << cframeabs.GetPos().x() << "," << cframeabs.GetPos().y() << ","
                        << cframeabs.GetPos().z() << "),";
            assets_file << "(" << cframeabs.GetRot().e0() << "," << cframeabs.GetRot().e1() << ","
                        << cframeabs.GetRot().e2() << "," << cframeabs.GetRot().e3() << ")";
            assets_file << ")\n";
            assets_file << "bpy.context.scene.camera = new_object\n\n";
        }
    }

    // This forces saving the non-mutable assets in the assets_file, at initial state.
    ExportData();

    this->framenumber--;  // so that it starts again from 0 when calling ExportData() in the simulation while() loop:
}

void ChBlender::ExportAssets(ChStreamOutAsciiFile& assets_file, ChStreamOutAsciiFile& state_file) {
    for (const auto& item : m_items) {
        ExportShapes(assets_file, state_file, item);
    }
}

// Write geometries and materials in the Blender assets script for all physics items with a visual model
void ChBlender::ExportShapes(ChStreamOutAsciiFile& assets_file,
                             ChStreamOutAsciiFile& state_file,
                             std::shared_ptr<ChPhysicsItem> item) {
    // Nothing to do if the item does not have a visual model
    if (!item->GetVisualModel())
        return;

    // In a second pass, export shape geometry
    for (const auto& shape_instance : item->GetVisualModel()->GetShapes()) {
        const auto& shape = shape_instance.first;

        ChStreamOutAsciiFile* mfile;
        std::unordered_map<size_t, std::shared_ptr<ChVisualShape>>* m_shapes;
        std::unordered_map<size_t, std::shared_ptr<ChVisualMaterial>>* m_materials;
        std::string collection;
        bool per_frame;

        if (shape->IsMutable() || !this->single_asset_file) {
            mfile = &state_file;
            m_shapes = &this->m_blender_frame_shapes;
            m_materials = &this->m_blender_frame_materials;
            collection = "chrono_frame_assets";
            per_frame = true;
        } else {
            mfile = &assets_file;
            m_shapes = &this->m_blender_shapes;
            m_materials = &this->m_blender_materials;
            collection = "chrono_assets";
            per_frame = false;
        }

        // const auto& shape_frame = shape_instance.second; // not needed, shape frame will be set later via
        // make_chrono_object_assetlist in py files

        // Export shape materials
        ExportMaterials(*mfile, *m_materials, shape->GetMaterials(), per_frame, shape);

        std::string shapename("shape_" + std::to_string((size_t)shape.get()));

        // Do nothing if the shape was already processed (because it is shared)
        // Otherwise, add the shape to the cache list and process it
        if (m_shapes->find((size_t)shape.get()) != m_shapes->end())
            continue;

        if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
            *mfile << "bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=1.0, calc_uvs=True) \n"
                   << "new_object = bpy.context.object \n"
                   << "new_object.name = '" << shapename << "' \n"
                   << "new_object.data.polygons.foreach_set('use_smooth', [True] * len(new_object.data.polygons)) \n"
                   << "new_object.data.materials.append(None) \n"
                   << collection << ".objects.link(new_object) \n"
                   << "bpy.context.scene.collection.objects.unlink(new_object)\n\n";
            // radius will be set later in ExportItemState to avoid having n meshes per each radius
            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
            *mfile << "bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=1.0, calc_uvs=True) \n"
                   << "new_object = bpy.context.object \n"
                   << "new_object.name = '" << shapename << "' \n"
                   << "new_object.data.polygons.foreach_set('use_smooth', [True] * len(new_object.data.polygons)) \n"
                   << "new_object.data.materials.append(None) \n"
                   << collection << ".objects.link(new_object) \n"
                   << "bpy.context.scene.collection.objects.unlink(new_object)\n\n";
            // radii will be set later in ExportItemState to avoid having n meshes per each radius
            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
            *mfile << "bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=1.0, depth=1.0, calc_uvs=True) \n"
                   << "new_object = bpy.context.object \n"
                   << "new_object.name = '" << shapename << "' \n"
                   << "new_object.data.materials.append(None) \n"
                   << collection << ".objects.link(new_object) \n"
                   << "bpy.context.scene.collection.objects.unlink(new_object)\n"
                   << "with bpy.context.temp_override(selected_editable_objects=[new_object]):\n"
                   << "    bpy.ops.object.shade_smooth(use_auto_smooth=True) \n\n";
            // radius and height will be set later in ExportItemState to avoid having n meshes per each radius
            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto cone = std::dynamic_pointer_cast<ChConeShape>(shape)) {
            *mfile
                << "bpy.ops.mesh.primitive_cone_add(vertices=32, radius1=1.0, radius2=0, depth=1.0, calc_uvs=True) \n"
                << "new_object = bpy.context.object \n"
                << "new_object.name = '" << shapename << "' \n"
                << "new_object.data.materials.append(None) \n"
                << collection << ".objects.link(new_object) \n"
                << "bpy.context.scene.collection.objects.unlink(new_object)\n"
                << "with bpy.context.temp_override(selected_editable_objects=[new_object]):\n"
                << "    bpy.ops.object.shade_smooth(use_auto_smooth=True) \n\n";
            // radius etc will be set later in ExportItemState to avoid having n meshes per each radius
            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
            *mfile << "bpy.ops.mesh.primitive_cube_add(size=1,calc_uvs=True) \n"
                   << "new_object = bpy.context.object \n"
                   << "new_object.name = '" << shapename << "' \n"
                   << "new_object.data.materials.append(None) \n"
                   << collection << ".objects.link(new_object) \n"
                   << "bpy.context.scene.collection.objects.unlink(new_object)\n\n";
            // xyz sizes will be set later in ExportItemState to avoid having n meshes
            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto obj_shape = std::dynamic_pointer_cast<ChModelFileShape>(shape)) {
            std::string abspath_obj = filesystem::path(obj_shape->GetFilename()).make_absolute().str();
            std::replace(abspath_obj.begin(), abspath_obj.end(), '\\', '/');
            *mfile << "try: \n"
                   << "    bpy.context.view_layer.active_layer_collection = bpy.context.view_layer.layer_collection \n"
                   << "    file_loc = '" << abspath_obj.c_str() << "'\n"
                   << "    imported_object = bpy.ops.import_scene.obj(filepath=file_loc) \n"
                   << "    new_object = bpy.context.selected_objects[-1] \n"
                   << "    new_object.name= '" << shapename << "' \n"
                   << "    new_object.data.materials.append(None) \n"
                   << "    " << collection << ".objects.link(new_object) \n"
                   << "    bpy.context.scene.collection.objects.unlink(new_object) \n"
                   << "except: \n"
                   << "    print('Cannot load .OBJ file: ', file_loc) \n\n";
            // if it fails to load (ex.: missing file, bad obj, etc) it prints error to console
            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto mesh_shape = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
            std::shared_ptr<ChTriangleMeshConnected> mesh = mesh_shape->GetMesh();
            bool wireframe = mesh_shape->IsWireframe();

            *mfile << "verts = [ \n";
            for (unsigned int iv = 0; iv < mesh->m_vertices.size(); iv++) {
                *mfile << "(" << mesh->m_vertices[iv].x() << "," << mesh->m_vertices[iv].y() << ","
                       << mesh->m_vertices[iv].z() << "),\n";
            }
            *mfile << "] \n";

            *mfile << "faces = [ \n";
            for (unsigned int ip = 0; ip < mesh->m_face_v_indices.size(); ip++) {
                *mfile << "(" << mesh->m_face_v_indices[ip].x() << "," << mesh->m_face_v_indices[ip].y() << ","
                       << mesh->m_face_v_indices[ip].z() << "),\n";
            }
            *mfile << "] \n";

            *mfile << "edges = [] \n";

            *mfile << "new_mesh = bpy.data.meshes.new('mesh_mesh') \n";
            *mfile << "new_mesh.from_pydata(verts, edges, faces) \n";

            if (mesh->m_face_uv_indices.size() == mesh->m_face_v_indices.size()) {
                // UV per each triangle (corner):
                *mfile << "uvs = [ \n";
                for (unsigned int it = 0; it < mesh->m_face_uv_indices.size(); it++) {
                    *mfile << mesh->m_UV[mesh->m_face_uv_indices[it].x()].x() << ","
                           << mesh->m_UV[mesh->m_face_uv_indices[it].x()].y() << ","
                           << mesh->m_UV[mesh->m_face_uv_indices[it].y()].x() << ","
                           << mesh->m_UV[mesh->m_face_uv_indices[it].y()].y() << ","
                           << mesh->m_UV[mesh->m_face_uv_indices[it].z()].x() << ","
                           << mesh->m_UV[mesh->m_face_uv_indices[it].z()].y() << ","
                           << "\n";
                }
                *mfile << "] \n";
                *mfile << "uv_layer = new_mesh.uv_layers.new(name = 'UVMap') \n";
                *mfile << "uv_layer.data.foreach_set('uv', uvs) \n";
            } else if (mesh->m_face_uv_indices.size() == 0 && mesh->m_UV.size() == mesh->m_vertices.size()) {
                // UV per each vertex (no m_face_uv_indices, assume indexing in m_face_v_indices used per vertex
                // coordinates )
                *mfile << "uvs = [ \n";
                for (unsigned int it = 0; it < mesh->m_UV.size(); it += 3) {
                    *mfile << mesh->m_UV[mesh->m_face_v_indices[it].x()].x() << ","
                           << mesh->m_UV[mesh->m_face_v_indices[it].x()].y() << ","
                           << mesh->m_UV[mesh->m_face_v_indices[it].y()].x() << ","
                           << mesh->m_UV[mesh->m_face_v_indices[it].y()].y() << ","
                           << mesh->m_UV[mesh->m_face_v_indices[it].z()].x() << ","
                           << mesh->m_UV[mesh->m_face_v_indices[it].z()].y() << ","
                           << "\n";
                }
                *mfile << "] \n";
                *mfile << "uv_layer = new_mesh.uv_layers.new(name = 'UVMap') \n";
                *mfile << "uv_layer.data.foreach_set('uv', uvs) \n";
            }

            if (mesh->m_face_mat_indices.size() == mesh->getNumTriangles()) {
                *mfile << "mat_indices = [ \n";
                for (unsigned int ip = 0; ip < mesh->m_face_mat_indices.size(); ip++) {
                    *mfile << mesh->m_face_mat_indices[ip] << ",\n";
                }
                *mfile << "] \n";
                *mfile << "for i, myface in enumerate(new_mesh.polygons): \n";
                *mfile << "     myface.material_index = mat_indices[i] \n";
            }

            *mfile << "new_mesh.update() \n"
                   << "new_object = bpy.data.objects.new('mesh_object', new_mesh) \n"
                   << "new_object.data.polygons.foreach_set('use_smooth', [True] * len(new_object.data.polygons)) \n"
                   << "new_object.name= '" << shapename << "' \n"
                   << "new_object.data.materials.append(None) \n"
                   << "modifier = new_object.modifiers.new(name='edgesplit', type='EDGE_SPLIT') \n"
                   << "#modifier.split_angle = 1.0 \n";
            if (wireframe) {
                *mfile << "modifier = new_object.modifiers.new(name='wireframe', type='WIREFRAME') \n";
                *mfile << "modifier.thickness = " << wireframe_thickness << "\n";
            }
            *mfile << collection << ".objects.link(new_object) \n\n";

            if (mesh->m_colors.size() == mesh->m_vertices.size() || mesh->getPropertiesPerVertex().size() ||
                mesh->getPropertiesPerFace().size()) {
                *mfile << "meshsetting = setup_meshsetting(new_object)\n";
            }

            if (mesh->m_colors.size() == mesh->m_vertices.size()) {
                *mfile << "colors = [ \n";
                for (unsigned int iv = 0; iv < mesh->m_colors.size(); iv++) {
                    *mfile << "(" << mesh->m_colors[iv].R << "," << mesh->m_colors[iv].G << "," << mesh->m_colors[iv].B
                           << "),\n";
                }
                *mfile << "] \n";
                *mfile << "add_mesh_data_vectors(new_object, colors, 'chrono_color', mdomain='POINT') \n";

                *mfile << "property = setup_property_color(meshsetting, 'chrono_color', matname='mat_"
                       << std::to_string((size_t)shape.get()).c_str() << "_col')\n";
                *mfile << "mat = update_meshsetting_color_material(new_object,meshsetting, 'chrono_color')\n";
            }

            for (auto mprop : mesh->getPropertiesPerVertex()) {
                if (auto mprop_scalar = dynamic_cast<ChPropertyT<double>*>(mprop)) {
                    *mfile << "data_scalars = [ \n";
                    for (unsigned int iv = 0; iv < mprop_scalar->data.size(); iv++) {
                        *mfile << mprop_scalar->data[iv] << ",\n";
                    }
                    *mfile << "] \n";
                    *mfile << "add_mesh_data_floats(new_object, data_scalars, '" << mprop_scalar->name
                           << "', mdomain='POINT') \n";

                    *mfile << "property = setup_property_scalar(meshsetting, '" << mprop_scalar->name.c_str()
                           << "',min=" << mprop_scalar->min << ", max=" << mprop_scalar->max << ", matname='mat_"
                           << std::to_string((size_t)shape.get()).c_str() << "_" << mprop_scalar->name.c_str()
                           << "')\n";
                    *mfile << "mat = update_meshsetting_falsecolor_material(new_object,meshsetting, '"
                           << mprop_scalar->name.c_str() << "')\n";
                }
                if (auto mprop_vectors = dynamic_cast<ChPropertyT<ChVector<>>*>(mprop)) {
                    *mfile << "vectors = [ \n";
                    for (unsigned int iv = 0; iv < mprop_vectors->data.size(); iv++) {
                        *mfile << "(" << mprop_vectors->data[iv].x() << "," << mprop_vectors->data[iv].y() << ","
                               << mprop_vectors->data[iv].z() << "),\n";
                    }
                    *mfile << "] \n";
                    *mfile << "add_mesh_data_vectors(new_object, vectors, '" << mprop_vectors->name
                           << "', mdomain='POINT') \n";

                    *mfile << "property = setup_property_vector(meshsetting, '" << mprop_vectors->name.c_str()
                           << "',min=" << mprop_vectors->min << ", max=" << mprop_vectors->max << ", matname='mat_"
                           << std::to_string((size_t)shape.get()).c_str() << "_" << mprop_vectors->name.c_str()
                           << "')\n";
                    *mfile << "mat = update_meshsetting_falsecolor_material(new_object,meshsetting, '"
                           << mprop_vectors->name.c_str() << "')\n";
                }
            }
            for (auto mprop : mesh->getPropertiesPerFace()) {
                if (auto mprop_scalar = dynamic_cast<ChPropertyT<double>*>(mprop)) {
                    *mfile << "data_scalars = [ \n";
                    for (unsigned int iv = 0; iv < mprop_scalar->data.size(); iv++) {
                        *mfile << mprop_scalar->data[iv] << ",\n";
                    }
                    *mfile << "] \n";
                    *mfile << "add_mesh_data_floats(new_object, data_scalars, '" << mprop_scalar->name
                           << "', mdomain='POINT') \n";

                    *mfile << "property = setup_property_scalar(meshsetting, '" << mprop_scalar->name.c_str()
                           << "',min=" << mprop_scalar->min << ", max=" << mprop_scalar->max << ", matname='mat_"
                           << std::to_string((size_t)shape.get()).c_str() << "_" << mprop_scalar->name.c_str()
                           << "')\n";
                    *mfile << "mat = update_meshsetting_falsecolor_material(new_object,meshsetting, '"
                           << mprop_scalar->name.c_str() << "')\n";
                }
                if (auto mprop_vectors = dynamic_cast<ChPropertyT<ChVector<>>*>(mprop)) {
                    *mfile << "vectors = [ \n";
                    for (unsigned int iv = 0; iv < mprop_vectors->data.size(); iv++) {
                        *mfile << "(" << mprop_vectors->data[iv].x() << "," << mprop_vectors->data[iv].y() << ","
                               << mprop_vectors->data[iv].z() << "),\n";
                    }
                    *mfile << "] \n";
                    *mfile << "add_mesh_data_vectors(new_object, vectors, '" << mprop_vectors->name
                           << "', mdomain='FACE') \n";

                    *mfile << "property = setup_property_vector(meshsetting, '" << mprop_vectors->name.c_str()
                           << "',min=" << mprop_vectors->min << ", max=" << mprop_vectors->max << ", matname='mat_"
                           << std::to_string((size_t)shape.get()).c_str() << "_" << mprop_vectors->name.c_str()
                           << "')\n";
                    *mfile << "mat = update_meshsetting_falsecolor_material(new_object,meshsetting, '"
                           << mprop_vectors->name.c_str() << "')\n";
                }
            }
            *mfile << "\n";

            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto glyph_shape = std::dynamic_pointer_cast<ChGlyphs>(shape)) {
            *mfile << "points_" << shapename << " = [ \n";
            for (unsigned int iv = 0; iv < glyph_shape->points.size(); iv++) {
                *mfile << "(" << glyph_shape->points[iv].x() << "," << glyph_shape->points[iv].y() << ","
                       << glyph_shape->points[iv].z() << "),\n";
            }
            *mfile << "] \n";

            state_file << "glyphsetting = setup_glyph_setting('" << shapename << "',\n";
            if (glyph_shape->glyph_width_type == ChGlyphs::eCh_GlyphWidth::CONSTANT) {
                state_file << "\twidth_type='CONST', width_scale=" << glyph_shape->glyph_scalewidth
                           << ", \n";  // constant width
            }
            if (glyph_shape->glyph_width_type == ChGlyphs::eCh_GlyphWidth::PROPERTY) {
                auto mprop = std::find_if(std::begin(glyph_shape->m_properties), std::end(glyph_shape->m_properties),
                                          [&](auto const& p) { return p->name == glyph_shape->glyph_width_prop; });
                int myprop_id =
                    (mprop != glyph_shape->m_properties.end()) ? mprop - glyph_shape->m_properties.begin() : 0;
                state_file << "\twidth_type='PROPERTY', property_index_width=" << myprop_id
                           << ", width_scale=" << glyph_shape->glyph_scalewidth
                           << ",\n";  // will use 1st property, the 'F', for width
            }
            if (glyph_shape->glyph_color_type == ChGlyphs::eCh_GlyphColor::CONSTANT) {
                state_file << "\tcolor_type='CONST', const_color=(" << glyph_shape->glyph_color_constant.R << ","
                           << glyph_shape->glyph_color_constant.G << "," << glyph_shape->glyph_color_constant.B
                           << "), \n";  // constant color
            }
            if (glyph_shape->glyph_color_type == ChGlyphs::eCh_GlyphColor::PROPERTY) {
                auto mprop = std::find_if(std::begin(glyph_shape->m_properties), std::end(glyph_shape->m_properties),
                                          [&](auto const& p) { return p->name == glyph_shape->glyph_color_prop; });
                int myprop_id =
                    (mprop != glyph_shape->m_properties.end()) ? mprop - glyph_shape->m_properties.begin() : 0;
                state_file << "\tcolor_type='PROPERTY', property_index_color=" << myprop_id << ", \n";
            }
            if (glyph_shape->GetDrawMode() == ChGlyphs::GLYPH_POINT) {
                state_file << "\tglyph_type = 'POINT', \n";
            }
            if (glyph_shape->GetDrawMode() == ChGlyphs::GLYPH_VECTOR) {
                state_file << "\tglyph_type = 'VECTOR', \n";
                state_file << "\tdir_type='PROPERTY', property_index_dir=0, \n";
                state_file << "\tproperty_index_basis=0, \n";
                if (glyph_shape->glyph_length_type == ChGlyphs::eCh_GlyphLength::CONSTANT) {
                    state_file << "\tlength_type='CONST', length_scale=" << glyph_shape->glyph_scalelenght
                               << ", \n";  // constant length
                }
                if (glyph_shape->glyph_length_type == ChGlyphs::eCh_GlyphLength::PROPERTY) {
                    auto mprop =
                        std::find_if(std::begin(glyph_shape->m_properties), std::end(glyph_shape->m_properties),
                                     [&](auto const& p) { return p->name == glyph_shape->glyph_length_prop; });
                    int myprop_id =
                        (mprop != glyph_shape->m_properties.end()) ? mprop - glyph_shape->m_properties.begin() : 0;
                    state_file << "\tlength_type='PROPERTY', property_index_length=" << myprop_id
                               << ", length_scale=" << glyph_shape->glyph_scalelenght << ",\n";
                }
                if (glyph_shape->vector_tip)
                    state_file << "\tdo_tip=True,\n";
                else
                    state_file << "\tdo_tip=False,\n";
            }
            if (glyph_shape->GetDrawMode() == ChGlyphs::GLYPH_COORDSYS) {
                state_file << "\tglyph_type = 'COORDSYS', \n";
                if (glyph_shape->glyph_basis_type == ChGlyphs::eCh_GlyphBasis::CONSTANT) {
                    state_file << "\tbasis_type='CONST', const_basis=(" << glyph_shape->glyph_basis_constant.e0() << ","
                               << glyph_shape->glyph_basis_constant.e1() << ","
                               << glyph_shape->glyph_basis_constant.e2() << ","
                               << glyph_shape->glyph_basis_constant.e3() << "), \n";  // constant basis
                }
                if (glyph_shape->glyph_basis_type == ChGlyphs::eCh_GlyphBasis::PROPERTY) {
                    auto mprop =
                        std::find_if(std::begin(glyph_shape->m_properties), std::end(glyph_shape->m_properties),
                                     [&](auto const& p) { return p->name == glyph_shape->glyph_basis_prop; });
                    int myprop_id =
                        (mprop != glyph_shape->m_properties.end()) ? mprop - glyph_shape->m_properties.begin() : 0;
                    state_file << "\tbasis_type='PROPERTY', property_index_basis=" << myprop_id << ", \n";
                }
            }
            if (glyph_shape->GetDrawMode() == ChGlyphs::GLYPH_TENSOR) {
                state_file << "\tglyph_type = 'TENSOR', \n";
                if (glyph_shape->glyph_basis_type == ChGlyphs::eCh_GlyphBasis::CONSTANT) {
                    state_file << "\tbasis_type='CONST', const_basis=(" << glyph_shape->glyph_basis_constant.e0() << ","
                               << glyph_shape->glyph_basis_constant.e1() << ","
                               << glyph_shape->glyph_basis_constant.e2() << ","
                               << glyph_shape->glyph_basis_constant.e3() << "), \n";  // constant basis
                }
                if (glyph_shape->glyph_basis_type == ChGlyphs::eCh_GlyphBasis::PROPERTY) {
                    auto mprop =
                        std::find_if(std::begin(glyph_shape->m_properties), std::end(glyph_shape->m_properties),
                                     [&](auto const& p) { return p->name == glyph_shape->glyph_basis_prop; });
                    int myprop_id =
                        (mprop != glyph_shape->m_properties.end()) ? mprop - glyph_shape->m_properties.begin() : 0;
                    state_file << "\tbasis_type='PROPERTY', property_index_basis=" << myprop_id << ", \n";
                }
                if (glyph_shape->glyph_eigenvalues_type == ChGlyphs::eCh_GlyphEigenvalues::CONSTANT) {
                    state_file << "\teigenvalues_type='CONST', const_eigenvalues=("
                               << glyph_shape->glyph_eigenvalue_constant.x() << ","
                               << glyph_shape->glyph_eigenvalue_constant.y() << ","
                               << glyph_shape->glyph_eigenvalue_constant.z() << "), \n";  // constant basis
                }
                if (glyph_shape->glyph_eigenvalues_type == ChGlyphs::eCh_GlyphEigenvalues::PROPERTY) {
                    auto mprop =
                        std::find_if(std::begin(glyph_shape->m_properties), std::end(glyph_shape->m_properties),
                                     [&](auto const& p) { return p->name == glyph_shape->glyph_eigenvalues_prop; });
                    int myprop_id =
                        (mprop != glyph_shape->m_properties.end()) ? mprop - glyph_shape->m_properties.begin() : 0;
                    state_file << "\teigenvalues_type='PROPERTY', property_index_eigenvalues=" << myprop_id << ",\n";
                }
            }
            state_file << "\t)\n";

            for (auto mprop : glyph_shape->getProperties()) {
                if (auto mprop_scalar = dynamic_cast<ChPropertyT<double>*>(mprop)) {
                    *mfile << "data_" << mprop_scalar->name.c_str() << " = [ \n";
                    for (unsigned int iv = 0; iv < mprop_scalar->data.size(); iv++) {
                        *mfile << mprop_scalar->data[iv] << ",\n";
                    }
                    *mfile << "] \n";
                    *mfile << "property = setup_property_scalar(glyphsetting, '" << mprop_scalar->name.c_str()
                           << "',min=" << mprop_scalar->min << ", max=" << mprop_scalar->max << ", matname='mat_"
                           << std::to_string((size_t)shape.get()).c_str() << "_" << mprop_scalar->name.c_str()
                           << "', per_instance=True)\n";
                }
                if (auto mprop_vectors = dynamic_cast<ChPropertyT<ChVector<>>*>(mprop)) {
                    *mfile << "data_" << mprop_vectors->name.c_str() << " = [ \n";
                    for (unsigned int iv = 0; iv < mprop_vectors->data.size(); iv++) {
                        *mfile << "(" << mprop_vectors->data[iv].x() << "," << mprop_vectors->data[iv].y() << ","
                               << mprop_vectors->data[iv].z() << "),\n";
                    }
                    *mfile << "] \n";
                    *mfile << "property = setup_property_vector(glyphsetting, '" << mprop_vectors->name.c_str()
                           << "',min=" << mprop_vectors->min << ", max=" << mprop_vectors->max << ", matname='mat_"
                           << std::to_string((size_t)shape.get()).c_str() << "_" << mprop_vectors->name.c_str()
                           << "', per_instance=True)\n";
                }
                if (auto mprop_rots = dynamic_cast<ChPropertyT<ChQuaternion<>>*>(mprop)) {
                    *mfile << "data_" << mprop_rots->name.c_str() << " = [ \n";
                    for (unsigned int iv = 0; iv < mprop_rots->data.size(); iv++) {
                        *mfile << "(" << mprop_rots->data[iv].e0() << "," << mprop_rots->data[iv].e1() << ","
                               << mprop_rots->data[iv].e2() << "," << mprop_rots->data[iv].e3() << "),\n";
                    }
                    *mfile << "] \n";
                    *mfile << "property = setup_property_quaternion(glyphsetting, '" << mprop_rots->name.c_str()
                           << "',min=" << mprop_rots->min << ", max=" << mprop_rots->max << ", matname='mat_"
                           << std::to_string((size_t)shape.get()).c_str() << "_" << mprop_rots->name.c_str()
                           << "', per_instance=True)\n";
                }
                if (auto mprop_cols = dynamic_cast<ChPropertyT<ChColor>*>(mprop)) {
                    *mfile << "data_" << mprop_cols->name.c_str() << " = [ \n";
                    for (unsigned int iv = 0; iv < mprop_cols->data.size(); iv++) {
                        *mfile << "(" << mprop_cols->data[iv].R << "," << mprop_cols->data[iv].G << ","
                               << mprop_cols->data[iv].B << "),\n";
                    }
                    *mfile << "] \n";
                    *mfile << "property = setup_property_color(glyphsetting, '" << mprop_cols->name.c_str()
                           << "', matname='mat_" << std::to_string((size_t)shape.get()).c_str() << "_"
                           << mprop_cols->name.c_str() << "', per_instance=True)\n";
                }
            }
            state_file << "new_objects = update_make_glyphs(glyphsetting, '" << shapename << "',";
            state_file << "(" << blender_frame.GetPos().x() << "," << blender_frame.GetPos().y() << ","
                       << blender_frame.GetPos().z() << "),";
            state_file << "(" << blender_frame.GetRot().e0() << "," << blender_frame.GetRot().e1() << ","
                       << blender_frame.GetRot().e2() << "," << blender_frame.GetRot().e3() << "), \n";
            state_file << "  points_" << shapename << ",\n";
            state_file << "  list_attributes=[ \n";
            for (auto mprop : glyph_shape->getProperties()) {
                state_file << "   ['" << mprop->name.c_str() << "', data_" << mprop->name.c_str() << "], \n";
            }
            state_file << "  ], \n";
            state_file << ") \n";

            m_shapes->insert({(size_t)shape.get(), shape});
        }

        if (auto line_shape = std::dynamic_pointer_cast<ChLineShape>(shape)) {
            *mfile << "create_chrono_path('" << shapename << "',\n";
            *mfile << "[ \n";
            for (int i = 0; i < line_shape->GetNumRenderPoints(); ++i) {
                ChVector<> pt;
                line_shape->GetLineGeometry()->Evaluate(pt, (double)i / (double)(line_shape->GetNumRenderPoints() - 1));
                *mfile << "(" << pt.x() << "," << pt.y() << "," << pt.z() << "),\n";
            }
            *mfile << "],\n";
            *mfile << "(" << line_shape->GetColor().R << "," << line_shape->GetColor().G << ","
                   << line_shape->GetColor().B << ",1),\n";
            *mfile << line_shape->GetThickness() << ",\n";
            if (per_frame)
                *mfile << "chrono_frame_materials"
                       << ",\n";
            else
                *mfile << "chrono_materials"
                       << ",\n";
            *mfile << collection << "\n";
            *mfile << ")\n\n";

            m_shapes->insert({(size_t)shape.get(), shape});
        }
        if (auto line_shape = std::dynamic_pointer_cast<ChPathShape>(shape)) {
            *mfile << "create_chrono_path('" << shapename << "',\n";
            *mfile << "[ \n";
            for (int i = 0; i < line_shape->GetNumRenderPoints(); ++i) {
                ChVector<> pt;
                line_shape->GetPathGeometry()->Evaluate(pt, (double)i / (double)(line_shape->GetNumRenderPoints() - 1));
                *mfile << "(" << pt.x() << "," << pt.y() << "," << pt.z() << "),\n";
            }
            *mfile << "],\n";
            *mfile << "(" << line_shape->GetColor().R << "," << line_shape->GetColor().G << ","
                   << line_shape->GetColor().B << ",1),\n";
            *mfile << line_shape->GetThickness() << ",\n";
            if (per_frame)
                *mfile << "chrono_frame_materials"
                       << ",\n";
            else
                *mfile << "chrono_materials"
                       << ",\n";
            *mfile << collection << "\n";
            *mfile << ")\n\n";

            m_shapes->insert({(size_t)shape.get(), shape});
        }
    }

    // Write cameras. Assume cameras properties (FOV etc.) are not mutable,
    // and write them always in the assets_file, otherwise if regenerated each frame as if stored in state_file
    // they would be quite useless in Blender.
    for (const auto& camera_instance : item->GetCameras()) {
        // Do nothing if the camera was already processed (because it is shared)
        // Otherwise, add the shape to the cache list and process it
        if (this->m_blender_cameras.find((size_t)camera_instance.get()) != this->m_blender_cameras.end())
            continue;

        ChStreamOutAsciiFile* mfile;
        mfile = &assets_file;

        std::string cameraname("camera_" + std::to_string((size_t)camera_instance.get()));
        *mfile << "bpy.ops.object.camera_add(enter_editmode=False, location=(0, 0, 0), scale=(1, 1, 1)) \n"
               << "new_object = bpy.context.object \n"
               << "new_object.name= '" << cameraname << "' \n"
               << "new_object.data.lens_unit='FOV' \n";
        if (camera_instance->IsOrthographic()) {
            *mfile << "new_object.data.type='ORTHO'\n";
            *mfile << "new_object.data.ortho_scale="
                   << double((camera_instance->GetPosition() - camera_instance->GetAimPoint()).Length() *
                             tan(0.5 * camera_instance->GetAngle() * chrono::CH_C_DEG_TO_RAD))
                   << "\n";
        } else {
            *mfile << "new_object.data.type='PERSP'\n";
            *mfile << "new_object.data.angle=" << camera_instance->GetAngle() * chrono::CH_C_DEG_TO_RAD << "\n";
        }
        *mfile << "chrono_cameras.objects.link(new_object) \n"
               << "bpy.context.scene.collection.objects.unlink(new_object) \n\n";

        m_blender_cameras.insert({(size_t)camera_instance.get(), camera_instance});
    }
}

void ChBlender::ExportMaterials(ChStreamOutAsciiFile& mfile,
                                std::unordered_map<size_t, std::shared_ptr<ChVisualMaterial>>& m_materials,
                                const std::vector<std::shared_ptr<ChVisualMaterial>>& materials,
                                bool per_frame,
                                std::shared_ptr<ChVisualShape> mshape) {
    if (std::dynamic_pointer_cast<ChPathShape>(mshape) || std::dynamic_pointer_cast<ChLineShape>(mshape))
        return;

    for (const auto& mat : materials) {
        // Do nothing if the material was already processed (because it is shared)
        // Otherwise, add the material to the cache list and process it
        if (m_materials.find((size_t)mat.get()) != m_materials.end())
            continue;
        m_materials.insert({(size_t)mat.get(), mat});

        std::string matname("material_" + std::to_string((size_t)mat.get()));

        mfile << "new_mat = make_bsdf_material('" << matname << "',";

        if (mat->GetKdTexture().empty())
            mfile << "(" << mat->GetDiffuseColor().R << "," << mat->GetDiffuseColor().G << ","
                  << mat->GetDiffuseColor().B << ", 1"
                  << "),";
        else {
            std::string abspath_texture = filesystem::path(mat->GetKdTexture()).make_absolute().str();
            std::replace(abspath_texture.begin(), abspath_texture.end(), '\\', '/');
            mfile << "'" << abspath_texture.c_str() << "',";
        }

        if (mat->GetMetallicTexture().empty())
            mfile << "metallic=" << mat->GetMetallic() << ",";
        else {
            std::string abspath_texture = filesystem::path(mat->GetMetallicTexture()).make_absolute().str();
            std::replace(abspath_texture.begin(), abspath_texture.end(), '\\', '/');
            mfile << "metallic='" << abspath_texture.c_str() << "',";
        }

        mfile << "transmission=" << 1.0 - mat->GetOpacity() << ",";

        if (mat->GetRoughnessTexture().empty())
            mfile << "roughness=" << mat->GetRoughness() << ",";
        else {
            std::string abspath_texture = filesystem::path(mat->GetRoughnessTexture()).make_absolute().str();
            std::replace(abspath_texture.begin(), abspath_texture.end(), '\\', '/');
            mfile << "roughness='" << abspath_texture.c_str() << "',";
        }

        if (!mat->GetNormalMapTexture().empty()) {
            std::string abspath_texture = filesystem::path(mat->GetNormalMapTexture()).make_absolute().str();
            std::replace(abspath_texture.begin(), abspath_texture.end(), '\\', '/');
            mfile << "bump_map='" << abspath_texture.c_str() << "',";
        }

        mfile << "emissionRGB=(" << mat->GetEmissiveColor().R << "," << mat->GetEmissiveColor().G << ","
              << mat->GetEmissiveColor().B << ", 1"
              << ")";
        mfile << ") \n";

        if (per_frame)
            mfile << "chrono_frame_materials.append(new_mat) \n\n";
        else
            mfile << "chrono_materials.append(new_mat) \n\n";
    }
}

void ChBlender::ExportItemState(ChStreamOutAsciiFile& state_file,
                                std::shared_ptr<ChPhysicsItem> item,
                                const ChFrame<>& parentframe) {
    auto vis_model = item->GetVisualModel();

    bool has_stored_assets = false;
    bool has_stored_cameras = false;
    for (const auto& shape_instance : vis_model->GetShapes()) {
        const auto& shape = shape_instance.first;
        if (m_blender_shapes.find((size_t)shape.get()) != m_blender_shapes.end()) {
            has_stored_assets = true;
            break;
        }
        if (m_blender_frame_shapes.find((size_t)shape.get()) != m_blender_frame_shapes.end()) {
            has_stored_assets = true;
            break;
        }
    }
    for (const auto& camera_instance : item->GetCameras()) {
        if (m_blender_cameras.find((size_t)camera_instance.get()) != m_blender_cameras.end()) {
            has_stored_cameras = true;
            break;
        }
    }

    if (has_stored_assets) {
        if (auto particleclones = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
            state_file << "make_chrono_object_clones('" << item->GetName() << "',"
                       << "(" << parentframe.GetPos().x() << "," << parentframe.GetPos().y() << ","
                       << parentframe.GetPos().z() << "),"
                       << "(" << parentframe.GetRot().e0() << "," << parentframe.GetRot().e1() << ","
                       << parentframe.GetRot().e2() << "," << parentframe.GetRot().e3() << "), \n";
        } else {
            state_file << "make_chrono_object_assetlist('" << item->GetName() << "',"
                       << "(" << parentframe.GetPos().x() << "," << parentframe.GetPos().y() << ","
                       << parentframe.GetPos().z() << "),"
                       << "(" << parentframe.GetRot().e0() << "," << parentframe.GetRot().e1() << ","
                       << parentframe.GetRot().e2() << "," << parentframe.GetRot().e3() << "), \n";
        }

        // List visual shapes to use as children of the Blender object (parent)

        state_file << "[\n";
        for (const auto& shape_instance : vis_model->GetShapes()) {
            const auto& shape = shape_instance.first;

            // Process only "known" shapes (i.e., shapes that were included in the assets file)
            if ((m_blender_shapes.find((size_t)shape.get()) != m_blender_shapes.end()) ||
                (m_blender_frame_shapes.find((size_t)shape.get()) != m_blender_frame_shapes.end())) {
                ChVector<> aux_scale(0, 0, 0);

                std::string shapename("shape_" + std::to_string((size_t)shape.get()));
                auto shape_frame = shape_instance.second;

                // corner cases for performance reason (in case of multipe sphere asset with different radii, one
                // blender mesh asset is used anyway, then use scale here)
                if (auto mshpere = std::dynamic_pointer_cast<ChSphereShape>(shape))
                    aux_scale = ChVector<>(mshpere->GetRadius());
                else if (auto mellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape))
                    aux_scale = mellipsoid->GetSemiaxes();
                else if (auto mbox = std::dynamic_pointer_cast<ChBoxShape>(shape))
                    aux_scale = mbox->GetLengths();
                else if (auto mcone = std::dynamic_pointer_cast<ChConeShape>(shape))
                    aux_scale = ChVector<>(mcone->GetRadius(), mcone->GetRadius(), mcone->GetHeight());
                else if (auto mcyl = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
                    aux_scale = ChVector<>(mcyl->GetRadius(), mcyl->GetRadius(), mcyl->GetHeight());
                }

                state_file << " [";
                state_file << "'" << shapename << "',(" << shape_frame.GetPos().x() << "," << shape_frame.GetPos().y()
                           << "," << shape_frame.GetPos().z() << "),";
                state_file << "(" << shape_frame.GetRot().e0() << "," << shape_frame.GetRot().e1() << ","
                           << shape_frame.GetRot().e2() << "," << shape_frame.GetRot().e3() << "),";
                state_file << "[";
                if (shape->GetNumMaterials() && (!std::dynamic_pointer_cast<ChLineShape>(shape)) &&
                    (!std::dynamic_pointer_cast<ChPathShape>(shape))) {
                    for (int im = 0; im < shape->GetNumMaterials(); ++im) {
                        state_file << "'";
                        auto mat = shape->GetMaterial(im);
                        std::string matname("material_" + std::to_string((size_t)mat.get()));
                        state_file << matname;
                        state_file << "',";
                    }
                }
                state_file << "],";
                if (aux_scale != VNULL) {
                    state_file << "(" << aux_scale.x() << "," << aux_scale.y() << "," << aux_scale.z() << ")";
                }
                state_file << "],\n";
            }

        }  // end loop on shape instances

        state_file << "],\n";

        // in case of particle clones, add array of positions&rotations of particles

        if (auto particleclones = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
            state_file << " [";
            for (unsigned int m = 0; m < particleclones->GetNparticles(); ++m) {
                // Get the current coordinate frame of the i-th particle
                ChCoordsys<> partframe = particleclones->GetParticle(m).GetCoord();
                state_file << "[(" << partframe.pos.x() << "," << partframe.pos.y() << "," << partframe.pos.z() << "),";
                state_file << "(" << partframe.rot.e0() << "," << partframe.rot.e1() << "," << partframe.rot.e2() << ","
                           << partframe.rot.e3() << ")], \n";
            }
            state_file << "]\n";
        }
        state_file << ") \n\n";

    }  // end if has_stored_assets

    if (has_stored_cameras) {
        // cameras
        for (const auto& camera_instance : item->GetCameras()) {
            std::string cameraname("camera_" + std::to_string((size_t)camera_instance.get()));
            auto& cpos = camera_instance->GetPosition();
            ChVector<> cdirzm = camera_instance->GetAimPoint() - camera_instance->GetPosition();
            ChVector<> cdirx = Vcross(cdirzm, camera_instance->GetUpVector());
            ChVector<> cdiry = Vcross(cdirx, cdirzm);
            ChMatrix33<> cmrot;
            cmrot.Set_A_axis(cdirx.GetNormalized(), cdiry.GetNormalized(), -cdirzm.GetNormalized());
            ChFrame<> cframeloc(cpos, cmrot);
            ChFrame<> cframeabs = cframeloc >> parentframe;
            state_file << "update_camera_coordinates(";
            state_file << "'" << cameraname << "',";
            state_file << "(" << cframeabs.GetPos().x() << "," << cframeabs.GetPos().y() << ","
                       << cframeabs.GetPos().z() << "),";
            state_file << "(" << cframeabs.GetRot().e0() << "," << cframeabs.GetRot().e1() << ","
                       << cframeabs.GetRot().e2() << "," << cframeabs.GetRot().e3() << ")";
            state_file << ")\n\n";
        }
    }

    // Check if there are custom commands set for this physics item, assuming them always mutable (per-frame)
    auto commands = m_custom_commands.find((size_t)item.get());
    if (commands != m_custom_commands.end()) {
        state_file << commands->second << "\n";
    }
}

// This function is used at each timestep to export data formatted in a way that it can be load with the python scripts
// generated by ExportScript(). The generated filename must be set at the beginning of the animation via
// SetOutputDataFilebase(), and then a number is automatically appended and incremented at each ExportData(), e.g.,
//  state00001.dat, state00002.dat, ...
// The user should call this function in the while() loop of the simulation, once per frame.
void ChBlender::ExportData() {
    char fullname[200];
    sprintf(fullname, "%s%05d", out_data_filename.c_str(), framenumber);
    ExportData(out_path + "/" + std::string(fullname));
}

void ChBlender::ExportData(const std::string& filename) {
    // Regenerate the list of objects that need POV rendering
    UpdateRenderList();

    // Open the non-mutable single assets file in "append" mode:
    std::string assets_filename = out_script_filename + ".assets.py";
    ChStreamOutAsciiFile assets_file((base_path + assets_filename).c_str(), std::ios::app);

    // Generate the nnnnn.dat and nnnnn.py files:
    try {
        ChStreamOutAsciiFile data_file((base_path + filename + ".dat").c_str());
        ChStreamOutAsciiFile state_file((base_path + filename + ".py").c_str());

        // reset the maps of mutable (per-frame) assets, so that these will be saved at ExportAssets()
        m_blender_frame_shapes.clear();
        m_blender_frame_materials.clear();

        // Save assets
        // - non mutable assets will go into assets_file, mutable will go into state_file
        // - in both cases, assets that are already present assets will not be appended
        ExportAssets(assets_file, state_file);

        // Write custom data commands, if provided by the user
        if (custom_data.size() > 0) {
            state_file << "# Custom user-added script: \n\n";
            state_file << custom_data;
            state_file << "\n\n";
        }

        // Save time-dependent data for the geometry of objects in ...nnnn.POV and in ...nnnn.DAT file
        for (const auto& item : m_items) {
            // Nothing to do if no visual model attached to this physics item
            if (!item->GetVisualModel())
                continue;

            // saving a body?
            if (const auto& body = std::dynamic_pointer_cast<ChBody>(item)) {
                // Get the current coordinate frame of the i-th object
                const ChFrame<>& bodyframe = body->GetFrame_REF_to_abs();

                // Dump the POV macro that generates the contained asset(s) tree
                ExportItemState(state_file, body, bodyframe >> blender_frame);
            }

            // saving a cluster of particles?
            if (const auto& clones = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
                ExportItemState(state_file, clones, blender_frame);
            }

            // saving an FEA mesh?
            if (auto fea_mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
                ExportItemState(state_file, fea_mesh, blender_frame);
            }

            // saving a ChLinkMateGeneric constraint?
            if (auto linkmate = std::dynamic_pointer_cast<ChLinkMateGeneric>(item)) {
                if (linkmate->GetBody1() && linkmate->GetBody2() && frames_links_show) {
                    ChFrame<> frAabs = linkmate->GetFrame1() >> *linkmate->GetBody1() >> blender_frame;
                    ChFrame<> frBabs = linkmate->GetFrame2() >> *linkmate->GetBody2() >> blender_frame;
                    state_file << "if chrono_view_links_csys:\n";
                    state_file << "\tmcsysA = make_chrono_csys(";
                    state_file << "(" << frAabs.GetPos().x() << ", " << frAabs.GetPos().y() << ", "
                               << frAabs.GetPos().z() << "),";
                    state_file << "(" << frAabs.GetRot().e0() << "," << frAabs.GetRot().e1() << ","
                               << frAabs.GetRot().e2() << "," << frAabs.GetRot().e3() << "),";
                    state_file << "None, chrono_view_links_csys_size) \n";
                    state_file << "\tmcsysA.name = '" << linkmate->GetName() << "_frame_A"
                               << "'\n";
                    state_file << "\tchrono_frame_objects.objects.link(mcsysA)\n";
                    state_file << "\tmcsysB = make_chrono_csys(";
                    state_file << "(" << frBabs.GetPos().x() << ", " << frBabs.GetPos().y() << ", "
                               << frBabs.GetPos().z() << "),";
                    state_file << "(" << frBabs.GetRot().e0() << "," << frBabs.GetRot().e1() << ","
                               << frBabs.GetRot().e2() << "," << frBabs.GetRot().e3() << "),";
                    state_file << "None, chrono_view_links_csys_size) \n";
                    state_file << "\tmcsysB.name = '" << linkmate->GetName() << "_frame_B"
                               << "'\n";
                    state_file << "\tchrono_frame_objects.objects.link(mcsysB)\n";
                }
            }

        }  // end loop on objects

        // #) saving contacts ?
        if (this->mSystem->GetNcontacts() &&
            (this->contacts_show == ContactSymbolType::VECTOR || this->contacts_show == ContactSymbolType::SPHERE)) {
            // ChStreamOutAsciiFile data_contacts((base_path + filename + ".contacts").c_str());

            class _reporter_class : public ChContactContainer::ReportContactCallback {
              public:
                virtual bool OnReportContact(
                    const ChVector<>& pA,             // contact pA
                    const ChVector<>& pB,             // contact pB
                    const ChMatrix33<>& plane_coord,  // contact plane coordsystem (A column 'X' is contact normal)
                    const double& distance,           // contact distance
                    const double& eff_radius,         // effective radius of curvature at contact
                    const ChVector<>& react_forces,   // react.forces (in coordsystem 'plane_coord')
                    const ChVector<>& react_torques,  // react.torques (if rolling friction)
                    ChContactable* contactobjA,       // model A (note: could be nullptr)
                    ChContactable* contactobjB        // model B (note: could be nullptr)
                    ) override {
                    if (fabs(react_forces.x()) > 1e-8 || fabs(react_forces.y()) > 1e-8 ||
                        fabs(react_forces.z()) > 1e-8) {
                        // ChMatrix33<> localmatr(plane_coord);
                        ChQuaternion<> q = plane_coord.Get_A_quaternion();
                        // ChVector<> n1 = localmatr.Get_A_Xaxis();
                        // ChVector<> absreac = localmatr * react_forces;
                        (*mfile) << "\t\t[";
                        (*mfile) << pA.x() << ", ";
                        (*mfile) << pA.y() << ", ";
                        (*mfile) << pA.z();
                        (*mfile) << ",";
                        (*mfile) << q.e0() << ", ";
                        (*mfile) << q.e1() << ", ";
                        (*mfile) << q.e2() << ", ";
                        (*mfile) << q.e3();
                        (*mfile) << ",";
                        (*mfile) << react_forces.x() << ", ";
                        (*mfile) << react_forces.y() << ", ";
                        (*mfile) << react_forces.z();
                        (*mfile) << "],\n";
                    }
                    return true;  // to continue scanning contacts
                }
                // Data
                ChStreamOutAsciiFile* mfile;
            };

            state_file << "if chrono_view_contacts:\n";
            state_file << "\tcontacts= np.array([ \n";

            auto my_contact_reporter = chrono_types::make_shared<_reporter_class>();
            my_contact_reporter->mfile = &state_file;

            // scan all contacts
            mSystem->GetContactContainer()->ReportAllContacts(my_contact_reporter);

            state_file << "\t])\n";

            state_file << "\tif len(contacts):\n";
            state_file << "\t\tglyphsetting = setup_glyph_setting('contacts', glyph_type ='VECTOR LOCAL',\n";
            state_file << "\t\t\tdir_type='PROPERTY', property_index_dir=0, \n";  // will use 1st property, the 'F', for
                                                                                  // direction of force vector, in local
                                                                                  // frame of contact plane
            state_file << "\t\t\tproperty_index_basis=1, \n";  // will use 2nd property, the 'loc_rot', for local
                                                               // rotation of contact plane
            if (this->contacts_vector_length_type == ContactSymbolVectorLength::CONSTANT) {
                state_file << "\t\t\tlength_type='CONST', length_scale=" << this->contacts_vector_scalelenght
                           << ", \n";  // constant length
            }
            if (this->contacts_vector_length_type == ContactSymbolVectorLength::PROPERTY) {
                state_file << "\t\t\tlength_type='PROPERTY', property_index_length=0, length_scale="
                           << this->contacts_vector_scalelenght << ",\n";  // will use 1st property, the 'F', for length
            }
            if (this->contacts_vector_width_type == ContactSymbolVectorWidth::CONSTANT) {
                state_file << "\t\t\twidth_type='CONST', width_scale=" << this->contacts_vector_scalewidth
                           << ", \n";  // constant width
            }
            if (this->contacts_vector_width_type == ContactSymbolVectorWidth::PROPERTY) {
                state_file << "\t\t\twidth_type='PROPERTY', property_index_width=0, width_scale="
                           << this->contacts_vector_scalewidth << ",\n";  // will use 1st property, the 'F', for width
            }
            if (this->contacts_color_type == ContactSymbolColor::CONSTANT) {
                state_file << "\t\t\tcolor_type='CONST', const_color=(" << contacts_color_constant.R << ","
                           << contacts_color_constant.G << "," << contacts_color_constant.B
                           << "), \n";  // constant color
            }
            if (this->contacts_color_type == ContactSymbolColor::PROPERTY) {
                state_file
                    << "\t\t\tcolor_type='PROPERTY', property_index_color=0, \n";  // will use 1st property, the 'F',
                                                                                   // for color falsecolor scale
            }
            if (this->contacts_vector_tip)
                state_file << "\t\t\tdo_tip=True,\n";
            else
                state_file << "\t\t\tdo_tip=False,\n";
            state_file << "\t\t)\n";

            state_file << "\t\tproperty = setup_property_vector(glyphsetting, 'F', matname = 'contacts_color_F', ";
            state_file << "mcolormap = 'colormap_cooltowarm', ";
            state_file << "min=" << this->contacts_colormap_startscale << ", ";
            state_file << "max=" << this->contacts_colormap_endscale << ", ";
            state_file << "per_instance = True) \n";

            state_file << "\t\tproperty = setup_property_quaternion(glyphsetting, 'loc_rot', "
                          "matname='contacts_color_rot', per_instance=True)\n";

            state_file << "\t\tnew_objects = update_make_glyphs(glyphsetting, 'contacts',";
            state_file << "(" << blender_frame.GetPos().x() << "," << blender_frame.GetPos().y() << ","
                       << blender_frame.GetPos().z() << "),";
            state_file << "(" << blender_frame.GetRot().e0() << "," << blender_frame.GetRot().e1() << ","
                       << blender_frame.GetRot().e2() << "," << blender_frame.GetRot().e3() << "), \n";
            state_file << "\t\t  list(map(tuple,contacts[:,0:3])),\n";
            state_file << "\t\t  list_attributes=[ \n";
            state_file << "\t\t    ['F', list(map(tuple,contacts[:,7:10]))], \n";
            state_file << "\t\t    ['loc_rot', list(map(tuple,contacts[:,3:7]))], \n";
            state_file << "\t\t  ], \n";
            state_file << "\t\t) \n";
        }

    } catch (const ChException&) {
        char error[400];
        sprintf(error, "Can't save data into file %s.py (or .dat)", filename.c_str());
        throw(ChException(error));
    }

    // Increment the number of the frame.
    framenumber++;
}

}  // end namespace postprocess
}  // end namespace chrono
