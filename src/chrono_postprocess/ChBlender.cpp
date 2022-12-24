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
// Authors: Alessandro Tasora 
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
#include "chrono/assets/ChConeShape.h"
#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChCylinderShape.h"
#include "chrono/assets/ChObjFileShape.h"
#include "chrono/assets/ChSphereShape.h"
#include "chrono/assets/ChEllipsoidShape.h"
#include "chrono/assets/ChTexture.h"
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
    template_filename = GetChronoDataFile("Blender_chrono_template.py");
    out_script_filename = "exported";
    out_data_filename = "state";
    framenumber = 0;
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
    antialias = false;
    antialias_depth = 2;
    antialias_treshold = 0.1;
    picture_height = 600;
    picture_width = 800;
    ambient_light = ChColor(2, 2, 2);
    COGs_show = false;
    COGs_size = 0.04;
    frames_show = false;
    frames_size = 0.05;
    links_show = false;
    links_size = 0.04;
    contacts_show = false;
    contacts_maxsize = 0.1;
    contacts_scale = 0.01;
    contacts_scale_mode = ContactSymbol::VECTOR_SCALELENGTH;
    contacts_width = 0.001;
    contacts_colormap_startscale = 0;
    contacts_colormap_endscale = 10;
    contacts_do_colormap = true;
    wireframe_thickness = 0.001;
    single_asset_file = true;
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
void ChBlender::SetShowFrames(bool show, double msize) {
    frames_show = show;
    if (show)
        frames_size = msize;
}
void ChBlender::SetShowLinks(bool show, double msize) {
    links_show = show;
    if (show)
        links_size = msize;
}


void ChBlender::ExportScript(const std::string& filename) {
    // Regenerate the list of objects that need Blender rendering
    UpdateRenderList();

    out_script_filename = filename;

    m_blender_shapes.clear();
    m_blender_materials.clear();

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

    // Generate the xxx.assets.py script (initial state, it will be populated later by
    // appending assets as they enter the exporter, only once if shared, using ExportAssets() )

    std::string assets_filename = out_script_filename + ".assets.py";
    {
        ChStreamOutAsciiFile assets_file((base_path + assets_filename).c_str());
        assets_file << "# File containing meshes and objects for rendering Blender scenes, shared through all frames.\n";
        assets_file << "# This file must be imported in Blender using File/Import/chrono import menu, \n";
        assets_file << "# that is available in Blender if you installed the chrono_import.py add-on.\n\n";

        // Write Blender custom code
        if (custom_script.size() > 0) {
            assets_file << "# Custom user-added script: \n\n";
            assets_file << custom_script;
            assets_file << "\n\n";
        }

        // If using a single-file asset, update it at the beginning
        if (single_asset_file) {
            // populate assets (note that already present assets won't be appended!)
            ExportAssets(assets_file);
        }
    }

    // Generate the .INI script - NO, was for POV
    //ini_file << "Output_File_Name=\"" << (pic_path + "/" + pic_filename).c_str() << "\"\n";


    // Generate the script: - NO, was for POV
    /*
    ChStreamOutAsciiFile mfile((base_path + out_script_filename).c_str());

    // Rough way to load the template head file in the string buffer
    if (template_filename != "") {

        std::cout << "USE TEMPLATE FILE: " << template_filename << std::endl;

        ChStreamInAsciiFile templatefile(template_filename.c_str());
        std::string buffer_template;
        while (!templatefile.End_of_stream()) {
            char m;
            try {
                templatefile >> m;
            } catch (const ChException&) {
            }

            buffer_template += m;
        }

        // Do template replacement of [xxxyyyzzz] keywords, if any
        bl_replaceAll(buffer_template, "[xxxyyyzzz]", "blabla");

        mfile << buffer_template;
    }

    // Write default global settings and background
    /*
    mfile << "global_settings { \n"
          << " ambient_light rgb<" << ambient_light.R << "," << ambient_light.G << "," << ambient_light.B << "> \n"
          << " assumed_gamma 1.0\n";
    mfile << "}\n\n\n";
    mfile << "background { \n"
          << " rgb <" << background.R << "," << background.G << "," << background.B << "> \n";
    mfile << "}\n\n\n";

    // Write default camera

    mfile << "camera { \n";
    if (camera_orthographic) {
        mfile << " orthographic \n";
        mfile << " right x * " << (camera_location - camera_aim).Length() << " * tan ((( " << camera_angle
              << " *0.5)/180)*3.14) \n";
        mfile << " up y * image_height/image_width * " << (camera_location - camera_aim).Length() << " * tan ((("
              << camera_angle << "*0.5)/180)*3.14) \n";
        ChVector<> mdir = (camera_aim - camera_location) * 0.00001;
        mfile << " direction <" << mdir.x() << "," << mdir.y() << "," << mdir.z() << "> \n";
    } else {
        mfile << " right -x*image_width/image_height \n";
        mfile << " angle " << camera_angle << " \n";
    }
    mfile << " location <" << camera_location.x() << "," << camera_location.y() << "," << camera_location.z() << "> \n"
          << " look_at <" << camera_aim.x() << "," << camera_aim.y() << "," << camera_aim.z() << "> \n"
          << " sky <" << camera_up.x() << "," << camera_up.y() << "," << camera_up.z() << "> \n";
    mfile << "}\n\n\n";

    // Write default light

    mfile << "light_source { \n"
          << " <" << def_light_location.x() << "," << def_light_location.y() << "," << def_light_location.z() << "> \n"
          << " color rgb<" << def_light_color.R << "," << def_light_color.G << "," << def_light_color.B << "> \n";
    if (!def_light_cast_shadows)
        mfile << " shadowless \n";
    mfile << "}\n\n\n";
    */

}

void ChBlender::ExportAssets(ChStreamOutAsciiFile& mfile, bool single_file) {
    for (const auto& item : m_items) {
        ExportShapes(mfile, single_file, item);
    }
}


// Write geometries and materials in the Blender assets script for all physics items with a visual model
void ChBlender::ExportShapes(ChStreamOutAsciiFile& mfile, bool single_asset_file, std::shared_ptr<ChPhysicsItem> item) {
    // Nothing to do if the item does not have a visual model
    if (!item->GetVisualModel())
        return;

    // In a first pass, export materials from all visual shapes
    for (const auto& shape_instance : item->GetVisualModel()->GetShapes()) {
        const auto& shape = shape_instance.first;
        ExportMaterials(mfile, single_asset_file, shape->GetMaterials());
    }

    // In a second pass, export shape geometry
    for (const auto& shape_instance : item->GetVisualModel()->GetShapes()) {
        const auto& shape = shape_instance.first;
        //const auto& shape_frame = shape_instance.second; // not needed, will be set later via  make_chrono_object_assetlist in py files
        std::string shapename("object_" + std::to_string((size_t)shape.get()));

        std::string collection = "chrono_collection_assets";
        if (!single_asset_file)
            collection = "chrono_collection_objects";

        // Do nothing if the shape was already processed (because it is shared)
        // Otherwise, add the shape to the cache list and process it
        if (m_blender_shapes.find((size_t)shape.get()) != m_blender_shapes.end())
            continue;

        if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
            mfile
                << "bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=1.0, calc_uvs=True) \n"
                << "new_object = bpy.context.object \n"
                << "new_object.name = '" << shapename << "' \n"
                << "new_object.data.polygons.foreach_set('use_smooth', [True] * len(new_object.data.polygons)) \n"
                << collection << ".objects.link(new_object) \n"
                << "bpy.context.scene.collection.objects.unlink(new_object)\n\n";
            // radius will be set later in ExportObjData to avoid having n meshes per each radius 
            m_blender_shapes.insert({(size_t)shape.get(), shape});
        }

        if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
            mfile
                << "bpy.ops.mesh.primitive_uv_sphere_add(segments=32, ring_count=16, radius=1.0, calc_uvs=True) \n"
                << "new_object = bpy.context.object \n"
                << "new_object.name = '" << shapename << "' \n"
                << "new_object.data.polygons.foreach_set('use_smooth', [True] * len(new_object.data.polygons)) \n"
                << collection << ".objects.link(new_object) \n"
                << "bpy.context.scene.collection.objects.unlink(new_object)\n\n";
            // radii will be set later in ExportObjData to avoid having n meshes per each radius 
            m_blender_shapes.insert({(size_t)shape.get(), shape});
        }

        if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
            mfile
                << "bpy.ops.mesh.primitive_cylinder_add(vertices=32, radius=1.0, depth=1.0, calc_uvs=True) \n"
                << "new_object = bpy.context.object \n"
                << "new_object.name = '" << shapename << "' \n"
                << collection << ".objects.link(new_object) \n"
                << "bpy.context.scene.collection.objects.unlink(new_object)\n"
                << "with bpy.context.temp_override(selected_editable_objects=[new_object]):\n"
                << "    bpy.ops.object.shade_smooth(use_auto_smooth=True) \n\n";
            // radius, p1 and p2 will be set later in ExportObjData to avoid having n meshes per each radius 
            m_blender_shapes.insert({(size_t)shape.get(), shape});
        }

        if (auto cone = std::dynamic_pointer_cast<ChConeShape>(shape)) {
            mfile
                << "bpy.ops.mesh.primitive_cone_add(vertices=32, radius1=1.0, radius2=0, depth=1.0, calc_uvs=True) \n"
                << "new_object = bpy.context.object \n"
                << "new_object.name = '" << shapename << "' \n"
                << collection << ".objects.link(new_object) \n"
                << "bpy.context.scene.collection.objects.unlink(new_object)\n"
                << "with bpy.context.temp_override(selected_editable_objects=[new_object]):\n"
                << "    bpy.ops.object.shade_smooth(use_auto_smooth=True) \n\n";
            // radius etc will be set later in ExportObjData to avoid having n meshes per each radius 
            m_blender_shapes.insert({(size_t)shape.get(), shape});
        }

        if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
            mfile
                << "bpy.ops.mesh.primitive_cube_add(size=1,calc_uvs=True) \n"
                << "new_object = bpy.context.object \n"
                << "new_object.name = '" << shapename << "' \n"
                << collection << ".objects.link(new_object) \n"
                << "bpy.context.scene.collection.objects.unlink(new_object)\n\n";
            // xyz sizes will be set later in ExportObjData to avoid having n meshes
            m_blender_shapes.insert({(size_t)shape.get(), shape});
        }

        if (auto obj_shape = std::dynamic_pointer_cast<ChObjFileShape>(shape)) {
            std::string abspath_obj = filesystem::path(obj_shape->GetFilename()).make_absolute().str();
            std::replace(abspath_obj.begin(), abspath_obj.end(), '\\', '/');
            mfile
                << "try: \n"
                << "    bpy.context.view_layer.active_layer_collection = bpy.context.view_layer.layer_collection \n"
                << "    file_loc = '" << abspath_obj.c_str() << "'\n"
                << "    imported_object = bpy.ops.import_scene.obj(filepath=file_loc) \n"
                << "    new_object = bpy.context.selected_objects[-1] \n"
                << "    new_object.name= '" << shapename << "' \n"
                << "    " << collection << ".objects.link(new_object) \n"
                << "    bpy.context.scene.collection.objects.unlink(new_object) \n"
                << "except: \n"
                << "    print('Cannot load .OBJ file: ', file_loc) \n\n";
            // if it fails to load (ex.: missing file, bad obj, etc) it prints error to console
            m_blender_shapes.insert({(size_t)shape.get(), shape});
        }

        if (auto mesh_shape = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape)) {
            std::shared_ptr<ChTriangleMeshConnected> mesh = mesh_shape->GetMesh();
            bool wireframe = mesh_shape->IsWireframe();

            mfile << "verts = [ \n";
            for (unsigned int iv = 0; iv < mesh->m_vertices.size(); iv++) {
                mfile << "(" << mesh->m_vertices[iv].x() << "," << mesh->m_vertices[iv].y() << "," << mesh->m_vertices[iv].z() << "),\n";
            }
            mfile << "] \n";

            mfile << "faces = [ \n";
            for (unsigned int ip = 0; ip < mesh->m_face_v_indices.size(); ip++) {
                mfile << "(" << mesh->m_face_v_indices[ip].x() << "," << mesh->m_face_v_indices[ip].y() << "," << mesh->m_face_v_indices[ip].z() << "),\n";
            }
            mfile << "] \n";

            mfile << "edges = [] \n";

            mfile << "new_mesh = bpy.data.meshes.new('mesh_mesh') \n";
            mfile << "new_mesh.from_pydata(verts, edges, faces) \n";

            if (mesh->m_face_uv_indices.size() == mesh->m_face_v_indices.size()) {
                // shared UVs via m_face_uv_indices
                mfile << "uvs = [ \n";
                for (unsigned int it = 0; it < mesh->m_face_uv_indices.size(); it++) {
                    mfile 
                        << mesh->m_UV[mesh->m_face_uv_indices[it].x()].x() << "," << mesh->m_UV[mesh->m_face_v_indices[it].x()].y() << ","
                        << mesh->m_UV[mesh->m_face_uv_indices[it].y()].x() << "," << mesh->m_UV[mesh->m_face_v_indices[it].y()].y() << ","
                        << mesh->m_UV[mesh->m_face_uv_indices[it].z()].x() << "," << mesh->m_UV[mesh->m_face_v_indices[it].z()].y() << ","
                        << "\n";
                }
                mfile << "] \n";
                mfile << "uv_layer = new_mesh.uv_layers.new(name = 'UVMap') \n";
                mfile << "uv_layer.data.foreach_set('uv', uvs) \n";
            } 
            else if (mesh->m_UV.size() == 3*mesh->m_face_v_indices.size()) {
                // no m_face_uv_indices, but assume one UV per face index
                mfile << "uvs = [ \n";
                for (unsigned int it = 0; it < mesh->m_UV.size(); it+=3) {
                    mfile  
                        << mesh->m_UV[it  ].x() << "," << mesh->m_UV[it  ].y() << ","
                        << mesh->m_UV[it+1].x() << "," << mesh->m_UV[it+1].y() << ","
                        << mesh->m_UV[it+2].x() << "," << mesh->m_UV[it+2].y() << ","
                        << "\n";
                }
                mfile << "] \n";
                mfile << "uv_layer = new_mesh.uv_layers.new(name = 'UVMap') \n";
                mfile << "uv_layer.data.foreach_set('uv', uvs) \n";
            }

            mfile
                << "new_mesh.update() \n"
                << "new_object = bpy.data.objects.new('mesh_object', new_mesh) \n"
                << "new_object.data.polygons.foreach_set('use_smooth', [True] * len(new_object.data.polygons)) \n"
                << "modifier = new_object.modifiers.new(name='edgesplit', type='EDGE_SPLIT') \n"
                << "#modifier.split_angle = 1.0 \n";
            if (wireframe) {
                mfile << "modifier = new_object.modifiers.new(name='wireframe', type='WIREFRAME') \n";
                mfile << "modifier.thickness = " << wireframe_thickness << "\n";
            }
            mfile << collection << ".objects.link(new_object) \n\n";

            m_blender_shapes.insert({(size_t)shape.get(), shape});
        }

        
    }

    // Check if there are custom commands set for this physics item
    auto commands = m_custom_commands.find((size_t)item.get());
    if (commands != m_custom_commands.end()) {
        mfile << commands->second << "\n";
    }
}

void ChBlender::ExportMaterials(ChStreamOutAsciiFile& mfile, bool single_asset_file, const std::vector<std::shared_ptr<ChVisualMaterial>>& materials) {
    for (const auto& mat : materials) {

        // Do nothing if the material was already processed (because it is shared)
        // Otherwise, add the material to the cache list and process it
        if (m_blender_materials.find((size_t)mat.get()) != m_blender_materials.end())
            continue;
        m_blender_materials.insert({(size_t)mat.get(), mat});
        
        std::string matname("material_" + std::to_string((size_t)mat.get()));

        mfile << "make_bsdf_material('" << matname << "',";
        if (mat->GetKdTexture().empty())
            mfile << "(" << mat->GetDiffuseColor().R << "," << mat->GetDiffuseColor().G << "," << mat->GetDiffuseColor().B << ", 1" << "),";
        else
            mfile << "'" << filesystem::path(mat->GetKdTexture()).make_absolute().filename().c_str() << "',";
        mfile << "metallic=" << mat->GetMetallic() << ",";
        mfile << "transmission=" << 1.0-mat->GetOpacity() << ",";
        mfile << "roughness=" << mat->GetRoughness() << ",";
        mfile << "emissionRGB=(" << mat->GetEmissiveColor().R << "," << mat->GetEmissiveColor().G << "," << mat->GetEmissiveColor().B <<  ", 1" << ")";
        mfile << ") \n\n";
    }
}

void ChBlender::ExportObjData(ChStreamOutAsciiFile& state_file,
                             std::shared_ptr<ChPhysicsItem> item,
                             const ChFrame<>& parentframe) {
    // Check for custom command for this item
    auto commands = m_custom_commands.find((size_t)item.get());

    auto vis_model = item->GetVisualModel();
    
    bool has_stored_assets = false;
    for (const auto& shape_instance : vis_model->GetShapes()) {
        const auto& shape = shape_instance.first;
        if (m_blender_shapes.find((size_t)shape.get()) != m_blender_shapes.end()) {
            has_stored_assets = true;
            break;
        }
    }

    if (has_stored_assets) {

        if (auto particleclones = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
            state_file
                << "make_chrono_object_clones('" << item->GetName() << "',"
                << "(" << 0 << "," << 0 << "," << 0 << "),(1,0,0,0), \n";
        }
        if (auto body = std::dynamic_pointer_cast<ChBody>(item)) {
            state_file
                << "make_chrono_object_assetlist('" << item->GetName() << "',"
                << "(" << parentframe.GetPos().x() << "," << parentframe.GetPos().y() << "," << parentframe.GetPos().z() << "),"
                << "(" << parentframe.GetRot().e0() << "," << parentframe.GetRot().e1() << "," << parentframe.GetRot().e2() << "," << parentframe.GetRot().e3() << "), \n";
        }
        
        // Scan visual shapes in the visual model
        state_file << "[\n";
        for (const auto& shape_instance : vis_model->GetShapes()) {
            const auto& shape = shape_instance.first;
            const auto& shape_frame = shape_instance.second;
            std::string shapename("object_" + std::to_string((size_t)shape.get()));

            // Process only "known" shapes (i.e., shapes that were included in the assets file)
            if (m_blender_shapes.find((size_t)shape.get()) != m_blender_shapes.end()) {
                ChVector<> aux_scale(0,0,0);

                // corner cases for performance reason (in case of multipe sphere asset with different radii, one blender mesh asset is used anyway, then use scale here)
                if (auto mshpere = std::dynamic_pointer_cast<ChSphereShape>(shape))
                    aux_scale = ChVector<>(mshpere->GetSphereGeometry().rad, mshpere->GetSphereGeometry().rad, mshpere->GetSphereGeometry().rad);
                if (auto mellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape))
                    aux_scale = ChVector<>(mellipsoid->GetEllipsoidGeometry().rad.x(), mellipsoid->GetEllipsoidGeometry().rad.y(), mellipsoid->GetEllipsoidGeometry().rad.z());
                if (auto mbox = std::dynamic_pointer_cast<ChBoxShape>(shape))
                    aux_scale = ChVector<>(mbox->GetBoxGeometry().GetLengths().x(), mbox->GetBoxGeometry().GetLengths().y(), mbox->GetBoxGeometry().GetLengths().z());
                if (auto mcone = std::dynamic_pointer_cast<ChConeShape>(shape)) {
                    aux_scale = ChVector<>(mcone->GetConeGeometry().rad.x(), mcone->GetConeGeometry().rad.y(), mcone->GetConeGeometry().rad.z());
                if (auto mcyl = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
                    aux_scale = ChVector<>(mcyl->GetCylinderGeometry().rad, mcyl->GetCylinderGeometry().rad, (mcyl->GetCylinderGeometry().p2-mcyl->GetCylinderGeometry().p1).Length());
                }

                state_file << " [";
                state_file << "'" << shapename << "',(" << shape_frame.GetPos().x() << "," << shape_frame.GetPos().y() << "," << shape_frame.GetPos().z() << "),";
                state_file << "(" << shape_frame.GetRot().e0() << "," << shape_frame.GetRot().e1() << "," << shape_frame.GetRot().e2() << "," << shape_frame.GetRot().e3() << "),";
                state_file << "'";
                if (shape->GetNumMaterials()) {
                    auto mat = shape->GetMaterial(0); // use only 1st of materials for a single shape
                    std::string matname("material_" + std::to_string((size_t)mat.get()));
                    state_file <<  matname;
                }
                state_file << "',";
                if (aux_scale != VNULL) {
                    state_file << "(" << aux_scale.x() << "," << aux_scale.y() << "," << aux_scale.z() << ")";
                }
                state_file << "],\n";
            }
        }
        state_file << "],\n";

        if (auto particleclones = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
            state_file << " [";
            for (unsigned int m = 0; m < particleclones->GetNparticles(); ++m) {
                // Get the current coordinate frame of the i-th particle
                ChCoordsys<> partframe = particleclones->GetParticle(m).GetCoord();
                state_file << "[(" << partframe.pos.x() << "," << partframe.pos.y() << "," << partframe.pos.z() << "),";
                state_file << "(" << partframe.rot.e0() << "," << partframe.rot.e1() << "," << partframe.rot.e2() << "," << partframe.rot.e3() << ")], \n";
            }
            state_file << "]\n";
        }
        state_file << ") \n\n";

    }// end if has_stored_assets


    // Scan FEA visual shapes in the visual model
    //// RADU TODO
    /*
    for (const auto& shapeFEA : vis_model->GetShapesFEA()) {
    }
    */

    /*
    // Check for any cameras attached to the physics item
    //// RADU TODO: allow using more than one camera at a time?
    for (const auto& camera : item->GetCameras()) {
        camera_found_in_assets = true;

        camera_location = camera->GetPosition() >> parentframe;
        camera_aim = camera->GetAimPoint() >> parentframe;
        camera_up = camera->GetUpVector() >> parentframe;
        camera_angle = camera->GetAngle();
        camera_orthographic = camera->IsOrthographic();       
    }

    // Invoke the custom commands string (if any)
    if (num_commands > 0) {
        state_file << "cm_" << (size_t)item.get() << "()\n";
    }

    */
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

    // If using a single-file asset, update it by appending (in case new visual shapes were created during simulation)
    if (single_asset_file) {
        // open asset file in append mode
        std::string assets_filename = out_script_filename + ".assets";
        ChStreamOutAsciiFile assets_file((base_path + assets_filename).c_str(), std::ios::app);
        // populate assets (already present assets will not be appended)
        ExportAssets(assets_file);
    }

    // Generate the nnnnn.dat and nnnnn.py files:
    try {
        char pathdat[200];
        sprintf(pathdat, "%s.dat", filename.c_str());
        ChStreamOutAsciiFile data_file((base_path + filename + ".dat").c_str());
        ChStreamOutAsciiFile state_file((base_path + filename + ".py").c_str());

        camera_found_in_assets = false;

        // If embedding assets in the nnnnn.py file:
        if (!single_asset_file) {
            m_blender_shapes.clear();
            ExportAssets(state_file, false);
        }

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
                ChCoordsys<> assetcsys = bodyframe.GetCoord();

                // Dump the POV macro that generates the contained asset(s) tree
                ExportObjData(state_file, body, bodyframe);

                /*
                // Show body COG?
                if (COGs_show) {
                    const ChCoordsys<>& cogcsys = body->GetFrame_COG_to_abs().GetCoord();
                    state_file << "sh_csysCOG(";
                    state_file << cogcsys.pos.x() << "," << cogcsys.pos.y() << "," << cogcsys.pos.z() << ",";
                    state_file << cogcsys.rot.e0() << "," << cogcsys.rot.e1() << "," << cogcsys.rot.e2() << ","
                             << cogcsys.rot.e3() << ",";
                    state_file << COGs_size << ")\n";
                }
                // Show body frame ref?
                if (frames_show) {
                    state_file << "sh_csysFRM(";
                    state_file << assetcsys.pos.x() << "," << assetcsys.pos.y() << "," << assetcsys.pos.z() << ",";
                    state_file << assetcsys.rot.e0() << "," << assetcsys.rot.e1() << "," << assetcsys.rot.e2() << ","
                             << assetcsys.rot.e3() << ",";
                    state_file << frames_size << ")\n";
                }
                */
            }

            // saving a cluster of particles?
            if (const auto& clones = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
                ExportObjData(state_file, clones, ChFrame<>());
            }

            // saving a ChLinkMateGeneric constraint?
            /*
            if (auto linkmate = std::dynamic_pointer_cast<ChLinkMateGeneric>(item)) {
                if (linkmate->GetBody1() && linkmate->GetBody2() && links_show) {
                    ChFrame<> frAabs = linkmate->GetFrame1() >> *linkmate->GetBody1();
                    ChFrame<> frBabs = linkmate->GetFrame2() >> *linkmate->GetBody2();
                    state_file << "sh_csysFRM(";
                    state_file << frAabs.GetPos().x() << "," << frAabs.GetPos().y() << "," << frAabs.GetPos().z() << ",";
                    state_file << frAabs.GetRot().e0() << "," << frAabs.GetRot().e1() << "," << frAabs.GetRot().e2()
                             << "," << frAabs.GetRot().e3() << ",";
                    state_file << links_size * 0.7 << ")\n";  // smaller, as 'slave' csys.
                    state_file << "sh_csysFRM(";
                    state_file << frBabs.GetPos().x() << "," << frBabs.GetPos().y() << "," << frBabs.GetPos().z() << ",";
                    state_file << frBabs.GetRot().e0() << "," << frBabs.GetRot().e1() << "," << frBabs.GetRot().e2()
                             << "," << frBabs.GetRot().e3() << ",";
                    state_file << links_size << ")\n";
                }
            }
            */

            // saving an FEA mesh?
            if (auto fea_mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
                ExportObjData(state_file, fea_mesh, ChFrame<>());
            }

        }  // end loop on objects

        // #) saving contacts ?
        /*
        if (contacts_show) {
            ChStreamOutAsciiFile data_contacts((base_path + filename + ".contacts").c_str());

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
                        ChMatrix33<> localmatr(plane_coord);
                        ChVector<> n1 = localmatr.Get_A_Xaxis();
                        ChVector<> absreac = localmatr * react_forces;
                        (*mfile) << pA.x() << ", ";
                        (*mfile) << pA.y() << ", ";
                        (*mfile) << pA.z() << ", ";
                        (*mfile) << n1.x() << ", ";
                        (*mfile) << n1.y() << ", ";
                        (*mfile) << n1.z() << ", ";
                        (*mfile) << absreac.x() << ", ";
                        (*mfile) << absreac.y() << ", ";
                        (*mfile) << absreac.z() << ", \n";
                    }
                    return true;  // to continue scanning contacts
                }
                // Data
                ChStreamOutAsciiFile* mfile;
            };

            auto my_contact_reporter = chrono_types::make_shared<_reporter_class>();
            my_contact_reporter->mfile = &data_contacts;

            // scan all contacts
            mSystem->GetContactContainer()->ReportAllContacts(my_contact_reporter);
        }
        */

        /*
        // If a camera have been found in assets, create it and override the default one
        if (camera_found_in_assets) {
            state_file << "camera { \n";
            if (camera_orthographic) {
                state_file << " orthographic \n";
                state_file << " right x * " << (camera_location - camera_aim).Length() << " * tan ((( " << camera_angle
                         << " *0.5)/180)*3.14) \n";
                state_file << " up y * image_height/image_width * " << (camera_location - camera_aim).Length()
                         << " * tan (((" << camera_angle << "*0.5)/180)*3.14) \n";
                ChVector<> mdir = (camera_aim - camera_location) * 0.00001;
                state_file << " direction <" << mdir.x() << "," << mdir.y() << "," << mdir.z() << "> \n";
            } else {
                state_file << " right -x*image_width/image_height \n";
                state_file << " angle " << camera_angle << " \n";
            }
            state_file << " location <" << camera_location.x() << "," << camera_location.y() << "," << camera_location.z()
                     << "> \n"
                     << " look_at <" << camera_aim.x() << "," << camera_aim.y() << "," << camera_aim.z() << "> \n"
                     << " sky <" << camera_up.x() << "," << camera_up.y() << "," << camera_up.z() << "> \n";
            state_file << "}\n\n\n";
        }
        */

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
