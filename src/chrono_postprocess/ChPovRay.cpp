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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#include "chrono/assets/ChBoxShape.h"
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

#include "chrono_postprocess/ChPovRay.h"

#include "chrono_thirdparty/filesystem/path.h"

namespace chrono {
namespace postprocess {

using namespace geometry;

ChPovRay::ChPovRay(ChSystem* system) : ChPostProcessBase(system) {
    base_path = "";
    pic_path = "anim";
    out_path = "output";
    pic_filename = "picture";
    template_filename = GetChronoDataFile("_template_POV.pov");
    out_script_filename = "render_frames.pov";
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

void ChPovRay::Add(std::shared_ptr<ChPhysicsItem> item) {
    m_items.insert(item);
}

void ChPovRay::Remove(std::shared_ptr<ChPhysicsItem> item) {
    m_items.erase(item);
}

void ChPovRay::AddAll() {
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

void ChPovRay::RemoveAll() {
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

void ChPovRay::SetCustomCommands(std::shared_ptr<ChPhysicsItem> item, const std::string& commands) {
    m_custom_commands[(size_t)item.get()] = commands;
}

void ChPovRay::UpdateRenderList() {
    // Remove physics items from the render list if the only reference to them is in this list.
    for (auto item = m_items.begin(); item != m_items.end();) {
        if (item->use_count() == 1)
            item = m_items.erase(item);
        else
            ++item;
    }
}

std::string replaceOnce(std::string result, const std::string& replaceWhat, const std::string& replaceWithWhat) {
    const std::string::size_type pos = result.find(replaceWhat);
    if (pos == std::string::npos)
        return result;
    result.replace(pos, replaceWhat.size(), replaceWithWhat);
    return result;
}

std::string replaceAll(std::string result, const std::string& replaceWhat, const std::string& replaceWithWhat) {
    while (1) {
        const std::string::size_type pos = result.find(replaceWhat);
        if (pos == std::string::npos)
            break;
        result.replace(pos, replaceWhat.size(), replaceWithWhat);
    }
    return result;
}

void ChPovRay::SetCamera(ChVector<> location, ChVector<> aim, double angle, bool ortho) {
    camera_location = location;
    camera_aim = aim;
    camera_angle = angle;
    camera_orthographic = ortho;
}

void ChPovRay::SetLight(ChVector<> location, ChColor color, bool cast_shadow) {
    def_light_location = location;
    def_light_color = color;
    def_light_cast_shadows = cast_shadow;
}

void ChPovRay::SetShowCOGs(bool show, double msize) {
    COGs_show = show;
    if (show)
        COGs_size = msize;
}
void ChPovRay::SetShowFrames(bool show, double msize) {
    frames_show = show;
    if (show)
        frames_size = msize;
}
void ChPovRay::SetShowLinks(bool show, double msize) {
    links_show = show;
    if (show)
        links_size = msize;
}
void ChPovRay::SetShowContacts(bool show,
                               ContactSymbol mode,
                               double scale,
                               double width,
                               double max_size,
                               bool do_colormap,
                               double colormap_start,
                               double colormap_end) {
    contacts_show = show;
    if (show) {
        contacts_scale_mode = mode;
        contacts_scale = scale;
        contacts_width = width;
        contacts_maxsize = max_size;
        contacts_do_colormap = do_colormap;
        contacts_colormap_startscale = colormap_start;
        contacts_colormap_endscale = colormap_end;
    }
}

void ChPovRay::ExportScript(const std::string& filename) {
    // Regenerate the list of objects that need POV rendering
    UpdateRenderList();

    out_script_filename = filename;

    m_pov_shapes.clear();
    m_pov_materials.clear();

    // Create directories
    if (base_path != "") {
        if (!filesystem::create_directory(filesystem::path(base_path))) {
            std::cout << "Error creating base directory \"" << base_path << "\" for the POV files." << std::endl;
            return;
        }
        base_path = base_path + "/";
    }
    filesystem::create_directory(filesystem::path(base_path + pic_path));
    filesystem::create_directory(filesystem::path(base_path + out_path));

    // Generate the _assets.pov script (initial state, it will be populated later by
    // appending assets as they enter the exporter, only once if shared, using ExportAssets() )

    std::string assets_filename = out_script_filename + ".assets";
    {
        ChStreamOutAsciiFile assets_file((base_path + assets_filename).c_str());
        assets_file << "// File containing meshes and objects for rendering POV scenes.\n";
        assets_file << "// This file is automatically included by " << out_script_filename << ".pov , \n";
        assets_file << "// and you should not modify it.\n\n";
    }

    // Generate the .INI script

    ChStreamOutAsciiFile ini_file((base_path + out_script_filename + ".ini").c_str());

    ini_file << "; Script for rendering an animation with POV-Ray. \n";
    ini_file << "; Generated automatically by Chrono::Engine. \n\n";
    if (antialias)
        ini_file << "Antialias=On \n";
    else
        ini_file << "Antialias=Off\n";
    ini_file << "Antialias_Threshold=" << antialias_treshold << "\n";
    ini_file << "Antialias_Depth=" << antialias_depth << "\n";
    ini_file << "Height=" << picture_height << "\n";
    ini_file << "Width =" << picture_width << "\n";
    ini_file << "Input_File_Name=\"" << out_script_filename << "\"\n";
    ini_file << "Output_File_Name=\"" << (pic_path + "/" + pic_filename).c_str() << "\"\n";
    ini_file << "Initial_Frame=0000\n";
    ini_file << "Final_Frame=0999\n";
    ini_file << "Initial_Clock=0\n";
    ini_file << "Final_Clock=1\n";
    ini_file << "Pause_when_Done=Off\n\n";

    // Generate the .POV script:

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
        replaceAll(buffer_template, "[xxxyyyzzz]", "blabla");

        mfile << buffer_template;
    }

    // Write default global settings and background

    mfile << "global_settings { \n"
          << " ambient_light rgb<" << ambient_light.R << "," << ambient_light.G << "," << ambient_light.B << "> \n";
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

    // Write POV custom code

    if (custom_script.size() > 0) {
        mfile << "// Custom user-added script: \n\n";
        mfile << custom_script;
        mfile << "\n\n";
    }

    // Write POV code to open the asset file

    mfile << "// Include shape assets (triangle meshes): \n\n";
    mfile << "#include \"" << assets_filename << "\"\n\n";

    // Write POV code to open the n.th scene file

    mfile << "// Include POV code to for the n.th scene file: \n\n";
    mfile << "#declare scene_file = concat(\"" << (out_path + "/" + out_data_filename).c_str()
          << "\", str(frame_number,-5,0), \".pov\") \n";
    mfile << "#include scene_file \n\n";

    // Write POV code to load and display contacts

    if (contacts_show) {
        mfile << "// Load contacts and create objects to show them: \n\n";

        mfile << "#declare contacts_scale=" << contacts_scale << ";\n";
        mfile << "#declare contacts_width=" << contacts_width << ";\n";
        mfile << "#declare contacts_maxsize=" << contacts_maxsize << ";\n";
        mfile << "#declare contacts_do_colormap=" << ((int)contacts_do_colormap) << ";\n";
        mfile << "#declare contacts_colormap_startscale=" << contacts_colormap_startscale << ";\n";
        mfile << "#declare contacts_colormap_endscale=" << contacts_colormap_endscale << ";\n";
        mfile << "#declare contacts_defaultcolor= rgb<0.0,0.9,0.2>; \n";
        switch (contacts_scale_mode) {
            case ContactSymbol::VECTOR_SCALELENGTH:
                mfile << "#declare contacts_scale_mode=1;\n";
                mfile << "#declare draw_contacts_asspheres =0;\n";
                mfile << "#declare draw_contacts_ascylinders =1;\n";
                break;
            case ContactSymbol::VECTOR_SCALERADIUS:
                mfile << "#declare contacts_scale_mode=2;\n";
                mfile << "#declare draw_contacts_asspheres =0;\n";
                mfile << "#declare draw_contacts_ascylinders =1;\n";
                break;
            case ContactSymbol::VECTOR_NOSCALE:
                mfile << "#declare contacts_scale_mode=0;\n";
                mfile << "#declare draw_contacts_asspheres =0;\n";
                mfile << "#declare draw_contacts_ascylinders =1;\n";
                break;
            case ContactSymbol::SPHERE_SCALERADIUS:
                mfile << "#declare contacts_scale_mode=1;\n";
                mfile << "#declare draw_contacts_asspheres =1;\n";
                mfile << "#declare draw_contacts_ascylinders =0;\n";
                break;
        }
        mfile << "\n";

        mfile << "#declare contacts_file = concat(\"" << (out_path + "/" + out_data_filename).c_str()
              << "\", str(frame_number,-5,0), \".contacts\") \n";
        mfile << "#fopen MyContactsFile contacts_file read \n";

        mfile << " \n\
			union{\n\
				#read (MyContactsFile, apx, apy, apz, anx, any, anz,  afx, afy, afz ) \n\
				#while (defined(MyContactsFile)) \n\
					#read (MyContactsFile, apx, apy, apz, anx, any, anz,  afx, afy, afz ) \n\
					make_contact(apx, apy, apz, anx, any, anz,  afx, afy, afz) \n\
				#end  \n\
			} \n";
    }

    // If using a single-file asset, update it (because maybe that during the
    // animation someone created an object with asset)
    if (single_asset_file) {
        // open asset file in append mode
        assets_filename = out_script_filename + ".assets";
        ChStreamOutAsciiFile assets_file((base_path + assets_filename).c_str(), std::ios::app);
        // populate assets (note that already present assets won't be appended!)
        ExportAssets(assets_file);
    }
}

void ChPovRay::ExportAssets(ChStreamOutAsciiFile& assets_file) {
    for (const auto& item : m_items) {
        ExportShapes(assets_file, item);
    }
}

// Write geometries and materials in the POV assets script for all physics items with a visual model
void ChPovRay::ExportShapes(ChStreamOutAsciiFile& assets_file, std::shared_ptr<ChPhysicsItem> item) {
    // Nothing to do if the item does not have a visual model
    if (!item->GetVisualModel())
        return;

    for (const auto& shape_instance : item->GetVisualModel()->GetShapes()) {
        const auto& shape = shape_instance.first;
        const auto& shape_frame = shape_instance.second;

        // Do nothing if the shape was already processed (because it is shared)
        // Otherwise, add the shape to the cache list and process it
        if (m_pov_shapes.find((size_t)shape.get()) != m_pov_shapes.end())
            continue;
        m_pov_shapes.insert({(size_t)shape.get(), shape});

        auto obj_shape = std::dynamic_pointer_cast<ChObjFileShape>(shape);
        auto mesh_shape = std::dynamic_pointer_cast<ChTriangleMeshShape>(shape);

        if (obj_shape || mesh_shape) {
            std::shared_ptr<ChTriangleMeshConnected> mesh;
            bool wireframe = false;

            if (obj_shape) {
                auto temp_allocated_loadtrimesh =
                    ChTriangleMeshConnected::CreateFromWavefrontFile(obj_shape->GetFilename(), true, true);
                if (temp_allocated_loadtrimesh) {
                    mesh = temp_allocated_loadtrimesh;
                } else {
                    char error[400];
                    sprintf(error, "Cannot read .obj file %s", obj_shape->GetFilename().c_str());
                    throw(ChException(error));
                }
            } else if (mesh_shape) {
                mesh = mesh_shape->GetMesh();
                wireframe = mesh_shape->IsWireframe();
            }

            // POV macro to build the asset - begin
            assets_file << "#macro sh_" << (size_t)shape.get() << "()\n";

            if (!wireframe) {
                // Create mesh
                assets_file << "mesh2  {\n";

                assets_file << " vertex_vectors {\n";
                assets_file << (int)mesh->m_vertices.size() << ",\n";
                for (unsigned int iv = 0; iv < mesh->m_vertices.size(); iv++)
                    assets_file << "  <" << mesh->m_vertices[iv].x() << "," << mesh->m_vertices[iv].y() << ","
                                << mesh->m_vertices[iv].z() << ">,\n";
                assets_file << " }\n";

                assets_file << " normal_vectors {\n";
                assets_file << (int)mesh->m_normals.size() << ",\n";
                for (unsigned int iv = 0; iv < mesh->m_normals.size(); iv++)
                    assets_file << "  <" << mesh->m_normals[iv].x() << "," << mesh->m_normals[iv].y() << ","
                                << mesh->m_normals[iv].z() << ">,\n";
                assets_file << " }\n";

                assets_file << " uv_vectors {\n";
                assets_file << (int)mesh->m_UV.size() << ",\n";
                for (unsigned int iv = 0; iv < mesh->m_UV.size(); iv++)
                    assets_file << "  <" << mesh->m_UV[iv].x() << "," << mesh->m_UV[iv].y() << ">,\n";
                assets_file << " }\n";

                if (mesh->m_colors.size() == mesh->m_vertices.size()) {
                    assets_file << " texture_list {\n";
                    assets_file << (int)(mesh->m_colors.size()) << ",\n";
                    for (unsigned int iv = 0; iv < mesh->m_vertices.size(); iv++) {
                        assets_file << " texture{pigment{rgb <" << mesh->m_colors[iv].R << "," << mesh->m_colors[iv].G
                                    << "," << mesh->m_colors[iv].B << ">}},\n";
                    }
                    assets_file << " }\n";
                }

                assets_file << " face_indices {\n";
                assets_file << (int)mesh->m_face_v_indices.size() << ",\n";
                for (unsigned int it = 0; it < mesh->m_face_v_indices.size(); it++) {
                    assets_file << "  <" << mesh->m_face_v_indices[it].x() << "," << mesh->m_face_v_indices[it].y()
                                << "," << mesh->m_face_v_indices[it].z() << ">";
                    if (mesh->m_colors.size() == mesh->m_vertices.size())
                        assets_file << mesh->m_face_v_indices[it].x() << "," << mesh->m_face_v_indices[it].y() << ","
                                    << mesh->m_face_v_indices[it].z();
                    assets_file << ",\n";
                }
                assets_file << " }\n";

                if (mesh->m_face_n_indices.size() > 0) {
                    assets_file << " normal_indices {\n";
                    assets_file << (int)mesh->m_face_n_indices.size() << ",\n";
                    for (unsigned int it = 0; it < mesh->m_face_n_indices.size(); it++)
                        assets_file << "  <" << mesh->m_face_n_indices[it].x() << "," << mesh->m_face_n_indices[it].y()
                                    << "," << mesh->m_face_n_indices[it].z() << ">,\n";
                    assets_file << " }\n";
                }
                if ((mesh->m_face_uv_indices.size() != mesh->m_face_v_indices.size()) &&
                    (mesh->m_face_uv_indices.size() > 0)) {
                    assets_file << " uv_indices {\n";
                    assets_file << (int)mesh->m_face_uv_indices.size() << ",\n";
                    for (unsigned int it = 0; it < mesh->m_face_uv_indices.size(); it++)
                        assets_file << "  <" << mesh->m_face_uv_indices[it].x() << ","
                                    << mesh->m_face_uv_indices[it].y() << "," << mesh->m_face_uv_indices[it].z()
                                    << ">,\n";
                    assets_file << " }\n";
                }

                assets_file << "}\n";
            } else {
                // wireframed mesh
                std::map<std::pair<int, int>, std::pair<int, int>> edges;
                mesh->ComputeWingedEdges(edges, true);
                for (const auto& edge : edges) {
                    assets_file << " cylinder {<" << mesh->m_vertices[edge.first.first].x() << ","
                                << mesh->m_vertices[edge.first.first].y() << ","
                                << mesh->m_vertices[edge.first.first].z() << ">,";
                    assets_file << "<" << mesh->m_vertices[edge.first.second].x() << ","
                                << mesh->m_vertices[edge.first.second].y() << ","
                                << mesh->m_vertices[edge.first.second].z() << ">,";
                    assets_file << (wireframe_thickness * 0.5) << "\n no_shadow ";
                    if (mesh->m_colors.size() == mesh->m_vertices.size())
                        assets_file << "finish{ ambient rgb<" << mesh->m_colors[edge.first.first].R << ","
                                    << mesh->m_colors[edge.first.first].G << "," << mesh->m_colors[edge.first.first].B
                                    << "> diffuse 0}";
                    assets_file << "}\n";
                }
            }

            // POV macro - end
            assets_file << "#end \n";
        }

        if (auto sphere = std::dynamic_pointer_cast<ChSphereShape>(shape)) {
            // POV macro to build the asset - begin
            assets_file << "#macro sh_" << (size_t)shape.get() << "()\n";

            // POV will make the sphere
            assets_file << "sphere  {\n";

            assets_file << " <" << shape_frame.GetPos().x();
            assets_file << "," << shape_frame.GetPos().y();
            assets_file << "," << shape_frame.GetPos().z() << ">\n";
            assets_file << " " << sphere->GetSphereGeometry().rad << "\n";

            assets_file << "}\n";

            // POV macro - end
            assets_file << "#end \n";
        }

        if (auto ellipsoid = std::dynamic_pointer_cast<ChEllipsoidShape>(shape)) {
            // POV macro to build the asset - begin
            assets_file << "#macro sh_" << (size_t)shape.get() << "()\n";

            // POV will make the sphere
            assets_file << "sphere  {\n";

            assets_file << " <" << shape_frame.GetPos().x();
            assets_file << "," << shape_frame.GetPos().y();
            assets_file << "," << shape_frame.GetPos().z() << ">\n";
            assets_file << " " << 1.0 << "\n";
            assets_file << " scale ";
            assets_file << "<" << ellipsoid->GetEllipsoidGeometry().rad.x();
            assets_file << "," << ellipsoid->GetEllipsoidGeometry().rad.y();
            assets_file << "," << ellipsoid->GetEllipsoidGeometry().rad.z() << ">\n";
            assets_file << "}\n";

            // POV macro - end
            assets_file << "#end \n";
        }

        if (auto cylinder = std::dynamic_pointer_cast<ChCylinderShape>(shape)) {
            // POV macro to build the asset - begin
            assets_file << "#macro sh_" << (size_t)shape.get() << "()\n";

            // POV will make the sphere
            assets_file << "cylinder  {\n";

            assets_file << " <" << cylinder->GetCylinderGeometry().p1.x();
            assets_file << "," << cylinder->GetCylinderGeometry().p1.y();
            assets_file << "," << cylinder->GetCylinderGeometry().p1.z() << ">,\n";
            assets_file << " <" << cylinder->GetCylinderGeometry().p2.x();
            assets_file << "," << cylinder->GetCylinderGeometry().p2.y();
            assets_file << "," << cylinder->GetCylinderGeometry().p2.z() << ">,\n";
            assets_file << " " << cylinder->GetCylinderGeometry().rad << "\n";

            assets_file << "}\n";

            // POV macro - end
            assets_file << "#end \n";
        }

        if (auto box = std::dynamic_pointer_cast<ChBoxShape>(shape)) {
            // POV macro to build the asset - begin
            assets_file << "#macro sh_" << (size_t)shape.get() << "()\n";

            // POV will make the box
            assets_file << "union  {\n";
            assets_file << "box  {\n";

            assets_file << " <" << -box->GetBoxGeometry().Size.x();
            assets_file << "," << -box->GetBoxGeometry().Size.y();
            assets_file << "," << -box->GetBoxGeometry().Size.z() << ">\n";
            assets_file << " <" << box->GetBoxGeometry().Size.x();
            assets_file << "," << box->GetBoxGeometry().Size.y();
            assets_file << "," << box->GetBoxGeometry().Size.z() << ">\n";

            const auto& pos = shape_frame.GetPos();
            const auto& rot = shape_frame.GetRot();
            assets_file << " quatRotation(<" << rot.e0();
            assets_file << "," << rot.e1();
            assets_file << "," << rot.e2();
            assets_file << "," << rot.e3() << ">) \n";
            assets_file << " translate  <" << pos.x();
            assets_file << "," << pos.y();
            assets_file << "," << pos.z() << "> \n";

            assets_file << "}\n";  // end box
            assets_file << "}\n";  // end union

            // POV macro - end
            assets_file << "#end \n";
        }

        // Save materials used by this shape
        ExportMaterials(assets_file, shape->GetMaterials());
    }

    // Check if there are custom commands set for this physics item
    auto commands = m_custom_commands.find((size_t)item.get());
    if (commands != m_custom_commands.end()) {
        assets_file << "#macro cm_" << (size_t)item.get() << "()\n";
        assets_file << commands->second << "\n";
        assets_file << "#end \n";
    }
}

void ChPovRay::ExportMaterials(ChStreamOutAsciiFile& assets_file,
                               const std::vector<std::shared_ptr<ChVisualMaterial>>& materials) {
    for (const auto& mat : materials) {
        // Do nothing if the material was alrwady processed (because it is shared)
        // Otherwise, add the material to the cache list and process it
        if (m_pov_materials.find((size_t)mat.get()) != m_pov_materials.end())
            continue;
        m_pov_materials.insert({(size_t)mat.get(), mat});

        // POV macro to build the asset - begin
        assets_file << "#macro mt_" << (size_t)mat.get() << "()\n";

        // add POV  texture (changing the path to absolute to allow base_path different to the one of .exe)
        if (!mat->GetKdTexture().empty()) {
            auto rel_path = filesystem::path(mat->GetKdTexture());
            auto abs_path = rel_path.make_absolute().str();
            auto ext = rel_path.extension();
            if (ext == "jpg")
                ext = "jpeg";
            auto abs_path_no_ext = abs_path.substr(0, abs_path.size() - ext.size()); // strip .jpg etc. as if in "jpeg" mode, .jpg files give issues in POV

            assets_file << "texture { uv_mapping pigment { image_map {";
            assets_file << ext.c_str() << " ";
            assets_file << "\"" << abs_path_no_ext << "\"";
            assets_file << " }}}\n";
        }

        // add POV  pigment (only if no texture has been added, otherwise POV complains)
        if (mat->GetKdTexture().empty()) {
            const auto& color = mat->GetDiffuseColor();
            assets_file << "pigment {color rgbt <" << color.R << "," << color.G << "," << color.B << ","
                << 1 - mat->GetOpacity() << "> }\n";
        }

        // POV macro - end
        assets_file << "#end \n";
    }
}

void ChPovRay::ExportObjData(ChStreamOutAsciiFile& pov_file,
                             std::shared_ptr<ChPhysicsItem> item,
                             const ChFrame<>& parentframe) {
    pov_file << "union{\n";  // begin union

    // Scan visual shapes in the visual model
    for (const auto& shape_instance : item->GetVisualModel()->GetShapes()) {
        const auto& shape = shape_instance.first;

        // Process only "known" shapes (i.e., shapes that were included in the assets file)
        if (std::dynamic_pointer_cast<ChObjFileShape>(shape) || std::dynamic_pointer_cast<ChTriangleMeshShape>(shape) ||
            std::dynamic_pointer_cast<ChSphereShape>(shape) || std::dynamic_pointer_cast<ChEllipsoidShape>(shape) ||
            std::dynamic_pointer_cast<ChCylinderShape>(shape) || std::dynamic_pointer_cast<ChBoxShape>(shape)) {
            // Invoke the geometry macro
            pov_file << "sh_" << (size_t)shape.get() << "()\n";

            // Invoke the material macros
            for (const auto& mat : shape->GetMaterials()) {
                pov_file << "mt_" << (size_t)mat.get() << "()\n";
            }
        }
    }

    // Scan FEA visual shapes in the visual model
    for (const auto& shapeFEA : item->GetVisualModel()->GetShapesFEA()) {
        //// RADU TODO
    }

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
    auto commands = m_custom_commands.find((size_t)item.get());
    if (commands != m_custom_commands.end()) {
        pov_file << "cm_" << (size_t)item.get() << "()\n";
    }

    // Write the rotation and position
    if (!(parentframe.GetCoord() == CSYSNORM)) {
        pov_file << " quatRotation(<" << parentframe.GetRot().e0();
        pov_file << "," << parentframe.GetRot().e1();
        pov_file << "," << parentframe.GetRot().e2();
        pov_file << "," << parentframe.GetRot().e3() << ">) \n";
        pov_file << " translate  <" << parentframe.GetPos().x();
        pov_file << "," << parentframe.GetPos().y();
        pov_file << "," << parentframe.GetPos().z() << "> \n";
    }

    pov_file << "}\n";  // end union
}

// This function is used at each timestep to export data formatted in a way that it can be load with the POV scripts
// generated by ExportScript(). The generated filename must be set at the beginning of the animation via
// SetOutputDataFilebase(), and then a number is automatically appended and incremented at each ExportData(), e.g.,
//  state0001.dat, state0002.dat, ...
// The user should call this function in the while() loop of the simulation, once per frame.
void ChPovRay::ExportData() {
    char fullname[200];
    sprintf(fullname, "%s%05d", out_data_filename.c_str(), framenumber);
    ExportData(out_path + "/" + std::string(fullname));
}

void ChPovRay::ExportData(const std::string& filename) {
    // Regenerate the list of objects that need POV rendering
    UpdateRenderList();

    // If using a single-file asset, update it (in case new visual shapes were created during simulation)
    if (single_asset_file) {
        // open asset file in append mode
        std::string assets_filename = out_script_filename + ".assets";
        ChStreamOutAsciiFile assets_file((base_path + assets_filename).c_str(), std::ios::app);
        // populate assets (already present assets will not be appended)
        ExportAssets(assets_file);
    }

    // Generate the nnnn.dat and nnnn.pov files:
    try {
        char pathdat[200];
        sprintf(pathdat, "%s.dat", filename.c_str());
        ChStreamOutAsciiFile data_file((base_path + filename + ".dat").c_str());
        ChStreamOutAsciiFile pov_file((base_path + filename + ".pov").c_str());

        camera_found_in_assets = false;

        // If embedding assets in the .pov file:
        if (!single_asset_file) {
            m_pov_shapes.clear();
            ExportAssets(pov_file);
        }

        // Write custom data commands, if provided by the user
        if (custom_data.size() > 0) {
            pov_file << "// Custom user-added script: \n\n";
            pov_file << custom_data;
            pov_file << "\n\n";
        }

        // Tell POV to open the .dat file, that could be used by
        // ChParticleClones for efficiency (xyz raw data with center of particles will
        // be saved in dat and load using a #while POV loop, helping to reduce size of .pov file)
        pov_file << "#declare dat_file = \"" << (filename + ".dat").c_str() << "\"\n";
        pov_file << "#fopen MyDatFile dat_file read \n\n";

        // Save time-dependent data for the geometry of objects in ...nnnn.POV and in ...nnnn.DAT file
        for (const auto& item : m_items) {
            // Nothing to do if no visual model attached to this physics item
            if (!item->GetVisualModel())
                continue;

            // saving a body?
            if (const auto& body = std::dynamic_pointer_cast<ChBody>(item)) {
                // Get the current coordinate frame of the i-th object
                ChCoordsys<> assetcsys = CSYSNORM;
                const ChFrame<>& bodyframe = body->GetFrame_REF_to_abs();
                assetcsys = bodyframe.GetCoord();

                // Dump the POV macro that generates the contained asset(s) tree
                ExportObjData(pov_file, body, bodyframe);

                // Show body COG?
                if (COGs_show) {
                    const ChCoordsys<>& cogcsys = body->GetFrame_COG_to_abs().GetCoord();
                    pov_file << "sh_csysCOG(";
                    pov_file << cogcsys.pos.x() << "," << cogcsys.pos.y() << "," << cogcsys.pos.z() << ",";
                    pov_file << cogcsys.rot.e0() << "," << cogcsys.rot.e1() << "," << cogcsys.rot.e2() << ","
                             << cogcsys.rot.e3() << ",";
                    pov_file << COGs_size << ")\n";
                }
                // Show body frame ref?
                if (frames_show) {
                    pov_file << "sh_csysFRM(";
                    pov_file << assetcsys.pos.x() << "," << assetcsys.pos.y() << "," << assetcsys.pos.z() << ",";
                    pov_file << assetcsys.rot.e0() << "," << assetcsys.rot.e1() << "," << assetcsys.rot.e2() << ","
                             << assetcsys.rot.e3() << ",";
                    pov_file << frames_size << ")\n";
                }
            }

            // saving a cluster of particles?
            if (const auto& clones = std::dynamic_pointer_cast<ChParticleCloud>(item)) {
                pov_file << " \n";
                // pov_file << "union{\n";
                pov_file << "#declare Index = 0; \n";
                pov_file << "#while(Index < " << clones->GetNparticles() << ") \n";
                pov_file << "  #read (MyDatFile, apx, apy, apz, aq0, aq1, aq2, aq3) \n";
                pov_file << "  union{\n";
                ChFrame<> nullframe(CSYSNORM);
                ExportObjData(pov_file, clones, nullframe);
                pov_file << "  quatRotation(<aq0,aq1,aq2,aq3>)\n";
                pov_file << "  translate(<apx,apy,apz>)\n";
                pov_file << "  }\n";
                pov_file << "  #declare Index = Index + 1; \n";
                pov_file << "#end \n";
                // pov_file << "} \n";

                // Loop on all particle clones
                for (unsigned int m = 0; m < clones->GetNparticles(); ++m) {
                    // Get the current coordinate frame of the i-th particle
                    ChCoordsys<> assetcsys = CSYSNORM;
                    assetcsys = clones->GetParticle(m).GetCoord();

                    data_file << assetcsys.pos.x() << ", ";
                    data_file << assetcsys.pos.y() << ", ";
                    data_file << assetcsys.pos.z() << ", ";
                    data_file << assetcsys.rot.e0() << ", ";
                    data_file << assetcsys.rot.e1() << ", ";
                    data_file << assetcsys.rot.e2() << ", ";
                    data_file << assetcsys.rot.e3() << ", \n";
                }  // end loop on particles
            }

            //// RADU TODO: add capability for springs and dampers
            //// RADU TODO: why only MateGeneric links?!?

            // saving a ChLinkMateGeneric constraint?
            if (auto linkmate = std::dynamic_pointer_cast<ChLinkMateGeneric>(item)) {
                if (linkmate->GetBody1() && linkmate->GetBody2() && links_show) {
                    ChFrame<> frAabs = linkmate->GetFrame1() >> *linkmate->GetBody1();
                    ChFrame<> frBabs = linkmate->GetFrame2() >> *linkmate->GetBody2();
                    pov_file << "sh_csysFRM(";
                    pov_file << frAabs.GetPos().x() << "," << frAabs.GetPos().y() << "," << frAabs.GetPos().z() << ",";
                    pov_file << frAabs.GetRot().e0() << "," << frAabs.GetRot().e1() << "," << frAabs.GetRot().e2()
                             << "," << frAabs.GetRot().e3() << ",";
                    pov_file << links_size * 0.7 << ")\n";  // smaller, as 'slave' csys.
                    pov_file << "sh_csysFRM(";
                    pov_file << frBabs.GetPos().x() << "," << frBabs.GetPos().y() << "," << frBabs.GetPos().z() << ",";
                    pov_file << frBabs.GetRot().e0() << "," << frBabs.GetRot().e1() << "," << frBabs.GetRot().e2()
                             << "," << frBabs.GetRot().e3() << ",";
                    pov_file << links_size << ")\n";
                }
            }

            // saving an FEA mesh?
            if (auto fea_mesh = std::dynamic_pointer_cast<fea::ChMesh>(item)) {
                ExportObjData(pov_file, fea_mesh, ChFrame<>());
            }

        }  // end loop on objects

        // #) saving contacts ?
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

        // If a camera have been found in assets, create it and override the default one
        if (camera_found_in_assets) {
            pov_file << "camera { \n";
            if (camera_orthographic) {
                pov_file << " orthographic \n";
                pov_file << " right x * " << (camera_location - camera_aim).Length() << " * tan ((( " << camera_angle
                         << " *0.5)/180)*3.14) \n";
                pov_file << " up y * image_height/image_width * " << (camera_location - camera_aim).Length()
                         << " * tan (((" << camera_angle << "*0.5)/180)*3.14) \n";
                ChVector<> mdir = (camera_aim - camera_location) * 0.00001;
                pov_file << " direction <" << mdir.x() << "," << mdir.y() << "," << mdir.z() << "> \n";
            } else {
                pov_file << " right -x*image_width/image_height \n";
                pov_file << " angle " << camera_angle << " \n";
            }
            pov_file << " location <" << camera_location.x() << "," << camera_location.y() << "," << camera_location.z()
                     << "> \n"
                     << " look_at <" << camera_aim.x() << "," << camera_aim.y() << "," << camera_aim.z() << "> \n"
                     << " sky <" << camera_up.x() << "," << camera_up.y() << "," << camera_up.z() << "> \n";
            pov_file << "}\n\n\n";
        }

        // At the end of the .pov file, remember to close the .dat
        pov_file << "\n\n#fclose MyDatFile \n";
    } catch (const ChException&) {
        char error[400];
        sprintf(error, "Can't save data into file %s.pov (or .dat)", filename.c_str());
        throw(ChException(error));
    }

    // Increment the number of the frame.
    framenumber++;
}

}  // end namespace postprocess
}  // end namespace chrono
