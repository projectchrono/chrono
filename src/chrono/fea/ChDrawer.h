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

#ifndef CHDRAWER_H
#define CHDRAWER_H

#include "chrono/fea/ChVisualDataExtractor.h"
#include "chrono/fea/ChFieldData.h"
#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"

namespace chrono {
namespace fea {



/// @addtogroup chrono_fea
/// @{




/// Generic finite element drawing interface. 
/// The idea is that, after you implement some element inherited from ChFieldElement, you can (optionally)
/// implement a concrete class inherited from this ChDrawer, where you take care of how the element 
/// is drawn on the screen. 
/// Note that methods of this class will be most often called indirectly, through the ChElementDrawerDispatcher
/// that takes care of invoking the proper drawing functions depending on the type of the element; for this reason
/// your ChDrawer classes must be registered in such dispatcher via RegisterDrawer(..) for the element to appear on the screen.

class ChDrawer {
public:
    virtual ~ChDrawer() = default;

    /// This method takes a ChFieldElement and increments the counters of total used
    /// number of triangle vertexes, number of triangles, number of normals, used in 
    /// a triangle mesh that shows the shape of the finite element. This function is 
    /// called on all finite elements, to get the total amount of vertexes, triangles etc.
    /// in a ChTriangleMeshConnected that will be used to show finite elements on the screen.
    
    virtual void IncrementBufferSizes(ChFieldElement& melement,
        size_t& num_vertexes,
        size_t& num_triangles,
        size_t& num_normals) const = 0;

    /// This method takes a ChFieldElement and, given a ChTriangleMeshConnected& mmesh, does this:
    /// 1) per each triangle from tri_offset sets the indexes in mmesh.GetIndicesVertexes() to 3 triangle vertexes;
    /// 2) per each triangle from tri_offset sets the indexes in mmesh.GetIndicesNormals() to 3 triangle normals;
    /// 3) per each normal from norm_offset sets its value in mmesh.GetCoordsNormals() (can be automated via ChDrawer::AutoComputeNormals());
    /// Assumes there is no need to update the position of vertexes in mmesh.GetCoordsVertices() because already update by
    /// the caller, that sets the first N triangle vertices as the N finite element nodes - and same for N colors in mmesh.GetCoordsColors(),
    /// and N scalars/vectors/etc in myproperty.data of all properties in mmesh.GetPropertiesPerVertex(). However, if one has more 
    /// triangle vertexes than N element nodes, there could be an additional task:
    /// 4) (optional) per each vertex beyond the N element nodes, interpolate position/color/properties by looking at values 
    /// already updated in the position/color/properties of the first N vertexes of the mmesh, starting from vert_offset.
    
    virtual void UpdateBuffers(ChFieldElement& melement, 
        ChTriangleMeshConnected& mmesh, 
        size_t& vert_offset,
        size_t& tri_offset,
        size_t& norm_offset) const = 0;

protected:

    /// This is a helper function that can be called at the end of each element's UpdateBuffers(),
    /// to avoid setting all normals with ad-hoc formulas. The ratio is that AutoComputeNormals() just
    /// iterates through all triangles, compute a auxiliary triangle normal, then average those auxiliary
    /// triangle normals to get the normals at vertexes - if vertexes share normals. In most cases this 
    /// gives a good result and auto-smoothes curved surfaces avoiding the faceted look. Note that if you
    /// do not want triangle edges tobe 'smoothed', as in the case of hexahedrons where the square edges must be
    /// sharp and the edges inside the eight faces must be smoothed, in IncrementBufferSizes() and UpdateBuffers() you must  
    /// setup normal indices so that triangles joined at a sharp edge do NOT share normals at the two edge vertices;
    /// that's why ChDrawerHexahedron_8 assumes 24 independent normals, as 4 per face x 6 faces, not 1 per the 8 vertices.
    void AutoComputeNormals(
                    ChTriangleMeshConnected& mmesh,
                    const size_t tri_offset,
                    const size_t norm_offset,
                    const size_t num_triangles,
                    const size_t num_normals
    ) const {
        for (unsigned int nn = 0; nn < num_normals; ++nn) {
            mmesh.GetCoordsNormals()[norm_offset + nn] = ChVector3d(0, 0, 0);
        }
        for (unsigned int nt = 0; nt < num_triangles; ++nt) {
            auto vert_indexes = mmesh.GetIndicesVertexes()[tri_offset + nt];
            auto norm_indexes = mmesh.GetIndicesNormals()[tri_offset + nt];
            ChVector3d tnorm = Vcross(mmesh.GetCoordsVertices()[vert_indexes.y()] - mmesh.GetCoordsVertices()[vert_indexes.x()],
                mmesh.GetCoordsVertices()[vert_indexes.z()] - mmesh.GetCoordsVertices()[vert_indexes.x()])
                .GetNormalized();
            mmesh.GetCoordsNormals()[norm_indexes.x()] += tnorm;
            mmesh.GetCoordsNormals()[norm_indexes.y()] += tnorm;
            mmesh.GetCoordsNormals()[norm_indexes.z()] += tnorm;
        }
        for (unsigned int nn = 0; nn < num_normals; ++nn) {
            mmesh.GetCoordsNormals()[norm_offset + nn] = mmesh.GetCoordsNormals()[norm_offset + nn].GetNormalized();
        }
    }
};

/// Drawing function to draw a ChFieldElementTetrahedron_4 finite element.
/// Allows a 3D visualization of the finite element in real time rendering, 
/// raytraced postprocessing, etc.

class ChDrawerTetrahedron_4 : public ChDrawer {
public:
    virtual void IncrementBufferSizes(ChFieldElement& melement,
                                size_t& num_vertexes,
                                size_t& num_triangles,
                                size_t& num_normals) const {
        num_vertexes    += 4;
        num_triangles   += 4;
        num_normals     += 4;
    };

    virtual void UpdateBuffers(ChFieldElement& melement, 
                                ChTriangleMeshConnected& mmesh, 
                                size_t& vert_offset,
                                size_t& tri_offset,
                                size_t& norm_offset) const {
        //ChFieldElementTetrahedron_4& element = static_cast<ChFieldElementTetrahedron_4&>(melement);

        // Setup triangles: vertex indexes per each triangle
        mmesh.GetIndicesVertexes()[tri_offset+0] = ChVector3i((int)vert_offset + 0, (int)vert_offset + 1, (int)vert_offset + 2);
        mmesh.GetIndicesVertexes()[tri_offset+1] = ChVector3i((int)vert_offset + 1, (int)vert_offset + 3, (int)vert_offset + 2);
        mmesh.GetIndicesVertexes()[tri_offset+2] = ChVector3i((int)vert_offset + 2, (int)vert_offset + 0, (int)vert_offset + 3);
        mmesh.GetIndicesVertexes()[tri_offset+3] = ChVector3i((int)vert_offset + 3, (int)vert_offset + 1, (int)vert_offset + 0);
        
        // Normal indexes per each triangle. Assuming 4 normals, one per tetrahedron face.
        ChVector3i inorm_offset = ChVector3i((int)norm_offset, (int)norm_offset, (int)norm_offset);
        mmesh.GetIndicesNormals()[tri_offset + 0] = ChVector3i(0, 0, 0) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 1] = ChVector3i(1, 1, 1) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 2] = ChVector3i(2, 2, 2) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 3] = ChVector3i(3, 3, 3) + inorm_offset;

        // Normals are recomputed from average of triangle normals, if shared between triangles:
        this->AutoComputeNormals(mmesh, tri_offset, norm_offset, 4, 4);

        tri_offset  += 4;
        vert_offset += 4;
        norm_offset += 4;
    }
};

/// Drawing function to draw a ChFieldElementHexahedron_8 finite element, for
/// the 3D visualization of the finite element in real time rendering, postprocessing, etc.
/// Each of the 8 faces is made with 2 triangles. 

class ChDrawerHexahedron_8 : public ChDrawer {
public:
    virtual void IncrementBufferSizes(ChFieldElement& melement,
                                size_t& num_vertexes,
                                size_t& num_triangles,
                                size_t& num_normals) const {
        num_vertexes  +=  8;
        num_triangles += 12;
        num_normals   += 24;
    };

    virtual void UpdateBuffers( ChFieldElement& melement, 
                                ChTriangleMeshConnected& mmesh, 
                                size_t& vert_offset,
                                size_t& tri_offset,
                                size_t& norm_offset) const {
        //ChFieldElementHexahedron_8& element = static_cast<ChFieldElementHexahedron_8&>(melement);

        // Setup triangles: vertex indexes per each triangle
        ChVector3i ivert_offset((int)vert_offset, (int)vert_offset, (int)vert_offset);
        mmesh.GetIndicesVertexes()[tri_offset + 0] = ChVector3i(0, 2, 1) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 1] = ChVector3i(0, 3, 2) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 2] = ChVector3i(4, 5, 6) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 3] = ChVector3i(4, 6, 7) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 4] = ChVector3i(0, 7, 3) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 5] = ChVector3i(0, 4, 7) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 6] = ChVector3i(0, 5, 4) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 7] = ChVector3i(0, 1, 5) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 8] = ChVector3i(3, 7, 6) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 9] = ChVector3i(3, 6, 2) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 10] = ChVector3i(2, 5, 1) + ivert_offset;
        mmesh.GetIndicesVertexes()[tri_offset + 11] = ChVector3i(2, 6, 5) + ivert_offset;

        // Normal indexes per each triangle. Assuming 24 normals, 4 per each hexa face.
        ChVector3i inorm_offset = ChVector3i((int)norm_offset, (int)norm_offset, (int)norm_offset);
        mmesh.GetIndicesNormals()[tri_offset + 0] = ChVector3i(0, 2, 1) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 1] = ChVector3i(0, 3, 2) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 2] = ChVector3i(4, 5, 6) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 3] = ChVector3i(4, 6, 7) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 4] = ChVector3i(8, 9, 10) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 5] = ChVector3i(8, 11, 9) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 6] = ChVector3i(12, 13, 14) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 7] = ChVector3i(12, 15, 13) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 8] = ChVector3i(16, 18, 17) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 9] = ChVector3i(16, 17, 19) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 10] = ChVector3i(20, 21, 23) + inorm_offset;
        mmesh.GetIndicesNormals()[tri_offset + 11] = ChVector3i(20, 22, 21) + inorm_offset;
        
        // Normals are recomputed from average of triangle normals, if shared between triangles:
        this->AutoComputeNormals(mmesh, tri_offset, norm_offset, 12, 24);
        
        vert_offset += 8;
        tri_offset  += 12;
        norm_offset += 24;
    }
};


////////////////////////////////////////////////////////////////////////////////////////////////



/// This object takes care of invoking the proper ChDrawer methods, like UpdateBuffers(),
/// depending on the type of the element. The reason is that the UpdateBuffers() could be
/// a virtual method from a ChFieldElement base class, but we prefer not to pollute the
/// ChFieldElement base class with stuff that is related to drawing into a
/// ChTriangleMeshConnected. So this ChElementDrawerDispatcher class works like an
/// 'extension' of the polymorphism of finite elements, at the cost of a lookup into hash
/// map that associates element type_index to the corresponding ChDrawerXXYY. 
/// [To do performance optimization: since this is invoked many times iterating on a stl 
/// container, and most likely elements are stored in sequences of same type_index, the 
/// hash lookup in drawers.find(..) could be shortcut by first testing if the last 
/// key-value can be reused.]

class ChElementDrawerDispatcher {
private:
    std::unordered_map<std::type_index, std::unique_ptr<ChDrawer>> drawers;

public:
    template<typename T>
    void RegisterDrawer(std::unique_ptr<ChDrawer> calc) {
        drawers[typeid(T)] = std::move(calc);
    }

    /// This method takes a ChFieldElement and increments the counters of total used
    /// number of triangle vertexes, number of triangles, number of normals, used in 
    /// a triangle mesh that shows the shape of the finite element. This function is 
    /// called on all finite elements, to get the total amount of vertexes, triangles etc.
    /// in a ChTriangleMeshConnected that will be used to show finite elements on the screen.
    void IncrementBufferSizes(ChFieldElement& melement,
                        size_t& num_vertexes,
                        size_t& num_triangles,
                        size_t& num_normals) {
        auto it = drawers.find(typeid(melement));
        if (it != drawers.end()) {
            it->second->IncrementBufferSizes(melement, num_vertexes, num_triangles, num_normals);
        }
    }

    /// This method takes a ChFieldElement and, given a ChTriangleMeshConnected& mmesh, does this:
    /// 1) per each triangle from tri_offset sets the indexes in mmesh.GetIndicesVertexes() to 3 triangle vertexes;
    /// 2) per each triangle from tri_offset sets the indexes in mmesh.GetIndicesNormals() to 3 triangle normals;
    /// 3) per each normal from norm_offset sets its value in mmesh.GetCoordsNormals() (can be automated via ChDrawer::AutoComputeNormals());
    /// Assumes there is no need to update the position of vertexes in mmesh.GetCoordsVertices() because already update by
    /// the caller, that sets the first N triangle vertices as the N finite element nodes - and same for N colors in mmesh.GetCoordsColors(),
    /// and N scalars/vectors/etc in myproperty.data of all properties in mmesh.GetPropertiesPerVertex(). However, if one has more 
    /// triangle vertexes than N element nodes, there could be an additional task:
    /// 4) (optional) per each vertex beyond the N element nodes, interpolate position/color/properties by looking at values 
    /// already updated in the position/color/properties of the first N vertexes of the mmesh, starting from vert_offset.
    void UpdateBuffers(ChFieldElement& melement, 
                        ChTriangleMeshConnected& mmesh, 
                        size_t& vert_offset,
                        size_t& tri_offset,
                        size_t& norm_offset) {
        auto it = drawers.find(typeid(melement));
        if (it != drawers.end()) {
            it->second->UpdateBuffers(melement,mmesh, vert_offset, tri_offset, norm_offset);
        }
    }
};


//////////////////////////////////////////////////////////////////////////////////////////////////////////


/// Attach this visual asset to a ChPhysicsItem (ex.a ChDomain) in order to provide an automatic visualization 
/// of Gauss material points and finite element nodes, as 3D glyps (arrows, dots, etc.).
/// If the visualization system supports glyphs, these will be rendered (ex. in Irrlicht 3d view,
/// but also in Blender postprocessing etc.
/// The system is modular and expandable, so that you can visualize properties such as stress, strain,
/// speed, acceleration, temperature etc., by adding ChVisualDataExtractor objects to this visual asset.

class ChVisualDomainGlyphs : public ChGlyphs {
public:

    ChVisualDomainGlyphs(std::shared_ptr<ChDomain> adomain) : mdomain(adomain) {
        is_mutable = true;

        ChPropertyColor my_colors;
        my_colors.name = "color";
        this->AddProperty(my_colors);
        this->colors = &((ChPropertyColor*)(m_properties.back()))->data;

        i_vector_prop_colorized = -1;
        i_scalar_prop_colorized = -1;
        use_colormap = false;

        draw_materialpoints = false;
        draw_nodes = false;
    }
    virtual ~ChVisualDomainGlyphs() {}


    /// Attach scalar property extractor. Also turns the glyph rendering into ChGlyph::GLYPH_POINT mode. 
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(const ChVisualDataExtractorScalarBase& mextractor, double min = 0, double max = 1, std::string mname = "Scalar") {
        auto owned_extractor = std::dynamic_pointer_cast<ChVisualDataExtractorScalarBase>(std::shared_ptr<ChVisualDataExtractor>(mextractor.clone()));
        auto T_property = new ChPropertyScalar;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_scalar_properties.push_back({ owned_extractor, T_property });
        this->m_properties.push_back(T_property);

        this->SetDrawMode(GLYPH_POINT);

        i_scalar_prop_colorized = (int)extractors_scalar_properties.size() - 1;
        i_vector_prop_colorized = -1;

        if (owned_extractor->IsDataAtMaterialpoint()) this->draw_materialpoints = true;
        if (owned_extractor->IsDataAtNode()) this->draw_nodes = true;
    }

    /// Attach vector property extractor. Also turns the glyph rendering into ChGlyph::GLYPH_POINT mode.
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(const ChVisualDataExtractorVectorBase& mextractor, double min = 0, double max = 1, std::string mname = "Vect") {
        auto owned_extractor = std::dynamic_pointer_cast<ChVisualDataExtractorVectorBase>(std::shared_ptr<ChVisualDataExtractor>(mextractor.clone()));
        auto T_property = new ChPropertyVector;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_vector_properties.push_back({ owned_extractor, T_property });
        this->m_properties.push_back(T_property);

        this->SetDrawMode(GLYPH_VECTOR);
        this->vectors = &T_property->data;

        i_scalar_prop_colorized = -1;
        i_vector_prop_colorized = (int)extractors_vector_properties.size() - 1;

        if (owned_extractor->IsDataAtMaterialpoint()) this->draw_materialpoints = true;
        if (owned_extractor->IsDataAtNode()) this->draw_nodes = true;
    }

    /// Attach 3x3 matrix property extractor. Also turns the glyph rendering into ChGlyph::GLYPH_TENSOR mode.
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(const ChVisualDataExtractorMatrix33Base& mextractor, double min = 0, double max = 1, std::string mname = "Matr") {
        auto owned_extractor = std::dynamic_pointer_cast<ChVisualDataExtractorMatrix33Base>(std::shared_ptr<ChVisualDataExtractor>(mextractor.clone()));
        auto T_vproperty = new ChPropertyVector;
        T_vproperty->min = min;
        T_vproperty->max = max;
        T_vproperty->name = mname;
        this->m_properties.push_back(T_vproperty);
        auto T_qproperty = new ChPropertyQuaternion;
        T_qproperty->name = mname;
        this->m_properties.push_back(T_qproperty);

        this->extractors_tensor_properties.push_back({ owned_extractor, {T_qproperty,T_vproperty} });

        this->SetDrawMode(GLYPH_TENSOR);
        this->eigenvalues = &T_vproperty->data;
        this->rotations = &T_qproperty->data;

        i_scalar_prop_colorized = -1;
        i_vector_prop_colorized = (int)extractors_vector_properties.size() - 1;

        if (owned_extractor->IsDataAtMaterialpoint()) this->draw_materialpoints = true;
        if (owned_extractor->IsDataAtNode()) this->draw_nodes = true;
    }

    // Set the extractor that gets the 3D position where to draw the glyph (ex. node spatial coords, 
    // for nonlinear deformation analysis). If not set, by default the drawing system will fall 
    // back to the 3D position of the node (material space reference coords). 
    void AddPositionExtractor(const ChVisualDataExtractorVectorBase& mextractor) {
        auto owned_extractor = std::dynamic_pointer_cast<ChVisualDataExtractorVectorBase>(std::shared_ptr<ChVisualDataExtractor>(mextractor.clone()));
        extractor_position = owned_extractor;
    }

    /// Set the colormap for rendering the property via falsecolor.  
    void SetColormap(const ChColormap& mcmap) {
        this->colormap = mcmap;
        use_colormap = true;
    }

    /// Set the color for rendering the glyphs as constant uniform color.
    /// This disables the colormap falsecolor rendering. 
    void SetColormap(const ChColor& col) {
        ChVisualShape::SetColor(col);
        use_colormap = false;
    }


protected:

    /// This Update() is called automatically by the parent ChPhysicsItem all time that 
    /// the system needs to update the visual assets.
    /// The main task of this Update is to generate/update the lists of glyps (vectors, poionts, etc.)
    /// that represents the visualization of the nodes and material points of the domain.
    
    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
        if (!mdomain)
            return;

        // Count num of glyphs and..
        unsigned int tot_glyphs = 0;
        if (this->draw_nodes) {
            /*
            for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                tot_glyphs += mdomain->GetField(ifield)->GetNumNodes();
            }
            */
            // Assume all fields of this domain have the same number of nodes (not necessarily true in a general context.. maybe enforce this?)
            tot_glyphs += mdomain->GetField(0)->GetNumNodes();
        }
        if (this->draw_materialpoints) {
            auto ele_iter = mdomain->CreateIteratorOnElements();
            while (!ele_iter->is_end()) {
                tot_glyphs += ele_iter->get_element()->GetNumQuadraturePoints();
                ele_iter->next();
            }
        }
        // ..resize the vector of points and all the vectors of properties
        this->Reserve(tot_glyphs);

        auto mcolor = this->GetColor();

        unsigned int i = 0;
        
        // Update from nodes
        if (this->draw_nodes) {

            // Fill the 3d positions vector in ChGlyphs::points  with the node positions (if possible, 
            // via extractor_position, otherwise falls back to ChNodeFEAfieldXYZ reference position)
            
            bool has_position_fetched = false;
            for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                i = 0;
                auto mfield = mdomain->GetField(ifield);

                auto node_iterator = mfield->CreateIteratorOnNodes();
                while (!node_iterator->is_end()) {
                    if (auto nodexyz = std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(node_iterator->get().first)) {

                        ChVector3d mpos = *nodexyz; // default node pos: the reference pos
                        if (extractor_position)
                            if (auto fetched_pos = extractor_position->Extract(&node_iterator->get().second)) {
                                mpos = fetched_pos.value();
                                has_position_fetched = true;
                            }
                        this->points[i] = mpos;
                    }
                    ++i;
                    node_iterator->next();
                }
                if (has_position_fetched)
                    break;
            }// end loop on fields


            // Fill the properties of field nodes:
            
            for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                i = 0;
                auto mfield = mdomain->GetField(ifield);

                auto node_iterator = mfield->CreateIteratorOnNodes();
                while (!node_iterator->is_end()) {
                    if (auto nodexyz = std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(node_iterator->get().first)) {

                        for (auto& mfetch_s : extractors_scalar_properties)
                            if (auto fetched_s = mfetch_s.first->Extract(&node_iterator->get().second))
                                mfetch_s.second->data[i] = fetched_s.value();
                        for (auto& mfetch_v : extractors_vector_properties)
                            if (auto fetched_v = mfetch_v.first->Extract(&node_iterator->get().second))
                                mfetch_v.second->data[i] = fetched_v.value();

                        (*this->colors)[i] = mcolor;
                    }
                    ++i;
                    node_iterator->next();
                }

            }// end loop on fields

        }

        // Update from material points
        if (this->draw_materialpoints) {
            auto ele_iter = mdomain->CreateIteratorOnElements();
            while (!ele_iter->is_end()) {
                auto element = ele_iter->get_element();

                ChMatrixDynamic<double> Xhat(3, element->GetNumNodes());
                ChRowVectorDynamic<double> N(element->GetNumNodes());

                // build the Xhat matrix, with all node xyz positions as columns, Xhat=[x1|x2|x3|...]
                bool has_position_fetched = false;
                if (extractor_position) {
                    for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                        for (unsigned int inode = 0; inode < ele_iter->get_element()->GetNumNodes(); ++inode) {
                            if (auto fetched_s = extractor_position->Extract(ele_iter->get_data_per_node(inode, ifield))) {
                                Xhat.col(inode) = fetched_s.value().eigen();
                                has_position_fetched = true;
                            }
                        }
                        if (has_position_fetched)
                            break;
                    }
                }
                if (!has_position_fetched) {
                    for (unsigned int inode = 0; inode < ele_iter->get_element()->GetNumNodes(); ++inode) {
                        if (auto nodexyz = std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(ele_iter->get_element()->GetNode(inode))) {
                            Xhat.col(inode) = (*nodexyz).eigen();
                        }
                    }
                }

                for (int imatpoint = 0; imatpoint < element->GetNumQuadraturePoints(); ++imatpoint) {
                    ChVector3d eta;
                    double weight;
                    //ele_iter->get_data_per_matpoint(imatpoint);
                    element->GetQuadraturePointWeight(element->GetQuadratureOrder(), imatpoint, weight, eta);

                    element->ComputeN(eta, N);

                    this->points[i] = Xhat * N.transpose();  // position interpolated at quadrature point
                    (*this->colors)[i] = mcolor;

                    for (auto& mfetch_s : extractors_scalar_properties) {
                        if (mfetch_s.first->IsDataAtNode()) {
                            if (auto fetched_s = mfetch_s.first->Extract(ele_iter->get_data_per_matpoint(imatpoint)))
                                mfetch_s.second->data[i] = fetched_s.value();
                            if (auto fetched_s = mfetch_s.first->Extract(ele_iter->get_data_per_matpoint_aux(imatpoint)))
                                mfetch_s.second->data[i] = fetched_s.value();
                        }
                    }
                    for (auto& mfetch_v : extractors_vector_properties) {
                        if (auto fetched_v = mfetch_v.first->Extract(ele_iter->get_data_per_matpoint(imatpoint)))
                            mfetch_v.second->data[i] = fetched_v.value();
                        if (auto fetched_v = mfetch_v.first->Extract(ele_iter->get_data_per_matpoint_aux(imatpoint)))
                            mfetch_v.second->data[i] = fetched_v.value();
                    }
                    for (auto& mfetch_t : extractors_tensor_properties) {
                        auto fetched_t = mfetch_t.first->Extract(ele_iter->get_data_per_matpoint(imatpoint));
                        if (!fetched_t)
                             fetched_t = mfetch_t.first->Extract(ele_iter->get_data_per_matpoint_aux(imatpoint));
                        if (fetched_t) {
                            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(fetched_t.value());
                            ChMatrix33d Meigenvectors = solver.eigenvectors();
                            ChVector3d Meigenvalues = solver.eigenvalues();
                            if (Meigenvectors.determinant() < 0) {
                                Meigenvectors.col(2) *= -1.0;
                            }
                            mfetch_t.second.first->data[i] = Meigenvectors.GetQuaternion(); // rotation from eigenvectors
                            mfetch_t.second.second->data[i] = Meigenvalues;  // vector containing 3 eigenvalues
                        }    
                    }
                    ++i;
                }
                ele_iter->next();
            }
        }

        // Update glyph colors via colormap, filling the color vector, i.e. iterating into the .data of 
        // the flagged property, and storing the computed color in the .data of the "color" property that is added by default:
        if (use_colormap) {
            for (unsigned int ip = 0; ip < this->points.size(); ++ip) {
                if (i_scalar_prop_colorized >= 0) {
                    ChPropertyScalar* mprop = extractors_scalar_properties[i_scalar_prop_colorized].second;
                    (*this->colors)[ip] = colormap.Get(mprop->data[ip], mprop->min, mprop->max);
                }
                if (i_vector_prop_colorized >= 0) {
                    ChPropertyVector* mprop = extractors_vector_properties[i_vector_prop_colorized].second;
                    (*this->colors)[ip] = colormap.Get(mprop->data[ip].Length(), mprop->min, mprop->max);
                }
            }
        }

    }

    std::shared_ptr<ChVisualDataExtractorVectorBase> extractor_position;
    std::vector<std::pair<std::shared_ptr<ChVisualDataExtractorScalarBase>, ChPropertyScalar*> > extractors_scalar_properties;
    std::vector<std::pair<std::shared_ptr<ChVisualDataExtractorVectorBase>, ChPropertyVector*> > extractors_vector_properties;
    std::vector<std::pair<std::shared_ptr<ChVisualDataExtractorMatrix33Base>, std::pair<ChPropertyQuaternion*, ChPropertyVector*>> > extractors_tensor_properties;

    std::shared_ptr<ChDomain> mdomain;

    ChColormap colormap;
    bool use_colormap;

    int i_scalar_prop_colorized;
    int i_vector_prop_colorized;

    bool draw_materialpoints;
    bool draw_nodes;
};



//////////////////////////////////////////////////////////////////////////////////////////////////////////


/// Attach this visual asset to a ChPhysicsItem (ex.a ChDomain) in order to provide an automatic visualization 
/// of finite elements of a ChDomain, as 3D triangle meshes that can be rendered.
/// If the visualization system supports ChVisualShapeTriangleMesh, this will be rendered (ex. in Irrlicht 3d view,
/// but also in Blender postprocessing etc.)
/// The system is modular and expandable, so that you can visualize properties such as stress, strain,
/// speed, acceleration, temperature etc., by adding ChVisualDataExtractor objects to this visual asset.
/// Note that conversion of finite elements into triangle meshes is obtained through a modular expandable system
/// based on ChDrawer objects, registered into a ChElementDrawerDispatcher; if the user defines a new finite element "Foo",
/// then he also needs to implement the corresponding ChDrawerFoo class that generates its triangle mesh representation,
/// and he also needs to register ChDrawerFoo into ChElementDrawerDispatcher, otherwise the element will be invisible.

class ChVisualDomainMesh : public ChVisualShapeTriangleMesh {
public:
    ChVisualDomainMesh(std::shared_ptr<ChDomain> adomain) : mdomain(adomain) {
        is_mutable = true;

        i_vector_prop_colorized = -1;
        i_scalar_prop_colorized = -1;
        use_colormap = false;

        // If new ChDrawer... classes are developed, remember to register them in this
        // list: 
        elem_dispatcher.RegisterDrawer<ChFieldElementTetrahedron4>(std::make_unique<ChDrawerTetrahedron_4>());
        elem_dispatcher.RegisterDrawer<ChFieldElementHexahedron8>(std::make_unique<ChDrawerHexahedron_8>());
        //...
        shrink_elements = false;
        shrink_factor = 0.8;
    }
    virtual ~ChVisualDomainMesh() {}


    /// Attach a scalar property extractor. Every time step, the scalar values from the extractor will be fetched 
    /// and stored in the corresponding ChProperty attached to the triangle mesh, with proper interpolation.
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(const ChVisualDataExtractorScalarBase& mextractor, double min = 0, double max = 1, std::string mname = "Scalar") {
        auto owned_extractor = std::dynamic_pointer_cast<ChVisualDataExtractorScalarBase>(std::shared_ptr<ChVisualDataExtractor>(mextractor.clone()));
        auto T_property = new ChPropertyScalar;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_scalar_properties.push_back({ owned_extractor, T_property });
        this->GetMesh()->m_properties_per_vertex.push_back(T_property);

        i_scalar_prop_colorized = (int)extractors_scalar_properties.size() - 1;
        i_vector_prop_colorized = -1;
    }

    /// Attach a vector property extractor. Every time step, the vector values from the extractor will be fetched 
    /// and stored in the corresponding ChProperty attached to the triangle mesh, with proper interpolation.
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(const ChVisualDataExtractorVectorBase& mextractor, double min = 0, double max = 1, std::string mname = "Vect") {
        auto owned_extractor = std::dynamic_pointer_cast<ChVisualDataExtractorVectorBase>(std::shared_ptr<ChVisualDataExtractor>(mextractor.clone()));
        auto T_property = new ChPropertyVector;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_vector_properties.push_back({ owned_extractor, T_property });
        this->GetMesh()->m_properties_per_vertex.push_back(T_property);

        i_scalar_prop_colorized = -1;
        i_vector_prop_colorized = (int)extractors_vector_properties.size() - 1;
    }

    // Set the extractor that gets the 3D position where to draw the glyph (ex. node spatial coords, 
    // for nonlinear deformation analysis). If not set, by default the drawing system will fall 
    // back to the 3D position of the node (material space reference coords). 
    void AddPositionExtractor(const ChVisualDataExtractorVectorBase& mextractor) {
        auto owned_extractor = std::dynamic_pointer_cast<ChVisualDataExtractorVectorBase>(std::shared_ptr<ChVisualDataExtractor>(mextractor.clone()));
        extractor_position = owned_extractor;
    }

    /// Set the colormap for rendering the property via falsecolor.  
    void SetColormap(const ChColormap& mcmap) {
        this->colormap = mcmap;
        use_colormap = true;
    }

    /// Set the color for rendering the glyphs as constant uniform color.
    /// This disables the colormap falsecolor rendering. 
    void SetColormap(const ChColor& col) {
        ChVisualShape::SetColor(col);
        use_colormap = false;
    }

    /// Set shrinkage of elements during drawing.
    void SetShrinkElements(bool shrink, double factor) {
        this->shrink_elements = shrink;
        this->shrink_factor = factor;
    }

protected:

    /// This Update() is called automatically by the parent ChPhysicsItem all time that 
    /// the system needs to update the visual assets.
    /// The main task of this Update is to generate/update the triangle mesh that represents 
    /// the visualization of the finite elements of the domain.
    
    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
        if (!mdomain)
            return;

        size_t n_vertexes = 0;
        size_t n_triangles = 0;
        size_t n_normals = 0;
        
        // Resize buffers if necessary.
        auto el_iter = mdomain->CreateIteratorOnElements();
        while (!el_iter->is_end()) {
            
            elem_dispatcher.IncrementBufferSizes(*el_iter->get_element().get(), 
                n_vertexes, 
                n_triangles, 
                n_normals);

            el_iter->next();
        }
        this->GetMesh()->GetCoordsVertices().resize(n_vertexes);
        this->GetMesh()->GetCoordsColors().resize(n_vertexes);
        this->GetMesh()->GetIndicesVertexes().resize(n_triangles);
        this->GetMesh()->GetCoordsNormals().resize(n_normals);
        this->GetMesh()->GetIndicesNormals().resize(n_triangles);
        for (auto& mprop : GetMesh()->GetPropertiesPerVertex())
            mprop->SetSize(n_vertexes);

        auto mcolor = this->GetColor();
        for (auto& mcol : this->GetMesh()->GetCoordsColors())
            mcol = mcolor;

        // Loop on elements to generate & update the triangle mesh

        size_t offset_tri = 0;
        size_t offset_vert = 0;
        size_t offset_norm = 0;
        auto ele_iter = mdomain->CreateIteratorOnElements();
        while (!ele_iter->is_end()) {

            unsigned int ele_num_nodes = ele_iter->get_element()->GetNumNodes();

            // Given N nodes in the element, fill the first N values in the std::vector 
            // this->GetMesh()->GetCoordsVertices()  with the N positions (if possible, 
            // via extractor_position, otherwise falls back to ChNodeFEAfieldXYZ reference position)
            bool has_position_fetched = false;
            if (extractor_position) {
                for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                    for (unsigned int inode = 0; inode < ele_num_nodes; ++inode) {
                        if (auto fetched_s = extractor_position->Extract(ele_iter->get_data_per_node(inode, ifield))) {
                            this->GetMesh()->GetCoordsVertices()[offset_vert + inode] = fetched_s.value();
                            has_position_fetched = true;
                        }
                    }
                    if (has_position_fetched)
                        break;
                }
            }
            if (!has_position_fetched) {
                for (unsigned int inode = 0; inode < ele_num_nodes; ++inode) {
                    if (auto nodexyz = std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(ele_iter->get_element()->GetNode(inode))) {
                        this->GetMesh()->GetCoordsVertices()[offset_vert + inode] = *nodexyz;
                    }
                }
            }

            if (shrink_elements) {
                ChVector3d vc(0, 0, 0);
                for (unsigned int inode = 0; inode < ele_num_nodes; ++inode)
                    vc += this->GetMesh()->GetCoordsVertices()[offset_vert + inode];
                vc = vc * (1.0 / ele_num_nodes);  // average, center of element
                for (unsigned int inode = 0; inode < ele_num_nodes; ++inode)
                    this->GetMesh()->GetCoordsVertices()[offset_vert + inode] = vc + shrink_factor * (this->GetMesh()->GetCoordsVertices()[offset_vert + inode] - vc);
            }

            // Given N nodes in the element, fill the first N values in the std::vector 
            // embedded in this->GetMesh()->GetPropertiesPerVertex()  with the N values of property
            for (auto& mfetch_s : extractors_scalar_properties) {
                for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                    for (unsigned int inode = 0; inode < ele_num_nodes; ++inode) {
                        if (auto fetched_s = mfetch_s.first->Extract(ele_iter->get_data_per_node(inode, ifield))) {
                            mfetch_s.second->data[offset_vert + inode] = fetched_s.value();
                        }
                    }
                    // TEST: EXTRAPOLATE FROM GAUSSPOINTS - fast method
                    if (mfetch_s.first->Extract(ele_iter->get_data_per_matpoint_aux(0))) { // bypass if type of extractor & domain not matching
                        ChRowVectorDynamic<> V_num(ele_iter->get_element()->GetNumNodes()); V_num.setZero();
                        ChRowVectorDynamic<> V_den(ele_iter->get_element()->GetNumNodes()); V_den.setZero();
                        bool extrapolate_gp = true;
                        for (int imatpoint = 0; imatpoint < ele_iter->get_element()->GetNumQuadraturePoints(); ++imatpoint) {
                            ChVector3d eta;
                            double weight;
                            ele_iter->get_element()->GetQuadraturePointWeight(ele_iter->get_element()->GetQuadratureOrder(), imatpoint, weight, eta);
                            ChRowVectorDynamic<> N;
                            ele_iter->get_element()->ComputeN(eta, N);
                            if (auto fetched_s = mfetch_s.first->Extract(ele_iter->get_data_per_matpoint_aux(imatpoint))) {
                                V_num += weight * N * fetched_s.value();
                                V_den += weight * N;
                            }
                            else {
                                extrapolate_gp = false;
                                break;
                            }
                        }
                        if (extrapolate_gp) {
                            for (unsigned int inode = 0; inode < ele_num_nodes; ++inode) {
                                mfetch_s.second->data[offset_vert + inode] = V_num(inode) / V_den(inode);
                            }
                        }
                    }
                    // END GAUSSPOINT EXTRAPOLATION
                }
            }
            for (auto& mfetch_s : extractors_vector_properties) {
                for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                    for (unsigned int inode = 0; inode < ele_num_nodes; ++inode) {
                        if (auto fetched_s = mfetch_s.first->Extract(ele_iter->get_data_per_node(inode, ifield))) {
                            mfetch_s.second->data[offset_vert + inode] = fetched_s.value();
                        }
                    }
                }
            }

            // Setup and update the triangle mesh, and interpolate nodal properties to
            // triangle vertexes of mesh (if any) that are not at the nodes.
            elem_dispatcher.UpdateBuffers(*ele_iter->get_element().get(), 
                *this->GetMesh().get(), 
                offset_vert, 
                offset_tri, 
                offset_norm);

            ele_iter->next();

        } // end loop on elements

        // Update mesh colors via colormap, filling the color vector, i.e. iterating into the .data of 
        // the flagged property, and storing the computed color in the .data of the "color" property that is added by default:
        if (use_colormap) {
            for (unsigned int ip = 0; ip < this->GetMesh()->GetCoordsVertices().size(); ++ip) {
                if (i_scalar_prop_colorized >= 0) {
                    ChPropertyScalar* mprop = extractors_scalar_properties[i_scalar_prop_colorized].second;
                    this->GetMesh()->GetCoordsColors()[ip] = colormap.Get(mprop->data[ip], mprop->min, mprop->max);
                }
                if (i_vector_prop_colorized >= 0) {
                    ChPropertyVector* mprop = extractors_vector_properties[i_vector_prop_colorized].second;
                    this->GetMesh()->GetCoordsColors()[ip] = colormap.Get(mprop->data[ip].Length(), mprop->min, mprop->max);
                }
            }
        }


    }

    std::shared_ptr<ChVisualDataExtractorVectorBase> extractor_position;
    std::vector<std::pair<std::shared_ptr<ChVisualDataExtractorScalarBase>, ChPropertyScalar*> > extractors_scalar_properties;
    std::vector<std::pair<std::shared_ptr<ChVisualDataExtractorVectorBase>, ChPropertyVector*> > extractors_vector_properties;

    std::shared_ptr<ChDomain> mdomain;

    ChElementDrawerDispatcher elem_dispatcher;

    ChColormap colormap;
    bool use_colormap;

    int i_scalar_prop_colorized;
    int i_vector_prop_colorized;

    bool shrink_elements;
    double shrink_factor;
};


/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
