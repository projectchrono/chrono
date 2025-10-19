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

#ifndef CHVISUALDATAEXTRACTOR_H
#define CHVISUALDATAEXTRACTOR_H

#include <optional>
#include "chrono/core/ChTensors.h"
#include "chrono/fea/ChFieldData.h"
#include "chrono/fea/ChDomain.h"
#include "chrono/fea/ChNodeFEAfieldXYZ.h"
#include "chrono/fea/ChFieldElementHexahedron8.h"
#include "chrono/fea/ChFieldElementTetrahedron4.h"

namespace chrono {
namespace fea {



/// @addtogroup chrono_fea
/// @{


class ChVisualDataExtractor {
};

template <class T_field_data>
class ChVisualDataExtractorImpl : public ChVisualDataExtractor {
};

class ChVisualDataExtractorScalarBase {
public:
    std::optional<double> virtual Extract(ChFieldData* data) const = 0;
};

class ChVisualDataExtractorVectorBase {
public:
    std::optional<ChVector3d> virtual Extract(ChFieldData* data) const = 0;
};

class ChVisualDataExtractorTensorBase {
public:
    std::optional<ChMatrix33d> virtual Extract(ChFieldData* data) const = 0;
};

class ChVisualDataExtractorQuaternionBase {
public:
    std::optional<ChQuaternion<double>> virtual Extract(ChFieldData* data) const = 0;
};


template <class T_field_data>
class ChVisualDataExtractorScalar : public ChVisualDataExtractorImpl<T_field_data>, public ChVisualDataExtractorScalarBase {
public:
    std::optional<double> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual double ExtractImpl(const T_field_data* fdata) const = 0;
};

template <class T_field_data>
class ChVisualDataExtractorVector : public ChVisualDataExtractorImpl<T_field_data>, public ChVisualDataExtractorVectorBase {
public:
    std::optional<ChVector3d> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual ChVector3d ExtractImpl(const T_field_data* fdata) const = 0;
};

template <class T_field_data>
class ChVisualDataExtractorTensor : public ChVisualDataExtractorImpl<T_field_data>, public ChVisualDataExtractorTensorBase {
public:
    std::optional<ChMatrix33d> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual ChMatrix33d ExtractImpl(const T_field_data* fdata) const = 0;
};

template <class T_field_data>
class ChVisualDataExtractorQuaternion : public ChVisualDataExtractorImpl<T_field_data>, public ChVisualDataExtractorQuaternionBase {
public:
    std::optional<ChQuaternion<double>> Extract(ChFieldData* data) const override {
        if (const auto* typed_data = dynamic_cast<T_field_data*>(data)) {
            return this->ExtractImpl(typed_data);
        }
        return std::nullopt;
    }
    virtual ChQuaternion<double> ExtractImpl(const T_field_data* fdata) const = 0;
};

class ChVisualDataExtractorVectorComponent : public ChVisualDataExtractorScalarBase {
public:
    ChVisualDataExtractorVectorComponent(std::shared_ptr<ChVisualDataExtractorVectorBase> mvector_extractor, int n_component) :
        vector_extractor(mvector_extractor),
        id_component(n_component)
    {};
    std::optional<double> Extract(ChFieldData* data) const override {
        if (auto retv = vector_extractor->Extract(data)) {
            if (id_component == 0)
                return retv.value().x();
            if (id_component == 1)
                return retv.value().y();
            if (id_component == 2)
                return retv.value().z();
        }
        return std::nullopt;
    }
    int id_component; // 0=x, 1=y, 2=z;
    std::shared_ptr<ChVisualDataExtractorVectorBase> vector_extractor;
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////


class ChVisualDataExtractorPos : public ChVisualDataExtractorVector<ChFieldDataPos3D> {
    virtual ChVector3d ExtractImpl(const ChFieldDataPos3D* fdata)  const override {
        return fdata->GetPos();
    }
};
class ChVisualDataExtractorPosDt : public ChVisualDataExtractorVector<ChFieldDataPos3D> {
    virtual ChVector3d ExtractImpl(const ChFieldDataPos3D* fdata)  const override {
        return fdata->GetPosDt();
    }
};
class ChVisualDataExtractorPosDt2 : public ChVisualDataExtractorVector<ChFieldDataPos3D> {
    virtual ChVector3d ExtractImpl(const ChFieldDataPos3D* fdata)  const override {
        return fdata->GetPosDt2();
    }
};
class ChVisualDataExtractorTemperature : public ChVisualDataExtractorScalar<ChFieldDataTemperature> {
    virtual double ExtractImpl(const ChFieldDataTemperature* fdata)  const override {
        return const_cast<ChFieldDataTemperature*>(fdata)->T();
    }
};
class ChVisualDataExtractorTemperatureDt : public ChVisualDataExtractorScalar<ChFieldDataTemperature> {
    virtual double ExtractImpl(const ChFieldDataTemperature* fdata)  const override {
        return const_cast<ChFieldDataTemperature*>(fdata)->T_dt();
    }
};
class ChVisualDataExtractorElectricPotential : public ChVisualDataExtractorScalar<ChFieldDataElectricPotential> {
    virtual double ExtractImpl(const ChFieldDataElectricPotential* fdata)  const override {
        return const_cast<ChFieldDataElectricPotential*>(fdata)->V();
    }
};






//////////////////////////////////////////////////////////////////////////////////////////////////////////

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

/// ...


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
    }
    virtual ~ChVisualDomainGlyphs() {}


    /// Attach scalar property extractor. Also turns the glyph rendering into ChGlyph::GLYPH_POINT mode. 
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(std::shared_ptr<ChVisualDataExtractorScalarBase> mextractor, double min = 0, double max = 1, std::string mname = "Scalar") {
        auto T_property = new ChPropertyScalar;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_scalar_properties.push_back({ mextractor, T_property });
        this->m_properties.push_back(T_property);

        this->SetDrawMode(GLYPH_POINT);

        i_scalar_prop_colorized = (int)extractors_scalar_properties.size() - 1;
        i_vector_prop_colorized = -1;
    }

    /// Attach vector property extractor. Also turns the glyph rendering into ChGlyph::GLYPH_POINT mode.
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(std::shared_ptr<ChVisualDataExtractorVectorBase> mextractor, double min = 0, double max = 1, std::string mname = "Vect") {
        auto T_property = new ChPropertyVector;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_vector_properties.push_back({ mextractor, T_property });
        this->m_properties.push_back(T_property);

        this->SetDrawMode(GLYPH_VECTOR);
        this->vectors = &T_property->data;

        i_scalar_prop_colorized = -1;
        i_vector_prop_colorized = (int)extractors_vector_properties.size() - 1;
    }

    // Set the extractor that gets the 3D position where to draw the glyph (ex. node spatial coords, 
    // for nonlinear deformation analysis). If not set, by default the drawing system will fall 
    // back to the 3D position of the node (material space reference coords). 
    void AddPositionExtractor(std::shared_ptr<ChVisualDataExtractorVectorBase> mextractor) {
        extractor_position = mextractor;
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

    virtual void Update(ChObj* updater, const ChFrame<>& frame) override {
        if (!mdomain)
            return;

        // Count num of glyphs and..
        unsigned int tot_glyphs = 0;
        for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
            tot_glyphs += mdomain->GetField(ifield)->GetNumNodes();
        }
        auto ele_iter = mdomain->CreateIteratorOnElements();
        while (!ele_iter->is_end()) {
            tot_glyphs += ele_iter->get_element()->GetNumQuadraturePoints();
            ele_iter->next();
        }

        // ..resize the vector of points and all the vectors of properties
        this->Reserve(tot_glyphs);

        auto mcolor = this->GetColor();

        unsigned int i = 0;
        
        // Update from nodes
        for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
            auto mfield = mdomain->GetField(ifield);
            
            auto node_iterator = mfield->CreateIteratorOnNodes();
            while (!node_iterator->is_end()) {
                if (auto nodexyz = std::dynamic_pointer_cast<ChNodeFEAfieldXYZ>(node_iterator->get().first)) {

                    // Fill the 3d positions vector in ChGlyphs::points  with the node positions (if possible, 
                    // via extractor_position, otherwise falls back to ChNodeFEAfieldXYZ reference position)
                    ChVector3d mpos = *nodexyz; // default node pos: the reference pos
                    if (extractor_position)
                        if (auto fetched_pos = extractor_position->Extract(&node_iterator->get().second))
                            mpos = fetched_pos.value();
                    this->points[i] = mpos;
                    
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

        // Update from material points
        ele_iter = mdomain->CreateIteratorOnElements();
        while (!ele_iter->is_end()) {
            auto element = ele_iter->get_element();

            ChMatrixDynamic<double> Xhat(3, element->GetNumNodes());
            ChRowVectorDynamic<double> N(element->GetNumNodes());

            // build the Xhat matrix, with all node xyz positions as columns, Xhat=[x1|x2|x3|...]
            bool has_position_fetched = false;
            if (extractor_position) {
                for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                    for (unsigned int inode = 0; inode < ele_iter->get_element()->GetNumNodes(); ++inode) {
                        if (auto fetched_s = extractor_position->Extract(&ele_iter->get_data_per_node(inode, ifield))) {
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
                ele_iter->get_data_per_matpoint(imatpoint);
                element->GetQuadraturePointWeight(element->GetQuadratureOrder(), imatpoint, weight, eta);
                
                element->ComputeN(eta, N);

                this->points[i]  = Xhat * N.transpose();  // position interpolated at quadrature point
                (*this->colors)[i] = mcolor;

                for (auto& mfetch_s : extractors_scalar_properties)
                    if (auto fetched_s = mfetch_s.first->Extract(&ele_iter->get_data_per_matpoint(imatpoint)))
                        mfetch_s.second->data[i] = fetched_s.value();
                for (auto& mfetch_v : extractors_vector_properties)
                    if (auto fetched_v = mfetch_v.first->Extract(&ele_iter->get_data_per_matpoint(imatpoint)))
                        mfetch_v.second->data[i] = fetched_v.value();
                ++i;
            }
            ele_iter->next();
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

    std::shared_ptr<ChDomain> mdomain;

    ChColormap colormap;
    bool use_colormap;

    int i_scalar_prop_colorized;
    int i_vector_prop_colorized;
};



//////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    }
    virtual ~ChVisualDomainMesh() {}


    /// Attach a scalar property extractor. Every time step, the scalar values from the extractor will be fetched 
    /// and stored in the corresponding ChProperty attached to the triangle mesh, with proper interpolation.
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(std::shared_ptr<ChVisualDataExtractorScalarBase> mextractor, double min = 0, double max = 1, std::string mname = "Scalar") {
        auto T_property = new ChPropertyScalar;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_scalar_properties.push_back({ mextractor, T_property });
        this->GetMesh()->m_properties_per_vertex.push_back(T_property);

        i_scalar_prop_colorized = (int)extractors_scalar_properties.size() - 1;
        i_vector_prop_colorized = -1;
    }

    /// Attach a vector property extractor. Every time step, the vector values from the extractor will be fetched 
    /// and stored in the corresponding ChProperty attached to the triangle mesh, with proper interpolation.
    /// If you added more than one property via AddPropertyExtractor(), by default the last one will
    /// be used for the falsecolor rendering.
    void AddPropertyExtractor(std::shared_ptr<ChVisualDataExtractorVectorBase> mextractor, double min = 0, double max = 1, std::string mname = "Vect") {
        auto T_property = new ChPropertyVector;
        T_property->min = min;
        T_property->max = max;
        T_property->name = mname;
        this->extractors_vector_properties.push_back({ mextractor, T_property });
        this->GetMesh()->m_properties_per_vertex.push_back(T_property);

        i_scalar_prop_colorized = -1;
        i_vector_prop_colorized = (int)extractors_vector_properties.size() - 1;
    }

    // Set the extractor that gets the 3D position where to draw the glyph (ex. node spatial coords, 
    // for nonlinear deformation analysis). If not set, by default the drawing system will fall 
    // back to the 3D position of the node (material space reference coords). 
    void AddPositionExtractor(std::shared_ptr<ChVisualDataExtractorVectorBase> mextractor) {
        extractor_position = mextractor;
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

            // Given N nodes in the element, fill the first N values in the std::vector 
            // this->GetMesh()->GetCoordsVertices()  with the N positions (if possible, 
            // via extractor_position, otherwise falls back to ChNodeFEAfieldXYZ reference position)
            bool has_position_fetched = false;
            if (extractor_position) {
                for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                    for (unsigned int inode = 0; inode < ele_iter->get_element()->GetNumNodes(); ++inode) {
                        if (auto fetched_s = extractor_position->Extract(&ele_iter->get_data_per_node(inode, ifield))) {
                            this->GetMesh()->GetCoordsVertices()[offset_vert + inode] = fetched_s.value();
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
                        this->GetMesh()->GetCoordsVertices()[offset_vert + inode] = *nodexyz;
                    }
                }
            }

            // Given N nodes in the element, fill the first N values in the std::vector 
            // embedded in this->GetMesh()->GetPropertiesPerVertex()  with the N values of property
            for (auto& mfetch_s : extractors_scalar_properties) {
                for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                    for (unsigned int inode = 0; inode < ele_iter->get_element()->GetNumNodes(); ++inode) {
                        if (auto fetched_s = mfetch_s.first->Extract(&ele_iter->get_data_per_node(inode, ifield))) {
                            mfetch_s.second->data[offset_vert + inode] = fetched_s.value();
                        }
                    }
                }
            }
            for (auto& mfetch_s : extractors_vector_properties) {
                for (int ifield = 0; ifield < mdomain->GetNumFields(); ++ifield) {
                    for (unsigned int inode = 0; inode < ele_iter->get_element()->GetNumNodes(); ++inode) {
                        if (auto fetched_s = mfetch_s.first->Extract(&ele_iter->get_data_per_node(inode, ifield))) {
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
};


/// @} chrono_fea

}  // end namespace fea

}  // end namespace chrono

#endif
