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

#ifndef CHBEAMSECTION_H
#define CHBEAMSECTION_H

#include "chrono/core/ChMathematics.h"
#include "chrono/core/ChVector.h"
#include "chrono/fea/ChBeamSectionShape.h"

namespace chrono {
namespace fea {

/// @addtogroup fea_utils
/// @{

/// Base class for internal variables of materials.
/// Especially useful for plasticity, where internal variables are used
/// to carry information on plastic flow, accumulated flow, etc.
class ChApi ChBeamMaterialInternalData {
  public:
    ChBeamMaterialInternalData() : p_strain_acc(0) {}

    virtual ~ChBeamMaterialInternalData(){};

    virtual void Copy(const ChBeamMaterialInternalData& other) { p_strain_acc = other.p_strain_acc; }

    double p_strain_acc;  // accumulated flow,  \overbar\eps^p  in Neto-Owen book
};






/// Base class for properties of beam sections.
/// A beam section can be shared between multiple beams.
/// A beam section contains the models for elasticity, plasticity, damping, etc.
class ChApi ChBeamSection {
  public:


    ChBeamSection()  {
        // default visualization as 1cm square tube
        this->draw_shape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(0.01, 0.01);
    }

    virtual ~ChBeamSection() {}


    /// Set the graphical representation for this section. Might be used for collision too.
    /// This is a 2D profile used for 3D tesselation and visualization of the beam, but NOT used for physical
    /// properties, that you should rather define with other components of more specialized ChBeamSection,
    /// such as for example adding ChBeamSectionCosseratElasticity to a ChBeamSectionCosserat, etc.
    void SetDrawShape(std::shared_ptr<ChBeamSectionShape> mshape) {
        this->draw_shape = mshape;
    }

    /// Get the drawing shape of this section (i.e.a 2D profile used for drawing 3D tesselation and visualization)
    /// By default a thin square section, use SetDrawShape() to change it.
    std::shared_ptr<ChBeamSectionShape> GetDrawShape() const { 
        return this->draw_shape; 
    }


    /// Shortcut: adds a ChBeamSectionShapeRectangular for visualization as a centered rectangular beam,
    /// and sets its width/height. 
    /// NOTE: only for visualization - these thickness values do NOT have any meaning at a physical level, that is set in other ways.
    void SetDrawThickness(double thickness_y, double thickness_z) {
        this->draw_shape = chrono_types::make_shared<ChBeamSectionShapeRectangular>(thickness_y, thickness_z);
    }

    /// Shortcut: adds a ChBeamSectionShapeCircular for visualization as a centered circular beam,
    /// and sets its radius. 
    /// NOTE: only for visualization - this radius do NOT have any meaning at a physical level, that is set in other ways.
    void SetDrawCircularRadius(double draw_rad) { 
        this->draw_shape = chrono_types::make_shared<ChBeamSectionShapeCircular>(draw_rad);
    }

    ///***OBSOLETE*** only for backward compability
    void SetCircular(bool ic) { 
        ///***OBSOLETE*** 
    }

private:
    std::shared_ptr< ChBeamSectionShape > draw_shape;
};







/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
