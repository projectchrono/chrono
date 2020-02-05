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

/// Base class for properties of all beam sections.
/// A beam section can be shared between multiple beams.
/// A beam section contains the models for elasticity, plasticity, damping, etc.
class ChApi ChBeamSectionProperties {
  public:
    double y_drawsize;
    double z_drawsize;
    bool is_circular;

    double Area;
    double density;

    ChBeamSectionProperties() : y_drawsize(0.01), z_drawsize(0.01), is_circular(false), Area(1), density(1000) {}

    virtual ~ChBeamSectionProperties() {}

    /// Sets the rectangular thickness of the beam on y and z directions,
    /// only for drawing/rendering purposes (these thickness values do NOT
    /// have any meaning at a physical level, use SetAsRectangularSection()
    ////instead if you want to affect also the inertias of the beam section).
    void SetDrawThickness(double thickness_y, double thickness_z) {
        this->y_drawsize = thickness_y;
        this->z_drawsize = thickness_z;
    }
    double GetDrawThicknessY() { return this->y_drawsize; }
    double GetDrawThicknessZ() { return this->z_drawsize; }

    /// Tells if the section must be drawn as a circular
    /// section instead than default rectangular
    bool IsCircular() { return is_circular; }
    /// Set if the section must be drawn as a circular
    /// section instead than default rectangular
    void SetCircular(bool ic) { is_circular = ic; }

    /// Sets the radius of the beam if in 'circular section' draw mode,
    /// only for drawing/rendering purposes (this radius value do NOT
    /// have any meaning at a physical level, use ChBeamSectionBasic::SetAsCircularSection()
    ////instead if you want to affect also the inertias of the beam section).
    void SetDrawCircularRadius(double draw_rad) { this->y_drawsize = draw_rad; }
    double GetDrawCircularRadius() { return this->y_drawsize; }

    /// Set the cross sectional area A of the beam (m^2)
    void SetArea(const double ma) { this->Area = ma; }
    double GetArea() const { return this->Area; }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Shortcut: set elastic and plastic constants
    /// at once, given the y and z widths of the beam assumed
    /// with rectangular shape.
    virtual void SetAsRectangularSection(double width_y, double width_z) = 0;

    /// Shortcut: set elastic and plastic constants
    /// at once, given the diameter of the beam assumed
    /// with circular shape.
    virtual void SetAsCircularSection(double diameter) = 0;
};

//
// OLD CLASSES FOR BEAM MATERIALS
//

/// Base class for properties of beam sections.
/// A beam section can be shared between multiple beams.
/// A beam section contains the models for elasticity, plasticity, damping, etc.
class ChApi ChBeamSection {
  public:
    double y_drawsize;
    double z_drawsize;
    bool is_circular;

    ChBeamSection() : y_drawsize(0.01), z_drawsize(0.01), is_circular(false) {}

    virtual ~ChBeamSection() {}

    /// Sets the rectangular thickness of the beam on y and z directions,
    /// only for drawing/rendering purposes (these thickness values do NOT
    /// have any meaning at a physical level, use ChBeamSectionBasic::SetAsRectangularSection()
    ////instead if you want to affect also the inertias of the beam section).
    void SetDrawThickness(double thickness_y, double thickness_z) {
        this->y_drawsize = thickness_y;
        this->z_drawsize = thickness_z;
    }
    double GetDrawThicknessY() { return this->y_drawsize; }
    double GetDrawThicknessZ() { return this->z_drawsize; }

    /// Tells if the section must be drawn as a circular
    /// section instead than default rectangular
    bool IsCircular() { return is_circular; }
    /// Set if the section must be drawn as a circular
    /// section instead than default rectangular
    void SetCircular(bool ic) { is_circular = ic; }

    /// Sets the radius of the beam if in 'circular section' draw mode,
    /// only for drawing/rendering purposes (this radius value do NOT
    /// have any meaning at a physical level, use ChBeamSectionBasic::SetAsCircularSection()
    ////instead if you want to affect also the inertias of the beam section).
    void SetDrawCircularRadius(double draw_rad) { this->y_drawsize = draw_rad; }
    double GetDrawCircularRadius() { return this->y_drawsize; }
};

/// Basic geometry for a beam section in 3D, along with basic material
/// properties (zz and yy moments of inertia, area, Young modulus, etc.)
/// This material can be shared between multiple beams.
class ChApi ChBeamSectionBasic : public ChBeamSection {
  public:
    double Area;
    double Iyy;
    double Izz;
    double J;
    double G;
    double E;
    double density;
    double rdamping;
    double Ks_y;
    double Ks_z;

    ChBeamSectionBasic()
        : E(0.01e9),      // default E stiffness: (almost rubber)
          density(1000),  // default density: water
          rdamping(0.01)  // default Rayleigh damping.
    {
        SetGwithPoissonRatio(0.3);            // default G (low poisson ratio)
        SetAsRectangularSection(0.01, 0.01);  // defaults Area, Ixx, Iyy, Ks_y, Ks_z, J
    }

    virtual ~ChBeamSectionBasic() {}

    /// Set the cross sectional area A of the beam (m^2)
    void SetArea(const double ma) { this->Area = ma; }
    double GetArea() const { return this->Area; }

    /// Set the Iyy moment of inertia of the beam (for flexion about y axis)
    /// Note: some textbook calls this Iyy as Iz
    void SetIyy(double ma) { this->Iyy = ma; }
    double GetIyy() const { return this->Iyy; }

    /// Set the Izz moment of inertia of the beam (for flexion about z axis)
    /// Note: some textbook calls this Izz as Iy
    void SetIzz(double ma) { this->Izz = ma; }
    double GetIzz() const { return this->Izz; }

    /// Set the J torsion constant of the beam (for torsion about x axis)
    void SetJ(double ma) { this->J = ma; }
    double GetJ() const { return this->J; }

    /// Set the Timoshenko shear coefficient Ks for y shear, usually about 0.8,
    /// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
    void SetKsy(double ma) { this->Ks_y = ma; }
    double GetKsy() const { return this->Ks_y; }

    /// Set the Timoshenko shear coefficient Ks for z shear, usually about 0.8,
    /// (for elements that use this, ex. the Timoshenko beams, or Reddy's beams)
    void SetKsz(double ma) { this->Ks_z = ma; }
    double GetKsz() const { return this->Ks_z; }

    /// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
    /// at once, given the y and z widths of the beam assumed
    /// with rectangular shape.
    void SetAsRectangularSection(double width_y, double width_z) {
        this->Area = width_y * width_z;
        this->Izz = (1.0 / 12.0) * width_z * pow(width_y, 3);
        this->Iyy = (1.0 / 12.0) * width_y * pow(width_z, 3);

        // use Roark's formulas for torsion of rectangular sect:
        double t = ChMin(width_y, width_z);
        double b = ChMax(width_y, width_z);
        this->J = b * pow(t, 3) * ((1.0 / 3.0) - 0.210 * (t / b) * (1.0 - (1.0 / 12.0) * pow((t / b), 4)));

        // set Ks using Timoshenko-Gere formula for solid rect.shapes
        double poisson = this->E / (2.0 * this->G) - 1.0;
        this->Ks_y = 10.0 * (1.0 + poisson) / (12.0 + 11.0 * poisson);
        this->Ks_z = this->Ks_y;

        this->is_circular = false;
        this->y_drawsize = width_y;
        this->z_drawsize = width_z;
    }

    /// Shortcut: set Area, Ixx, Iyy, Ksy, Ksz and J torsion constant
    /// at once, given the diameter of the beam assumed
    /// with circular shape.
    void SetAsCircularSection(double diameter) {
        this->Area = CH_C_PI * pow((0.5 * diameter), 2);
        this->Izz = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);
        this->Iyy = Izz;

        // exact expression for circular beam J = Ixx ,
        // where for polar theorem Ixx = Izz+Iyy
        this->J = Izz + Iyy;

        // set Ks using Timoshenko-Gere formula for solid circular shape
        double poisson = this->E / (2.0 * this->G) - 1.0;
        this->Ks_y = 6.0 * (1.0 + poisson) / (7.0 + 6.0 * poisson);
        this->Ks_z = this->Ks_y;

        this->is_circular = true;
        this->SetDrawCircularRadius(diameter / 2);
    }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set G, the shear modulus
    void SetGshearModulus(double mG) { this->G = mG; }
    double GetGshearModulus() const { return this->G; }

    /// Set G, the shear modulus, given current E and the specified Poisson ratio
    void SetGwithPoissonRatio(double mpoisson) { this->G = this->E / (2.0 * (1.0 + mpoisson)); }

    /// Set the Rayleigh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
    void SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
    double GetBeamRaleyghDamping() { return this->rdamping; }
};

/// Geometry for a beam section in 3D, along with basic material
/// properties. It also supports the advanced case of
/// Iyy and Izz axes rotated respect reference, centroid with offset
/// from reference, and shear center with offset from reference.
/// This material can be shared between multiple beams.
class ChApi ChBeamSectionAdvanced : public ChBeamSectionBasic {
  public:
    double alpha;  // Rotation of Izz Iyy respect to reference line x
    double Cy;     // Centroid (elastic center, tension center)
    double Cz;
    double Sy;  // Shear center
    double Sz;

    ChBeamSectionAdvanced() : alpha(0), Cy(0), Cz(0), Sy(0), Sz(0) {}

    virtual ~ChBeamSectionAdvanced() {}

    /// Set the rotation of the section for which the Iyy Izz are defined.
    void SetSectionRotation(double ma) { this->alpha = ma; }
    double GetSectionRotation() { return this->alpha; }

    /// Set the displacement of the centroid C (i.e. the elastic center,
    /// or tension center) with respect to the reference beam line.
    void SetCentroid(double my, double mz) {
        this->Cy = my;
        this->Cz = mz;
    }
    double GetCentroidY() { return this->Cy; }
    double GetCentroidZ() { return this->Cz; }

    /// Set the displacement of the shear center S with respect to the reference beam line.
    /// For shapes like rectangles, rotated rectangles, etc., it corresponds to the centroid C,
    /// but for "L" shaped or "U" shaped beams this is not always true, and the shear center
    /// accounts for torsion effects when a shear force is applied.
    void SetShearCenter(double my, double mz) {
        this->Sy = my;
        this->Sz = mz;
    }
    double GetShearCenterY() { return this->Sy; }
    double GetShearCenterZ() { return this->Sz; }
};

/// Simplified geometry for a 'cable' beam section in 3D, that is a beam
/// without torsional stiffness and with circular section (i.e.same Ixx and Iyy properties).
/// This material can be shared between multiple beams.
class ChApi ChBeamSectionCable : public ChBeamSection {
  public:
    double Area;
    double I;
    double E;
    double density;
    double rdamping;

    ChBeamSectionCable()
        : E(0.01e9),      // default E stiffness: (almost rubber)
          density(1000),  // default density: water
          rdamping(0.01)  // default Rayleigh damping.
    {
        SetDiameter(0.01);  // defaults Area, I
    }

    virtual ~ChBeamSectionCable() {}

    /// Set the cross sectional area A of the beam (m^2)
    void SetArea(const double ma) { this->Area = ma; }
    double GetArea() const { return this->Area; }

    /// Set the I moment of inertia of the beam (for flexion about y axis or z axis)
    /// Note: since this simple section assumes circular section, Iyy=Izz=I
    void SetI(double ma) { this->I = ma; }
    double GetI() const { return this->I; }

    /// Shortcut: set Area and I inertia at once,
    /// given the diameter of the beam assumed
    /// with circular shape.
    void SetDiameter(double diameter) {
        this->Area = CH_C_PI * pow((0.5 * diameter), 2);
        this->I = (CH_C_PI / 4.0) * pow((0.5 * diameter), 4);

        this->is_circular = true;
        this->SetDrawCircularRadius(diameter / 2);
    }

    /// Set the density of the beam (kg/m^3)
    void SetDensity(double md) { this->density = md; }
    double GetDensity() const { return this->density; }

    /// Set E, the Young elastic modulus (N/m^2)
    void SetYoungModulus(double mE) { this->E = mE; }
    double GetYoungModulus() const { return this->E; }

    /// Set the Rayleigh damping ratio r (as in: R = r * K ), to do: also mass-proportional term
    void SetBeamRaleyghDamping(double mr) { this->rdamping = mr; }
    double GetBeamRaleyghDamping() { return this->rdamping; }
};

/// @} fea_utils

}  // end namespace fea
}  // end namespace chrono

#endif
