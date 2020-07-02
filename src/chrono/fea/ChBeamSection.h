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
#include <vector>

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



/// Base class for drawing tesselated profiles of beams in 3D views, if needed.
/// This cross section visualization shape is independent from physical properties 
/// (area, inertia, etc.) that you can define with other components of the ChBeamSection,
/// such as for example ChBeamSectionCosseratElasticity, etc.
/// Used as a component of ChBeamSection.

class ChApi ChBeamSectionShape {
public:
    //
    // Functions for drawing the shape via triangulation:
    //

    /// Get the n. of lines making the profile of the section, for meshing purposes.
    /// C0 continuity is required between lines, C1 also required within each line.
    /// Ex. a circle has 1 line, a cube 4 lines, etc. Sharp corners can be done mith multiple lines.
    virtual int GetNofLines() const = 0;

    /// Get the n. of points to be allocated per each section, for the i-th line in the section.
    /// We assume one also allocates a n. of 3d normals equal to n of points.
    virtual int GetNofPoints(const int i_line) const = 0;

    /// Compute the points (in the reference of the section), for the i-th line in the section. 
    /// Note: mpoints must already have the proper size.
    virtual void GetPoints(const int i_line, std::vector<ChVector<>>& mpoints) const = 0;

    /// Compute the normals (in the reference of the section) at each point, for the i-th line in the section. 
    /// Note: mnormals must already have the proper size.
    virtual void GetNormals(const int i_line, std::vector<ChVector<>>& mnormals) const = 0;


    /// Returns the axis-aligned bounding box (assuming axes of local reference of the section) 
    /// This functions has many uses, ex.for drawing, optimizations, collisions.
    /// We provide a fallback default implementation that iterates over all points thanks to GetPoints(),
    /// but one could override this if a more efficient implementaiton is possible (ex for circular beams, etc.)
    virtual void GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const {
        ymin = 1e30;
        ymax = -1e30;
        zmin = 1e30;
        zmax = -1e30;
        for (int nl = 0; nl < GetNofLines(); ++nl) {
            std::vector<ChVector<>> mpoints(GetNofPoints(nl));
            GetPoints(nl, mpoints);
            for (int np = 0; np < GetNofPoints(nl); ++nl) {
                if (mpoints[np].y() < ymin)
                    ymin = mpoints[np].y();
                if (mpoints[np].y() > ymax)
                    ymax = mpoints[np].y();
                if (mpoints[np].z() < zmin)
                    zmin = mpoints[np].z();
                if (mpoints[np].z() > zmax)
                    zmax = mpoints[np].z();
            }   
        }
    };

};


/// A ready-to-use class for drawing properties of circular beams.
/// Used as a component of ChBeamSection

class ChApi ChBeamSectionShapeCircular : public ChBeamSectionShape {
public:

    ChBeamSectionShapeCircular(double mradius, int mresolution = 10) {
        radius = mradius;
        resolution = mresolution;
        this->UpdateProfile();
    }

    //
    // Functions for drawing the shape via triangulation:
    //

    virtual int GetNofLines() const override {
        return 1;
    };

    virtual int GetNofPoints(const int i_line) const override {
        return resolution+1;
    };

    /// Compute the points (in the reference of the section). 
    /// Note: mpoints must already have the proper size.
    virtual void GetPoints(const int i_line, std::vector<ChVector<>>& mpoints) const override { 
        mpoints = points; 
    };

    /// Compute the normals (in the reference of the section) at each point. 
    /// Note: mnormals must already have the proper size.
    virtual void GetNormals(const int i_line, std::vector<ChVector<>>& mnormals) const override {
        mnormals = normals;
    }

    //
    // Functions for drawing, optimizations, collisions
    //

    /// Returns the axis-aligned bounding box (assuming axes of local reference of the section) 
    /// This functions has many uses, ex.for drawing, optimizations, collisions.
    virtual void GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const override {
        ymin = -radius;
        ymax =  radius;
        zmin = -radius;
        zmax =  radius;
    }

private:
    // internal: update internal precomputed vertex arrays
    void UpdateProfile() {
        points.resize(resolution+1);
        normals.resize(resolution+1);
        for (size_t is = 0; is < points.size(); ++is) {
             double sangle = CH_C_2PI * ((double)is / (double)resolution);
             points[is]  = ChVector<>(0, cos(sangle) * radius, sin(sangle) * radius);
             normals[is] = ChVector<>(0, cos(sangle) , sin(sangle) );
        }
    }
    
    int resolution;
    double radius;
    std::vector<ChVector<>> points;
    std::vector<ChVector<>> normals;
};


/// A ready-to-use class for drawing properties of rectangular beams.
/// Used as a component of ChBeamSection.

class ChApi ChBeamSectionShapeRectangular : public ChBeamSectionShape {
public:

    ChBeamSectionShapeRectangular(double y_width, double z_width) {
        z_thick = z_width;
        y_thick = y_width;
        this->UpdateProfile();
    }

    //
    // Functions for drawing the shape via triangulation:
    //

    virtual int GetNofLines() const override {
        return 4;
    };

    virtual int GetNofPoints(const int i_line) const override {
        return 2;
    };

    /// Compute the points (in the reference of the section). 
    /// Note: mpoints must already have the proper size.
    virtual void GetPoints(const int i_line, std::vector<ChVector<>>& mpoints) const override { 
        mpoints = ml_points[i_line]; 
    };

    /// Compute the normals (in the reference of the section) at each point. 
    /// Note: mnormals must already have the proper size.
    virtual void GetNormals(const int i_line, std::vector<ChVector<>>& mnormals) const override {
        mnormals = ml_normals[i_line];
    }

    /// Returns the axis-aligned bounding box (assuming axes of local reference of the section) 
    virtual void GetAABB(double& ymin, double& ymax, double& zmin, double& zmax) const override {
        ymin = -y_thick*0.5;
        ymax =  y_thick*0.5;
        zmin = -z_thick*0.5;
        zmax =  z_thick*0.5;
    }

private:

    // internal: update internal precomputed vertex arrays
    void UpdateProfile() {

        ml_points.resize(4);
        ml_normals.resize(4);

        double y_thick_half = 0.5 * y_thick;
        double z_thick_half = 0.5 * z_thick;

        ml_points[0].resize(2);
        ml_points[0][0].Set(0, -y_thick_half, -z_thick_half);
        ml_points[0][1].Set(0,  y_thick_half, -z_thick_half);

        ml_points[1].resize(2);
        ml_points[1][0].Set(0,  y_thick_half, -z_thick_half);
        ml_points[1][1].Set(0,  y_thick_half,  z_thick_half);

        ml_points[2].resize(2);
        ml_points[2][0].Set(0,  y_thick_half,  z_thick_half);
        ml_points[2][1].Set(0, -y_thick_half,  z_thick_half);

        ml_points[3].resize(2);
        ml_points[3][0].Set(0, -y_thick_half,  z_thick_half);
        ml_points[3][1].Set(0, -y_thick_half, -z_thick_half);


        ml_normals[0].resize(2);
        ml_normals[0][0].Set(0, 0, -1);
        ml_normals[0][1].Set(0, 0, -1);

        ml_normals[1].resize(2);
        ml_normals[1][0].Set(0,  1, 0);
        ml_normals[1][1].Set(0,  1, 0);

        ml_normals[2].resize(2);
        ml_normals[2][0].Set(0,  0,  1);
        ml_normals[2][1].Set(0,  0,  1);

        ml_normals[3].resize(2);
        ml_normals[3][0].Set(0, -1, 0);
        ml_normals[3][1].Set(0, -1, 0);
    }

    double y_thick;
    double z_thick;
    std::vector< std::vector<ChVector<>> > ml_points;
    std::vector< std::vector<ChVector<>> > ml_normals;
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





//
// OLD CLASSES FOR BEAM MATERIALS
//



/// Basic geometry for a beam section in 3D, along with basic material
/// properties (zz and yy moments of inertia, area, Young modulus, etc.)
/// This material can be shared between multiple beams.
/// 
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElasticityCosseratSimple.png"
///
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

        this->SetDrawThickness(width_y, width_z);
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
/// 
/// \image html "http://www.projectchrono.org/assets/manual/fea_ChElementBeamEuler_section.png"
///
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
