// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// M113 idler subsystem.
//
// =============================================================================

#ifndef M113_IDLER_H
#define M113_IDLER_H

#include <string>

#include "chrono_vehicle/ChSubsysDefs.h"
#include "chrono_vehicle/tracked_vehicle/idler/ChDoubleIdler.h"

namespace m113 {

///
///
///
class M113_Idler : public chrono::vehicle::ChDoubleIdler {
  public:
    virtual ~M113_Idler() {}

    /// Return the location of the specified hardpoint.
    /// The returned location must be expressed in the idler subsystem reference frame.
    virtual const chrono::ChVector<> GetLocation(PointId which) override;

    /// Return the mass of the idler wheel body.
    virtual double GetWheelMass() const override { return m_wheel_mass; }
    /// Return the moments of inertia of the idler wheel body.
    virtual const chrono::ChVector<>& GetWheelInertia() override { return m_wheel_inertia; }
    /// Return the radius of the idler wheel.
    virtual double GetWheelRadius() const override { return m_wheel_radius; }
    /// Return the total width of the idler wheel.
    virtual double GetWheelWidth() const override { return m_wheel_width; }
    /// Return the gap width.
    virtual double GetWheelGap() const override { return m_wheel_gap; }

    /// Return the mass of the carrier body.
    virtual double GetCarrierMass() const override { return m_carrier_mass; }
    /// Return the moments of inertia of the carrier body.
    virtual const chrono::ChVector<>& GetCarrierInertia() override { return m_carrier_inertia; }
    /// Return a visualization radius for the carrier body.
    virtual double GetCarrierVisRadius() const override { return m_carrier_radius; }

    /// Return the pitch angle of the prismatic joint.
    virtual double GetPrismaticPitchAngle() const override { return 0; }

    /// Return the callback function for spring force.
    virtual chrono::ChSpringForceCallback* GetTensionerForceCallback() const override { return m_tensionerForceCB; }

    /// Add visualization of the idler wheel.
    virtual void AddWheelVisualization() override;

    /// Set the idler wheel visualization type.
    void SetVisType(chrono::vehicle::VisualizationType vis) { m_vis_type = vis; }

    /// Export the wheel mesh Wavefront OBJ as a POV-Ray mesh macro.
    void ExportMeshPovray(const std::string& out_dir);

  protected:
    M113_Idler(const std::string& name);

    virtual chrono::vehicle::VehicleSide GetVehicleSide() const = 0;

    virtual const std::string& GetMeshName() const = 0;
    virtual const std::string& GetMeshFile() const = 0;

    chrono::ChSpringForceCallback* m_tensionerForceCB;

    static const double m_wheel_mass;
    static const chrono::ChVector<> m_wheel_inertia;
    static const double m_wheel_radius;
    static const double m_wheel_width;
    static const double m_wheel_gap;

    static const double m_carrier_mass;
    static const chrono::ChVector<> m_carrier_inertia;
    static const double m_carrier_radius;

    static const double m_tensioner_l0;
    static const double m_tensioner_k;
    static const double m_tensioner_c;
    static const double m_tensioner_f;

    chrono::vehicle::VisualizationType m_vis_type;
};

class M113_IdlerLeft : public M113_Idler {
  public:
    M113_IdlerLeft() : M113_Idler("M113_IdlerLeft") {}
    ~M113_IdlerLeft() {}

    virtual chrono::vehicle::VehicleSide GetVehicleSide() const override { return chrono::vehicle::LEFT; }

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

class M113_IdlerRight : public M113_Idler {
  public:
    M113_IdlerRight() : M113_Idler("M113_IdlerRight") {}
    ~M113_IdlerRight() {}

    virtual chrono::vehicle::VehicleSide GetVehicleSide() const override { return chrono::vehicle::RIGHT; }

    virtual const std::string& GetMeshName() const override { return m_meshName; }
    virtual const std::string& GetMeshFile() const override { return m_meshFile; }

  private:
    static const std::string m_meshName;
    static const std::string m_meshFile;
};

}  // end namespace m113

#endif
