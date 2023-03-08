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
// Authors: Radu Serban, Marcel Offermans
// =============================================================================
//
// Generic wheeled vehicle suspension constructed with data from file.
//
// =============================================================================

#ifndef GENERIC_WHEELED_SUSPENSION_H
#define GENERIC_WHEELED_SUSPENSION_H

#include "chrono_vehicle/ChApiVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/suspension/ChGenericWheeledSuspension.h"
#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {
namespace vehicle {

/// @addtogroup vehicle_wheeled_suspension
/// @{

/// Generic wheeled vehicle suspension constructed with data from file.
class CH_VEHICLE_API GenericWheeledSuspension : public ChGenericWheeledSuspension {
  public:
    GenericWheeledSuspension(const std::string& filename);
    GenericWheeledSuspension(const rapidjson::Document& d);
    ~GenericWheeledSuspension();

    virtual double getCamberAngle() const override { return m_camberAngle; }
    virtual double getToeAngle() const override { return m_toeAngle; }

    virtual double getSpindleMass() const override { return m_spindleMass; }
    virtual ChVector<> getSpindlePos() const override { return m_spindlePosition; }
    virtual double getSpindleRadius() const override { return m_spindleRadius; }
    virtual double getSpindleWidth() const override { return m_spindleWidth; }
    virtual const ChVector<>& getSpindleInertia() const override { return m_spindleInertia; }

    virtual double getAxleInertia() const override { return m_axleInertia; }

    virtual bool IsSteerable() const final override { return m_steerable; }
    virtual bool IsIndependent() const final override { return m_independent; }

    virtual BodyIdentifier getSpindleAttachmentBody() const override { return m_spindleAttachmentBody; }
    virtual BodyIdentifier getAntirollBody() const override { return m_antirollBody; }

  private:
    virtual void Create(const rapidjson::Document& d) override;

    bool m_steerable;
    bool m_independent;

    double m_camberAngle;
    double m_toeAngle;

    double m_spindleMass;
    ChVector<> m_spindlePosition;
    double m_spindleRadius;
    double m_spindleWidth;
    ChVector<> m_spindleInertia;

    BodyIdentifier m_spindleAttachmentBody;
    BodyIdentifier m_antirollBody;

    double m_axleInertia;
};

/// @} vehicle_wheeled_suspension

}  // end namespace vehicle
}  // end namespace chrono

#endif
