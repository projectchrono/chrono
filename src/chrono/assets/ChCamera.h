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

#ifndef CH_CAMERA_H
#define CH_CAMERA_H

#include "chrono/core/ChVector.h"

namespace chrono {

/// @addtogroup chrono_assets
/// @{

// Forward declaration
class ChPhysicsItem;

/// Class for defining a camera point of view with basic settings.
class ChApi ChCamera {
  public:
    ChCamera();
    virtual ~ChCamera() {}

    /// Sets the position of the observer (eye point).
    /// Expressed in the local coordinate system.
    void SetPosition(const ChVector<>& pos) { position = pos; }
    /// Gets the position of the observer (eye point).
    /// Expressed in the local coordinate system.
    const ChVector<>& GetPosition() const { return position; }

    /// Sets the position of the target point (aim point).
    /// Expressed in the local coordinate system.
    void SetAimPoint(const ChVector<>& point) { aimpoint = point; }
    /// Gets the position of the target point (aim point).
    /// Expressed in the local coordinate system.
    const ChVector<>& GetAimPoint() const { return aimpoint; }

    /// Sets the position of the 'up' direction of the camera (default is vertical, VECT_Y).
    /// Expressed in the local coordinate system.
    void SetUpVector(const ChVector<>& up) { upvector = up.GetNormalized(); }
    /// Gets the position of the 'up' direction of the camera (default is vertical, VECT_Y).
    /// Expressed in the local coordinate system.
    const ChVector<>& GetUpVector() const { return upvector; }

    /// Sets the opening angle of the lenses, in degrees, on horizontal direction.
    /// E.g., 60=wide angle, 30=tele, etc.
    void SetAngle(double deg) { angle = deg; }
    /// Gets the opening angle of the lenses, in degrees, on horizontal direction
    double GetAngle() const { return angle; }

    /// Sets the Field Of View, if the visualization supports focusing.
    void SetFOV(double f) { fov = f; }
    /// Gets the Field Of View, if the visualization supports focusing.
    double GetFOV() const { return fov; }

    /// Sets the Horizontal/Vertical size ratio. Default = 4/3.
    /// For instance, 4/3 can be used if rendering to 800x600 images with square pixels.
    void SetHVratio(double mf) { hvratio = mf; }
    /// Gets the Horizontal/Vertical size ratio. Default = 4/3.
    /// For instance, 4/3 can be used if rendering to 800x600 images with square pixels.
    double GetHVratio() const { return hvratio; }

    /// Set to 'true' if you want to disable perspective and get a 'flat' view.
    /// This must be supported by the visualization system. By default is 'false'.
    void SetOrthographic(bool mode) { isometric = mode; }
    /// If 'true' it means that perspective projection is disabled and you get a 'flat' view.
    /// This must be supported by the visualization system. By default is 'false'.
    bool IsOrthographic() const { return isometric; }

    /// Update the camera state.
    /// This function is automatically called by the owner physics item.
    virtual void Update() {}

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive);

  private:
    ChVector<> position;
    ChVector<> aimpoint;
    ChVector<> upvector;
    double angle;
    double fov;
    double hvratio;
    bool isometric;

    ChPhysicsItem* m_owner;

    friend class ChPhysicsItem;
};

/// @} chrono_assets

CH_CLASS_VERSION(ChCamera, 0)

}  // end namespace chrono

#endif
