// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2025 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Huzaifa Mustafa Unjhawala
// =============================================================================
//
// Lunar Lander Model Class.
// This class contains model for a simple lunar lander with a cylindrical body,
// four legs, and optional footpads.
//
// =============================================================================

#ifndef LANDER_H
#define LANDER_H

#include <array>
#include <memory>

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChContactMaterial.h"
#include "chrono/assets/ChColor.h"

// Lunar Lander model.
// The lander model consists of a cylindrical body with four rigidly attached legs
// and optional footpads at the end of each leg.
class Lander {
  public:
    // Construct a Lander within the specified Chrono system.
    Lander(chrono::ChSystem* system,                                    // containing system
           std::shared_ptr<chrono::ChContactMaterial> contact_material  // contact material for all lander parts
    );

    ~Lander();

    // Get the containing system.
    chrono::ChSystem* GetSystem() { return m_system; }

    // Initialize the lander at the specified position and orientation.
    // The position specifies where the lowest point of the lander (footpad or leg end)
    // should be placed, accounting for ground clearance.
    void Initialize(const chrono::ChFrame<>& pos,   // position and orientation of the lander
                    double ground_clearance = 0.01  // clearance above ground (meters)
    );

    // Set initial velocity for the lander body.
    // This should be called after Initialize().
    void SetInitialVelocity(const chrono::ChVector3d& velocity);

    // Set initial velocity from drop height.
    // Calculates velocity as v = sqrt(2 * g * h) downward.
    void SetInitialVelocityFromDropHeight(double drop_height, double gravity = 1.62);

    // Setter methods for dimensional parameters

    // Set cylinder body parameters.
    void SetCylinderParameters(double length,    // cylinder length (meters)
                               double diameter,  // cylinder diameter (meters)
                               double mass       // cylinder mass (kg)
    );

    // Set leg parameters.
    void SetLegParameters(double length,  // leg length (meters)
                          double radius,  // leg radius (meters)
                          double angle,   // leg angle from vertical (radians)
                          double spread,  // leg spread from cylinder center (meters)
                          double mass     // leg mass per leg (kg)
    );

    // Set footpad parameters.
    void SetFootpadParameters(double diameter,  // footpad diameter (meters)
                              double height,    // footpad height (meters)
                              double offset,    // footpad offset in z-direction (meters)
                              double mass       // footpad mass per footpad (kg)
    );

    // Enable/disable footpads (default: true).
    void SetUseFootpads(bool use_footpads) { m_use_footpads = use_footpads; }

    // Set footpad joint type (default: false = rigid lock).
    // true = spherical joint (allows rotation), false = rigid lock (fixed orientation).
    void SetUseSphericalJoint(bool use_spherical) { m_use_spherical_joint = use_spherical; }

    // Set visual colors.
    void SetBodyColor(const chrono::ChColor& color) { m_body_color = color; }
    void SetLegColor(const chrono::ChColor& color) { m_leg_color = color; }
    void SetFootpadColor(const chrono::ChColor& color) { m_footpad_color = color; }

    // =============================================================================
    // Getter methods for ChBody components
    // =============================================================================

    // Get the lander body (cylinder).
    std::shared_ptr<chrono::ChBody> GetBody() const { return m_lander_body; }

    // Get the specified leg body.
    std::shared_ptr<chrono::ChBody> GetLeg(int index) const {
        if (index >= 0 && index < 4) {
            return m_legs[index];
        }
        return nullptr;
    }

    // Get all leg bodies.
    std::array<std::shared_ptr<chrono::ChBody>, 4> GetLegs() const { return m_legs; }

    // Get the specified footpad body (if footpads are enabled).
    std::shared_ptr<chrono::ChBody> GetFootpad(int index) const {
        if (m_use_footpads && index >= 0 && index < 4) {
            return m_footpads[index];
        }
        return nullptr;
    }

    // Get all footpad bodies (if footpads are enabled).
    std::array<std::shared_ptr<chrono::ChBody>, 4> GetFootpads() const { return m_footpads; }

    // Get all leg joints.
    std::array<std::shared_ptr<chrono::ChLinkLockLock>, 4> GetLegJoints() const { return m_leg_joints; }

    // Get the specified footpad joint (if footpads are enabled).
    std::shared_ptr<chrono::ChLinkLock> GetFootpadJoint(int index) const {
        if (m_use_footpads && index >= 0 && index < 4) {
            return m_footpad_joints[index];
        }
        return nullptr;
    }

    // Get all footpad joints (if footpads are enabled).
    std::array<std::shared_ptr<chrono::ChLinkLock>, 4> GetFootpadJoints() const { return m_footpad_joints; }

    // =============================================================================
    // Getter methods for parameters
    // =============================================================================

    double GetCylinderLength() const { return m_cylinder_length; }
    double GetCylinderDiameter() const { return m_cylinder_diameter; }
    double GetCylinderRadius() const { return m_cylinder_radius; }
    double GetLanderMass() const { return m_lander_mass; }

    double GetLegLength() const { return m_leg_length; }
    double GetLegRadius() const { return m_leg_radius; }
    double GetLegAngle() const { return m_leg_angle; }
    double GetLegSpread() const { return m_leg_spread; }
    double GetLegMass() const { return m_leg_mass; }

    double GetFootpadDiameter() const { return m_footpad_diameter; }
    double GetFootpadRadius() const { return m_footpad_radius; }
    double GetFootpadHeight() const { return m_footpad_height; }
    double GetFootpadOffset() const { return m_footpad_offset; }
    double GetFootpadMass() const { return m_footpad_mass; }

    bool GetUseFootpads() const { return m_use_footpads; }
    bool GetUseSphericalJoint() const { return m_use_spherical_joint; }

  private:
    // Create the lander components.
    void Create();

    chrono::ChSystem* m_system;                                     // pointer to the Chrono system
    std::shared_ptr<chrono::ChContactMaterial> m_contact_material;  // contact material

    bool m_initialized;  // flag indicating whether the lander was initialized

    // Lander components
    std::shared_ptr<chrono::ChBody> m_lander_body;              // main lander body (cylinder)
    std::array<std::shared_ptr<chrono::ChBody>, 4> m_legs;      // leg bodies
    std::array<std::shared_ptr<chrono::ChBody>, 4> m_footpads;  // footpad bodies (if enabled)

    // Joints
    std::array<std::shared_ptr<chrono::ChLinkLockLock>, 4> m_leg_joints;  // joints connecting legs to body
    std::array<std::shared_ptr<chrono::ChLinkLock>, 4> m_footpad_joints;  // joints connecting footpads to legs

    // Dimensional parameters
    double m_cylinder_length;    // cylinder length (meters)
    double m_cylinder_diameter;  // cylinder diameter (meters)
    double m_cylinder_radius;    // cylinder radius (meters)
    double m_lander_mass;        // lander body mass (kg)

    double m_leg_length;  // leg length (meters)
    double m_leg_radius;  // leg radius (meters)
    double m_leg_angle;   // leg angle from vertical (radians)
    double m_leg_spread;  // leg spread from cylinder center (meters)
    double m_leg_mass;    // leg mass per leg (kg)

    double m_footpad_diameter;  // footpad diameter (meters)
    double m_footpad_radius;    // footpad radius (meters)
    double m_footpad_height;    // footpad height (meters)
    double m_footpad_offset;    // footpad offset in z-direction (meters)
    double m_footpad_mass;      // footpad mass per footpad (kg)

    // Options
    bool m_use_footpads;         // enable/disable footpads
    bool m_use_spherical_joint;  // use spherical joint vs rigid lock for footpads

    // Visual colors
    chrono::ChColor m_body_color;     // body visual color
    chrono::ChColor m_leg_color;      // leg visual color
    chrono::ChColor m_footpad_color;  // footpad visual color
};

#endif
