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

#include <cmath>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/assets/ChVisualShapeCylinder.h"

#include "model/Lander.h"

using namespace chrono;

Lander::Lander(ChSystem* system, std::shared_ptr<ChContactMaterial> contact_material)
    : m_system(system),
      m_contact_material(contact_material),
      m_initialized(false),
      // Default dimensional parameters (from demo)
      m_cylinder_length(4.0),
      m_cylinder_diameter(1.0),
      m_cylinder_radius(0.5),
      m_lander_mass(2000.0),
      m_leg_length(1.5),
      m_leg_radius(0.05),
      m_leg_angle(CH_PI / 6.0),  // 30 degrees
      m_leg_spread(0.6),
      m_leg_mass(10.0),
      m_footpad_diameter(5.0 * 0.05),  // 5x leg radius
      m_footpad_radius(5.0 * 0.05 / 2.0),
      m_footpad_height(0.02),
      m_footpad_offset(0.02),
      m_footpad_mass(2.0),
      // Default options
      m_use_footpads(true),
      m_use_spherical_joint(false),
      // Default colors
      m_body_color(ChColor(0.7f, 0.7f, 0.7f)),
      m_leg_color(ChColor(0.7f, 0.7f, 0.7f)),
      m_footpad_color(ChColor(0.6f, 0.6f, 0.6f)) {
    // Initialize arrays
    for (int i = 0; i < 4; i++) {
        m_legs[i] = nullptr;
        m_footpads[i] = nullptr;
        m_leg_joints[i] = nullptr;
        m_footpad_joints[i] = nullptr;
    }
}

Lander::~Lander() {}

void Lander::SetCylinderParameters(double length, double diameter, double mass) {
    m_cylinder_length = length;
    m_cylinder_diameter = diameter;
    m_cylinder_radius = diameter / 2.0;
    m_lander_mass = mass;
}

void Lander::SetLegParameters(double length, double radius, double angle, double spread, double mass) {
    m_leg_length = length;
    m_leg_radius = radius;
    m_leg_angle = angle;
    m_leg_spread = spread;
    m_leg_mass = mass;
}

void Lander::SetFootpadParameters(double diameter, double height, double offset, double mass) {
    m_footpad_diameter = diameter;
    m_footpad_radius = diameter / 2.0;
    m_footpad_height = height;
    m_footpad_offset = offset;
    m_footpad_mass = mass;
}

void Lander::SetInitialVelocity(const ChVector3d& velocity) {
    if (m_initialized && m_lander_body) {
        m_lander_body->SetPosDt(velocity);
    }
}

void Lander::SetInitialVelocityFromDropHeight(double drop_height, double gravity) {
    if (m_initialized && m_lander_body) {
        // Calculate velocity from drop height: v = sqrt(2 * g * h)
        // Negative because velocity is downward
        double initial_velocity_z = -std::sqrt(2.0 * gravity * drop_height);
        m_lander_body->SetPosDt(ChVector3d(0, 0, initial_velocity_z));
    }
}

void Lander::Initialize(const ChFrame<>& pos, double ground_clearance) {
    if (m_initialized) {
        std::cerr << "Warning: Lander already initialized. Skipping." << std::endl;
        return;
    }

    // =============================================================================
    // Create the lander body (cylinder)
    // =============================================================================
    // Calculate density from mass and volume
    double lander_volume = CH_PI * m_cylinder_radius * m_cylinder_radius * m_cylinder_length;
    double lander_density = m_lander_mass / lander_volume;

    // Create cylinder using ChBodyEasyCylinder (automatically calculates inertia)
    m_lander_body = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, m_cylinder_radius, m_cylinder_length,
                                                                  lander_density, m_contact_material);
    m_lander_body->SetName("LanderBody");

    // Calculate the lowest point of the lander (footpad bottom or leg end)
    // Leg extends downward from cylinder bottom by: LEG_LENGTH * cos(LEG_ANGLE)
    double leg_vertical_extension = m_leg_length * std::cos(m_leg_angle);
    double footpad_extension = m_use_footpads ? (m_footpad_height / 2.0 + m_footpad_offset) : 0.0;
    double total_extension_below_cylinder = leg_vertical_extension + footpad_extension;

    // Position cylinder so lowest point is at the specified position + ground_clearance
    // The position frame specifies where the lowest point should be (in global coordinates)
    ChVector3d lowest_point_global = pos.GetPos();
    lowest_point_global.z() += ground_clearance;

    // Calculate cylinder center position in global coordinates
    // The cylinder center is above the lowest point by: total_extension + cylinder_length/2
    double cylinder_center_offset_z = total_extension_below_cylinder + m_cylinder_length / 2.0;

    // Create a frame at the lowest point with the same orientation as pos
    ChFrame<> lowest_point_frame(lowest_point_global, pos.GetRot());

    // Apply the offset in the local frame (z-direction of the pos frame)
    ChVector3d cylinder_center_offset_local(0, 0, cylinder_center_offset_z);
    ChVector3d cylinder_center_global = lowest_point_frame.TransformPointLocalToParent(cylinder_center_offset_local);

    m_lander_body->SetPos(cylinder_center_global);
    m_lander_body->SetRot(pos.GetRot());  // Use orientation from pos frame

    // Set visual color
    if (auto vis_model = m_lander_body->GetVisualModel()) {
        for (unsigned int i = 0; i < vis_model->GetNumShapes(); i++) {
            if (auto cyl_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(vis_model->GetShape(i))) {
                cyl_shape->SetColor(m_body_color);
            }
        }
    }

    m_system->Add(m_lander_body);

    // =============================================================================
    // Create 4 legs and rigidly attach them to the cylinder
    // =============================================================================
    // Leg attachment points: symmetric around the bottom of the cylinder
    // Positions relative to cylinder center (in local frame)
    ChVector3d leg_attach_local[4] = {
        ChVector3d(m_leg_spread, 0, -m_cylinder_length / 2.0),   // +X direction
        ChVector3d(-m_leg_spread, 0, -m_cylinder_length / 2.0),  // -X direction
        ChVector3d(0, m_leg_spread, -m_cylinder_length / 2.0),   // +Y direction
        ChVector3d(0, -m_leg_spread, -m_cylinder_length / 2.0)   // -Y direction
    };

    // Leg directions: extend outward and downward from attachment point
    // Each leg extends in its horizontal direction and downward at LEG_ANGLE from vertical
    double leg_horizontal = m_leg_length * std::sin(m_leg_angle);
    double leg_vertical = -m_leg_length * std::cos(m_leg_angle);  // Negative = downward

    ChVector3d leg_directions[4] = {
        ChVector3d(leg_horizontal, 0, leg_vertical).GetNormalized(),   // +X direction
        ChVector3d(-leg_horizontal, 0, leg_vertical).GetNormalized(),  // -X direction
        ChVector3d(0, leg_horizontal, leg_vertical).GetNormalized(),   // +Y direction
        ChVector3d(0, -leg_horizontal, leg_vertical).GetNormalized()   // -Y direction
    };

    for (int i = 0; i < 4; i++) {
        // Calculate density from mass and volume
        double leg_volume = CH_PI * m_leg_radius * m_leg_radius * m_leg_length;
        double leg_density = m_leg_mass / leg_volume;

        // Create leg using ChBodyEasyCylinder (automatically calculates inertia)
        auto leg = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, m_leg_radius, m_leg_length, leg_density,
                                                                 m_contact_material);
        leg->SetName("Leg" + std::to_string(i + 1));

        // Get global position of leg attachment point
        ChVector3d global_leg_attach = m_lander_body->TransformPointLocalToParent(leg_attach_local[i]);

        // Leg center is halfway along the leg direction from attachment point
        ChVector3d leg_center_local = leg_attach_local[i] + leg_directions[i] * (m_leg_length / 2.0);
        ChVector3d global_leg_center = m_lander_body->TransformPointLocalToParent(leg_center_local);

        leg->SetPos(global_leg_center);

        // Orient leg along its direction
        // The leg direction in global frame
        ChVector3d global_leg_dir = m_lander_body->TransformDirectionLocalToParent(leg_directions[i]);
        global_leg_dir.Normalize();

        // Create rotation to align leg along its direction (leg cylinder axis is Z by default)
        ChVector3d leg_z = global_leg_dir;
        ChVector3d leg_y = ChVector3d(0, 0, 1).Cross(leg_z);
        if (leg_y.Length() < 1e-6) {
            leg_y = ChVector3d(0, 1, 0).Cross(leg_z);
        }
        leg_y.Normalize();
        ChVector3d leg_x = leg_y.Cross(leg_z);
        ChMatrix33<> leg_rot_mat;
        leg_rot_mat.SetFromDirectionAxes(leg_x, leg_y, leg_z);
        leg->SetRot(leg_rot_mat.GetQuaternion());

        // Set visual color
        if (auto vis_model = leg->GetVisualModel()) {
            for (unsigned int j = 0; j < vis_model->GetNumShapes(); j++) {
                if (auto cyl_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(vis_model->GetShape(j))) {
                    cyl_shape->SetColor(m_leg_color);
                }
            }
        }

        m_system->Add(leg);
        m_legs[i] = leg;

        // Rigidly attach leg to lander body using ChLinkLockLock
        auto leg_joint = chrono_types::make_shared<ChLinkLockLock>();
        ChFrame<> joint_frame(global_leg_attach, QUNIT);
        leg_joint->Initialize(m_lander_body, leg, joint_frame);
        m_system->AddLink(leg_joint);
        m_leg_joints[i] = leg_joint;

        // =============================================================================
        // Create footpad at the end of the leg
        // =============================================================================
        if (m_use_footpads) {
            // Calculate the end position of the leg (where footpad attaches)
            ChVector3d leg_end_local = leg_attach_local[i] + leg_directions[i] * m_leg_length;
            leg_end_local.z() -= m_footpad_offset;
            ChVector3d global_leg_end = m_lander_body->TransformPointLocalToParent(leg_end_local);

            // Calculate density from mass and volume
            double footpad_volume = CH_PI * m_footpad_radius * m_footpad_radius * m_footpad_height;
            double footpad_density = m_footpad_mass / footpad_volume;

            // Create footpad using ChBodyEasyCylinder (automatically calculates inertia)
            auto footpad = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Z, m_footpad_radius, m_footpad_height,
                                                                         footpad_density, m_contact_material);
            footpad->SetName("Footpad" + std::to_string(i + 1));

            // Position footpad: center at leg end position
            // The footpad center should be at the leg end, with the flat face parallel to ground
            footpad->SetPos(global_leg_end);

            // Orient footpad so it's flat on the ground (flat face horizontal)
            ChQuaternion<> footpad_rot = QUNIT;  // Upright orientation (cylinder axis along Z)
            footpad->SetRot(footpad_rot);

            // Set visual color
            if (auto vis_model = footpad->GetVisualModel()) {
                for (unsigned int j = 0; j < vis_model->GetNumShapes(); j++) {
                    if (auto cyl_shape = std::dynamic_pointer_cast<ChVisualShapeCylinder>(vis_model->GetShape(j))) {
                        cyl_shape->SetColor(m_footpad_color);
                    }
                }
            }

            m_system->Add(footpad);
            m_footpads[i] = footpad;

            // Connect footpad to leg with either spherical joint or rigid lock
            if (m_use_spherical_joint) {
                // Spherical joint: allows footpad to rotate freely
                // Initial orientation is set flat, but it can rotate during simulation
                auto footpad_joint = chrono_types::make_shared<ChLinkLockSpherical>();
                ChFrame<> footpad_joint_frame(global_leg_end, QUNIT);
                footpad_joint->Initialize(leg, footpad, footpad_joint_frame);
                m_system->AddLink(footpad_joint);
                m_footpad_joints[i] = footpad_joint;
            } else {
                // Rigid lock: footpad is fixed at the calculated orientation (flat on ground)
                auto footpad_joint = chrono_types::make_shared<ChLinkLockLock>();
                ChFrame<> footpad_joint_frame(global_leg_end, footpad_rot);
                footpad_joint->Initialize(leg, footpad, footpad_joint_frame);
                m_system->AddLink(footpad_joint);
                m_footpad_joints[i] = footpad_joint;
            }
        }
    }

    m_initialized = true;
}
