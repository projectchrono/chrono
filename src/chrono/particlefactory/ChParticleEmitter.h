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

#ifndef CHPARTICLEEMITTER_H
#define CHPARTICLEEMITTER_H

#include "chrono/particlefactory/ChRandomShapeCreator.h"
#include "chrono/particlefactory/ChRandomParticlePosition.h"
#include "chrono/particlefactory/ChRandomParticleAlignment.h"
#include "chrono/particlefactory/ChRandomParticleVelocity.h"

#include "chrono/core/ChRandom.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/physics/ChSystem.h"

namespace chrono {

/// \brief Namespace for classes that generate flows of particles.
/// Namespace for helper classes that build a system for generating
/// flows of particles. This system heavily relies on statistical
/// approaches, for example positions, rotations, shapes of particles
/// can be generated according to statistical distributions.
namespace particlefactory {

/// Class for emitters of particles, with random positions,
/// rotations, and random shapes. You can setup a variety of
/// different emitters by assembling different types of
/// items inherited by classes like ChRandomShapeCreator,
/// ChRandomParticlePosition, etc.
class ChParticleEmitter {
  public:
    enum eChFlowMode {
        FLOW_PARTICLESPERSECOND,
        FLOW_MASSPERSECOND,
    };

    ChParticleEmitter()
        : flow_mode(FLOW_PARTICLESPERSECOND),
          particles_per_second(100),
          mass_per_second(1),
          creation_callback(nullptr),
          use_particle_reservoir(false),
          use_mass_reservoir(false),
          particle_reservoir(1000),
          mass_reservoir(1),
          created_particles(0),
          created_mass(0),
          off_mass(0),
          off_count(0),
          inherit_owner_speed(true),
          jitter_declustering(true) {
        // defaults:
        particle_creator = chrono_types::make_shared<ChRandomShapeCreatorSpheres>();
        particle_positioner = chrono_types::make_shared<ChRandomParticlePositionRectangleOutlet>();
        particle_aligner = chrono_types::make_shared<ChRandomParticleAlignmentUniform>();
        particle_velocity = chrono_types::make_shared<ChRandomParticleVelocity>();
        particle_angular_velocity = chrono_types::make_shared<ChRandomParticleVelocity>();
    }

    /// Function to creates particles with random shape, position and alignment each time it is called.
    /// Typically, one calls this function once per timestep.
    void EmitParticles(ChSystem& msystem, double mdt, ChFrameMoving<> pre_transform = ChFrameMoving<>()) {
        double done_particles_per_step = this->off_count;
        double done_mass_per_step = this->off_mass;

        double particles_per_step = mdt * particles_per_second;
        double mass_per_step = mdt * mass_per_second;

        // Loop for creating particles at the timestep. Note that
        // it would run forever, if there were no returns when flow amount is reached.
        while (true) {
            if (use_particle_reservoir && this->particle_reservoir <= 0)
                return;

            if (use_mass_reservoir && this->mass_reservoir <= 0)
                return;

            // Flow control: break cycle when done
            // enough particles, even with non-integer cases
            if (this->flow_mode == FLOW_PARTICLESPERSECOND) {
                if (done_particles_per_step > particles_per_step) {
                    this->off_count = done_particles_per_step - particles_per_step;
                    return;
                }
            }
            if (this->flow_mode == FLOW_MASSPERSECOND) {
                if (done_mass_per_step > mass_per_step) {
                    this->off_mass = done_mass_per_step - mass_per_step;
                    return;
                }
            }

            //
            // Create the particle
            //

            // 1) compute
            // Random position
            ChCoordsys<> mcoords;
            mcoords.pos = particle_positioner->RandomPosition();

            // 2)
            // Random alignment
            mcoords.rot = particle_aligner->RandomAlignment();

            // transform if pre_transform is used
            ChCoordsys<> mcoords_abs;
            mcoords_abs = mcoords >> pre_transform.GetCoordsys();

            // 3)
            // Random creation of particle
            std::shared_ptr<ChBody> mbody = particle_creator->RandomGenerateAndCallbacks(mcoords_abs);

            // 4)
            // Random velocity and angular speed
            ChVector3d mv_loc = particle_velocity->RandomVelocity();
            ChVector3d mw_loc = particle_angular_velocity->RandomVelocity();

            ChVector3d mv_abs;
            ChVector3d mw_abs;
            ChVector3d jitter;

            // in case everything is transformed
            if (inherit_owner_speed) {
                mv_abs = pre_transform.PointSpeedLocalToParent(mcoords.pos, mv_loc);
                mw_abs = pre_transform.TransformDirectionLocalToParent(mw_loc) + pre_transform.GetAngVelParent();
            } else {
                mv_abs = pre_transform.TransformDirectionLocalToParent(mv_loc);
                mw_abs = pre_transform.TransformDirectionLocalToParent(mw_loc);
            }
            mbody->SetPosDt(mv_abs);
            mbody->SetAngVelParent(mw_abs);

            if (this->jitter_declustering) {
                // jitter term: high speed jet clustering
                jitter = (ChRandom::Get() * mdt) * mv_abs;
                // jitter term: moving source
                jitter -= (ChRandom::Get() * mdt) * pre_transform.PointSpeedLocalToParent(mcoords.pos, VNULL);
                mbody->Move(jitter);
            }

            msystem.AddBatch(
                mbody);  // the Add() alone woud not be thread safe if called from items inserted in system's lists

            if (this->creation_callback)
                this->creation_callback->OnAddBody(mbody, mcoords_abs, *particle_creator.get());

            this->particle_reservoir -= 1;
            this->mass_reservoir -= mbody->GetMass();

            this->created_particles += 1;
            this->created_mass += mbody->GetMass();

            // Increment counters for flow control
            done_particles_per_step += 1;
            done_mass_per_step += mbody->GetMass();
        }
    }

    /// Pass an object from a ChPostCreationCallback-inherited class if you want to
    /// set additional stuff on each created particle (ex.set some random asset, set some random material, or such)
    void RegisterAddBodyCallback(std::shared_ptr<ChRandomShapeCreator::AddBodyCallback> callback) {
        creation_callback = callback;
    }

    /// Set the particle creator, that is an object whose class is
    /// inherited from ChRandomShapeCreator
    void SetParticleCreator(std::shared_ptr<ChRandomShapeCreator> mc) { particle_creator = mc; }

    /// Set the particle positioner, that generates different positions for each particle
    void SetParticlePositioner(std::shared_ptr<ChRandomParticlePosition> mc) { particle_positioner = mc; }

    /// Set the particle aligner, that generates different rotations for each particle
    void SetParticleAligner(std::shared_ptr<ChRandomParticleAlignment> mc) { particle_aligner = mc; }

    /// Set the generator of particle velocities, that generates different initial speed for each particle
    void SetParticleVelocity(std::shared_ptr<ChRandomParticleVelocity> mc) { particle_velocity = mc; }

    /// Set the generator of angular velocities, for different initial angular velocity for each particle
    void SetParticleAngularVelocity(std::shared_ptr<ChRandomParticleVelocity> mc) { particle_angular_velocity = mc; }

    /// define a flow rate measured as n.of particles per second [part/s], as by default
    /// or a flow rate measured as kg per second  [kg/s].
    /// Then, use ParticlesPerSecond() or MassPerSecond() to tune the flow.
    void SetFlowControlMode(eChFlowMode mymode) {
        this->flow_mode = mymode;
        this->off_count = 0;
        this->off_mass = 0;
    }
    /// Report if flow rate is measured as [part/s] or [kg/s].
    eChFlowMode GetFlowControlMode() { return this->flow_mode; }

    /// Access the flow rate, measured as n.of particles per second [part/s].
    /// This is meaningful only if in
    double& ParticlesPerSecond() { return particles_per_second; }

    /// Access the flow rate, measured as kg per second  [kg/s].
    double& MassPerSecond() { return mass_per_second; }

    /// Turn on this to limit on max amount of particles.
    void SetUseParticleReservoir(bool ml) { this->use_particle_reservoir = ml; }

    /// Turn on this to limit on max mass of particles.
    void SetUseMassReservoir(bool ml) { this->use_mass_reservoir = ml; }

    /// Access the max number of particles to create - after this goes to 0, the creation stops.
    /// Remember to turn on this limit with SetLimitParticleAmount()
    int& ParticleReservoirAmount() { return particle_reservoir; }

    /// Access the max mass of particles to create - after this goes to 0, the creation stops.
    /// Remember to turn on this limit with SetLimitMassAmount()
    double& MassReservoirAmount() { return mass_reservoir; }

    /// Get the total amount of created particles
    int GetTotCreatedParticles() { return created_particles; }

    /// Get the total mass of created particles
    double GetTotCreatedMass() { return created_mass; }

    /// Turn on this to have the particles 'inherit' the speed of the owner body in pre_transform.
    void SetInheritSpeed(bool mi) { this->inherit_owner_speed = mi; }

    /// Turn on this to avoid quantization if generating from an object that has high speed
    /// in pre_transform (so avoids clusters in jets of particles).
    void SetJitterDeclustering(bool mj) { this->jitter_declustering = mj; }

  private:
    eChFlowMode flow_mode;
    double particles_per_second;
    double mass_per_second;

    std::shared_ptr<ChRandomShapeCreator> particle_creator;
    std::shared_ptr<ChRandomParticlePosition> particle_positioner;
    std::shared_ptr<ChRandomParticleAlignment> particle_aligner;
    std::shared_ptr<ChRandomParticleVelocity> particle_velocity;
    std::shared_ptr<ChRandomParticleVelocity> particle_angular_velocity;

    std::shared_ptr<ChRandomShapeCreator::AddBodyCallback> creation_callback;

    int particle_reservoir;
    bool use_particle_reservoir;

    double mass_reservoir;
    bool use_mass_reservoir;

    int created_particles;
    double created_mass;

    double off_count;
    double off_mass;

    bool inherit_owner_speed;
    bool jitter_declustering;
};

}  // end of namespace particlefactory
}  // end of namespace chrono

#endif