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
// Authors: Simone Benatti
// =============================================================================
//
// Structures for links, contacts, body properties in PBD systems and their lists
//
// =============================================================================

#ifndef CH_PBD_UTILS_H
#define CH_PBD_UTILS_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkLock.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorLinearSpeed.h"
#include "chrono/physics/ChLinkMotorRotationTorque.h"
#include "chrono/physics/ChLinkUniversal.h"

namespace chrono {
	
	// Forward references
	class ChSystemPBD;

	/// Struct collecting a Chrono ChLink together with additional info needed by 
	class ChApi ChLinkPBD {
	public:

		/// Create a LinkPBD
		ChLinkPBD(ChSystemPBD* sys) { PBDsys = sys; };

		/// Copy constructor
		//ChLinkPBD(const ChLinkPBD& other);

		/// Destructor
		virtual ~ChLinkPBD() {}

		/// Correct position to respect constraint
		void SolvePositions();
	protected:
		/// Objects needed by PBD link
		// System
		ChSystemPBD* PBDsys;
		// Linked bodies 
		ChBody* Body1;
		ChBody* Body2;
		// Relative Position of the link w.r.t. body 1 & 2 respectively
		ChFrame<> f1;
		ChFrame<> f2;
		// mass properties
		double invm1;
		double invm2;
		ChMatrix33<> Inv_I1;
		ChMatrix33<> Inv_I2;
		double w1_rot;
		double w2_rot;
		double w1;
		double w2;
		// constrained DOF
		bool mask[6] = {};
		// By element-wise multiplication these vectors constrain only along the locked directions
		ChVector<> p_dir;
		// Skip the whole correction if the pos/rot not constrained at all
		bool p_free;
		bool r_free;
		// Rotational DOF
		bool r_locked = false;
		ChVector<> a = VNULL;
		// variable true for distance constraints
		bool dist_constr = false;
		// distance value for distance constraints
        double dist;
		// Lagrangian of force and torque
		double lambda_f = 0;
		double lambda_t = 0;
		ChVector<> lambda_f_dir;
		ChVector<> lambda_t_dir;
		//ChVector<> p1_old;
		//ChVector<> p2_old;
		// Tangential friction inv masses
		double w1_tf;
		double w2_tf;
		// Compliance (TODO: make it settable)
		double alpha = 0.00001;

		// Limits
		bool is_rot_limited = false;
		bool is_displ_limited = false;
		bool displ_lims[3] = {false, false, false};
		double displ_lims_low[3] = {};
		double displ_lims_high[3] = {};
		double rot_lims_low[3] = {};
		double rot_lims_high[3] = {};

		// Actuation
		bool rot_actuated = false;
		bool displ_actuated = false;
		bool speed_actuated = false;
		/// Rotation motor and linear motor are both actuated along z axis
		int actuation_dir = 2;
		/// pointer to the function determining the motor value
		std::shared_ptr<ChFunction> motor_func;
		/// old position value for speed actuated motor
		double old_val = 0;

		/// evaluate the quaternion correction depending on rot DOF
		virtual ChVector<> getQdelta();

		/// evaluate the quaternion correction depending on rot DOF
		void findRDOF();

		/// evaluate inv masses and set to 0 if the body is fixed
		void EvalMasses();

		/// If the displacement is limited or actuated, apply this
		ChVector<> ApplyDisplLimAct(ChVector<> local_disp);

		// SERIALIZATION
		/* TODO
		/// Method to allow serialization of transient data to archives.
		virtual void ArchiveOUT(ChArchiveOut& marchive) override;

		/// Method to allow deserialization of transient data from archives.
		virtual void ArchiveIN(ChArchiveIn& marchive) override;
		*/
	};

	CH_CLASS_VERSION(ChLinkPBD, 0)

	class ChApi ChLinkPBDLock : public ChLinkPBD {
		public:
			ChLinkLock* link;
			/// Create a LinkPBD
			ChLinkPBDLock(ChLinkLock* alink, ChSystemPBD* sys);

			/// Copy constructor
			//ChLinkPBD(const ChLinkPBD& other);

			/// Destructor
			virtual ~ChLinkPBDLock() {};

			/// Translates the ChLinkLock limits into PBD formulation
			void SetLimits();
	};

	CH_CLASS_VERSION(ChLinkPBDLock, 0);


	class ChApi ChLinkPBDMate : public ChLinkPBD {
	  public:
		ChLinkMateGeneric* MGlink;
		/// Create a LinkPBD
		ChLinkPBDMate(ChLinkMateGeneric* alink, ChSystemPBD* sys);

		/// Copy constructor
		//ChLinkPBD(const ChLinkPBD& other);

		/// Destructor
		virtual ~ChLinkPBDMate() {};


	};

	CH_CLASS_VERSION(ChLinkPBDMate, 0)

	class ChApi ChLinkPBDMotor : public ChLinkPBDMate {
	public:
		ChLinkMotor* MGlink;
		/// Create a LinkPBD
		ChLinkPBDMotor(ChLinkMotor* alink, ChSystemPBD* sys);

		/// Copy constructor
		//ChLinkPBD(const ChLinkPBD& other);

		/// Destructor
		virtual ~ChLinkPBDMotor() {};


};

CH_CLASS_VERSION(ChLinkPBDMotor, 0)


	class ChApi ChContactPBD : public ChLinkPBD {
		private:
			// initially the contact is dynamic since lambda_n is 0 at the first substep.
			bool is_dynamic = false;
			// distance between points along the contact normal
			double d;
			// TODO: static friction
			// dynamic friction coefficient
			double mu_d;
			// contact normal
			ChVector<> n;
			double v_n_old;

			// Tangential friction module and direction
			double lambda_contact_tf = 0;
			ChVector<> lambda_tf_dir;


		public:
			//ChContactPBD* link;
			/// Create a LinkPBD
			ChContactPBD(ChBody* body1, ChBody* body2, ChSystemPBD* sys, ChFrame<>& frame1, ChFrame<>& frame2, double frict);

			/// Copy constructor
			//ChLinkPBD(const ChLinkPBD& other);

			/// Destructor
			virtual ~ChContactPBD() {};

			/// Velocity correction due to dynamic friction
			void SolveContactPositions();

			/// Velocity correction due to dynamic friction
			void SolveVelocity();

			/// Store contact bodies old position
			/// This is a waste of memory and time, to be replaced with a more efficient ChState usage
	};

	CH_CLASS_VERSION(ChContactPBD, 0);

	class ChApi ChLinkPBDUniversal : public ChLinkPBD{
	public:
		ChLinkUniversal* Ulink;
		/// Create a LinkPBD
		ChLinkPBDUniversal(ChLinkUniversal* alink, ChSystemPBD* sys);

		/// Copy constructor
		//ChLinkPBD(const ChLinkPBD& other);

		/// Destructor
		virtual ~ChLinkPBDUniversal() {};

		ChVector<> getQdelta()override;
	};

	CH_CLASS_VERSION(ChLinkPBDUniversal, 0)

	class ChApi ChLinkPBDDistance : public ChLinkPBD {
	public:
		ChLinkDistance* link;
		/// Create a LinkPBD
		ChLinkPBDDistance(ChLinkDistance* alink, ChSystemPBD* sys);

		/// Copy constructor
		//ChLinkPBD(const ChLinkPBD& other);

		/// Destructor
		virtual ~ChLinkPBDDistance() {};

		/// Translates the ChLinkLock limits into PBD formulation
		//void SetLimits();
	};

	CH_CLASS_VERSION(ChLinkPBDDistance, 0);

	class ChApi ChLinkPBDLinActuator : public ChLinkPBD {
      public:
        ChLinkLinActuator* link;
        /// Create a LinkPBD
        ChLinkPBDLinActuator(ChLinkLinActuator* linact, ChSystemPBD* sys);

        /// Copy constructor
        // ChLinkPBD(const ChLinkPBD& other);

        /// Destructor
        virtual ~ChLinkPBDLinActuator(){};

        /// Translates the ChLinkLock limits into PBD formulation
        //void SetLimits();
    };

    CH_CLASS_VERSION(ChLinkPBDLinActuator, 0);
	/*
	/// PBD method timesteppers.
	class ChApi ChTimestepperPBD : public ChTimestepperIorder {
	protected:
	// In base class
	//ChState Y;
	//ChStateDelta dYdt;


	public:
	/// Constructor
	ChTimestepperPBD(ChIntegrable* intgr = nullptr) : ChTimestepper(intgr) { SetIntegrable(intgr); }

	/// Destructor
	virtual ~ChTimestepperIorder() {}

	/// Access the state at current time
	virtual ChState& get_Y() { return Y; }

	/// Access the derivative of state at current time
	virtual ChStateDelta& get_dYdt() { return dYdt; }

	/// Set the integrable object
	virtual void SetIntegrable(ChSystemPBD* intgr) {
	ChTimestepper::SetIntegrable(intgr);
	Y.setZero(1, intgr);
	dYdt.setZero(1, intgr);
	}
	};
	*/
}  // end namespace chrono

#endif
