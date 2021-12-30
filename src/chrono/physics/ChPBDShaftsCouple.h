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

#ifndef CH_PBD_SHAFTCOUPLE_H
#define CH_PBD_SHAFTCOUPLE_H

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChShaft.h"
#include "chrono/physics/ChShaftsGear.h"
#include "chrono/physics/ChShaftsClutch.h"
#include "chrono/physics/ChShaftsPlanetary.h"
#include "chrono/physics/ChShaftsTorqueBase.h"
#include "chrono/physics/ChShaftsBody.h"

namespace chrono {
	
	// Forward references
	class ChSystemPBD;

	/// Struct collecting a Chrono ChLink together with additional info needed by 
	class ChApi ChPBDShaftsCouple {
	public:

	  /// Create a LinkPBD
      ChPBDShaftsCouple(ChSystemPBD* sys, ChShaft* shafta, ChShaft* shaftb)
          : shaft1(shafta), shaft2(shaftb), PBDsys(sys){};

		/// Copy constructor
		//ChLinkPBD(const ChLinkPBD& other);

		/// Destructor
        //virtual ~ChShaftsCouplePBD() {};
	  
	  // Will evaluate the violation and apply torque on the connected elements. Inherited classes will implement it. 
	  virtual void SolveShaftCoupling() = 0;

      // Shortcut to set shaft mass to 0 if fized, 1/I othewise
      double GetShaftInvI(ChShaft* shaft);

    protected:
        ChShaft* shaft1;
        ChShaft* shaft2;
        ChSystemPBD* PBDsys;
		// shaft couplings in PBD are violation-based like links, just 1-dimensional
        double alpha = 0;
        double lambda = 0;
	};

//CH_CLASS_VERSION(ChPBDShaftsCouple, 0)

	class ChApi ChPBDShaftsCoupleGear : public ChPBDShaftsCouple {
      public:
        /// Create a LinkPBD
        ChPBDShaftsCoupleGear(ChSystemPBD* sys, ChShaftsGear* gear);
        // Solves constraint
        void SolveShaftCoupling() override;

      private:
        ChShaftsGear* shaftGear;
		  
    };

	class ChApi ChPBDShaftsCoupleClutch : public ChPBDShaftsCouple {
      public:
        /// Create a LinkPBD
        ChPBDShaftsCoupleClutch(ChSystemPBD* sys, ChShaftsClutch* clutch);
        // Solves constraint
        void SolveShaftCoupling() override;

      private:
        ChShaftsClutch* clutchptr;
    };


    class ChApi ChPBDShaftsCouplePlanetary : public ChPBDShaftsCouple {
      public:
        /// Create a LinkPBD
        ChPBDShaftsCouplePlanetary(ChSystemPBD* sys, ChShaftsPlanetary* planetary);
        // Solves constraint
        void SolveShaftCoupling() override;

      private:
        ChShaftsPlanetary* planetaryptr;
        ChShaft* shaft3;
        double lambda2 = 0;
    };

    class ChApi ChPBDShaftsCoupleTorque: public ChPBDShaftsCouple {
      public:
        /// Create a LinkPBD
        ChPBDShaftsCoupleTorque(ChSystemPBD* sys, ChShaftsTorqueBase* torquelink);
        // Applies torque (calculated during Update) to the shafts
        void SolveShaftCoupling() override;

      private:
        ChShaftsTorqueBase* shaftTorqueptr;
        // We store the old value to subtract it before adding the new one. 
        // If we just set torque we might override any other torque acting on the shaft
        double torqueOld = 0;
    };

    class ChApi ChPBDShaftsCoupleBody : public ChPBDShaftsCouple {
      public:
        /// Create a LinkPBD
        ChPBDShaftsCoupleBody(ChSystemPBD* sys, ChShaftsBody* shaftbodylink);
        // Applies torque (calculated during Update) to the shafts
        void SolveShaftCoupling() override;

      private:
        ChShaftsBody* shaftBodyptr;
        // ptr to the constrained body
        ChBody* bodyptr;
    };

	void PopulateShaftCouplingPBD(std::vector<std::shared_ptr<ChPBDShaftsCouple>>& listPBD,
                                  const std::vector<std::shared_ptr<ChPhysicsItem>>& otherlist,
                                  ChSystemPBD* sys);

	
}  // end namespace chrono

#endif
