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
// Authors: Radu Serban
//
// Linear spring-damper-actuator with force specified through a user-supplied
// functor object.
// Optionally, a ChLinkSpringCB can have internal dynamics, described by a
// system of ODEs. The internal states are integrated simultaneous with the
// containing system. They can be accessed and used in the force calculation.
// Such objects can be used in active suspension models.
// =============================================================================

#ifndef CH_LINK_SPRING_CB_H
#define CH_LINK_SPRING_CB_H

#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/solver/ChVariablesGenericDiagonalMass.h"

namespace chrono {

/// Class for spring-damper systems with the force specified through a functor object.
/// Optionally, these objects may have internal dynamics, represented by a system of ODEs.
///
/// \deprecated Use ChLinkTSDA instead. This class will be removed in a future Chrono release.
class ChApi
/// \cond
CH_DEPRECATED("deprecated. Use ChLinkTSDA instead.")
/// \endcond
ChLinkSpringCB : public ChLinkMarkers {
  public:
    ChLinkSpringCB();
    ChLinkSpringCB(const ChLinkSpringCB& other);
    ~ChLinkSpringCB();

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkSpringCB* Clone() const override;

    // Set spring rest (free) length.
    // Optionally, the free length can be calculated from the initial configuration (see #Initialize).
    void SetSpringRestLength(double len) { m_rest_length = len; }

    // Get current states.
    const ChVectorDynamic<>& GetStates() const { return m_states; }

    // Get the spring rest (free) length.
    double GetSpringRestLength() const { return m_rest_length; }

    double GetSpringDeform() const { return dist - m_rest_length; }
    double GetSpringLength() const { return dist; }
    double GetSpringVelocity() const { return dist_dt; }
    double GetSpringReact() const { return m_force; }

    /// Class to be used as a callback interface for calculating the general spring-damper force.
    class ChApi ForceFunctor {
      public:
        virtual ~ForceFunctor() {}

        /// Calculate and return the general spring-damper force at the specified configuration.
        virtual double operator()(double time,          ///< current time
                                  double rest_length,   ///< undeformed length
                                  double length,        ///< current length
                                  double vel,           ///< current velocity (positive when extending)
                                  ChLinkSpringCB* link  ///< back-pointer to associated link
                                  ) = 0;
    };

    /// Specify the functor object for calculating the force.
    void RegisterForceFunctor(ForceFunctor* functor) { m_force_fun = functor; }

    /// Class to be used as a callback interface for specifying the ODE, y' = f(t,y); y(0) = y0.
    class ChApi ODE {
      public:
        virtual ~ODE() {}

        /// Specify number of states (dimension of y).
        virtual int GetNumStates() const = 0;

        /// Set initial conditions.
        /// Must load y0 = y(0).
        virtual void SetInitialConditions(ChVectorDynamic<>& states,  ///< output initial conditions vector
                                          ChLinkSpringCB* link        ///< back-pointer to associated link
                                          ) = 0;

        /// Calculate and return the ODE right-hand side at the provided time and states.
        /// Must load f(t,y).
        virtual void CalculateRHS(double time,                      ///< current time
                                  const ChVectorDynamic<>& states,  ///< current states
                                  ChVectorDynamic<>& rhs,           ///< output ODE right-hand side vector
                                  ChLinkSpringCB* link              ///< back-pointer to associated link
                                  ) = 0;
    };

    /// Specify the functor object for calculating the ODE right-hand side.
    void RegisterODE(ODE* functor);

    /// Specialized initialization for springs, given the two bodies to be connected, the positions of the two anchor
    /// endpoints of the spring (each expressed in body or abs. coordinates) and the imposed rest length of the spring.
    /// Note: Two markers are automatically created and associated with the two connected bodies.
    void Initialize(
        std::shared_ptr<ChBody> body1,  ///< first body to link
        std::shared_ptr<ChBody> body2,  ///< second body to link
        bool pos_are_relative,          ///< true: following pos. are relative to bodies
        ChVector<> pos1,                ///< pos. of spring endpoint for 1st body (rel. or abs., see flag above)
        ChVector<> pos2,                ///< pos. of spring endpoint for 2nd body (rel. or abs., see flag above)
        bool auto_rest_length = true,   ///< if true, initializes the rest length as the distance between pos1 and pos2
        double rest_length = 0          ///< rest length (no need to define if auto_rest_length=true.)
    );

    /// Get the 1st spring endpoint (expressed in Body1 coordinate system)
    ChVector<> GetEndPoint1Rel() { return marker1->GetPos(); }
    /// Set the 1st spring endpoint (expressed in Body1 coordinate system)
    void SetEndPoint1Rel(const ChVector<>& mset) { marker1->Impose_Rel_Coord(ChCoordsys<>(mset, QUNIT)); }
    /// Get the 1st spring endpoint (expressed in absolute coordinate system)
    ChVector<> GetEndPoint1Abs() { return marker1->GetAbsCoord().pos; }
    /// Set the 1st spring endpoint (expressed in absolute coordinate system)
    void SetEndPoint1Abs(ChVector<>& mset) { marker1->Impose_Abs_Coord(ChCoordsys<>(mset, QUNIT)); }

    /// Get the 2nd spring endpoint (expressed in Body2 coordinate system)
    ChVector<> GetEndPoint2Rel() { return marker2->GetPos(); };
    /// Set the 2nd spring endpoint (expressed in Body2 coordinate system)
    void SetEndPoint2Rel(const ChVector<>& mset) { marker2->Impose_Rel_Coord(ChCoordsys<>(mset, QUNIT)); }
    /// Get the 1st spring endpoint (expressed in absolute coordinate system)
    ChVector<> GetEndPoint2Abs() { return marker2->GetAbsCoord().pos; }
    /// Set the 1st spring endpoint (expressed in absolute coordinate system)
    void SetEndPoint2Abs(ChVector<>& mset) { marker2->Impose_Abs_Coord(ChCoordsys<>(mset, QUNIT)); }

    /// Inherits, then also adds the spring custom forces to the C_force and C_torque.
    virtual void UpdateForces(double time) override;

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) override;

  private:
    virtual void Update(double mytime, bool update_assets = true) override;

    virtual int GetDOF() override { return m_nstates; }

    // Interface to state operations
    virtual void IntStateGather(const unsigned int off_x,
                                ChState& x,
                                const unsigned int off_v,
                                ChStateDelta& v,
                                double& T) override;
    virtual void IntStateScatter(const unsigned int off_x,
                                 const ChState& x,
                                 const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const double T,
                                 bool full_update) override;
    virtual void IntStateGatherAcceleration(const unsigned int off_a, ChStateDelta& a) override;
    virtual void IntStateScatterAcceleration(const unsigned int off_a, const ChStateDelta& a) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void IntLoadResidual_Mv(const unsigned int off,
                                    ChVectorDynamic<>& R,
                                    const ChVectorDynamic<>& v,
                                    const double c) override;
    virtual void IntToDescriptor(const unsigned int off_v,
                                 const ChStateDelta& v,
                                 const ChVectorDynamic<>& R,
                                 const unsigned int off_L,
                                 const ChVectorDynamic<>& L,
                                 const ChVectorDynamic<>& Qc) override;
    virtual void IntFromDescriptor(const unsigned int off_v,
                                   ChStateDelta& v,
                                   const unsigned int off_L,
                                   ChVectorDynamic<>& L) override;

    // Interface to the solver
    ChVariables& Variables() { return *m_variables; }
    virtual void InjectVariables(ChSystemDescriptor& mdescriptor) override;

    virtual void VariablesFbReset() override;
    virtual void VariablesFbLoadForces(double factor = 1) override;
    virtual void VariablesQbLoadSpeed() override;
    virtual void VariablesQbSetSpeed(double step = 0) override;
    virtual void VariablesFbIncrementMq() override;
    virtual void VariablesQbIncrementPosition(double step) override;

    ForceFunctor* m_force_fun;  ///< functor for force calculation
    ODE* m_ode_fun;             ///< functor for ODE specification
    int m_nstates;              ///< number of internal states
    double m_rest_length;       ///< undeform length
    double m_force;             ///< resulting force in distance coord

    ChVectorDynamic<> m_states;                   ///< vector of internal states
    ChVectorDynamic<> m_rhs;                      ///< current ODE right-hand side
    ChVariablesGenericDiagonalMass* m_variables;  ///< carrier for internal dynamics states

    friend class ChSystemParallel;
};

CH_CLASS_VERSION(ChLinkSpringCB, 0)

}  // end namespace chrono

#endif
