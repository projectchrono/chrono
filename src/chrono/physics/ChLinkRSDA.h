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
// =============================================================================

#ifndef CH_LINK_RSDA_H
#define CH_LINK_RSDA_H

#include "chrono/physics/ChLink.h"
#include "chrono/physics/ChBody.h"
#include "chrono/assets/ChVisualShapePointPoint.h"

#include "chrono_thirdparty/rapidjson/document.h"

namespace chrono {

/// Class for rotational spring-damper-actuator (RSDA) with the torque specified through a functor object.
/// By default, models a linear RSDA. The torque is applied in the current direction of the relative axis of rotation
/// (the Z axis of the joint reference frame). It is the user's responsibility to ensure that the kinematics of the
/// mechanism ensure that the two RSDA frames maintain their Z axes parallel.
/// The angle is measured positive from the X RSDA axis on the first body to the X RSDA axis on the second body.
class ChApi ChLinkRSDA : public ChLink {
  public:
    ChLinkRSDA();
    ChLinkRSDA(const ChLinkRSDA& other);
    ~ChLinkRSDA() {}

    /// "Virtual" copy constructor (covariant return type).
    virtual ChLinkRSDA* Clone() const override { return new ChLinkRSDA(*this); }

    /// Set spring coefficient (default: 0).
    /// Used only if no torque functor is provided (see RegisterTorqueFunctor).
    void SetSpringCoefficient(double k) { m_k = k; }

    /// Set damping coefficient (default: 0).
    /// Used only if no torque functor is provided (see RegisterTorqueFunctor).
    void SetDampingCoefficient(double r) { m_r = r; }

    /// Set constant actuation torque (default: 0).
    /// Used only if no torque functor is provided (see RegisterTorqueFunctor).
    void SetActuatorTorque(double t) { m_t = t; }

    /// Set the RSDA rest angle (in radians).
    /// By default, this is calculated from the initial configuration.
    void SetRestAngle(double rest_angle);

    /// Set the number of initial RSDA revolutions (default: 0).
    /// This number can be either positive or negative.
    void SetNumInitRevolutions(int n);

    /// Get the value of the spring coefficient.
    /// Meaningful only if no torque functor is provided.
    double GetSpringCoefficient() const { return m_k; }

    /// Get the value of the damping coefficient.
    /// Meaningful only if no torque functor is provided.
    double GetDampingCoefficient() const { return m_r; }

    /// Get the constant acutation torque.
    /// Meaningful only if no torque functor is provided.
    double GetActuatorTorque() const { return m_t; }

    /// Class to be used as a callback interface for calculating the general spring-damper torque.
    /// A derived class must implement the virtual operator().
    class ChApi TorqueFunctor {
      public:
        virtual ~TorqueFunctor() {}

        /// Calculate and return the general spring-damper torque at the specified configuration.
        virtual double evaluate(double time,            ///< current time
                                double rest_angle,      ///< undeformed angle
                                double angle,           ///< relative angle of rotation
                                double vel,             ///< relative angular speed
                                const ChLinkRSDA& link  ///< associated RSDA link
                                ) = 0;

#ifndef SWIG
        /// Optional reporting function to generate a JSON value with functor information.
        virtual rapidjson::Value exportJSON(rapidjson::Document::AllocatorType& allocator) {
            return rapidjson::Value();
        }
#endif
    };

    /// Specify the callback object for calculating the torque.
    void RegisterTorqueFunctor(std::shared_ptr<TorqueFunctor> functor) { m_torque_fun = functor; }

    /// Return the functor object for calculating the torque (may be empty).
    std::shared_ptr<TorqueFunctor> GetTorqueFunctor() const { return m_torque_fun; }

    /// Get the spring rest (free) angle.
    double GetRestAngle() const;

    /// Get current angle.
    double GetAngle() const;

    /// Get current deformation.
    double GetDeformation() const;

    /// Get current angle rate of change.
    double GetVelocity() const;

    /// Get current torque (in the direction of the torque element).
    double GetTorque() const;

    /// Get the link coordinate system, expressed relative to Body2.
    /// This represents the 'main' reference of the link: reaction torques are expressed in this coordinate system.
    virtual ChCoordsys<> GetLinkRelativeCoords() override;

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// Return the coordinate system on body1.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) override;

    /// Initialize the rotational spring by specifying the two bodies to be connected and an RSDA frame specified in the
    /// absolute frame. The RSDA is constructed such that it acts on the Z axis of the joint frame. Unless
    /// SetRestAngle() is explicitly called, the RSDA rest angle is set to 0.
    void Initialize(std::shared_ptr<ChBody> body1,  ///< first body frame
                    std::shared_ptr<ChBody> body2,  ///< second body frame
                    const ChCoordsys<>& csys        ///< RSDA frame orientation (in absolute frame)
    );

    /// Initialize the rotational spring by specifying the two bodies to be connected and RSDA frames on each body.
    /// If local = true, it is assumed that these quantities are specified in the local body frames. Otherwise, it is
    /// assumed that they are specified in the absolute frame. The RSDA is constructed such that it acts on the (assumed
    /// common) Z axis of the joint frames. Unless SetRestAngle() is explicitly called, the RSDA rest angle is
    /// calculated from the initial configuration.
    void Initialize(std::shared_ptr<ChBody> body1,  ///< first body frame
                    std::shared_ptr<ChBody> body2,  ///< second body frame
                    bool local,                     ///< true if data given in body local frames
                    const ChCoordsys<>& csys1,      ///< RSDA frame orientation on body 1
                    const ChCoordsys<>& csys2       ///< RSDA frame orientation on body 2
    );

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive) override;

    /// Method to allow deserialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive) override;

  private:
    virtual void Update(double time, bool update_assets = true) override;
    virtual void IntLoadResidual_F(const unsigned int off, ChVectorDynamic<>& R, const double c) override;
    virtual void ConstraintsFbLoadForces(double factor = 1) override;

    void CalcAngle();
    void AdjustAngle();

    // Joint frame orientations (in body local frames)
    ChCoordsys<> m_csys1;  ///< joint frame orientation on body 1
    ChCoordsys<> m_csys2;  ///< joint frame orientation on body 2

    ChVector<> m_axis;  ///< RSDA axis (expressed in absolute frame)

    double m_k;  ///< spring coefficient (if no torque functor provided)
    double m_r;  ///< damping coefficient (if no torque functor provided)
    double m_t;  ///< constant actuation (if no torque functor provided)

    std::shared_ptr<TorqueFunctor> m_torque_fun;  ///< functor for torque calculation

    int m_turns;             ///< number of revolutions
    bool m_auto_rest_angle;  ///< if true, rest angle set at initialization
    double m_rest_angle;     ///< undeformed length
    double m_last_angle;     ///< angle at previuous evaluation
    double m_angle;          ///< current angle
    double m_angle_dt;       ///< current angle rate of change
    double m_torque;         ///< resulting torque along relative axis of rotation

    friend class ChVisualShapeRotSpring;
};

CH_CLASS_VERSION(ChLinkRSDA, 0)

}  // end namespace chrono

#endif
