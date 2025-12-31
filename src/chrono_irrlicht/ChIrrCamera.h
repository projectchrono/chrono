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

#ifndef CHIRRCAMERA_H
#define CHIRRCAMERA_H

#include <irrlicht.h>

#include "chrono_irrlicht/ChApiIrr.h"

namespace chrono {
namespace irrlicht {

/// @addtogroup irrlicht_module
/// @{

/// Class to create an interactive videocamera in Irrlicht, that is similar to
/// the Maya camera but hasn't the problems that the Maya camera has in
/// Irrlicht 1.5.
/// This code is based on work by "CmdKewin" (from the Irrlicht forum).
class ChApiIrr RTSCamera : public irr::scene::ICameraSceneNode {
  public:
    RTSCamera(irr::IrrlichtDevice* devicepointer,
              irr::scene::ISceneNode* parent,
              irr::scene::ISceneManager* smgr,
              irr::s32 id,
              irr::f32 rotateSpeed = -160.0f,
              irr::f32 zoomSpeed = 1.0f,
              irr::f32 translationSpeed = 0.003f);

    virtual ~RTSCamera();

    // Events
    virtual void render() override;
    virtual bool OnEvent(const irr::SEvent& event) override;
    virtual void OnRegisterSceneNode() override;
    virtual void OnAnimate(irr::u32 timeMs) override;

    // Setup
    virtual void setInputReceiverEnabled(bool enabled) override { InputReceiverEnabled = enabled; }
    virtual bool isInputReceiverEnabled() const override;

    // Gets
    virtual const irr::core::aabbox3d<irr::f32>& getBoundingBox() const override { return BBox; }
    virtual const irr::core::matrix4& getProjectionMatrix() const override { return Projection; }
    virtual const irr::scene::SViewFrustum* getViewFrustum() const override { return &ViewArea; }
    virtual const irr::core::matrix4& getViewMatrix() const override { return View; }
    virtual const irr::core::matrix4& getViewMatrixAffector() const override { return Affector; }
    virtual const irr::core::vector3df& getUpVector() const override { return UpVector; }
    virtual const irr::core::vector3df& getTarget() const override { return Target; }
    virtual irr::f32 getNearValue() const override { return ZNear; }
    virtual irr::f32 getFarValue() const override { return ZFar; }
    virtual irr::f32 getAspectRatio() const override { return Aspect; }
    virtual irr::f32 getFOV() const override { return Fovy; }

    irr::f32 getZoomSpeed() const { return zoomSpeed; };
    irr::f32 getTranslateSpeed() const { return translateSpeed; };
    irr::f32 getRotationSpeed() const { return rotateSpeed; };

    // Sets
    virtual void setNearValue(irr::f32 zn) override;
    virtual void setFarValue(irr::f32 zf) override;
    virtual void setAspectRatio(irr::f32 aspect) override;
    virtual void setFOV(irr::f32 fovy) override;
    virtual void setViewMatrixAffector(const irr::core::matrix4& affector) override;
    virtual void setProjectionMatrix(const irr::core::matrix4& projection, bool isOrthogonal = false) override;
    virtual void setPosition(const irr::core::vector3df& newpos) override;
    virtual void setTarget(const irr::core::vector3df& newpos) override;
    virtual void setRotation(const irr::core::vector3df& rotation) override {}
    void setZUp();

    void setZoomSpeed(irr::f32 value);
    void setTranslateSpeed(irr::f32 value);
    void setRotationSpeed(irr::f32 value);

    virtual void bindTargetAndRotation(bool bound) override {}
    virtual bool getTargetAndRotationBinding(void) const override { return false; }

    // Helper Functions
    void pointCameraAtNode(ISceneNode* selected_node);
    void setMinZoom(irr::f32 amount);
    void setMaxZoom(irr::f32 amount);

    // Type Return
    virtual irr::scene::ESCENE_NODE_TYPE getType() const override { return irr::scene::ESNT_CAMERA; }

    // Public Attributes
    bool atMinDistance;
    bool atMaxDistance;
    ISceneNode* selectednode;

  protected:
    // Properties
    irr::core::vector3df Target;
    irr::core::vector3df UpVector;
    bool y_up;
    irr::core::matrix4 Projection;
    irr::core::matrix4 View;
    irr::core::matrix4 Affector;  //**ALEX
    irr::scene::SViewFrustum ViewArea;
    irr::core::aabbox3d<irr::f32> BBox;
    bool InputReceiverEnabled;
    irr::core::dimension2d<irr::f32> screenDim;
    irr::f32 Fovy;    /// Field of view, in radians.
    irr::f32 Aspect;  /// Aspect ratio.
    irr::f32 ZNear;   /// Value of the near view-plane.
    irr::f32 ZFar;    /// Z-value of the far view-plane.

    void recalculateProjectionMatrix();
    void recalculateViewArea();

  private:
    irr::IrrlichtDevice* device;
    irr::core::vector3df Pos;
    bool zooming, rotating, moving, translating;
    irr::f32 zoomSpeed;
    irr::f32 translateSpeed;
    irr::f32 rotateSpeed;
    irr::f32 rotateStartX, rotateStartY;
    irr::f32 zoomStartX, zoomStartY;
    irr::f32 translateStartX, translateStartY;
    irr::f32 currentZoom;
    irr::f32 rotX, rotY;
    irr::core::vector3df oldTarget;
    irr::core::vector2df MousePos;
    bool Keys[irr::KEY_KEY_CODES_COUNT];
    bool MouseKeys[3];
    irr::f32 targetMinDistance;
    irr::f32 targetMaxDistance;

    enum MOUSE_BUTTON { MOUSE_BUTTON_LEFT, MOUSE_BUTTON_MIDDLE, MOUSE_BUTTON_RIGHT };

    void allKeysUp();
    void allMouseKeysUp();
    bool isKeyDown(irr::s32 key);
    bool isMouseKeyDown(irr::s32 key);
    void animate();
    void updateAnimationState();
    void updateMatrices();
    virtual void setUpVector(const irr::core::vector3df& pos) override final {}
};

/// @} irrlicht_module

}  // end namespace irrlicht
}  // end namespace chrono

#endif
