//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCAMERA_H
#define CHCAMERA_H

#include "chrono/assets/ChAsset.h"
#include "chrono/core/ChVector.h"

namespace chrono {

/// Class for defining a videocamera point of view
/// with basic settings

class ChApi ChCamera : public ChAsset {
    // Tag needed for class factory in archive (de)serialization:
    CH_FACTORY_TAG(ChCamera)

  private:
    //
    // DATA
    //
    ChVector<> position;
    ChVector<> aimpoint;
    ChVector<> upvector;
    double angle;
    double fov;
    double hvratio;
    bool isometric;

  public:
    //
    // CONSTRUCTORS
    //

    ChCamera() {
        position = ChVector<>(0, 1, 1);
        aimpoint = VNULL;
        upvector = VECT_Y;
        angle = 50;
        fov = 3;
        hvratio = 4. / 3.;
        isometric = false;
    };

    virtual ~ChCamera(){};

    //
    // FUNCTIONS
    //

    /// Sets the position of the observer (eye point).
    /// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
    void SetPosition(ChVector<> mv) { this->position = mv; }
    /// Gets the position of the observer (eye point).
    /// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
    ChVector<> GetPosition() { return this->position; }

    /// Sets the position of the target point (aim point).
    /// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
    void SetAimPoint(ChVector<> mv) { this->aimpoint = mv; }
    /// Gets the position of the target point (aim point).
    /// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
    ChVector<> GetAimPoint() { return this->aimpoint; }

    /// Sets the position of the 'up' direction of the camera (default is vertical, VECT_Y)
    /// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
    void SetUpVector(ChVector<> mv) { this->upvector = mv.GetNormalized(); }
    /// Gets the position of the 'up' direction of the camera (default is vertical, VECT_Y)
    /// Expressed in the local coordinate system (ex. of the owner ChBody, of ChAssetLevel,..)
    ChVector<> GetUpVector() { return this->upvector; }

    /// Sets the opening angle of the lenses, in degrees, on horizontal direction.
    /// Ex. 60=wide angle, 30=tele, etc.
    void SetAngle(double mdeg) { this->angle = mdeg; }
    /// Gets the opening angle of the lenses, in degrees, on horizontal direction
    double GetAngle() { return this->angle; }

    /// Sets the Field Of View, if the visualization supports focusing.
    void SetFOV(double mf) { this->fov = mf; }
    /// Gets the Field Of View, if the visualization supports focusing.
    double GetFOV() { return this->fov; }

    /// Sets the Horizontal/Vertical size ratio. Default = 4/3.
    /// For instance, 4/3 can be used if rendering to 800x600 images with square pixels.
    void SetHVratio(double mf) { this->hvratio = mf; }
    /// Gets the Horizontal/Vertical size ratio. Default = 4/3.
    /// For instance, 4/3 can be used if rendering to 800x600 images with square pixels.
    double GetHVratio() { return this->hvratio; }

    /// Set to 'true' if you want to disable perspective and get a 'flat' view.
    /// This must be supported by the visualization system. By default is 'false'.
    void SetOrthographic(bool mb) { this->isometric = mb; }
    /// If 'true' it means that perspective projection is disabled and you get a 'flat' view.
    /// This must be supported by the visualization system. By default is 'false'.
    bool GetOrthographic() { return this->isometric; }


    //
    // SERIALIZATION
    //

    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        // version number
        marchive.VersionWrite<ChCamera>();
        // serialize parent class
        ChAsset::ArchiveOUT(marchive);
        // serialize all member data:
        marchive << CHNVP(position);
        marchive << CHNVP(aimpoint);
        marchive << CHNVP(upvector);
        marchive << CHNVP(angle);
        marchive << CHNVP(fov);
        marchive << CHNVP(hvratio);
        marchive << CHNVP(isometric);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        // version number
        int version = marchive.VersionRead<ChCamera>();
        // deserialize parent class
        ChAsset::ArchiveIN(marchive);
        // stream in all member data:
        marchive >> CHNVP(position);
        marchive >> CHNVP(aimpoint);
        marchive >> CHNVP(upvector);
        marchive >> CHNVP(angle);
        marchive >> CHNVP(fov);
        marchive >> CHNVP(hvratio);
        marchive >> CHNVP(isometric);
    }
};

CH_CLASS_VERSION(ChCamera,0)

}  // end namespace chrono

#endif
