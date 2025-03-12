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

#ifndef CH_OBJECT_H
#define CH_OBJECT_H

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "chrono/core/ChApiCE.h"
#include "chrono/core/ChFrame.h"

#include "chrono/assets/ChCamera.h"
#include "chrono/assets/ChVisualModel.h"

namespace chrono {

/// @addtogroup chrono_physics
/// @{

/// Base class for all Chrono objects. 
/// Each object receives a unique identifier and can be named and/or tagged.
class ChApi ChObj {
  public:
    ChObj();
    ChObj(const ChObj& other);
    virtual ~ChObj() {}

    /// "Virtual" copy constructor.
    virtual ChObj* Clone() const = 0;

    /// Get the unique integer identifier of this object.
    /// Object identifiers are generated automatically in incremental order based on the order in which objects are created.
    /// These identifiers are transient and as such are not serialized.
    /// However, user code can cache the identifier of any Chrono object and use it later (e.g., to search the item in a ChAssembly).
    int GetIdentifier() const { return m_identifier; }

    /// Set an object integer tag (default: -1).
    /// Unlike the object identifier, this tag is completely under user control and not used anywhere else in Chrono.
    /// Tags are serialized and de-serialized.
    void SetTag(int tag) { m_tag = tag; }

    /// Get the tag of this object.
    int GetTag() const { return m_tag; }

    /// Set the name of this object.
    void SetName(const std::string& myname) { m_name = myname; }

    /// Get the name of this object.
    const std::string& GetName() const { return m_name; }

    /// Gets the simulation time of this object.
    double GetChTime() const { return ChTime; }

    /// Sets the simulation time of this object.
    void SetChTime(double m_time) { ChTime = m_time; }

    /// Add an (optional) visualization model.
    /// Not that an instance of the given visual model is associated with this object, thus allowing sharing the
    /// same model among multiple objects.
    void AddVisualModel(std::shared_ptr<ChVisualModel> model);

    /// Access the visualization model (if any).
    /// Note that this model may be shared with other objects that may instance it.
    /// Returns nullptr if no visual model is present.
    std::shared_ptr<ChVisualModel> GetVisualModel() const;

    /// Add the specified visual shape to the visualization model.
    /// If this object does not have a visual model, one is created.
    void AddVisualShape(std::shared_ptr<ChVisualShape> shape, const ChFrame<>& frame = ChFrame<>());

    /// Access the specified visualization shape in the visualization model (if any).
    /// Note that no range check is performed.
    std::shared_ptr<ChVisualShape> GetVisualShape(unsigned int i) const;

    /// Add the specified FEA visualization object to the visualization model.
    /// If this object does not have a visual model, one is created.
    void AddVisualShapeFEA(std::shared_ptr<ChVisualShapeFEA> shapeFEA);

    /// Access the specified FEA visualization object in the visualization model (if any).
    /// Note that no range check is performed.
    std::shared_ptr<ChVisualShapeFEA> GetVisualShapeFEA(unsigned int i) const;

    /// Get the reference frame (expressed in and relative to the absolute frame) of the visual model.
    /// If the visual model is cloned (for example for an object modeling a particle system), this function returns
    /// the coordinate system of the specified clone.
    virtual ChFrame<> GetVisualModelFrame(unsigned int nclone = 0) const { return ChFrame<>(); }

    /// Return the number of clones of the visual model associated with this object.
    /// If the visual model is cloned (for example for an object modeling a particle system), this function should
    /// return the total number of copies of the visual model, including the "original".  The current coordinate frame
    /// of a given clone can be obtained by calling GetVisualModelFrame() with the corresponding clone identifier.
    virtual unsigned int GetNumVisualModelClones() const { return 0; }

    /// Attach a camera to this object.
    /// Multiple cameras can be attached to the same object.
    void AddCamera(std::shared_ptr<ChCamera> camera);

    /// Get the set of cameras attached to this object.
    std::vector<std::shared_ptr<ChCamera>> GetCameras() const { return cameras; }

    /// Perform any updates necessary at the current phase during the solution process.
    /// This function is called at least once per step to update auxiliary data, internal states, etc.
    /// The default implementation updates the item's time stamp and its visualization assets (if any are defined anf
    /// only if requested).
    virtual void Update(double time, bool update_assets);

    /// Utility function to update only the associated visual assets (if any).
    void UpdateVisualModel();

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& archive_out);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& archive_in);

    // Method to allow mnemonic names in (de)serialization of containers (std::vector, arrays, etc.)
    virtual std::string& ArchiveContainerName() { return m_name; }

  protected:
    double ChTime;       ///< object simulation time
    std::string m_name;  ///< object name
    int m_identifier;    ///< object unique identifier
    int m_tag;           ///< user-supplied tag

    std::shared_ptr<ChVisualModelInstance> vis_model_instance;  ///< instantiated visualization model
    std::vector<std::shared_ptr<ChCamera>> cameras;             ///< set of cameras

    int GenerateUniqueIdentifier();
};

CH_CLASS_VERSION(ChObj, 0)

/// @} chrono_physics

}  // end namespace chrono

#endif
