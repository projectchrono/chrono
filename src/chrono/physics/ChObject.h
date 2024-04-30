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

    int GenerateUniqueIdentifier();
};

CH_CLASS_VERSION(ChObj, 0)

/// @} chrono_physics

}  // end namespace chrono

#endif
