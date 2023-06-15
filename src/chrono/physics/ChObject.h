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
// Authors: Alessandro Tasora, Radu Serban
// =============================================================================

#ifndef CHOBJECT_H
#define CHOBJECT_H

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "chrono/core/ChLog.h"
#include "chrono/core/ChMath.h"

#include <vector>

namespace chrono {

/// @addtogroup chrono_physics
/// @{

// Forward references
class ChVar;
class ChTag;

/// Base class for items which can be named, deleted, copied. etc. as in the editor of a 3d modeler.
class ChApi ChObj {
  private:
    std::string m_name;  ///< name of object
    int m_identifier;    ///< object identifier

  protected:
    double ChTime;  ///< the time of simulation for the object

  public:
    ChObj();
    ChObj(const ChObj& other);
    virtual ~ChObj() {}

    /// "Virtual" copy constructor.
    /// Concrete derived classes must implement this.
    virtual ChObj* Clone() const = 0;

    /// Gets the numerical identifier of the object.
    int GetIdentifier() const { return m_identifier; }
    /// Sets the numerical identifier of the object.
    void SetIdentifier(int id) { m_identifier = id; }

    /// Gets the simulation time of this object
    double GetChTime() const { return ChTime; }
    /// Sets the simulation time of this object.
    void SetChTime(double m_time) { ChTime = m_time; }

    /// Gets the name of the object as C Ascii null-terminated string -for reading only!
    const char* GetName() const { return m_name.c_str(); }
    /// Sets the name of this object, as ascii string
    void SetName(const char myname[]) { m_name = myname; }

    /// Gets the name of the object as C Ascii null-terminated string.
    std::string GetNameString() const { return m_name; }
    /// Sets the name of this object, as std::string
    void SetNameString(const std::string& myname) { m_name = myname; }

    // Set-get generic LONG flags, passed as reference

    void MFlagsSetAllOFF(int& mflag) { mflag = 0; }
    void MFlagsSetAllON(int& mflag) {
        mflag = 0;
        mflag = ~mflag;
    }
    void MFlagSetON(int& mflag, int mask) { mflag |= mask; }
    void MFlagSetOFF(int& mflag, int mask) { mflag &= ~mask; }
    int MFlagGet(int& mflag, int mask) { return (mflag & mask); }

    /// Method to allow serialization of transient data to archives.
    virtual void ArchiveOut(ChArchiveOut& marchive);

    /// Method to allow de-serialization of transient data from archives.
    virtual void ArchiveIn(ChArchiveIn& marchive);

    // Method to allow mnemonic names in (de)serialization of containers (std::vector, arrays, etc.)
    virtual std::string& ArchiveContainerName() { return m_name; }
};

CH_CLASS_VERSION(ChObj, 0)

/// @} chrono_physics

}  // end namespace chrono

#endif
