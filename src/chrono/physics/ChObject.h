//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHOBJECT_H
#define CHOBJECT_H

//////////////////////////////////////////////////
//
//   ChObject.h
//
// Base class for objects which can be renamed,
// copied, etc. Provides interface to link objects to
// item in hosting applications, like geometric objects
// in the editor of a 3d modeler.
//
// ------------------------------------------------
//             www.deltaknowledge.com
// ------------------------------------------------
///////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <string>
#include <math.h>
#include <float.h>
#include <memory>

#include "core/ChLog.h"
#include "core/ChMath.h"
#include "core/ChLists.h"

#include <vector>

namespace chrono {

/// @addtogroup chrono_physics
/// @{

// Forward references
class ChVar;
class ChTag;

/// Base class for items which can be named, deleted, copied. etc. as in the editor of a 3d modeler.
///
/// Each ChObject has a pointer to user data (for example, the user data can be the encapsulating
/// object in case of implementation as a plugin for 3D modeling software.
///
/// Also, each ChObj object has a 32 bit identifier, in case unique identifiers are used
/// (hash algorithms, etc.)
class ChApi ChObj {
    // Chrono simulation of RTTI, needed for serialization
    CH_RTTI_ROOT(ChObj);

  private:
    //
    // DATA
    //

    // name of object
    std::string name;

    // ID for referencing
    int identifier;

  protected:
    // the time of simulation for the object
    double ChTime;

  public:
    //
    //	CONSTRUCTORS/DELETION
    //

    ChObj();
    virtual ~ChObj();

    void Copy(ChObj* source);

    //
    // FUNCTIONS
    //

    /// Gets the numerical identifier of the object.
    int GetIdentifier() const { return identifier; }
    /// Sets the numerical identifier of the object.
    void SetIdentifier(int id) { identifier = id; }

    /// Gets the simulation time of this object
    double GetChTime() const { return ChTime; }
    /// Sets the simulation time of this object.
    void SetChTime(double m_time) { ChTime = m_time; }

    /// Gets the name of the object as C Ascii null-terminated string -for reading only!
    const char* GetName() const;
    /// Sets the name of this object, as ascii string
    void SetName(const char myname[]);

    /// Gets the name of the object as C Ascii null-terminated string.
    std::string GetNameString() const;
    /// Sets the name of this object, as std::string
    void SetNameString(const std::string& myname);

    // Set-get generic LONG flags, passed as reference

    void MFlagsSetAllOFF(int& mflag) { mflag = 0; }
    void MFlagsSetAllON(int& mflag) {
        mflag = 0;
        mflag = ~mflag;
    }
    void MFlagSetON(int& mflag, int mask) { mflag |= mask; }
    void MFlagSetOFF(int& mflag, int mask) { mflag &= ~mask; }
    int MFlagGet(int& mflag, int mask) { return (mflag & mask); };

    //
    // STREAMING
    //

            /// Method to allow serialization of transient data in archives.
    virtual void ArchiveOUT(ChArchiveOut& marchive)
    {
        marchive.VersionWrite(1);

        // stream out all member data
        marchive << CHNVP(name);
        marchive << CHNVP(identifier);
        marchive << CHNVP(ChTime);
    }

    /// Method to allow de serialization of transient data from archives.
    virtual void ArchiveIN(ChArchiveIn& marchive) 
    {
        int version = marchive.VersionRead();

        // stream out all member data
        marchive >> CHNVP(name);
        marchive >> CHNVP(identifier);
        marchive >> CHNVP(ChTime);
    }

};

// Functions to manipulate STL containers of ChObj objects

template <class T, class Iterator>
T ChContainerSearchFromName(const char* m_name, Iterator from, Iterator to) {
    Iterator iter = from;
    while (iter != to) {
        if (!strcmp(m_name, (*iter)->GetName()))
            return (*iter);
        iter++;
    }
    return T(0);
}

template <class T, class Iterator>
T ChContainerSearchFromID(int myID, Iterator from, Iterator to) {
    Iterator iter = from;
    while (iter != to) {
        if (myID == (*iter)->GetIdentifier())
            return (*iter);
        iter++;
    }
    return T(0);
}

/// @} chrono_physics

}  // END_OF_NAMESPACE____

#endif
