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

#include "chrono/core/ChClassFactory.h"

namespace chrono {

/// Access the unique class factory here. It is unique even
/// between dll boundaries. It is allocated the 1st time it is called, if null.

ChClassFactory* ChClassFactory::GetGlobalClassFactory() {
    static ChClassFactory* mfactory = 0;
    if (!mfactory)
        mfactory = new ChClassFactory;
    return mfactory;
}

/// Delete the global class factory
void ChClassFactory::DisposeGlobalClassFactory() {
    delete ChClassFactory::GetGlobalClassFactory();
}


ChCastingMap::from_map& ChCastingMap::getData() {
    static from_map data;
    return data;
}

ChCastingMap::ChCastingMap(const std::string& from, const std::string& to, std::function<void*(void*)> fun) {
    addElements(from, to, fun);
}

void ChCastingMap::addElements(const std::string& from, const std::string& to, std::function<void*(void*)> fun) {
    getData()[from][to] = fun;
}

void ChCastingMap::printData() {
    for (const auto& element_from : getData()) {
        std::cout << "From: " << element_from.first << "\nTo: " << std::endl;
        for (const auto& element_to : element_from.second) {
            std::cout << "- " << element_to.first << std::endl;
        }
    }
}

void* ChCastingMap::convert(const std::string& from, const std::string& to, void* vptr) {

    if (from.compare(to) == 0)
        return vptr;

    from_map& data = getData();

    // look for a conversion that starts from 'from'
    from_map::iterator from_it = data.find(from);
    if (from_it != data.end())
    {
        // try to find a *direct* conversion from 'from' to 'to'
        // from --> to
        to_map::iterator to_it = from_it->second.find(to);
        if (to_it != from_it->second.end()){
            return to_it->second(vptr);
        }

        // since a *direct* conversion does not exist, it might be that a multi-step conversion is viable:
        // from --> intermediate --> to
        // look for a conversion from 'intermediate' to 'to'; if exists, convert from 'from' to 'intermediate' and from 'intermediate' to 'to'
        for (to_map::iterator intermediate_it = from_it->second.begin(); intermediate_it != from_it->second.end(); ++intermediate_it){
            void* intermediate_vptr = ChCastingMap::convert(
                intermediate_it->first, // tentative intermediate class name
                to,
                intermediate_it->second(vptr) // convert the void* from 'from' to the 'intermediate' so that, if the intermediate conversion is available, we already have the result available in intermediate_ptr
            );
            if (intermediate_vptr){
                return intermediate_vptr;
            } // else continue, and try with another intermediate class
        }

    }


    return nullptr;
}



}  // end namespace chrono
