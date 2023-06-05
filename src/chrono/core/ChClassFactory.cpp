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



ChCastingMap::ChCastingMap(const std::string& from, const std::type_index& from_ti, const std::string& to, const std::type_index& to_ti, std::function<void*(void*)> fun) {
    AddCastingFunction(from, from_ti, to, to_ti, fun);
}

ChCastingMap::from_map_type& ChCastingMap::getCastingMap() {
    static from_map_type casting_map;
    return casting_map;
}

ChCastingMap::ti_map_type& ChCastingMap::getTypeIndexMap() {
    static ti_map_type typeindex_map;
    return typeindex_map;
}


void* ChCastingMap::Convert(const std::type_index& from_it, const std::type_index& to_it, void* vptr){
        std::string from = getTypeIndexMap()[from_it];
        std::string to = getTypeIndexMap()[to_it];
        return ChCastingMap::Convert(from, to, vptr);
}

void* ChCastingMap::Convert(const std::string& from, const std::type_index& to_it, void* vptr){
        std::string to = getTypeIndexMap()[to_it];
        return ChCastingMap::Convert(from, to, vptr);
}

void* ChCastingMap::Convert(const std::type_index& from_it, const std::string& to, void* vptr){
        std::string from = getTypeIndexMap()[from_it];
        return ChCastingMap::Convert(from, to, vptr);
}

void* ChCastingMap::Convert(const std::string& from, const std::string& to, void* vptr) {

    if (from.compare(to) == 0)
        return vptr;

    from_map_type& casting_map = getCastingMap();

    // look for a conversion that starts from 'from'
    from_map_type::iterator from_it = casting_map.find(from);
    if (from_it != casting_map.end())
    {
        // try to find a *direct* conversion from 'from' to 'to'
        // from --> to
        to_map_type::iterator to_it = from_it->second.find(to);
        if (to_it != from_it->second.end()){
            return to_it->second(vptr);
        }

        // since a *direct* conversion does not exist, it might be that a multi-step conversion is viable:
        // from --> intermediate --> to
        // look for a conversion from 'intermediate' to 'to'; if exists, convert from 'from' to 'intermediate' and from 'intermediate' to 'to'
        for (to_map_type::iterator intermediate_it = from_it->second.begin(); intermediate_it != from_it->second.end(); ++intermediate_it){
            void* intermediate_vptr = ChCastingMap::Convert(
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


void ChCastingMap::AddCastingFunction(const std::string& from, const std::type_index& from_ti, const std::string& to, const std::type_index& to_ti, std::function<void*(void*)> fun) {
    getCastingMap()[from][to] = fun;
    getTypeIndexMap()[from_ti] = from;
    getTypeIndexMap()[to_ti] = to;
}

void ChCastingMap::PrintCastingFunctions() {
    for (const auto& element_from : getCastingMap()) {
        std::cout << "From: " << element_from.first << "\nTo: " << std::endl;
        for (const auto& element_to : element_from.second) {
            std::cout << "- " << element_to.first << std::endl;
        }
    }
}




}  // end namespace chrono
