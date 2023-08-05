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

/////////////// ChCastingMap ///////////////
ChCastingMap::ChCastingMap(const std::string& from,
                           const std::type_index& from_ti,
                           const std::string& to,
                           const std::type_index& to_ti,
                           std::function<void*(void*)> conv_ptr_fun,
                           std::function<std::shared_ptr<void>(std::shared_ptr<void>)> conv_shptr_fun) {
    ChCastingMap::AddCastingFunction(from, from_ti, to, to_ti, conv_ptr_fun, conv_shptr_fun);
}

ChCastingMap::conv_map_type& ChCastingMap::getCastingMap() {
    static ChCastingMap::conv_map_type casting_map;
    return casting_map;
}

ChCastingMap::ti_map_type& ChCastingMap::getTypeIndexMap() {
    static ChCastingMap::ti_map_type typeindex_map;
    return typeindex_map;
}

std::string ChCastingMap::GetClassnameFromPtrTypeindex(std::type_index typeindex) {
    return ChCastingMap::getTypeIndexMap()[typeindex];
}

void* ChCastingMap::Convert(const std::string& from, const std::string& to, void* vptr) {
    if (from.compare(to) == 0)
        return vptr;

    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

void* ChCastingMap::Convert(const std::string& from, const std::type_index& to_ti, void* vptr) {
    std::string to = ChCastingMap::GetClassnameFromPtrTypeindex(to_ti);
    if (to.compare("") == 0 || from.compare(to) == 0)
        return vptr;

    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

void* ChCastingMap::Convert(const std::type_index& from_ti, const std::string& to, void* vptr) {
    std::string from = ChCastingMap::GetClassnameFromPtrTypeindex(from_ti);
    if (from.compare("") == 0 || from.compare(to) == 0)
        return vptr;

    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

void* ChCastingMap::Convert(const std::type_index& from_ti, const std::type_index& to_ti, void* vptr) {
    std::string from = ChCastingMap::GetClassnameFromPtrTypeindex(from_ti);
    std::string to = ChCastingMap::GetClassnameFromPtrTypeindex(to_ti);
    if (from.compare("") == 0 || to.compare("") == 0 || from.compare(to) == 0)
        return vptr;

    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

std::shared_ptr<void> ChCastingMap::Convert(const std::string& from,
                                            const std::string& to,
                                            std::shared_ptr<void> vptr) {
    if (from.compare(to) == 0)
        return vptr;
    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

std::shared_ptr<void> ChCastingMap::Convert(const std::string& from,
                                            const std::type_index& to_ti,
                                            std::shared_ptr<void> vptr) {
    std::string to = ChCastingMap::GetClassnameFromPtrTypeindex(to_ti);
    if (to.compare("") == 0 || from.compare(to) == 0)
        return vptr;

    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

std::shared_ptr<void> ChCastingMap::Convert(const std::type_index& from_ti,
                                            const std::string& to,
                                            std::shared_ptr<void> vptr) {
    std::string from = ChCastingMap::GetClassnameFromPtrTypeindex(from_ti);
    if (from.compare("") == 0 || from.compare(to) == 0)
        return vptr;

    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

std::shared_ptr<void> ChCastingMap::Convert(const std::type_index& from_ti,
                                            const std::type_index& to_ti,
                                            std::shared_ptr<void> vptr) {
    std::string from = ChCastingMap::GetClassnameFromPtrTypeindex(from_ti);
    std::string to = ChCastingMap::GetClassnameFromPtrTypeindex(to_ti);
    if (from.compare("") == 0 || to.compare("") == 0 || from.compare(to) == 0)
        return vptr;

    bool success = true;
    return ChCastingMap::_convert(from, to, vptr, success);
}

void* ChCastingMap::_convert(const std::string& from, const std::string& to, void* vptr, bool& success) {
    ChCastingMap::conv_map_type& casting_map = ChCastingMap::getCastingMap();

    // look for a direct conversion from 'from' to 'to'
    ChCastingMap::conv_map_type::iterator conv_it = casting_map.find(std::make_pair(from, to));
    if (conv_it != casting_map.end()) {
        success = true;
        return conv_it->second.first(vptr);
    } else {
        // since a *direct* conversion does not exist, it might be that a multi-step conversion is viable:
        // from --> intermediate --> to
        // look for a conversion from 'from' to 'intermediate'; if exists, convert from 'intermediate' to 'to'
        for (ChCastingMap::conv_map_type::iterator from_it = casting_map.begin(); from_it != casting_map.end(); ++from_it) {
            if (from_it->first.first.compare(from) == 0) {
                bool from_intermediate_success = false;
                std::string intermediate = from_it->first.second;
                // convert from 'from' to 'intermediate': it should be trivial
                void* intermediate_ptr = ChCastingMap::_convert(from, intermediate, vptr, from_intermediate_success);
                assert(from_intermediate_success && "Developer error: the conversion was expected to successfully execute but something went wrong.");
                // the intermediate_ptr is now of type intermediate; let's see if it can cast from 'intermediate' to 'to'
                void* to_vptr = ChCastingMap::_convert(intermediate, to, intermediate_ptr, success);
                if (success) {
                    return to_vptr;
                }
            }
        }
    }

    success = false;
    return vptr;
}



// TODO: DARIOM this function is completely identical to the other _conver except for:
// - 'void*' replaced by 'std::shared_ptr<void>'
// - 'return conv_it->second.first(vptr)' replaced by 'return conv_it->second.second(vptr)'
std::shared_ptr<void> ChCastingMap::_convert(const std::string& from, const std::string& to, std::shared_ptr<void> vptr, bool& success) {
    ChCastingMap::conv_map_type& casting_map = ChCastingMap::getCastingMap();

    // look for a direct conversion from 'from' to 'to'
    ChCastingMap::conv_map_type::iterator conv_it = casting_map.find(std::make_pair(from, to));
    if (conv_it != casting_map.end()) {
        success = true;
        return conv_it->second.second(vptr);
    } else {
        // since a *direct* conversion does not exist, it might be that a multi-step conversion is viable:
        // from --> intermediate --> to
        // look for a conversion from 'from' to 'intermediate'; if exists, convert from 'intermediate' to 'to'
        for (ChCastingMap::conv_map_type::iterator from_it = casting_map.begin(); from_it != casting_map.end(); ++from_it) {
            if (from_it->first.first.compare(from) == 0) {
                bool from_intermediate_success = false;
                std::string intermediate = from_it->first.second;
                // convert from 'from' to 'intermediate': it should be trivial
                std::shared_ptr<void> intermediate_ptr = ChCastingMap::_convert(from, intermediate, vptr, from_intermediate_success);
                assert(from_intermediate_success && "Developer error: the conversion was expected to successfully execute but something went wrong.");
                // the intermediate_ptr is now of type intermediate; let's see if it can cast from 'intermediate' to 'to'
                std::shared_ptr<void> to_vptr = ChCastingMap::_convert(intermediate, to, intermediate_ptr, success);
                if (success) {
                    return to_vptr;
                }
            }
        }
    }

    success = false;
    return vptr;
}

void ChCastingMap::AddCastingFunction(const std::string& from,
                                      const std::type_index& from_ti,
                                      const std::string& to,
                                      const std::type_index& to_ti,
                                      std::function<void*(void*)> conv_ptr_fun,
                                      std::function<std::shared_ptr<void>(std::shared_ptr<void>)> conv_shptr_fun) {
    ChCastingMap::getCastingMap()[std::make_pair(from, to)] = std::make_pair(conv_ptr_fun, conv_shptr_fun);
    ChCastingMap::getTypeIndexMap()[from_ti] = from;
    ChCastingMap::getTypeIndexMap()[to_ti] = to;
}

void ChCastingMap::PrintCastingFunctions() {
    for (const auto& element_from : ChCastingMap::getCastingMap()) {
        std::cout << "From: " << element_from.first.first << " -> " << element_from.first.second
                  << "\nTo: " << std::endl;
    }
}

}  // end namespace chrono
