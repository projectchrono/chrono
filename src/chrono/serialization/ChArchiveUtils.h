// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2018 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHARCHIVEUTILS_H
#define CHARCHIVEUTILS_H

#include "chrono/serialization/ChArchive.h"
#include <unordered_map>

namespace chrono {
	
/// Helper class that simply builds a map of pointers-tags for all objects that implement a int GetTag() function,
/// i.e. custom c++ objects embedded in an object (mostly the ChSystem), that is traversed like in a serialization. 
/// The map can be used for debugging or to be used as a ExternalPointersMap()
/// of an ChArchiveIn, for transplanting data to be re-bind incrementally assuming that tags from GetTag() are unique.

class ChArchivePointerMap : public ChArchiveOut {
public:
	ChArchivePointerMap() {}

	virtual ~ChArchivePointerMap() {}

	virtual void out(ChNameValue<bool> bVal) {};
	virtual void out(ChNameValue<int> bVal) {};
	virtual void out(ChNameValue<double> bVal) {};
	virtual void out(ChNameValue<float> bVal) {};
	virtual void out(ChNameValue<char> bVal) {};
	virtual void out(ChNameValue<unsigned int> bVal) {};
	virtual void out(ChNameValue<unsigned long> bVal) {};
	virtual void out(ChNameValue<unsigned long long> bVal) {};
	virtual void out(ChNameValue<ChEnumMapperBase> bVal) {};
	virtual void out(ChNameValue<const char*> bVal) {};
	virtual void out(ChNameValue<std::string> bVal) {};
	virtual void out_array_pre(ChValue& bVal, size_t size) {};
	virtual void out_array_between(ChValue& bVal, size_t size) {};
	virtual void out_array_end(ChValue& bVal, size_t size) {};

	virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
		if (process_objects_byval && bVal.HasGetTag()) {
			if(build_map_id_ptr)
				this->pointer_map_id_ptr[bVal.CallGetTag()] = bVal.GetRawPtr();
			if(build_map_ptr_id)
				this->pointer_map_ptr_id[bVal.GetRawPtr()] = bVal.CallGetTag();
		}
		bVal.CallArchiveOut(*this);
	}
	virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
		if (!already_inserted) {
			if (process_objects_byref && bVal.HasGetTag()) {
				if (build_map_id_ptr)
					this->pointer_map_id_ptr[bVal.CallGetTag()] = bVal.GetRawPtr();
				if (build_map_ptr_id)
					this->pointer_map_ptr_id[bVal.GetRawPtr()] = bVal.CallGetTag();
			}
			bVal.CallArchiveOut(*this);
		}
	}

	virtual void out_ref_sharedptr(ChValue& bVal_wrapped, std::shared_ptr<void> bVal_shptr_wrapper, bool already_inserted, size_t obj_ID, size_t ext_ID) {
		if (!already_inserted) {
			if (process_objects_byref && bVal_wrapped.HasGetTag()) {
				
				if (build_map_id_ptr)
					this->pointer_map_id_ptr[bVal_wrapped.CallGetTag()] = bVal_wrapped.GetRawPtr();
				if (build_map_ptr_id)
					this->pointer_map_ptr_id[bVal_wrapped.GetRawPtr()] = bVal_wrapped.CallGetTag();
				
				this->shared_ptr_map.emplace(
					std::make_pair(bVal_wrapped.GetRawPtr(), 
						std::make_pair(bVal_shptr_wrapper, bVal_wrapped.GetClassRegisteredName())));
			}
			bVal_wrapped.CallArchiveOut(*this);
		}
	}


	bool process_objects_byval = false;
	bool process_objects_byref = true;
	bool build_map_id_ptr = true;
	bool build_map_ptr_id = true;

	/// Result goes here:

	std::unordered_map<size_t, void*> pointer_map_id_ptr;
	std::unordered_map<void*, size_t> pointer_map_ptr_id;

	std::unordered_map<void*, std::pair<std::shared_ptr<void>, std::string>> shared_ptr_map;
};


/// Helper class that simply sets the tag of objects, using unique tag IDs via SetTag() for c++ classes that has
/// a void SetTag(int) function, traversing the object and sub objects like in a serialization. 

class ChArchiveSetUniqueTags : public ChArchiveOut {
public:
	ChArchiveSetUniqueTags() {}

	virtual ~ChArchiveSetUniqueTags() {}

	virtual void out(ChNameValue<bool> bVal) {};
	virtual void out(ChNameValue<int> bVal) {};
	virtual void out(ChNameValue<double> bVal) {};
	virtual void out(ChNameValue<float> bVal) {};
	virtual void out(ChNameValue<char> bVal) {};
	virtual void out(ChNameValue<unsigned int> bVal) {};
	virtual void out(ChNameValue<unsigned long> bVal) {};
	virtual void out(ChNameValue<unsigned long long> bVal) {};
	virtual void out(ChNameValue<ChEnumMapperBase> bVal) {};
	virtual void out(ChNameValue<const char*> bVal) {};
	virtual void out(ChNameValue<std::string> bVal) {};
	virtual void out_array_pre(ChValue& bVal, size_t size) {};
	virtual void out_array_between(ChValue& bVal, size_t size) {};
	virtual void out_array_end(ChValue& bVal, size_t size) {};

	virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
		if (process_objects_byval && bVal.HasSetTag() && bVal.HasGetTag()) {
			if (!(skip_already_tagged && (bVal.CallGetTag() !=-1))) {
				bVal.CallSetTag(this->currentID);
				currentID++;
			}
		}
		bVal.CallArchiveOut(*this);
	}
	virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
		if (!already_inserted) {
			if (process_objects_byval && bVal.HasSetTag() && bVal.HasGetTag()) {
				if (!(skip_already_tagged && (bVal.CallGetTag() !=-1))) {
					bVal.CallSetTag(this->currentID);
					currentID++;
				}
			}
			bVal.CallArchiveOut(*this);
		}
	}

	int currentID = 1;
	bool process_objects_byval = true;
	bool process_objects_byref = true;
	bool skip_already_tagged = true;
};



}  // end namespace chrono

#endif
