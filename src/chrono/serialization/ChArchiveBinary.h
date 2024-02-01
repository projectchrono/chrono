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

#ifndef CHARCHIVEBINARY_H
#define CHARCHIVEBINARY_H

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChLog.h"

namespace chrono {

///
/// This is a class for serializing to binary archives
///

class ChArchiveOutBinary : public ChArchiveOut {
  public:
    ChArchiveOutBinary(ChStreamOutBinary& mostream) { ostream = &mostream; };

    virtual ~ChArchiveOutBinary(){};

    virtual void out(ChNameValue<bool> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<int> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<double> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<float> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<char> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<unsigned int> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<const char*> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<std::string> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<unsigned long> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<unsigned long long> bVal) { (*ostream) << bVal.value(); }
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) { (*ostream) << bVal.value().GetValueAsInt(); }

    virtual void out_array_pre(ChValue& bVal, size_t msize) { (*ostream) << msize; }
    virtual void out_array_between(ChValue& bVal, size_t msize) {}
    virtual void out_array_end(ChValue& bVal, size_t msize) {}

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
        if (tracked)
        {
            std::string classname = bVal.GetClassRegisteredName();
            *ostream << "typ" << classname;

            std::string str("oID");
            *ostream << str << obj_ID;

        }
        bVal.CallArchiveOut(*this);
    }

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
        std::string classname = bVal.GetClassRegisteredName();
        if (classname.length() > 0) {
            std::string str("typ");
            (*ostream) << str << classname;
        }

        if (!already_inserted) {
            // New Object, we have to full serialize it
            std::string str("oID");
            (*ostream) << str;     // serialize 'this was already saved' info as "oID" string
            (*ostream) << obj_ID;  // serialize obj_ID in pointers vector as ID
            bVal.CallArchiveOutConstructor(*this);
            bVal.CallArchiveOut(*this);
        } else {
            if (obj_ID || bVal.IsNull()) {
                // Object already in list. Only store obj_ID as ID
                std::string str("rID");
                (*ostream) << str;     // serialize 'this was already saved' info as "oID" string
                (*ostream) << obj_ID;  // serialize obj_ID in pointers vector as ID
            }
            if (ext_ID) {
                // Object is external. Only store ref_ID as ID
                std::string str("eID");
                (*ostream) << str;     // serialize info as "eID" string
                (*ostream) << ext_ID;  // serialize ext_ID in pointers vector as ID
            }
        }
    }

  protected:
    ChStreamOutBinary* ostream;
};

///
/// This is a class for serializing from binary archives
///

class ChArchiveInBinary : public ChArchiveIn {
  public:
    ChArchiveInBinary(ChStreamInBinary& mistream) {
        istream = &mistream;
        can_tolerate_missing_tokens = false;
    };

    virtual ~ChArchiveInBinary(){};

    virtual bool in(ChNameValue<bool> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<int> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<double> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<float> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<char> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<unsigned int> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<std::string> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<unsigned long> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<unsigned long long> bVal) override { (*istream) >> bVal.value(); return true; }
    virtual bool in(ChNameValue<ChEnumMapperBase> bVal) override {
        int foo;
        (*istream) >> foo;
        bVal.value().SetValueAsInt(foo);
        return true;
    }
    // for wrapping arrays and lists
    virtual bool in_array_pre(const std::string& name, size_t& msize) override { (*istream) >> msize; return true; }
    virtual void in_array_between(const std::string& name) override {}
    virtual void in_array_end(const std::string& name) override {}

    //  for custom c++ objects:
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal) {
        if (bVal.flags() & NVP_TRACK_OBJECT) {
            std::string entry;
            size_t obj_ID = 0;

            // TODO: DARIOM check if it works this way
            (*istream) >> entry;
            if (entry == "typ") {
                (*istream) >> entry;
                (*istream) >> entry;
            }
            if (entry == "oID") {
                (*istream) >> obj_ID;
            }

            PutNewPointer(bVal.value().GetRawPtr(), obj_ID);
        }
        bVal.value().CallArchiveIn(*this);
        return true;
    }

    virtual bool in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) {
        void* new_ptr = nullptr;

        std::string entry;
        (*istream) >> entry;

        if (entry == "typ") {
            (*istream) >> true_classname;
            (*istream) >> entry;
        }

        // TODO: DARIOM check when things are not found like in JSON and XML

        if (entry == "rID") {
            size_t ref_ID = 0;
            //  Was a shared object: just get the pointer to already-retrieved
            (*istream) >> ref_ID;

            if (this->internal_id_ptr.find(ref_ID) == this->internal_id_ptr.end())
                throw(ChExceptionArchive("In object '" + std::string(bVal.name()) + "' the reference ID " +
                                         std::to_string((int)ref_ID) + " is not a valid number."));

            bVal.value().SetRawPtr(
                ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), internal_id_ptr[ref_ID]));
        } else if (entry == "eID") {
            size_t ext_ID = 0;
            // Was an external object: just get the pointer to external
            (*istream) >> ext_ID;

            if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end())
                throw(ChExceptionArchive("In object '" + std::string(bVal.name()) + "' the external reference ID " + std::to_string((int)ext_ID) + " cannot be rebuilt."));

            bVal.value().SetRawPtr(
                ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), external_id_ptr[ext_ID]));
        } else if (entry == "oID") {
            size_t obj_ID = 0;
            (*istream) >> obj_ID;

            // see ChArchiveJSON for further details
            bVal.value().CallConstructor(*this, true_classname);

            void* new_ptr_void = bVal.value().GetRawPtr();

            if (new_ptr_void) {
                PutNewPointer(new_ptr_void, obj_ID);
                // 3) Deserialize
                bVal.value().CallArchiveIn(*this, true_classname);
            } else {
                throw(ChExceptionArchive("Archive cannot create object" + true_classname + "\n"));
            }
            new_ptr = bVal.value().GetRawPtr();
        }

        *ptr = new_ptr;
        return true;
    }

  protected:
    ChStreamInBinary* istream;
};

}  // end namespace chrono

#endif
