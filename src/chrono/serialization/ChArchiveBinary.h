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

#include <cstring>

#include "chrono/serialization/ChArchive.h"

namespace chrono {

///
/// This is a class for serializing to binary archives
///

class ChArchiveOutBinary : public ChArchiveOut {
  public:
    ChArchiveOutBinary(std::ostream& mostream) { ostream = &mostream; }

    virtual ~ChArchiveOutBinary() {}

    virtual void out(ChNameValue<bool> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<int> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<double> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<float> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<char> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<unsigned int> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<unsigned long> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<unsigned long long> bVal) { binaryWrite(bVal.value()); }
    virtual void out(ChNameValue<ChEnumMapperBase> bVal) {
        int int_val = bVal.value().GetValueAsInt();
        binaryWrite(int_val);
    }

    virtual void out(ChNameValue<const char*> bVal) { binaryWrite(bVal.value()); }

    virtual void out(ChNameValue<std::string> bVal) { binaryWrite(bVal.value()); }

    virtual void out_array_pre(ChValue& bVal, size_t size) { binaryWrite(size); }
    virtual void out_array_between(ChValue& bVal, size_t size) {}
    virtual void out_array_end(ChValue& bVal, size_t size) {}

    // for custom c++ objects:
    virtual void out(ChValue& bVal, bool tracked, size_t obj_ID) {
        if (tracked) {
            ////TODO: DARIOM here the original code uses const char* instead of std::string as in any other place
            binaryWrite("typ");
            binaryWrite(bVal.GetClassRegisteredName());

            binaryWrite(std::string("oID"));
            binaryWrite(obj_ID);
        }
        bVal.CallArchiveOut(*this);
    }

    virtual void out_ref(ChValue& bVal, bool already_inserted, size_t obj_ID, size_t ext_ID) {
        std::string classname = bVal.GetClassRegisteredName();
        if (classname.length() > 0) {
            binaryWrite(std::string("typ"));
            binaryWrite(classname);
        }

        if (!already_inserted) {
            // new object, we have to full serialize it
            binaryWrite(std::string("oID"));  // serialize 'this was already saved' info as "oID" string
            binaryWrite(obj_ID);              // serialize obj_ID in pointers vector as ID
            bVal.CallArchiveOutConstructor(*this);
            bVal.CallArchiveOut(*this);
        } else {
            if (obj_ID || bVal.IsNull()) {
                // Object already in list. Only store obj_ID as ID
                binaryWrite(std::string("rID"));  // serialize 'this was already saved' info as "rID" string
                binaryWrite(obj_ID);              // serialize obj_ID in pointers vector as ID
            }
            if (ext_ID) {
                // Object is external. Only store ref_ID as ID
                binaryWrite(std::string("eID"));  // serialize info as "eID" string
                binaryWrite(ext_ID);              // serialize ext_ID in pointers vector as ID
            }
        }
    }

  protected:
    std::ostream* ostream;

    ////TODO: shall I specify that T is a reference or should I let the compiler optimize it?
    template <class T>
    std::ostream& binaryWrite(T val) {
        return ostream->write(reinterpret_cast<char*>(&val), sizeof(T));
    }

    template <>
    std::ostream& binaryWrite<std::string>(std::string val) {
        ////TODO: int might not have the same sizeof in every architecture
        ////TODO: behaviour differ from char* because of the length!!!
        //// should be 'return binaryWrite(val.c_str());'

        int str_len = (int)strlen(val.c_str());
        binaryWrite(str_len);
        return ostream->write(val.c_str(), str_len);
    }

    template <>
    std::ostream& binaryWrite<bool>(bool val) {
        ////TODO: could be return ostream->write(val ? "1" : "0", 1);
        char bool_val = static_cast<char>(val);
        return ostream->write(&bool_val, 1);
    }

    template <>
    std::ostream& binaryWrite<const char*>(const char* val) {
        int str_len = (int)strlen(val) + 1;
        binaryWrite(str_len);
        return ostream->write(val, str_len);
    }
};

///
/// This is a class for serializing from binary archives
///

class ChArchiveInBinary : public ChArchiveIn {
  public:
    ChArchiveInBinary(std::istream& mistream) {
        istream = &mistream;
        can_tolerate_missing_tokens = false;
    };

    virtual ~ChArchiveInBinary(){};

    virtual bool in(ChNameValue<bool> bVal) override {
        char bool_char;
        istream->read(&bool_char, sizeof(char));
        bVal.value() = static_cast<bool>(bool_char);
        return true;
    }

    ////Can they merged in just one instance?
    virtual bool in(ChNameValue<int> bVal) override {
        binaryRead(bVal.value());
        return true;
    }
    virtual bool in(ChNameValue<double> bVal) override {
        binaryRead(bVal.value());
        return true;
    }
    virtual bool in(ChNameValue<float> bVal) override {
        binaryRead(bVal.value());
        return true;
    }
    virtual bool in(ChNameValue<char> bVal) override {
        binaryRead(bVal.value());
        return true;
    }
    virtual bool in(ChNameValue<unsigned int> bVal) override {
        binaryRead(bVal.value());
        return true;
    }
    virtual bool in(ChNameValue<unsigned long> bVal) override {
        binaryRead(bVal.value());
        return true;
    }
    virtual bool in(ChNameValue<unsigned long long> bVal) override {
        binaryRead(bVal.value());
        return true;
    }

    virtual bool in(ChNameValue<std::string> bVal) override {
        binaryRead(bVal.value());
        return true;
    }
    virtual bool in(ChNameValue<ChEnumMapperBase> bVal) override {
        int foo;
        binaryRead(foo);
        bVal.value().SetValueAsInt(foo);
        return true;
    }

    // for wrapping arrays and lists
    virtual bool in_array_pre(const std::string& name, size_t& size) override {
        binaryRead(size);
        return true;
    }
    virtual void in_array_between(const std::string& name) override {}
    virtual void in_array_end(const std::string& name) override {}

    //  for custom c++ objects:
    virtual bool in(ChNameValue<ChFunctorArchiveIn> bVal) {
        if (bVal.flags() & NVP_TRACK_OBJECT) {
            std::string entry;
            size_t obj_ID = 0;

            // TODO: DARIOM check if it works this way
            binaryRead(entry);
            if (entry == "typ") {
                binaryRead(entry);
                binaryRead(entry);
            }
            if (entry == "oID") {
                binaryRead(obj_ID);
            }

            PutNewPointer(bVal.value().GetRawPtr(), obj_ID);
        }
        bVal.value().CallArchiveIn(*this);
        return true;
    }

    virtual bool in_ref(ChNameValue<ChFunctorArchiveIn> bVal, void** ptr, std::string& true_classname) {
        void* new_ptr = nullptr;

        std::string entry;
        binaryRead(entry);

        if (entry == "typ") {
            binaryRead(true_classname);
            binaryRead(entry);
        }

        // TODO: DARIOM check when things are not found like in JSON and XML

        if (entry == "rID") {
            size_t ref_ID = 0;
            //  Was a shared object: just get the pointer to already-retrieved
            binaryRead(ref_ID);

            if (this->internal_id_ptr.find(ref_ID) == this->internal_id_ptr.end())
                throw(ChExceptionArchive("In object '" + std::string(bVal.name()) + "' the reference ID " +
                                         std::to_string((int)ref_ID) + " is not a valid number."));

            bVal.value().SetRawPtr(
                ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), internal_id_ptr[ref_ID]));
        } else if (entry == "eID") {
            size_t ext_ID = 0;
            // Was an external object: just get the pointer to external
            binaryRead(ext_ID);

            if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end())
                throw(ChExceptionArchive("In object '" + std::string(bVal.name()) + "' the external reference ID " +
                                         std::to_string((int)ext_ID) + " cannot be rebuilt."));

            bVal.value().SetRawPtr(
                ChCastingMap::Convert(true_classname, bVal.value().GetObjectPtrTypeindex(), external_id_ptr[ext_ID]));
        } else if (entry == "oID") {
            size_t obj_ID = 0;
            binaryRead(obj_ID);

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

        ////TODO: DARIOM check if it is needed
        ////else
        ////    bVal.value().SetRawPtr(nullptr);

        *ptr = new_ptr;
        return true;
    }

  protected:
    std::istream* istream;

    template <class T>
    std::istream& binaryRead(T& val) {
        return istream->read(reinterpret_cast<char*>(&val), sizeof(T));
    }

    template <>
    std::istream& binaryRead<std::string>(std::string& val) {
        ////TODO: DARIOM int might not have the same sizeof in every architecture

        val = "";
        int str_len = 0;
        binaryRead(str_len);
        val.reserve(str_len);

        ////TODO: DARIOM awkward way to skip \0 at the end of the string while still parsing the stream
        char buf[2];
        buf[1] = '\0';
        for (int i = 0; i < str_len; i++) {
            istream->read(buf, 1);
            val.append(buf);
        }

        return *istream;
        // std::remove(val.begin(), val.end(), '\0');
    }

    template <>
    std::istream& binaryRead<bool>(bool& val) {
        char bool_val;
        return istream->read(&bool_val, 1);
        val = static_cast<bool>(bool_val);
    }

    template <>
    std::istream& binaryRead<char*>(char*& val) {
        int str_len = 0;
        binaryRead(str_len);
        return istream->read(val, str_len);
    }
};

}  // end namespace chrono

#endif
