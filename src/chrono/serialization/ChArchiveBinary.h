//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHARCHIVEBINARY_H
#define CHARCHIVEBINARY_H

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChLog.h"

namespace chrono {

///
/// This is a class for serializing to binary archives
///

class  ChArchiveOutBinary : public ChArchiveOut {
  public:

      ChArchiveOutBinary( ChStreamOutBinary& mostream) {
          ostream = &mostream;
      };

      virtual ~ChArchiveOutBinary() {};

      virtual void out     (ChNameValue<bool> bVal) {
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<int> bVal) {
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<double> bVal) {
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<float> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<char> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<unsigned int> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<const char*> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<std::string> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<unsigned long> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<unsigned long long> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValue<ChEnumMapperBase> bVal) {
            (*ostream) << bVal.value().GetValueAsInt();
      }

      virtual void out_array_pre (const char* name, size_t msize, const char* classname) {
            (*ostream) << msize;
      }
      virtual void out_array_between (size_t msize, const char* classname) {}
      virtual void out_array_end (size_t msize,const char* classname) {}


        // for custom c++ objects:
      virtual void out     (ChNameValue<ChFunctorArchiveOut> bVal, const char* classname, bool tracked, size_t obj_ID) {
          bVal.value().CallArchiveOut(*this);
      }


      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t obj_ID, size_t ext_ID, const char* classname) 
      {
          if (!already_inserted) {
            // New Object, we have to full serialize it
            std::string str(classname); 
            (*ostream) << str;    
            bVal.value().CallArchiveOutConstructor(*this);
            bVal.value().CallArchiveOut(*this);
          } else {
              if (obj_ID || bVal.value().IsNull() ) {
                // Object already in list. Only store obj_ID as ID
                std::string str("oID");
                (*ostream) << str;       // serialize 'this was already saved' info as "oID" string
                (*ostream) << obj_ID;    // serialize obj_ID in pointers vector as ID
              }
              if (ext_ID) {
                // Object is external. Only store ref_ID as ID
                std::string str("eID");
                (*ostream) << str;       // serialize info as "eID" string
                (*ostream) << ext_ID;    // serialize ext_ID in pointers vector as ID
              }
          }
      }

  protected:
      ChStreamOutBinary* ostream;
};





///
/// This is a class for for serializing from binary archives
///

class  ChArchiveInBinary : public ChArchiveIn {
  public:

      ChArchiveInBinary( ChStreamInBinary& mistream) {
          istream = &mistream;
      };

      virtual ~ChArchiveInBinary() {};

      virtual void in     (ChNameValue<bool> bVal) {
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<int> bVal) {
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<double> bVal) {
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<float> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<char> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<unsigned int> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<std::string> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<unsigned long> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<unsigned long long> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValue<ChEnumMapperBase> bVal) {
            int foo;
            (*istream) >>  foo;
            bVal.value().SetValueAsInt(foo);
      }
         // for wrapping arrays and lists
      virtual void in_array_pre (const char* name, size_t& msize) {
            (*istream) >> msize;
      }
      virtual void in_array_between (const char* name) {}
      virtual void in_array_end (const char* name) {}

        //  for custom c++ objects:
      virtual void in     (ChNameValue<ChFunctorArchiveIn> bVal) {
          if (bVal.flags() & NVP_TRACK_OBJECT){
              bool already_stored; size_t obj_ID;
              PutPointer(bVal.value().GetRawPtr(), already_stored, obj_ID);
          }
          bVal.value().CallArchiveIn(*this);
      }

      virtual void* in_ref          (ChNameValue<ChFunctorArchiveIn> bVal)
      {
          void* new_ptr = nullptr;

          std::string cls_name;
          (*istream) >> cls_name;

          if (cls_name == "oID") {
            size_t obj_ID = 0;
            //  Was a shared object: just get the pointer to already-retrieved
            (*istream) >> obj_ID;

            if (this->internal_id_ptr.find(obj_ID) == this->internal_id_ptr.end()) 
                    throw (ChExceptionArchive( "In object '" + std::string(bVal.name()) +"' the reference ID " + std::to_string((int)obj_ID) +" is not a valid number." ));

            bVal.value().SetRawPtr(internal_id_ptr[obj_ID]);
          }
          else if (cls_name == "eID") {
            size_t ext_ID = 0;
            // Was an external object: just get the pointer to external
            (*istream) >> ext_ID;

            if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end()) 
                    throw (ChExceptionArchive( "In object '" + std::string(bVal.name()) +"' the external reference ID " + std::to_string((int)ext_ID) +" cannot be rebuilt." ));

            bVal.value().SetRawPtr(external_id_ptr[ext_ID]);
          }
          else {
            // Dynamically create (no class factory will be invoked for non-polimorphic obj):
            // call new(), or deserialize constructor params+call new():
            bVal.value().CallArchiveInConstructor(*this, cls_name.c_str()); 

            if (bVal.value().GetRawPtr()) {
                bool already_stored; size_t obj_ID;
                PutPointer(bVal.value().GetRawPtr(), already_stored, obj_ID);
                // 3) Deserialize
                bVal.value().CallArchiveIn(*this);
            } else {
                throw(ChExceptionArchive("Archive cannot create object" + cls_name + "\n"));
            }
            new_ptr = bVal.value().GetRawPtr();
          } 

          return new_ptr;
      }

  protected:
      ChStreamInBinary* istream;
};

}  // end namespace chrono

#endif
