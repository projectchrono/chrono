//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010-2012 Alessandro Tasora
// Copyright (c) 2013 Project Chrono
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHARCHIVEBINARY_H
#define CHARCHIVEBINARY_H


#include "serialization/ChArchive.h"
#include "core/ChLog.h"

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

      virtual void out     (ChNameValueOut<bool> bVal) {
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<int> bVal) {
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<double> bVal) {
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<float> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<char> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<unsigned int> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<const char*> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<std::string> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<unsigned long> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<unsigned long long> bVal){
            (*ostream) << bVal.value();
      }
      virtual void out     (ChNameValueOut<ChEnumMapperOutBase> bVal) {
            (*ostream) << bVal.value().GetValueAsInt();
      }

      virtual void out_array_pre (const char* name, size_t msize, const char* classname) {
            (*ostream) << msize;
      }
      virtual void out_array_between (size_t msize, const char* classname) {}
      virtual void out_array_end (size_t msize,const char* classname) {}


        // for custom c++ objects:
      virtual void out     (ChNameValueOut<ChFunctorArchiveOut> bVal, const char* classname, bool tracked, size_t position) {
          bVal.value().CallArchiveOut(*this);
      }

        // for pointed objects (if pointer hasn't been already serialized, otherwise save offset)
      virtual void out_ref_abstract (ChNameValueOut<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) 
      {
          if (!already_inserted) {
            // New Object, we have to full serialize it
            std::string str(classname);
            (*ostream) << str;    // serialize class type as string (platform/compiler independent), for class factory later
            bVal.value().CallArchiveOut(*this);
          } else {
            // Object already in list. Only store position as ID
            std::string str("POS");
            (*ostream) << str;       // serialize 'this was already saved' info
            (*ostream) << position;  // serialize position in pointers vector as ID
          }
      }

      virtual void out_ref          (ChNameValueOut<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position,  const char* classname) 
      {
          if (!already_inserted) {
            // New Object, we have to full serialize it
            std::string str("OBJ"); // not usable for class factory.
            (*ostream) << str;    // serialize class type
            bVal.value().CallArchiveOut(*this);
          } else {
            // Object already in list. Only store position
            std::string str("POS");
            (*ostream) << str;       // serialize 'this was already saved' info
            (*ostream) << position;  // serialize position in pointers vector
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

      virtual void in     (ChNameValueIn<bool> bVal) {
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<int> bVal) {
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<double> bVal) {
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<float> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<char> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<unsigned int> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<std::string> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<unsigned long> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<unsigned long long> bVal){
            (*istream) >> bVal.value();
      }
      virtual void in     (ChNameValueIn<ChEnumMapperInBase> bVal) {
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
      virtual void in     (ChNameValueIn<ChFunctorArchiveIn> bVal) {
          if (bVal.flags() & NVP_TRACK_OBJECT){
              objects_pointers.push_back(bVal.value().CallGetRawPtr(*this));
          }
          bVal.value().CallArchiveIn(*this);
      }

      // for pointed objects (if position != -1 , pointer has been already serialized)
      virtual void in_ref_abstract (ChNameValueIn<ChFunctorArchiveIn> bVal) 
      {
          std::string cls_name;
          (*istream) >> cls_name;

          if (!(cls_name == "POS")) {
            // 2) Dynamically create using class factory
            bVal.value().CallNewAbstract(*this, cls_name.c_str()); 

            if (bVal.value().CallGetRawPtr(*this)) {
                objects_pointers.push_back(bVal.value().CallGetRawPtr(*this));
                // 3) Deserialize
                bVal.value().CallArchiveIn(*this);
            } else {
                throw(ChExceptionArchive("Archive cannot create abstract object \'" + cls_name + "\' " ));
            }

          } else {
            size_t pos = 0;
            // Was a shared object: just get the pointer to already-retrieved
            (*istream) >> pos;

            bVal.value().CallSetRawPtr(*this, objects_pointers[pos]);
          }
      }

      virtual void in_ref          (ChNameValueIn<ChFunctorArchiveIn> bVal)
      {
          std::string cls_name;
          (*istream) >> cls_name;

          if (!(cls_name == "POS")) {
            // 2) Dynamically create using class factory
            bVal.value().CallNew(*this);
            
            if (bVal.value().CallGetRawPtr(*this)) {
                objects_pointers.push_back(bVal.value().CallGetRawPtr(*this));
                // 3) Deserialize
                bVal.value().CallArchiveIn(*this);
            } else {
                throw(ChExceptionArchive("Archive cannot create object"));
            }

          } else {
            size_t pos = 0;
            //  Was a shared object: just get the pointer to already-retrieved
            (*istream) >> pos;

            bVal.value().CallSetRawPtr(*this, objects_pointers[pos]);
          }
      }

  protected:
      ChStreamInBinary* istream;
};




}  // END_OF_NAMESPACE____

#endif
