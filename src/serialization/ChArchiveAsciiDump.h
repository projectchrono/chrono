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

#ifndef CHARCHIVEASCIIDUMP_H
#define CHARCHIVEASCIIDUMP_H


#include "serialization/ChArchiveAsciiDump.h"


namespace chrono {

/// ASCII 'LOG' ARCHIVES (only output, for debugging etc.)


///
/// This is a class for serializing to ascii logging 
///

class  ChArchiveAsciiDump : public ChArchiveOut {
  public:

      ChArchiveAsciiDump( ChStreamOutAsciiFile& mostream) {
          ostream = &mostream;
          tablevel = 0;
          use_versions = false;
      };

      virtual ~ChArchiveAsciiDump() {};

      void indent() {
          for (int i=0; i<tablevel; ++i)
              (*ostream) << "\t";
      }

      virtual void out     (ChNameValue<bool> bVal) {
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<int> bVal) {
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<double> bVal) {
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<float> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<char> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<unsigned int> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<std::string> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t\"";
            (*ostream) << bVal.value();
            (*ostream) << "\"\n";
      }
      virtual void out     (ChNameValue<unsigned long> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<unsigned long long> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out_array_pre (const char* name, size_t msize, const char* classname) {
            indent();
            (*ostream) << name << "   array of "<< msize << " [" << classname << "]\n";
            ++tablevel;
            indent();
            (*ostream) << "[ \n";
            ++tablevel;
      }
      virtual void out_array_between (size_t msize, const char* classname) {
      }
      virtual void out_array_end (size_t msize,const char* classname) {
            --tablevel;
            indent();
            (*ostream) << "] \n";
            --tablevel;
      }

        // for custom c++ objects:
      virtual void out     (ChNameValue<ChFunctorArchiveOut> bVal, const char* classname) {
            indent();
            (*ostream) << bVal.name() << "   [" << classname << "]\n";
            ++tablevel;
            bVal.value().CallArchiveOut(*this);
            --tablevel;
      }

         // for pointed objects (if pointer hasn't been already serialized, otherwise save ID)
      virtual void out_ref_abstract (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) 
      {
          indent();
          (*ostream) << bVal.name() << "->   [" << classname << "]  (class factory support)   ID=" << position <<"\n";
          ++tablevel;
          if (!already_inserted) {
              if (!bVal.value().IsNull()) {
                    // New Object, we have to full serialize it
                    bVal.value().CallArchiveOut(*this);
              } else {
                  (*ostream) << "NULL\n";
              }
          }
          --tablevel;
      }

      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal,  bool already_inserted, size_t position, const char* classname) 
      {
          indent();
          (*ostream) << bVal.name() << "->   [" << classname << "]   ID=" << position <<"\n";
          ++tablevel;
          if (!already_inserted) {
             if (!bVal.value().IsNull()) {
                    // New Object, we have to full serialize it
                    bVal.value().CallArchiveOut(*this);
              } else {
                  (*ostream) << "NULL\n";
              }
          } 
          --tablevel;
      }

  protected:
      int tablevel;
      ChStreamOutAsciiFile* ostream;
};





}  // END_OF_NAMESPACE____

#endif
