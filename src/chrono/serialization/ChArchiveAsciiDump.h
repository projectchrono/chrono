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

#ifndef CHARCHIVEASCIIDUMP_H
#define CHARCHIVEASCIIDUMP_H

#include "chrono/serialization/ChArchive.h"

namespace chrono {

/// ASCII 'LOG' ARCHIVES (only output, for debugging etc.)


///
/// This is a class for serializing to ascii logging 
///

class  ChArchiveAsciiDump : public ChArchiveOut {
  public:

      ChArchiveAsciiDump( ChStreamOutAscii& mostream) {
          ostream = &mostream;
          tablevel = 0;
          use_versions = false;
          suppress_names= false;
      };

      virtual ~ChArchiveAsciiDump() {};

      /// If true, the variables namesare not printed. 
      /// Useful when used for GetLog() << ...  (for more compact formatting).
      void SetSuppressNames(bool msu) {suppress_names = msu;}

      /// Access the stream used by the archive.
      ChStreamOutAscii* GetStream() {return ostream;}

      void indent() {
          for (int i=0; i<tablevel; ++i)
              (*ostream) << "\t";
      }

      virtual void out     (ChNameValue<bool> bVal) {
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<int> bVal) {
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<double> bVal) {
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<float> bVal){
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
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
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<std::string> bVal){
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << "\"";
            (*ostream) << bVal.value();
            (*ostream) << "\"\n";
      }
      virtual void out     (ChNameValue<unsigned long> bVal){
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<unsigned long long> bVal){
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << bVal.value();
            (*ostream) << "\n";
      }
      virtual void out     (ChNameValue<ChEnumMapperBase> bVal) {
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "\t";
            (*ostream) << "\"";
            std::string mstr = bVal.value().GetValueAsString();
            (*ostream) << mstr;
            (*ostream) << "\"\n";
      }

      virtual void out_array_pre (const char* name, size_t msize, const char* classname) {
            indent();
            if (!suppress_names) {
                (*ostream) << name << "  ";
            }
            (*ostream) << "array of "<< msize << " [" << classname << "]\n";
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
      virtual void out     (ChNameValue<ChFunctorArchiveOut> bVal, const char* classname, bool tracked, size_t obj_ID) {
            indent();
            if (!suppress_names) 
                (*ostream) << bVal.name() << "  "; 
            (*ostream) << "[" << classname << "]";
            if (tracked)
                (*ostream) << " (tracked)   ID= " << obj_ID; 
            if (this->use_versions)
                (*ostream) << " version=" << bVal.value().GetClassVersion();
            (*ostream) << " \n";
            ++tablevel;
            bVal.value().CallArchiveOut(*this);
            --tablevel;
      }

      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal,  bool already_inserted, size_t obj_ID, size_t ext_ID, const char* classname) 
      {
          indent();
          if (!suppress_names) 
                (*ostream) << bVal.name(); 
          (*ostream) << "->";
          if(strlen(classname)>0) {
                (*ostream) << " [" << classname << "] (registered type)";
          } else {
                (*ostream) << " [" << bVal.value().GetTypeidName() << "]";
          }
          if (obj_ID)
            (*ostream) << "  ID=" << obj_ID;
          if (ext_ID)
            (*ostream) << "  external_ID=" << ext_ID;
          if (this->use_versions)
            (*ostream) << " version=" << bVal.value().GetClassVersion();
          (*ostream) << "\n";
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
      ChStreamOutAscii* ostream;
      bool suppress_names;
};




/// This is used to stream out in 'readable' form on a ChStreamOutAscii 
/// stream whatever C++ object that implements the archive serialization, i.e. 
/// objects that have ArchiveOUT implemented.
/// For example:  GetLog() << mymatrix;

template <class T>
ChStreamOutAscii & operator<<(ChStreamOutAscii &mstream, const T& obj) {
    std::vector<char> mvect;
    ChStreamOutAsciiVector mtempstream(&mvect);
    mtempstream.SetNumFormat(mstream.GetNumFormat());
    ChArchiveAsciiDump marchive(mtempstream);
    // this avoids printing too much except the object:
    marchive.SetCutAllPointers(true);
    marchive.SetSuppressNames(true);
    marchive.SetUseVersions(false);
    marchive << CHNVP(obj,"");
    std::string mystring(mtempstream.GetVector()->begin(),mtempstream.GetVector()->end());
    return mstream << mystring;
}

}  // end namespace chrono

#endif
