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

#ifndef CHARCHIVEJSON_H
#define CHARCHIVEJSON_H

#include "chrono/serialization/ChArchive.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChMathematics.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/prettywriter.h"
#include "chrono_thirdparty/rapidjson/filereadstream.h"
#include "chrono_thirdparty/rapidjson/filewritestream.h"

#include <stack>
#include <fstream>
#include <iostream>
#include <sstream>

namespace chrono {

/// ASCII 'LOG' ARCHIVES (only output, for debugging etc.)


///
/// This is a class for serializing to ascii logging 
///

class  ChArchiveOutJSON : public ChArchiveOut {
  public:

      ChArchiveOutJSON( ChStreamOutAsciiFile& mostream) {
          ostream = &mostream;
          
          (*ostream) << "{ ";
          ++tablevel;

          tablevel = 1;
          nitems.push(0);
          is_array.push(false);
      };

      virtual ~ChArchiveOutJSON() {
          --tablevel;
          nitems.pop();
          is_array.pop();

          (*ostream) << "\n}\n";
      };

      void indent() {
          for (int i=0; i<tablevel; ++i)
              (*ostream) << "\t";
      }

      void comma_cr() {
          if (this->nitems.top() > 0) {
            (*ostream) << ",";
          } 
          (*ostream) << "\n";
      }

      virtual void out     (ChNameValue<bool> bVal) {
            comma_cr();
            indent();
            if (is_array.top()==false) 
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            if (bVal.value())
                (*ostream) << "true";
            else
                (*ostream) << "false";
            ++nitems.top();
      }
      virtual void out     (ChNameValue<int> bVal) {
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"";
            (*ostream) << "\t: ";
            (*ostream) << bVal.value();
            ++nitems.top();
      }
      virtual void out     (ChNameValue<double> bVal) {
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            (*ostream) << bVal.value();
            ++nitems.top();
      }
      virtual void out     (ChNameValue<float> bVal){
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            (*ostream) << bVal.value();
            ++nitems.top();
      }
      virtual void out     (ChNameValue<char> bVal){
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            (*ostream) << (int)bVal.value();
            ++nitems.top();
      }
      virtual void out     (ChNameValue<unsigned int> bVal){
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            (*ostream) << bVal.value();
            ++nitems.top();
      }
      virtual void out     (ChNameValue<const char*> bVal){
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name()  << "\"" << "\t: ";
            (*ostream) << "\"" << bVal.value() << "\"";
            ++nitems.top();
      }
      virtual void out     (ChNameValue<std::string> bVal){
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            (*ostream) << "\"" << bVal.value() << "\"";
            ++nitems.top();
      }
      virtual void out     (ChNameValue<unsigned long> bVal){
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            (*ostream) << bVal.value();
            ++nitems.top();
      }
      virtual void out     (ChNameValue<unsigned long long> bVal){
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            (*ostream) << bVal.value();
            ++nitems.top();
      }
      virtual void out     (ChNameValue<ChEnumMapperBase> bVal) {
            comma_cr();
            indent();
            if (is_array.top()==false)
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: ";
            std::string mstr = bVal.value().GetValueAsString();
            (*ostream) << "\"" << mstr << "\"";
            ++nitems.top();
      }

      virtual void out_array_pre (const char* name, size_t msize, const char* classname) {
            comma_cr();
            if (is_array.top()==false)
            {
                indent();
                (*ostream) << "\"" << name << "\"" << "\t: ";
            }
            (*ostream) << "\n";
            indent();
            (*ostream) << "[ ";

            ++tablevel;
            nitems.push(0);
            is_array.push(true);
      }
      virtual void out_array_between (size_t msize, const char* classname) {

      }
      virtual void out_array_end (size_t msize,const char* classname) {
            --tablevel;
            nitems.pop();
            is_array.pop();

            (*ostream) << "\n";
            indent();
            (*ostream) << "]";
            ++nitems.top();
      }

        // for custom c++ objects:
      virtual void out     (ChNameValue<ChFunctorArchiveOut> bVal, const char* classname, bool tracked, size_t obj_ID) {
            comma_cr();
            if (is_array.top()==false)
            {
                indent();
                (*ostream) << "\"" << bVal.name() << "\"" << "\t: \n";
            }
            indent();
            (*ostream) << "{";
            
            ++tablevel;
            nitems.push(0);
            is_array.push(false);

            if(tracked) {
                comma_cr();
                indent();
                (*ostream) << "\"_object_ID\"\t: "  << obj_ID;
                ++nitems.top();
            }

            bVal.value().CallArchiveOut(*this);
            
            --tablevel;
            nitems.pop();
            is_array.pop();
            
            (*ostream) << "\n";
            indent();
            (*ostream) << "}";
            ++nitems.top();
      }


      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal,  bool already_inserted, size_t obj_ID, size_t ext_ID, const char* classname)  {
          comma_cr();
          indent();
          if (is_array.top()==false)
            (*ostream) << "\"" << bVal.name() << "\"" << "\t: \n";
          indent();
          (*ostream) << "{ ";
          
          ++tablevel;
          nitems.push(0);
          is_array.push(false);

          if(strlen(classname)>0) {
              comma_cr();
              indent();
              (*ostream) << "\"_type\"\t: "  << "\"" << classname << "\"";
              ++nitems.top();
          }
          
          if (!already_inserted) {
            comma_cr();
            indent();
            (*ostream) << "\"_object_ID\"\t: "  << obj_ID;
            ++nitems.top();
          
            // New Object, we have to full serialize it
            bVal.value().CallArchiveOutConstructor(*this);
            bVal.value().CallArchiveOut(*this);
          } else {
              if (obj_ID || bVal.value().IsNull() ) {
                comma_cr();
                indent();
                (*ostream) << "\"_reference_ID\"\t: "  << obj_ID;
                ++nitems.top();
              }
              if (ext_ID) {
                comma_cr();
                indent();
                (*ostream) << "\"_external_ID\"\t: "  << ext_ID;
                ++nitems.top();
              }
          }
          --tablevel;
          nitems.pop();
          is_array.pop();

          (*ostream) << "\n";
          indent();
          (*ostream) << "}";
          ++nitems.top();
      }

  protected:
      int tablevel;
      ChStreamOutAsciiFile* ostream;
      std::stack<int> nitems;
      std::stack<bool> is_array;
};





///
/// This is a class for for deserializing from JSON archives
///


class  ChArchiveInJSON : public ChArchiveIn {
  public:

      ChArchiveInJSON( ChStreamInAsciiFile& mistream) {
            istream = &mistream;

            std::stringstream buffer;
			buffer << istream->GetFstream().rdbuf();
			std::string mstr = buffer.str();
			const char* stringbuffer = mstr.c_str();

			document.Parse<0>( stringbuffer );
			if (document.HasParseError()) {
				std::string errstrA( (const char*)(&stringbuffer[ChMax((int)document.GetErrorOffset()-10,0)]) ); errstrA.resize(10);
				std::string errstrB( (const char*)(&stringbuffer[document.GetErrorOffset()]) ); errstrB.resize(20);
				throw (ChExceptionArchive("the file has bad JSON syntax," + std::to_string(document.GetParseError()) + " \n\n[...]" + errstrA + " <--- " + errstrB + "[...]\n" ));
			}
			if (!document.IsObject())
				throw (ChExceptionArchive("the file is not a valid JSON document"));

            level = &document;
            levels.push(level);
            is_array.push(false);

            tolerate_missing_tokens = false;
      }

      virtual ~ChArchiveInJSON() {};

      rapidjson::Value* GetValueFromNameOrArray(const char* mname)
      {
          rapidjson::Value* mval;
          if (this->is_array.top() == true) {
              if (!level->IsArray()) {throw (ChExceptionArchive( "Cannot retrieve from ID num in non-array object."));}
              mval = &(*level)[this->array_index.top()];
          } else {
              if (level->HasMember(mname))
                  mval = &(*level)[mname];
              else { token_notfound(mname);}
          }
          return mval;
      }

      virtual void in     (ChNameValue<bool> bVal) {
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
		    if (!mval->IsBool()) {throw (ChExceptionArchive( "Invalid true/false flag after '"+std::string(bVal.name())+"'"));}
			bVal.value() = mval->GetBool();
      }
      virtual void in     (ChNameValue<int> bVal) {
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
			if (!mval->IsInt()) {throw (ChExceptionArchive( "Invalid integer number after '"+std::string(bVal.name())+"'"));}
			bVal.value() = mval->GetInt();
      }
      virtual void in     (ChNameValue<double> bVal) {
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
		    if (!mval->IsNumber()) {throw (ChExceptionArchive( "Invalid number after '"+std::string(bVal.name())+"'"));}
			bVal.value() = mval->GetDouble();
      }
      virtual void in     (ChNameValue<float> bVal){
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
			if (!mval->IsNumber()) {throw (ChExceptionArchive( "Invalid number after '"+std::string(bVal.name())+"'"));}
			bVal.value() = (float)mval->GetDouble();
      }
      virtual void in     (ChNameValue<char> bVal){
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
			if (!mval->IsInt()) {throw (ChExceptionArchive( "Invalid char code after '"+std::string(bVal.name())+"'"));}
			bVal.value() = (char)mval->GetInt();
      }
      virtual void in     (ChNameValue<unsigned int> bVal){
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
			if (!mval->IsUint()) {throw (ChExceptionArchive( "Invalid unsigned integer number after '"+std::string(bVal.name())+"'"));}
			bVal.value() = mval->GetUint();
      }
      virtual void in     (ChNameValue<std::string> bVal){
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
            if (!mval->IsString()) {throw (ChExceptionArchive( "Invalid string after '"+std::string(bVal.name())+"'"));}
			bVal.value() = mval->GetString();
      }
      virtual void in     (ChNameValue<unsigned long> bVal){
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
            if (!mval->IsUint64()) {throw (ChExceptionArchive( "Invalid unsigned long number after '"+std::string(bVal.name())+"'"));}
			bVal.value() = (unsigned long)mval->GetUint64();
      }
      virtual void in     (ChNameValue<unsigned long long> bVal){
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
            if (!mval->IsUint64()) {throw (ChExceptionArchive( "Invalid unsigned long long number after '"+std::string(bVal.name())+"'"));}
			bVal.value() = mval->GetUint64();
      }
      virtual void in     (ChNameValue<ChEnumMapperBase> bVal) {
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
            if (!mval->IsString()) {throw (ChExceptionArchive( "Invalid string after '"+std::string(bVal.name())+"'"));}
			std::string mstr = mval->GetString();
            if (!bVal.value().SetValueAsString(mstr)) {throw (ChExceptionArchive( "Not recognized enum type '"+mstr+"'"));}
      }

         // for wrapping arrays and lists
      virtual void in_array_pre (const char* name, size_t& msize) {
            rapidjson::Value* mval = GetValueFromNameOrArray(name);
            if (!mval->IsArray()) {throw (ChExceptionArchive( "Invalid array [...] after '"+std::string(name)+"'"));}
            msize = mval->Size();
            this->levels.push(mval);
            this->level = this->levels.top();
            this->is_array.push(true);
            this->array_index.push(0);
      }
      virtual void in_array_between (const char* name) {
          ++this->array_index.top();
      }
      virtual void in_array_end (const char* name) {
          this->levels.pop();
          this->level = this->levels.top();
          this->is_array.pop();
          this->array_index.pop();
      }

        //  for custom c++ objects:
      virtual void in     (ChNameValue<ChFunctorArchiveIn> bVal) {
            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
            if (!mval->IsObject()) {throw (ChExceptionArchive( "Invalid object {...} after '"+std::string(bVal.name())+"'"));}

            if (bVal.flags() & NVP_TRACK_OBJECT){
              bool already_stored; size_t obj_ID;
              PutPointer(bVal.value().GetRawPtr(), already_stored, obj_ID);  
            }
            
            this->levels.push(mval);
            this->level = this->levels.top();
            this->is_array.push(false);

	        bVal.value().CallArchiveIn(*this);

            this->levels.pop();
            this->level = this->levels.top();
            this->is_array.pop();
      }

        // for objects to construct, return non-null ptr if new object, return null ptr if just reused obj
      virtual void* in_ref          (ChNameValue<ChFunctorArchiveIn> bVal)
      {
            void* new_ptr = nullptr;

            rapidjson::Value* mval = GetValueFromNameOrArray(bVal.name());
            if (!mval->IsObject()) {throw (ChExceptionArchive( "Invalid object {...} after '"+std::string(bVal.name())+"'"));}
            this->levels.push(mval);
            this->level = this->levels.top();
            this->is_array.push(false);

            std::string cls_name = "";
            if (bVal.value().IsPolymorphic()) {
                if (level->HasMember("_type")) {
                    if (!(*level)["_type"].IsString()) {throw (ChExceptionArchive( "Invalid string after '"+std::string(bVal.name())+"'"));}
                    cls_name = (*level)["_type"].GetString();
                }
            }
            bool is_reference = false;
            size_t ref_ID = 0;
            if (level->HasMember("_reference_ID")) {
                if (!(*level)["_reference_ID"].IsUint64()) {throw (ChExceptionArchive( "Invalid number after '"+std::string(bVal.name())+"'"));}
                ref_ID = (*level)["_reference_ID"].GetUint64();
                is_reference = true;
            }
            size_t ext_ID = 0;
            if (level->HasMember("_external_ID")) {
                if (!(*level)["_external_ID"].IsUint64()) {throw (ChExceptionArchive( "Invalid number after '"+std::string(bVal.name())+"'"));}
                ext_ID = (*level)["_external_ID"].GetUint64();
                is_reference = true;
            }

            if (!is_reference) {
                // 2) Dynamically create 
                // call new(), or deserialize constructor params+call new():
                bVal.value().CallArchiveInConstructor(*this, cls_name.c_str());
            
                if (bVal.value().GetRawPtr()) {
                    bool already_stored; size_t obj_ID;
                    PutPointer(bVal.value().GetRawPtr(), already_stored, obj_ID);
                    // 3) Deserialize
                    bVal.value().CallArchiveIn(*this);
                } else {
                    throw(ChExceptionArchive("Archive cannot create object " + std::string(bVal.name()) +"\n"));
                }
                new_ptr = bVal.value().GetRawPtr();
            } 
            else {
                if (this->internal_id_ptr.find(ref_ID) == this->internal_id_ptr.end()) {
                    throw (ChExceptionArchive( "In object '" + std::string(bVal.name()) +"' the _reference_ID " + std::to_string((int)ref_ID) +" is not a valid number." ));
                }
                bVal.value().SetRawPtr(internal_id_ptr[ref_ID]);

                if (ext_ID) {
                    if (this->external_id_ptr.find(ext_ID) == this->external_id_ptr.end()) {
                        throw (ChExceptionArchive( "In object '" + std::string(bVal.name()) +"' the _external_ID " + std::to_string((int)ext_ID) +" is not valid." ));
                    }
                    bVal.value().SetRawPtr(external_id_ptr[ext_ID]);
                }
            }
            this->levels.pop();
            this->level = this->levels.top();
            this->is_array.pop();

            return new_ptr;
      }


        /// By default, if a token is missing (respect to those required by << deserialization in c++) 
        /// an exception is thrown. This function can deactivate the exception throwing, so 
        /// the default c++ variables values are left, if no token is found.
      void SetTolerateMissingTokens(bool mtol) { tolerate_missing_tokens = mtol;}

  protected:

      void token_notfound(const char* mname) {
          if (!tolerate_missing_tokens)
            throw (ChExceptionArchive( "Cannot find '"+std::string(mname)+"'"));
      }

      ChStreamInAsciiFile* istream;
      rapidjson::Document document;
      rapidjson::Value* level;
      std::stack<rapidjson::Value*> levels; 
      std::stack<bool> is_array;
      std::stack<int>  array_index;
      bool tolerate_missing_tokens;   
};

}  // end namespace chrono

#endif
