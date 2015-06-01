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

#ifndef CHARCHIVE_H
#define CHARCHIVE_H


#include "core/ChApiCE.h"
#include "ChStream.h"
#include "ChSmartpointers.h"
#include <string>
#include <vector>

namespace chrono {

// forward reference
class ChArchiveOut;
class ChArchiveIn;


/// Functor to call the ArchiveOUT function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveOut
/// easier and equippable with virtual functions.

class ChFunctorArchiveOut {
public:
    virtual void CallArchiveOut(ChArchiveOut& marchive)=0;    
};

template <class TClass> 
class ChFunctorArchiveOutSpecific : public ChFunctorArchiveOut
{
private:
      void (TClass::*fpt)(ChArchiveOut&);   // pointer to member function
      TClass* pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member
      ChFunctorArchiveOutSpecific(TClass* _pt2Object, void(TClass::*_fpt)(ChArchiveOut&))
         { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveOut(ChArchiveOut& marchive)
        { (*pt2Object.*fpt)(marchive);};             // execute member function
};


/// Functor to call the ArchiveIN function for unrelated classes that
/// implemented them. This helps stripping out the templating, to make ChArchiveOut
/// easier and equippable with virtual functions.

class ChFunctorArchiveIn {
public:
    virtual void CallArchiveIn(ChArchiveIn& marchive)=0;
    virtual void CallNew(ChArchiveIn& marchive) {};
    virtual void CallNewAbstract(ChArchiveIn& marchive, const char* classname) {};
    virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) {};
    virtual void* CallGetRawPtr(ChArchiveIn& marchive) { return 0;};
};

template <class TClass> 
class ChFunctorArchiveInSpecific : public ChFunctorArchiveIn
{
private:
      void (TClass::*fpt)(ChArchiveIn&);   // pointer to member function
      TClass* pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member 
      ChFunctorArchiveInSpecific(TClass* _pt2Object, void(TClass::*_fpt)(ChArchiveIn&))
         { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveIn(ChArchiveIn& marchive)
        { (*pt2Object.*fpt)(marchive);};             // execute member function
};

template <class TClass> 
class ChFunctorArchiveInSpecificPtr : public ChFunctorArchiveIn
{
private:
      void (TClass::*fpt)(ChArchiveIn&);   // pointer to member function
      TClass** pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member 
      ChFunctorArchiveInSpecificPtr(TClass** _pt2Object, void(TClass::*_fpt)(ChArchiveIn&))
         { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveIn(ChArchiveIn& marchive)
        { (**pt2Object.*fpt)(marchive);};             // execute member function

      virtual void CallNew(ChArchiveIn& marchive)
        { *pt2Object = new(TClass); }

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { *pt2Object = static_cast<TClass*>(mptr); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(*pt2Object); };
};


template <class TClass> 
class ChFunctorArchiveInSpecificPtrAbstract : public ChFunctorArchiveIn
{
private:
      void (TClass::*fpt)(ChArchiveIn&);   // pointer to member function
      TClass** pt2Object;                    // pointer to object

public:

      // constructor - takes pointer to an object and pointer to a member 
      ChFunctorArchiveInSpecificPtrAbstract(TClass** _pt2Object, void(TClass::*_fpt)(ChArchiveIn&))
        { pt2Object = _pt2Object;  fpt=_fpt; };

      virtual void CallArchiveIn(ChArchiveIn& marchive)
        { (**pt2Object.*fpt)(marchive);};             // execute member function

      virtual void CallNewAbstract(ChArchiveIn& marchive, const char* classname)
        { chrono::create(classname, pt2Object); }

      virtual void  CallSetRawPtr(ChArchiveIn& marchive, void* mptr) 
        { *pt2Object = static_cast<TClass*>(mptr); };

      virtual void* CallGetRawPtr(ChArchiveIn& marchive) 
        { return static_cast<void*>(*pt2Object); };
};




///
/// This is a base class for name-value pairs
///

template<class T>
class  ChNameValue {
  public:
        ChNameValue(const char* mname, T& mvalue) : _name(mname), _value((T*)(& mvalue)) {}

        virtual ~ChNameValue() {};

        const char * name() const {
            return this->_name;
        }

        T & value() const {
            return *(this->_value);
        }

        const T & const_value() const {
            return *(this->_value);
        }

  protected:
        T* _value;
        const char* _name;
};



template<class T>
ChNameValue< T > make_ChNameValue(const char * name, T & t){
    return ChNameValue< T >(name, t);
}

/// Macros to create ChNameValue objects easier

#define CHNVP2(name,val) \
    make_ChNameValue(name,val)

#define CHNVP(val) \
    make_ChNameValue(#val,val)



///
/// This is a base class for archives with pointers to shared objects 
///

class ChArchive {
  protected:

    /// vector of pointers to stored/retrieved objects,
    /// to avoid saving duplicates or deadlocks
    std::vector<void*> objects_pointers;

  public:
    ChArchive() {
        Init();
    }

    virtual ~ChArchive() {};

    /// Reinitialize the vector of pointers to loaded/saved objects
    void Init() {
        objects_pointers.clear();
        objects_pointers.push_back(0);
    }
    /// Put a pointer in pointer vector, but only if it
    /// was not previously inserted. Returns position of pointer
    /// if already existing, otherwise -1.
    void PutPointer(void* object, bool& already_stored, size_t& pos) {
        for (size_t i = 0; i < objects_pointers.size(); ++i) {
            if (objects_pointers[i] == object)
            {
                already_stored = true;
                pos = i;
                return;
            }
        }
        // wasn't in list.. add to it
        objects_pointers.push_back(object);

        already_stored = false;
        pos = objects_pointers.size()-1;
        return;
    }
};



///
/// This is a base class for serializing into archives
///

class  ChArchiveOut : public ChArchive {
  public:
      virtual ~ChArchiveOut() {};

      //---------------------------------------------------
      // INTERFACES - to be implemented by children classes
      //

        // for integral types:
      virtual void out     (ChNameValue<bool> bVal) = 0;
      virtual void out     (ChNameValue<int> bVal) = 0;
      virtual void out     (ChNameValue<double> bVal) = 0;
      virtual void out     (ChNameValue<float> bVal) = 0;
      virtual void out     (ChNameValue<char> bVal) = 0;
      virtual void out     (ChNameValue<unsigned int> bVal) = 0;
      virtual void out     (ChNameValue<char*> bVal) = 0;
      virtual void out     (ChNameValue<const char*> bVal) = 0;
      virtual void out     (ChNameValue<std::string> bVal) = 0;
      virtual void out     (ChNameValue<unsigned long> bVal) = 0;
      virtual void out     (ChNameValue<unsigned long long> bVal) = 0;

        // for custom C++ objects - see 'wrapping' trick below
      virtual void out     (ChNameValue<ChFunctorArchiveOut> bVal, const char* classname) = 0;

        // for pointed objects with abstract class system (i.e. supporting class factory)
      virtual void out_ref_abstract (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) = 0;
      
        // for pointed objects without class abtraction
      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) = 0;

        // for wrapping arrays and lists
      virtual void out_array_pre (const char* name, size_t msize, const char* classname) = 0;
      virtual void out_array_between (size_t msize, const char* classname) = 0;
      virtual void out_array_end (size_t msize,const char* classname) = 0;


      //---------------------------------------------------

        // trick to wrap stl::vector container
      template<class T>
      void out     (ChNameValue< std::vector<T> > bVal) {
          this->out_array_pre(bVal.name(), bVal.value().size(), typeid(T).name());
          for (size_t i = 0; i<bVal.value().size(); ++i)
          {
              char buffer[20];
              sprintf(buffer, "el_%d", i);
              ChNameValue< T > array_val(buffer, bVal.value()[i]);
              this->out (array_val);
              this->out_array_between(bVal.value().size(), typeid(bVal.value()).name());
          }
          this->out_array_end(bVal.value().size(), typeid(bVal.value()).name());
      }

        // trick to call out_ref on ChSharedPointer:
      template<class T>
      void out     (ChNameValue< ChSharedPtr<T> > bVal) {
          bool already_stored; size_t pos;
          PutPointer(bVal.value(), already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(bVal.value()->get_ptr(), &T::ArchiveOUT);
          this->out_ref_abstract(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA), 
              already_stored, 
              pos, 
              bVal.value()->GetRTTI()->GetName() );
      }

        // trick to call out_ref on plain pointers:
      template<class T>
      void out     (ChNameValue<T*> bVal) {
          bool already_stored; size_t pos;
          PutPointer(bVal.value(), already_stored, pos);
          ChFunctorArchiveOutSpecific<T> specFuncA(bVal.value(), &T::ArchiveOUT);
          this->out_ref_abstract(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA), 
              already_stored,
              pos, 
              bVal.value()->GetRTTI()->GetName() );
      }

        // trick to apply 'virtual out..' on unrelated C++ objects that has a function "ArchiveOUT":
      template<class T>
      void out     (ChNameValue<T> bVal) {
          ChFunctorArchiveOutSpecific<T> specFuncA(&bVal.value(), &T::ArchiveOUT);
          this->out(
              ChNameValue<ChFunctorArchiveOut>(bVal.name(), specFuncA), 
              typeid(T).name() );
      }

        /// Operator to allow easy serialization as   myarchive >> mydata;
      template<class T>
      ChArchiveOut& operator<<(ChNameValue<T> bVal) {
          this->out(bVal);
          return (*this);
      }

      void VersionWrite(int mver) {
            this->out(ChNameValue<int>("version",mver));
      }
      
  protected:

};



///
/// This is a base class for serializing from archives
///

class  ChArchiveIn : public ChArchive {
  public:
      virtual ~ChArchiveIn() {};

      //---------------------------------------------------
      // INTERFACES - to be implemented by children classes
      //

        // for integral types:
      virtual void in     (ChNameValue<bool> bVal) = 0;
      virtual void in     (ChNameValue<int> bVal) = 0;
      virtual void in     (ChNameValue<double> bVal) = 0;
      virtual void in     (ChNameValue<float> bVal) = 0;
      virtual void in     (ChNameValue<char> bVal) = 0;
      virtual void in     (ChNameValue<unsigned int> bVal) = 0;
      virtual void in     (ChNameValue<char*> bVal) = 0;
      virtual void in     (ChNameValue<std::string> bVal) = 0;
      virtual void in     (ChNameValue<unsigned long> bVal) = 0;
      virtual void in     (ChNameValue<unsigned long long> bVal) = 0;

        // for custom C++ objects - see 'wrapping' trick below
      virtual void in     (ChNameValue<ChFunctorArchiveIn> bVal) = 0;

        // for pointed objects (if position != -1 , pointer has been already serialized)
      virtual void in_ref_abstract (ChNameValue<ChFunctorArchiveIn> bVal) = 0;
      virtual void in_ref          (ChNameValue<ChFunctorArchiveIn> bVal) = 0;

        // for wrapping arrays and lists
      virtual void in_array_pre (size_t& msize) = 0;
      virtual void in_array_between () = 0;
      virtual void in_array_end () = 0;

      //---------------------------------------------------

              // trick to wrap stl::vector container
      template<class T>
      void in     (ChNameValue< std::vector<T> > bVal) {
          bVal.value().clear();
          size_t arraysize;
          this->in_array_pre(arraysize);
          bVal.value().resize(arraysize);
          for (size_t i = 0; i<arraysize; ++i)
          {
              T element;
              ChNameValue< T > array_val("el", element);
              this->in (array_val);
              bVal.value()[i]=element;
              this->in_array_between();
          }
          this->in_array_end();
      }

        // trick to call in_ref on ChSharedPointer:
      template<class T>
      void in     (ChNameValue< ChSharedPtr<T> > bVal) {
          T* mptr;
          ChNameValue< T* > tempVal(&mptr, bVal.name());
          ChFunctorArchiveInSpecificPtrAbstract<T> specFuncA(&mptr, &T::ArchiveIN);
          this->in_ref_abstract(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA) );
          *bVal.value() = ChSharedPtr<T> ( *tempVal.value() );
      }

        // trick to call in_ref on plain pointers:
      template<class T>
      void in     (ChNameValue<T*> bVal) {
          ChFunctorArchiveInSpecificPtrAbstract<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in_ref_abstract(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA) );
      }

        // trick to apply 'virtual in..' on unrelated C++ objects that has a function "ArchiveIN":
      template<class T>
      void in     (ChNameValue<T> bVal) {
          ChFunctorArchiveInSpecific<T> specFuncA(&bVal.value(), &T::ArchiveIN);
          this->in(ChNameValue<ChFunctorArchiveIn>(bVal.name(), specFuncA));
      }

        /// Operator to allow easy serialization as   myarchive << mydata;
      template<class T>
      ChArchiveIn& operator>>(ChNameValue<T> bVal) {
          this->in(bVal);
          return (*this);
      }
      
      int VersionRead() {
          int mver;
          this->in(ChNameValue<int>("version",mver));
          return mver;
      }

  protected:

};



/////////////////////////////////////////////////////////////////////////
/// BINARY ARCHIVES


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
      virtual void out     (ChNameValue<char*> bVal){
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

      virtual void out_array_pre (const char* name, size_t msize, const char* classname) {
            (*ostream) << msize;
      }
      virtual void out_array_between (size_t msize, const char* classname) {}
      virtual void out_array_end (size_t msize,const char* classname) {}


        // for custom c++ objects:
      virtual void out     (ChNameValue<ChFunctorArchiveOut> bVal, const char* classname) {
            bVal.value().CallArchiveOut(*this);
      }

        // for pointed objects (if pointer hasn't been already serialized, otherwise save offset)
      virtual void out_ref_abstract (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position, const char* classname) 
      {
          if (!already_inserted) {
            // New Object, we have to full serialize it
            std::string str(classname);
 GetLog() << "-out_ref_abstract classname: " << classname << "\n";
            (*ostream) << str;    // serialize class type as string (platform/compiler independent), for class factory later
            bVal.value().CallArchiveOut(*this);
          } else {
            // Object already in list. Only store position as ID
            std::string str("ID");
            (*ostream) << str;       // serialize 'this was already saved' info
            (*ostream) << position;  // serialize position in pointers vector as ID
          }
      }

      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal, bool already_inserted, size_t position,  const char* classname) 
      {
          if (!already_inserted) {
            // New Object, we have to full serialize it
            std::string str("OBJ"); // not usable for class factory.
            (*ostream) << str;    // serialize class type
            bVal.value().CallArchiveOut(*this);
          } else {
            // Object already in list. Only store position
            std::string str("ID");
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
      virtual void in     (ChNameValue<char*> bVal){
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

         // for wrapping arrays and lists
      virtual void in_array_pre (size_t& msize) {
            (*istream) >> msize;
      }
      virtual void in_array_between () {}
      virtual void in_array_end () {}

        //  for custom c++ objects:
      virtual void in     (ChNameValue<ChFunctorArchiveIn> bVal) {
            bVal.value().CallArchiveIn(*this);
      }

      // for pointed objects (if position != -1 , pointer has been already serialized)
      virtual void in_ref_abstract (ChNameValue<ChFunctorArchiveIn> bVal) 
      {
          std::string cls_name;
          (*istream) >> cls_name;
GetLog() << "- in_ref_abstract classname: " << cls_name << "\n";
          if (cls_name != "ID") {
            // 2) Dynamically create using class factory
            bVal.value().CallNewAbstract(*this, cls_name.c_str()); 

            if (bVal.value().CallGetRawPtr(*this)) {
                objects_pointers.push_back(bVal.value().CallGetRawPtr(*this));
                // 3) Deserialize
                bVal.value().CallArchiveIn(*this);
            } else {
                throw(ChException("Archive cannot create abstract object"));
            }

          } else {
            int pos = 0;
            // 2b) Was a shared object: just get the pointer to already-retrieved
            (*istream) >> pos;

            bVal.value().CallSetRawPtr(*this, objects_pointers[pos]);
          }
      }

      virtual void in_ref          (ChNameValue<ChFunctorArchiveIn> bVal)
      {
          std::string cls_name;
          (*istream) >> cls_name;

          if (cls_name != "ID") {
            // 2) Dynamically create using class factory
            bVal.value().CallNew(*this);
            
            if (bVal.value().CallGetRawPtr(*this)) {
                objects_pointers.push_back(bVal.value().CallGetRawPtr(*this));
                // 3) Deserialize
                bVal.value().CallArchiveIn(*this);
            } else {
                throw(ChException("Archive cannot create object"));
            }

          } else {
            int pos = 0;
            // 2b) Was a shared object: just get the pointer to already-retrieved
            (*istream) >> pos;

            bVal.value().CallSetRawPtr(*this, objects_pointers[pos]);
          }
      }

  protected:
      ChStreamInBinary* istream;
};


 

/////////////////////////////////////////////////////////////////////////
/// ASCII 'LOG' ARCHIVES (only output, for debugging etc.)


///
/// This is a class for serializing to ascii logging 
///

class  ChArchiveOutAsciiLogging : public ChArchiveOut {
  public:

      ChArchiveOutAsciiLogging( ChStreamOutAsciiFile& mostream) {
          ostream = &mostream;
          tablevel = 0;
      };

      virtual ~ChArchiveOutAsciiLogging() {};

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
      virtual void out     (ChNameValue<char*> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t\"";
            (*ostream) << bVal.value();
            (*ostream) << "\"\n";
      }
      virtual void out     (ChNameValue<const char*> bVal){
            indent();
            (*ostream) << bVal.name();
            (*ostream) << "\t\"";
            (*ostream) << bVal.value();
            (*ostream) << "\"\n";
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
            --tablevel;
            indent();
            (*ostream) << ", \n";
            ++tablevel;
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
            // New Object, we have to full serialize it
            bVal.value().CallArchiveOut(*this);
          }
          --tablevel;
      }

      virtual void out_ref          (ChNameValue<ChFunctorArchiveOut> bVal,  bool already_inserted, size_t position, const char* classname) 
      {
          indent();
          (*ostream) << bVal.name() << "->   [" << classname << "]   ID=" << position <<"\n";
          ++tablevel;
          if (!already_inserted) {
            // New Object, we have to full serialize it
            bVal.value().CallArchiveOut(*this);
          } 
          --tablevel;
      }

  protected:
      int tablevel;
      ChStreamOutAsciiFile* ostream;
};




 




}  // END_OF_NAMESPACE____

#endif
