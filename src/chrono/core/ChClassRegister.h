//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010, 2012 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCLASSREGISTER_H
#define CHCLASSREGISTER_H

//////////////////////////////////////////////////
//
//   ChClassRegister.h
//
//   A class factory to register C++ classes. This can
//   be used, for example, for dynamic object creation
//   from a type ID textual name, in serialization etc.
//   (the so called 'class factory' mechanism).
//
//   These classes are based on a modified version of
//   the "Eternity" persistence library, copyright
//   1999-2003 Nicola Santi.
//
//   This functionality can be applied ONLY to classes
//   which implement the custom Chrono Run-time type
//   information (RTTI), that is classes with the
//   CH_RTTI_xxx macros from the file ChRunTimeType.h
//
//   HEADER file for CHRONO,
//	 Multibody dynamics engine
//
// ------------------------------------------------
//             http://www.projectchrono.org
// ------------------------------------------------
///////////////////////////////////////////////////

///////////////////////////////////////////////////
//
//  HOW TO REGISTER A CLASS IN THE CLASS FACTORY
//
//   Remember: you can register only classes which have the
//  'chrono run-time-type-checking' enabled, that is classes
//  which have the macro  CH_RTTI(...) inside their class{...}
//  declaration!!!
//   Assuming you have such a class, say it is named 'myEmployee',
//  you just have to put the following line in your .cpp code
//  (for example in myEmployee.cpp, but not in myEmployee.h!):
//
//  chrono::ChClassRegister<myEmployee> my_registration;
//
/////////////////////////////////////////////////////

#include <stdio.h>
#include <string>
#include <typeinfo>
#include "core/ChLog.h"
#include "core/ChRunTimeType.h"
#include <unordered_map>

namespace chrono {

/// ChClassRegisterCommon is a base abstract class which allows
/// a common management of all template classes ChClassRegister<t>.

class ChApi ChClassRegisterCommon {
  public:
    //
    // DATA
    //

    /// Each item in the gloabl list of ChClassRegister<t>
    /// object has a pointer to the next item
    /// named m_pNext. The last element has a NULL
    /// value inside m_pNext.
    ChClassRegisterCommon* m_pNext;

    //
    // METHODS
    //

    /// All instancied ChClassRegister<t> are collect inside
    /// an unique global list. this 'head' is the head of
    /// that STATIC list.
    static ChClassRegisterCommon** GetStaticHeadAddr();

    /// The signature of create method for derived classes.
    virtual void* create(std::string& class_name) = 0;

    /// The signature of get_conventional_name method for derived classes.
    virtual std::string get_conventional_name(std::string& compiler_name) = 0;
};

/// ChClassRegisterABSTRACT<t> is like the more specialized
/// ChClassRegister<t>, which you will use more often, but the
/// ..ABSTRACT version is needed to register abstract classes (those
/// with some pure virutal member, which cannot be instantiated with 'new'. )

template <class t>
class ChClassRegisterABSTRACT : public ChClassRegisterCommon {
  protected:
    //
    // DATA
    //

    /// Name of the class for dynamic creation
    std::string m_sConventionalName;

  public:
    //
    // CONSTRUCTORS
    //

    /// Default constructor: uses the chrono simulated RTTI
    /// features to get a name for the class.
    /// IT CAN BE USED ONLY ON CHRONO CLASSES WITH RTTI MACROS

    ChClassRegisterABSTRACT() {
        m_pNext = *GetStaticHeadAddr();
        *GetStaticHeadAddr() = this;

        // set name using the 'fake' RTTI system of Chrono
        this->m_sConventionalName = t::GetClassRTTI()->GetName();
    }

    /// Destructor (removes this from the global list of
    /// ChClassRegister<t> object.
    virtual ~ChClassRegisterABSTRACT() {
        ChClassRegisterCommon** ppNext = GetStaticHeadAddr();
        for (; *ppNext; ppNext = &(*ppNext)->m_pNext) {
            if (*ppNext == this) {
                *ppNext = (*ppNext)->m_pNext;
                break;
            }
        }
    }

    //
    // METHODS
    //

    virtual bool IsAbstract() { return true; }

    virtual void* create(std::string& class_name) {
        assert(this->m_sConventionalName != class_name);  // cannot instance an abstract class
        return 0;
    }

    /// Return the conventional name ID of class t if
    /// typeid(t).name() match with compiler_name.
    ///	Otherwise return an empty.
    virtual std::string get_conventional_name(std::string& compiler_name) {
        return (compiler_name == typeid(t).name()) ? this->m_sConventionalName : std::string("");
    }
};

/// ChClassRegister<t> is responsable for registration of C++ classes
/// at compile time (each class name ID enters a static list of
/// items when the program starts).
/// During run time, it can be used in whatever moment to create
/// classes given their name ID.

template <class t>
class ChClassRegister : public ChClassRegisterABSTRACT<t> {
  public:
    virtual bool IsAbstract() { return false; }

    /// Create an instance from class t if the name ID is the same
    /// specified in class_name. If the name does not match, returns NULL.
    virtual void* create(std::string& class_name) {
        return (this->m_sConventionalName == class_name) ? (void*)(new t) : 0;
    }
};

/// This function return a pointer to an object
///	that belong to the class specified by the
///	string cls_name. If success put in *ppObj
///	that pointer, otherwise put a NULL value
///	in *ppObj.

template <class T>
void create(std::string cls_name, T** ppObj) {
    ChClassRegisterCommon* pCurrent = *ChClassRegisterCommon::GetStaticHeadAddr();

    // the follow line reinterpret several times
    // invalid null pointer so compilers could
    // warn about type mismatch.
    for (; pCurrent; pCurrent = pCurrent->m_pNext)
        if ((*ppObj = reinterpret_cast<T*>(pCurrent->create(cls_name))) != NULL)
            break;
}


////////////////////////////////




/// Base class for all registration data of classes 
/// whose objects can be created via a class factory.

class ChApi ChClassRegistrationBase {
public:
    /// The signature of create method for derived classes.
    virtual void* create() = 0;
};



/// A class factory. 
/// It can create C++ objects from their string name.
/// Just use the  ChClassFactory::create()  function, given a string.
/// NOTE: desired classes must be previously registered 
///  via the CH_FACTORY_REGISTER macro, or using ClassRegister, otherwise
///  the ChClassFactory::create()  throws an exception.
/// NOTE: You do not need to explicitly create it: a static ChClassFactory
///  class factory is automatically instanced once, at the first time
///  that someone registers a class. It is consistent also across different DLLs.


class ChApi ChClassFactory {
  public:

    ChClassFactory () {
        GetLog() << "Create ChClassFactory \n";
    }
    ~ChClassFactory () {
        for(const auto & it : class_map ) {
           GetLog() << "   registered: " << it.first << "\n";
        }
        GetLog() << "Delete ChClassFactory \n";
    }

    //
    // METHODS
    //

    /// Register a class into the global class factory.
    /// Provide an unique name and a ChClassRegistration object.
    /// If multiple registrations with the same name are attempted, only one is done.
    static void ClassRegister(std::string& keyName, ChClassRegistrationBase* mregistration) {
        ChClassFactory* global_factory = GetGlobalClassFactory();

        global_factory->_ClassRegister(keyName, mregistration);
    }
    
    /// Unregister a class from the global class factory.
    /// Provide an unique name.
    static void ClassUnregister(std::string& keyName) {
        ChClassFactory* global_factory = GetGlobalClassFactory();

        global_factory->_ClassUnregister(keyName);

        if (global_factory->_GetNumberOfRegisteredClasses()==0)
            DisposeGlobalClassFactory();
    }

    /// Create from tag name, for registered classes. 
    /// Returns object as void*, must use reinterpret_cast<>() later if needed.
    /*
    static void* create(std::string& keyName) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        return global_factory->_create(keyName);
    }
    */
    /// Create from tag name, for registered classes.
    /// The created object is returned in "ptr"
    template <class T>
    static void create(std::string& keyName, T** ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        *ptr = reinterpret_cast<T*>(global_factory->_create(keyName));
    }

private:
    /// Access the unique class factory here. It is unique even 
    /// between dll boundaries. It is allocated the 1st time it is called, if null.
    static ChClassFactory* GetGlobalClassFactory();

    /// Delete the global class factory
    static void DisposeGlobalClassFactory();

    void _ClassRegister(std::string& keyName, ChClassRegistrationBase* mregistration)
    {
       class_map[keyName] = mregistration;
       GetLog() << " register class: " << keyName << "\n";
    }

    void _ClassUnregister(std::string& keyName)
    {
       class_map.erase(keyName);
    }

    bool _IsClassRegistered(std::string& keyName) {
        const auto &it = class_map.find(keyName);
        if (it != class_map.end())
            return true;
        else 
            return false;
    }

    size_t _GetNumberOfRegisteredClasses() {
        return class_map.size();
    }

    void* _create(std::string& keyName) {
        const auto &it = class_map.find(keyName);
        if (it != class_map.end()) {
            return it->second->create();
        }
        throw ("ChClassFactory::create() cannot find the class with name " + keyName + ". Please register it.\n" );
    }

private:
    std::unordered_map<std::string, ChClassRegistrationBase*> class_map;
};



/// Class for registration data of classes 
/// whose objects can be created via a class factory.

template <class t>
class ChClassRegistration : public ChClassRegistrationBase {
  protected:
    //
    // DATA
    //

    /// Name of the class for dynamic creation
    std::string m_sConventionalName;

  public:
    //
    // CONSTRUCTORS
    //

    /// Creator (adds this to the global list of
    /// ChClassRegistration<t> objects).
    ChClassRegistration() {
        // set name using the 'fake' RTTI system of Chrono
        this->m_sConventionalName = t::FactoryClassNameTag();

        // register in global class factory
        ChClassFactory::ClassRegister(this->m_sConventionalName, this);
    }

    /// Destructor (removes this from the global list of
    /// ChClassRegistration<t> objects).
    virtual ~ChClassRegistration() {

        // register in global class factory
        ChClassFactory::ClassUnregister(this->m_sConventionalName);
    }

    //
    // METHODS
    //

    virtual void* create() {
        return (void*)(new t);
    }

};








/// MACRO TO MARK CLASSES FOR CLASS FACTORY
/// Different compilers use different name decorations, so typeid(ptr).name() is 
/// not guaranteed to be the same across different platforms/compilers. 
/// The solution is adding a static function FactoryClassNameTag() in classes, 
/// that return a unique string.
/// Use this macro inside the body of a class declaration, better if just
/// at the beginning, for example:
///
/// class my_class {
///     CH_FACTORY_TAG(my_class)
/// }
///
/// NOTE! to support polimorphism, for ChArchive, an additional FactoryNameTag()  is made
/// virtual: a side effect is that the class and its children will be promoted VIRTUAL, keep this
/// in mind if you care about extreme performance with small objects (ex. 3d vectors, etc.).
/// This not a big issue anyway, as class factories in ChArchive are needed mostly when 
/// polimorphism comes into play (serialization of polimorphic objects, for example).

#define CH_FACTORY_TAG(classname)                           \
  public:                                                   \
    static const std::string& FactoryClassNameTag() {       \
        static std::string mtag(#classname);                \
        return mtag;                                        \
    }                                                       \
    virtual const std::string& FactoryNameTag() const {     \
        static std::string mtag(#classname);                \
        return mtag;                                        \
    }                                                       \


/// MACRO TO REGISTER A CLASS INTO THE GLOBAL CLASS FACTORY 
/// - Put this macro into a .cpp, where you prefer, but not into a .h header!
/// - Use it as 
///      CH_FACTORY_REGISTER(my_class)
/// - Note that my_class must be marked with the CH_FACTORY_TAG macro (see above)

#define CH_FACTORY_REGISTER(classname)                                          \
namespace class_factory {                                                       \
    static ChClassRegistration< classname > classname ## _factory_registration; \
}                                                                               \


}  // END_OF_NAMESPACE____

#endif
