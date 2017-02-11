//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

#ifndef CHCLASSFACTORY_H
#define CHCLASSFACTORY_H


//
//
//  HOW TO REGISTER A CLASS IN THE CLASS FACTORY
//
//   Assuming you have such a class, say it is named 'myEmployee',
//  you just have to put the following line in your .cpp code
//  (for example in myEmployee.cpp, but not in myEmployee.h!):
//
//  CH_FACTORY_REGISTER(myEmployee)
//
//  Also, there is a problem: class names cannot be inferred from
//  standard C++ stuff like:  typeid(myclass).name() because the result
//  is compiler-dependent, with different name decorations on different 
//  platforms. To overcome this issue, it is strongly suggested to put 
//  the CH_FACTORY_TAG macro in the class declaration, as in:
//  
//  class myclass {
//      CH_FACTORY_TAG(myclass)
//   public:
//      ...
//   };
//
//

#include <cstdio>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "chrono/core/ChLog.h"
#include "chrono/core/ChTemplateExpressions.h"

namespace chrono {

// forward decl.
class ChArchiveIn;


/// Base class for all registration data of classes 
/// whose objects can be created via a class factory.

class ChApi ChClassRegistrationBase {
public:
    /// The signature of create method for derived classes. Calls new().
    virtual void* create() = 0;

    /// Call the ArchiveInCreate(ChArchiveIn&) function if available (deserializes constructor params and return new()),
    /// otherwise just call new().
    virtual void* archive_in_create(ChArchiveIn& marchive) = 0;

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
    }
    ~ChClassFactory () {
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
    /// The created object is returned in "ptr"
    template <class T>
    static void create(std::string& keyName, T** ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        *ptr = reinterpret_cast<T*>(global_factory->_create(keyName));
    }

    /// Create from tag name, for registered classes.
    /// If a static T* ArchiveInCreate(ChArchiveIn&) function is available, call it instead.
    /// The created object is returned in "ptr"
    template <class T>
    static void archive_in_create(std::string& keyName, ChArchiveIn& marchive, T** ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        *ptr = reinterpret_cast<T*>(global_factory->_archive_in_create(keyName, marchive));
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
       //GetLog() << " register class: " << keyName << "\n";
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
    void* _archive_in_create(std::string& keyName, ChArchiveIn& marchive) {
        const auto &it = class_map.find(keyName);
        if (it != class_map.end()) {
            return it->second->archive_in_create(marchive);
        }
        throw ("ChClassFactory::archive_in_create() cannot find the class with name " + keyName + ". Please register it.\n" );
    }

private:
    std::unordered_map<std::string, ChClassRegistrationBase*> class_map;
};


/// Macro to create a  ChDetect_ArchiveINconstructor  
CH_CREATE_MEMBER_DETECTOR(ArchiveINconstructor)




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
        return _create();
    }

    virtual void* archive_in_create(ChArchiveIn& marchive) {
        return _archive_in_create(marchive);
    }
 
protected:

    template <class Tc=t>
    typename enable_if< std::is_default_constructible<Tc>::value, void* >::type
    _create() {
        return reinterpret_cast<void*>(new Tc);
    }
    template <class Tc=t>
    typename enable_if< !std::is_default_constructible<Tc>::value, void* >::type
    _create() {
        throw ("ChClassFactory::create() failed for class " + std::string(typeid(Tc).name())  + ": it has no default constructor.\n" );
    }

    template <class Tc=t>
    typename enable_if< ChDetect_ArchiveINconstructor<Tc>::value, void* >::type
    _archive_in_create(ChArchiveIn& marchive) {
        return reinterpret_cast<void*>(Tc::ArchiveINconstructor(marchive));
    }
    template <class Tc=t>
    typename enable_if< !ChDetect_ArchiveINconstructor<Tc>::value, void* >::type 
    _archive_in_create(ChArchiveIn& marchive) {
        return reinterpret_cast<void*>(new Tc);
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

}  // end namespace chrono

#endif
