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
//     CH_FACTORY_REGISTER(my_class)
//
//  This allows creating an object from a string with its class name.
//  Also, this sets a compiler-independent type name, which you can retrieve
//  by ChClassFactory::GetClassTagName(); in fact different compilers might
//  have different name decorations in type_index.name, which cannot be
//  used for serialization, for example.
//

#include <cstdio>
#include <string>
#include <functional>
#include <typeindex>
#include <unordered_map>
#include <memory>

#include "chrono/core/ChLog.h"
#include "chrono/core/ChTemplateExpressions.h"

namespace chrono {

/// \brief Type-erase pointer-to-object after making sure that it is pointing to the proper address
/// Casting to void* through a static_cast leads to different results:
/// - if the object is *not* polymorphic the result is just the type-erasure;
/// - if the object *is* polymorphic, and the pointer is not of the most derived class,
///   the resulting address (if the derived class has multiple inheritance) might not point to the original object
///   address
/// In this latter case, a dynamic_cast is required. However, the dynamic_cast is not possible for non-polymorphic
/// classes, thus the need for a compile-time switch.
template <typename T, typename std::enable_if<!std::is_polymorphic<T>::value>::type* = nullptr>
void* getVoidPointer(T* ptr) {
    return static_cast<void*>(ptr);
}

template <typename T, typename std::enable_if<std::is_polymorphic<T>::value>::type* = nullptr>
void* getVoidPointer(T* ptr) {
    return dynamic_cast<void*>(ptr);
}

// forward decl.
class ChArchiveIn;
class ChArchiveOut;

/// Base class for all registration data of classes
/// whose objects can be created via a class factory.

class ChApi ChClassRegistrationBase {
  public:
    /// The signature of create method for derived classes. Calls new().
    virtual void* create() = 0;

    /// Call the ArchiveInConstructor(ChArchiveIn&) function if available (deserializes constructor params and return
    /// new()), otherwise just call new().
    virtual void* archive_in_create(ChArchiveIn& marchive) = 0;

    /// Call the ArchiveIn(ChArchiveIn&) function if available, populating an already existing object
    virtual void archive_in(ChArchiveIn& marchive, void* ptr) = 0;

    virtual void archive_out_constructor(ChArchiveOut& marchive, void* ptr) = 0;

    virtual void archive_out(ChArchiveOut& marchive, void* ptr) = 0;

    /// Get the type_index of the class
    virtual std::type_index get_type_index() = 0;

    /// Get the name used for registering
    virtual std::string& get_tag_name() = 0;

    /// Tells if the class is polymorphic
    virtual bool is_polymorphic() = 0;

    /// Tells if the class is default constructible
    virtual bool is_default_constructible() = 0;

    /// Tells if the class is abstract
    virtual bool is_abstract() = 0;

    /// Tells if it implements the function
    virtual bool has_ArchiveInConstructor() = 0;

    /// Tells if it implements the function
    virtual bool has_ArchiveIn() = 0;

    /// Tells if it implements the function
    virtual bool has_ArchiveOutConstructor() = 0;

    /// Tells if it implements the function
    virtual bool has_ArchiveOut() = 0;
};

/// \class ChCastingMap
/// \brief Stores type-casting functions between different type pairs, allowing to pick them at runtime from
/// std::type_index or classname Converting pointers between different classes is usually possible by standard type
/// casting, given that the source and destination class types are known *at compile time*. This class allows to
/// typecast between class types that are known *only at runtime*. The requirement is that the typecasting function has
/// to be prepared in advance (i.e. *at compile time*), when the types are still known. For each potential conversion an
/// instance of ChCastingMap has to be declared, together with its typecasting function. This procedure is simplified by
/// the macros #CH_UPCASTING(FROM, TO) and #CH_UPCASTING_SANITIZED(FROM, TO, UNIQUETAG) When the conversion should take
/// place the following can be called: `ConversionMap::Convert(std::string("source_classname"),
/// std::string("destination_classname"), <void* to object>)`

class ChApi ChCastingMap {
  private:
    struct PairHash {
        template <typename T1, typename T2>
        std::size_t operator()(const std::pair<T1, T2>& p) const {
            return std::hash<T1>()(p.first) ^ std::hash<T2>()(p.second);
        }
    };

    struct EqualHash {
        template <typename T1, typename T2>
        bool operator()(const std::pair<T1, T2>& p, const std::pair<T1, T2>& q) const {
            return (p.first.compare(q.first) == 0) && (p.second.compare(q.second) == 0);
        }
    };

    using conv_fun_pair_type =
        std::pair<std::function<void*(void*)>, std::function<std::shared_ptr<void>(std::shared_ptr<void>)>>;
    using conv_key_pair_type = std::pair<std::string, std::string>;
    using conv_map_type = std::unordered_map<conv_key_pair_type, conv_fun_pair_type, PairHash, EqualHash>;
    using ti_map_type = std::unordered_map<std::type_index, std::string>;

  public:
    ChCastingMap(const std::string& from,
                 const std::type_index& from_ti,
                 const std::string& to,
                 const std::type_index& to_ti,
                 std::function<void*(void*)> conv_ptr_fun,
                 std::function<std::shared_ptr<void>(std::shared_ptr<void>)> conv_shptr_fun);

    static void AddCastingFunction(const std::string& from,
                                   const std::type_index& from_ti,
                                   const std::string& to,
                                   const std::type_index& to_ti,
                                   std::function<void*(void*)> conv_ptr_fun,
                                   std::function<std::shared_ptr<void>(std::shared_ptr<void>)> conv_shptr_fun);

    static void PrintCastingFunctions();

    static void* Convert(const std::string& from, const std::string& to, void* vptr);
    static void* Convert(const std::type_index& from_it, const std::type_index& to_it, void* vptr);
    static void* Convert(const std::string& from, const std::type_index& to_it, void* vptr);
    static void* Convert(const std::type_index& from_it, const std::string& to, void* vptr);

    static std::shared_ptr<void> Convert(const std::string& from, const std::string& to, std::shared_ptr<void> vptr);
    static std::shared_ptr<void> Convert(const std::type_index& from_it,
                                         const std::type_index& to_it,
                                         std::shared_ptr<void> vptr);
    static std::shared_ptr<void> Convert(const std::string& from,
                                         const std::type_index& to_it,
                                         std::shared_ptr<void> vptr);
    static std::shared_ptr<void> Convert(const std::type_index& from_it,
                                         const std::string& to,
                                         std::shared_ptr<void> vptr);

    static std::string GetClassnameFromPtrTypeindex(std::type_index typeindex);

  private:
    static void* _convert(const std::string& from, const std::string& to, void* vptr, bool& success);
    static std::shared_ptr<void> _convert(const std::string& from,
                                          const std::string& to,
                                          std::shared_ptr<void> vptr,
                                          bool& success);

    static conv_map_type& getCastingMap();
    static ti_map_type& getTypeIndexMap();
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
    ChClassFactory() {}
    ~ChClassFactory() {}

    //
    // METHODS
    //

    /// Register a class into the global class factory.
    /// Provide an unique name and a ChClassRegistration object.
    /// If multiple registrations with the same name are attempted, only one is done.
    static void ClassRegister(const std::string& keyName, ChClassRegistrationBase* mregistration) {
        ChClassFactory* global_factory = GetGlobalClassFactory();

        global_factory->_ClassRegister(keyName, mregistration);
    }

    /// Unregister a class from the global class factory.
    /// Provide an unique name.
    static void ClassUnregister(const std::string& keyName) {
        ChClassFactory* global_factory = GetGlobalClassFactory();

        global_factory->_ClassUnregister(keyName);

        if (global_factory->_GetNumberOfRegisteredClasses() == 0)
            DisposeGlobalClassFactory();
    }

    /// Tell if a class is registered, from class name. Name is the mnemonic tag given at registration.
    static bool IsClassRegistered(const std::string& keyName) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        return global_factory->_IsClassRegistered(keyName);
    }

    /// Tell if a class is registered, from std::type_index.
    static bool IsClassRegistered(const std::type_index& mtypeid) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        return global_factory->_IsClassRegistered(mtypeid);
    }

    /// Get class registration info from class name. Name is the mnemonic tag given at registration.
    /// Return nullptr if not registered.
    static ChClassRegistrationBase* GetClass(const std::string& keyName) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        const auto& it = global_factory->class_map.find(keyName);
        if (it != global_factory->class_map.end())
            return it->second;
        else
            return nullptr;
    }

    /// Tell the class name, from type_index.
    /// This is a mnemonic tag name, given at registration, not the typeid.name().
    static std::string& GetClassTagName(const std::type_index& mtypeid) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        return global_factory->_GetClassTagName(mtypeid);
    }

    /// Create from class name, for registered classes. Name is the mnemonic tag given at registration.
    /// The created object is returned in "ptr"
    template <class T>
    static void create(const std::string& keyName, T** ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        *ptr = reinterpret_cast<T*>(
            ChCastingMap::Convert(keyName, std::type_index(typeid(T*)), global_factory->_create(keyName)));
    }

    /// Create from class name, for registered classes. Name is the mnemonic tag given at registration.
    /// If a static T* ArchiveInConstructor(ChArchiveIn&) function is available, call it instead.
    /// The created object is returned in "ptr"
    template <class T>
    static void create(const std::string& keyName, ChArchiveIn& marchive, T** ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        *ptr = reinterpret_cast<T*>(ChCastingMap::Convert(keyName, std::type_index(typeid(T*)),
                                                          global_factory->_archive_in_create(keyName, marchive)));
    }

    /// Populate an already existing object with its ArchiveIn
    template <class T>
    static void archive_in(const std::string& keyName, ChArchiveIn& marchive, T* ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        global_factory->_archive_in(keyName, marchive, getVoidPointer<T>(ptr));
    }

    /// Archive out the most derived object pointed by \a ptr
    template <class T>
    static void archive_out(const std::string& keyName, ChArchiveOut& marchive, T* ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        global_factory->_archive_out(keyName, marchive, getVoidPointer<T>(ptr));
    }

    /// Populate an already existing object with its ArchiveIn
    template <class T>
    static void archive_out_constructor(const std::string& keyName, ChArchiveOut& marchive, T* ptr) {
        ChClassFactory* global_factory = GetGlobalClassFactory();
        global_factory->_archive_out_constructor(keyName, marchive, getVoidPointer<T>(ptr));
    }

  private:
    /// Access the unique class factory here. It is unique even
    /// between dll boundaries. It is allocated the 1st time it is called, if null.
    static ChClassFactory* GetGlobalClassFactory();

    /// Delete the global class factory
    static void DisposeGlobalClassFactory();

    void _ClassRegister(const std::string& keyName, ChClassRegistrationBase* mregistration) {
        class_map[keyName] = mregistration;
        class_map_typeids[mregistration->get_type_index()] = mregistration;
    }

    void _ClassUnregister(const std::string& keyName) {
        // GetLog() << " unregister class: " << keyName << "  map n." << class_map.size() << "  map_typeids n." <<
        // class_map_typeids.size() << "\n";
        class_map_typeids.erase(class_map[keyName]->get_type_index());
        class_map.erase(keyName);
    }

    bool _IsClassRegistered(const std::string& keyName) {
        const auto& it = class_map.find(keyName);
        if (it != class_map.end())
            return true;
        else
            return false;
    }

    bool _IsClassRegistered(const std::type_index& mtypeid) {
        const auto& it = class_map_typeids.find(mtypeid);
        if (it != class_map_typeids.end())
            return true;
        else
            return false;
    }

    std::string& _GetClassTagName(const std::type_index& mtypeid) {
        const auto& it = class_map_typeids.find(mtypeid);
        if (it != class_map_typeids.end()) {
            return it->second->get_tag_name();
        }
        throw(ChException("ChClassFactory::GetClassTagName() cannot find the class with type_index::name: " +
                          std::string(mtypeid.name()) + ". Please register it.\n"));
    }

    size_t _GetNumberOfRegisteredClasses() { return class_map.size(); }

    void* _create(const std::string& keyName) {
        const auto& it = class_map.find(keyName);
        if (it != class_map.end()) {
            return it->second->create();
        }
        throw(ChException("ChClassFactory::create() cannot find the class with name " + keyName +
                          ". Please register it.\n"));
    }

    void* _archive_in_create(const std::string& keyName, ChArchiveIn& marchive) {
        const auto& it = class_map.find(keyName);
        if (it != class_map.end()) {
            return it->second->archive_in_create(marchive);
        }
        throw(ChException("ChClassFactory::create() cannot find the class with name " + keyName +
                          ". Please register it.\n"));
    }

    void _archive_in(const std::string& keyName, ChArchiveIn& marchive, void* ptr) {
        const auto& it = class_map.find(keyName);
        if (it != class_map.end()) {
            it->second->archive_in(marchive, ptr);
        } else
            throw(ChException("ChClassFactory::archive_in() cannot find the class with name " + keyName +
                              ". Please register it.\n"));
    }

    void _archive_out(const std::string& keyName, ChArchiveOut& marchive, void* ptr) {
        const auto& it = class_map.find(keyName);
        if (it != class_map.end()) {
            it->second->archive_out(marchive, ptr);
        } else
            throw(ChException("ChClassFactory::archive_out() cannot find the class with name " + keyName +
                              ". Please register it.\n"));
    }

    void _archive_out_constructor(const std::string& keyName, ChArchiveOut& marchive, void* ptr) {
        const auto& it = class_map.find(keyName);
        if (it != class_map.end()) {
            it->second->archive_out_constructor(marchive, ptr);
        } else
            throw(ChException("ChClassFactory::archive_out() cannot find the class with name " + keyName +
                              ". Please register it.\n"));
    }

  private:
    std::unordered_map<std::string, ChClassRegistrationBase*> class_map;
    std::unordered_map<std::type_index, ChClassRegistrationBase*> class_map_typeids;
};

/// Macro to create a  ChDetect_ArchiveInConstructor
CH_CREATE_MEMBER_DETECTOR(ArchiveInConstructor)

/// Macro to create a  ChDetect_ArchiveOutConstructor that can be used in
/// templates, to select which specialized template to use
CH_CREATE_MEMBER_DETECTOR(ArchiveOutConstructor)

/// Macro to create a  ChDetect_ArchiveOut that can be used in
/// templates, to select which specialized template to use
CH_CREATE_MEMBER_DETECTOR(ArchiveOut)

/// Macro to create a  ChDetect_ArchiveIn that can be used in
/// templates, to select which specialized template to use
CH_CREATE_MEMBER_DETECTOR(ArchiveIn)

/// Macro to create a  ChDetect_ArchiveContainerName that can be used in
/// templates, to select which specialized template to use
CH_CREATE_MEMBER_DETECTOR(ArchiveContainerName)

/// Class for registration data of classes
/// whose objects can be created via a class factory.

template <class t>
class ChClassRegistration : public ChClassRegistrationBase {
  protected:
    //
    // DATA
    //

    /// Name of the class for dynamic creation
    std::string m_sTagName;

  public:
    //
    // CONSTRUCTORS
    //

    /// Creator (adds this to the global list of
    /// ChClassRegistration<t> objects).
    ChClassRegistration(const char* mtag_name) {
        // set name using the 'fake' RTTI system of Chrono
        this->m_sTagName = mtag_name;  // t::FactoryClassNameTag();

        // register in global class factory
        ChClassFactory::ClassRegister(this->m_sTagName, this);
    }

    /// Destructor (removes this from the global list of
    /// ChClassRegistration<t> objects).
    virtual ~ChClassRegistration() {
        // register in global class factory
        ChClassFactory::ClassUnregister(this->m_sTagName);
    }

    //
    // METHODS
    //

    virtual void* create() override { return _create(); }

    virtual void* archive_in_create(ChArchiveIn& marchive) override { return _archive_in_create(marchive); }

    virtual void archive_in(ChArchiveIn& marchive, void* ptr) override { _archive_in(marchive, ptr); }

    virtual void archive_out_constructor(ChArchiveOut& marchive, void* ptr) override {
        _archive_out_constructor(marchive, ptr);
    }

    virtual void archive_out(ChArchiveOut& marchive, void* ptr) override { _archive_out(marchive, ptr); }

    virtual std::type_index get_type_index() override { return std::type_index(typeid(t)); }

    virtual std::string& get_tag_name() override { return _get_tag_name(); }

    virtual bool is_polymorphic() override { return std::is_polymorphic<t>::value; }
    virtual bool is_default_constructible() override { return std::is_default_constructible<t>::value; }
    virtual bool is_abstract() override { return std::is_abstract<t>::value; }
    virtual bool has_ArchiveInConstructor() override { return _has_ArchiveInConstructor(); }
    virtual bool has_ArchiveIn() override { return _has_ArchiveIn(); }
    virtual bool has_ArchiveOutConstructor() override { return _has_ArchiveOutConstructor(); }
    virtual bool has_ArchiveOut() override { return _has_ArchiveOut(); }

  protected:
    template <class Tc = t>
    typename enable_if<std::is_default_constructible<Tc>::value, void*>::type _create() {
        return reinterpret_cast<void*>(new Tc);
    }
    template <class Tc = t>
    typename enable_if<!std::is_default_constructible<Tc>::value, void*>::type _create() {
        throw("ChClassFactory::create() failed for class " + std::string(typeid(Tc).name()) +
              ": it has no default constructor nor archive constructor.\n");
    }

    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveInConstructor<Tc>::value, void*>::type _archive_in_create(
        ChArchiveIn& marchive) {
        return reinterpret_cast<void*>(Tc::ArchiveInConstructor(marchive));
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveInConstructor<Tc>::value, void*>::type _archive_in_create(
        ChArchiveIn& marchive) {
        // rolling back to simple creation
        return _create();
    }

    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive, void* ptr) {
        reinterpret_cast<Tc*>(ptr)->ArchiveIn(marchive);
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveIn<Tc>::value, void>::type _archive_in(ChArchiveIn& marchive, void* ptr) {
        // do nothing, ArchiveIn does not esist for this type
    }

    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveOut<Tc>::value, void>::type _archive_out(ChArchiveOut& marchive, void* ptr) {
        reinterpret_cast<Tc*>(ptr)->ArchiveOut(marchive);
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveOut<Tc>::value, void>::type _archive_out(ChArchiveOut& marchive, void* ptr) {
        // do nothing, ArchiveIn does not esist for this type
    }

    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveOutConstructor<Tc>::value, void>::type _archive_out_constructor(
        ChArchiveOut& marchive,
        void* ptr) {
        reinterpret_cast<Tc*>(ptr)->ArchiveOutConstructor(marchive);
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveOutConstructor<Tc>::value, void>::type _archive_out_constructor(
        ChArchiveOut& marchive,
        void* ptr) {
        // do nothing, ArchiveOutConstructor does not esist for this type
    }

    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveInConstructor<Tc>::value, bool>::type _has_ArchiveInConstructor() {
        return true;
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveInConstructor<Tc>::value, bool>::type _has_ArchiveInConstructor() {
        return false;
    }
    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveIn<Tc>::value, bool>::type _has_ArchiveIn() {
        return true;
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveIn<Tc>::value, bool>::type _has_ArchiveIn() {
        return false;
    }

    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveOutConstructor<Tc>::value, bool>::type _has_ArchiveOutConstructor() {
        return true;
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveOutConstructor<Tc>::value, bool>::type _has_ArchiveOutConstructor() {
        return false;
    }
    template <class Tc = t>
    typename enable_if<ChDetect_ArchiveOut<Tc>::value, bool>::type _has_ArchiveOut() {
        return true;
    }
    template <class Tc = t>
    typename enable_if<!ChDetect_ArchiveOut<Tc>::value, bool>::type _has_ArchiveOut() {
        return false;
    }

    std::string& _get_tag_name() { return m_sTagName; }
};

/// MACRO TO REGISTER A CLASS INTO THE GLOBAL CLASS FACTORY
/// - Put this macro into a .cpp, where you prefer, but not into a .h header!
/// - Use it as
///      CH_FACTORY_REGISTER(my_class)

#define CH_FACTORY_REGISTER(classname)                                                  \
    namespace class_factory {                                                           \
    static ChClassRegistration<classname> classname##_factory_registration(#classname); \
    }

#define CH_FACTORY_REGISTER_CUSTOMNAME(classname, customname)                            \
    namespace class_factory {                                                            \
    static ChClassRegistration<classname> customname##_factory_registration(#classname); \
    }

/// \brief Initialization of pointer up-casting functions
/// A pointer of a parent-class type can safely points to an object of derived class;
/// however, if the derived class has multiple parents it might happen that the pointer of the base class and of the
/// derived class have different values: Derived d; Derived* d_ptr = &d; Base*    b_ptr = &d; Usually d_ptr == b_ptr,
/// but if Derived is defined with multiple inheritance, then d_ptr != b_ptr This is usually not a problem, since the
/// conversion between pointers of different types is usually done automatically; However during serialization the
/// type-erasure impedes this automatic conversion. This can have distruptive consequences: Suppose that (as during
/// serialization):
/// - an object of type Derived is created: `Derived d;`
/// - a pointer to such object is type-erased to void* (e.g. to be stored in a common container): `void* v_ptr =
/// getVoidPointer<Derived>(&d);`
/// - another object might contain a pointer *of upper class type* (e.g. `Base* b_ptr`) that needs to be bound to the
/// object above;
/// - however, assigning the type-erased pointer (e.g. `b_ptr = v_ptr) will not trigger any automatic conversion, thus
/// leading to wrong results; The following macro allows the creation of an auxiliary class that can take care of this
/// conversion manually. This macro should be used in any class with inheritance, in this way:
///     `CH_UPCASTING(DerivedType, BaseType)`
/// or, in case of template parent classes
///     `CH_UPCASTING_SANITIZED(DerivedType, BaseType<6>, DerivedType_BaseType_6)`
/// and repeated for each base class, e.g.
///     `CH_UPCASTING(DerivedType, BaseType1)`
///     `CH_UPCASTING(DerivedType, BaseType2)`
/// Whenever a conversion is needed, it suffices to call `ConversionMap::Convert(std::string("source_classname"),
/// std::string("destination_classname"), <void* to object>)` or
/// `ConversionMap::Convert(std::type_index(typeid(SourceClassType)), std::type_index(typeid(DestinationClassType)),
/// <void* to object>)` e.g. \code{.cpp} CH_UPCASTING(ChBody, ChBodyFrame) CH_UPCASTING(ChBody, ChPhysicsItem)
/// CH_UPCASTING(ChBody, etc....)
/// \endcode
/// then, assuming that:
/// \code{.cpp}
/// ChBody b;
/// void* vptr = getVoidPointer<ChBody>(&b);
/// \endcode
/// then
/// \code{.cpp}
/// void* bf_ptr ConversionMap::Convert("ChBody", "ChBodyFrame", vptr);
/// ChBodyFrame* bframe_ptr = bf_ptr; // CORRECT
/// \endcode
/// in fact, this would have been wrong:
///     `ChBodyFrame* bframe_ptr = vptr; // WRONG`
/// Refer to \ref ConversionMap for further details.
#define CH_UPCASTING(FROM, TO)                                                                                         \
    namespace class_factory {                                                                                          \
    ChCastingMap convfun_from_##FROM##_##TO(                                                                           \
        std::string(#FROM),                                                                                            \
        std::type_index(typeid(FROM*)),                                                                                \
        std::string(#TO),                                                                                              \
        std::type_index(typeid(TO*)),                                                                                  \
        [](void* vptr) { return static_cast<void*>(static_cast<TO*>(reinterpret_cast<FROM*>(vptr))); },                \
        [](std::shared_ptr<void> vptr) {                                                                               \
            return std::static_pointer_cast<void>(std::static_pointer_cast<TO>(std::static_pointer_cast<FROM>(vptr))); \
        });                                                                                                            \
    }

#define CH_UPCASTING_SANITIZED(FROM, TO, UNIQUETAG)                                                                    \
    namespace class_factory {                                                                                          \
    ChCastingMap convfun_from_##UNIQUETAG(                                                                             \
        std::string(#FROM),                                                                                            \
        std::type_index(typeid(FROM*)),                                                                                \
        std::string(#TO),                                                                                              \
        std::type_index(typeid(TO*)),                                                                                  \
        [](void* vptr) { return static_cast<void*>(static_cast<TO*>(reinterpret_cast<FROM*>(vptr))); },                \
        [](std::shared_ptr<void> vptr) {                                                                               \
            return std::static_pointer_cast<void>(std::static_pointer_cast<TO>(std::static_pointer_cast<FROM>(vptr))); \
        });                                                                                                            \
    }

// Class version registration

// Default version=0 for class whose version is not registered.
namespace class_factory {
template <class T>
class ChClassVersion {
  public:
    static const int version = 0;
};
}  // namespace class_factory

}  // end namespace chrono

/// Call this macro to register a custom version for a class "classname".
/// - this macro must be used inside the "chrono" namespace!
/// - you can put this in .h files
/// If you do not do this, the default version for all classes is 0.
/// The m_version parameter should be an integer greater than 0.

#define CH_CLASS_VERSION(classname, m_version) \
    namespace class_factory {                  \
    template <>                                \
    class ChClassVersion<classname> {          \
      public:                                  \
        static const int version = m_version;  \
    };                                         \
    };

#endif
