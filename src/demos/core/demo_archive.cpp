//
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2010 Alessandro Tasora
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file at the top level of the distribution
// and at http://projectchrono.org/license-chrono.txt.
//

//////////////////////////////////////////////////
//
//   Demos code about
//
//     - archives for serialization,
//     - serialization, with versioning and dynamic
//       creation (class factory)
//     - class runtime type identification, without
//       enabling the RTTI compiler feature
//
///////////////////////////////////////////////////

#include <math.h>

#include "core/ChLog.h"
#include "core/ChArchive.h"
#include "core/ChVector.h"
#include "core/ChMatrixDynamic.h"
#include "core/ChMatrix.h"
#include "core/ChException.h"

using namespace chrono;

//
// Define some example classes just for showing how CHRONO serialization
// system works, even with inheritance and versioning...
//
// The statements marked with //##### are mandatory if you want to
// take advantage of basic Chrono streaming and serialization mechanisms,
// that is if you want to save/load the object to&from a stream.
//
// The statements marked with //***** are needed only if you want to
// take advantage of Chrono advanced serialization mechanism, that is the
// AbstractWrite() and AbstractReadCreate() methods which can load an
// object from stream even if the object class is not known in advance.
// This more advanced feature requires a 'class factory' registration
// of your object type by means of the ChClassRegister<> statement (in
// its turn, it requires that your object has also the Chrono simulation
// of run-time-type information enabled, that is the CH_RTTI_.. macros).
//

class myEmployee {
    // Remember to enable the Chrono RTTI features with the CH_RTTI_.. macro
    // if you want to use the AbstractReadCreate() feature, to deserialize
    // objects whose exact class is not known in advance!

    CH_RTTI_ROOT(myEmployee)  //***** for _advanced_ Chrono serialization

  public:
    int age;
    double wages;

    myEmployee(int m_age = 18, double m_wages = 1020.3) : age(m_age), wages(m_wages){};

    // MEMBER FUNCTIONS FOR BINARY I/O
    // NOTE!!!In order to allow serialization with Chrono approach,
    // at least implement these two functions, with the exact names
    // StreamIN() and StreamOUT():

    virtual void ArchiveOUT(ChArchiveOut& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        marchive.VersionWrite(1);
        // stream out all member data
        marchive << CHNVP(age);
        marchive << CHNVP(wages);
    }
    virtual void ArchiveIN(ChArchiveIn& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        int version = marchive.VersionRead();
        // stream in all member data
        marchive >> CHNVP(age);
        marchive >> CHNVP(wages);
    }

    // Optional: stream of data into readable ASCII format, if you later want
    // (ex) to output to console using chrono::GetLog() << my_object;
    virtual void StreamOUT(ChStreamOutAscii& mstream) {
        mstream << "Age is:  " << age << "\n";
        mstream << "Wage is: " << wages << "\n";
    }
};

// Somewhere in your cpp code (not in .h headers!) you should put the
// 'class factory' registration of your class, assuming it has the CH_RTTI_..,
// if you want to use the AbstractReadCreate() feature, to deserialize
// objects whose exact class is not known in advance.

chrono::ChClassRegister<myEmployee> a_registration1;  //***** for _advanced_ Chrono serialization

// ............ ok, more difficult! an inherited class ............

class myEmployeeBoss : public myEmployee {
    CH_RTTI(myEmployeeBoss, myEmployee)  //***** for _advanced_ Chrono serialization

  public:
    bool is_dumb;
    myEmployee slave;

    myEmployeeBoss(int m_age = 38, double m_wages = 9000.4, bool m_is_dumb = true)
        : myEmployee(m_age, m_wages), is_dumb(m_is_dumb), slave(21, 300){};

    // MEMBER FUNCTIONS FOR BINARY I/O

    virtual void ArchiveOUT(ChArchiveOut& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        marchive.VersionWrite(2);
        // remember to serialize the parent class data too!!!
        myEmployee::ArchiveOUT(marchive);

        // stream out member data
        marchive << CHNVP(is_dumb);
        marchive << CHNVP(slave);  // this added only from version >1
    }
    virtual void ArchiveIN(ChArchiveIn& marchive)  //##### for Chrono serialization
    {
        // suggested: use versioning
        int version = marchive.VersionRead();
        // remember to deserialize the parent class data too!!!
        myEmployee::ArchiveIN(marchive);

        // stream in member data
        marchive >> CHNVP(is_dumb);
        if (version > 1)
            marchive >> CHNVP(slave);  // this added only from version >1
    }

    // Optional: stream of data into readable ASCII format:
    virtual void StreamOUT(ChStreamOutAscii& mstream) {
        myEmployee::StreamOUT(mstream);

        mstream << "is dumb? =" << is_dumb << "\n";
        mstream << "..the boss has a slave employee: \n" << slave << "\n";
    }
};

chrono::ChClassRegister<myEmployeeBoss> a_registration2;  //***** for _advanced_ Chrono serialization

int main(int argc, char* argv[]) {
    // To write something to the console, use the chrono::GetLog()
    // statement, which returns a global output stream to the console (just
    // like the std::out stream).
    GetLog() << "\n"
             << "CHRONO foundation classes demo: archives (serialization)\n\n";

    /*
     *  TEST SOME BASIC FILE I/O , AS ASCII FILE WRITE/SAVE
     *
     */

   
    //  Archives inherited from the base class ChArchiveOut can be
    // used to serialize objects, and streams inherited from ChArchiveIn
    // can be used to get them back. For example, file streams like
    // ChArchiveOutBinary and ChArchiveInBinary can be used for this
    // purpose.
    //  All basic primitives (strings, int,etc.) and objects that has
    // a StreamOUT() function defined can beserialized in archives.
    //  Viceversa, you should implement also the StreamOUT() method
    // to allow using the >> operator to get the object back from a
    // ChStreamInBinary archive.

    try {       
        // Open a file of class "ChStreamOutBinaryFile" as a container
        ChStreamOutBinaryFile mfileo("foo_archive.dat");

        // Create a binary archive, that uses the binary file as storage.
        ChArchiveOutBinary marchive(mfileo);
       
        /*
        // Open a file of class "ChStreamOutBinaryFile" as a container
        ChStreamOutAsciiFile mfileo("foo_archive.txt");

        // Create a binary archive, that uses the binary file as storage.
        ChArchiveOutAsciiLogging marchive(mfileo);
        */

        // Write from transient data into persistent binary file
        const char* m_text = "test string";
        double m_double = 0.123456;
        int m_int = -123;
        std::string m_string = "hey! stl string";
        std::vector< double > m_stlvector = {2.3,45.3,66.44};
        ChMatrixDynamic<double> m_matr(3, 5);
        m_matr.FillRandom(10, 0);
        ChVector<> m_vect(0.5, 0.6, 0.7);
        ChQuaternion<> m_quat(0.1, 0.2, 0.3, 0.4);
        myEmployeeBoss m_boss(53, 12000.34, true);

        marchive << CHNVP(m_text);    // store data n.1
        marchive << CHNVP(m_double);  // store data n.2
        marchive << CHNVP(m_int);     // store data n.3
        marchive << CHNVP(m_string);  // store data n....
        marchive << CHNVP(m_stlvector);
        marchive << CHNVP(m_matr);    
        marchive << CHNVP(m_vect);    
        marchive << CHNVP(m_quat);    
        marchive << CHNVP(m_boss);    // store our c++ object!

        // Also store c++ objects referenced by pointer, using the
        // class abstraction (class factory) mechanism, so that it
        // can be loaded later even if we do not know if it was an object of
        // class 'myEmployee' or specialized class 'myEmployeeBoss'...
        myEmployeeBoss* a_boss = new myEmployeeBoss(64, 22356, false);
        a_boss->slave.age = 24;
        marchive << CHNVP(a_boss);  //  object was referenced by pointer.
      //  marchive << CHNVP(a_boss);  //  object was referenced by pointer.
      //  marchive << CHNVP(a_boss);  //  object was referenced by pointer.
        delete a_boss;


    } catch (ChException myex) {
        GetLog() << "ERROR: " << myex.what();
    }

  
    // Well, now try to load data back, to see if things worked ok...

    try {
        // Open a file of class "ChStreamInBinaryFile" as a container
        ChStreamInBinaryFile mfilei("foo_archive.dat");

        // Create a binary archive, that uses the binary file as storage.
        ChArchiveInBinary marchive(mfilei);
        

        // Read from persistent binary file to transient data
        char mbuffer[200];
        char* m_text = mbuffer;
        double m_double;
        int m_int;
        std::string m_string;
        std::vector< double > m_stlvector;
        ChMatrixDynamic<> m_matr;
        ChVector<> m_vect;
        ChQuaternion<> m_quat;
        myEmployeeBoss m_boss;
        marchive >> CHNVP(m_text);    // retrieve data n.1
        marchive >> CHNVP(m_double);  // retrieve data n.2
        marchive >> CHNVP(m_int);     // retrieve data n.3
        marchive >> CHNVP(m_string);  // retrieve data n....
        marchive >> CHNVP(m_stlvector);
        marchive >> CHNVP(m_matr);
        marchive >> CHNVP(m_vect);  
        marchive >> CHNVP(m_quat);   
        marchive >> CHNVP(m_boss); 

        // Also retrieve c++ objects, referenced by a base class pointer, using the
        // class abstraction (class factory) mechanism, so that it
        // can be loaded even if we do not know if it was an object of
        // the base class 'myEmployee' or the specialized 'myEmployeeBoss' class..
        myEmployee* a_boss = 0;
        myEmployee* a_boss1 = 0;
        myEmployee* a_boss2 = 0;
        marchive >> CHNVP(a_boss);
       // marchive >> CHNVP(a_boss1);
       // marchive >> CHNVP(a_boss2);

        GetLog() << "\nResult of binary I/O: \n " << m_text << " \n " << m_int << " \n" << m_double << "\n";
        GetLog() << m_matr;
        GetLog() << m_vect;
        GetLog() << m_quat;
        GetLog() << m_string.c_str();
        GetLog() << "\n stl::vector of size " <<  m_stlvector.size() << ":\n";
        for(int i=0; i< m_stlvector.size(); ++i)
            GetLog() << m_stlvector[i] << " ";
        GetLog() << "\n\n We also loaded a myEmployeeBoss object:\n";
        GetLog() << m_boss;

        if (a_boss1) {
            GetLog() << "\n\n We used AbstractReadCreate() to load an 2nd obj inherited from myEmployee class:\n";
            GetLog() << *a_boss1;
        }

        if (a_boss) {
            GetLog() << "\n\n We used AbstractReadCreate() to load an obj inherited from myEmployee class:\n";
            GetLog() << *a_boss;

            // By the way, now show some feaures of Chrono run-time-type-identifier
            // methods, since class had CH_RTTI enabled.
            // Note that Chrono RTTI is _not_ the standard C++ RTTI! Also, it
            // does not need RTTI to be enabled in compilation. It is platform-
            // and compiler-independent.
            // The only drawback of Chrono RTTI is that it is a bit slower than
            // standard C++ RTTI, and it introduces a virtual method in the class (not a
            // big problem however, since also C++ RTTI eats some memory...)

            GetLog() << "\n";
            GetLog() << "loaded object is a myEmployee?     :" << ChIsExactlyClass(myEmployee, a_boss) << "\n";
            GetLog() << "loaded object is a myEmployeeBoss? :" << ChIsExactlyClass(myEmployeeBoss, a_boss) << "\n";
            GetLog() << "loaded object derives myEmployee?  :" << ChIsDerivedFromClass(myEmployee, a_boss) << "\n";
            GetLog() << "Obvious! we loaded an object of class: " << a_boss->GetRTTI()->GetName() << "\n";
            delete a_boss;
        }

    } catch (ChException myex) {
        // Ops.. file could not be opened or read.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }


   

    return 0;
}
