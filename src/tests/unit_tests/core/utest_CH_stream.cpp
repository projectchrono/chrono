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
// Authors: Alessandro Tasora
// =============================================================================
//   Demos code about
//     - streams,
//     - serialization, with versioning and dynamic
//       creation (class factory)
// =============================================================================

#include <cmath>

#include "core/ChLog.h"
#include "core/ChMatrix.h"
#include "core/ChVector.h"
#include "core/ChClassFactory.h"
#include "core/ChException.h"
#include <sstream>

using namespace chrono;

//
// Define some example classes just for showing how CHRONO serialization
// system works, even with inheritance and versioning.

class myEmployee {


  public:
    int age;
    double wages;

    myEmployee(int m_age = 18, double m_wages = 1020.3) : age(m_age), wages(m_wages){};

    // MEMBER FUNCTIONS FOR BINARY I/O
    // NOTE!!!In order to allow serialization with Chrono approach,
    // at least implement these two functions, with the exact names
    // StreamIn() and StreamOut():

    virtual void StreamOut(ChStreamOutBinary& mstream)  //##### for Chrono serialization
    {
        // suggested: use versioning
        mstream.VersionWrite(1);
        // stream out all member data
        mstream << age;
        mstream << wages;
    }
    virtual void StreamIn(ChStreamInBinary& mstream)  //##### for Chrono serialization
    {
        // suggested: use versioning
        int version = mstream.VersionRead();
        // stream in all member data
        mstream >> age;
        mstream >> wages;
    }

    // Optional: stream of data into readable ASCII format, if you later want
    // (ex) to output to console using chrono::GetLog() << my_object;
    virtual void StreamOut(ChStreamOutAscii& mstream) {
        mstream << "Age is:  " << age << "\n";
        mstream << "Wage is: " << wages << "\n";
    }
};

CH_FACTORY_REGISTER(myEmployee)  //***** for _advanced_ Chrono serialization

// ............ ok, more difficult! an inherited class ............

class myEmployeeBoss : public myEmployee {

  public:
    bool is_dumb;
    myEmployee slave;

    myEmployeeBoss(int m_age = 38, double m_wages = 9000.4, bool m_is_dumb = true)
        : myEmployee(m_age, m_wages), is_dumb(m_is_dumb), slave(21, 300){};

    // MEMBER FUNCTIONS FOR BINARY I/O

    virtual void StreamOut(ChStreamOutBinary& mstream)  //##### for Chrono serialization
    {
        // suggested: use versioning
        mstream.VersionWrite(2);
        // remember to serialize the parent class data too!!!
        myEmployee::StreamOut(mstream);

        // stream out member data
        mstream << is_dumb;
        mstream << slave;  // this added only from version >1
    }
    virtual void StreamIn(ChStreamInBinary& mstream)  //##### for Chrono serialization
    {
        // suggested: use versioning
        int version = mstream.VersionRead();
        // remember to deserialize the parent class data too!!!
        myEmployee::StreamIn(mstream);

        // stream in member data
        mstream >> is_dumb;
        if (version > 1)
            mstream >> slave;  // this added only from version >1
    }

    // Optional: stream of data into readable ASCII format:
    virtual void StreamOut(ChStreamOutAscii& mstream) {
        myEmployee::StreamOut(mstream);

        mstream << "is dumb? =" << is_dumb << "\n";
        mstream << "..the boss has a slave employee: \n" << slave << "\n";
    }
};

CH_FACTORY_REGISTER(myEmployeeBoss) //***** for _advanced_ Chrono serialization

int main(int argc, char* argv[]) {
    // To write something to the console, use the chrono::GetLog()
    // statement, which returns a global output stream to the console (just
    // like the std::out stream).
    GetLog() << "\n"
             << "CHRONO foundation classes test: streaming and serialization\n\n";

    /*
     *  TEST SOME BASIC FILE I/O , AS ASCII FILE WRITE/SAVE
     *
     */

    // Chrono stream classes use exceptions for error handling,
    // so you should use the try-catch mechanism.
    // Exceptions thrown are of class ChException.
    try {
        // Open a file of class "ChStreamOutAsciiFile" for writing ascii
        ChStreamOutAsciiFile* mfileo = new ChStreamOutAsciiFile("foo_file.txt");

        // Write some items, space-separated, inside the ascii file.
        // The class ChStreamOutAsciiFile implements the << operator for most
        // basic types (double, int, string, etc.).
        *mfileo << "test_token"
                << " " << 123 << " " << 0.123437;
        delete mfileo;
    } catch (ChException myex) {
        // Ops.. file could not be opened or written.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }

    // Ok, you wrote something in your pollo_file.txt file,
    // so now try to load from it...
    try {
        // Open a file for reading ascii: the ChStreamInAsciiFile has
        // some minimal parsing capabilities...
        ChStreamInAsciiFile* mfilei = new ChStreamInAsciiFile("foo_file.txt");

        // Try to load some text tokens and convert them (at least each token
        // separated by space or linefeed..)
        char sbuff[200];
        int mint;
        double mdouble;
        *mfilei >> sbuff >> mint >> mdouble;

        // Write to the console the values which have been read from file..
        GetLog() << "\nResult of ascii  I/O:  " << sbuff << " " << mint << " " << mdouble << "\n";
        delete mfilei;
    } catch (ChException myex) {
        // Ops.. file could not be opened or read.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }

    /*
     *  TEST BINARY STREAMING
     */

    //  Streams inherited from the base class ChStreamOutBinary can be
    // used to serialize objects, and streams inherited from ChStreamInBinary
    // can be used to get them back. For example, file streams like
    // ChStreamOutBinaryFile and ChStreamInBinaryFile can be used for this
    // purpose.
    //  All basic primitives (strings, int,etc.) and objects implementing the <<
    // operator can be streamed into ChStreamOutBinary streams like in the
    // following example.
    //  In order to use << operator also on objects from your classes,
    // just remember to add the method StreamOut() in your class!
    //  Viceversa, you should implement also the StreamIn() method
    // to allow using the >> operator to get the object back from a
    // ChStreamInBinary archive.
    try {
        // Open a file of class "ChStreamOutBinaryFile" for serializing
        ChStreamOutBinaryFile mfileo("foo_archive.dat");

        // Write from transient data into persistent binary file
        char m_text[] = "test string";
        double m_double = 0.123456;
        int m_int = -123;
        std::string m_string = "hey! stl string";
        ChMatrixDynamic<> m_matr(3, 5);
        m_matr.FillRandom(10, 0);
        Vector m_vect(0.5, 0.6, 0.7);
        Quaternion m_quat(0.1, 0.2, 0.3, 0.4);
        myEmployeeBoss m_boss(53, 12000.34, true);

        mfileo << m_text;    // store data n.1
        mfileo << m_double;  // store data n.2
        mfileo << m_int;     // store data n.3
        mfileo << m_string;  // store data n.4
        mfileo << m_matr;    // store data n.5 (chrono math object can be streamed!)
        mfileo << m_vect;    // store data n.6 (chrono math object can be streamed!)
        mfileo << m_quat;    // store data n.7 (chrono math object can be streamed!)
        mfileo << m_boss;    // store data n.8 (stream our inherited class!)

        // store data n.9
        // Also store object n.9, referenced by pointer, using the
        // AbstractWrite() - AbstractReadCreate() mechanism, so that it
        // can be loaded later even if we do not know if it was an object of
        // class 'myEmployee' or specialized class 'myEmployeeBoss'...
        myEmployeeBoss* a_boss = new myEmployeeBoss(64, 22356, false);
        a_boss->slave.age = 24;
        mfileo.AbstractWrite(a_boss);  //  object was referenced by pointer.
        delete a_boss;
    } catch (ChException myex) {
        GetLog() << "ERROR: " << myex.what();
    }

    // Well, now try to load data back, to see if things worked ok...

    try {
        // Open a file of class "ChStreamOutBinaryFile" for deserializing
        ChStreamInBinaryFile mfilei("foo_archive.dat");

        // Read from persistent binary file to transient data
        char m_text[200];
        int m_int;
        double m_double;
        std::string m_string;
        ChMatrixDynamic<> m_matr;
        Vector m_vect;
        Quaternion m_quat;
        myEmployeeBoss m_boss;
        mfilei >> m_text;    // retrieve data n.1
        mfilei >> m_double;  // retrieve data n.2
        mfilei >> m_int;     // retrieve data n.3
        mfilei >> m_string;  // retrieve data n.4
        mfilei >> m_matr;    // retrieve data n.5
        mfilei >> m_vect;    // retrieve data n.6
        mfilei >> m_quat;    // retrieve data n.7
        mfilei >> m_boss;    // retrieve data n.8

        // retrieve data n.9
        // Also retrieve object, referenced by a base class pointer, using the
        // AbstractWrite() - AbstractReadCreate() mechanism, so that it
        // can be loaded even if we do not know if it was an object of
        // the base class 'myEmployee' or the specialized 'myEmployeeBoss' class..
        myEmployee* a_boss = NULL;
        mfilei.AbstractReadCreate(&a_boss);

        GetLog() << "\nResult of binary I/O: " << m_text << " " << m_int << " " << m_double << "\n";
        GetLog() << m_matr;
        GetLog() << m_vect;
        GetLog() << m_quat;
        GetLog() << m_string.c_str();
        GetLog() << "\n\n We also loaded a myEmployeeBoss object:\n";
        GetLog() << m_boss;

        if (a_boss) {
            GetLog() << "\n\n We used AbstractReadCreate() to load an obj inherited from myEmployee class:\n";
            GetLog() << *a_boss;

            delete a_boss;
        }
    } catch (ChException myex) {
        // Ops.. file could not be opened or read.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }

    /*
     *  TEST std:: STREAM WRAPPING
     */

    // In the previous examples we showed how to use Chrono::Engine
    // file streams such as  ChStreamInBinaryFile and ChStreamOutBinaryFile,
    // but in the following we show that we can also wrap whatever object
    // of type std::istream or std::ostream (already opened files, string streams,
    // console logs, etc), thank to ChStreamOutBinaryStream and
    // ChStreamInBinaryStream. All the concepts learned until now, such as
    // serialization through << and >> , are still possible.
    try {
        std::stringstream mstream;

        ChStreamOutBinaryStream mchstreamo(&mstream);
        ChVector<> mv(12, 23, 45.34);
        mchstreamo << mv;
        // GetLog() << "Vector a: " << mv;

        ChVector<> mz;
        ChStreamInBinaryStream mchstreami(&mstream);
        mchstreami >> mz;
        // GetLog() << "Vector b: " << mz;
    } catch (ChException myex) {
        // Ops.. some error.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }

    return 0;
}
