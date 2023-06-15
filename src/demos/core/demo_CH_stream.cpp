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
//
//   Demo code about streams
//
// =============================================================================

#include <cmath>
#include <sstream>

#include "chrono/core/ChGlobal.h"
#include "chrono/core/ChLog.h"
#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChVector.h"
#include "chrono/core/ChClassFactory.h"
#include "chrono/core/ChException.h"

#include "chrono_thirdparty/filesystem/path.h"

using namespace chrono;

// NOTE: the old serialization method, based on ChStream and StreamIn and StreamOut methods 
// has been replaced with the new serialization based on ChArchive and ArchiveIn and ArchiveOut methods,
// so if you are interested on object serialization look rather at 
//   demo_archive.cpp 

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    
    // To write something to the console, use the chrono::GetLog()
    // statement, which returns a global output stream to the console (just
    // like the std::out stream).
    GetLog() << "\nCHRONO foundation classes demo: streaming and serialization\n\n";

    // Create (if needed) output directory
    const std::string out_dir = GetChronoOutputPath() + "DEMO_STREAM";
    if (!filesystem::create_directory(filesystem::path(out_dir))) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }

    /*
     *  TEST SOME BASIC FILE I/O , AS ASCII FILE WRITE/SAVE
     *
     */

    // Chrono stream classes use exceptions for error handling,
    // so you should use the try-catch mechanism.
    // Exceptions thrown are of class ChException.
    try {
        // Open a file of class "ChStreamOutAsciiFile" for writing ascii
        std::string asciifile = out_dir + "/foo_file.txt";
        ChStreamOutAsciiFile mfileo(asciifile.c_str());

        // Write some items, space-separated, inside the ascii file.
        // The class ChStreamOutAsciiFile implements the << operator for most
        // basic types (double, int, string, etc.).
        mfileo << "test_token  " << 123 << " " << 0.123437;

    } catch (const ChException &myex) {
        // Ops.. file could not be opened or written.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }

    // Ok, you wrote something in your pollo_file.txt file,
    // so now try to load from it...
    try {
        // Open a file for reading ascii: the ChStreamInAsciiFile has
        // some minimal parsing capabilities...
        std::string asciifile = out_dir + "/foo_file.txt";
        ChStreamInAsciiFile mfilei(asciifile.c_str());

        // Try to load some text tokens and convert them (at least each token
        // separated by space or linefeed..)
        char sbuff[200];
        int mint;
        double mdouble;
        mfilei >> sbuff >> mint >> mdouble;

        // Write to the console the values which have been read from file..
        GetLog() << "\nResult of ascii  I/O:  " << sbuff << " " << mint << " " << mdouble << "\n";

    } catch (const ChException &myex) {
        // Ops.. file could not be opened or read.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }

    /*
     *  TEST BINARY STREAMING
     */

    //  Streams inherited from the base class ChStreamOutBinary can be
    // used to serialize, and streams inherited from ChStreamInBinary
    // can be used to get them back. For example, file streams like
    // ChStreamOutBinaryFile and ChStreamInBinaryFile can be used for this
    // purpose.
    //  All basic primitives (strings, int,etc.) and objects implementing the <<
    // operator can be streamed into ChStreamOutBinary streams like in the
    // following example.

    try {
        // Open a file of class "ChStreamOutBinaryFile" for serializing
        std::string binfile = out_dir + "/foo_archive.dat";
        ChStreamOutBinaryFile mfileo(binfile.c_str());

        // Write from transient data into persistent binary file
        char m_text[] = "foo_string";
        double m_double = 5.7766;
        int m_int = -348;
        std::string m_string = "hey! stl string";

        mfileo << m_text;    // store data n.1
        mfileo << m_double;  // store data n.2
        mfileo << m_int;     // store data n.3
        mfileo << m_string;  // store data n.4

    } catch (const ChException &myex) {
        GetLog() << "ERROR: " << myex.what();
    }

    // Well, now try to load data back, to see if things worked ok...

    try {
        // Open a file of class "ChStreamOutBinaryFile" for deserializing
        std::string binfile = out_dir + "/foo_archive.dat";
        ChStreamInBinaryFile mfilei(binfile.c_str());

        // Read from persistent binary file to transient data
        char m_text[200];
        int m_int;
        double m_double;
        std::string m_string;

        mfilei >> m_text;    // retrieve data n.1
        mfilei >> m_double;  // retrieve data n.2
        mfilei >> m_int;     // retrieve data n.3
        mfilei >> m_string;  // retrieve data n.4

        GetLog() << "\nResult of binary I/O: " << m_text << " " << m_int << " " << m_double << "\n";
        
    } catch (const ChException &myex) {
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
        double md_out = 12.5;
        mchstreamo << md_out;

        double md_in;
        ChStreamInBinaryStream mchstreami(&mstream);
        mchstreami >> md_in;

        GetLog() << "\nResult of binary I/O from wrapped std::stream: " << md_in << "\n";

    } catch (const ChException &myex) {
        // Ops.. some error.. echo what happened!
        GetLog() << "ERROR: " << myex.what();
    }

    return 0;
}
