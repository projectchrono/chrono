// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

#ifndef __JOYSTICK_HH__
#define __JOYSTICK_HH__

#include <iostream>
#include <string>

#include "unistd.h"
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#define JS_EVENT_BUTTON 0x01  // button pressed/released
#define JS_EVENT_AXIS 0x02    // joystick moved
#define JS_EVENT_INIT 0x80    // initial state of device

/**
 * Encapsulates all data relevant to a sampled joystick event.
 */
class JoystickEvent {
  public:
    /** Minimum value of axes range */
    static const short MIN_AXES_VALUE = -32768;

    /** Maximum value of axes range */
    static const short MAX_AXES_VALUE = 32767;

    /**
     * The timestamp of the event, in milliseconds.
     */
    unsigned int time;

    /**
     * The value associated with this joystick event.
     * For buttons this will be either 1 (down) or 0 (up).
     * For axes, this will range between MIN_AXES_VALUE and MAX_AXES_VALUE.
     */
    short value;

    /**
     * The event type.
     */
    unsigned char type;

    /**
     * The axis/button number.
     */
    unsigned char number;

    /**
     * Returns true if this event is the result of a button press.
     */
    bool isButton() { return (type & JS_EVENT_BUTTON) != 0; }

    /**
     * Returns true if this event is the result of an axis movement.
     */
    bool isAxis() { return (type & JS_EVENT_AXIS) != 0; }

    /**
     * Returns true if this event is part of the initial state obtained when
     * the joystick is first connected to.
     */
    bool isInitialState() { return (type & JS_EVENT_INIT) != 0; }

    /**
     * The ostream inserter needs to be a friend so it can access the
     * internal data structures.
     */
    friend std::ostream& operator<<(std::ostream& os, const JoystickEvent& e);
};

/**
 * Stream insertion function so you can do this:
 *    cout << event << endl;
 */
std::ostream& operator<<(std::ostream& os, const JoystickEvent& e) {
    os << "type=" << static_cast<int>(e.type) << " number=" << static_cast<int>(e.number)
       << " value=" << static_cast<int>(e.value);

    return os;
}

/**
 * Represents a joystick device. Allows data to be sampled from it.
 */
class Joystick {
  private:
    int _fd;

    void openPath(std::string devicePath, bool blocking = false) {
        // Open the device using either blocking or non-blocking
        _fd = open(devicePath.c_str(), blocking ? O_RDONLY : O_RDONLY | O_NONBLOCK);
    }
  public:
    ~Joystick() { close(_fd); }

    /**
     * Initialises an instance for the first joystick: /dev/input/js0
     */
    Joystick() { openPath("/dev/input/js0"); }

    /**
     * Initialises an instance for the joystick with the specified,
     * zero-indexed number.
     */
    Joystick(int joystickNumber) {
        std::stringstream sstm;
        sstm << "/dev/input/js" << joystickNumber;
        openPath(sstm.str());
    }

    /**
     * Initialises an instance for the joystick device specified.
     */
    Joystick(std::string devicePath) { openPath(devicePath); }

    /**
     * Joystick objects cannot be copied
     */
    Joystick(Joystick const&) = delete;

    /**
     * Joystick objects can be moved
     */
    Joystick(Joystick&&) = default;

    /**
     * Initialises an instance for the joystick device specified and provide
     * the option of blocking I/O.
     */
    Joystick(std::string devicePath, bool blocking) { openPath(devicePath, blocking); }

    /**
     * Returns true if the joystick was found and may be used, otherwise false.
     */
    bool isFound() { return _fd >= 0; }

    /**
     * Attempts to populate the provided JoystickEvent instance with data
     * from the joystick. Returns true if data is available, otherwise false.
     */
    bool sample(JoystickEvent* event) {
        int bytes = read(_fd, event, sizeof(*event));

        if (bytes == -1)
            return false;

        // NOTE if this condition is not met, we're probably out of sync and this
        // Joystick instance is likely unusable
        return bytes == sizeof(*event);
    }
};

#endif
