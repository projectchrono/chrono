//////////////////////////////////////////////////
//
//   ChModuleROS.i
//
//   SWIG configuration file for the Chrono::ROS Python wrapper
//   ('import pychrono.ros').
//
//   Schema-driven API (see src/chrono_ros/CLAUDE.md): the module wraps a small,
//   fixed set of classes - manager, bridge, message, handles, callbacks - and
//   links NO ROS libraries (the ROS side lives in the chrono_ros_node
//   subprocess). New data pathways are authored entirely in Python by
//   subclassing ChROSHandler / ChROSSubscriptionCallback (SWIG directors).
//
///////////////////////////////////////////////////

%module(directors="1") ros

%feature("autodoc", "1");
%feature("flatnested", "1");

// Intercept C++ exceptions (FieldError, runtime_error, ...) as Python exceptions.
%include "exception.i"
%exception {
  try {
    $action
  } catch (const std::exception& e) {
    SWIG_exception(SWIG_RuntimeError, e.what());
  }
}

// Surface the real Python traceback when a director method (a user's
// ChROSHandler.Initialize/Tick or ChROSSubscriptionCallback.OnMessage) raises,
// instead of a generic "SWIG director method error".
%feature("director:except") {
    if ($error != NULL) {
        PyErr_Print();
        throw Swig::DirectorMethodException();
    }
}

%include "../chrono_cast.i"
%include <std_shared_ptr.i>

// ---------------------------------------------------------------------------
// C++ headers for the generated wrapper
// ---------------------------------------------------------------------------
%{
#include <cstdint>

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSSubscription.h"
#include "chrono_ros/ChROSQoS.h"

using namespace chrono;
using namespace chrono::ros;
%}

// Director classes: subclass these in Python to add pathways / receive data.
%feature("director") chrono::ros::ChROSHandler;
%feature("director") chrono::ros::ChROSSubscriptionCallback;

// SWIG can't see Chrono's API export macros.
#define CH_ROS_API
#define ChApi

%include "std_string.i"
%include "std_vector.i"
%include "stdint.i"

%template(vector_string) std::vector<std::string>;

// ---------------------------------------------------------------------------
// A - shared_ptr enablement (every public class handed around as shared_ptr)
// ---------------------------------------------------------------------------
%shared_ptr(chrono::ros::ChROSBridge)
%shared_ptr(chrono::ros::ChROSHandler)
%shared_ptr(chrono::ros::ChROSSubscriptionCallback)
%shared_ptr(chrono::ros::ChROSPublisher)
%shared_ptr(chrono::ros::ChROSSubscription)

// ---------------------------------------------------------------------------
// B - things SWIG must not wrap as-is
// ---------------------------------------------------------------------------
// The std::function overload of CreateSubscription cannot be wrapped; Python
// uses the ChROSSubscriptionCallback director overload instead.
%ignore chrono::ros::ChROSBridge::CreateSubscription(
    const std::string&, const std::string&,
    std::function<void(const ChROSMessageView&)>, const ChROSQoS&);

// Wrapped below by a thin Python shim (section D) that pins the callback's
// Python object to the returned subscription, so a callback constructed inline
// (bridge.CreateSubscription(topic, type, MyCallback())) is not garbage-
// collected while C++ still holds it. Expose the real binding under a private
// name for the shim to call.
%rename(_CreateSubscription) chrono::ros::ChROSBridge::CreateSubscription;

// Bulk blob I/O (camera/lidar) needs a Python buffer-protocol typemap and a
// byte<->element-count conversion; that is the next Phase-4 iteration. For now
// the raw-pointer/BlobView blob methods are hidden so the module builds cleanly
// with only scalar/string/array fields exposed. (Phase 5 sensor handlers will
// drive the numpy zero-copy blob work.)
%ignore chrono::ros::ChROSMessage::SetBlob;
%ignore chrono::ros::ChROSMessage::SetBlobCopy;
%ignore chrono::ros::ChROSMessageView::GetBlob;
%ignore chrono::ros::ChROSMessageView::CopyBlob;

// ---------------------------------------------------------------------------
// B2 - keep Python director objects alive once C++ owns them
// ---------------------------------------------------------------------------
// A handler is typically registered as a temporary:
//     manager.RegisterHandler(MyHandler(...))
// C++ holds it via shared_ptr, but without a Python reference the director's
// Python 'self' is garbage-collected; the C++ director then upcalls into freed
// (and reused) memory. Stash registered handlers on the manager so their Python
// lifetime matches the C++ one. (Subscription callbacks get the same treatment
// in section D, pinned to the subscription they belong to.)
%feature("pythonappend") chrono::ros::ChROSManager::RegisterHandler %{
    if not hasattr(self, "_kept_handlers"):
        self._kept_handlers = []
    self._kept_handlers.append(handler)
%}

// ---------------------------------------------------------------------------
// C - include the public headers (base classes before derived/users)
// ---------------------------------------------------------------------------
%include "../../../chrono_ros/core/ChROSQoSSpec.h"
%include "../../../chrono_ros/ChROSQoS.h"
%include "../../../chrono_ros/ChROSMessage.h"
%include "../../../chrono_ros/ChROSPublisher.h"
%include "../../../chrono_ros/ChROSSubscription.h"
%include "../../../chrono_ros/ChROSHandler.h"
%include "../../../chrono_ros/ChROSBridge.h"
%include "../../../chrono_ros/ChROSManager.h"

// ---------------------------------------------------------------------------
// D - Pythonic sugar
// ---------------------------------------------------------------------------
#ifdef SWIGPYTHON

// Keep a subscription's Python callback object alive for as long as the
// subscription itself. The real binding is exposed as _CreateSubscription
// (renamed in section B); this shim pins the callback onto the returned
// subscription so an inline `bridge.CreateSubscription(t, ty, MyCallback())`
// is safe. *args forwards the optional QoS argument unchanged.
%extend chrono::ros::ChROSBridge {
%pythoncode %{
    def CreateSubscription(self, topic, type_name, callback, *args):
        subscription = self._CreateSubscription(topic, type_name, callback, *args)
        if subscription is not None:
            subscription._kept_callback = callback
        return subscription
%}
};

// msg["header.frame_id"] = "cam";  msg["data"] = 42;  msg["is_dense"] = True
// Dispatch on the Python value type to the right typed setter. bytes -> blob.
%extend chrono::ros::ChROSMessage {
%pythoncode %{
    def __setitem__(self, path, value):
        if isinstance(value, bool):
            self.SetBool(path, value)
        elif isinstance(value, int):
            if value < 0:
                self.SetInt(path, value)
            else:
                self.SetUInt(path, value)
        elif isinstance(value, float):
            self.SetDouble(path, value)
        elif isinstance(value, str):
            self.SetString(path, value)
        elif isinstance(value, (list, tuple)):
            self.SetStringArray(path, [str(v) for v in value])
        else:
            # bytes/numpy blob assignment is the next Phase-4 iteration.
            raise TypeError("unsupported value type for ChROSMessage['%s']: %r" % (path, type(value)))
%}
};

// Read-side: msg["linear.x"] returns a float for numeric fields; explicit
// typed getters (GetString/GetInt/...) remain available for other types.
%extend chrono::ros::ChROSMessageView {
%pythoncode %{
    def __getitem__(self, path):
        return self.GetDouble(path)
%}
};

#endif  // SWIGPYTHON
