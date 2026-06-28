//////////////////////////////////////////////////
//
//   ChModuleROS.i
//
//   SWIG configuration file for the Chrono::ROS Python wrapper
//   ('import pychrono.ros').
//
//   Schema-driven API: the module wraps a small, fixed set of classes - manager,
//   bridge, message, handles, callbacks - and links NO ROS libraries (the ROS
//   side lives in the chrono_ros_node subprocess). New data pathways are authored
//   entirely in Python by subclassing ChROSHandler / ChROSSubscriptionCallback
//   (SWIG directors).
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

// Importing ChBody.i (section C0) transitively pulls the ChContactMaterial /
// ChCollisionModel / ChMarker hierarchies, for which SWIG emits shared_ptr
// up/down-cast helper functions into THIS wrapper. Those helpers need the full
// type definitions to compile, so include the concrete system headers (the same
// way the vehicle module includes ChSystem.h). Compile-time only - the Chrono_ros
// library still links nothing beyond Chrono_core.
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChSystemSMC.h"

#include "chrono_ros/ChROSManager.h"
#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSHandler.h"
#include "chrono_ros/ChROSMessage.h"
#include "chrono_ros/ChROSPublisher.h"
#include "chrono_ros/ChROSSubscription.h"
#include "chrono_ros/ChROSQoS.h"

#include "chrono_ros/handlers/ChROSClockHandler.h"
#include "chrono_ros/handlers/ChROSBodyHandler.h"
#include "chrono_ros/handlers/ChROSTFHandler.h"
#include "chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#ifdef CHRONO_SENSOR
// Sensor handlers + the concrete sensor headers, so the shared_ptr cast helpers
// that importing ChSensor/ChGPSSensor/ChIMUSensor (section C0) emits into this
// wrapper have their type definitions to compile. (The handler headers pull
// ChGPSSensor.h / ChIMUSensor.h; ChSensor.h + Sensor.h cover the shared base.)
#include "chrono_sensor/sensors/ChSensor.h"
#include "chrono_sensor/sensors/Sensor.h"
#include "chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
#include "chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
#include "chrono_ros/handlers/sensor/ChROSGPSHandler.h"
#include "chrono_ros/handlers/sensor/ChROSIMUHandler.h"
#endif

#ifdef CHRONO_HAS_OPTIX
// Camera/lidar handlers + the concrete OptiX sensor headers. Importing
// ChOptixSensor.i (section C0) emits shared_ptr cast helpers for the whole
// OptiX-sensor family it wraps (camera, segmentation, depth, lidar, normal,
// radar, physcam); include their definitions here so those helpers compile.
#include "chrono_sensor/sensors/ChOptixSensor.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChSegmentationCamera.h"
#include "chrono_sensor/sensors/ChDepthCamera.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"
#include "chrono_sensor/sensors/ChNormalCamera.h"
#include "chrono_sensor/sensors/ChRadarSensor.h"
#include "chrono_sensor/sensors/ChPhysCameraSensor.h"
#include "chrono_ros/handlers/sensor/ChROSCameraHandler.h"
#include "chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#endif

#ifdef CHRONO_ROS_HAS_VEHICLE
// DriverInputs handler. Its ctor takes a chrono::vehicle::ChDriver (wrapped in
// pychrono.vehicle). We do NOT %import the vehicle module: SWIG treats ChDriver
// as an opaque shared_ptr argument and the process-global SWIG type registry
// resolves the veh.ChDriver proxy at runtime. The header's #include of
// ChDriver.h is compiled by the C++ compiler against the Chrono_vehicle include
// dirs the ros target links.
#include "chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#endif

#ifdef CHRONO_ROS_HAS_ROBOT
// Viper handler. Its ctor takes a chrono::viper::ViperDCMotorControl
// (pychrono.robot); same opaque-argument / runtime-resolution approach.
#include "chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"
#endif

using namespace chrono;
using namespace chrono::ros;
%}

// Director classes: subclass these in Python to add pathways / receive data.
%feature("director") chrono::ros::ChROSHandler;
%feature("director") chrono::ros::ChROSSubscriptionCallback;

// SWIG can't see Chrono's API export / helper macros. The ChApi/CH_ROS_API
// exports plus the helpers below mirror ChModuleCore.i, so SWIG can parse the
// imported core headers (ChFrame/ChBody/...) that the built-in handlers use.
#define CH_ROS_API
#define ChApi
#define CH_SENSOR_API   // Chrono::Sensor export macro - needed to parse the sensor headers imported (under CHRONO_SENSOR) below
#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define CH_DEPRECATED(msg)
%ignore CH_ENUM_MAPPER_BEGIN;
%ignore CH_ENUM_VAL;
%ignore CH_ENUM_MAPPER_END;
%ignore CH_CLASS_VERSION;

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
// Built-in handlers (concrete ChROSHandler subclasses, registered as shared_ptr).
%shared_ptr(chrono::ros::ChROSClockHandler)
%shared_ptr(chrono::ros::ChROSBodyHandler)
%shared_ptr(chrono::ros::ChROSTFHandler)
%shared_ptr(chrono::ros::ChROSRobotModelHandler)
#ifdef CHRONO_SENSOR
%shared_ptr(chrono::ros::ChROSAccelerometerHandler)
%shared_ptr(chrono::ros::ChROSGyroscopeHandler)
%shared_ptr(chrono::ros::ChROSMagnetometerHandler)
%shared_ptr(chrono::ros::ChROSGPSHandler)
%shared_ptr(chrono::ros::ChROSIMUHandler)
#endif
#ifdef CHRONO_HAS_OPTIX
%shared_ptr(chrono::ros::ChROSCameraHandler)
%shared_ptr(chrono::ros::ChROSLidarHandler)
#endif
#ifdef CHRONO_ROS_HAS_VEHICLE
%shared_ptr(chrono::ros::ChROSDriverInputsHandler)
#endif
#ifdef CHRONO_ROS_HAS_ROBOT
%shared_ptr(chrono::ros::ChROSViperDCMotorControlHandler)
#endif

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

// Bulk blob I/O (camera pixels, lidar points, any primitive array field). The
// raw-pointer/BlobView C++ methods take `const void*`/return a BlobView and
// cannot be wrapped directly; Python reaches them through the buffer-protocol
// %extend methods in section D (SetArray / GetBytes / GetMemoryView), so the
// underlying ones are hidden from the generated wrapper here.
%ignore chrono::ros::ChROSMessage::SetBlob;
%ignore chrono::ros::ChROSMessage::SetBlobCopy;
%ignore chrono::ros::ChROSMessage::SetBlobBytes;
%ignore chrono::ros::ChROSMessageView::GetBlob;
%ignore chrono::ros::ChROSMessageView::CopyBlob;

#ifdef CHRONO_SENSOR
// ChROSTFHandler::AddSensor takes the BASE chrono::sensor::ChSensor by shared_ptr.
// This is the one Chrono-type argument we do NOT wrap: unlike the concrete-type
// arguments elsewhere (ChBody, ChDriver, ChParserURDF, leaf sensors), converting a
// *derived* sensor proxy (e.g. sensor.ChCameraSensor) into shared_ptr<ChSensor>
// needs ChSensor's shared_ptr hierarchy imported - which drags ChSensorBuffer.i's
// bare-template SensorBufferT (breaks under %import). It would compile but not
// reliably convert at runtime. The exact equivalent uses already-wrapped types:
//   tf.AddTransform(sensor.GetParent(), pfid, sensor.GetOffsetPose(), cfid)
%ignore chrono::ros::ChROSTFHandler::AddSensor;
#endif

// RobotModel's ChParserURDF ctor and TF AddURDF take a chrono::parsers::ChParserURDF
// (a plain class wrapped in pychrono.parsers). They wrap like the vehicle/robot
// handlers - the argument crosses opaquely and the global type registry resolves the
// parsers.ChParserURDF proxy at runtime; no parsers %import. SWIG sees these methods
// when -DCHRONO_HAS_URDF is passed (chrono_python/CMakeLists.txt, when the parsers
// module + URDF are present). Used by demo_ROS_urdf.py.

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
// C0 - import the core Chrono types used by the built-in handlers, from the
// already-built pychrono.core module (NOT re-wrapped here). The dependency
// order mirrors the core/vehicle modules; importing these also defines the
// chrono header include guards so the handler headers' #include of ChBody.h /
// ChFrame.h below resolve to the already-known types.
//
// Scope is kept to exactly what the handler API touches (ChBody, ChFrame, and
// their bases/math types). We deliberately do NOT import ChSystem.i / ChMatrix.i
// here: importing a hierarchy makes SWIG emit shared_ptr up/down-cast helpers
// for the WHOLE graph into this wrapper, and the ChSystem graph
// (ChContactMaterialNSC/SMC, ChAssembly, ChTimestepper, ChStaticAnalysis, ...)
// would then need its full C++ definitions #included here to compile. The
// vehicle module imports ChSystem only because it includes those headers; the
// handlers never use ChSystem, so we skip it. Everything needed below is
// defined transitively via the ChBody.h / ChFrame.h include in section above.
// ---------------------------------------------------------------------------
%import(module = "pychrono.core") "chrono_swig/interface/core/ChClassFactory.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector2.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChVector3.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChQuaternion.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChCoordsys.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChFrame.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChFrameMoving.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChPhysicsItem.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBodyFrame.i"
%import(module = "pychrono.core") "chrono_swig/interface/core/ChBody.i"

#ifdef CHRONO_SENSOR
// The handler ctors take leaf sensor types (ChGPSSensor / ChAccelerometer /
// Gyroscope / Magnetometer), wrapped in pychrono.sensor - import just those.
// We deliberately do NOT import ChSensor.i / ChSensorBuffer.i: ChSensorBuffer.i
// does %shared_ptr on the bare template SensorBufferT, which under %import makes
// SWIG emit a malformed cast helper (shared_ptr<SensorBufferT> with no template
// args). The leaf .i files don't pull it in, and our handler API never exposes
// sensor buffers. The shared ChSensor base (concrete, for the DefSharedPtr-
// DynamicCast helpers these files emit) is defined via the ChSensor.h #include
// in the %{ %} block above; SWIG treats it as an opaque imported base, which is
// all we need to pass a leaf-sensor shared_ptr into a handler ctor.
%import(module = "pychrono.sensor") "chrono_swig/interface/sensor/ChGPSSensor.i"
%import(module = "pychrono.sensor") "chrono_swig/interface/sensor/ChIMUSensor.i"
#endif

#ifdef CHRONO_HAS_OPTIX
// Camera/lidar handler ctors take ChCameraSensor / ChLidarSensor (OptiX sensors),
// wrapped together in ChOptixSensor.i. It does not pull ChSensorBuffer.i (so no
// SensorBufferT issue); the cast helpers it emits compile against the OptiX
// sensor headers #included in the %{ %} block above.
%import(module = "pychrono.sensor") "chrono_swig/interface/sensor/ChOptixSensor.i"
#endif

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

// Built-in handlers (derived from ChROSHandler, included after it).
%include "../../../chrono_ros/handlers/ChROSClockHandler.h"
%include "../../../chrono_ros/handlers/ChROSBodyHandler.h"
%include "../../../chrono_ros/handlers/ChROSTFHandler.h"
%include "../../../chrono_ros/handlers/robot/ChROSRobotModelHandler.h"

#ifdef CHRONO_SENSOR
%include "../../../chrono_ros/handlers/sensor/ChROSAccelerometerHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSGyroscopeHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSMagnetometerHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSGPSHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSIMUHandler.h"
#endif
#ifdef CHRONO_HAS_OPTIX
%include "../../../chrono_ros/handlers/sensor/ChROSCameraHandler.h"
%include "../../../chrono_ros/handlers/sensor/ChROSLidarHandler.h"
#endif

// Vehicle / robot handlers: SWIG parses each header with its Chrono-type ctor
// argument (ChDriver / ViperDCMotorControl) left opaque - it emits a "nothing
// known about <type>" warning and generates an opaque shared_ptr arg, resolved
// across modules at runtime (see the %{ %} note above). No vehicle/robot %import.
#ifdef CHRONO_ROS_HAS_VEHICLE
%include "../../../chrono_ros/handlers/vehicle/ChROSDriverInputsHandler.h"
#endif
#ifdef CHRONO_ROS_HAS_ROBOT
%include "../../../chrono_ros/handlers/robot/viper/ChROSViperDCMotorControlHandler.h"
#endif

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
    // Set a primitive array/sequence field (camera pixels, lidar points, any
    // numeric array) from a C-contiguous Python buffer: a numpy array, bytes,
    // bytearray, array.array, or memoryview. The buffer's byte length must be a
    // whole number of the field's elements (validated against the schema). The
    // bytes are copied into the message immediately, so the source buffer may
    // be reused or freed right after this call.
    void SetArray(const std::string& path, PyObject* buffer) {
        Py_buffer view;
        if (PyObject_GetBuffer(buffer, &view, PyBUF_C_CONTIGUOUS) != 0) {
            PyErr_Clear();
            throw std::runtime_error(
                "ChROSMessage['" + path + "'] expects a C-contiguous buffer "
                "(bytes, bytearray, array.array, or a contiguous numpy array; "
                "use numpy.ascontiguousarray() if the array is non-contiguous)");
        }
        try {
            $self->SetBlobBytes(path, view.buf, static_cast<size_t>(view.len));
        } catch (...) {
            PyBuffer_Release(&view);
            throw;
        }
        PyBuffer_Release(&view);
    }
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
            # Anything exposing the buffer protocol (numpy arrays, bytes,
            # bytearray, array.array, memoryview) is a primitive array/blob.
            try:
                memoryview(value)
            except TypeError:
                raise TypeError("unsupported value type for ChROSMessage['%s']: %r" % (path, type(value)))
            self.SetArray(path, value)
%}
};

// Read-side: msg["linear.x"] returns a float for numeric fields; explicit
// typed getters (GetString/GetInt/...) remain available for other types.
%extend chrono::ros::ChROSMessageView {
    // Copy a primitive array/sequence field out as Python `bytes` (safe to keep
    // past the callback). Wrap with numpy.frombuffer(b, dtype=...) to interpret.
    PyObject* GetBytes(const std::string& path) const {
        chrono::ros::core::BlobView v = $self->GetBlob(path);
        return PyBytes_FromStringAndSize(
            reinterpret_cast<const char*>(v.data), static_cast<Py_ssize_t>(v.SizeBytes()));
    }
    // Zero-copy read-only memoryview over a primitive array/sequence field.
    // Valid ONLY while this view (i.e. the callback's message) is alive - do not
    // retain it past the callback; use GetBytes for that. numpy.frombuffer over
    // this view yields an ndarray without copying.
    PyObject* GetMemoryView(const std::string& path) const {
        chrono::ros::core::BlobView v = $self->GetBlob(path);
        if (v.data == nullptr || v.SizeBytes() == 0) {
            return PyMemoryView_FromMemory(const_cast<char*>(""), 0, PyBUF_READ);
        }
        return PyMemoryView_FromMemory(
            const_cast<char*>(reinterpret_cast<const char*>(v.data)),
            static_cast<Py_ssize_t>(v.SizeBytes()), PyBUF_READ);
    }
%pythoncode %{
    def __getitem__(self, path):
        return self.GetDouble(path)
%}
};

#endif  // SWIGPYTHON
