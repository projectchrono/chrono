Chrono::ROS Internal IPC Protocol and Schema {#ros_protocol}
============================================

This page documents the protocol between the simulation process and the
`chrono_ros_node` subprocess. You do not need it to use Chrono::ROS — handlers
and the message API hide it entirely — but it is useful for debugging the bridge,
porting to a new ROS 2 distribution, or understanding how messages cross the
process boundary.

## Overview

The simulation process contains no ROS symbols; the `chrono_ros_node` subprocess
contains no Chrono physics symbols. They are connected by a bidirectional
shared-memory channel. For a published or received message, the payload that
crosses the channel is the ROS message serialized as **CDR** — the same bytes
ROS 2 places on the wire. The subprocess uses `rclcpp` generic publishers and
subscriptions together with introspection type support, so it never needs
compiled message types; the simulation process serializes and deserializes CDR
itself with a small, dependency-free codec.

## Frames

The channel carries a fixed set of frame kinds. Every frame is a header
(`magic`, protocol `version`, `kind`, `channel_id`, `sim_time_ns`,
`payload_size`) followed by its payload.

| Kind              | Direction  | Payload                                              |
|-------------------|------------|------------------------------------------------------|
| `HELLO`           | node → sim | protocol version, pid, ROS distro, rmw identifier    |
| `DESCRIBE_TYPE`   | sim → node | type name (e.g. `sensor_msgs/msg/Image`)             |
| `TYPE_SCHEMA`     | node → sim | status + schema blob (see below)                     |
| `VALIDATE_TYPE`   | sim → node | type name + CDR of a canonical test message          |
| `VALIDATE_RESULT` | node → sim | ok, or first-difference offset and bytes             |
| `ADVERTISE`       | sim → node | channel id, publish/subscribe, topic, type name, QoS |
| `ADVERTISE_ACK`   | node → sim | channel id, ok or human-readable error               |
| `PUBLISH`         | sim → node | channel id + CDR bytes                                |
| `RECEIVED`        | node → sim | channel id + CDR bytes                                |
| `UNADVERTISE`     | sim → node | channel id                                            |
| `CHANNEL_INFO`    | node → sim | channel id + live subscriber/publisher count         |
| `SHUTDOWN`        | sim → node | —                                                    |

`CHANNEL_INFO` is what lets a handler skip work when nobody is listening
(`ChROSPublisher::GetSubscriptionCount()`); the count is updated as the ROS graph
changes.

## Runtime schema

The simulation process never parses `.msg` files. When a publisher or
subscription for a type is first created, the bridge sends `DESCRIBE_TYPE` with
the type-name string; the subprocess resolves it through the sourced ROS
environment, walks its `rosidl` introspection type support recursively, and
returns a packed **schema blob** (`TYPE_SCHEMA`). The bridge builds its
serializer and deserializer from that blob. This is why any installed message
type — including custom packages — works by name with no code in Chrono.

The blob is a versioned, packed, little-endian structure: a table of types, each
a list of field records, plus the index of the root type. A field record is
`{name, kind, array_kind (none | fixed | bounded | unbounded), array_size,
nested_type_index}`. Kinds are the rosidl primitives (`bool`, `byte`, `char`,
`int8`–`int64`, `uint8`–`uint64`, `float32`, `float64`, `string`, `wstring`) plus
`message` for nested types. Because both ends are built from the same source
tree, the blob never crosses a version boundary and stays a simple packed format.

## CDR encoding

The serialized form is XCDR1 ("plain CDR"): a 4-byte encapsulation header
(`0x00 0x01 0x00 0x00`, little-endian CDR) followed by the body, where:

- primitives are aligned to their own size, with the alignment origin at the
  first byte after the encapsulation header;
- a `string` is a `uint32` length (including the terminating NUL) followed by the
  bytes and the NUL;
- a sequence (bounded or unbounded) is a `uint32` element count followed by the
  aligned elements; a fixed-size array is just the elements;
- a nested message is its fields in declaration order.

All ROS 2 message types are `@final` and are serialized as plain CDR by both
`rmw_fastrtps` and `rmw_cyclonedds`.

## Type validation

The codec is verified per type at run time rather than trusted by assertion. At
`Initialize()`, for each advertised type, the bridge serializes a canonical
test-pattern message and sends `VALIDATE_TYPE`; the subprocess generically
constructs the message through introspection, deserializes the bytes with
`rclcpp` serialization, re-serializes, and byte-compares. A mismatch (for
example, a future rmw switching to XCDR2) aborts `Initialize()` with a precise
error instead of publishing corrupt data.

Note for anyone modifying the validator: rmw serializers do not zero-guarantee
interior padding bytes, so the node zero-fills its re-serialization buffer to
keep skipped-padding differences from being reported as field mismatches.

## Quality of Service

`ChROSQoS` exposes a minimal explicit surface — reliability (`reliable`,
`best_effort`), durability (`volatile`, `transient_local`), and history depth —
plus named presets: `Default()`, `SensorData()`, `Clock()`, and `Latched()`
(transient-local, required for latched topics such as `/robot_description`). The
QoS travels in the `ADVERTISE` frame and the subprocess applies it to the
generic publisher or subscription.
