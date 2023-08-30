// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from chrono_ros_interfaces:msg/ChDriverInputs.idl
// generated code does not contain a copyright notice

#include "chrono_ros_interfaces/msg/detail/ch_driver_inputs__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_type_hash_t *
chrono_ros_interfaces__msg__ChDriverInputs__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0xce, 0xbe, 0x35, 0x74, 0xef, 0x2d, 0x1c,
      0xea, 0xe2, 0x77, 0x2a, 0x6e, 0x50, 0x2f, 0xbc,
      0xc9, 0x9e, 0x76, 0x9a, 0x6e, 0xa5, 0xf6, 0xbf,
      0x8a, 0xae, 0xc6, 0x28, 0xa7, 0xc5, 0xb5, 0xe2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char chrono_ros_interfaces__msg__ChDriverInputs__TYPE_NAME[] = "chrono_ros_interfaces/msg/ChDriverInputs";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__header[] = "header";
static char chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__steering[] = "steering";
static char chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__throttle[] = "throttle";
static char chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__braking[] = "braking";

static rosidl_runtime_c__type_description__Field chrono_ros_interfaces__msg__ChDriverInputs__FIELDS[] = {
  {
    {chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__steering, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__throttle, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__msg__ChDriverInputs__FIELD_NAME__braking, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription chrono_ros_interfaces__msg__ChDriverInputs__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
chrono_ros_interfaces__msg__ChDriverInputs__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {chrono_ros_interfaces__msg__ChDriverInputs__TYPE_NAME, 40, 40},
      {chrono_ros_interfaces__msg__ChDriverInputs__FIELDS, 4, 4},
    },
    {chrono_ros_interfaces__msg__ChDriverInputs__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Input message that is to be sent to the vehicle\n"
  "#\n"
  "# The input includes steering, throttle, and braking. \n"
  "\n"
  "std_msgs/Header header\n"
  "\n"
  "# Steering value [-1,1]\n"
  "# -1 indicates turn fully to the left, 1 indicates turn fully to the right\n"
  "# Independent of the max turn of the vehicle hardware\n"
  "float64 steering\n"
  "\n"
  "# Throttle value [0, 1]\n"
  "# 0 indicates no throttle, 1 indicates 100% throttle\n"
  "# Independent of the acceleration profile for the car\n"
  "float64 throttle\n"
  "\n"
  "# Braking value [0, 1]\n"
  "# 0 indicates no braking, 1 indicates 100% braking\n"
  "float64 braking\n"
  "";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
chrono_ros_interfaces__msg__ChDriverInputs__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {chrono_ros_interfaces__msg__ChDriverInputs__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 543, 543},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
chrono_ros_interfaces__msg__ChDriverInputs__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *chrono_ros_interfaces__msg__ChDriverInputs__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
