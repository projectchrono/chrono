// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from chrono_ros_interfaces:srv/ChStartSimulation.idl
// generated code does not contain a copyright notice

#include "chrono_ros_interfaces/srv/detail/ch_start_simulation__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_type_hash_t *
chrono_ros_interfaces__srv__ChStartSimulation__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xa4, 0x66, 0x73, 0xb6, 0x58, 0xb0, 0x56, 0x9b,
      0xd0, 0x6c, 0x08, 0x4e, 0xae, 0x29, 0x9e, 0x92,
      0x18, 0xdc, 0xd2, 0x40, 0xb9, 0xd6, 0xbd, 0xeb,
      0x52, 0x46, 0x45, 0x17, 0x9f, 0x3d, 0x63, 0xfa,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_type_hash_t *
chrono_ros_interfaces__srv__ChStartSimulation_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x67, 0xdc, 0xf7, 0xfe, 0xfe, 0xe4, 0x7e, 0x1b,
      0x33, 0x49, 0xef, 0x1b, 0xf1, 0xd4, 0x31, 0xb1,
      0x79, 0x26, 0x10, 0x49, 0x8a, 0x78, 0xfe, 0xfc,
      0x4d, 0x3f, 0xc6, 0x9d, 0xfd, 0x34, 0x04, 0xb0,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_type_hash_t *
chrono_ros_interfaces__srv__ChStartSimulation_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x6c, 0xb4, 0xb6, 0x3e, 0x37, 0xcd, 0x98, 0x4a,
      0xdc, 0x86, 0x17, 0x74, 0x13, 0x3b, 0x14, 0x78,
      0xa0, 0xfb, 0xee, 0xb2, 0xee, 0xd1, 0xde, 0x38,
      0xd9, 0x05, 0x89, 0xdf, 0x24, 0x39, 0x9f, 0x8b,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_chrono_ros_interfaces
const rosidl_type_hash_t *
chrono_ros_interfaces__srv__ChStartSimulation_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x96, 0xc8, 0x53, 0xaf, 0x50, 0x91, 0x47, 0x2a,
      0xd5, 0x3e, 0xe5, 0xc2, 0x0d, 0x23, 0x07, 0xb1,
      0xe4, 0xf8, 0x7e, 0x84, 0x5b, 0x78, 0x7a, 0x4a,
      0xa5, 0x07, 0x3b, 0xcf, 0xf7, 0xfe, 0xc9, 0x04,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char chrono_ros_interfaces__srv__ChStartSimulation__TYPE_NAME[] = "chrono_ros_interfaces/srv/ChStartSimulation";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char chrono_ros_interfaces__srv__ChStartSimulation_Event__TYPE_NAME[] = "chrono_ros_interfaces/srv/ChStartSimulation_Event";
static char chrono_ros_interfaces__srv__ChStartSimulation_Request__TYPE_NAME[] = "chrono_ros_interfaces/srv/ChStartSimulation_Request";
static char chrono_ros_interfaces__srv__ChStartSimulation_Response__TYPE_NAME[] = "chrono_ros_interfaces/srv/ChStartSimulation_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char chrono_ros_interfaces__srv__ChStartSimulation__FIELD_NAME__request_message[] = "request_message";
static char chrono_ros_interfaces__srv__ChStartSimulation__FIELD_NAME__response_message[] = "response_message";
static char chrono_ros_interfaces__srv__ChStartSimulation__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field chrono_ros_interfaces__srv__ChStartSimulation__FIELDS[] = {
  {
    {chrono_ros_interfaces__srv__ChStartSimulation__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {chrono_ros_interfaces__srv__ChStartSimulation_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {chrono_ros_interfaces__srv__ChStartSimulation_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {chrono_ros_interfaces__srv__ChStartSimulation_Event__TYPE_NAME, 49, 49},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription chrono_ros_interfaces__srv__ChStartSimulation__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Event__TYPE_NAME, 49, 49},
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Response__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
chrono_ros_interfaces__srv__ChStartSimulation__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {chrono_ros_interfaces__srv__ChStartSimulation__TYPE_NAME, 43, 43},
      {chrono_ros_interfaces__srv__ChStartSimulation__FIELDS, 3, 3},
    },
    {chrono_ros_interfaces__srv__ChStartSimulation__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = chrono_ros_interfaces__srv__ChStartSimulation_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = chrono_ros_interfaces__srv__ChStartSimulation_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = chrono_ros_interfaces__srv__ChStartSimulation_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char chrono_ros_interfaces__srv__ChStartSimulation_Request__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field chrono_ros_interfaces__srv__ChStartSimulation_Request__FIELDS[] = {
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Request__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
chrono_ros_interfaces__srv__ChStartSimulation_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {chrono_ros_interfaces__srv__ChStartSimulation_Request__TYPE_NAME, 51, 51},
      {chrono_ros_interfaces__srv__ChStartSimulation_Request__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char chrono_ros_interfaces__srv__ChStartSimulation_Response__FIELD_NAME__success[] = "success";

static rosidl_runtime_c__type_description__Field chrono_ros_interfaces__srv__ChStartSimulation_Response__FIELDS[] = {
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
chrono_ros_interfaces__srv__ChStartSimulation_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {chrono_ros_interfaces__srv__ChStartSimulation_Response__TYPE_NAME, 52, 52},
      {chrono_ros_interfaces__srv__ChStartSimulation_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELD_NAME__info[] = "info";
static char chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELD_NAME__request[] = "request";
static char chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELDS[] = {
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {chrono_ros_interfaces__srv__ChStartSimulation_Request__TYPE_NAME, 51, 51},
    },
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {chrono_ros_interfaces__srv__ChStartSimulation_Response__TYPE_NAME, 52, 52},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription chrono_ros_interfaces__srv__ChStartSimulation_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Request__TYPE_NAME, 51, 51},
    {NULL, 0, 0},
  },
  {
    {chrono_ros_interfaces__srv__ChStartSimulation_Response__TYPE_NAME, 52, 52},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
chrono_ros_interfaces__srv__ChStartSimulation_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {chrono_ros_interfaces__srv__ChStartSimulation_Event__TYPE_NAME, 49, 49},
      {chrono_ros_interfaces__srv__ChStartSimulation_Event__FIELDS, 3, 3},
    },
    {chrono_ros_interfaces__srv__ChStartSimulation_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = chrono_ros_interfaces__srv__ChStartSimulation_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = chrono_ros_interfaces__srv__ChStartSimulation_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "---\n"
  "bool success";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
chrono_ros_interfaces__srv__ChStartSimulation__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {chrono_ros_interfaces__srv__ChStartSimulation__TYPE_NAME, 43, 43},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 16, 16},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
chrono_ros_interfaces__srv__ChStartSimulation_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {chrono_ros_interfaces__srv__ChStartSimulation_Request__TYPE_NAME, 51, 51},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
chrono_ros_interfaces__srv__ChStartSimulation_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {chrono_ros_interfaces__srv__ChStartSimulation_Response__TYPE_NAME, 52, 52},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
chrono_ros_interfaces__srv__ChStartSimulation_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {chrono_ros_interfaces__srv__ChStartSimulation_Event__TYPE_NAME, 49, 49},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
chrono_ros_interfaces__srv__ChStartSimulation__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *chrono_ros_interfaces__srv__ChStartSimulation__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *chrono_ros_interfaces__srv__ChStartSimulation_Event__get_individual_type_description_source(NULL);
    sources[3] = *chrono_ros_interfaces__srv__ChStartSimulation_Request__get_individual_type_description_source(NULL);
    sources[4] = *chrono_ros_interfaces__srv__ChStartSimulation_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
chrono_ros_interfaces__srv__ChStartSimulation_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *chrono_ros_interfaces__srv__ChStartSimulation_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
chrono_ros_interfaces__srv__ChStartSimulation_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *chrono_ros_interfaces__srv__ChStartSimulation_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
chrono_ros_interfaces__srv__ChStartSimulation_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *chrono_ros_interfaces__srv__ChStartSimulation_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *chrono_ros_interfaces__srv__ChStartSimulation_Request__get_individual_type_description_source(NULL);
    sources[3] = *chrono_ros_interfaces__srv__ChStartSimulation_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
