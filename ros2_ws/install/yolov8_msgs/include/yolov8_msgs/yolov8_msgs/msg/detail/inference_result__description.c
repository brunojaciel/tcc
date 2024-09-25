// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from yolov8_msgs:msg/InferenceResult.idl
// generated code does not contain a copyright notice

#include "yolov8_msgs/msg/detail/inference_result__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_yolov8_msgs
const rosidl_type_hash_t *
yolov8_msgs__msg__InferenceResult__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x53, 0x31, 0x3b, 0xd9, 0x8a, 0xd7, 0x67, 0x4e,
      0x29, 0x5c, 0xfd, 0x84, 0x17, 0xb9, 0x0f, 0xdc,
      0x73, 0xcf, 0xc7, 0x70, 0x8a, 0xd4, 0x75, 0xc4,
      0x5a, 0xd8, 0x47, 0x58, 0xd5, 0x69, 0x34, 0xd2,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char yolov8_msgs__msg__InferenceResult__TYPE_NAME[] = "yolov8_msgs/msg/InferenceResult";

// Define type names, field names, and default values
static char yolov8_msgs__msg__InferenceResult__FIELD_NAME__class_name[] = "class_name";
static char yolov8_msgs__msg__InferenceResult__FIELD_NAME__top[] = "top";
static char yolov8_msgs__msg__InferenceResult__FIELD_NAME__left[] = "left";
static char yolov8_msgs__msg__InferenceResult__FIELD_NAME__bottom[] = "bottom";
static char yolov8_msgs__msg__InferenceResult__FIELD_NAME__right[] = "right";
static char yolov8_msgs__msg__InferenceResult__FIELD_NAME__category[] = "category";

static rosidl_runtime_c__type_description__Field yolov8_msgs__msg__InferenceResult__FIELDS[] = {
  {
    {yolov8_msgs__msg__InferenceResult__FIELD_NAME__class_name, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolov8_msgs__msg__InferenceResult__FIELD_NAME__top, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolov8_msgs__msg__InferenceResult__FIELD_NAME__left, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolov8_msgs__msg__InferenceResult__FIELD_NAME__bottom, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolov8_msgs__msg__InferenceResult__FIELD_NAME__right, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {yolov8_msgs__msg__InferenceResult__FIELD_NAME__category, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
yolov8_msgs__msg__InferenceResult__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {yolov8_msgs__msg__InferenceResult__TYPE_NAME, 31, 31},
      {yolov8_msgs__msg__InferenceResult__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string class_name\n"
  "int64 top\n"
  "int64 left\n"
  "int64 bottom\n"
  "int64 right\n"
  "int8 category";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
yolov8_msgs__msg__InferenceResult__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {yolov8_msgs__msg__InferenceResult__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 78, 78},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
yolov8_msgs__msg__InferenceResult__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *yolov8_msgs__msg__InferenceResult__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
