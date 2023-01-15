// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/sensor/proto/adaptive_voxel_filter_options.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "csmlio/sensor/proto/adaptive_voxel_filter_options.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace csmlio {
namespace sensor {
namespace proto {
class AdaptiveVoxelFilterOptionsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<AdaptiveVoxelFilterOptions>
     _instance;
} _AdaptiveVoxelFilterOptions_default_instance_;

namespace protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[1];

}  // namespace

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTableField
    const TableStruct::entries[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  {0, 0, 0, ::google::protobuf::internal::kInvalidMask, 0, 0},
};

PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::AuxillaryParseTableField
    const TableStruct::aux[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ::google::protobuf::internal::AuxillaryParseTableField(),
};
PROTOBUF_CONSTEXPR_VAR ::google::protobuf::internal::ParseTable const
    TableStruct::schema[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { NULL, NULL, 0, -1, -1, -1, -1, NULL, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AdaptiveVoxelFilterOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AdaptiveVoxelFilterOptions, max_length_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AdaptiveVoxelFilterOptions, min_num_points_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(AdaptiveVoxelFilterOptions, max_range_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(AdaptiveVoxelFilterOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_AdaptiveVoxelFilterOptions_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/sensor/proto/adaptive_voxel_filter_options.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

}  // namespace
void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _AdaptiveVoxelFilterOptions_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_AdaptiveVoxelFilterOptions_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n=cartographer/sensor/proto/adaptive_vox"
      "el_filter_options.proto\022\031cartographer.se"
      "nsor.proto\"[\n\032AdaptiveVoxelFilterOptions"
      "\022\022\n\nmax_length\030\001 \001(\002\022\026\n\016min_num_points\030\002"
      " \001(\002\022\021\n\tmax_range\030\003 \001(\002b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 191);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/sensor/proto/adaptive_voxel_filter_options.proto", &protobuf_RegisterTypes);
}
} // anonymous namespace

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int AdaptiveVoxelFilterOptions::kMaxLengthFieldNumber;
const int AdaptiveVoxelFilterOptions::kMinNumPointsFieldNumber;
const int AdaptiveVoxelFilterOptions::kMaxRangeFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

AdaptiveVoxelFilterOptions::AdaptiveVoxelFilterOptions()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
}
AdaptiveVoxelFilterOptions::AdaptiveVoxelFilterOptions(const AdaptiveVoxelFilterOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&max_length_, &from.max_length_,
    static_cast<size_t>(reinterpret_cast<char*>(&max_range_) -
    reinterpret_cast<char*>(&max_length_)) + sizeof(max_range_));
  // @@protoc_insertion_point(copy_constructor:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
}

void AdaptiveVoxelFilterOptions::SharedCtor() {
  ::memset(&max_length_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&max_range_) -
      reinterpret_cast<char*>(&max_length_)) + sizeof(max_range_));
  _cached_size_ = 0;
}

AdaptiveVoxelFilterOptions::~AdaptiveVoxelFilterOptions() {
  // @@protoc_insertion_point(destructor:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  SharedDtor();
}

void AdaptiveVoxelFilterOptions::SharedDtor() {
}

void AdaptiveVoxelFilterOptions::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* AdaptiveVoxelFilterOptions::descriptor() {
  protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const AdaptiveVoxelFilterOptions& AdaptiveVoxelFilterOptions::default_instance() {
  protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto::InitDefaults();
  return *internal_default_instance();
}

AdaptiveVoxelFilterOptions* AdaptiveVoxelFilterOptions::New(::google::protobuf::Arena* arena) const {
  AdaptiveVoxelFilterOptions* n = new AdaptiveVoxelFilterOptions;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void AdaptiveVoxelFilterOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&max_length_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&max_range_) -
      reinterpret_cast<char*>(&max_length_)) + sizeof(max_range_));
  _internal_metadata_.Clear();
}

bool AdaptiveVoxelFilterOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // float max_length = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(13u /* 13 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &max_length_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float min_num_points = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &min_num_points_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float max_range = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &max_range_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  return false;
#undef DO_
}

void AdaptiveVoxelFilterOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float max_length = 1;
  if (this->max_length() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->max_length(), output);
  }

  // float min_num_points = 2;
  if (this->min_num_points() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->min_num_points(), output);
  }

  // float max_range = 3;
  if (this->max_range() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->max_range(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
}

::google::protobuf::uint8* AdaptiveVoxelFilterOptions::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float max_length = 1;
  if (this->max_length() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->max_length(), target);
  }

  // float min_num_points = 2;
  if (this->min_num_points() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->min_num_points(), target);
  }

  // float max_range = 3;
  if (this->max_range() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->max_range(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  return target;
}

size_t AdaptiveVoxelFilterOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // float max_length = 1;
  if (this->max_length() != 0) {
    total_size += 1 + 4;
  }

  // float min_num_points = 2;
  if (this->min_num_points() != 0) {
    total_size += 1 + 4;
  }

  // float max_range = 3;
  if (this->max_range() != 0) {
    total_size += 1 + 4;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void AdaptiveVoxelFilterOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const AdaptiveVoxelFilterOptions* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const AdaptiveVoxelFilterOptions>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
    MergeFrom(*source);
  }
}

void AdaptiveVoxelFilterOptions::MergeFrom(const AdaptiveVoxelFilterOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.max_length() != 0) {
    set_max_length(from.max_length());
  }
  if (from.min_num_points() != 0) {
    set_min_num_points(from.min_num_points());
  }
  if (from.max_range() != 0) {
    set_max_range(from.max_range());
  }
}

void AdaptiveVoxelFilterOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void AdaptiveVoxelFilterOptions::CopyFrom(const AdaptiveVoxelFilterOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool AdaptiveVoxelFilterOptions::IsInitialized() const {
  return true;
}

void AdaptiveVoxelFilterOptions::Swap(AdaptiveVoxelFilterOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void AdaptiveVoxelFilterOptions::InternalSwap(AdaptiveVoxelFilterOptions* other) {
  using std::swap;
  swap(max_length_, other->max_length_);
  swap(min_num_points_, other->min_num_points_);
  swap(max_range_, other->max_range_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata AdaptiveVoxelFilterOptions::GetMetadata() const {
  protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// AdaptiveVoxelFilterOptions

// float max_length = 1;
void AdaptiveVoxelFilterOptions::clear_max_length() {
  max_length_ = 0;
}
float AdaptiveVoxelFilterOptions::max_length() const {
  // @@protoc_insertion_point(field_get:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_length)
  return max_length_;
}
void AdaptiveVoxelFilterOptions::set_max_length(float value) {
  
  max_length_ = value;
  // @@protoc_insertion_point(field_set:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_length)
}

// float min_num_points = 2;
void AdaptiveVoxelFilterOptions::clear_min_num_points() {
  min_num_points_ = 0;
}
float AdaptiveVoxelFilterOptions::min_num_points() const {
  // @@protoc_insertion_point(field_get:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.min_num_points)
  return min_num_points_;
}
void AdaptiveVoxelFilterOptions::set_min_num_points(float value) {
  
  min_num_points_ = value;
  // @@protoc_insertion_point(field_set:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.min_num_points)
}

// float max_range = 3;
void AdaptiveVoxelFilterOptions::clear_max_range() {
  max_range_ = 0;
}
float AdaptiveVoxelFilterOptions::max_range() const {
  // @@protoc_insertion_point(field_get:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_range)
  return max_range_;
}
void AdaptiveVoxelFilterOptions::set_max_range(float value) {
  
  max_range_ = value;
  // @@protoc_insertion_point(field_set:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_range)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace sensor
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)
