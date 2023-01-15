// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "csmlio/lio/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"

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
namespace mapping {
namespace scan_matching {
namespace proto {
class RealTimeCorrelativeScanMatcherOptionsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<RealTimeCorrelativeScanMatcherOptions>
     _instance;
} _RealTimeCorrelativeScanMatcherOptions_default_instance_;

namespace protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto {


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
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RealTimeCorrelativeScanMatcherOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RealTimeCorrelativeScanMatcherOptions, linear_search_window_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RealTimeCorrelativeScanMatcherOptions, angular_search_window_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RealTimeCorrelativeScanMatcherOptions, translation_delta_cost_weight_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(RealTimeCorrelativeScanMatcherOptions, rotation_delta_cost_weight_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(RealTimeCorrelativeScanMatcherOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_RealTimeCorrelativeScanMatcherOptions_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  _RealTimeCorrelativeScanMatcherOptions_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_RealTimeCorrelativeScanMatcherOptions_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\nYcartographer/mapping/proto/scan_matchi"
      "ng/real_time_correlative_scan_matcher_op"
      "tions.proto\022(cartographer.mapping.scan_m"
      "atching.proto\"\257\001\n%RealTimeCorrelativeSca"
      "nMatcherOptions\022\034\n\024linear_search_window\030"
      "\001 \001(\001\022\035\n\025angular_search_window\030\002 \001(\001\022%\n\035"
      "translation_delta_cost_weight\030\003 \001(\001\022\"\n\032r"
      "otation_delta_cost_weight\030\004 \001(\001b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 319);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping/proto/scan_matching/real_time_correlative_scan_matcher_options.proto", &protobuf_RegisterTypes);
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

}  // namespace protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int RealTimeCorrelativeScanMatcherOptions::kLinearSearchWindowFieldNumber;
const int RealTimeCorrelativeScanMatcherOptions::kAngularSearchWindowFieldNumber;
const int RealTimeCorrelativeScanMatcherOptions::kTranslationDeltaCostWeightFieldNumber;
const int RealTimeCorrelativeScanMatcherOptions::kRotationDeltaCostWeightFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

RealTimeCorrelativeScanMatcherOptions::RealTimeCorrelativeScanMatcherOptions()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
}
RealTimeCorrelativeScanMatcherOptions::RealTimeCorrelativeScanMatcherOptions(const RealTimeCorrelativeScanMatcherOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&linear_search_window_, &from.linear_search_window_,
    static_cast<size_t>(reinterpret_cast<char*>(&rotation_delta_cost_weight_) -
    reinterpret_cast<char*>(&linear_search_window_)) + sizeof(rotation_delta_cost_weight_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
}

void RealTimeCorrelativeScanMatcherOptions::SharedCtor() {
  ::memset(&linear_search_window_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rotation_delta_cost_weight_) -
      reinterpret_cast<char*>(&linear_search_window_)) + sizeof(rotation_delta_cost_weight_));
  _cached_size_ = 0;
}

RealTimeCorrelativeScanMatcherOptions::~RealTimeCorrelativeScanMatcherOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  SharedDtor();
}

void RealTimeCorrelativeScanMatcherOptions::SharedDtor() {
}

void RealTimeCorrelativeScanMatcherOptions::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* RealTimeCorrelativeScanMatcherOptions::descriptor() {
  protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const RealTimeCorrelativeScanMatcherOptions& RealTimeCorrelativeScanMatcherOptions::default_instance() {
  protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::InitDefaults();
  return *internal_default_instance();
}

RealTimeCorrelativeScanMatcherOptions* RealTimeCorrelativeScanMatcherOptions::New(::google::protobuf::Arena* arena) const {
  RealTimeCorrelativeScanMatcherOptions* n = new RealTimeCorrelativeScanMatcherOptions;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void RealTimeCorrelativeScanMatcherOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&linear_search_window_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rotation_delta_cost_weight_) -
      reinterpret_cast<char*>(&linear_search_window_)) + sizeof(rotation_delta_cost_weight_));
  _internal_metadata_.Clear();
}

bool RealTimeCorrelativeScanMatcherOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double linear_search_window = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &linear_search_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double angular_search_window = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &angular_search_window_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double translation_delta_cost_weight = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &translation_delta_cost_weight_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double rotation_delta_cost_weight = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(33u /* 33 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &rotation_delta_cost_weight_)));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  return false;
#undef DO_
}

void RealTimeCorrelativeScanMatcherOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double linear_search_window = 1;
  if (this->linear_search_window() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->linear_search_window(), output);
  }

  // double angular_search_window = 2;
  if (this->angular_search_window() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->angular_search_window(), output);
  }

  // double translation_delta_cost_weight = 3;
  if (this->translation_delta_cost_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->translation_delta_cost_weight(), output);
  }

  // double rotation_delta_cost_weight = 4;
  if (this->rotation_delta_cost_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->rotation_delta_cost_weight(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
}

::google::protobuf::uint8* RealTimeCorrelativeScanMatcherOptions::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double linear_search_window = 1;
  if (this->linear_search_window() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->linear_search_window(), target);
  }

  // double angular_search_window = 2;
  if (this->angular_search_window() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->angular_search_window(), target);
  }

  // double translation_delta_cost_weight = 3;
  if (this->translation_delta_cost_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->translation_delta_cost_weight(), target);
  }

  // double rotation_delta_cost_weight = 4;
  if (this->rotation_delta_cost_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->rotation_delta_cost_weight(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  return target;
}

size_t RealTimeCorrelativeScanMatcherOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // double linear_search_window = 1;
  if (this->linear_search_window() != 0) {
    total_size += 1 + 8;
  }

  // double angular_search_window = 2;
  if (this->angular_search_window() != 0) {
    total_size += 1 + 8;
  }

  // double translation_delta_cost_weight = 3;
  if (this->translation_delta_cost_weight() != 0) {
    total_size += 1 + 8;
  }

  // double rotation_delta_cost_weight = 4;
  if (this->rotation_delta_cost_weight() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void RealTimeCorrelativeScanMatcherOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const RealTimeCorrelativeScanMatcherOptions* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const RealTimeCorrelativeScanMatcherOptions>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
    MergeFrom(*source);
  }
}

void RealTimeCorrelativeScanMatcherOptions::MergeFrom(const RealTimeCorrelativeScanMatcherOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.linear_search_window() != 0) {
    set_linear_search_window(from.linear_search_window());
  }
  if (from.angular_search_window() != 0) {
    set_angular_search_window(from.angular_search_window());
  }
  if (from.translation_delta_cost_weight() != 0) {
    set_translation_delta_cost_weight(from.translation_delta_cost_weight());
  }
  if (from.rotation_delta_cost_weight() != 0) {
    set_rotation_delta_cost_weight(from.rotation_delta_cost_weight());
  }
}

void RealTimeCorrelativeScanMatcherOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void RealTimeCorrelativeScanMatcherOptions::CopyFrom(const RealTimeCorrelativeScanMatcherOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool RealTimeCorrelativeScanMatcherOptions::IsInitialized() const {
  return true;
}

void RealTimeCorrelativeScanMatcherOptions::Swap(RealTimeCorrelativeScanMatcherOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void RealTimeCorrelativeScanMatcherOptions::InternalSwap(RealTimeCorrelativeScanMatcherOptions* other) {
  using std::swap;
  swap(linear_search_window_, other->linear_search_window_);
  swap(angular_search_window_, other->angular_search_window_);
  swap(translation_delta_cost_weight_, other->translation_delta_cost_weight_);
  swap(rotation_delta_cost_weight_, other->rotation_delta_cost_weight_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata RealTimeCorrelativeScanMatcherOptions::GetMetadata() const {
  protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2freal_5ftime_5fcorrelative_5fscan_5fmatcher_5foptions_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// RealTimeCorrelativeScanMatcherOptions

// double linear_search_window = 1;
void RealTimeCorrelativeScanMatcherOptions::clear_linear_search_window() {
  linear_search_window_ = 0;
}
double RealTimeCorrelativeScanMatcherOptions::linear_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.linear_search_window)
  return linear_search_window_;
}
void RealTimeCorrelativeScanMatcherOptions::set_linear_search_window(double value) {
  
  linear_search_window_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.linear_search_window)
}

// double angular_search_window = 2;
void RealTimeCorrelativeScanMatcherOptions::clear_angular_search_window() {
  angular_search_window_ = 0;
}
double RealTimeCorrelativeScanMatcherOptions::angular_search_window() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.angular_search_window)
  return angular_search_window_;
}
void RealTimeCorrelativeScanMatcherOptions::set_angular_search_window(double value) {
  
  angular_search_window_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.angular_search_window)
}

// double translation_delta_cost_weight = 3;
void RealTimeCorrelativeScanMatcherOptions::clear_translation_delta_cost_weight() {
  translation_delta_cost_weight_ = 0;
}
double RealTimeCorrelativeScanMatcherOptions::translation_delta_cost_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.translation_delta_cost_weight)
  return translation_delta_cost_weight_;
}
void RealTimeCorrelativeScanMatcherOptions::set_translation_delta_cost_weight(double value) {
  
  translation_delta_cost_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.translation_delta_cost_weight)
}

// double rotation_delta_cost_weight = 4;
void RealTimeCorrelativeScanMatcherOptions::clear_rotation_delta_cost_weight() {
  rotation_delta_cost_weight_ = 0;
}
double RealTimeCorrelativeScanMatcherOptions::rotation_delta_cost_weight() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.rotation_delta_cost_weight)
  return rotation_delta_cost_weight_;
}
void RealTimeCorrelativeScanMatcherOptions::set_rotation_delta_cost_weight(double value) {
  
  rotation_delta_cost_weight_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.scan_matching.proto.RealTimeCorrelativeScanMatcherOptions.rotation_delta_cost_weight)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)
