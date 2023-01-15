// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/hybrid_grid.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "csmlio/lio/proto/hybrid_grid.pb.h"

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
namespace proto {
class HybridGridDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<HybridGrid>
     _instance;
} _HybridGrid_default_instance_;

namespace protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto {


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
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, resolution_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, x_indices_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, y_indices_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, z_indices_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(HybridGrid, values_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(HybridGrid)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_HybridGrid_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping/proto/hybrid_grid.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  _HybridGrid_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_HybridGrid_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n,cartographer/mapping/proto/hybrid_grid"
      ".proto\022\032cartographer.mapping.proto\"i\n\nHy"
      "bridGrid\022\022\n\nresolution\030\001 \001(\002\022\021\n\tx_indice"
      "s\030\003 \003(\021\022\021\n\ty_indices\030\004 \003(\021\022\021\n\tz_indices\030"
      "\005 \003(\021\022\016\n\006values\030\006 \003(\005b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 189);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping/proto/hybrid_grid.proto", &protobuf_RegisterTypes);
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

}  // namespace protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int HybridGrid::kResolutionFieldNumber;
const int HybridGrid::kXIndicesFieldNumber;
const int HybridGrid::kYIndicesFieldNumber;
const int HybridGrid::kZIndicesFieldNumber;
const int HybridGrid::kValuesFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

HybridGrid::HybridGrid()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.HybridGrid)
}
HybridGrid::HybridGrid(const HybridGrid& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      x_indices_(from.x_indices_),
      y_indices_(from.y_indices_),
      z_indices_(from.z_indices_),
      values_(from.values_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  resolution_ = from.resolution_;
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.HybridGrid)
}

void HybridGrid::SharedCtor() {
  resolution_ = 0;
  _cached_size_ = 0;
}

HybridGrid::~HybridGrid() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.HybridGrid)
  SharedDtor();
}

void HybridGrid::SharedDtor() {
}

void HybridGrid::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* HybridGrid::descriptor() {
  protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const HybridGrid& HybridGrid::default_instance() {
  protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto::InitDefaults();
  return *internal_default_instance();
}

HybridGrid* HybridGrid::New(::google::protobuf::Arena* arena) const {
  HybridGrid* n = new HybridGrid;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void HybridGrid::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.HybridGrid)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  x_indices_.Clear();
  y_indices_.Clear();
  z_indices_.Clear();
  values_.Clear();
  resolution_ = 0;
  _internal_metadata_.Clear();
}

bool HybridGrid::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.HybridGrid)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // float resolution = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(13u /* 13 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &resolution_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated sint32 x_indices = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_x_indices())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 26u, input, this->mutable_x_indices())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated sint32 y_indices = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_y_indices())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(32u /* 32 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 34u, input, this->mutable_y_indices())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated sint32 z_indices = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(42u /* 42 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 input, this->mutable_z_indices())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_SINT32>(
                 1, 42u, input, this->mutable_z_indices())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated int32 values = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(50u /* 50 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, this->mutable_values())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(48u /* 48 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 1, 50u, input, this->mutable_values())));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.HybridGrid)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.HybridGrid)
  return false;
#undef DO_
}

void HybridGrid::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.HybridGrid)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float resolution = 1;
  if (this->resolution() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->resolution(), output);
  }

  // repeated sint32 x_indices = 3;
  if (this->x_indices_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(3, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _x_indices_cached_byte_size_));
  }
  for (int i = 0, n = this->x_indices_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->x_indices(i), output);
  }

  // repeated sint32 y_indices = 4;
  if (this->y_indices_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(4, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _y_indices_cached_byte_size_));
  }
  for (int i = 0, n = this->y_indices_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->y_indices(i), output);
  }

  // repeated sint32 z_indices = 5;
  if (this->z_indices_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(5, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _z_indices_cached_byte_size_));
  }
  for (int i = 0, n = this->z_indices_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteSInt32NoTag(
      this->z_indices(i), output);
  }

  // repeated int32 values = 6;
  if (this->values_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(6, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _values_cached_byte_size_));
  }
  for (int i = 0, n = this->values_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32NoTag(
      this->values(i), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.HybridGrid)
}

::google::protobuf::uint8* HybridGrid::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.HybridGrid)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float resolution = 1;
  if (this->resolution() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->resolution(), target);
  }

  // repeated sint32 x_indices = 3;
  if (this->x_indices_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      3,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::uint32>(
            _x_indices_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->x_indices_, target);
  }

  // repeated sint32 y_indices = 4;
  if (this->y_indices_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      4,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::uint32>(
            _y_indices_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->y_indices_, target);
  }

  // repeated sint32 z_indices = 5;
  if (this->z_indices_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      5,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::uint32>(
            _z_indices_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteSInt32NoTagToArray(this->z_indices_, target);
  }

  // repeated int32 values = 6;
  if (this->values_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      6,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::uint32>(
            _values_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteInt32NoTagToArray(this->values_, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.HybridGrid)
  return target;
}

size_t HybridGrid::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.HybridGrid)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated sint32 x_indices = 3;
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      SInt32Size(this->x_indices_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _x_indices_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated sint32 y_indices = 4;
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      SInt32Size(this->y_indices_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _y_indices_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated sint32 z_indices = 5;
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      SInt32Size(this->z_indices_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _z_indices_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // repeated int32 values = 6;
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      Int32Size(this->values_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _values_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  // float resolution = 1;
  if (this->resolution() != 0) {
    total_size += 1 + 4;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void HybridGrid::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.HybridGrid)
  GOOGLE_DCHECK_NE(&from, this);
  const HybridGrid* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const HybridGrid>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.HybridGrid)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.HybridGrid)
    MergeFrom(*source);
  }
}

void HybridGrid::MergeFrom(const HybridGrid& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.HybridGrid)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  x_indices_.MergeFrom(from.x_indices_);
  y_indices_.MergeFrom(from.y_indices_);
  z_indices_.MergeFrom(from.z_indices_);
  values_.MergeFrom(from.values_);
  if (from.resolution() != 0) {
    set_resolution(from.resolution());
  }
}

void HybridGrid::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.HybridGrid)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void HybridGrid::CopyFrom(const HybridGrid& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.HybridGrid)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool HybridGrid::IsInitialized() const {
  return true;
}

void HybridGrid::Swap(HybridGrid* other) {
  if (other == this) return;
  InternalSwap(other);
}
void HybridGrid::InternalSwap(HybridGrid* other) {
  using std::swap;
  x_indices_.InternalSwap(&other->x_indices_);
  y_indices_.InternalSwap(&other->y_indices_);
  z_indices_.InternalSwap(&other->z_indices_);
  values_.InternalSwap(&other->values_);
  swap(resolution_, other->resolution_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata HybridGrid::GetMetadata() const {
  protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fhybrid_5fgrid_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// HybridGrid

// float resolution = 1;
void HybridGrid::clear_resolution() {
  resolution_ = 0;
}
float HybridGrid::resolution() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.resolution)
  return resolution_;
}
void HybridGrid::set_resolution(float value) {
  
  resolution_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.resolution)
}

// repeated sint32 x_indices = 3;
int HybridGrid::x_indices_size() const {
  return x_indices_.size();
}
void HybridGrid::clear_x_indices() {
  x_indices_.Clear();
}
::google::protobuf::int32 HybridGrid::x_indices(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.x_indices)
  return x_indices_.Get(index);
}
void HybridGrid::set_x_indices(int index, ::google::protobuf::int32 value) {
  x_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.x_indices)
}
void HybridGrid::add_x_indices(::google::protobuf::int32 value) {
  x_indices_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.x_indices)
}
const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::x_indices() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.x_indices)
  return x_indices_;
}
::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_x_indices() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.x_indices)
  return &x_indices_;
}

// repeated sint32 y_indices = 4;
int HybridGrid::y_indices_size() const {
  return y_indices_.size();
}
void HybridGrid::clear_y_indices() {
  y_indices_.Clear();
}
::google::protobuf::int32 HybridGrid::y_indices(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.y_indices)
  return y_indices_.Get(index);
}
void HybridGrid::set_y_indices(int index, ::google::protobuf::int32 value) {
  y_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.y_indices)
}
void HybridGrid::add_y_indices(::google::protobuf::int32 value) {
  y_indices_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.y_indices)
}
const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::y_indices() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.y_indices)
  return y_indices_;
}
::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_y_indices() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.y_indices)
  return &y_indices_;
}

// repeated sint32 z_indices = 5;
int HybridGrid::z_indices_size() const {
  return z_indices_.size();
}
void HybridGrid::clear_z_indices() {
  z_indices_.Clear();
}
::google::protobuf::int32 HybridGrid::z_indices(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.z_indices)
  return z_indices_.Get(index);
}
void HybridGrid::set_z_indices(int index, ::google::protobuf::int32 value) {
  z_indices_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.z_indices)
}
void HybridGrid::add_z_indices(::google::protobuf::int32 value) {
  z_indices_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.z_indices)
}
const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::z_indices() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.z_indices)
  return z_indices_;
}
::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_z_indices() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.z_indices)
  return &z_indices_;
}

// repeated int32 values = 6;
int HybridGrid::values_size() const {
  return values_.size();
}
void HybridGrid::clear_values() {
  values_.Clear();
}
::google::protobuf::int32 HybridGrid::values(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.HybridGrid.values)
  return values_.Get(index);
}
void HybridGrid::set_values(int index, ::google::protobuf::int32 value) {
  values_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.HybridGrid.values)
}
void HybridGrid::add_values(::google::protobuf::int32 value) {
  values_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.HybridGrid.values)
}
const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
HybridGrid::values() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.HybridGrid.values)
  return values_;
}
::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
HybridGrid::mutable_values() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.HybridGrid.values)
  return &values_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)
