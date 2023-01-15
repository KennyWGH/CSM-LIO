// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/connected_components.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "csmlio/lio/proto/connected_components.pb.h"

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
class ConnectedComponents_ConnectedComponentDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<ConnectedComponents_ConnectedComponent>
     _instance;
} _ConnectedComponents_ConnectedComponent_default_instance_;
class ConnectedComponentsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<ConnectedComponents>
     _instance;
} _ConnectedComponents_default_instance_;

namespace protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[2];

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
  { NULL, NULL, 0, -1, -1, -1, -1, NULL, false },
};

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ConnectedComponents_ConnectedComponent, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ConnectedComponents_ConnectedComponent, trajectory_id_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ConnectedComponents, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(ConnectedComponents, connected_component_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(ConnectedComponents_ConnectedComponent)},
  { 6, -1, sizeof(ConnectedComponents)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_ConnectedComponents_ConnectedComponent_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&_ConnectedComponents_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping/proto/connected_components.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

}  // namespace
void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _ConnectedComponents_ConnectedComponent_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_ConnectedComponents_ConnectedComponent_default_instance_);_ConnectedComponents_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_ConnectedComponents_default_instance_);}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n5cartographer/mapping/proto/connected_c"
      "omponents.proto\022\032cartographer.mapping.pr"
      "oto\"\243\001\n\023ConnectedComponents\022_\n\023connected"
      "_component\030\001 \003(\0132B.cartographer.mapping."
      "proto.ConnectedComponents.ConnectedCompo"
      "nent\032+\n\022ConnectedComponent\022\025\n\rtrajectory"
      "_id\030\001 \003(\005B\037B\035ConnectedComponentsOuterCla"
      "ssb\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 290);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping/proto/connected_components.proto", &protobuf_RegisterTypes);
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

}  // namespace protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ConnectedComponents_ConnectedComponent::kTrajectoryIdFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ConnectedComponents_ConnectedComponent::ConnectedComponents_ConnectedComponent()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
}
ConnectedComponents_ConnectedComponent::ConnectedComponents_ConnectedComponent(const ConnectedComponents_ConnectedComponent& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      trajectory_id_(from.trajectory_id_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
}

void ConnectedComponents_ConnectedComponent::SharedCtor() {
  _cached_size_ = 0;
}

ConnectedComponents_ConnectedComponent::~ConnectedComponents_ConnectedComponent() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  SharedDtor();
}

void ConnectedComponents_ConnectedComponent::SharedDtor() {
}

void ConnectedComponents_ConnectedComponent::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* ConnectedComponents_ConnectedComponent::descriptor() {
  protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ConnectedComponents_ConnectedComponent& ConnectedComponents_ConnectedComponent::default_instance() {
  protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::InitDefaults();
  return *internal_default_instance();
}

ConnectedComponents_ConnectedComponent* ConnectedComponents_ConnectedComponent::New(::google::protobuf::Arena* arena) const {
  ConnectedComponents_ConnectedComponent* n = new ConnectedComponents_ConnectedComponent;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void ConnectedComponents_ConnectedComponent::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  trajectory_id_.Clear();
  _internal_metadata_.Clear();
}

bool ConnectedComponents_ConnectedComponent::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated int32 trajectory_id = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadPackedPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, this->mutable_trajectory_id())));
        } else if (
            static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {
          DO_((::google::protobuf::internal::WireFormatLite::ReadRepeatedPrimitiveNoInline<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 1, 10u, input, this->mutable_trajectory_id())));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  return false;
#undef DO_
}

void ConnectedComponents_ConnectedComponent::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated int32 trajectory_id = 1;
  if (this->trajectory_id_size() > 0) {
    ::google::protobuf::internal::WireFormatLite::WriteTag(1, ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED, output);
    output->WriteVarint32(static_cast< ::google::protobuf::uint32>(
        _trajectory_id_cached_byte_size_));
  }
  for (int i = 0, n = this->trajectory_id_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32NoTag(
      this->trajectory_id(i), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
}

::google::protobuf::uint8* ConnectedComponents_ConnectedComponent::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated int32 trajectory_id = 1;
  if (this->trajectory_id_size() > 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteTagToArray(
      1,
      ::google::protobuf::internal::WireFormatLite::WIRETYPE_LENGTH_DELIMITED,
      target);
    target = ::google::protobuf::io::CodedOutputStream::WriteVarint32ToArray(
        static_cast< ::google::protobuf::uint32>(
            _trajectory_id_cached_byte_size_), target);
    target = ::google::protobuf::internal::WireFormatLite::
      WriteInt32NoTagToArray(this->trajectory_id_, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  return target;
}

size_t ConnectedComponents_ConnectedComponent::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated int32 trajectory_id = 1;
  {
    size_t data_size = ::google::protobuf::internal::WireFormatLite::
      Int32Size(this->trajectory_id_);
    if (data_size > 0) {
      total_size += 1 +
        ::google::protobuf::internal::WireFormatLite::Int32Size(
            static_cast< ::google::protobuf::int32>(data_size));
    }
    int cached_size = ::google::protobuf::internal::ToCachedSize(data_size);
    GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
    _trajectory_id_cached_byte_size_ = cached_size;
    GOOGLE_SAFE_CONCURRENT_WRITES_END();
    total_size += data_size;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void ConnectedComponents_ConnectedComponent::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  GOOGLE_DCHECK_NE(&from, this);
  const ConnectedComponents_ConnectedComponent* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ConnectedComponents_ConnectedComponent>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
    MergeFrom(*source);
  }
}

void ConnectedComponents_ConnectedComponent::MergeFrom(const ConnectedComponents_ConnectedComponent& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  trajectory_id_.MergeFrom(from.trajectory_id_);
}

void ConnectedComponents_ConnectedComponent::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ConnectedComponents_ConnectedComponent::CopyFrom(const ConnectedComponents_ConnectedComponent& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ConnectedComponents_ConnectedComponent::IsInitialized() const {
  return true;
}

void ConnectedComponents_ConnectedComponent::Swap(ConnectedComponents_ConnectedComponent* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ConnectedComponents_ConnectedComponent::InternalSwap(ConnectedComponents_ConnectedComponent* other) {
  using std::swap;
  trajectory_id_.InternalSwap(&other->trajectory_id_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata ConnectedComponents_ConnectedComponent::GetMetadata() const {
  protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// ConnectedComponents_ConnectedComponent

// repeated int32 trajectory_id = 1;
int ConnectedComponents_ConnectedComponent::trajectory_id_size() const {
  return trajectory_id_.size();
}
void ConnectedComponents_ConnectedComponent::clear_trajectory_id() {
  trajectory_id_.Clear();
}
::google::protobuf::int32 ConnectedComponents_ConnectedComponent::trajectory_id(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
  return trajectory_id_.Get(index);
}
void ConnectedComponents_ConnectedComponent::set_trajectory_id(int index, ::google::protobuf::int32 value) {
  trajectory_id_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
}
void ConnectedComponents_ConnectedComponent::add_trajectory_id(::google::protobuf::int32 value) {
  trajectory_id_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
}
const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
ConnectedComponents_ConnectedComponent::trajectory_id() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
  return trajectory_id_;
}
::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
ConnectedComponents_ConnectedComponent::mutable_trajectory_id() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
  return &trajectory_id_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int ConnectedComponents::kConnectedComponentFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

ConnectedComponents::ConnectedComponents()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.ConnectedComponents)
}
ConnectedComponents::ConnectedComponents(const ConnectedComponents& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      connected_component_(from.connected_component_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.ConnectedComponents)
}

void ConnectedComponents::SharedCtor() {
  _cached_size_ = 0;
}

ConnectedComponents::~ConnectedComponents() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.ConnectedComponents)
  SharedDtor();
}

void ConnectedComponents::SharedDtor() {
}

void ConnectedComponents::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* ConnectedComponents::descriptor() {
  protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const ConnectedComponents& ConnectedComponents::default_instance() {
  protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::InitDefaults();
  return *internal_default_instance();
}

ConnectedComponents* ConnectedComponents::New(::google::protobuf::Arena* arena) const {
  ConnectedComponents* n = new ConnectedComponents;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void ConnectedComponents::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.ConnectedComponents)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  connected_component_.Clear();
  _internal_metadata_.Clear();
}

bool ConnectedComponents::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.ConnectedComponents)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
                input, add_connected_component()));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.ConnectedComponents)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.ConnectedComponents)
  return false;
#undef DO_
}

void ConnectedComponents::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.ConnectedComponents)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->connected_component_size()); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->connected_component(static_cast<int>(i)), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.ConnectedComponents)
}

::google::protobuf::uint8* ConnectedComponents::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.ConnectedComponents)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->connected_component_size()); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, this->connected_component(static_cast<int>(i)), deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.ConnectedComponents)
  return target;
}

size_t ConnectedComponents::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.ConnectedComponents)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->connected_component_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->connected_component(static_cast<int>(i)));
    }
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void ConnectedComponents::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.ConnectedComponents)
  GOOGLE_DCHECK_NE(&from, this);
  const ConnectedComponents* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const ConnectedComponents>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.ConnectedComponents)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.ConnectedComponents)
    MergeFrom(*source);
  }
}

void ConnectedComponents::MergeFrom(const ConnectedComponents& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.ConnectedComponents)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  connected_component_.MergeFrom(from.connected_component_);
}

void ConnectedComponents::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.ConnectedComponents)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ConnectedComponents::CopyFrom(const ConnectedComponents& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.ConnectedComponents)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ConnectedComponents::IsInitialized() const {
  return true;
}

void ConnectedComponents::Swap(ConnectedComponents* other) {
  if (other == this) return;
  InternalSwap(other);
}
void ConnectedComponents::InternalSwap(ConnectedComponents* other) {
  using std::swap;
  connected_component_.InternalSwap(&other->connected_component_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata ConnectedComponents::GetMetadata() const {
  protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// ConnectedComponents

// repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
int ConnectedComponents::connected_component_size() const {
  return connected_component_.size();
}
void ConnectedComponents::clear_connected_component() {
  connected_component_.Clear();
}
const ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent& ConnectedComponents::connected_component(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_.Get(index);
}
::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent* ConnectedComponents::mutable_connected_component(int index) {
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_.Mutable(index);
}
::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent* ConnectedComponents::add_connected_component() {
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_.Add();
}
::google::protobuf::RepeatedPtrField< ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent >*
ConnectedComponents::mutable_connected_component() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return &connected_component_;
}
const ::google::protobuf::RepeatedPtrField< ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent >&
ConnectedComponents::connected_component() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_;
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)
