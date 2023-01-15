// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/map_builder_options.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "csmlio/lio/proto/map_builder_options.pb.h"

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
class MapBuilderOptionsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<MapBuilderOptions>
     _instance;
} _MapBuilderOptions_default_instance_;

namespace protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto {


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
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapBuilderOptions, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapBuilderOptions, use_trajectory_builder_2d_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapBuilderOptions, use_trajectory_builder_3d_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapBuilderOptions, num_background_threads_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapBuilderOptions, pose_graph_options_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapBuilderOptions, collate_by_trajectory_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(MapBuilderOptions)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_MapBuilderOptions_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping/proto/map_builder_options.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  ::csmlio::mapping::proto::protobuf_cartographer_2fmapping_2fproto_2fpose_5fgraph_5foptions_2eproto::InitDefaults();
  _MapBuilderOptions_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_MapBuilderOptions_default_instance_);_MapBuilderOptions_default_instance_._instance.get_mutable()->pose_graph_options_ = const_cast< ::csmlio::mapping::proto::PoseGraphOptions*>(
      ::csmlio::mapping::proto::PoseGraphOptions::internal_default_instance());
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n4cartographer/mapping/proto/map_builder"
      "_options.proto\022\032cartographer.mapping.pro"
      "to\0323cartographer/mapping/proto/pose_grap"
      "h_options.proto\"\342\001\n\021MapBuilderOptions\022!\n"
      "\031use_trajectory_builder_2d\030\001 \001(\010\022!\n\031use_"
      "trajectory_builder_3d\030\002 \001(\010\022\036\n\026num_backg"
      "round_threads\030\003 \001(\005\022H\n\022pose_graph_option"
      "s\030\004 \001(\0132,.cartographer.mapping.proto.Pos"
      "eGraphOptions\022\035\n\025collate_by_trajectory\030\005"
      " \001(\010b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 372);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping/proto/map_builder_options.proto", &protobuf_RegisterTypes);
  ::csmlio::mapping::proto::protobuf_cartographer_2fmapping_2fproto_2fpose_5fgraph_5foptions_2eproto::AddDescriptors();
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

}  // namespace protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int MapBuilderOptions::kUseTrajectoryBuilder2DFieldNumber;
const int MapBuilderOptions::kUseTrajectoryBuilder3DFieldNumber;
const int MapBuilderOptions::kNumBackgroundThreadsFieldNumber;
const int MapBuilderOptions::kPoseGraphOptionsFieldNumber;
const int MapBuilderOptions::kCollateByTrajectoryFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

MapBuilderOptions::MapBuilderOptions()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.MapBuilderOptions)
}
MapBuilderOptions::MapBuilderOptions(const MapBuilderOptions& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_pose_graph_options()) {
    pose_graph_options_ = new ::csmlio::mapping::proto::PoseGraphOptions(*from.pose_graph_options_);
  } else {
    pose_graph_options_ = NULL;
  }
  ::memcpy(&use_trajectory_builder_2d_, &from.use_trajectory_builder_2d_,
    static_cast<size_t>(reinterpret_cast<char*>(&num_background_threads_) -
    reinterpret_cast<char*>(&use_trajectory_builder_2d_)) + sizeof(num_background_threads_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.MapBuilderOptions)
}

void MapBuilderOptions::SharedCtor() {
  ::memset(&pose_graph_options_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_background_threads_) -
      reinterpret_cast<char*>(&pose_graph_options_)) + sizeof(num_background_threads_));
  _cached_size_ = 0;
}

MapBuilderOptions::~MapBuilderOptions() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.MapBuilderOptions)
  SharedDtor();
}

void MapBuilderOptions::SharedDtor() {
  if (this != internal_default_instance()) delete pose_graph_options_;
}

void MapBuilderOptions::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* MapBuilderOptions::descriptor() {
  protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const MapBuilderOptions& MapBuilderOptions::default_instance() {
  protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto::InitDefaults();
  return *internal_default_instance();
}

MapBuilderOptions* MapBuilderOptions::New(::google::protobuf::Arena* arena) const {
  MapBuilderOptions* n = new MapBuilderOptions;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void MapBuilderOptions::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.MapBuilderOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == NULL && pose_graph_options_ != NULL) {
    delete pose_graph_options_;
  }
  pose_graph_options_ = NULL;
  ::memset(&use_trajectory_builder_2d_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&num_background_threads_) -
      reinterpret_cast<char*>(&use_trajectory_builder_2d_)) + sizeof(num_background_threads_));
  _internal_metadata_.Clear();
}

bool MapBuilderOptions::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.MapBuilderOptions)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // bool use_trajectory_builder_2d = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(8u /* 8 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &use_trajectory_builder_2d_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // bool use_trajectory_builder_3d = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(16u /* 16 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &use_trajectory_builder_3d_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 num_background_threads = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(24u /* 24 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &num_background_threads_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .cartographer.mapping.proto.PoseGraphOptions pose_graph_options = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_pose_graph_options()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // bool collate_by_trajectory = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(40u /* 40 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   bool, ::google::protobuf::internal::WireFormatLite::TYPE_BOOL>(
                 input, &collate_by_trajectory_)));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.MapBuilderOptions)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.MapBuilderOptions)
  return false;
#undef DO_
}

void MapBuilderOptions::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.MapBuilderOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // bool use_trajectory_builder_2d = 1;
  if (this->use_trajectory_builder_2d() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(1, this->use_trajectory_builder_2d(), output);
  }

  // bool use_trajectory_builder_3d = 2;
  if (this->use_trajectory_builder_3d() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(2, this->use_trajectory_builder_3d(), output);
  }

  // int32 num_background_threads = 3;
  if (this->num_background_threads() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->num_background_threads(), output);
  }

  // .cartographer.mapping.proto.PoseGraphOptions pose_graph_options = 4;
  if (this->has_pose_graph_options()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, *this->pose_graph_options_, output);
  }

  // bool collate_by_trajectory = 5;
  if (this->collate_by_trajectory() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteBool(5, this->collate_by_trajectory(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.MapBuilderOptions)
}

::google::protobuf::uint8* MapBuilderOptions::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.MapBuilderOptions)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // bool use_trajectory_builder_2d = 1;
  if (this->use_trajectory_builder_2d() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(1, this->use_trajectory_builder_2d(), target);
  }

  // bool use_trajectory_builder_3d = 2;
  if (this->use_trajectory_builder_3d() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(2, this->use_trajectory_builder_3d(), target);
  }

  // int32 num_background_threads = 3;
  if (this->num_background_threads() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->num_background_threads(), target);
  }

  // .cartographer.mapping.proto.PoseGraphOptions pose_graph_options = 4;
  if (this->has_pose_graph_options()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        4, *this->pose_graph_options_, deterministic, target);
  }

  // bool collate_by_trajectory = 5;
  if (this->collate_by_trajectory() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteBoolToArray(5, this->collate_by_trajectory(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.MapBuilderOptions)
  return target;
}

size_t MapBuilderOptions::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.MapBuilderOptions)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // .cartographer.mapping.proto.PoseGraphOptions pose_graph_options = 4;
  if (this->has_pose_graph_options()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->pose_graph_options_);
  }

  // bool use_trajectory_builder_2d = 1;
  if (this->use_trajectory_builder_2d() != 0) {
    total_size += 1 + 1;
  }

  // bool use_trajectory_builder_3d = 2;
  if (this->use_trajectory_builder_3d() != 0) {
    total_size += 1 + 1;
  }

  // bool collate_by_trajectory = 5;
  if (this->collate_by_trajectory() != 0) {
    total_size += 1 + 1;
  }

  // int32 num_background_threads = 3;
  if (this->num_background_threads() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->num_background_threads());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void MapBuilderOptions::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.MapBuilderOptions)
  GOOGLE_DCHECK_NE(&from, this);
  const MapBuilderOptions* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const MapBuilderOptions>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.MapBuilderOptions)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.MapBuilderOptions)
    MergeFrom(*source);
  }
}

void MapBuilderOptions::MergeFrom(const MapBuilderOptions& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.MapBuilderOptions)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_pose_graph_options()) {
    mutable_pose_graph_options()->::csmlio::mapping::proto::PoseGraphOptions::MergeFrom(from.pose_graph_options());
  }
  if (from.use_trajectory_builder_2d() != 0) {
    set_use_trajectory_builder_2d(from.use_trajectory_builder_2d());
  }
  if (from.use_trajectory_builder_3d() != 0) {
    set_use_trajectory_builder_3d(from.use_trajectory_builder_3d());
  }
  if (from.collate_by_trajectory() != 0) {
    set_collate_by_trajectory(from.collate_by_trajectory());
  }
  if (from.num_background_threads() != 0) {
    set_num_background_threads(from.num_background_threads());
  }
}

void MapBuilderOptions::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.MapBuilderOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MapBuilderOptions::CopyFrom(const MapBuilderOptions& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.MapBuilderOptions)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MapBuilderOptions::IsInitialized() const {
  return true;
}

void MapBuilderOptions::Swap(MapBuilderOptions* other) {
  if (other == this) return;
  InternalSwap(other);
}
void MapBuilderOptions::InternalSwap(MapBuilderOptions* other) {
  using std::swap;
  swap(pose_graph_options_, other->pose_graph_options_);
  swap(use_trajectory_builder_2d_, other->use_trajectory_builder_2d_);
  swap(use_trajectory_builder_3d_, other->use_trajectory_builder_3d_);
  swap(collate_by_trajectory_, other->collate_by_trajectory_);
  swap(num_background_threads_, other->num_background_threads_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata MapBuilderOptions::GetMetadata() const {
  protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fmap_5fbuilder_5foptions_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// MapBuilderOptions

// bool use_trajectory_builder_2d = 1;
void MapBuilderOptions::clear_use_trajectory_builder_2d() {
  use_trajectory_builder_2d_ = false;
}
bool MapBuilderOptions::use_trajectory_builder_2d() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapBuilderOptions.use_trajectory_builder_2d)
  return use_trajectory_builder_2d_;
}
void MapBuilderOptions::set_use_trajectory_builder_2d(bool value) {
  
  use_trajectory_builder_2d_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.MapBuilderOptions.use_trajectory_builder_2d)
}

// bool use_trajectory_builder_3d = 2;
void MapBuilderOptions::clear_use_trajectory_builder_3d() {
  use_trajectory_builder_3d_ = false;
}
bool MapBuilderOptions::use_trajectory_builder_3d() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapBuilderOptions.use_trajectory_builder_3d)
  return use_trajectory_builder_3d_;
}
void MapBuilderOptions::set_use_trajectory_builder_3d(bool value) {
  
  use_trajectory_builder_3d_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.MapBuilderOptions.use_trajectory_builder_3d)
}

// int32 num_background_threads = 3;
void MapBuilderOptions::clear_num_background_threads() {
  num_background_threads_ = 0;
}
::google::protobuf::int32 MapBuilderOptions::num_background_threads() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapBuilderOptions.num_background_threads)
  return num_background_threads_;
}
void MapBuilderOptions::set_num_background_threads(::google::protobuf::int32 value) {
  
  num_background_threads_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.MapBuilderOptions.num_background_threads)
}

// .cartographer.mapping.proto.PoseGraphOptions pose_graph_options = 4;
bool MapBuilderOptions::has_pose_graph_options() const {
  return this != internal_default_instance() && pose_graph_options_ != NULL;
}
void MapBuilderOptions::clear_pose_graph_options() {
  if (GetArenaNoVirtual() == NULL && pose_graph_options_ != NULL) delete pose_graph_options_;
  pose_graph_options_ = NULL;
}
const ::csmlio::mapping::proto::PoseGraphOptions& MapBuilderOptions::pose_graph_options() const {
  const ::csmlio::mapping::proto::PoseGraphOptions* p = pose_graph_options_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapBuilderOptions.pose_graph_options)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::mapping::proto::PoseGraphOptions*>(
      &::csmlio::mapping::proto::_PoseGraphOptions_default_instance_);
}
::csmlio::mapping::proto::PoseGraphOptions* MapBuilderOptions::mutable_pose_graph_options() {
  
  if (pose_graph_options_ == NULL) {
    pose_graph_options_ = new ::csmlio::mapping::proto::PoseGraphOptions;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.MapBuilderOptions.pose_graph_options)
  return pose_graph_options_;
}
::csmlio::mapping::proto::PoseGraphOptions* MapBuilderOptions::release_pose_graph_options() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.MapBuilderOptions.pose_graph_options)
  
  ::csmlio::mapping::proto::PoseGraphOptions* temp = pose_graph_options_;
  pose_graph_options_ = NULL;
  return temp;
}
void MapBuilderOptions::set_allocated_pose_graph_options(::csmlio::mapping::proto::PoseGraphOptions* pose_graph_options) {
  delete pose_graph_options_;
  pose_graph_options_ = pose_graph_options;
  if (pose_graph_options) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.MapBuilderOptions.pose_graph_options)
}

// bool collate_by_trajectory = 5;
void MapBuilderOptions::clear_collate_by_trajectory() {
  collate_by_trajectory_ = false;
}
bool MapBuilderOptions::collate_by_trajectory() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapBuilderOptions.collate_by_trajectory)
  return collate_by_trajectory_;
}
void MapBuilderOptions::set_collate_by_trajectory(bool value) {
  
  collate_by_trajectory_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.MapBuilderOptions.collate_by_trajectory)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)
