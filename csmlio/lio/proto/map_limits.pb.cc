// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/map_limits.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "csmlio/lio/proto/map_limits.pb.h"

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
class MapLimitsDefaultTypeInternal {
public:
 ::google::protobuf::internal::ExplicitlyConstructed<MapLimits>
     _instance;
} _MapLimits_default_instance_;

namespace protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto {


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
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapLimits, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapLimits, resolution_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapLimits, max_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(MapLimits, cell_limits_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(MapLimits)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_MapLimits_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping/proto/map_limits.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
  ::csmlio::mapping::proto::protobuf_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto::InitDefaults();
  ::csmlio::transform::proto::protobuf_cartographer_2ftransform_2fproto_2ftransform_2eproto::InitDefaults();
  _MapLimits_default_instance_._instance.DefaultConstruct();
  ::google::protobuf::internal::OnShutdownDestroyMessage(
      &_MapLimits_default_instance_);_MapLimits_default_instance_._instance.get_mutable()->max_ = const_cast< ::csmlio::transform::proto::Vector2d*>(
      ::csmlio::transform::proto::Vector2d::internal_default_instance());
  _MapLimits_default_instance_._instance.get_mutable()->cell_limits_ = const_cast< ::csmlio::mapping::proto::CellLimits*>(
      ::csmlio::mapping::proto::CellLimits::internal_default_instance());
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
namespace {
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n+cartographer/mapping/proto/map_limits."
      "proto\022\032cartographer.mapping.proto\032/carto"
      "grapher/mapping/proto/cell_limits_2d.pro"
      "to\032,cartographer/transform/proto/transfo"
      "rm.proto\"\221\001\n\tMapLimits\022\022\n\nresolution\030\001 \001"
      "(\001\0223\n\003max\030\002 \001(\0132&.cartographer.transform"
      ".proto.Vector2d\022;\n\013cell_limits\030\003 \001(\0132&.c"
      "artographer.mapping.proto.CellLimitsb\006pr"
      "oto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 324);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping/proto/map_limits.proto", &protobuf_RegisterTypes);
  ::csmlio::mapping::proto::protobuf_cartographer_2fmapping_2fproto_2fcell_5flimits_5f2d_2eproto::AddDescriptors();
  ::csmlio::transform::proto::protobuf_cartographer_2ftransform_2fproto_2ftransform_2eproto::AddDescriptors();
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

}  // namespace protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int MapLimits::kResolutionFieldNumber;
const int MapLimits::kMaxFieldNumber;
const int MapLimits::kCellLimitsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

MapLimits::MapLimits()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.proto.MapLimits)
}
MapLimits::MapLimits(const MapLimits& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_max()) {
    max_ = new ::csmlio::transform::proto::Vector2d(*from.max_);
  } else {
    max_ = NULL;
  }
  if (from.has_cell_limits()) {
    cell_limits_ = new ::csmlio::mapping::proto::CellLimits(*from.cell_limits_);
  } else {
    cell_limits_ = NULL;
  }
  resolution_ = from.resolution_;
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.proto.MapLimits)
}

void MapLimits::SharedCtor() {
  ::memset(&max_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&resolution_) -
      reinterpret_cast<char*>(&max_)) + sizeof(resolution_));
  _cached_size_ = 0;
}

MapLimits::~MapLimits() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.proto.MapLimits)
  SharedDtor();
}

void MapLimits::SharedDtor() {
  if (this != internal_default_instance()) delete max_;
  if (this != internal_default_instance()) delete cell_limits_;
}

void MapLimits::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* MapLimits::descriptor() {
  protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const MapLimits& MapLimits::default_instance() {
  protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto::InitDefaults();
  return *internal_default_instance();
}

MapLimits* MapLimits::New(::google::protobuf::Arena* arena) const {
  MapLimits* n = new MapLimits;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void MapLimits::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.proto.MapLimits)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == NULL && max_ != NULL) {
    delete max_;
  }
  max_ = NULL;
  if (GetArenaNoVirtual() == NULL && cell_limits_ != NULL) {
    delete cell_limits_;
  }
  cell_limits_ = NULL;
  resolution_ = 0;
  _internal_metadata_.Clear();
}

bool MapLimits::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.proto.MapLimits)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double resolution = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &resolution_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .cartographer.transform.proto.Vector2d max = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_max()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .cartographer.mapping.proto.CellLimits cell_limits = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtual(
               input, mutable_cell_limits()));
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
  // @@protoc_insertion_point(parse_success:cartographer.mapping.proto.MapLimits)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.proto.MapLimits)
  return false;
#undef DO_
}

void MapLimits::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.proto.MapLimits)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double resolution = 1;
  if (this->resolution() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->resolution(), output);
  }

  // .cartographer.transform.proto.Vector2d max = 2;
  if (this->has_max()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, *this->max_, output);
  }

  // .cartographer.mapping.proto.CellLimits cell_limits = 3;
  if (this->has_cell_limits()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, *this->cell_limits_, output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.proto.MapLimits)
}

::google::protobuf::uint8* MapLimits::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.proto.MapLimits)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double resolution = 1;
  if (this->resolution() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->resolution(), target);
  }

  // .cartographer.transform.proto.Vector2d max = 2;
  if (this->has_max()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        2, *this->max_, deterministic, target);
  }

  // .cartographer.mapping.proto.CellLimits cell_limits = 3;
  if (this->has_cell_limits()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        3, *this->cell_limits_, deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.proto.MapLimits)
  return target;
}

size_t MapLimits::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.proto.MapLimits)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // .cartographer.transform.proto.Vector2d max = 2;
  if (this->has_max()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->max_);
  }

  // .cartographer.mapping.proto.CellLimits cell_limits = 3;
  if (this->has_cell_limits()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
        *this->cell_limits_);
  }

  // double resolution = 1;
  if (this->resolution() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void MapLimits::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.proto.MapLimits)
  GOOGLE_DCHECK_NE(&from, this);
  const MapLimits* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const MapLimits>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.proto.MapLimits)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.proto.MapLimits)
    MergeFrom(*source);
  }
}

void MapLimits::MergeFrom(const MapLimits& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.proto.MapLimits)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_max()) {
    mutable_max()->::csmlio::transform::proto::Vector2d::MergeFrom(from.max());
  }
  if (from.has_cell_limits()) {
    mutable_cell_limits()->::csmlio::mapping::proto::CellLimits::MergeFrom(from.cell_limits());
  }
  if (from.resolution() != 0) {
    set_resolution(from.resolution());
  }
}

void MapLimits::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.proto.MapLimits)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void MapLimits::CopyFrom(const MapLimits& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.proto.MapLimits)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MapLimits::IsInitialized() const {
  return true;
}

void MapLimits::Swap(MapLimits* other) {
  if (other == this) return;
  InternalSwap(other);
}
void MapLimits::InternalSwap(MapLimits* other) {
  using std::swap;
  swap(max_, other->max_);
  swap(cell_limits_, other->cell_limits_);
  swap(resolution_, other->resolution_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata MapLimits::GetMetadata() const {
  protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_cartographer_2fmapping_2fproto_2fmap_5flimits_2eproto::file_level_metadata[kIndexInFileMessages];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// MapLimits

// double resolution = 1;
void MapLimits::clear_resolution() {
  resolution_ = 0;
}
double MapLimits::resolution() const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapLimits.resolution)
  return resolution_;
}
void MapLimits::set_resolution(double value) {
  
  resolution_ = value;
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.MapLimits.resolution)
}

// .cartographer.transform.proto.Vector2d max = 2;
bool MapLimits::has_max() const {
  return this != internal_default_instance() && max_ != NULL;
}
void MapLimits::clear_max() {
  if (GetArenaNoVirtual() == NULL && max_ != NULL) delete max_;
  max_ = NULL;
}
const ::csmlio::transform::proto::Vector2d& MapLimits::max() const {
  const ::csmlio::transform::proto::Vector2d* p = max_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapLimits.max)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::transform::proto::Vector2d*>(
      &::csmlio::transform::proto::_Vector2d_default_instance_);
}
::csmlio::transform::proto::Vector2d* MapLimits::mutable_max() {
  
  if (max_ == NULL) {
    max_ = new ::csmlio::transform::proto::Vector2d;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.MapLimits.max)
  return max_;
}
::csmlio::transform::proto::Vector2d* MapLimits::release_max() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.MapLimits.max)
  
  ::csmlio::transform::proto::Vector2d* temp = max_;
  max_ = NULL;
  return temp;
}
void MapLimits::set_allocated_max(::csmlio::transform::proto::Vector2d* max) {
  delete max_;
  max_ = max;
  if (max) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.MapLimits.max)
}

// .cartographer.mapping.proto.CellLimits cell_limits = 3;
bool MapLimits::has_cell_limits() const {
  return this != internal_default_instance() && cell_limits_ != NULL;
}
void MapLimits::clear_cell_limits() {
  if (GetArenaNoVirtual() == NULL && cell_limits_ != NULL) delete cell_limits_;
  cell_limits_ = NULL;
}
const ::csmlio::mapping::proto::CellLimits& MapLimits::cell_limits() const {
  const ::csmlio::mapping::proto::CellLimits* p = cell_limits_;
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.MapLimits.cell_limits)
  return p != NULL ? *p : *reinterpret_cast<const ::csmlio::mapping::proto::CellLimits*>(
      &::csmlio::mapping::proto::_CellLimits_default_instance_);
}
::csmlio::mapping::proto::CellLimits* MapLimits::mutable_cell_limits() {
  
  if (cell_limits_ == NULL) {
    cell_limits_ = new ::csmlio::mapping::proto::CellLimits;
  }
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.MapLimits.cell_limits)
  return cell_limits_;
}
::csmlio::mapping::proto::CellLimits* MapLimits::release_cell_limits() {
  // @@protoc_insertion_point(field_release:cartographer.mapping.proto.MapLimits.cell_limits)
  
  ::csmlio::mapping::proto::CellLimits* temp = cell_limits_;
  cell_limits_ = NULL;
  return temp;
}
void MapLimits::set_allocated_cell_limits(::csmlio::mapping::proto::CellLimits* cell_limits) {
  delete cell_limits_;
  cell_limits_ = cell_limits;
  if (cell_limits) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:cartographer.mapping.proto.MapLimits.cell_limits)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)
