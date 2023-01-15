// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/sensor/proto/adaptive_voxel_filter_options.proto

#ifndef PROTOBUF_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3004000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3004000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace csmlio {
namespace sensor {
namespace proto {
class AdaptiveVoxelFilterOptions;
class AdaptiveVoxelFilterOptionsDefaultTypeInternal;
extern AdaptiveVoxelFilterOptionsDefaultTypeInternal _AdaptiveVoxelFilterOptions_default_instance_;
}  // namespace proto
}  // namespace sensor
}  // namespace csmlio

namespace csmlio {
namespace sensor {
namespace proto {

namespace protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[];
  static const ::google::protobuf::uint32 offsets[];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static void InitDefaultsImpl();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto

// ===================================================================

class AdaptiveVoxelFilterOptions : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.sensor.proto.AdaptiveVoxelFilterOptions) */ {
 public:
  AdaptiveVoxelFilterOptions();
  virtual ~AdaptiveVoxelFilterOptions();

  AdaptiveVoxelFilterOptions(const AdaptiveVoxelFilterOptions& from);

  inline AdaptiveVoxelFilterOptions& operator=(const AdaptiveVoxelFilterOptions& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  AdaptiveVoxelFilterOptions(AdaptiveVoxelFilterOptions&& from) noexcept
    : AdaptiveVoxelFilterOptions() {
    *this = ::std::move(from);
  }

  inline AdaptiveVoxelFilterOptions& operator=(AdaptiveVoxelFilterOptions&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const AdaptiveVoxelFilterOptions& default_instance();

  static inline const AdaptiveVoxelFilterOptions* internal_default_instance() {
    return reinterpret_cast<const AdaptiveVoxelFilterOptions*>(
               &_AdaptiveVoxelFilterOptions_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(AdaptiveVoxelFilterOptions* other);
  friend void swap(AdaptiveVoxelFilterOptions& a, AdaptiveVoxelFilterOptions& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline AdaptiveVoxelFilterOptions* New() const PROTOBUF_FINAL { return New(NULL); }

  AdaptiveVoxelFilterOptions* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const AdaptiveVoxelFilterOptions& from);
  void MergeFrom(const AdaptiveVoxelFilterOptions& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(AdaptiveVoxelFilterOptions* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // float max_length = 1;
  void clear_max_length();
  static const int kMaxLengthFieldNumber = 1;
  float max_length() const;
  void set_max_length(float value);

  // float min_num_points = 2;
  void clear_min_num_points();
  static const int kMinNumPointsFieldNumber = 2;
  float min_num_points() const;
  void set_min_num_points(float value);

  // float max_range = 3;
  void clear_max_range();
  static const int kMaxRangeFieldNumber = 3;
  float max_range() const;
  void set_max_range(float value);

  // @@protoc_insertion_point(class_scope:cartographer.sensor.proto.AdaptiveVoxelFilterOptions)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  float max_length_;
  float min_num_points_;
  float max_range_;
  mutable int _cached_size_;
  friend struct protobuf_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// AdaptiveVoxelFilterOptions

// float max_length = 1;
inline void AdaptiveVoxelFilterOptions::clear_max_length() {
  max_length_ = 0;
}
inline float AdaptiveVoxelFilterOptions::max_length() const {
  // @@protoc_insertion_point(field_get:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_length)
  return max_length_;
}
inline void AdaptiveVoxelFilterOptions::set_max_length(float value) {
  
  max_length_ = value;
  // @@protoc_insertion_point(field_set:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_length)
}

// float min_num_points = 2;
inline void AdaptiveVoxelFilterOptions::clear_min_num_points() {
  min_num_points_ = 0;
}
inline float AdaptiveVoxelFilterOptions::min_num_points() const {
  // @@protoc_insertion_point(field_get:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.min_num_points)
  return min_num_points_;
}
inline void AdaptiveVoxelFilterOptions::set_min_num_points(float value) {
  
  min_num_points_ = value;
  // @@protoc_insertion_point(field_set:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.min_num_points)
}

// float max_range = 3;
inline void AdaptiveVoxelFilterOptions::clear_max_range() {
  max_range_ = 0;
}
inline float AdaptiveVoxelFilterOptions::max_range() const {
  // @@protoc_insertion_point(field_get:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_range)
  return max_range_;
}
inline void AdaptiveVoxelFilterOptions::set_max_range(float value) {
  
  max_range_ = value;
  // @@protoc_insertion_point(field_set:cartographer.sensor.proto.AdaptiveVoxelFilterOptions.max_range)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace proto
}  // namespace sensor
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fsensor_2fproto_2fadaptive_5fvoxel_5ffilter_5foptions_2eproto__INCLUDED
