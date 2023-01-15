// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/connected_components.proto

#ifndef PROTOBUF_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto__INCLUDED
#define PROTOBUF_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto__INCLUDED

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
namespace mapping {
namespace proto {
class ConnectedComponents;
class ConnectedComponentsDefaultTypeInternal;
extern ConnectedComponentsDefaultTypeInternal _ConnectedComponents_default_instance_;
class ConnectedComponents_ConnectedComponent;
class ConnectedComponents_ConnectedComponentDefaultTypeInternal;
extern ConnectedComponents_ConnectedComponentDefaultTypeInternal _ConnectedComponents_ConnectedComponent_default_instance_;
}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

namespace csmlio {
namespace mapping {
namespace proto {

namespace protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto {
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
}  // namespace protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto

// ===================================================================

class ConnectedComponents_ConnectedComponent : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent) */ {
 public:
  ConnectedComponents_ConnectedComponent();
  virtual ~ConnectedComponents_ConnectedComponent();

  ConnectedComponents_ConnectedComponent(const ConnectedComponents_ConnectedComponent& from);

  inline ConnectedComponents_ConnectedComponent& operator=(const ConnectedComponents_ConnectedComponent& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ConnectedComponents_ConnectedComponent(ConnectedComponents_ConnectedComponent&& from) noexcept
    : ConnectedComponents_ConnectedComponent() {
    *this = ::std::move(from);
  }

  inline ConnectedComponents_ConnectedComponent& operator=(ConnectedComponents_ConnectedComponent&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const ConnectedComponents_ConnectedComponent& default_instance();

  static inline const ConnectedComponents_ConnectedComponent* internal_default_instance() {
    return reinterpret_cast<const ConnectedComponents_ConnectedComponent*>(
               &_ConnectedComponents_ConnectedComponent_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(ConnectedComponents_ConnectedComponent* other);
  friend void swap(ConnectedComponents_ConnectedComponent& a, ConnectedComponents_ConnectedComponent& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ConnectedComponents_ConnectedComponent* New() const PROTOBUF_FINAL { return New(NULL); }

  ConnectedComponents_ConnectedComponent* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ConnectedComponents_ConnectedComponent& from);
  void MergeFrom(const ConnectedComponents_ConnectedComponent& from);
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
  void InternalSwap(ConnectedComponents_ConnectedComponent* other);
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

  // repeated int32 trajectory_id = 1;
  int trajectory_id_size() const;
  void clear_trajectory_id();
  static const int kTrajectoryIdFieldNumber = 1;
  ::google::protobuf::int32 trajectory_id(int index) const;
  void set_trajectory_id(int index, ::google::protobuf::int32 value);
  void add_trajectory_id(::google::protobuf::int32 value);
  const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
      trajectory_id() const;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
      mutable_trajectory_id();

  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< ::google::protobuf::int32 > trajectory_id_;
  mutable int _trajectory_id_cached_byte_size_;
  mutable int _cached_size_;
  friend struct protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class ConnectedComponents : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:cartographer.mapping.proto.ConnectedComponents) */ {
 public:
  ConnectedComponents();
  virtual ~ConnectedComponents();

  ConnectedComponents(const ConnectedComponents& from);

  inline ConnectedComponents& operator=(const ConnectedComponents& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  ConnectedComponents(ConnectedComponents&& from) noexcept
    : ConnectedComponents() {
    *this = ::std::move(from);
  }

  inline ConnectedComponents& operator=(ConnectedComponents&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const ConnectedComponents& default_instance();

  static inline const ConnectedComponents* internal_default_instance() {
    return reinterpret_cast<const ConnectedComponents*>(
               &_ConnectedComponents_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(ConnectedComponents* other);
  friend void swap(ConnectedComponents& a, ConnectedComponents& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline ConnectedComponents* New() const PROTOBUF_FINAL { return New(NULL); }

  ConnectedComponents* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const ConnectedComponents& from);
  void MergeFrom(const ConnectedComponents& from);
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
  void InternalSwap(ConnectedComponents* other);
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

  typedef ConnectedComponents_ConnectedComponent ConnectedComponent;

  // accessors -------------------------------------------------------

  // repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
  int connected_component_size() const;
  void clear_connected_component();
  static const int kConnectedComponentFieldNumber = 1;
  const ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent& connected_component(int index) const;
  ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent* mutable_connected_component(int index);
  ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent* add_connected_component();
  ::google::protobuf::RepeatedPtrField< ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent >*
      mutable_connected_component();
  const ::google::protobuf::RepeatedPtrField< ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent >&
      connected_component() const;

  // @@protoc_insertion_point(class_scope:cartographer.mapping.proto.ConnectedComponents)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent > connected_component_;
  mutable int _cached_size_;
  friend struct protobuf_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ConnectedComponents_ConnectedComponent

// repeated int32 trajectory_id = 1;
inline int ConnectedComponents_ConnectedComponent::trajectory_id_size() const {
  return trajectory_id_.size();
}
inline void ConnectedComponents_ConnectedComponent::clear_trajectory_id() {
  trajectory_id_.Clear();
}
inline ::google::protobuf::int32 ConnectedComponents_ConnectedComponent::trajectory_id(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
  return trajectory_id_.Get(index);
}
inline void ConnectedComponents_ConnectedComponent::set_trajectory_id(int index, ::google::protobuf::int32 value) {
  trajectory_id_.Set(index, value);
  // @@protoc_insertion_point(field_set:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
}
inline void ConnectedComponents_ConnectedComponent::add_trajectory_id(::google::protobuf::int32 value) {
  trajectory_id_.Add(value);
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
}
inline const ::google::protobuf::RepeatedField< ::google::protobuf::int32 >&
ConnectedComponents_ConnectedComponent::trajectory_id() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
  return trajectory_id_;
}
inline ::google::protobuf::RepeatedField< ::google::protobuf::int32 >*
ConnectedComponents_ConnectedComponent::mutable_trajectory_id() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.ConnectedComponents.ConnectedComponent.trajectory_id)
  return &trajectory_id_;
}

// -------------------------------------------------------------------

// ConnectedComponents

// repeated .cartographer.mapping.proto.ConnectedComponents.ConnectedComponent connected_component = 1;
inline int ConnectedComponents::connected_component_size() const {
  return connected_component_.size();
}
inline void ConnectedComponents::clear_connected_component() {
  connected_component_.Clear();
}
inline const ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent& ConnectedComponents::connected_component(int index) const {
  // @@protoc_insertion_point(field_get:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_.Get(index);
}
inline ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent* ConnectedComponents::mutable_connected_component(int index) {
  // @@protoc_insertion_point(field_mutable:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_.Mutable(index);
}
inline ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent* ConnectedComponents::add_connected_component() {
  // @@protoc_insertion_point(field_add:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent >*
ConnectedComponents::mutable_connected_component() {
  // @@protoc_insertion_point(field_mutable_list:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return &connected_component_;
}
inline const ::google::protobuf::RepeatedPtrField< ::csmlio::mapping::proto::ConnectedComponents_ConnectedComponent >&
ConnectedComponents::connected_component() const {
  // @@protoc_insertion_point(field_list:cartographer.mapping.proto.ConnectedComponents.connected_component)
  return connected_component_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)


}  // namespace proto
}  // namespace mapping
}  // namespace csmlio

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_cartographer_2fmapping_2fproto_2fconnected_5fcomponents_2eproto__INCLUDED
