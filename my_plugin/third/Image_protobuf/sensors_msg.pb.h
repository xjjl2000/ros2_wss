// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sensors_msg.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_sensors_5fmsg_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_sensors_5fmsg_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3020000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3020003 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_sensors_5fmsg_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_sensors_5fmsg_2eproto {
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_sensors_5fmsg_2eproto;
namespace sensors_msg {
class ImageProto;
struct ImageProtoDefaultTypeInternal;
extern ImageProtoDefaultTypeInternal _ImageProto_default_instance_;
}  // namespace sensors_msg
PROTOBUF_NAMESPACE_OPEN
template<> ::sensors_msg::ImageProto* Arena::CreateMaybeMessage<::sensors_msg::ImageProto>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace sensors_msg {

// ===================================================================

class ImageProto final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:sensors_msg.ImageProto) */ {
 public:
  inline ImageProto() : ImageProto(nullptr) {}
  ~ImageProto() override;
  explicit PROTOBUF_CONSTEXPR ImageProto(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  ImageProto(const ImageProto& from);
  ImageProto(ImageProto&& from) noexcept
    : ImageProto() {
    *this = ::std::move(from);
  }

  inline ImageProto& operator=(const ImageProto& from) {
    CopyFrom(from);
    return *this;
  }
  inline ImageProto& operator=(ImageProto&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const ImageProto& default_instance() {
    return *internal_default_instance();
  }
  static inline const ImageProto* internal_default_instance() {
    return reinterpret_cast<const ImageProto*>(
               &_ImageProto_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ImageProto& a, ImageProto& b) {
    a.Swap(&b);
  }
  inline void Swap(ImageProto* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(ImageProto* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  ImageProto* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<ImageProto>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const ImageProto& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const ImageProto& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(ImageProto* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "sensors_msg.ImageProto";
  }
  protected:
  explicit ImageProto(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFrameIdFieldNumber = 3,
    kFormatFieldNumber = 7,
    kDataFieldNumber = 8,
    kHeaderStampSecFieldNumber = 1,
    kHeaderStampNanosecFieldNumber = 2,
    kHeightFieldNumber = 4,
    kWidthFieldNumber = 5,
    kStepFieldNumber = 6,
  };
  // string header_frame_id = 3;
  void clear_header_frame_id();
  const std::string& header_frame_id() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_header_frame_id(ArgT0&& arg0, ArgT... args);
  std::string* mutable_header_frame_id();
  PROTOBUF_NODISCARD std::string* release_header_frame_id();
  void set_allocated_header_frame_id(std::string* header_frame_id);
  private:
  const std::string& _internal_header_frame_id() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_header_frame_id(const std::string& value);
  std::string* _internal_mutable_header_frame_id();
  public:

  // string format = 7;
  void clear_format();
  const std::string& format() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_format(ArgT0&& arg0, ArgT... args);
  std::string* mutable_format();
  PROTOBUF_NODISCARD std::string* release_format();
  void set_allocated_format(std::string* format);
  private:
  const std::string& _internal_format() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_format(const std::string& value);
  std::string* _internal_mutable_format();
  public:

  // bytes data = 8;
  void clear_data();
  const std::string& data() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_data(ArgT0&& arg0, ArgT... args);
  std::string* mutable_data();
  PROTOBUF_NODISCARD std::string* release_data();
  void set_allocated_data(std::string* data);
  private:
  const std::string& _internal_data() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_data(const std::string& value);
  std::string* _internal_mutable_data();
  public:

  // int32 header_stamp_sec = 1;
  void clear_header_stamp_sec();
  int32_t header_stamp_sec() const;
  void set_header_stamp_sec(int32_t value);
  private:
  int32_t _internal_header_stamp_sec() const;
  void _internal_set_header_stamp_sec(int32_t value);
  public:

  // int32 header_stamp_nanosec = 2;
  void clear_header_stamp_nanosec();
  int32_t header_stamp_nanosec() const;
  void set_header_stamp_nanosec(int32_t value);
  private:
  int32_t _internal_header_stamp_nanosec() const;
  void _internal_set_header_stamp_nanosec(int32_t value);
  public:

  // uint32 height = 4;
  void clear_height();
  uint32_t height() const;
  void set_height(uint32_t value);
  private:
  uint32_t _internal_height() const;
  void _internal_set_height(uint32_t value);
  public:

  // uint32 width = 5;
  void clear_width();
  uint32_t width() const;
  void set_width(uint32_t value);
  private:
  uint32_t _internal_width() const;
  void _internal_set_width(uint32_t value);
  public:

  // uint32 step = 6;
  void clear_step();
  uint32_t step() const;
  void set_step(uint32_t value);
  private:
  uint32_t _internal_step() const;
  void _internal_set_step(uint32_t value);
  public:

  // @@protoc_insertion_point(class_scope:sensors_msg.ImageProto)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr header_frame_id_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr format_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr data_;
  int32_t header_stamp_sec_;
  int32_t header_stamp_nanosec_;
  uint32_t height_;
  uint32_t width_;
  uint32_t step_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  friend struct ::TableStruct_sensors_5fmsg_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ImageProto

// int32 header_stamp_sec = 1;
inline void ImageProto::clear_header_stamp_sec() {
  header_stamp_sec_ = 0;
}
inline int32_t ImageProto::_internal_header_stamp_sec() const {
  return header_stamp_sec_;
}
inline int32_t ImageProto::header_stamp_sec() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.header_stamp_sec)
  return _internal_header_stamp_sec();
}
inline void ImageProto::_internal_set_header_stamp_sec(int32_t value) {
  
  header_stamp_sec_ = value;
}
inline void ImageProto::set_header_stamp_sec(int32_t value) {
  _internal_set_header_stamp_sec(value);
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.header_stamp_sec)
}

// int32 header_stamp_nanosec = 2;
inline void ImageProto::clear_header_stamp_nanosec() {
  header_stamp_nanosec_ = 0;
}
inline int32_t ImageProto::_internal_header_stamp_nanosec() const {
  return header_stamp_nanosec_;
}
inline int32_t ImageProto::header_stamp_nanosec() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.header_stamp_nanosec)
  return _internal_header_stamp_nanosec();
}
inline void ImageProto::_internal_set_header_stamp_nanosec(int32_t value) {
  
  header_stamp_nanosec_ = value;
}
inline void ImageProto::set_header_stamp_nanosec(int32_t value) {
  _internal_set_header_stamp_nanosec(value);
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.header_stamp_nanosec)
}

// string header_frame_id = 3;
inline void ImageProto::clear_header_frame_id() {
  header_frame_id_.ClearToEmpty();
}
inline const std::string& ImageProto::header_frame_id() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.header_frame_id)
  return _internal_header_frame_id();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void ImageProto::set_header_frame_id(ArgT0&& arg0, ArgT... args) {
 
 header_frame_id_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.header_frame_id)
}
inline std::string* ImageProto::mutable_header_frame_id() {
  std::string* _s = _internal_mutable_header_frame_id();
  // @@protoc_insertion_point(field_mutable:sensors_msg.ImageProto.header_frame_id)
  return _s;
}
inline const std::string& ImageProto::_internal_header_frame_id() const {
  return header_frame_id_.Get();
}
inline void ImageProto::_internal_set_header_frame_id(const std::string& value) {
  
  header_frame_id_.Set(value, GetArenaForAllocation());
}
inline std::string* ImageProto::_internal_mutable_header_frame_id() {
  
  return header_frame_id_.Mutable(GetArenaForAllocation());
}
inline std::string* ImageProto::release_header_frame_id() {
  // @@protoc_insertion_point(field_release:sensors_msg.ImageProto.header_frame_id)
  return header_frame_id_.Release();
}
inline void ImageProto::set_allocated_header_frame_id(std::string* header_frame_id) {
  if (header_frame_id != nullptr) {
    
  } else {
    
  }
  header_frame_id_.SetAllocated(header_frame_id, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (header_frame_id_.IsDefault()) {
    header_frame_id_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:sensors_msg.ImageProto.header_frame_id)
}

// uint32 height = 4;
inline void ImageProto::clear_height() {
  height_ = 0u;
}
inline uint32_t ImageProto::_internal_height() const {
  return height_;
}
inline uint32_t ImageProto::height() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.height)
  return _internal_height();
}
inline void ImageProto::_internal_set_height(uint32_t value) {
  
  height_ = value;
}
inline void ImageProto::set_height(uint32_t value) {
  _internal_set_height(value);
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.height)
}

// uint32 width = 5;
inline void ImageProto::clear_width() {
  width_ = 0u;
}
inline uint32_t ImageProto::_internal_width() const {
  return width_;
}
inline uint32_t ImageProto::width() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.width)
  return _internal_width();
}
inline void ImageProto::_internal_set_width(uint32_t value) {
  
  width_ = value;
}
inline void ImageProto::set_width(uint32_t value) {
  _internal_set_width(value);
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.width)
}

// uint32 step = 6;
inline void ImageProto::clear_step() {
  step_ = 0u;
}
inline uint32_t ImageProto::_internal_step() const {
  return step_;
}
inline uint32_t ImageProto::step() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.step)
  return _internal_step();
}
inline void ImageProto::_internal_set_step(uint32_t value) {
  
  step_ = value;
}
inline void ImageProto::set_step(uint32_t value) {
  _internal_set_step(value);
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.step)
}

// string format = 7;
inline void ImageProto::clear_format() {
  format_.ClearToEmpty();
}
inline const std::string& ImageProto::format() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.format)
  return _internal_format();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void ImageProto::set_format(ArgT0&& arg0, ArgT... args) {
 
 format_.Set(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.format)
}
inline std::string* ImageProto::mutable_format() {
  std::string* _s = _internal_mutable_format();
  // @@protoc_insertion_point(field_mutable:sensors_msg.ImageProto.format)
  return _s;
}
inline const std::string& ImageProto::_internal_format() const {
  return format_.Get();
}
inline void ImageProto::_internal_set_format(const std::string& value) {
  
  format_.Set(value, GetArenaForAllocation());
}
inline std::string* ImageProto::_internal_mutable_format() {
  
  return format_.Mutable(GetArenaForAllocation());
}
inline std::string* ImageProto::release_format() {
  // @@protoc_insertion_point(field_release:sensors_msg.ImageProto.format)
  return format_.Release();
}
inline void ImageProto::set_allocated_format(std::string* format) {
  if (format != nullptr) {
    
  } else {
    
  }
  format_.SetAllocated(format, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (format_.IsDefault()) {
    format_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:sensors_msg.ImageProto.format)
}

// bytes data = 8;
inline void ImageProto::clear_data() {
  data_.ClearToEmpty();
}
inline const std::string& ImageProto::data() const {
  // @@protoc_insertion_point(field_get:sensors_msg.ImageProto.data)
  return _internal_data();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void ImageProto::set_data(ArgT0&& arg0, ArgT... args) {
 
 data_.SetBytes(static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:sensors_msg.ImageProto.data)
}
inline std::string* ImageProto::mutable_data() {
  std::string* _s = _internal_mutable_data();
  // @@protoc_insertion_point(field_mutable:sensors_msg.ImageProto.data)
  return _s;
}
inline const std::string& ImageProto::_internal_data() const {
  return data_.Get();
}
inline void ImageProto::_internal_set_data(const std::string& value) {
  
  data_.Set(value, GetArenaForAllocation());
}
inline std::string* ImageProto::_internal_mutable_data() {
  
  return data_.Mutable(GetArenaForAllocation());
}
inline std::string* ImageProto::release_data() {
  // @@protoc_insertion_point(field_release:sensors_msg.ImageProto.data)
  return data_.Release();
}
inline void ImageProto::set_allocated_data(std::string* data) {
  if (data != nullptr) {
    
  } else {
    
  }
  data_.SetAllocated(data, GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (data_.IsDefault()) {
    data_.Set("", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:sensors_msg.ImageProto.data)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace sensors_msg

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_sensors_5fmsg_2eproto
