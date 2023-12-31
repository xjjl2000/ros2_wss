// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: Imu_msgs.proto

#include "Imu_msgs.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace Imu_msgs {
PROTOBUF_CONSTEXPR Imu::Imu(
    ::_pbi::ConstantInitialized)
  : angular_velocity_covariance_()
  , linear_acceleration_covariance_()
  , orientation_covariance_()
  , header_(nullptr)
  , linear_acceleration_(nullptr)
  , angular_velocity_(nullptr)
  , orientation_(nullptr){}
struct ImuDefaultTypeInternal {
  PROTOBUF_CONSTEXPR ImuDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~ImuDefaultTypeInternal() {}
  union {
    Imu _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 ImuDefaultTypeInternal _Imu_default_instance_;
PROTOBUF_CONSTEXPR MagneticField::MagneticField(
    ::_pbi::ConstantInitialized)
  : header_(nullptr)
  , magnetic_field_(nullptr)
  , magnetic_field_covariance_(nullptr){}
struct MagneticFieldDefaultTypeInternal {
  PROTOBUF_CONSTEXPR MagneticFieldDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~MagneticFieldDefaultTypeInternal() {}
  union {
    MagneticField _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 MagneticFieldDefaultTypeInternal _MagneticField_default_instance_;
}  // namespace Imu_msgs
static ::_pb::Metadata file_level_metadata_Imu_5fmsgs_2eproto[2];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_Imu_5fmsgs_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_Imu_5fmsgs_2eproto = nullptr;

const uint32_t TableStruct_Imu_5fmsgs_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, header_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, linear_acceleration_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, angular_velocity_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, orientation_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, angular_velocity_covariance_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, linear_acceleration_covariance_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::Imu, orientation_covariance_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::MagneticField, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::MagneticField, header_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::MagneticField, magnetic_field_),
  PROTOBUF_FIELD_OFFSET(::Imu_msgs::MagneticField, magnetic_field_covariance_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::Imu_msgs::Imu)},
  { 13, -1, -1, sizeof(::Imu_msgs::MagneticField)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::Imu_msgs::_Imu_default_instance_._instance,
  &::Imu_msgs::_MagneticField_default_instance_._instance,
};

const char descriptor_table_protodef_Imu_5fmsgs_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\016Imu_msgs.proto\022\010Imu_msgs\032\025common_geome"
  "try.proto\032\023common_header.proto\"\224\002\n\003Imu\022\036"
  "\n\006header\030\001 \001(\0132\016.common.Header\022,\n\023linear"
  "_acceleration\030\002 \001(\0132\017.common.Point3D\022)\n\020"
  "angular_velocity\030\003 \001(\0132\017.common.Point3D\022"
  "\'\n\013orientation\030\004 \001(\0132\022.common.Quaternion"
  "\022#\n\033angular_velocity_covariance\030\005 \003(\001\022&\n"
  "\036linear_acceleration_covariance\030\006 \003(\001\022\036\n"
  "\026orientation_covariance\030\007 \003(\001\"\217\001\n\rMagnet"
  "icField\022\036\n\006header\030\001 \001(\0132\016.common.Header\022"
  "\'\n\016magnetic_field\030\002 \001(\0132\017.common.Point3D"
  "\0225\n\031magnetic_field_covariance\030\003 \001(\0132\022.co"
  "mmon.Quaternionb\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_Imu_5fmsgs_2eproto_deps[2] = {
  &::descriptor_table_common_5fgeometry_2eproto,
  &::descriptor_table_common_5fheader_2eproto,
};
static ::_pbi::once_flag descriptor_table_Imu_5fmsgs_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_Imu_5fmsgs_2eproto = {
    false, false, 503, descriptor_table_protodef_Imu_5fmsgs_2eproto,
    "Imu_msgs.proto",
    &descriptor_table_Imu_5fmsgs_2eproto_once, descriptor_table_Imu_5fmsgs_2eproto_deps, 2, 2,
    schemas, file_default_instances, TableStruct_Imu_5fmsgs_2eproto::offsets,
    file_level_metadata_Imu_5fmsgs_2eproto, file_level_enum_descriptors_Imu_5fmsgs_2eproto,
    file_level_service_descriptors_Imu_5fmsgs_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_Imu_5fmsgs_2eproto_getter() {
  return &descriptor_table_Imu_5fmsgs_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_Imu_5fmsgs_2eproto(&descriptor_table_Imu_5fmsgs_2eproto);
namespace Imu_msgs {

// ===================================================================

class Imu::_Internal {
 public:
  static const ::common::Header& header(const Imu* msg);
  static const ::common::Point3D& linear_acceleration(const Imu* msg);
  static const ::common::Point3D& angular_velocity(const Imu* msg);
  static const ::common::Quaternion& orientation(const Imu* msg);
};

const ::common::Header&
Imu::_Internal::header(const Imu* msg) {
  return *msg->header_;
}
const ::common::Point3D&
Imu::_Internal::linear_acceleration(const Imu* msg) {
  return *msg->linear_acceleration_;
}
const ::common::Point3D&
Imu::_Internal::angular_velocity(const Imu* msg) {
  return *msg->angular_velocity_;
}
const ::common::Quaternion&
Imu::_Internal::orientation(const Imu* msg) {
  return *msg->orientation_;
}
void Imu::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void Imu::clear_linear_acceleration() {
  if (GetArenaForAllocation() == nullptr && linear_acceleration_ != nullptr) {
    delete linear_acceleration_;
  }
  linear_acceleration_ = nullptr;
}
void Imu::clear_angular_velocity() {
  if (GetArenaForAllocation() == nullptr && angular_velocity_ != nullptr) {
    delete angular_velocity_;
  }
  angular_velocity_ = nullptr;
}
void Imu::clear_orientation() {
  if (GetArenaForAllocation() == nullptr && orientation_ != nullptr) {
    delete orientation_;
  }
  orientation_ = nullptr;
}
Imu::Imu(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  angular_velocity_covariance_(arena),
  linear_acceleration_covariance_(arena),
  orientation_covariance_(arena) {
  SharedCtor();
  // @@protoc_insertion_point(arena_constructor:Imu_msgs.Imu)
}
Imu::Imu(const Imu& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      angular_velocity_covariance_(from.angular_velocity_covariance_),
      linear_acceleration_covariance_(from.linear_acceleration_covariance_),
      orientation_covariance_(from.orientation_covariance_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_linear_acceleration()) {
    linear_acceleration_ = new ::common::Point3D(*from.linear_acceleration_);
  } else {
    linear_acceleration_ = nullptr;
  }
  if (from._internal_has_angular_velocity()) {
    angular_velocity_ = new ::common::Point3D(*from.angular_velocity_);
  } else {
    angular_velocity_ = nullptr;
  }
  if (from._internal_has_orientation()) {
    orientation_ = new ::common::Quaternion(*from.orientation_);
  } else {
    orientation_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:Imu_msgs.Imu)
}

inline void Imu::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&orientation_) -
    reinterpret_cast<char*>(&header_)) + sizeof(orientation_));
}

Imu::~Imu() {
  // @@protoc_insertion_point(destructor:Imu_msgs.Imu)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void Imu::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete linear_acceleration_;
  if (this != internal_default_instance()) delete angular_velocity_;
  if (this != internal_default_instance()) delete orientation_;
}

void Imu::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Imu::Clear() {
// @@protoc_insertion_point(message_clear_start:Imu_msgs.Imu)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  angular_velocity_covariance_.Clear();
  linear_acceleration_covariance_.Clear();
  orientation_covariance_.Clear();
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && linear_acceleration_ != nullptr) {
    delete linear_acceleration_;
  }
  linear_acceleration_ = nullptr;
  if (GetArenaForAllocation() == nullptr && angular_velocity_ != nullptr) {
    delete angular_velocity_;
  }
  angular_velocity_ = nullptr;
  if (GetArenaForAllocation() == nullptr && orientation_ != nullptr) {
    delete orientation_;
  }
  orientation_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Imu::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .common.Point3D linear_acceleration = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_linear_acceleration(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .common.Point3D angular_velocity = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_angular_velocity(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .common.Quaternion orientation = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 34)) {
          ptr = ctx->ParseMessage(_internal_mutable_orientation(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated double angular_velocity_covariance = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 42)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_angular_velocity_covariance(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 41) {
          _internal_add_angular_velocity_covariance(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // repeated double linear_acceleration_covariance = 6;
      case 6:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 50)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_linear_acceleration_covariance(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 49) {
          _internal_add_linear_acceleration_covariance(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      // repeated double orientation_covariance = 7;
      case 7:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 58)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_orientation_covariance(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 57) {
          _internal_add_orientation_covariance(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Imu::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:Imu_msgs.Imu)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .common.Header header = 1;
  if (this->_internal_has_header()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::header(this),
        _Internal::header(this).GetCachedSize(), target, stream);
  }

  // .common.Point3D linear_acceleration = 2;
  if (this->_internal_has_linear_acceleration()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::linear_acceleration(this),
        _Internal::linear_acceleration(this).GetCachedSize(), target, stream);
  }

  // .common.Point3D angular_velocity = 3;
  if (this->_internal_has_angular_velocity()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, _Internal::angular_velocity(this),
        _Internal::angular_velocity(this).GetCachedSize(), target, stream);
  }

  // .common.Quaternion orientation = 4;
  if (this->_internal_has_orientation()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(4, _Internal::orientation(this),
        _Internal::orientation(this).GetCachedSize(), target, stream);
  }

  // repeated double angular_velocity_covariance = 5;
  if (this->_internal_angular_velocity_covariance_size() > 0) {
    target = stream->WriteFixedPacked(5, _internal_angular_velocity_covariance(), target);
  }

  // repeated double linear_acceleration_covariance = 6;
  if (this->_internal_linear_acceleration_covariance_size() > 0) {
    target = stream->WriteFixedPacked(6, _internal_linear_acceleration_covariance(), target);
  }

  // repeated double orientation_covariance = 7;
  if (this->_internal_orientation_covariance_size() > 0) {
    target = stream->WriteFixedPacked(7, _internal_orientation_covariance(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Imu_msgs.Imu)
  return target;
}

size_t Imu::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Imu_msgs.Imu)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double angular_velocity_covariance = 5;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_angular_velocity_covariance_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::_pbi::WireFormatLite::Int32Size(static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // repeated double linear_acceleration_covariance = 6;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_linear_acceleration_covariance_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::_pbi::WireFormatLite::Int32Size(static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // repeated double orientation_covariance = 7;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_orientation_covariance_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::_pbi::WireFormatLite::Int32Size(static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // .common.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .common.Point3D linear_acceleration = 2;
  if (this->_internal_has_linear_acceleration()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *linear_acceleration_);
  }

  // .common.Point3D angular_velocity = 3;
  if (this->_internal_has_angular_velocity()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *angular_velocity_);
  }

  // .common.Quaternion orientation = 4;
  if (this->_internal_has_orientation()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *orientation_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Imu::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Imu::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Imu::GetClassData() const { return &_class_data_; }

void Imu::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Imu *>(to)->MergeFrom(
      static_cast<const Imu &>(from));
}


void Imu::MergeFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Imu_msgs.Imu)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  angular_velocity_covariance_.MergeFrom(from.angular_velocity_covariance_);
  linear_acceleration_covariance_.MergeFrom(from.linear_acceleration_covariance_);
  orientation_covariance_.MergeFrom(from.orientation_covariance_);
  if (from._internal_has_header()) {
    _internal_mutable_header()->::common::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_linear_acceleration()) {
    _internal_mutable_linear_acceleration()->::common::Point3D::MergeFrom(from._internal_linear_acceleration());
  }
  if (from._internal_has_angular_velocity()) {
    _internal_mutable_angular_velocity()->::common::Point3D::MergeFrom(from._internal_angular_velocity());
  }
  if (from._internal_has_orientation()) {
    _internal_mutable_orientation()->::common::Quaternion::MergeFrom(from._internal_orientation());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Imu::CopyFrom(const Imu& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Imu_msgs.Imu)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Imu::IsInitialized() const {
  return true;
}

void Imu::InternalSwap(Imu* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  angular_velocity_covariance_.InternalSwap(&other->angular_velocity_covariance_);
  linear_acceleration_covariance_.InternalSwap(&other->linear_acceleration_covariance_);
  orientation_covariance_.InternalSwap(&other->orientation_covariance_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Imu, orientation_)
      + sizeof(Imu::orientation_)
      - PROTOBUF_FIELD_OFFSET(Imu, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Imu::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_Imu_5fmsgs_2eproto_getter, &descriptor_table_Imu_5fmsgs_2eproto_once,
      file_level_metadata_Imu_5fmsgs_2eproto[0]);
}

// ===================================================================

class MagneticField::_Internal {
 public:
  static const ::common::Header& header(const MagneticField* msg);
  static const ::common::Point3D& magnetic_field(const MagneticField* msg);
  static const ::common::Quaternion& magnetic_field_covariance(const MagneticField* msg);
};

const ::common::Header&
MagneticField::_Internal::header(const MagneticField* msg) {
  return *msg->header_;
}
const ::common::Point3D&
MagneticField::_Internal::magnetic_field(const MagneticField* msg) {
  return *msg->magnetic_field_;
}
const ::common::Quaternion&
MagneticField::_Internal::magnetic_field_covariance(const MagneticField* msg) {
  return *msg->magnetic_field_covariance_;
}
void MagneticField::clear_header() {
  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
}
void MagneticField::clear_magnetic_field() {
  if (GetArenaForAllocation() == nullptr && magnetic_field_ != nullptr) {
    delete magnetic_field_;
  }
  magnetic_field_ = nullptr;
}
void MagneticField::clear_magnetic_field_covariance() {
  if (GetArenaForAllocation() == nullptr && magnetic_field_covariance_ != nullptr) {
    delete magnetic_field_covariance_;
  }
  magnetic_field_covariance_ = nullptr;
}
MagneticField::MagneticField(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  // @@protoc_insertion_point(arena_constructor:Imu_msgs.MagneticField)
}
MagneticField::MagneticField(const MagneticField& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    header_ = new ::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from._internal_has_magnetic_field()) {
    magnetic_field_ = new ::common::Point3D(*from.magnetic_field_);
  } else {
    magnetic_field_ = nullptr;
  }
  if (from._internal_has_magnetic_field_covariance()) {
    magnetic_field_covariance_ = new ::common::Quaternion(*from.magnetic_field_covariance_);
  } else {
    magnetic_field_covariance_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:Imu_msgs.MagneticField)
}

inline void MagneticField::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&header_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&magnetic_field_covariance_) -
    reinterpret_cast<char*>(&header_)) + sizeof(magnetic_field_covariance_));
}

MagneticField::~MagneticField() {
  // @@protoc_insertion_point(destructor:Imu_msgs.MagneticField)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void MagneticField::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete magnetic_field_;
  if (this != internal_default_instance()) delete magnetic_field_covariance_;
}

void MagneticField::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void MagneticField::Clear() {
// @@protoc_insertion_point(message_clear_start:Imu_msgs.MagneticField)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaForAllocation() == nullptr && header_ != nullptr) {
    delete header_;
  }
  header_ = nullptr;
  if (GetArenaForAllocation() == nullptr && magnetic_field_ != nullptr) {
    delete magnetic_field_;
  }
  magnetic_field_ = nullptr;
  if (GetArenaForAllocation() == nullptr && magnetic_field_covariance_ != nullptr) {
    delete magnetic_field_covariance_;
  }
  magnetic_field_covariance_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* MagneticField::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .common.Point3D magnetic_field = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_magnetic_field(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .common.Quaternion magnetic_field_covariance = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_magnetic_field_covariance(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* MagneticField::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:Imu_msgs.MagneticField)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .common.Header header = 1;
  if (this->_internal_has_header()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::header(this),
        _Internal::header(this).GetCachedSize(), target, stream);
  }

  // .common.Point3D magnetic_field = 2;
  if (this->_internal_has_magnetic_field()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::magnetic_field(this),
        _Internal::magnetic_field(this).GetCachedSize(), target, stream);
  }

  // .common.Quaternion magnetic_field_covariance = 3;
  if (this->_internal_has_magnetic_field_covariance()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, _Internal::magnetic_field_covariance(this),
        _Internal::magnetic_field_covariance(this).GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:Imu_msgs.MagneticField)
  return target;
}

size_t MagneticField::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:Imu_msgs.MagneticField)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // .common.Header header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *header_);
  }

  // .common.Point3D magnetic_field = 2;
  if (this->_internal_has_magnetic_field()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *magnetic_field_);
  }

  // .common.Quaternion magnetic_field_covariance = 3;
  if (this->_internal_has_magnetic_field_covariance()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *magnetic_field_covariance_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData MagneticField::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    MagneticField::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*MagneticField::GetClassData() const { return &_class_data_; }

void MagneticField::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<MagneticField *>(to)->MergeFrom(
      static_cast<const MagneticField &>(from));
}


void MagneticField::MergeFrom(const MagneticField& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:Imu_msgs.MagneticField)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (from._internal_has_header()) {
    _internal_mutable_header()->::common::Header::MergeFrom(from._internal_header());
  }
  if (from._internal_has_magnetic_field()) {
    _internal_mutable_magnetic_field()->::common::Point3D::MergeFrom(from._internal_magnetic_field());
  }
  if (from._internal_has_magnetic_field_covariance()) {
    _internal_mutable_magnetic_field_covariance()->::common::Quaternion::MergeFrom(from._internal_magnetic_field_covariance());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void MagneticField::CopyFrom(const MagneticField& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:Imu_msgs.MagneticField)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool MagneticField::IsInitialized() const {
  return true;
}

void MagneticField::InternalSwap(MagneticField* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(MagneticField, magnetic_field_covariance_)
      + sizeof(MagneticField::magnetic_field_covariance_)
      - PROTOBUF_FIELD_OFFSET(MagneticField, header_)>(
          reinterpret_cast<char*>(&header_),
          reinterpret_cast<char*>(&other->header_));
}

::PROTOBUF_NAMESPACE_ID::Metadata MagneticField::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_Imu_5fmsgs_2eproto_getter, &descriptor_table_Imu_5fmsgs_2eproto_once,
      file_level_metadata_Imu_5fmsgs_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace Imu_msgs
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::Imu_msgs::Imu*
Arena::CreateMaybeMessage< ::Imu_msgs::Imu >(Arena* arena) {
  return Arena::CreateMessageInternal< ::Imu_msgs::Imu >(arena);
}
template<> PROTOBUF_NOINLINE ::Imu_msgs::MagneticField*
Arena::CreateMaybeMessage< ::Imu_msgs::MagneticField >(Arena* arena) {
  return Arena::CreateMessageInternal< ::Imu_msgs::MagneticField >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
