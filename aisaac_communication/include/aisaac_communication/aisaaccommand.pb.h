// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: aisaaccommand.proto

#ifndef PROTOBUF_INCLUDED_aisaaccommand_2eproto
#define PROTOBUF_INCLUDED_aisaaccommand_2eproto

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3006001
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#define PROTOBUF_INTERNAL_EXPORT_protobuf_aisaaccommand_2eproto 

namespace protobuf_aisaaccommand_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
}  // namespace protobuf_aisaaccommand_2eproto
class AIsaacCommand;
class AIsaacCommandDefaultTypeInternal;
extern AIsaacCommandDefaultTypeInternal _AIsaacCommand_default_instance_;
class Kick;
class KickDefaultTypeInternal;
extern KickDefaultTypeInternal _Kick_default_instance_;
namespace google {
namespace protobuf {
template<> ::AIsaacCommand* Arena::CreateMaybeMessage<::AIsaacCommand>(Arena*);
template<> ::Kick* Arena::CreateMaybeMessage<::Kick>(Arena*);
}  // namespace protobuf
}  // namespace google

// ===================================================================

class Kick : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:Kick) */ {
 public:
  Kick();
  virtual ~Kick();

  Kick(const Kick& from);

  inline Kick& operator=(const Kick& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Kick(Kick&& from) noexcept
    : Kick() {
    *this = ::std::move(from);
  }

  inline Kick& operator=(Kick&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Kick& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Kick* internal_default_instance() {
    return reinterpret_cast<const Kick*>(
               &_Kick_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  void Swap(Kick* other);
  friend void swap(Kick& a, Kick& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Kick* New() const final {
    return CreateMaybeMessage<Kick>(NULL);
  }

  Kick* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<Kick>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const Kick& from);
  void MergeFrom(const Kick& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Kick* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // int32 sensor_use_type = 1;
  void clear_sensor_use_type();
  static const int kSensorUseTypeFieldNumber = 1;
  ::google::protobuf::int32 sensor_use_type() const;
  void set_sensor_use_type(::google::protobuf::int32 value);

  // bool kick_type = 2;
  void clear_kick_type();
  static const int kKickTypeFieldNumber = 2;
  bool kick_type() const;
  void set_kick_type(bool value);

  // int32 kick_strength = 3;
  void clear_kick_strength();
  static const int kKickStrengthFieldNumber = 3;
  ::google::protobuf::int32 kick_strength() const;
  void set_kick_strength(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:Kick)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int32 sensor_use_type_;
  bool kick_type_;
  ::google::protobuf::int32 kick_strength_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_aisaaccommand_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class AIsaacCommand : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:AIsaacCommand) */ {
 public:
  AIsaacCommand();
  virtual ~AIsaacCommand();

  AIsaacCommand(const AIsaacCommand& from);

  inline AIsaacCommand& operator=(const AIsaacCommand& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  AIsaacCommand(AIsaacCommand&& from) noexcept
    : AIsaacCommand() {
    *this = ::std::move(from);
  }

  inline AIsaacCommand& operator=(AIsaacCommand&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const AIsaacCommand& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const AIsaacCommand* internal_default_instance() {
    return reinterpret_cast<const AIsaacCommand*>(
               &_AIsaacCommand_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  void Swap(AIsaacCommand* other);
  friend void swap(AIsaacCommand& a, AIsaacCommand& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline AIsaacCommand* New() const final {
    return CreateMaybeMessage<AIsaacCommand>(NULL);
  }

  AIsaacCommand* New(::google::protobuf::Arena* arena) const final {
    return CreateMaybeMessage<AIsaacCommand>(arena);
  }
  void CopyFrom(const ::google::protobuf::Message& from) final;
  void MergeFrom(const ::google::protobuf::Message& from) final;
  void CopyFrom(const AIsaacCommand& from);
  void MergeFrom(const AIsaacCommand& from);
  void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) final;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const final;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(AIsaacCommand* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .Kick kick = 9;
  bool has_kick() const;
  void clear_kick();
  static const int kKickFieldNumber = 9;
  private:
  const ::Kick& _internal_kick() const;
  public:
  const ::Kick& kick() const;
  ::Kick* release_kick();
  ::Kick* mutable_kick();
  void set_allocated_kick(::Kick* kick);

  // int32 robot_command_coordinate_system_type = 1;
  void clear_robot_command_coordinate_system_type();
  static const int kRobotCommandCoordinateSystemTypeFieldNumber = 1;
  ::google::protobuf::int32 robot_command_coordinate_system_type() const;
  void set_robot_command_coordinate_system_type(::google::protobuf::int32 value);

  // int32 target_x = 2;
  void clear_target_x();
  static const int kTargetXFieldNumber = 2;
  ::google::protobuf::int32 target_x() const;
  void set_target_x(::google::protobuf::int32 value);

  // int32 target_y = 3;
  void clear_target_y();
  static const int kTargetYFieldNumber = 3;
  ::google::protobuf::int32 target_y() const;
  void set_target_y(::google::protobuf::int32 value);

  // int32 target_angle = 4;
  void clear_target_angle();
  static const int kTargetAngleFieldNumber = 4;
  ::google::protobuf::int32 target_angle() const;
  void set_target_angle(::google::protobuf::int32 value);

  // bool vision_data_valid = 5;
  void clear_vision_data_valid();
  static const int kVisionDataValidFieldNumber = 5;
  bool vision_data_valid() const;
  void set_vision_data_valid(bool value);

  // int32 current_x = 6;
  void clear_current_x();
  static const int kCurrentXFieldNumber = 6;
  ::google::protobuf::int32 current_x() const;
  void set_current_x(::google::protobuf::int32 value);

  // int32 current_y = 7;
  void clear_current_y();
  static const int kCurrentYFieldNumber = 7;
  ::google::protobuf::int32 current_y() const;
  void set_current_y(::google::protobuf::int32 value);

  // int32 current_angle = 8;
  void clear_current_angle();
  static const int kCurrentAngleFieldNumber = 8;
  ::google::protobuf::int32 current_angle() const;
  void set_current_angle(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:AIsaacCommand)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::Kick* kick_;
  ::google::protobuf::int32 robot_command_coordinate_system_type_;
  ::google::protobuf::int32 target_x_;
  ::google::protobuf::int32 target_y_;
  ::google::protobuf::int32 target_angle_;
  bool vision_data_valid_;
  ::google::protobuf::int32 current_x_;
  ::google::protobuf::int32 current_y_;
  ::google::protobuf::int32 current_angle_;
  mutable ::google::protobuf::internal::CachedSize _cached_size_;
  friend struct ::protobuf_aisaaccommand_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Kick

// int32 sensor_use_type = 1;
inline void Kick::clear_sensor_use_type() {
  sensor_use_type_ = 0;
}
inline ::google::protobuf::int32 Kick::sensor_use_type() const {
  // @@protoc_insertion_point(field_get:Kick.sensor_use_type)
  return sensor_use_type_;
}
inline void Kick::set_sensor_use_type(::google::protobuf::int32 value) {
  
  sensor_use_type_ = value;
  // @@protoc_insertion_point(field_set:Kick.sensor_use_type)
}

// bool kick_type = 2;
inline void Kick::clear_kick_type() {
  kick_type_ = false;
}
inline bool Kick::kick_type() const {
  // @@protoc_insertion_point(field_get:Kick.kick_type)
  return kick_type_;
}
inline void Kick::set_kick_type(bool value) {
  
  kick_type_ = value;
  // @@protoc_insertion_point(field_set:Kick.kick_type)
}

// int32 kick_strength = 3;
inline void Kick::clear_kick_strength() {
  kick_strength_ = 0;
}
inline ::google::protobuf::int32 Kick::kick_strength() const {
  // @@protoc_insertion_point(field_get:Kick.kick_strength)
  return kick_strength_;
}
inline void Kick::set_kick_strength(::google::protobuf::int32 value) {
  
  kick_strength_ = value;
  // @@protoc_insertion_point(field_set:Kick.kick_strength)
}

// -------------------------------------------------------------------

// AIsaacCommand

// int32 robot_command_coordinate_system_type = 1;
inline void AIsaacCommand::clear_robot_command_coordinate_system_type() {
  robot_command_coordinate_system_type_ = 0;
}
inline ::google::protobuf::int32 AIsaacCommand::robot_command_coordinate_system_type() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.robot_command_coordinate_system_type)
  return robot_command_coordinate_system_type_;
}
inline void AIsaacCommand::set_robot_command_coordinate_system_type(::google::protobuf::int32 value) {
  
  robot_command_coordinate_system_type_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.robot_command_coordinate_system_type)
}

// int32 target_x = 2;
inline void AIsaacCommand::clear_target_x() {
  target_x_ = 0;
}
inline ::google::protobuf::int32 AIsaacCommand::target_x() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.target_x)
  return target_x_;
}
inline void AIsaacCommand::set_target_x(::google::protobuf::int32 value) {
  
  target_x_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.target_x)
}

// int32 target_y = 3;
inline void AIsaacCommand::clear_target_y() {
  target_y_ = 0;
}
inline ::google::protobuf::int32 AIsaacCommand::target_y() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.target_y)
  return target_y_;
}
inline void AIsaacCommand::set_target_y(::google::protobuf::int32 value) {
  
  target_y_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.target_y)
}

// int32 target_angle = 4;
inline void AIsaacCommand::clear_target_angle() {
  target_angle_ = 0;
}
inline ::google::protobuf::int32 AIsaacCommand::target_angle() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.target_angle)
  return target_angle_;
}
inline void AIsaacCommand::set_target_angle(::google::protobuf::int32 value) {
  
  target_angle_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.target_angle)
}

// bool vision_data_valid = 5;
inline void AIsaacCommand::clear_vision_data_valid() {
  vision_data_valid_ = false;
}
inline bool AIsaacCommand::vision_data_valid() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.vision_data_valid)
  return vision_data_valid_;
}
inline void AIsaacCommand::set_vision_data_valid(bool value) {
  
  vision_data_valid_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.vision_data_valid)
}

// int32 current_x = 6;
inline void AIsaacCommand::clear_current_x() {
  current_x_ = 0;
}
inline ::google::protobuf::int32 AIsaacCommand::current_x() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.current_x)
  return current_x_;
}
inline void AIsaacCommand::set_current_x(::google::protobuf::int32 value) {
  
  current_x_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.current_x)
}

// int32 current_y = 7;
inline void AIsaacCommand::clear_current_y() {
  current_y_ = 0;
}
inline ::google::protobuf::int32 AIsaacCommand::current_y() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.current_y)
  return current_y_;
}
inline void AIsaacCommand::set_current_y(::google::protobuf::int32 value) {
  
  current_y_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.current_y)
}

// int32 current_angle = 8;
inline void AIsaacCommand::clear_current_angle() {
  current_angle_ = 0;
}
inline ::google::protobuf::int32 AIsaacCommand::current_angle() const {
  // @@protoc_insertion_point(field_get:AIsaacCommand.current_angle)
  return current_angle_;
}
inline void AIsaacCommand::set_current_angle(::google::protobuf::int32 value) {
  
  current_angle_ = value;
  // @@protoc_insertion_point(field_set:AIsaacCommand.current_angle)
}

// .Kick kick = 9;
inline bool AIsaacCommand::has_kick() const {
  return this != internal_default_instance() && kick_ != NULL;
}
inline void AIsaacCommand::clear_kick() {
  if (GetArenaNoVirtual() == NULL && kick_ != NULL) {
    delete kick_;
  }
  kick_ = NULL;
}
inline const ::Kick& AIsaacCommand::_internal_kick() const {
  return *kick_;
}
inline const ::Kick& AIsaacCommand::kick() const {
  const ::Kick* p = kick_;
  // @@protoc_insertion_point(field_get:AIsaacCommand.kick)
  return p != NULL ? *p : *reinterpret_cast<const ::Kick*>(
      &::_Kick_default_instance_);
}
inline ::Kick* AIsaacCommand::release_kick() {
  // @@protoc_insertion_point(field_release:AIsaacCommand.kick)
  
  ::Kick* temp = kick_;
  kick_ = NULL;
  return temp;
}
inline ::Kick* AIsaacCommand::mutable_kick() {
  
  if (kick_ == NULL) {
    auto* p = CreateMaybeMessage<::Kick>(GetArenaNoVirtual());
    kick_ = p;
  }
  // @@protoc_insertion_point(field_mutable:AIsaacCommand.kick)
  return kick_;
}
inline void AIsaacCommand::set_allocated_kick(::Kick* kick) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete kick_;
  }
  if (kick) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      kick = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, kick, submessage_arena);
    }
    
  } else {
    
  }
  kick_ = kick;
  // @@protoc_insertion_point(field_set_allocated:AIsaacCommand.kick)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)


// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_INCLUDED_aisaaccommand_2eproto
