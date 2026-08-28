#include "robot_bag_play_tool/message_parser.hpp"

#include <algorithm>
#include <iomanip>
#include <new>
#include <sstream>
#include <stdexcept>

#include "rmw/rmw.h"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace robot_bag_play_tool
{
namespace
{
constexpr size_t kMaxArrayItems = 50;
constexpr int kMaxDepth = 8;

const auto * introspectionMembers(const rosidl_message_type_support_t * type_support)
{
  if (type_support == nullptr || type_support->data == nullptr) {
    return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(nullptr);
  }
  return static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(type_support->data);
}

QString boundedString(const std::string & value)
{
  constexpr size_t kMaxStringLength = 240;
  if (value.size() <= kMaxStringLength) {
    return QString::fromStdString(value);
  }
  return QString::fromStdString(value.substr(0, kMaxStringLength)) + " ...";
}
}  // namespace

ParsedMessageNode MessageParser::parse(
  const std::string & topic_type,
  const QByteArray & serialized_data)
{
  ParsedMessageNode root;
  root.name = QString::fromStdString(topic_type);
  root.type = "message";

  const auto * type_support = getTypeSupport(topic_type);
  const auto * members = introspectionMembers(type_support);
  if (members == nullptr) {
    root.value = "No introspection type support";
    return root;
  }

  void * message_memory = ::operator new(members->size_of_);
  members->init_function(message_memory, rosidl_runtime_cpp::MessageInitialization::ALL);

  rmw_serialized_message_t serialized_message = rmw_get_zero_initialized_serialized_message();
  serialized_message.buffer = reinterpret_cast<uint8_t *>(const_cast<char *>(serialized_data.constData()));
  serialized_message.buffer_length = static_cast<size_t>(serialized_data.size());
  serialized_message.buffer_capacity = static_cast<size_t>(serialized_data.size());
  serialized_message.allocator = rcutils_get_default_allocator();

  const auto ret = rmw_deserialize(&serialized_message, type_support, message_memory);
  if (ret != RMW_RET_OK) {
    root.value = QString("Deserialize failed: ") + rcutils_get_error_string().str;
    rcutils_reset_error();
    members->fini_function(message_memory);
    ::operator delete(message_memory);
    return root;
  }

  root = parseMessage(message_memory, type_support, QString::fromStdString(topic_type), 0);
  members->fini_function(message_memory);
  ::operator delete(message_memory);
  return root;
}

QString MessageParser::rawPreview(const QByteArray & serialized_data) const
{
  std::ostringstream stream;
  stream << "bytes: " << serialized_data.size() << "\n";
  const int limit = std::min(serialized_data.size(), 1024);
  for (int i = 0; i < limit; ++i) {
    if (i % 16 == 0) {
      stream << std::setw(4) << std::setfill('0') << std::hex << i << ": ";
    }
    const auto byte = static_cast<unsigned char>(serialized_data.at(i));
    stream << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(byte) << ' ';
    if (i % 16 == 15) {
      stream << '\n';
    }
  }
  if (serialized_data.size() > limit) {
    stream << "\n... truncated";
  }
  return QString::fromStdString(stream.str());
}

const rosidl_message_type_support_t * MessageParser::getTypeSupport(const std::string & topic_type)
{
  const auto found = type_support_cache_.find(topic_type);
  if (found != type_support_cache_.end()) {
    return found->second.handle;
  }

  TypeSupportEntry entry;
  entry.library = rosbag2_cpp::get_typesupport_library(
    topic_type, rosidl_typesupport_introspection_cpp::typesupport_identifier);
  entry.handle = rosbag2_cpp::get_typesupport_handle(
    topic_type, rosidl_typesupport_introspection_cpp::typesupport_identifier, entry.library);
  const auto inserted = type_support_cache_.emplace(topic_type, entry);
  return inserted.first->second.handle;
}

ParsedMessageNode MessageParser::parseMessage(
  const void * message,
  const rosidl_message_type_support_t * type_support,
  const QString & name,
  int depth) const
{
  ParsedMessageNode node;
  node.name = name;
  node.type = "message";

  const auto * members = introspectionMembers(type_support);
  if (members == nullptr) {
    node.value = "No members";
    return node;
  }

  node.value = QString("%1 fields").arg(members->member_count_);
  if (depth >= kMaxDepth) {
    node.value = "Depth limit";
    return node;
  }

  for (uint32_t i = 0; i < members->member_count_; ++i) {
    const auto & member = members->members_[i];
    const auto * field = reinterpret_cast<const uint8_t *>(message) + member.offset_;
    node.children.push_back(parseField(field, message, &member, depth + 1));
  }
  return node;
}

ParsedMessageNode MessageParser::parseField(
  const void * field,
  const void *,
  const void * message_member,
  int depth) const
{
  const auto & member =
    *static_cast<const rosidl_typesupport_introspection_cpp::MessageMember *>(message_member);

  ParsedMessageNode node;
  node.name = QString::fromUtf8(member.name_);
  node.type = typeName(member.type_id_);

  if (!member.is_array_) {
    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      return parseMessage(field, member.members_, node.name, depth);
    }
    node.value = scalarValue(field, member.type_id_);
    return node;
  }

  const size_t size = member.size_function != nullptr ? member.size_function(field) : member.array_size_;
  node.value = QString("size=%1").arg(size);
  const size_t preview_size = std::min(size, kMaxArrayItems);

  for (size_t i = 0; i < preview_size; ++i) {
    ParsedMessageNode child;
    child.name = QString("[%1]").arg(i);
    child.type = node.type;
    const void * item = member.get_const_function != nullptr ? member.get_const_function(field, i) : nullptr;
    if (item == nullptr) {
      child.value = "unavailable";
    } else if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      child = parseMessage(item, member.members_, child.name, depth + 1);
    } else {
      child.value = scalarValue(item, member.type_id_);
    }
    node.children.push_back(child);
  }

  if (size > preview_size) {
    ParsedMessageNode more;
    more.name = "...";
    more.value = QString("%1 more items").arg(size - preview_size);
    node.children.push_back(more);
  }
  return node;
}

QString MessageParser::scalarValue(const void * field, uint8_t type_id) const
{
  using namespace rosidl_typesupport_introspection_cpp;
  switch (type_id) {
    case ROS_TYPE_FLOAT:
      return QString::number(*static_cast<const float *>(field), 'g', 8);
    case ROS_TYPE_DOUBLE:
      return QString::number(*static_cast<const double *>(field), 'g', 12);
    case ROS_TYPE_BOOLEAN:
      return *static_cast<const bool *>(field) ? "true" : "false";
    case ROS_TYPE_CHAR:
    case ROS_TYPE_INT8:
      return QString::number(*static_cast<const int8_t *>(field));
    case ROS_TYPE_WCHAR:
    case ROS_TYPE_UINT8:
    case ROS_TYPE_OCTET:
      return QString::number(*static_cast<const uint8_t *>(field));
    case ROS_TYPE_INT16:
      return QString::number(*static_cast<const int16_t *>(field));
    case ROS_TYPE_UINT16:
      return QString::number(*static_cast<const uint16_t *>(field));
    case ROS_TYPE_INT32:
      return QString::number(*static_cast<const int32_t *>(field));
    case ROS_TYPE_UINT32:
      return QString::number(*static_cast<const uint32_t *>(field));
    case ROS_TYPE_INT64:
      return QString::number(static_cast<qlonglong>(*static_cast<const int64_t *>(field)));
    case ROS_TYPE_UINT64:
      return QString::number(static_cast<qulonglong>(*static_cast<const uint64_t *>(field)));
    case ROS_TYPE_STRING:
      return boundedString(*static_cast<const std::string *>(field));
    case ROS_TYPE_WSTRING: {
      const auto & value = *static_cast<const std::wstring *>(field);
      return QString::fromStdWString(value.substr(0, 240));
    }
    default:
      return "<unsupported>";
  }
}

QString MessageParser::typeName(uint8_t type_id) const
{
  using namespace rosidl_typesupport_introspection_cpp;
  switch (type_id) {
    case ROS_TYPE_FLOAT:
      return "float32";
    case ROS_TYPE_DOUBLE:
      return "float64";
    case ROS_TYPE_LONG_DOUBLE:
      return "long double";
    case ROS_TYPE_CHAR:
      return "char";
    case ROS_TYPE_WCHAR:
      return "wchar";
    case ROS_TYPE_BOOLEAN:
      return "bool";
    case ROS_TYPE_OCTET:
      return "byte";
    case ROS_TYPE_UINT8:
      return "uint8";
    case ROS_TYPE_INT8:
      return "int8";
    case ROS_TYPE_UINT16:
      return "uint16";
    case ROS_TYPE_INT16:
      return "int16";
    case ROS_TYPE_UINT32:
      return "uint32";
    case ROS_TYPE_INT32:
      return "int32";
    case ROS_TYPE_UINT64:
      return "uint64";
    case ROS_TYPE_INT64:
      return "int64";
    case ROS_TYPE_STRING:
      return "string";
    case ROS_TYPE_WSTRING:
      return "wstring";
    case ROS_TYPE_MESSAGE:
      return "message";
    default:
      return "unknown";
  }
}

}  // namespace robot_bag_play_tool
