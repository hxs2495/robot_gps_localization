#ifndef ROBOT_BAG_PLAY_TOOL__MESSAGE_PARSER_HPP_
#define ROBOT_BAG_PLAY_TOOL__MESSAGE_PARSER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <QByteArray>
#include <QString>

#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"

namespace robot_bag_play_tool
{

struct ParsedMessageNode
{
  QString name;
  QString type;
  QString value;
  std::vector<ParsedMessageNode> children;
};

class MessageParser
{
public:
  ParsedMessageNode parse(const std::string & topic_type, const QByteArray & serialized_data);
  QString rawPreview(const QByteArray & serialized_data) const;

private:
  struct TypeSupportEntry
  {
    std::shared_ptr<rcpputils::SharedLibrary> library;
    const rosidl_message_type_support_t * handle = nullptr;
  };

  const rosidl_message_type_support_t * getTypeSupport(const std::string & topic_type);
  ParsedMessageNode parseMessage(
    const void * message,
    const rosidl_message_type_support_t * type_support,
    const QString & name,
    int depth) const;
  ParsedMessageNode parseField(
    const void * field,
    const void * parent_message,
    const void * message_member,
    int depth) const;
  QString scalarValue(const void * field, uint8_t type_id) const;
  QString typeName(uint8_t type_id) const;

  std::unordered_map<std::string, TypeSupportEntry> type_support_cache_;
};

}  // namespace robot_bag_play_tool

#endif  // ROBOT_BAG_PLAY_TOOL__MESSAGE_PARSER_HPP_
