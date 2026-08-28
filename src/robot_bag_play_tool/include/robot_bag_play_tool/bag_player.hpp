#ifndef ROBOT_BAG_PLAY_TOOL__BAG_PLAYER_HPP_
#define ROBOT_BAG_PLAY_TOOL__BAG_PLAYER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <QObject>
#include <QString>

#include "rclcpp/generic_publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace robot_bag_play_tool
{

struct TopicInfo
{
  QString name;
  QString type;
  uint64_t message_count = 0;
  int64_t start_time_ns = 0;
  int64_t end_time_ns = 0;
  double frequency_hz = 0.0;
  bool checked = true;
  bool publisher_ready = false;
};

class BagPlayer : public QObject
{
  Q_OBJECT

public:
  explicit BagPlayer(rclcpp::Node::SharedPtr node, QObject * parent = nullptr);
  ~BagPlayer() override;

  bool loadBag(const QString & bag_uri, QString * error_message);
  std::vector<TopicInfo> topics() const;
  QString bagUri() const;
  QString storageId() const;
  QString serializationFormat() const;
  int64_t startTimeNs() const;
  int64_t endTimeNs() const;
  uint64_t bagSize() const;
  uint64_t totalMessageCount() const;

  void setTopicChecked(const QString & topic_name, bool checked);
  void setPlaybackRate(double rate);
  double playbackRate() const;

public slots:
  void closeBag();
  void playTopic(const QString & topic_name);
  void pauseTopic(const QString & topic_name);
  void resumeTopic(const QString & topic_name);
  void stopTopic(const QString & topic_name);
  void playAll();
  void stopAll();
  void playChecked();
  void seekTo(int64_t timestamp_ns);

signals:
  void bagLoaded();
  void loadFailed(const QString & error_message);
  void topicStatusChanged(
    const QString & topic_name,
    const QString & status,
    uint64_t published_count,
    double frequency_hz);
  void topicMessage(
    const QString & topic_name,
    const QString & topic_type,
    int64_t timestamp_ns,
    const QByteArray & serialized_data);
  void bagClockChanged(int64_t timestamp_ns);
  void playbackProgress(double progress);
  void bagClosed();

private:
  enum class TopicPlaybackState
  {
    Stopped,
    Playing,
    Paused,
    Error
  };

  struct TopicRuntime
  {
    TopicInfo info;
    std::shared_ptr<rclcpp::GenericPublisher> publisher;
    TopicPlaybackState state = TopicPlaybackState::Stopped;
    uint64_t published_count = 0;
    std::chrono::steady_clock::time_point last_status_emit;
    std::chrono::steady_clock::time_point last_message_emit;
  };

  TopicRuntime * runtimeForLocked(const QString & topic_name);
  const TopicRuntime * runtimeForLocked(const QString & topic_name) const;
  void requestStopAndJoin();
  void startPlaybackFrom(int64_t timestamp_ns);
  void playbackLoop(int64_t start_timestamp_ns);
  bool waitUntilTimestamp(
    int64_t timestamp_ns,
    int64_t playback_start_time_ns,
    std::chrono::steady_clock::time_point & playback_wall_start);
  bool anyTopicPlayingLocked() const;
  bool anyTopicScheduledLocked() const;
  void resetPlaybackClockLocked();
  void publishClock(int64_t timestamp_ns);
  void emitTopicStateLocked(const TopicRuntime & runtime, double frequency_hz = 0.0);
  QString stateText(TopicPlaybackState state) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, TopicRuntime> topics_;
  std::thread playback_worker_;
  std::atomic_bool stop_requested_{false};
  std::atomic_bool playback_active_{false};
  QString bag_uri_;
  QString storage_id_;
  QString serialization_format_;
  int64_t start_time_ns_ = 0;
  int64_t end_time_ns_ = 0;
  int64_t last_clock_time_ns_ = -1;
  uint64_t bag_size_ = 0;
  uint64_t total_message_count_ = 0;
  std::atomic<double> playback_rate_{1.0};
};

}  // namespace robot_bag_play_tool

#endif  // ROBOT_BAG_PLAY_TOOL__BAG_PLAYER_HPP_
