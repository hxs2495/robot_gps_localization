#include "robot_bag_play_tool/bag_player.hpp"

#include <algorithm>
#include <chrono>
#include <deque>
#include <exception>
#include <limits>
#include <utility>

#include <QByteArray>

#include "rclcpp/serialized_message.hpp"
#include "rmw/rmw.h"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/storage_options.hpp"

namespace robot_bag_play_tool
{
namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr auto kStatusInterval = std::chrono::milliseconds(250);
constexpr auto kMessagePreviewInterval = std::chrono::milliseconds(33);

int64_t toNanoseconds(const std::chrono::time_point<std::chrono::high_resolution_clock> & time)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count();
}
}  // namespace

BagPlayer::BagPlayer(rclcpp::Node::SharedPtr node, QObject * parent)
: QObject(parent),
  node_(std::move(node))
{
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
}

BagPlayer::~BagPlayer()
{
  closeBag();
}

bool BagPlayer::loadBag(const QString & bag_uri, QString * error_message)
{
  closeBag();

  try {
    rosbag2_storage::MetadataIo metadata_io;
    auto metadata = metadata_io.read_metadata(bag_uri.toStdString());

    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = bag_uri.toStdString();
    storage_options.storage_id = metadata.storage_identifier;

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format =
      metadata.topics_with_message_count.empty() ?
      rmw_get_serialization_format() :
      metadata.topics_with_message_count.front().topic_metadata.serialization_format;
    if (converter_options.input_serialization_format.empty()) {
      converter_options.input_serialization_format = rmw_get_serialization_format();
    }
    converter_options.output_serialization_format = rmw_get_serialization_format();
    reader.open(storage_options, converter_options);
    metadata = reader.get_metadata();

    const int64_t start_time_ns = toNanoseconds(metadata.starting_time);
    const int64_t end_time_ns = start_time_ns + metadata.duration.count();
    const double duration_seconds =
      std::max(1e-9, static_cast<double>(std::max<int64_t>(1, end_time_ns - start_time_ns)) / kNanosecondsPerSecond);

    std::unordered_map<std::string, TopicRuntime> new_topics;
    for (const auto & topic_info : metadata.topics_with_message_count) {
      TopicRuntime runtime;
      runtime.info.name = QString::fromStdString(topic_info.topic_metadata.name);
      runtime.info.type = QString::fromStdString(topic_info.topic_metadata.type);
      runtime.info.message_count = topic_info.message_count;
      runtime.info.start_time_ns = start_time_ns;
      runtime.info.end_time_ns = end_time_ns;
      runtime.info.frequency_hz = static_cast<double>(topic_info.message_count) / duration_seconds;
      runtime.info.checked = true;
      try {
        runtime.publisher = node_->create_generic_publisher(
          topic_info.topic_metadata.name,
          topic_info.topic_metadata.type,
          rclcpp::QoS(10));
        runtime.info.publisher_ready = true;
      } catch (const std::exception &) {
        runtime.publisher.reset();
        runtime.info.publisher_ready = false;
      }
      new_topics.emplace(topic_info.topic_metadata.name, std::move(runtime));
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      topics_ = std::move(new_topics);
      bag_uri_ = bag_uri;
      storage_id_ = QString::fromStdString(
        metadata.storage_identifier.empty() ? "sqlite3" : metadata.storage_identifier);
      serialization_format_ = QString::fromStdString(converter_options.input_serialization_format);
      start_time_ns_ = start_time_ns;
      end_time_ns_ = std::max(end_time_ns, start_time_ns + 1);
      last_clock_time_ns_ = start_time_ns_ - 1;
      bag_size_ = metadata.bag_size;
      total_message_count_ = metadata.message_count;
    }

    emit bagLoaded();
    return true;
  } catch (const std::exception & ex) {
    if (error_message != nullptr) {
      *error_message = QString::fromUtf8(ex.what());
    }
    emit loadFailed(QString::fromUtf8(ex.what()));
    return false;
  }
}

std::vector<TopicInfo> BagPlayer::topics() const
{
  std::vector<TopicInfo> result;
  std::lock_guard<std::mutex> lock(mutex_);
  result.reserve(topics_.size());
  for (const auto & item : topics_) {
    result.push_back(item.second.info);
  }
  std::sort(result.begin(), result.end(), [](const TopicInfo & left, const TopicInfo & right) {
    return left.name < right.name;
  });
  return result;
}

QString BagPlayer::bagUri() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return bag_uri_;
}

QString BagPlayer::storageId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return storage_id_;
}

QString BagPlayer::serializationFormat() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return serialization_format_;
}

int64_t BagPlayer::startTimeNs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return start_time_ns_;
}

int64_t BagPlayer::endTimeNs() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return end_time_ns_;
}

uint64_t BagPlayer::bagSize() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return bag_size_;
}

uint64_t BagPlayer::totalMessageCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return total_message_count_;
}

void BagPlayer::setTopicChecked(const QString & topic_name, bool checked)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto * runtime = runtimeForLocked(topic_name);
  if (runtime != nullptr) {
    runtime->info.checked = checked;
  }
}

void BagPlayer::setPlaybackRate(double rate)
{
  playback_rate_.store(std::max(0.01, rate));
}

double BagPlayer::playbackRate() const
{
  return playback_rate_.load();
}

void BagPlayer::closeBag()
{
  requestStopAndJoin();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    topics_.clear();
    bag_uri_.clear();
    storage_id_.clear();
    serialization_format_.clear();
    start_time_ns_ = 0;
    end_time_ns_ = 0;
    last_clock_time_ns_ = -1;
    bag_size_ = 0;
    total_message_count_ = 0;
  }
  emit playbackProgress(0.0);
  emit bagClosed();
}

void BagPlayer::playTopic(const QString & topic_name)
{
  bool should_start = false;
  int64_t start_time = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto * runtime = runtimeForLocked(topic_name);
    if (runtime == nullptr) {
      return;
    }

    if (!playback_active_.load()) {
      resetPlaybackClockLocked();
      for (auto & item : topics_) {
        item.second.published_count = 0;
        item.second.state = TopicPlaybackState::Stopped;
      }
      start_time = start_time_ns_;
      should_start = true;
    } else {
      start_time = std::max(start_time_ns_, last_clock_time_ns_);
    }

    runtime->state = TopicPlaybackState::Playing;
    emitTopicStateLocked(*runtime);
  }

  if (should_start) {
    startPlaybackFrom(start_time);
  }
}

void BagPlayer::pauseTopic(const QString & topic_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto * runtime = runtimeForLocked(topic_name);
  if (runtime != nullptr && runtime->state == TopicPlaybackState::Playing) {
    runtime->state = TopicPlaybackState::Paused;
    emitTopicStateLocked(*runtime);
  }
}

void BagPlayer::resumeTopic(const QString & topic_name)
{
  bool should_start = false;
  int64_t start_time = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto * runtime = runtimeForLocked(topic_name);
    if (runtime == nullptr) {
      return;
    }
    runtime->state = TopicPlaybackState::Playing;
    start_time = std::max(start_time_ns_, last_clock_time_ns_);
    should_start = !playback_active_.load();
    emitTopicStateLocked(*runtime);
  }
  if (should_start) {
    startPlaybackFrom(start_time);
  }
}

void BagPlayer::stopTopic(const QString & topic_name)
{
  bool should_stop = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto * runtime = runtimeForLocked(topic_name);
    if (runtime == nullptr) {
      return;
    }
    runtime->state = TopicPlaybackState::Stopped;
    emitTopicStateLocked(*runtime);
    should_stop = playback_active_.load() && !anyTopicScheduledLocked();
  }
  if (should_stop) {
    requestStopAndJoin();
  }
}

void BagPlayer::playAll()
{
  requestStopAndJoin();
  int64_t start_time = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resetPlaybackClockLocked();
    start_time = start_time_ns_;
    for (auto & item : topics_) {
      item.second.published_count = 0;
      item.second.state = TopicPlaybackState::Playing;
      item.second.last_status_emit = {};
      item.second.last_message_emit = {};
      emitTopicStateLocked(item.second);
    }
  }
  startPlaybackFrom(start_time);
}

void BagPlayer::stopAll()
{
  requestStopAndJoin();
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto & item : topics_) {
    item.second.state = TopicPlaybackState::Stopped;
    emitTopicStateLocked(item.second);
  }
}

void BagPlayer::playChecked()
{
  requestStopAndJoin();
  int64_t start_time = 0;
  bool has_checked = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    resetPlaybackClockLocked();
    start_time = start_time_ns_;
    for (auto & item : topics_) {
      item.second.published_count = 0;
      item.second.state = item.second.info.checked ? TopicPlaybackState::Playing : TopicPlaybackState::Stopped;
      item.second.last_status_emit = {};
      item.second.last_message_emit = {};
      has_checked = has_checked || item.second.info.checked;
      emitTopicStateLocked(item.second);
    }
  }
  if (has_checked) {
    startPlaybackFrom(start_time);
  }
}

void BagPlayer::seekTo(int64_t timestamp_ns)
{
  const bool was_active = playback_active_.load();
  requestStopAndJoin();

  bool should_restart = false;
  int64_t seek_time = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    seek_time = std::clamp(timestamp_ns, start_time_ns_, end_time_ns_);
    last_clock_time_ns_ = seek_time - 1;
    should_restart = was_active && anyTopicScheduledLocked();
  }

  publishClock(seek_time);
  if (should_restart) {
    startPlaybackFrom(seek_time);
  }
}

BagPlayer::TopicRuntime * BagPlayer::runtimeForLocked(const QString & topic_name)
{
  const auto found = topics_.find(topic_name.toStdString());
  if (found == topics_.end()) {
    return nullptr;
  }
  return &found->second;
}

const BagPlayer::TopicRuntime * BagPlayer::runtimeForLocked(const QString & topic_name) const
{
  const auto found = topics_.find(topic_name.toStdString());
  if (found == topics_.end()) {
    return nullptr;
  }
  return &found->second;
}

void BagPlayer::requestStopAndJoin()
{
  stop_requested_.store(true);
  if (playback_worker_.joinable()) {
    playback_worker_.join();
  }
  playback_active_.store(false);
  stop_requested_.store(false);
}

void BagPlayer::startPlaybackFrom(int64_t timestamp_ns)
{
  requestStopAndJoin();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (bag_uri_.isEmpty() || !anyTopicScheduledLocked()) {
      return;
    }
  }
  stop_requested_.store(false);
  playback_active_.store(true);
  playback_worker_ = std::thread(&BagPlayer::playbackLoop, this, timestamp_ns);
}

void BagPlayer::playbackLoop(int64_t start_timestamp_ns)
{
  try {
    rosbag2_storage::StorageOptions storage_options;
    rosbag2_cpp::ConverterOptions converter_options;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      storage_options.uri = bag_uri_.toStdString();
      storage_options.storage_id = storage_id_.toStdString();
      converter_options.input_serialization_format = serialization_format_.toStdString();
    }
    converter_options.output_serialization_format = rmw_get_serialization_format();

    rosbag2_cpp::Reader reader;
    reader.open(storage_options, converter_options);
    reader.seek(start_timestamp_ns);

    auto playback_wall_start = std::chrono::steady_clock::now();
    std::unordered_map<std::string, std::deque<std::chrono::steady_clock::time_point>> publish_times;

    while (!stop_requested_.load() && reader.has_next()) {
      const auto bag_message = reader.read_next();
      const int64_t stamp = bag_message->time_stamp;

      if (!waitUntilTimestamp(stamp, start_timestamp_ns, playback_wall_start)) {
        break;
      }

      TopicRuntime snapshot;
      bool should_publish = false;
      bool known_topic = true;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto found = topics_.find(bag_message->topic_name);
        if (found == topics_.end()) {
          known_topic = false;
        } else {
          if (!anyTopicScheduledLocked()) {
            break;
          }
          should_publish = found->second.state == TopicPlaybackState::Playing;
          snapshot = found->second;
        }
      }

      publishClock(stamp);
      if (!known_topic || !should_publish) {
        continue;
      }

      rclcpp::SerializedMessage serialized_message(*bag_message->serialized_data);
      if (snapshot.publisher) {
        snapshot.publisher->publish(serialized_message);
      }

      QByteArray raw(
        reinterpret_cast<const char *>(bag_message->serialized_data->buffer),
        static_cast<int>(bag_message->serialized_data->buffer_length));

      const auto now = std::chrono::steady_clock::now();
      double frequency = 0.0;
      uint64_t count = 0;
      bool emit_message = false;
      bool emit_status = false;
      TopicRuntime state_snapshot;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        auto & runtime = topics_.at(bag_message->topic_name);
        runtime.published_count += 1;
        count = runtime.published_count;

        auto & window = publish_times[bag_message->topic_name];
        window.push_back(now);
        while (!window.empty() && now - window.front() > std::chrono::seconds(2)) {
          window.pop_front();
        }
        if (window.size() >= 2) {
          const auto seconds = std::chrono::duration<double>(window.back() - window.front()).count();
          if (seconds > 0.0) {
            frequency = static_cast<double>(window.size() - 1) / seconds;
          }
        }

        emit_message = runtime.last_message_emit.time_since_epoch().count() == 0 ||
          now - runtime.last_message_emit >= kMessagePreviewInterval;
        emit_status = runtime.last_status_emit.time_since_epoch().count() == 0 ||
          now - runtime.last_status_emit >= kStatusInterval ||
          count == runtime.info.message_count;
        if (emit_message) {
          runtime.last_message_emit = now;
        }
        if (emit_status) {
          runtime.last_status_emit = now;
        }
        state_snapshot = runtime;
      }

      if (emit_message) {
        emit topicMessage(state_snapshot.info.name, state_snapshot.info.type, stamp, raw);
      }
      if (emit_status) {
        emit topicStatusChanged(state_snapshot.info.name, "播放中", count, frequency);
      }
    }
  } catch (const std::exception & ex) {
    std::vector<TopicRuntime> affected;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto & item : topics_) {
        if (item.second.state == TopicPlaybackState::Playing ||
          item.second.state == TopicPlaybackState::Paused)
        {
          item.second.state = TopicPlaybackState::Error;
          affected.push_back(item.second);
        }
      }
    }
    for (const auto & runtime : affected) {
      emit topicStatusChanged(
        runtime.info.name,
        QString("错误：%1").arg(ex.what()),
        runtime.published_count,
        0.0);
    }
  }

  std::vector<TopicRuntime> completed;
  if (!stop_requested_.load()) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto & item : topics_) {
      if (item.second.state == TopicPlaybackState::Playing ||
        item.second.state == TopicPlaybackState::Paused)
      {
        item.second.state = TopicPlaybackState::Stopped;
        completed.push_back(item.second);
      }
    }
  }
  playback_active_.store(false);

  for (const auto & runtime : completed) {
    emit topicStatusChanged(runtime.info.name, "已完成", runtime.published_count, 0.0);
  }
}

bool BagPlayer::waitUntilTimestamp(
  int64_t timestamp_ns,
  int64_t playback_start_time_ns,
  std::chrono::steady_clock::time_point & playback_wall_start)
{
  while (true) {
    if (stop_requested_.load()) {
      return false;
    }

    bool sleep_for_paused_topics = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!anyTopicScheduledLocked()) {
        return false;
      }
      if (!anyTopicPlayingLocked()) {
        sleep_for_paused_topics = true;
      }
    }
    if (sleep_for_paused_topics) {
      const auto pause_start = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      playback_wall_start += std::chrono::steady_clock::now() - pause_start;
      continue;
    }

    const auto now = std::chrono::steady_clock::now();
    const double rate = std::max(0.01, playback_rate_.load());
    const auto bag_offset_ns = std::max<int64_t>(0, timestamp_ns - playback_start_time_ns);
    const auto scaled_offset_ns = static_cast<int64_t>(static_cast<double>(bag_offset_ns) / rate);
    const auto target_time = playback_wall_start + std::chrono::nanoseconds(scaled_offset_ns);
    if (now >= target_time) {
      return true;
    }

    const auto remaining_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(target_time - now).count();
    const auto sleep_ns = std::min<int64_t>(remaining_ns, 20000000LL);
    std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_ns));
  }
}

bool BagPlayer::anyTopicPlayingLocked() const
{
  return std::any_of(topics_.begin(), topics_.end(), [](const auto & item) {
    return item.second.state == TopicPlaybackState::Playing;
  });
}

bool BagPlayer::anyTopicScheduledLocked() const
{
  return std::any_of(topics_.begin(), topics_.end(), [](const auto & item) {
    return item.second.state == TopicPlaybackState::Playing ||
      item.second.state == TopicPlaybackState::Paused;
  });
}

void BagPlayer::resetPlaybackClockLocked()
{
  last_clock_time_ns_ = start_time_ns_ - 1;
  emit playbackProgress(0.0);
}

void BagPlayer::publishClock(int64_t timestamp_ns)
{
  double progress = 0.0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (timestamp_ns < last_clock_time_ns_) {
      return;
    }
    last_clock_time_ns_ = timestamp_ns;
    const auto denominator = std::max<int64_t>(1, end_time_ns_ - start_time_ns_);
    progress = std::clamp(static_cast<double>(timestamp_ns - start_time_ns_) / denominator, 0.0, 1.0);
  }

  rosgraph_msgs::msg::Clock clock;
  clock.clock.sec = static_cast<int32_t>(timestamp_ns / kNanosecondsPerSecond);
  clock.clock.nanosec = static_cast<uint32_t>(timestamp_ns % kNanosecondsPerSecond);
  clock_publisher_->publish(clock);
  emit bagClockChanged(timestamp_ns);
  emit playbackProgress(progress);
}

void BagPlayer::emitTopicStateLocked(const TopicRuntime & runtime, double frequency_hz)
{
  emit topicStatusChanged(
    runtime.info.name,
    stateText(runtime.state),
    runtime.published_count,
    frequency_hz);
}

QString BagPlayer::stateText(TopicPlaybackState state) const
{
  switch (state) {
    case TopicPlaybackState::Playing:
      return "播放中";
    case TopicPlaybackState::Paused:
      return "已暂停";
    case TopicPlaybackState::Error:
      return "错误";
    case TopicPlaybackState::Stopped:
    default:
      return "已停止";
  }
}

}  // namespace robot_bag_play_tool
