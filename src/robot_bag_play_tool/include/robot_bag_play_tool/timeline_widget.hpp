#ifndef ROBOT_BAG_PLAY_TOOL__TIMELINE_WIDGET_HPP_
#define ROBOT_BAG_PLAY_TOOL__TIMELINE_WIDGET_HPP_

#include <vector>

#include <QWidget>

#include "robot_bag_play_tool/bag_player.hpp"

namespace robot_bag_play_tool
{

class TimelineWidget : public QWidget
{
  Q_OBJECT

public:
  explicit TimelineWidget(QWidget * parent = nullptr);

  void setTopics(const std::vector<TopicInfo> & topics);
  void setTimeRange(int64_t start_time_ns, int64_t end_time_ns);
  void setCurrentTime(int64_t timestamp_ns);
  void zoomIn();
  void zoomOut();
  void fit();

signals:
  void seekRequested(int64_t timestamp_ns);

protected:
  void paintEvent(QPaintEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void wheelEvent(QWheelEvent * event) override;

private:
  int64_t xToTime(int x) const;
  int timeToX(int64_t timestamp_ns) const;
  QString formatOffset(int64_t timestamp_ns) const;
  double visibleDurationNs() const;
  void clampView();

  std::vector<TopicInfo> topics_;
  int64_t start_time_ns_ = 0;
  int64_t end_time_ns_ = 0;
  int64_t current_time_ns_ = 0;
  int64_t view_start_ns_ = 0;
  double zoom_ = 1.0;
  bool dragging_cursor_ = false;
};

}  // namespace robot_bag_play_tool

#endif  // ROBOT_BAG_PLAY_TOOL__TIMELINE_WIDGET_HPP_
