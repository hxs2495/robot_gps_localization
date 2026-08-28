#include "robot_bag_play_tool/timeline_widget.hpp"

#include <algorithm>
#include <cmath>

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

namespace robot_bag_play_tool
{
namespace
{
constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr int kLeftMargin = 230;
constexpr int kRightMargin = 22;
constexpr int kTopAxisHeight = 34;
constexpr int kRowHeight = 30;
constexpr int kBottomMargin = 12;

double niceStepSeconds(double raw_step)
{
  static constexpr double kSteps[] = {
    0.001, 0.002, 0.005, 0.01, 0.02, 0.05,
    0.1, 0.2, 0.5, 1.0, 2.0, 5.0,
    10.0, 15.0, 30.0, 60.0, 120.0, 300.0,
    600.0, 900.0, 1800.0, 3600.0};
  for (const auto step : kSteps) {
    if (raw_step <= step) {
      return step;
    }
  }
  return std::ceil(raw_step / 3600.0) * 3600.0;
}
}  // namespace

TimelineWidget::TimelineWidget(QWidget * parent)
: QWidget(parent)
{
  setMinimumHeight(180);
  setMouseTracking(true);
  setAutoFillBackground(false);
}

void TimelineWidget::setTopics(const std::vector<TopicInfo> & topics)
{
  topics_ = topics;
  setMinimumHeight(kTopAxisHeight + kBottomMargin + std::max(4, static_cast<int>(topics_.size())) * kRowHeight);
  updateGeometry();
  update();
}

void TimelineWidget::setTimeRange(int64_t start_time_ns, int64_t end_time_ns)
{
  start_time_ns_ = start_time_ns;
  end_time_ns_ = std::max(end_time_ns, start_time_ns + 1);
  current_time_ns_ = std::clamp(current_time_ns_ == 0 ? start_time_ns_ : current_time_ns_, start_time_ns_, end_time_ns_);
  view_start_ns_ = start_time_ns_;
  zoom_ = 1.0;
  update();
}

void TimelineWidget::setCurrentTime(int64_t timestamp_ns)
{
  current_time_ns_ = std::clamp(timestamp_ns, start_time_ns_, end_time_ns_);
  const auto visible = static_cast<int64_t>(visibleDurationNs());
  if (current_time_ns_ < view_start_ns_ || current_time_ns_ > view_start_ns_ + visible) {
    view_start_ns_ = current_time_ns_ - visible / 2;
    clampView();
  }
  update();
}

void TimelineWidget::zoomIn()
{
  zoom_ = std::min(zoom_ * 1.5, 200.0);
  clampView();
  update();
}

void TimelineWidget::zoomOut()
{
  zoom_ = std::max(zoom_ / 1.5, 1.0);
  clampView();
  update();
}

void TimelineWidget::fit()
{
  zoom_ = 1.0;
  view_start_ns_ = start_time_ns_;
  update();
}

void TimelineWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.fillRect(rect(), QColor("#ffffff"));

  const int timeline_width = std::max(1, width() - kLeftMargin - kRightMargin);
  const auto visible_ns = static_cast<int64_t>(visibleDurationNs());
  const int64_t view_end_ns = view_start_ns_ + visible_ns;

  painter.setPen(QColor("#e5eaf2"));
  painter.drawLine(kLeftMargin, kTopAxisHeight - 1, width() - kRightMargin, kTopAxisHeight - 1);
  painter.drawLine(kLeftMargin - 1, 0, kLeftMargin - 1, height());

  const double visible_seconds = static_cast<double>(visible_ns) / kNanosecondsPerSecond;
  const double step_seconds = niceStepSeconds(visible_seconds / 8.0);
  const int64_t step_ns = static_cast<int64_t>(step_seconds * kNanosecondsPerSecond);
  const int64_t first_tick =
    ((std::max<int64_t>(0, view_start_ns_ - start_time_ns_) + step_ns - 1) / step_ns) * step_ns + start_time_ns_;

  painter.setPen(QColor("#8a96aa"));
  QFont axis_font = painter.font();
  axis_font.setPointSize(9);
  painter.setFont(axis_font);
  for (int64_t tick = first_tick; tick <= view_end_ns; tick += step_ns) {
    const int x = timeToX(tick);
    painter.setPen(QColor("#d9e1ec"));
    painter.drawLine(x, kTopAxisHeight - 8, x, height() - kBottomMargin);
    painter.setPen(QColor("#667085"));
    painter.drawText(x + 4, 20, formatOffset(tick));
  }

  QFont topic_font = painter.font();
  topic_font.setPointSize(9);
  painter.setFont(topic_font);

  for (int row = 0; row < static_cast<int>(topics_.size()); ++row) {
    const auto & topic = topics_[row];
    const int y = kTopAxisHeight + row * kRowHeight;
    const QRect row_rect(0, y, width(), kRowHeight);
    if (row % 2 == 1) {
      painter.fillRect(row_rect, QColor("#fafcff"));
    }

    painter.setPen(QColor("#223048"));
    const QString label = topic.name.size() > 36 ? topic.name.left(33) + "..." : topic.name;
    painter.drawText(QRect(12, y, kLeftMargin - 24, kRowHeight), Qt::AlignVCenter | Qt::AlignLeft, label);

    const int64_t topic_start = topic.start_time_ns > 0 ? topic.start_time_ns : start_time_ns_;
    const int64_t topic_end = topic.end_time_ns > topic_start ? topic.end_time_ns : end_time_ns_;
    const int x1 = std::clamp(timeToX(topic_start), kLeftMargin, kLeftMargin + timeline_width);
    const int x2 = std::clamp(timeToX(topic_end), kLeftMargin, kLeftMargin + timeline_width);
    QRect bar_rect(x1, y + 9, std::max(2, x2 - x1), 12);
    painter.setPen(Qt::NoPen);
    painter.setBrush(topic.checked ? QColor("#2f80ed") : QColor("#c8d2df"));
    painter.drawRoundedRect(bar_rect, 4, 4);
  }

  const int cursor_x = timeToX(current_time_ns_);
  if (cursor_x >= kLeftMargin && cursor_x <= width() - kRightMargin) {
    painter.setPen(QPen(QColor("#d92d20"), 2));
    painter.drawLine(cursor_x, 0, cursor_x, height() - kBottomMargin);
    painter.setBrush(QColor("#d92d20"));
    painter.setPen(Qt::NoPen);
    QPolygon triangle;
    triangle << QPoint(cursor_x - 6, 0) << QPoint(cursor_x + 6, 0) << QPoint(cursor_x, 9);
    painter.drawPolygon(triangle);
  }
}

void TimelineWidget::mousePressEvent(QMouseEvent * event)
{
  if (event->button() != Qt::LeftButton || event->x() < kLeftMargin) {
    return;
  }
  dragging_cursor_ = true;
  emit seekRequested(xToTime(event->x()));
}

void TimelineWidget::mouseMoveEvent(QMouseEvent * event)
{
  if (dragging_cursor_) {
    emit seekRequested(xToTime(event->x()));
  }
}

void TimelineWidget::mouseReleaseEvent(QMouseEvent * event)
{
  if (event->button() == Qt::LeftButton) {
    dragging_cursor_ = false;
  }
}

void TimelineWidget::wheelEvent(QWheelEvent * event)
{
  if (event->modifiers() & Qt::ControlModifier) {
    if (event->angleDelta().y() > 0) {
      zoomIn();
    } else {
      zoomOut();
    }
    event->accept();
    return;
  }

  const auto visible = static_cast<int64_t>(visibleDurationNs());
  const int64_t delta = static_cast<int64_t>(visible * -event->angleDelta().y() / 2400.0);
  view_start_ns_ += delta;
  clampView();
  update();
  event->accept();
}

int64_t TimelineWidget::xToTime(int x) const
{
  const int timeline_width = std::max(1, width() - kLeftMargin - kRightMargin);
  const double ratio = std::clamp(static_cast<double>(x - kLeftMargin) / timeline_width, 0.0, 1.0);
  const auto visible_ns = visibleDurationNs();
  return std::clamp(
    static_cast<int64_t>(view_start_ns_ + ratio * visible_ns),
    start_time_ns_,
    end_time_ns_);
}

int TimelineWidget::timeToX(int64_t timestamp_ns) const
{
  const int timeline_width = std::max(1, width() - kLeftMargin - kRightMargin);
  const double ratio = static_cast<double>(timestamp_ns - view_start_ns_) / visibleDurationNs();
  return kLeftMargin + static_cast<int>(ratio * timeline_width);
}

QString TimelineWidget::formatOffset(int64_t timestamp_ns) const
{
  const int64_t offset_ns = std::max<int64_t>(0, timestamp_ns - start_time_ns_);
  const int64_t total_ms = offset_ns / 1000000LL;
  const int64_t hours = total_ms / 3600000LL;
  const int64_t minutes = (total_ms / 60000LL) % 60LL;
  const int64_t seconds = (total_ms / 1000LL) % 60LL;
  const int64_t millis = total_ms % 1000LL;

  if (hours > 0) {
    return QString("%1:%2:%3").arg(hours).arg(minutes, 2, 10, QLatin1Char('0')).arg(seconds, 2, 10, QLatin1Char('0'));
  }
  if (minutes > 0) {
    return QString("%1:%2.%3").arg(minutes).arg(seconds, 2, 10, QLatin1Char('0')).arg(millis, 3, 10, QLatin1Char('0'));
  }
  return QString("%1.%2s").arg(seconds).arg(millis, 3, 10, QLatin1Char('0'));
}

double TimelineWidget::visibleDurationNs() const
{
  const double duration = static_cast<double>(std::max<int64_t>(1, end_time_ns_ - start_time_ns_));
  return std::max(1.0, duration / zoom_);
}

void TimelineWidget::clampView()
{
  const auto visible = static_cast<int64_t>(visibleDurationNs());
  view_start_ns_ = std::clamp(view_start_ns_, start_time_ns_, std::max(start_time_ns_, end_time_ns_ - visible));
}

}  // namespace robot_bag_play_tool
