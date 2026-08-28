#include "robot_bag_play_tool/main_window.hpp"

#include <algorithm>

#include <QAction>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFontDatabase>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLocale>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QSplitter>
#include <QStringList>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QWidget>

namespace robot_bag_play_tool
{
namespace
{
enum TopicColumns
{
  kCheckColumn = 0,
  kTopicColumn,
  kTypeColumn,
  kCountColumn,
  kStatusColumn,
  kFrequencyColumn,
  kActionColumn,
  kColumnCount
};

QPushButton * makeButton(const QString & text, QWidget * parent, const char * variant = "secondary")
{
  auto * button = new QPushButton(text, parent);
  button->setProperty("variant", variant);
  button->setCursor(Qt::PointingHandCursor);
  button->setMinimumHeight(34);
  return button;
}

QPushButton * makeCompactButton(const QString & text, QWidget * parent, const char * variant = "ghost")
{
  auto * button = makeButton(text, parent, variant);
  button->setProperty("compact", true);
  button->setMinimumHeight(26);
  button->setMaximumHeight(26);
  button->setMinimumWidth(58);
  return button;
}

QLabel * makeCaption(const QString & text, QWidget * parent)
{
  auto * label = new QLabel(text, parent);
  label->setProperty("role", "caption");
  return label;
}

QFrame * makePanel(QWidget * parent)
{
  auto * panel = new QFrame(parent);
  panel->setProperty("class", "panel");
  panel->setFrameShape(QFrame::NoFrame);
  return panel;
}

void applyModernStyle(QWidget * widget)
{
  widget->setStyleSheet(R"(
    QWidget#AppRoot {
      background: #f6f8fb;
      color: #172033;
      font-size: 13px;
    }

    QFrame[class="panel"] {
      background: #ffffff;
      border: 1px solid #dde4ef;
      border-radius: 8px;
    }

    QLabel[role="title"] {
      color: #111827;
      font-size: 22px;
      font-weight: 700;
    }

    QLabel[role="caption"] {
      color: #657185;
      font-size: 12px;
    }

    QLabel[role="metric"] {
      background: #f8fafc;
      border: 1px solid #e1e7f0;
      border-radius: 7px;
      color: #263348;
      padding: 8px 10px;
    }

    QPushButton {
      background: #ffffff;
      border: 1px solid #d6deea;
      border-radius: 7px;
      color: #263348;
      font-weight: 600;
      padding: 7px 14px;
    }

    QPushButton:hover {
      background: #f3f7fc;
      border-color: #b8c6d9;
    }

    QPushButton:pressed {
      background: #e9eff7;
    }

    QPushButton[variant="primary"] {
      background: #2563eb;
      border-color: #2563eb;
      color: #ffffff;
    }

    QPushButton[variant="primary"]:hover {
      background: #1d4ed8;
      border-color: #1d4ed8;
    }

    QPushButton[variant="danger"] {
      color: #b42318;
      border-color: #f2c7c2;
      background: #fff7f6;
    }

    QPushButton[variant="danger"]:hover {
      background: #fee4e2;
    }

    QPushButton[variant="ghost"] {
      background: #f8fafc;
      border-color: #e1e7f0;
      color: #334155;
      font-weight: 600;
    }

    QPushButton[compact="true"] {
      border-radius: 6px;
      padding: 3px 8px;
      font-size: 12px;
    }

    QComboBox {
      background: #ffffff;
      border: 1px solid #d6deea;
      border-radius: 7px;
      padding: 6px 30px 6px 10px;
      min-height: 22px;
      color: #263348;
    }

    QComboBox:hover {
      border-color: #a9b8cc;
    }

    QProgressBar {
      background: #eef3f9;
      border: 0;
      border-radius: 4px;
      height: 8px;
      text-align: center;
      color: transparent;
    }

    QProgressBar::chunk {
      background: #2563eb;
      border-radius: 4px;
    }

    QTableWidget, QTreeView, QPlainTextEdit {
      background: #ffffff;
      border: 1px solid #e1e7f0;
      border-radius: 7px;
      selection-background-color: #dbeafe;
      selection-color: #172033;
    }

    QTableWidget {
      gridline-color: #edf1f6;
      alternate-background-color: #fafcff;
    }

    QHeaderView::section {
      background: #f8fafc;
      border: 0;
      border-bottom: 1px solid #e1e7f0;
      color: #657185;
      font-size: 12px;
      font-weight: 700;
      padding: 8px 10px;
    }

    QTabWidget::pane {
      border: 0;
      padding-top: 8px;
    }

    QTabBar::tab {
      background: #f3f6fb;
      border: 1px solid #dde4ef;
      border-radius: 7px;
      color: #556276;
      margin-right: 6px;
      padding: 7px 14px;
    }

    QTabBar::tab:selected {
      background: #ffffff;
      color: #172033;
      border-color: #b8c6d9;
    }

    QSplitter::handle {
      background: #edf1f6;
      margin: 8px 4px;
      border-radius: 2px;
    }

    QCheckBox::indicator {
      width: 16px;
      height: 16px;
      border-radius: 4px;
      border: 1px solid #b8c6d9;
      background: #ffffff;
    }

    QCheckBox::indicator:checked {
      background: #2563eb;
      border-color: #2563eb;
    }
  )");
}

void setMetric(QLabel * label)
{
  label->setProperty("role", "metric");
  label->setMinimumHeight(38);
}
}

MainWindow::MainWindow(std::shared_ptr<BagPlayer> bag_player, QWidget * parent)
: QMainWindow(parent),
  bag_player_(std::move(bag_player))
{
  buildUi();
  connect(bag_player_.get(), &BagPlayer::bagLoaded, this, &MainWindow::refreshTopics);
  connect(bag_player_.get(), &BagPlayer::topicStatusChanged, this, &MainWindow::updateTopicStatus);
  connect(bag_player_.get(), &BagPlayer::topicMessage, this, &MainWindow::showTopicMessage);
  connect(bag_player_.get(), &BagPlayer::bagClockChanged, this, &MainWindow::updateClock);
  connect(bag_player_.get(), &BagPlayer::playbackProgress, this, &MainWindow::updateProgress);
  connect(bag_player_.get(), &BagPlayer::bagClosed, this, &MainWindow::resetUi);
}

void MainWindow::buildUi()
{
  setWindowTitle("机器人 Bag 播放工具");
  resize(1360, 820);
  applyModernStyle(this);

  auto * file_menu = menuBar()->addMenu("File");
  auto * open_action = file_menu->addAction("Open Bag");
  auto * close_action = file_menu->addAction("Close Bag");
  file_menu->addSeparator();
  auto * exit_action = file_menu->addAction("Exit");

  auto * playback_menu = menuBar()->addMenu("Playback");
  auto * play_checked_action = playback_menu->addAction("Play Selected");
  auto * play_all_action = playback_menu->addAction("Play All");
  auto * stop_all_action = playback_menu->addAction("Stop");

  auto * view_menu = menuBar()->addMenu("View");
  auto * zoom_in_action = view_menu->addAction("Zoom In");
  auto * zoom_out_action = view_menu->addAction("Zoom Out");
  auto * fit_action = view_menu->addAction("Fit Timeline");

  auto * central = new QWidget(this);
  central->setObjectName("AppRoot");
  auto * root_layout = new QVBoxLayout(central);
  root_layout->setContentsMargins(18, 18, 18, 18);
  root_layout->setSpacing(14);

  auto * top_panel = makePanel(central);
  auto * top_layout = new QVBoxLayout(top_panel);
  top_layout->setContentsMargins(16, 14, 16, 14);
  top_layout->setSpacing(12);

  auto * toolbar = new QHBoxLayout();
  toolbar->setSpacing(10);
  auto * title_stack = new QVBoxLayout();
  title_stack->setSpacing(2);
  auto * title = new QLabel("机器人 Bag 播放工具", top_panel);
  title->setProperty("role", "title");
  title_stack->addWidget(title);
  title_stack->addWidget(makeCaption("ROS2 Bag 话题播放与中断调试面板", top_panel));

  auto * open_button = makeButton("打开 Bag", top_panel, "primary");
  auto * close_button = makeButton("关闭 Bag", top_panel);
  auto * play_all_button = makeButton("播放全部", top_panel);
  auto * stop_all_button = makeButton("停止全部", top_panel, "danger");
  auto * play_checked_button = makeButton("播放勾选", top_panel);
  auto * zoom_in_button = makeButton("放大", top_panel);
  auto * zoom_out_button = makeButton("缩小", top_panel);
  auto * fit_button = makeButton("适配", top_panel);
  auto * speed_box = new QComboBox(top_panel);
  speed_box->addItems({"0.1x", "0.25x", "0.5x", "1x", "1.5x", "2x", "5x", "10x"});
  speed_box->setCurrentText("1x");
  speed_box->setMinimumWidth(96);
  toolbar->addLayout(title_stack);
  toolbar->addStretch();
  toolbar->addWidget(open_button);
  toolbar->addWidget(close_button);
  toolbar->addWidget(play_checked_button);
  toolbar->addWidget(play_all_button);
  toolbar->addWidget(stop_all_button);
  toolbar->addWidget(zoom_out_button);
  toolbar->addWidget(zoom_in_button);
  toolbar->addWidget(fit_button);
  toolbar->addSpacing(8);
  toolbar->addWidget(makeCaption("速度", top_panel));
  toolbar->addWidget(speed_box);

  bag_path_label_ = new QLabel("Bag：未加载", top_panel);
  bag_info_label_ = new QLabel("话题：0 | 消息：0 | 时间范围：-", top_panel);
  clock_label_ = new QLabel("时钟：-", top_panel);
  status_label_ = new QLabel("状态：等待加载", top_panel);
  setMetric(bag_path_label_);
  setMetric(bag_info_label_);
  setMetric(clock_label_);
  setMetric(status_label_);
  progress_bar_ = new QProgressBar(top_panel);
  progress_bar_->setRange(0, 1000);
  progress_bar_->setValue(0);
  progress_bar_->setTextVisible(false);

  auto * info_layout = new QHBoxLayout();
  info_layout->setSpacing(10);
  info_layout->addWidget(bag_path_label_, 2);
  info_layout->addWidget(bag_info_label_, 3);
  info_layout->addWidget(clock_label_, 1);
  info_layout->addWidget(status_label_, 1);

  top_layout->addLayout(toolbar);
  top_layout->addLayout(info_layout);
  top_layout->addWidget(progress_bar_);

  auto * timeline_panel = makePanel(central);
  auto * timeline_layout = new QVBoxLayout(timeline_panel);
  timeline_layout->setContentsMargins(14, 12, 14, 12);
  timeline_layout->setSpacing(8);
  timeline_layout->addWidget(makeCaption("时间轴", timeline_panel));
  timeline_widget_ = new TimelineWidget(timeline_panel);
  timeline_layout->addWidget(timeline_widget_);

  auto * topic_panel = makePanel(central);
  auto * topic_layout = new QVBoxLayout(topic_panel);
  topic_layout->setContentsMargins(14, 14, 14, 14);
  topic_layout->setSpacing(10);
  topic_layout->addWidget(makeCaption("话题列表", topic_panel));

  topic_table_ = new QTableWidget(0, kColumnCount, topic_panel);
  topic_table_->setHorizontalHeaderLabels({
    "启用", "话题", "消息类型", "消息数", "状态", "频率", "控制"});
  topic_table_->horizontalHeader()->setSectionResizeMode(kTopicColumn, QHeaderView::Stretch);
  topic_table_->horizontalHeader()->setSectionResizeMode(kTypeColumn, QHeaderView::Stretch);
  topic_table_->horizontalHeader()->setSectionResizeMode(kCheckColumn, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(kCountColumn, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(kStatusColumn, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(kFrequencyColumn, QHeaderView::ResizeToContents);
  topic_table_->horizontalHeader()->setSectionResizeMode(kActionColumn, QHeaderView::ResizeToContents);
  topic_table_->verticalHeader()->setVisible(false);
  topic_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  topic_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  topic_table_->setAlternatingRowColors(true);
  topic_table_->setShowGrid(false);
  topic_table_->setWordWrap(false);
  topic_table_->verticalHeader()->setDefaultSectionSize(42);
  topic_layout->addWidget(topic_table_);

  tree_model_ = new QStandardItemModel(this);
  tree_model_->setHorizontalHeaderLabels({"字段", "类型", "值"});
  tree_view_ = new QTreeView(central);
  tree_view_->setModel(tree_model_);
  tree_view_->setUniformRowHeights(true);
  tree_view_->setAlternatingRowColors(true);
  tree_view_->setHeaderHidden(false);

  raw_view_ = new QPlainTextEdit(central);
  raw_view_->setReadOnly(true);
  raw_view_->setLineWrapMode(QPlainTextEdit::NoWrap);
  raw_view_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
  current_message_label_ = new QLabel("当前消息：-", central);
  setMetric(current_message_label_);

  auto * message_tabs = new QTabWidget(central);
  message_tabs->addTab(tree_view_, "结构视图");
  message_tabs->addTab(raw_view_, "原始视图");

  auto * message_panel = makePanel(central);
  auto * message_layout = new QVBoxLayout(message_panel);
  message_layout->setContentsMargins(14, 14, 14, 14);
  message_layout->setSpacing(10);
  message_layout->addWidget(makeCaption("消息查看器", message_panel));
  message_layout->addWidget(current_message_label_);
  message_layout->addWidget(message_tabs);

  auto * splitter = new QSplitter(Qt::Horizontal, central);
  splitter->addWidget(topic_panel);
  splitter->addWidget(message_panel);
  splitter->setStretchFactor(0, 3);
  splitter->setStretchFactor(1, 2);

  root_layout->addWidget(top_panel);
  root_layout->addWidget(timeline_panel);
  root_layout->addWidget(splitter);
  setCentralWidget(central);

  connect(open_button, &QPushButton::clicked, this, &MainWindow::openBag);
  connect(close_button, &QPushButton::clicked, this, &MainWindow::closeBag);
  connect(play_all_button, &QPushButton::clicked, bag_player_.get(), &BagPlayer::playAll);
  connect(stop_all_button, &QPushButton::clicked, bag_player_.get(), &BagPlayer::stopAll);
  connect(play_checked_button, &QPushButton::clicked, bag_player_.get(), &BagPlayer::playChecked);
  connect(zoom_in_button, &QPushButton::clicked, timeline_widget_, &TimelineWidget::zoomIn);
  connect(zoom_out_button, &QPushButton::clicked, timeline_widget_, &TimelineWidget::zoomOut);
  connect(fit_button, &QPushButton::clicked, timeline_widget_, &TimelineWidget::fit);
  connect(timeline_widget_, &TimelineWidget::seekRequested, this, &MainWindow::seekTo);
  connect(speed_box, &QComboBox::currentTextChanged, this, [this](const QString & text) {
    QString value = text;
    value.chop(1);
    bag_player_->setPlaybackRate(value.toDouble());
    status_label_->setText(QString("状态：速度 %1").arg(text));
  });
  connect(open_action, &QAction::triggered, this, &MainWindow::openBag);
  connect(close_action, &QAction::triggered, this, &MainWindow::closeBag);
  connect(exit_action, &QAction::triggered, this, &QWidget::close);
  connect(play_checked_action, &QAction::triggered, bag_player_.get(), &BagPlayer::playChecked);
  connect(play_all_action, &QAction::triggered, bag_player_.get(), &BagPlayer::playAll);
  connect(stop_all_action, &QAction::triggered, bag_player_.get(), &BagPlayer::stopAll);
  connect(zoom_in_action, &QAction::triggered, timeline_widget_, &TimelineWidget::zoomIn);
  connect(zoom_out_action, &QAction::triggered, timeline_widget_, &TimelineWidget::zoomOut);
  connect(fit_action, &QAction::triggered, timeline_widget_, &TimelineWidget::fit);
}

void MainWindow::openBag()
{
  const auto directory = QFileDialog::getExistingDirectory(
    this, "打开 ROS2 Bag", QString(), QFileDialog::ShowDirsOnly);
  if (directory.isEmpty()) {
    return;
  }

  QString error;
  if (!bag_player_->loadBag(directory, &error)) {
    QMessageBox::critical(this, "打开 Bag 失败", error);
  }
}

void MainWindow::closeBag()
{
  bag_player_->closeBag();
}

void MainWindow::refreshTopics()
{
  const auto topics = bag_player_->topics();
  topic_rows_.clear();
  topic_table_->setRowCount(static_cast<int>(topics.size()));
  timeline_widget_->setTimeRange(bag_player_->startTimeNs(), bag_player_->endTimeNs());
  timeline_widget_->setTopics(topics);
  timeline_widget_->setCurrentTime(bag_player_->startTimeNs());

  for (int row = 0; row < static_cast<int>(topics.size()); ++row) {
    const auto & topic = topics[row];
    topic_rows_[topic.name.toStdString()] = row;

    auto * check_container = new QWidget(topic_table_);
    auto * check_layout = new QHBoxLayout(check_container);
    check_layout->setContentsMargins(0, 0, 0, 0);
    check_layout->setAlignment(Qt::AlignCenter);
    auto * check_box = new QCheckBox(check_container);
    check_box->setChecked(topic.checked);
    check_layout->addWidget(check_box);
    topic_table_->setCellWidget(row, kCheckColumn, check_container);
    connect(check_box, &QCheckBox::toggled, this, [this, name = topic.name](bool checked) {
      bag_player_->setTopicChecked(name, checked);
      timeline_widget_->setTopics(bag_player_->topics());
    });

    topic_table_->setItem(row, kTopicColumn, new QTableWidgetItem(topic.name));
    topic_table_->setItem(row, kTypeColumn, new QTableWidgetItem(topic.type));
    topic_table_->setItem(
      row,
      kCountColumn,
      new QTableWidgetItem(QLocale().toString(static_cast<qulonglong>(topic.message_count))));
    auto * status_item = new QTableWidgetItem(topic.publisher_ready ? "已停止" : "无发布器");
    status_item->setTextAlignment(Qt::AlignCenter);
    status_item->setForeground(topic.publisher_ready ? QColor("#667085") : QColor("#b42318"));
    status_item->setBackground(topic.publisher_ready ? QColor("#f2f4f7") : QColor("#fef3f2"));
    topic_table_->setItem(row, kStatusColumn, status_item);
    topic_table_->setItem(row, kFrequencyColumn, new QTableWidgetItem(QString::number(topic.frequency_hz, 'f', 2)));
    topic_table_->item(row, kCountColumn)->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
    topic_table_->item(row, kFrequencyColumn)->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);

    auto * controls = new QWidget(topic_table_);
    auto * layout = new QHBoxLayout(controls);
    layout->setContentsMargins(4, 4, 4, 4);
    layout->setSpacing(6);
    auto * play = makeCompactButton("播放", controls, "primary");
    auto * pause = makeCompactButton("暂停", controls);
    auto * resume = makeCompactButton("恢复", controls);
    auto * stop = makeCompactButton("停止", controls, "danger");
    layout->addWidget(play);
    layout->addWidget(pause);
    layout->addWidget(resume);
    layout->addWidget(stop);
    topic_table_->setCellWidget(row, kActionColumn, controls);
    connect(play, &QPushButton::clicked, bag_player_.get(), [this, name = topic.name]() {
      bag_player_->playTopic(name);
    });
    connect(pause, &QPushButton::clicked, bag_player_.get(), [this, name = topic.name]() {
      bag_player_->pauseTopic(name);
    });
    connect(resume, &QPushButton::clicked, bag_player_.get(), [this, name = topic.name]() {
      bag_player_->resumeTopic(name);
    });
    connect(stop, &QPushButton::clicked, bag_player_.get(), [this, name = topic.name]() {
      bag_player_->stopTopic(name);
    });
  }

  topic_table_->resizeColumnsToContents();

  bag_path_label_->setText(QString("Bag：%1").arg(bag_player_->bagUri()));
  bag_info_label_->setText(QString("话题：%1 | 消息：%2 | 大小：%3 | 存储：%4 | 序列化：%5 | 时长：%6")
    .arg(topics.size())
    .arg(QLocale().toString(static_cast<qulonglong>(bag_player_->totalMessageCount())))
    .arg(formatBytes(bag_player_->bagSize()))
    .arg(bag_player_->storageId())
    .arg(bag_player_->serializationFormat())
    .arg(formatDuration(bag_player_->endTimeNs() - bag_player_->startTimeNs())));
  clock_label_->setText(QString("时钟：%1 / %2")
    .arg(formatDuration(0))
    .arg(formatDuration(bag_player_->endTimeNs() - bag_player_->startTimeNs())));
  status_label_->setText("状态：Bag 已加载");
  progress_bar_->setValue(0);
}

void MainWindow::resetUi()
{
  topic_rows_.clear();
  topic_table_->setRowCount(0);
  timeline_widget_->setTopics({});
  timeline_widget_->setTimeRange(0, 1);
  timeline_widget_->setCurrentTime(0);
  tree_model_->removeRows(0, tree_model_->rowCount());
  raw_view_->clear();
  current_message_label_->setText("当前消息：-");
  bag_path_label_->setText("Bag：未加载");
  bag_info_label_->setText("话题：0 | 消息：0 | 时间范围：-");
  clock_label_->setText("时钟：-");
  status_label_->setText("状态：等待加载");
  progress_bar_->setValue(0);
}

void MainWindow::updateTopicStatus(
  const QString & topic_name,
  const QString & status,
  uint64_t published_count,
  double frequency_hz)
{
  const int row = rowForTopic(topic_name);
  if (row < 0) {
    return;
  }
  topic_table_->item(row, kStatusColumn)->setText(
    QString("%1 (%2)").arg(status).arg(published_count));
  topic_table_->item(row, kFrequencyColumn)->setText(QString::number(frequency_hz, 'f', 2));

  QColor background("#f2f4f7");
  QColor foreground("#667085");
  if (status.startsWith("播放中")) {
    background = QColor("#ecfdf3");
    foreground = QColor("#027a48");
  } else if (status.startsWith("已暂停")) {
    background = QColor("#fffaeb");
    foreground = QColor("#b54708");
  } else if (status.startsWith("已完成")) {
    background = QColor("#eff8ff");
    foreground = QColor("#175cd3");
  } else if (status.startsWith("错误")) {
    background = QColor("#fef3f2");
    foreground = QColor("#b42318");
  }
  topic_table_->item(row, kStatusColumn)->setBackground(background);
  topic_table_->item(row, kStatusColumn)->setForeground(foreground);
  status_label_->setText(QString("状态：%1 %2").arg(topic_name, status));
}

void MainWindow::showTopicMessage(
  const QString & topic_name,
  const QString & topic_type,
  int64_t timestamp_ns,
  const QByteArray & serialized_data)
{
  current_message_label_->setText(QString("当前消息：%1 | %2 | %3")
    .arg(topic_name)
    .arg(topic_type)
    .arg(formatTime(timestamp_ns)));
  raw_view_->setPlainText(parser_.rawPreview(serialized_data));

  try {
    populateTree(parser_.parse(topic_type.toStdString(), serialized_data));
  } catch (const std::exception & ex) {
    ParsedMessageNode error;
    error.name = topic_type;
    error.type = "error";
    error.value = QString::fromUtf8(ex.what());
    populateTree(error);
  }
}

void MainWindow::updateClock(int64_t timestamp_ns)
{
  const auto relative_ns = timestamp_ns - bag_player_->startTimeNs();
  clock_label_->setText(QString("时钟：%1 / %2")
    .arg(formatDuration(relative_ns))
    .arg(formatDuration(bag_player_->endTimeNs() - bag_player_->startTimeNs())));
  timeline_widget_->setCurrentTime(timestamp_ns);
}

void MainWindow::updateProgress(double progress)
{
  progress_bar_->setValue(static_cast<int>(std::clamp(progress, 0.0, 1.0) * 1000.0));
}

void MainWindow::seekTo(int64_t timestamp_ns)
{
  bag_player_->seekTo(timestamp_ns);
  timeline_widget_->setCurrentTime(timestamp_ns);
  status_label_->setText(QString("状态：跳转到 %1")
    .arg(formatDuration(timestamp_ns - bag_player_->startTimeNs())));
}

void MainWindow::populateTree(const ParsedMessageNode & node)
{
  tree_model_->removeRows(0, tree_model_->rowCount());
  addTreeNode(tree_model_->invisibleRootItem(), node);
  tree_view_->expandToDepth(2);
  tree_view_->resizeColumnToContents(0);
  tree_view_->resizeColumnToContents(1);
}

void MainWindow::addTreeNode(QStandardItem * parent, const ParsedMessageNode & node)
{
  QList<QStandardItem *> row;
  row << new QStandardItem(node.name);
  row << new QStandardItem(node.type);
  row << new QStandardItem(node.value);
  parent->appendRow(row);
  auto * item = row.front();
  for (const auto & child : node.children) {
    addTreeNode(item, child);
  }
}

QString MainWindow::formatTime(int64_t timestamp_ns) const
{
  const int64_t seconds = timestamp_ns / 1000000000LL;
  const int64_t nanoseconds = timestamp_ns % 1000000000LL;
  return QString("%1.%2").arg(seconds).arg(nanoseconds, 9, 10, QLatin1Char('0'));
}

QString MainWindow::formatDuration(int64_t duration_ns) const
{
  duration_ns = std::max<int64_t>(0, duration_ns);
  const int64_t total_ms = duration_ns / 1000000LL;
  const int64_t hours = total_ms / 3600000LL;
  const int64_t minutes = (total_ms / 60000LL) % 60LL;
  const int64_t seconds = (total_ms / 1000LL) % 60LL;
  const int64_t millis = total_ms % 1000LL;

  if (hours > 0) {
    return QString("%1:%2:%3.%4")
      .arg(hours)
      .arg(minutes, 2, 10, QLatin1Char('0'))
      .arg(seconds, 2, 10, QLatin1Char('0'))
      .arg(millis, 3, 10, QLatin1Char('0'));
  }
  return QString("%1:%2.%3")
    .arg(minutes, 2, 10, QLatin1Char('0'))
    .arg(seconds, 2, 10, QLatin1Char('0'))
    .arg(millis, 3, 10, QLatin1Char('0'));
}

QString MainWindow::formatBytes(uint64_t bytes) const
{
  if (bytes == 0) {
    return "-";
  }

  static const QStringList units = {"B", "KB", "MB", "GB", "TB"};
  double value = static_cast<double>(bytes);
  int unit_index = 0;
  while (value >= 1024.0 && unit_index < units.size() - 1) {
    value /= 1024.0;
    ++unit_index;
  }
  return QString("%1 %2").arg(value, 0, unit_index == 0 ? 'f' : 'f', unit_index == 0 ? 0 : 2).arg(units[unit_index]);
}

int MainWindow::rowForTopic(const QString & topic_name) const
{
  const auto found = topic_rows_.find(topic_name.toStdString());
  if (found == topic_rows_.end()) {
    return -1;
  }
  return found->second;
}

}  // namespace robot_bag_play_tool
