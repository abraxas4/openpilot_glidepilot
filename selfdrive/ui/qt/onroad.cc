#include "selfdrive/ui/qt/onroad.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <sstream>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>

#include "common/swaglog.h"
#include "common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map_panel.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

#include "system/hardware/hw.h"

static void drawIcon(QPainter &p, const QPoint &center, const QPixmap &img, const QBrush &bg, float opacity) {
  p.setRenderHint(QPainter::Antialiasing);
  p.setOpacity(1.0);  // bg dictates opacity of ellipse
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(center, btn_size / 2, btn_size / 2);
  p.setOpacity(opacity);
  p.drawPixmap(center - QPoint(img.width() / 2, img.height() / 2), img);
  p.setOpacity(1.0);
}

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  nvg = new AnnotatedCameraWidget(VISION_STREAM_ROAD, this);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addWidget(nvg);

  if (getenv("DUAL_CAMERA_VIEW")) {
    CameraWidget *arCam = new CameraView("camerad", VISION_STREAM_ROAD, true, this);
    split->insertWidget(0, arCam);
  }

  if (getenv("MAP_RENDER_VIEW")) {
    CameraWidget *map_render = new CameraView("navd", VISION_STREAM_MAP, false, this);
    split->insertWidget(0, map_render);
  }

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);
  QObject::connect(uiState(), &UIState::primeChanged, this, &OnroadWindow::primeChanged);
}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  alerts->updateAlert(alert);

  if (s.scene.map_on_left) {
    split->setDirection(QBoxLayout::LeftToRight);
  } else {
    split->setDirection(QBoxLayout::RightToLeft);
  }

  nvg->updateState(s);

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
#ifdef ENABLE_MAPS
  if (map != nullptr) {
    // Switch between map and sidebar when using navigate on openpilot
    bool sidebarVisible = geometry().x() > 0;
    bool show_map = /*uiState()->scene.navigate_on_openpilot ? sidebarVisible :*/ !sidebarVisible;
    map->setVisible(show_map && !map->isVisible());
  }
#endif
  // propagation event to parent(HomeWindow)
  QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->hasPrime() || !MAPBOX_TOKEN.isEmpty())) {
      auto m = new MapPanel(get_mapbox_settings());
      map = m;

      QObject::connect(m, &MapPanel::mapPanelRequested, this, &OnroadWindow::mapPanelRequested);
      QObject::connect(nvg->map_settings_btn, &MapSettingsButton::clicked, m, &MapPanel::toggleMapSettings);
      nvg->map_settings_btn->setEnabled(!Params().getBool("UseExternalNaviRoutes"));

      m->setFixedWidth(topWidget(this)->width() / 2 - UI_BORDER_SIZE);
      split->insertWidget(0, m);

      // hidden by default, made visible when navRoute is published
      m->setVisible(false);
    }
  }
#endif

  alerts->updateAlert({});
  nvg->disconnectVipc();
}

void OnroadWindow::primeChanged(bool prime) {
#ifdef ENABLE_MAPS
  if (map && (!prime && MAPBOX_TOKEN.isEmpty())) {
    nvg->map_settings_btn->setEnabled(false);
    nvg->map_settings_btn->setVisible(false);
    map->deleteLater();
    map = nullptr;
  }
#endif
}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}


// MJ
// OnroadAlerts 클래스는 경고 메시지를 업데이트하고 표시하는 기능을 담당합니다.

// ***** onroad widgets *****


// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a) {
  // 만약 새로운 경고 메시지가 있다면 화면을 업데이트합니다.
  if (!alert.equal(a)) {
    alert = a;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_heights = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_heights[alert.size];

  int margin = 40;
  int radius = 30;
  if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    margin = 0;
    radius = 0;
  }
  QRect r = QRect(0 + margin, height() - h + margin, width() - margin*2, h - margin*2);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);
  p.setBrush(QBrush(alert_colors[alert.status]));
  p.drawRoundedRect(r, radius, radius);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.drawRoundedRect(r, radius, radius);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    p.setFont(InterFont(74, QFont::DemiBold));
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    p.setFont(InterFont(88, QFont::Bold));
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    p.setFont(InterFont(66, QFont::Normal));
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    p.setFont(InterFont(l ? 132 : 177, QFont::Bold));
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    p.setFont(InterFont(88, QFont::Normal));
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// ExperimentalButton
ExperimentalButton::ExperimentalButton(QWidget *parent) : experimental_mode(false), engageable(false), QPushButton(parent) {
  setFixedSize(btn_size, btn_size);

  engage_img = loadPixmap("../assets/img_chffr_wheel.png", {img_size, img_size});
  experimental_img = loadPixmap("../assets/img_experimental.svg", {img_size, img_size});
  QObject::connect(this, &QPushButton::clicked, this, &ExperimentalButton::changeMode);
}

void ExperimentalButton::changeMode() {
  const auto cp = (*uiState()->sm)["carParams"].getCarParams();
  bool can_change = hasLongitudinalControl(cp) && params.getBool("ExperimentalModeConfirmed");
  if (can_change) {
    params.putBool("ExperimentalMode", !experimental_mode);
  }
}

void ExperimentalButton::updateState(const UIState &s) {
  const auto cs = (*s.sm)["controlsState"].getControlsState();
  bool eng = cs.getEngageable() || cs.getEnabled();
  if ((cs.getExperimentalMode() != experimental_mode) || (eng != engageable)) {
    engageable = eng;
    experimental_mode = cs.getExperimentalMode();
    update();
  }
}

void ExperimentalButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  QPixmap img = experimental_mode ? experimental_img : engage_img;
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), img, QColor(0, 0, 0, 166), (isDown() || !engageable) ? 0.6 : 1.0);
}

// MapSettingsButton
MapSettingsButton::MapSettingsButton(QWidget *parent) : QPushButton(parent) {
  setFixedSize(btn_size, btn_size);
  settings_img = loadPixmap("../assets/navigation/icon_directions_outlined.svg", {img_size, img_size});

  // hidden by default, made visible if map is created (has prime or mapbox token)
  setVisible(false);
  setEnabled(false);
}

void MapSettingsButton::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  drawIcon(p, QPoint(btn_size / 2, btn_size / 2), settings_img, QColor(0, 0, 0, 166), isDown() ? 0.6 : 1.0);
}


AnnotatedCameraWidget::AnnotatedCameraWidget(VisionStreamType type, QWidget* parent) : last_update_params(0), fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraWidget("camerad", type, true, parent) {
  pm = std::make_unique<PubMaster, const std::initializer_list<const char *>>({"uiDebug"});

  main_layout = new QVBoxLayout(this);
  main_layout->setMargin(UI_BORDER_SIZE);
  main_layout->setSpacing(0);

  experimental_btn = new ExperimentalButton(this);
  main_layout->addWidget(experimental_btn, 0, Qt::AlignTop | Qt::AlignRight);

  map_settings_btn = new MapSettingsButton(this);

  if(!Params().getBool("UseExternalNaviRoutes"))
    main_layout->addWidget(map_settings_btn, 0, Qt::AlignBottom | Qt::AlignRight);

  dm_img = loadPixmap("../assets/img_driver_face.png", {img_size + 5, img_size + 5});

  // neokii
  ic_brake = QPixmap("../assets/images/img_brake_disc.png").scaled(img_size, img_size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  ic_nda2 = QPixmap("../assets/images/img_nda2.png");
  ic_hda2 = QPixmap("../assets/images/img_hda2.png");
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_turn_signal_l = QPixmap("../assets/images/turn_signal_l.png");
  ic_turn_signal_r = QPixmap("../assets/images/turn_signal_r.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");

  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);

	recorder = new ScreenRecoder(this);
	main_layout->addWidget(recorder, 0, Qt::AlignBottom | Qt::AlignRight);
}

void AnnotatedCameraWidget::initializeGL() {
  CameraWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);
}

void AnnotatedCameraWidget::updateState(const UIState &s) {
  const SubMaster &sm = *(s.sm);

  experimental_btn->updateState(s);

  const auto cs = sm["controlsState"].getControlsState();

  // update DM icons
  auto dm_state = sm["driverMonitoringState"].getDriverMonitoringState();
  dmActive = dm_state.getIsActiveMode();
  rightHandDM = dm_state.getIsRHD();

  hideBottomIcons = (cs.getAlertSize() != cereal::ControlsState::AlertSize::NONE);
  dm_fade_state = std::clamp(dm_fade_state+0.2*(0.5-dmActive), 0.0, 1.0);

  // hide map settings button for alerts and flip for right hand DM
  if (map_settings_btn->isEnabled()) {
    map_settings_btn->setVisible(!hideBottomIcons);
    main_layout->setAlignment(map_settings_btn, (rightHandDM ? Qt::AlignLeft : Qt::AlignRight) | Qt::AlignBottom);
  }

  update();
}

void AnnotatedCameraWidget::updateFrameMat() {
  CameraWidget::updateFrameMat();
  UIState *s = uiState();
  int w = width(), h = height();

  s->fb_w = w;
  s->fb_h = h;

  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2 - x_offset, h / 2 - y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

// MJ
// 차선을 그리는 함수입니다. 차선의 위치와 확률에 따라 차선을 그리고 색상과 불투명도를 조절합니다.
void AnnotatedCameraWidget::drawLaneLines(QPainter &painter, const UIState *s) {
  painter.save();

  const UIScene &scene = s->scene;
  SubMaster &sm = *(s->sm);

  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i]);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i]);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, 0);
  if (sm["controlsState"].getControlsState().getExperimentalMode()) {
    // The first half of track_vertices are the points for the right side of the path
    // and the indices match the positions of accel from uiPlan
    const auto &acceleration = sm["uiPlan"].getUiPlan().getAccel();
    const int max_len = std::min<int>(scene.track_vertices.length() / 2, acceleration.size());

    for (int i = 0; i < max_len; ++i) {
      // Some points are out of frame
      if (scene.track_vertices[i].y() < 0 || scene.track_vertices[i].y() > height()) continue;

      // Flip so 0 is bottom of frame
      float lin_grad_point = (height() - scene.track_vertices[i].y()) / height();

      // speed up: 120, slow down: 0
      float path_hue = fmax(fmin(60 + acceleration[i] * 35, 120), 0);
      // FIXME: painter.drawPolygon can be slow if hue is not rounded
      path_hue = int(path_hue * 100 + 0.5) / 100;

      float saturation = fmin(fabs(acceleration[i] * 1.5), 1);
      float lightness = util::map_val(saturation, 0.0f, 1.0f, 0.95f, 0.62f);  // lighter when grey
      float alpha = util::map_val(lin_grad_point, 0.75f / 2.f, 0.75f, 0.4f, 0.0f);  // matches previous alpha fade
      bg.setColorAt(lin_grad_point, QColor::fromHslF(path_hue / 360., saturation, lightness, alpha));

      // Skip a point, unless next is last
      i += (i + 2) < max_len ? 1 : 0;
    }

  } else {
    bg.setColorAt(0.0, QColor::fromHslF(148 / 360., 0.94, 0.51, 0.4));
    bg.setColorAt(0.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.35));
    bg.setColorAt(1.0, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.0));
  }
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices);

  painter.restore();
}

// MJ
// 이 함수는 주행 중인 차량의 UI에, 
// 앞 차량과의 상대적 거리와 속도에 따라, 다른 시각적 효과를 나타내기 위한 로직을 포함하고 있으며, 
// 앞 차량이 가까워질수록 더 밝고 눈에 띄는 색상으로 표시됩니다. 
// 또한 레이더 센서에 의해 감지된 차량인지 아닌지에 따라 색상을 달리하여 사용자에게 시각적 구분을 제공합니다.
void AnnotatedCameraWidget::drawLead(QPainter &painter, const cereal::RadarState::LeadData::Reader &lead_data, const QPointF &vd, bool is_radar) {
  painter.save(); // QPainter 상태 저장. 이후 restore()가 호출되면 여기서 저장된 상태로 복원됨.

  // 속도와 거리에 따른 색상의 투명도를 계산하기 위한 버퍼 값 설정.
  const float speedBuff = 10.;
  const float leadBuff = 40.;

  // 실제 차량과 앞 차량의 상대 거리와 상대 속도를 가져옴.
  const float d_rel = lead_data.getDRel();
  const float v_rel = lead_data.getVRel();

  float fillAlpha = 0; // 초기 투명도를 0으로 설정.
  if (d_rel < leadBuff) { // 만약 앞 차량과의 거리가 leadBuff보다 작으면
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff)); // 거리에 따라 투명도 증가.
    if (v_rel < 0) {
      // 앞 차량이 접근 중이라면(상대 속도가 음수) 투명도를 추가로 증가.
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255)); // 투명도를 255 이하로 제한.
  }

  // 앞 차량과의 거리에 따라 아이콘 크기 조정.
  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  // 아이콘 위치 조정.
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  // 반짝이는 효과를 위한 X와 Y의 오프셋 설정.
  float g_xo = sz / 5;
  float g_yo = sz / 10;

  // 앞 차량 표시를 위한 반짝이는 도형을 그림.
  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(is_radar ? QColor(86, 121, 216, 255) : QColor(218, 202, 37, 255)); // 레이더 여부에 따라 색상 선택.
  painter.drawPolygon(glow, std::size(glow)); // 반짝이는 효과 도형을 그림.

  // 체브론 모양의 도형(앞 차량 표시)을 그림.
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  painter.setBrush(redColor(fillAlpha)); // 거리에 따른 색상과 투명도 설정.
  painter.drawPolygon(chevron, std::size(chevron)); // 체브론 도형을 그림.
  #if 0
  chevron이라는 배열은 세 개의 점으로 정의되어 있으며, 
  이 세 점은 QPainter를 사용하여 하나의 역삼각형을 그리는 데 사용됩니다. 
  즉, 이 코드에서는 drawPolygon 함수를 통해 하나의 chevron 역삼각형을 그리고 있습니다.
  #endif

  painter.restore(); // QPainter 상태 복원. 이전에 save()로 저장된 상태로 되돌림.
}

void AnnotatedCameraWidget::paintGL() {
}

void AnnotatedCameraWidget::paintEvent(QPaintEvent *event) {
  UIState *s = uiState();
  SubMaster &sm = *(s->sm);
  const double start_draw_t = millis_since_boot();
  const cereal::ModelDataV2::Reader &model = sm["modelV2"].getModelV2();

  QPainter p(this);

  // Wide or narrow cam dependent on speed
  bool has_wide_cam = available_streams.count(VISION_STREAM_WIDE_ROAD);
  if (has_wide_cam) {
    float v_ego = sm["carState"].getCarState().getVEgo();
    if ((v_ego < 10) || available_streams.size() == 1) {
      wide_cam_requested = true;
    } else if (v_ego > 15) {
      wide_cam_requested = false;
    }
    wide_cam_requested = wide_cam_requested && sm["controlsState"].getControlsState().getExperimentalMode();
    // for replay of old routes, never go to widecam
    wide_cam_requested = wide_cam_requested && s->scene.calibration_wide_valid;
  }
  CameraWidget::setStreamType(wide_cam_requested ? VISION_STREAM_WIDE_ROAD : VISION_STREAM_ROAD);

  s->scene.wide_cam = CameraWidget::streamType() == VISION_STREAM_WIDE_ROAD;
  if (s->scene.calibration_valid) {
    auto calib = s->scene.wide_cam ? s->scene.view_from_wide_calib : s->scene.view_from_calib;
    CameraWidget::updateCalibration(calib);
  } else {
    CameraWidget::updateCalibration(DEFAULT_CALIBRATION);
  }

  if (!CameraWidget::receiveFrame(sm["uiPlan"].getUiPlan().getFrameId())) {
    qDebug() << "skipping frame, not ready";
    return;
  }

  p.beginNativePainting();
  //CameraWidget::setFrameId(model.getFrameId());
  CameraWidget::paintGL();
  p.endNativePainting();

  if (s->scene.world_objects_visible) {
    update_model(s, model, sm["uiPlan"].getUiPlan());
    drawHud(p, model);

    // DMoji
    if (!hideBottomIcons && (sm.rcv_frame("driverStateV2") > s->scene.started_frame)) {
      update_dmonitoring(s, sm["driverStateV2"].getDriverStateV2(), dm_fade_state, false);
      drawDriverState(p, s);
    }
  }

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;

  // publish debug msg
  MessageBuilder msg;
  auto m = msg.initEvent().initUiDebug();
  m.setDrawTimeMillis(cur_draw_t - start_draw_t);
  pm->send("uiDebug", msg);
}

void AnnotatedCameraWidget::showEvent(QShowEvent *event) {
  CameraWidget::showEvent(event);

  auto now = millis_since_boot();
  if(now - last_update_params > 1000*5) {
    last_update_params = now;
    ui_update_params(uiState());
  }

  prev_draw_t = millis_since_boot();
}

void AnnotatedCameraWidget::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QRect real_rect = p.fontMetrics().boundingRect(text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void AnnotatedCameraWidget::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

// MJ
// 이 코드는 차량의 HUD(Head-Up Display, 헤드업 디스플레이)에 정보를 그리는 함수입니다. 
// HUD는 운전자가 도로를 주시하면서도 차량의 상태나 운전 관련 정보를 쉽게 볼 수 있게 해주는 시스템입니다. 
// 이 함수는 차량의 속도, 차선 정보, 앞차와의 거리 등을 그래픽으로 표현합니다.
void AnnotatedCameraWidget::drawHud(QPainter &p, const cereal::ModelDataV2::Reader &model) {
  // 안티앨리어싱을 활성화하여 그래픽의 선이 부드럽게 보이도록 합니다.
  p.setRenderHint(QPainter::Antialiasing);
  // 펜을 비활성화하여 선을 그리지 않고, 브러시로만 그림을 그립니다.
  p.setPen(Qt::NoPen);
  // 페인터의 불투명도를 완전히 불투명하게 설정합니다.
  p.setOpacity(1.0);

  // Header gradient
  // 화면 상단에 그라데이션 배경을 그립니다.
  QLinearGradient bg(0, UI_HEADER_HEIGHT - (UI_HEADER_HEIGHT / 2.5), 0, UI_HEADER_HEIGHT);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45)); // 그라데이션의 시작 색상
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));     // 그라데이션의 끝 색상
  p.fillRect(0, 0, width(), UI_HEADER_HEIGHT, bg);    // 상단 배경을 그림

  UIState *s = uiState(); // UI 상태를 가져옵니다.
  const SubMaster &sm = *(s->sm); // 데이터를 가져올 SubMaster 객체에 대한 참조를 가져옵니다.

  // 차선 라인을 그립니다.
  drawLaneLines(p, s);

  // 레이더 상태를 업데이트하고 모델에서 위치 데이터를 가져옵니다.
  auto radar_state = sm["radarState"].getRadarState();
  update_leads(s, radar_state, model.getPosition());
  
  // 앞차(Lead) 차량에 대한 정보를 가져와 그립니다.
  auto lead_one = radar_state.getLeadOne(); // 첫 번째 앞차 데이터
  auto lead_two = radar_state.getLeadTwo(); // 두 번째 앞차 데이터
  if (lead_two.getStatus()) {
    // 두 번째 앞차가 감지되었을 때 해당 위치에 역삼각형(chevron)을 그립니다.
    drawLead(p, lead_two, s->scene.lead_vertices[1], s->scene.lead_radar[1]);
  }
  if (lead_one.getStatus()) {
    // 첫 번째 앞차가 감지되었을 때 해당 위치에 역삼각형(chevron)을 그립니다.
    drawLead(p, lead_one, s->scene.lead_vertices[0], s->scene.lead_radar[0]);
  }

  // HUD에 표시될 다양한 정보들을 그립니다.
  drawMaxSpeed(p);     // 설정된 최고 속도
  drawSpeed(p);        // 현재 속도
  drawSteer(p);        // 조향 각도
  drawDeviceState(p);  // 디바이스 상태 (예: 저장 공간, CPU 온도 등)
  //drawTurnSignals(p);
  drawGpsStatus(p);    // GPS 상태
  drawMisc(p);         // 기타 정보
  drawDebugText(p);    // 디버깅을 위한 텍스트

  const auto controls_state = sm["controlsState"].getControlsState();
  //const auto car_params = sm["carParams"].getCarParams();
  const auto live_params = sm["liveParameters"].getLiveParameters();
  const auto car_control = sm["carControl"].getCarControl();
  const auto live_torque_params = sm["liveTorqueParameters"].getLiveTorqueParameters();
  const auto torque_state = controls_state.getLateralControlState().getTorqueState();

  // 차량의 제어 상태 및 파라미터 정보를 포맷팅하여 정보 텍스트로 설정합니다.
  QString infoTextEng; // 영어 정보 텍스트
  QString infoTextKor; // 한국어 정보 텍스트

  // 영어 로그 문자열 (원본 로그 포맷)
  infoTextEng.sprintf("TP(%.2f/%.2f) LTP(%.2f/%.2f/%.0f) AO(%.2f/%.2f) SR(%.2f) SAD(%.2f) SCC(%d)",
                    torque_state.getLatAccelFactor(),
                    torque_state.getFriction(),

                    live_torque_params.getLatAccelFactorRaw(),
                    live_torque_params.getFrictionCoefficientRaw(),
                    live_torque_params.getTotalBucketPoints(),

                    live_params.getAngleOffsetDeg(),
                    live_params.getAngleOffsetAverageDeg(),

                    car_control.getSteerRatio(),
                    car_control.getSteerActuatorDelay(),

                    car_control.getSccBus());

  // 한국어 로그 문자열 (번역된 로그 포맷)
  infoTextKor.sprintf("토크 P(%.2f/%.2f), 토크 I(%.2f/%.2f/%.0f), 각도 오프셋(%.2f/%.2f), 조향비(%.2f), 조향 액츄에이터 지연(%.2f), SCC(%d)",
                    torque_state.getLatAccelFactor(),
                    torque_state.getFriction(),

                    live_torque_params.getLatAccelFactorRaw(),
                    live_torque_params.getFrictionCoefficientRaw(),
                    live_torque_params.getTotalBucketPoints(),

                    live_params.getAngleOffsetDeg(),
                    live_params.getAngleOffsetAverageDeg(),

                    car_control.getSteerRatio(),
                    car_control.getSteerActuatorDelay(),

                    car_control.getSccBus());

  p.save(); // 현재 페인터 상태를 저장합니다.

  // MJ 
  int font_size = 34 * 1.20; // 원래 글꼴 크기의 120%로 설정
  p.setFont(InterFont(font_size, QFont::Normal)); // 글꼴 크기를 조정합니다.
  p.setPen(QColor(0xff, 0xff, 0xff, 200)); // 펜의 색상을 설정합니다.

  // MJ
  // 글꼴의 크기를 20% 증가시켜서, 보다 크게 정보를 표시할 수 있도록 하였습니다. 
  // 또한, 텍스트가 화면에 중앙에 오도록 가로 위치를 조정하였고, 
  // 텍스트가 하단 가장자리와 충돌하지 않도록 Y 위치를 조정하였습니다.
  // 정보 텍스트가 화면의 중앙에 오도록 가로 위치를 조정합니다.
  // 화면의 가로 너비에 대해 텍스트 너비를 20% 늘린 값을 기준으로 가운데 정렬합니다.

  // 텍스트가 화면 하단에 나타나도록 Y 위치를 조정합니다.
  // 글꼴 크기에 따라 하단에서 여백을 설정합니다.
  int text_y_offset = font_size + 10; // 영어와 한국어 로그 사이의 간격
  int text_y = rect().height() - (font_size * 2 + text_y_offset + 15); // 영어 로그 시작 위치
  // 영어 로그를 화면 하단에 그립니다.
  p.drawText(QRect(0, text_y, width(), font_size), Qt::AlignCenter, infoTextEng);
  // 한국어 로그를 영어 로그 바로 아래에 그립니다.
  text_y += font_size + text_y_offset; // 한국어 로그 시작 위치를 조정합니다.
  p.drawText(QRect(0, text_y, width(), font_size), Qt::AlignCenter, infoTextKor);

  p.restore(); // 저장된 페인터 상태를 복구합니다.

  drawBottomIcons(p); // 화면 하단에 아이콘들을 그립니다.
}

// MJ : color during accel/decel
void AnnotatedCameraWidget::drawSpeed(QPainter &p) {
  p.save();
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getVEgoCluster() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH));
  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  #if 0  
  코드는 투명도가 있는 기본 흰색을 정의한 다음 
  차량의 가속도에 따라 "속도 표시의 색상"이 변하는, 시각적 효과를 만들고 있습니다. 
  차량이 가속할 때(accel > 0), 색상은 파란색조로 변하고, (a, a, 255, 230)
  차량이 감속할 때(accel < 0), 색상은 붉은색조로 변합니다. (255, a, a, 230)
  4번째 값 : 0 == 투명 / 255 == 불투명
  #endif

  QColor color = QColor(255, 255, 255, 230);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 230); // MJ : color : bluish
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 230); // MJ : color : reddish
  }

  QString speed;
  speed.sprintf("%.0f", cur_speed);
  p.setFont(InterFont(176, QFont::Bold));
  drawTextWithColor(p, rect().center().x(), 230, speed, color);

  p.setFont(InterFont(66, QFont::Normal));
  drawText(p, rect().center().x(), 310, s->scene.is_metric ? "km/h" : "mph", 200);

  p.restore();
}

QRect getRect(QPainter &p, int flags, QString text) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  return fm.boundingRect(init_rect, flags, text);
}

#if 0
크루즈 컨트롤 설정 속도 표시:
bool is_cruise_set = cruiseState.getEnabled();
이 조건은 크루즈 컨트롤이 활성화되어 있는지를 확인합니다. 
is_cruise_set이 
  true이면 크루즈 컨트롤로 설정된 최고 속도(cruiseMaxSpeed)를 표시하고, 
  false이면 "N/A"로 표시하여 사용자에게 크루즈 컨트롤이 설정되지 않았음을 알립니다.

적용된 최대 속도 표시:
이 부분은 시스템이 자동으로 적용한 최대 속도(applyMaxSpeed)를 표시합니다. 
크루즈 컨트롤이 설정되어 있고 applyMaxSpeed가 0보다 클 경우 해당 속도를 표시하고, 
그렇지 않으면 "MAX"로 표시합니다.

메트릭 단위 변환:
bool is_metric = s->scene.is_metric;
사용자가 선택한 단위에 따라 속도 값을 km/h 또는 mph로 변환하여 표시합니다. 
is_metric이 true면 km/h 단위를, false면 mph 단위를 사용합니다.

내비게이션 데이터 표시:
카메라 또는 구간 제한 속도(camLimitSpeed, sectionLimitSpeed)와 관련된 데이터를 표시합니다. 
각각의 제한 속도와 남은 거리(camLimitSpeedLeftDist, sectionLeftDist)를 확인하여 화면에 그립니다.

내비게이션 디스플레이 활성화 상태 표시:
int activeNDA = navi_data.getActive();
내비게이션 디스플레이 어시스트(NDA)가 활성화되어 있을 경우 관련 아이콘을 그립니다. 
activeNDA 값에 따라 NDA 또는 HDA 아이콘을 화면에 표시합니다.
#endif
void AnnotatedCameraWidget::drawMaxSpeed(QPainter &p) {
  p.save(); // QPainter 상태를 저장합니다.

  // UI 상태 및 데이터를 가져옵니다.
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto car_control = sm["carControl"].getCarControl(); // 차량 제어 데이터를 가져옵니다.
  const auto car_state = sm["carState"].getCarState(); // 차량 상태 데이터를 가져옵니다.
  //const auto car_params = sm["carParams"].getCarParams(); // 차량 파라미터 데이터를 가져옵니다(현재 주석 처리).
  const auto navi_data = sm["naviData"].getNaviData(); // 내비게이션 데이터를 가져옵니다.

  // 크루즈 컨트롤 상태를 가져옵니다.
  const auto cruiseState = car_state.getCruiseState();

  // 메트릭 단위 사용 여부를 확인합니다.
  bool is_metric = s->scene.is_metric; // 메트릭 단위 사용 여부 (true: km/h, false: mph)
  //int scc_bus = car_params.getSccBus();

  // kph로 속도 값을 가져옵니다.
  float applyMaxSpeed = car_control.getApplyMaxSpeed(); // 적용할 최고 속도
  float cruiseMaxSpeed = car_control.getCruiseMaxSpeed(); // 크루즈 컨트롤로 설정된 최고 속도

  // 크루즈 컨트롤 설정 여부를 확인합니다.
  bool is_cruise_set = cruiseState.getEnabled(); // 크루즈 컨트롤이 설정되었는지 여부

  // 내비게이션 관련 데이터를 가져옵니다.
  int activeNDA = navi_data.getActive();
  int roadLimitSpeed = navi_data.getRoadLimitSpeed(); // 도로 제한 속도
  int camLimitSpeed = navi_data.getCamLimitSpeed(); // 카메라 제한 속도
  int camLimitSpeedLeftDist = navi_data.getCamLimitSpeedLeftDist(); // 카메라 제한 속도까지 남은 거리
  int sectionLimitSpeed = navi_data.getSectionLimitSpeed(); // 구간 제한 속도
  int sectionLeftDist = navi_data.getSectionLeftDist(); // 구간 제한 속도까지 남은 거리
  int isNda2 = navi_data.getIsNda2(); // NDA2 활성화 여부

  // 최고 속도와 남은 거리를 초기화합니다.
  int limit_speed = 0;
  int left_dist = 0;

  // 카메라나 구간 제한 속도가 설정되어 있다면 해당 값으로 설정합니다.
  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  // NDA가 활성화되어 있다면 관련 이미지를 그립니다.
  if(activeNDA > 0) {
      p.setOpacity(1.f); // 투명도 설정
      // NDA 아이콘을 그립니다.
      if(isNda2) {
        // 아이콘 크기 및 위치 설정
        int w = 155;
        int h = 54;
        int x = (width() + (UI_BORDER_SIZE*2))/2 - w/2 - UI_BORDER_SIZE;
        int y = 40 - UI_BORDER_SIZE;
        // 아이콘을 그립니다.
        p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda2 : ic_hda2);
      }
      else {
        // 아이콘 크기 및 위치 설정
        int w = 120;
        int h = 54;
        int x = (width() + (UI_BORDER_SIZE*2))/2 - w/2 - UI_BORDER_SIZE;
        int y = 40 - UI_BORDER_SIZE;
        // 아이콘을 그립니다.
        p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
      }
  }
  else {
    // NDA가 활성화되어 있지 않다면 차량 상태에서 내비게이션 속도 제한을 가져옵니다.
    limit_speed = car_state.getNavSpeedLimit();
  }

  // UI의 시작 위치를 설정합니다.
  const int x_start = 30;
  const int y_start = 30;

  // 속도 표시판의 크기를 설정합니다.
  int board_width = 210;
  int board_height = 384;

  // 모서리의 둥근 정도를 설정합니다.
  const int corner_radius = 32;
  int max_speed_height = 210;

  // 배경 색상과 투명도를 설정합니다.
  QColor bgColor = QColor(0, 0, 0, 166);
  QString str; // 표시할 문자열을 저장할 변수
  // 크루즈 컨트롤 설정 속도와 적용된 최대 속도를 왼쪽 정렬로 그립니다.
  // "SET" 부분은 원래 크기로 유지하고 "km/h"와 "mph" 부분은 더 작게 표시합니다.
  {
    QFont original_font = p.font(); // 원래 폰트를 저장해둡니다.    
    QFont speed_font = InterFont(50, QFont::Bold); // 속도를 나타내는 폰트를 설정합니다.
    QFont unit_font = InterFont(25, QFont::Bold); // 단위(km/h)를 나타내는 폰트를 설정합니다 (속도 폰트의 50%).

    // "SET" 문자열을 그립니다.
    if(is_cruise_set) {
      float display_speed = is_metric ? cruiseMaxSpeed : cruiseMaxSpeed * KM_TO_MILE;
      str.sprintf("SET=%d", (int)(display_speed + 0.5)); // 속도 값을 문자열로 설정합니다.
    } else {
      str = "SET=N/A"; // 크루즈 컨트롤이 설정되지 않았을 때
    }
    p.setFont(speed_font); // 속도 폰트를 적용합니다.
    p.setPen(QColor(255, 255, 255, 230)); // 펜 색상 및 투명도 설정
    p.drawText(QRect(x_start, y_start, 1000, 50), Qt::AlignLeft | Qt::AlignVCenter, str);

    // "km/h" 단위를 그립니다.
    p.setFont(unit_font); // 단위 폰트를 적용합니다.
    str = is_metric ? "km/h" : "mph";
    int unit_str_width = p.fontMetrics().horizontalAdvance(str);
    p.drawText(QRect(x_start + p.fontMetrics().horizontalAdvance(str) + 5, y_start, 1000, 25), Qt::AlignLeft | Qt::AlignVCenter, str);

    // "APPLIED" 문자열을 그립니다.
    int second_line_y_start = y_start + 50 + p.fontMetrics().height(); // Adjust Y start for the second line to avoid overlap
    if(is_cruise_set && applyMaxSpeed > 0) {
      float display_speed = is_metric ? applyMaxSpeed : applyMaxSpeed * KM_TO_MILE;
      str.sprintf("APPLIED=%d", (int)(display_speed + 0.5)); // 속도 값을 문자열로 설정합니다.
    } else {
      str = "APPLIED=MAX"; // 적용된 최대 속도가 설정되지 않았을 때
    }
    p.setFont(speed_font); // 속도 폰트를 적용합니다.
    p.setPen(QColor(255, 255, 255, 180)); // 펜 색상 및 투명도 설정
    p.drawText(QRect(x_start, second_line_y_start, 1000, 50), Qt::AlignLeft | Qt::AlignVCenter, str);

    // "km/h" 단위를 그립니다.
    p.setFont(unit_font); // 단위 폰트를 적용합니다.
    p.drawText(QRect(x_start + unit_str_width + 5, second_line_y_start, 1000, 25), Qt::AlignLeft | Qt::AlignVCenter, str);

    // 폰트를 원래대로 복구합니다.
    p.setFont(original_font);

    p.restore();
  }
  //
  if(limit_speed > 0) {
    QRect board_rect = QRect(x_start, y_start+board_height-board_width, board_width, board_width);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawEllipse(board_rect);

    padding = 18;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::red, 25));
    p.drawEllipse(board_rect);

    p.setPen(QPen(Qt::black, padding));

    str.sprintf("%d", limit_speed);
    p.setFont(InterFont(70, QFont::Bold));

    QRect text_rect = getRect(p, Qt::AlignCenter, str);
    QRect b_rect = board_rect;
    text_rect.moveCenter({b_rect.center().x(), 0});
    text_rect.moveTop(b_rect.top() + (b_rect.height() - text_rect.height()) / 2);
    p.drawText(text_rect, Qt::AlignCenter, str);

    if(left_dist > 0) {
      // left dist
      QRect rcLeftDist;
      QString strLeftDist;

      if(left_dist < 1000)
        strLeftDist.sprintf("%dm", left_dist);
      else
        strLeftDist.sprintf("%.1fkm", left_dist / 1000.f);

      QFont font("Inter");
      font.setPixelSize(55);
      font.setStyleName("Bold");

      QFontMetrics fm(font);
      int width = fm.width(strLeftDist);

      padding = 10;

      int center_x = x_start + board_width / 2;
      rcLeftDist.setRect(center_x - width / 2, y_start+board_height+15, width, font.pixelSize()+10);
      rcLeftDist.adjust(-padding*2, -padding, padding*2, padding);

      p.setPen(Qt::NoPen);
      p.setBrush(bgColor);
      p.drawRoundedRect(rcLeftDist, 20, 20);

      p.setFont(InterFont(55, QFont::Bold));
      p.setBrush(Qt::NoBrush);
      p.setPen(QColor(255, 255, 255, 230));
      p.drawText(rcLeftDist, Qt::AlignCenter|Qt::AlignVCenter, strLeftDist);
    }
  }
  else if(roadLimitSpeed > 0 && roadLimitSpeed < 200) {
    QRectF board_rect = QRectF(x_start, y_start+max_speed_height, board_width, board_height-max_speed_height);
    int padding = 14;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(QBrush(Qt::white));
    p.drawRoundedRect(board_rect, corner_radius-padding/2, corner_radius-padding/2);

    padding = 10;
    board_rect.adjust(padding, padding, -padding, -padding);
    p.setBrush(Qt::NoBrush);
    p.setPen(QPen(Qt::black, padding));
    p.drawRoundedRect(board_rect, corner_radius-12, corner_radius-12);

    {
      str = "SPEED\nLIMIT";
      p.setFont(InterFont(35, QFont::Bold));

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      str.sprintf("%d", roadLimitSpeed);
      p.setFont(InterFont(75, QFont::Bold));

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y()+board_rect.height()/2, board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 3);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }

    {
      p.setFont(InterFont(10, QFont::Bold));

      QRect text_rect = getRect(p, Qt::AlignCenter, str);
      QRect b_rect(board_rect.x(), board_rect.y(), board_rect.width(), board_rect.height()/2);
      text_rect.moveCenter({b_rect.center().x(), 0});
      text_rect.moveTop(b_rect.top() + 20);
      p.drawText(text_rect, Qt::AlignCenter, str);
    }
  }

  p.restore();//상대코드 : p.save(); 
}

void AnnotatedCameraWidget::drawSteer(QPainter &p) {
  p.save();

  int x = 30;
  int y = 540;

  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();

  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  p.setFont(InterFont(50, QFont::Bold));

  QString str;
  int width = 192;

  str.sprintf("%.0f°", steer_angle);
  QRect rect = QRect(x, y, width, width);

  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  str.sprintf("%.0f°", desire_angle);
  rect.setRect(x, y + 80, width, width);

  p.setPen(QColor(155, 255, 155, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  p.restore();
}

template <class T>
float interp(float x, std::initializer_list<T> x_list, std::initializer_list<T> y_list, bool extrapolate)
{
  std::vector<T> xData(x_list);
  std::vector<T> yData(y_list);
  int size = xData.size();

  int i = 0;
  if(x >= xData[size - 2]) {
    i = size - 2;
  }
  else {
    while ( x > xData[i+1] ) i++;
  }
  T xL = xData[i], yL = yData[i], xR = xData[i+1], yR = yData[i+1];
  if (!extrapolate) {
    if ( x < xL ) yR = yL;
    if ( x > xR ) yL = yR;
  }

  T dydx = ( yR - yL ) / ( xR - xL );
  return yL + dydx * ( x - xL );
}

void AnnotatedCameraWidget::drawDeviceState(QPainter &p) {
  p.save();

  const SubMaster &sm = *(uiState()->sm);
  auto deviceState = sm["deviceState"].getDeviceState();

  const auto freeSpacePercent = deviceState.getFreeSpacePercent();

  const auto cpuTempC = deviceState.getCpuTempC();
  //const auto gpuTempC = deviceState.getGpuTempC();
  float ambientTemp = deviceState.getAmbientTempC();

  float cpuTemp = 0.f;
  //float gpuTemp = 0.f;

  if(std::size(cpuTempC) > 0) {
    for(int i = 0; i < std::size(cpuTempC); i++) {
      cpuTemp += cpuTempC[i];
    }
    cpuTemp = cpuTemp / (float)std::size(cpuTempC);
  }

  /*if(std::size(gpuTempC) > 0) {
    for(int i = 0; i < std::size(gpuTempC); i++) {
      gpuTemp += gpuTempC[i];
    }
    gpuTemp = gpuTemp / (float)std::size(gpuTempC);
    cpuTemp = (cpuTemp + gpuTemp) / 2.f;
  }*/

  int w = 192;
  int x = width() - (30 + w) + 8;
  int y = 340;

  QString str;
  QRect rect;

  p.setFont(InterFont(50, QFont::Bold));
  str.sprintf("%.0f%%", freeSpacePercent);
  rect = QRect(x, y, w, w);

  int r = interp<float>(freeSpacePercent, {10.f, 90.f}, {255.f, 200.f}, false);
  int g = interp<float>(freeSpacePercent, {10.f, 90.f}, {200.f, 255.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  p.setFont(InterFont(25, QFont::Bold));
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "STORAGE");

  y += 80;
  p.setFont(InterFont(50, QFont::Bold));
  str.sprintf("%.0f°C", cpuTemp);
  rect = QRect(x, y, w, w);
  r = interp<float>(cpuTemp, {50.f, 90.f}, {200.f, 255.f}, false);
  g = interp<float>(cpuTemp, {50.f, 90.f}, {255.f, 200.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  p.setFont(InterFont(25, QFont::Bold));
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "CPU");

  y += 80;
  p.setFont(InterFont(50, QFont::Bold));
  str.sprintf("%.0f°C", ambientTemp);
  rect = QRect(x, y, w, w);
  r = interp<float>(ambientTemp, {35.f, 60.f}, {200.f, 255.f}, false);
  g = interp<float>(ambientTemp, {35.f, 60.f}, {255.f, 200.f}, false);
  p.setPen(QColor(r, g, 200, 200));
  p.drawText(rect, Qt::AlignCenter, str);

  y += 55;
  p.setFont(InterFont(25, QFont::Bold));
  rect = QRect(x, y, w, w);
  p.setPen(QColor(255, 255, 255, 200));
  p.drawText(rect, Qt::AlignCenter, "AMBIENT");

  p.restore();
}

void AnnotatedCameraWidget::drawTurnSignals(QPainter &p) {
  p.save();

  static int blink_index = 0;
  static int blink_wait = 0;
  static double prev_ts = 0.0;

  if(blink_wait > 0) {
    blink_wait--;
    blink_index = 0;
  }
  else {
    const SubMaster &sm = *(uiState()->sm);
    auto car_state = sm["carState"].getCarState();
    bool left_on = car_state.getLeftBlinker();
    bool right_on = car_state.getRightBlinker();

    const float img_alpha = 0.8f;
    const int fb_w = width() / 2 - 200;
    const int center_x = width() / 2;
    const int w = fb_w / 25;
    const int h = 160;
    const int gap = fb_w / 25;
    const int margin = (int)(fb_w / 3.8f);
    const int base_y = (height() - h) / 2;
    const int draw_count = 8;

    int x = center_x;
    int y = base_y;

    if(left_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        p.setOpacity(alpha);
        float factor = (float)draw_count / (i + draw_count);
        p.drawPixmap(x - w - margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_l);
        x -= gap + w;
      }
    }

    x = center_x;
    if(right_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        float factor = (float)draw_count / (i + draw_count);
        p.setOpacity(alpha);
        p.drawPixmap(x + margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_r);
        x += gap + w;
      }
    }

    if(left_on || right_on) {

      double now = millis_since_boot();
      if(now - prev_ts > 900/UI_FREQ) {
        prev_ts = now;
        blink_index++;
      }

      if(blink_index >= draw_count) {
        blink_index = draw_count - 1;
        blink_wait = UI_FREQ/4;
      }
    }
    else {
      blink_index = 0;
    }
  }

  p.restore();
}

void AnnotatedCameraWidget::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if(accuracy < 0.01f || accuracy > 20.f)
    return;

  int w = 90;
  int h = 75;
  int x = width() - w - 75 + 8;
  int y = 240;

  p.save();

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_satellite);

  p.setFont(InterFont(40, QFont::Bold));
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("%.1fm", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);

  p.restore();
}

void AnnotatedCameraWidget::drawDebugText(QPainter &p) {

  p.save();

  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  int y = 80;

  const int text_x = width()/2 + 220;
  auto car_control = sm["carControl"].getCarControl();

  p.setFont(InterFont(40, QFont::Normal));
  p.setPen(QColor(255, 255, 255, 200));

  QRect rect = QRect(text_x, y, width()/2 - 120, height() - y);

  p.drawText(rect, Qt::AlignLeft, QString::fromStdString(car_control.getDebugText().cStr()));

  p.restore();
}

void AnnotatedCameraWidget::drawDriverState(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;

  painter.save();

  // base icon
  int x = radius / 2 + (UI_BORDER_SIZE * 2); //+ (radius + 50);
  int y = rect().bottom() - UI_FOOTER_HEIGHT / 2 - 10;

  float opacity = dmActive ? 0.65f : 0.15f;
  drawIcon(painter, QPoint(x, y), dm_img, blackColor(70), opacity);

  // face
  QPointF face_kpts_draw[std::size(default_face_kpts_3d)];
  float kp;
  for (int i = 0; i < std::size(default_face_kpts_3d); ++i) {
    kp = (scene.face_kpts_draw[i].v[2] - 8) / 120 + 1.0;
    face_kpts_draw[i] = QPointF(scene.face_kpts_draw[i].v[0] * kp + x, scene.face_kpts_draw[i].v[1] * kp + y);
  }

  painter.setPen(QPen(QColor::fromRgbF(1.0, 1.0, 1.0, opacity), 5.2, Qt::SolidLine, Qt::RoundCap));
  painter.drawPolyline(face_kpts_draw, std::size(default_face_kpts_3d));

  // tracking arcs
  const int arc_l = 133;
  const float arc_t_default = 6.7;
  const float arc_t_extend = 12.0;
  QColor arc_color = QColor::fromRgbF(0.545 - 0.445 * s->engaged(),
                                      0.545 + 0.4 * s->engaged(),
                                      0.545 - 0.285 * s->engaged(),
                                      0.4 * (1.0 - dm_fade_state));
  float delta_x = -scene.driver_pose_sins[1] * arc_l / 2;
  float delta_y = -scene.driver_pose_sins[0] * arc_l / 2;
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[1] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(std::fmin(x + delta_x, x), y - arc_l / 2, fabs(delta_x), arc_l), (scene.driver_pose_sins[1]>0 ? 90 : -90) * 16, 180 * 16);
  painter.setPen(QPen(arc_color, arc_t_default+arc_t_extend*fmin(1.0, scene.driver_pose_diff[0] * 5.0), Qt::SolidLine, Qt::RoundCap));
  painter.drawArc(QRectF(x - arc_l / 2, std::fmin(y + delta_y, y), arc_l, fabs(delta_y)), (scene.driver_pose_sins[0]>0 ? 0 : 180) * 16, 180 * 16);

  painter.restore();
}

static const QColor get_tpms_color(float tpms) {
    if(tpms < 5 || tpms > 60) // N/A
        return QColor(255, 255, 255, 220);
    if(tpms < 31)
        return QColor(255, 90, 90, 220);
    return QColor(255, 255, 255, 220);
}

static const QString get_tpms_text(float tpms) {
    if(tpms < 5 || tpms > 60)
        return "";

    char str[32];
    snprintf(str, sizeof(str), "%.0f", round(tpms));
    return QString(str);
}

// 이 함수는 차량의 하단 HUD에 여러 아이콘들을 그리는 함수입니다.
void AnnotatedCameraWidget::drawBottomIcons(QPainter &p) {
  p.save(); // 현재 QPainter 상태를 저장합니다.
  // SubMaster 객체에서 차량 상태 정보를 가져옵니다.
  const SubMaster &sm = *(uiState()->sm);
  const auto car_state = sm["carState"].getCarState();
  const auto car_control = sm["carControl"].getCarControl();
  const auto car_params = (*uiState()->sm)["carParams"].getCarParams();

  int yOffset = 100; // The amount by which you want to raise the icons

  // 타이어 압력 모니터링 시스템(TPMS) 정보를 가져옵니다.
  const auto tpms = car_state.getTpms();

  int n = 1; // 아이콘을 배치하기 위한 인덱스 변수를 초기화합니다.

  // 타이어 압력 정보가 활성화되어 있을 경우
  if(tpms.getEnabled())
  {
    // 아이콘의 너비와 높이를 정의합니다.
    const int w = 58;
    const int h = 126;
    // 아이콘의 X, Y 위치를 계산합니다.
    const int x = radius / 2 + (UI_BORDER_SIZE * 2) + (radius + 50) * n - w/2;
    const int y = height() - h - 85 - yOffset;

    // 각 타이어의 압력 값을 가져옵니다.
    const float fl = tpms.getFl(); // 앞 왼쪽 타이어
    const float fr = tpms.getFr(); // 앞 오른쪽 타이어
    const float rl = tpms.getRl(); // 뒤 왼쪽 타이어
    const float rr = tpms.getRr(); // 뒤 오른쪽 타이어

    // 타이어 압력 아이콘의 불투명도를 설정합니다.
    p.setOpacity(0.8);
    // 타이어 압력 아이콘을 그립니다.
    p.drawPixmap(x, y, w, h, ic_tire_pressure);

    // 타이어 압력 값에 따라 색상과 텍스트를 설정하고 그립니다.
    p.setFont(InterFont(38, QFont::Bold)); // 폰트 크기와 스타일 설정
    QFontMetrics fm(p.font()); // 폰트 메트릭스 정보를 가져옵니다.
    QRect rcFont = fm.boundingRect("9"); // 숫자 9의 폰트 경계를 계산합니다.

    int center_x = x + 3; // 중앙 X 위치
    int center_y = y + h/2; // 중앙 Y 위치
    // 텍스트의 X, Y 마진을 계산합니다.
    const int marginX = (int)(rcFont.width() * 2.7f);
    const int marginY = (int)((h/2 - rcFont.height()) * 0.7f);

    // 타이어 압력 값을 그립니다.
    drawText2(p, center_x-marginX, center_y-marginY-rcFont.height(), Qt::AlignRight, get_tpms_text(fl), get_tpms_color(fl));
    drawText2(p, center_x+marginX, center_y-marginY-rcFont.height(), Qt::AlignLeft, get_tpms_text(fr), get_tpms_color(fr));
    drawText2(p, center_x-marginX, center_y+marginY, Qt::AlignRight, get_tpms_text(rl), get_tpms_color(rl));
    drawText2(p, center_x+marginX, center_y+marginY, Qt::AlignLeft, get_tpms_text(rr), get_tpms_color(rr));

    n++; // 다음 아이콘 위치를 위해 인덱스를 증가시킵니다.
  }

  // 크루즈 컨트롤 간격 조절 아이콘을 그립니다.
  int x = radius / 2 + (UI_BORDER_SIZE * 2) + (radius + 50) * n;
  const int y = rect().bottom() - UI_FOOTER_HEIGHT / 2 - 10 - yOffset;
  // 크루즈 컨트롤 간격 값을 가져옵니다.
  int gap = car_state.getCruiseState().getGapAdjust();
  int autoTrGap = car_control.getAutoTrGap(); // 자동 조절 간격 값

  // 간격 조절 아이콘의 배경을 그립니다.
  p.setPen(Qt::NoPen); // 펜 비활성화
  p.setBrush(QBrush(QColor(0, 0, 0, 255 * .1f))); // 브러시 설정
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius); // 원을 그립니다.

  // 간격 값에 따른 텍스트를 그립니다.
  QString str; // 텍스트를 담을 문자열 변수
  float textSize = 50.f; // 텍스트 크기
  QColor textColor = QColor(255, 255, 255, 200); // 텍스트 색상

  // 간격 값에 따라 다른 텍스트와 색상을 설정합니다.
  if(gap <= 0) {
    str = "N/A";  // 정보 없음
  }
  else if(gap == autoTrGap) {
    str = "AUTO"; // 자동 간격 조정
    textColor = QColor(120, 255, 120, 200); // 색상 변경
  }
  else {
    str.sprintf("%d", (int)gap); // 간격 값
    textColor = QColor(120, 255, 120, 200); // 색상 변경
    textSize = 70.f; // 크기 변경
  }

  p.setFont(InterFont(35, QFont::Bold)); // 폰트 설정
  drawText(p, x, y-20, "GAP", 200); // "GAP" 텍스트를 그립니다.

  p.setFont(InterFont(textSize, QFont::Bold)); // 텍스트 크기 재설정
  drawTextWithColor(p, x, y+50, str, textColor); // 설정된 텍스트와 색상으로 간격 값을 그립니다.
  n++; // 아이콘 인덱스 증가

  // 브레이크 아이콘
  x = radius / 2 + (UI_BORDER_SIZE * 2) + (radius + 50) * n;
  bool brake_valid = car_state.getBrakeLights();
  float img_alpha = brake_valid ? 1.0f : 0.15f; // 브레이크 상태에 따른 아이콘의 투명도 설정
  float bg_alpha = brake_valid ? 0.3f : 0.1f;   // 브레이크 상태에 따른 배경 투명도 설정
  drawIcon(p, QPoint(x, y), ic_brake, QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
  n++;

  // auto hold icon.
  if(car_params.getHasAutoHold()) {
    int autohold = car_state.getAutoHold();
    if(autohold >= 0) {
      x = radius / 2 + (UI_BORDER_SIZE * 2) + (radius + 50) * n;
      img_alpha = autohold > 0 ? 1.0f : 0.15f;  // 오토 홀드 활성화 상태에 따른 아이콘의 투명도
      bg_alpha = autohold > 0 ? 0.3f : 0.1f;    // 오토 홀드 활성화 상태에 따른 배경 투명도
      drawIcon(p, QPoint(x, y), autohold > 1 ? ic_autohold_warning : ic_autohold_active,
              QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
    }
    n++;
  }

  p.restore(); // 페인터 상태 복원
}

void AnnotatedCameraWidget::drawMisc(QPainter &p) {
  if(width() < 1080) return;

  p.save();
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);

  const auto navi_data = sm["naviData"].getNaviData();
  QString currentRoadName = QString::fromStdString(navi_data.getCurrentRoadName().cStr());

  QColor color = QColor(255, 255, 255, 230);

  p.setFont(InterFont(70, QFont::Normal));
  drawText(p, (width()-(UI_BORDER_SIZE*2))/4 + UI_BORDER_SIZE + 20, 140, currentRoadName, 200);

  p.restore();
}
