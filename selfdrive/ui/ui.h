#pragma once

#include <map>
#include <memory>
#include <string>

#include <QObject>
#include <QTimer>
#include <QColor>
#include <QFuture>
#include <QPolygonF>
#include <QTransform>

#include "cereal/messaging/messaging.h"
#include "common/mat.h"
#include "common/params.h"
#include "common/timing.h"
#include "system/hardware/hw.h"

const int UI_BORDER_SIZE = 20;
const int UI_HEADER_HEIGHT = 420;
const int UI_FOOTER_HEIGHT = 280;

const int UI_FREQ = 20; // Hz
const int BACKLIGHT_OFFROAD = 50;
typedef cereal::CarControl::HUDControl::AudibleAlert AudibleAlert;

const float MIN_DRAW_DISTANCE = 10.0;
const float MAX_DRAW_DISTANCE = 100.0;
constexpr mat3 DEFAULT_CALIBRATION = {{ 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0 }};
constexpr mat3 FCAM_INTRINSIC_MATRIX = (mat3){{2648.0, 0.0, 1928.0 / 2,
                                           0.0, 2648.0, 1208.0 / 2,
                                           0.0, 0.0, 1.0}};
// tici ecam focal probably wrong? magnification is not consistent across frame
// Need to retrain model before this can be changed
constexpr mat3 ECAM_INTRINSIC_MATRIX = (mat3){{567.0, 0.0, 1928.0 / 2,
                                           0.0, 567.0, 1208.0 / 2,
                                           0.0, 0.0, 1.0}};


constexpr vec3 default_face_kpts_3d[] = {
  {-5.98, -51.20, 8.00}, {-17.64, -49.14, 8.00}, {-23.81, -46.40, 8.00}, {-29.98, -40.91, 8.00}, {-32.04, -37.49, 8.00},
  {-34.10, -32.00, 8.00}, {-36.16, -21.03, 8.00}, {-36.16, 6.40, 8.00}, {-35.47, 10.51, 8.00}, {-32.73, 19.43, 8.00},
  {-29.30, 26.29, 8.00}, {-24.50, 33.83, 8.00}, {-19.01, 41.37, 8.00}, {-14.21, 46.17, 8.00}, {-12.16, 47.54, 8.00},
  {-4.61, 49.60, 8.00}, {4.99, 49.60, 8.00}, {12.53, 47.54, 8.00}, {14.59, 46.17, 8.00}, {19.39, 41.37, 8.00},
  {24.87, 33.83, 8.00}, {29.67, 26.29, 8.00}, {33.10, 19.43, 8.00}, {35.84, 10.51, 8.00}, {36.53, 6.40, 8.00},
  {36.53, -21.03, 8.00}, {34.47, -32.00, 8.00}, {32.42, -37.49, 8.00}, {30.36, -40.91, 8.00}, {24.19, -46.40, 8.00},
  {18.02, -49.14, 8.00}, {6.36, -51.20, 8.00}, {-5.98, -51.20, 8.00},
};

// MJ : struct Alert 
// text messages, alert type, size, status, and audible alert.
struct Alert {
  QString text1;
  QString text2;
  QString type;
  cereal::ControlsState::AlertSize size;
  cereal::ControlsState::AlertStatus status;
  AudibleAlert sound;

  bool equal(const Alert &a2) {
    return text1 == a2.text1 && text2 == a2.text2 && type == a2.type && sound == a2.sound;
  }

  // MJ
  // 이 코드는 Comma.ai의 openpilot 시스템에서 경고 메시지를 생성하는 함수입니다. 
  // 이 함수는 현재 상태와 관련된 경고를 만들고, 
  // 시스템의 상태에 따라 적절한 경고 메시지와 경고 유형을 결정합니다.
  // input 1 : a shared memory object containing control states received from another process.
  // input 2 : the starting frame number when monitoring the alert.
  static Alert get(const SubMaster &sm, uint64_t started_frame) {
    const cereal::ControlsState::Reader &cs = sm["controlsState"].getControlsState(); // MJ : "controlsState" 메시지를 받아옵니다.
    const uint64_t controls_frame = sm.rcv_frame("controlsState");  // MJ : "controlsState"의 가장 최근 수신 프레임 번호를 가져옵니다.

    Alert alert = {}; // MJ : 경고 구조체를 초기화합니다.
    // MJ
    // controls_frame keeps track of the latest Control State frame number, 
    // whereas started_frame indicates the reference point for updating the alert information.
    if (controls_frame >= started_frame) {  // Don't get old alert.   // MJ : 시작 프레임 이후의 경고만 처리하도록 합니다. 오래된 경고는 처리하지 않습니다.
      alert = {cs.getAlertText1().cStr(), cs.getAlertText2().cStr(),  // MJ : 경고 텍스트와 경고 유형을 설정합니다.
               cs.getAlertType().cStr(), cs.getAlertSize(), 
               cs.getAlertStatus(),
               cs.getAlertSound()};
    }

    // MJ : "controlsState"가 업데이트되지 않았고, 시작 프레임부터 현재까지 5초 이상 경과했으면 경고를 처리합니다.
    //    SubMaster는 특정 키에 대해 새 데이터를 받으면 업데이트됨을 표시합니다.
    //    UI_FREQ는 일반적으로 사용자 인터페이스(UI)의 갱신 주파수를 나타내는 변수
    //    예를 들어, UI가 60fps로 갱신된다면 UI_FREQ는 60Hz로 설정될 것입니다. 
    if (!sm.updated("controlsState") && (sm.frame - started_frame) > 5 * UI_FREQ) {
      const int CONTROLS_TIMEOUT = 5; // MJ : 컨트롤 타임아웃을 5초로 설정합니다.
      const int controls_missing = (nanos_since_boot() - sm.rcv_time("controlsState")) / 1e9;// MJ : 마지막 "controlsState" 수신 시간으로부터 경과한 시간을 계산합니다.

      // MJ : 컨트롤 타임아웃을 처리합니다.
      // Handle controls timeout
      if (controls_frame < started_frame) {
        // MJ : 차량은 시작되었지만, "controlsState"가 전혀 수신되지 않았습니다.
        // car is started, but controlsState hasn't been seen at all        
        alert = {"MJ0 openpilot Unavailable", "MJ0 Waiting for controls to start", // MJ : 대기 중인 경고 메시지를 설정합니다.
                 "MJ0 controlsWaiting", cereal::ControlsState::AlertSize::MID,
                 cereal::ControlsState::AlertStatus::NORMAL,
                 AudibleAlert::NONE};
      } else if (controls_missing > CONTROLS_TIMEOUT && !Hardware::PC()) {
        // MJ : 차량은 시작되었지만, 컨트롤이 지연되거나 작동이 멈췄습니다.
        // car is started, but controls is lagging or died
        if (cs.getEnabled() && (controls_missing - CONTROLS_TIMEOUT) < 10) {
          alert = {"MJ0 TAKE CONTROL IMMEDIATELY", "MJ0 Controls Unresponsive", // MJ : 즉각적인 조치가 필요한 경고 메시지를 설정합니다.
                   "MJ0 controlsUnresponsive", cereal::ControlsState::AlertSize::FULL,
                   cereal::ControlsState::AlertStatus::CRITICAL,
                   AudibleAlert::WARNING_IMMEDIATE};
        } else {
          alert = {"MJ0 Controls Unresponsive", "MJ0 Reboot Device",        // MJ : 경고 메시지를 설정하고, 재부팅을 권장합니다.
                   "MJ0 controlsUnresponsivePermanent", cereal::ControlsState::AlertSize::MID,
                   cereal::ControlsState::AlertStatus::NORMAL,
                   AudibleAlert::NONE};
        }
      }
    }
    return alert;// MJ : 생성된 경고를 반환합니다.
  }
};

typedef enum UIStatus {
  STATUS_DISENGAGED,
  STATUS_OVERRIDE,
  STATUS_ENGAGED,
} UIStatus;

enum PrimeType {
  UNKNOWN = -1,
  NONE = 0,
  MAGENTA = 1,
  LITE = 2,
  BLUE = 3,
  MAGENTA_NEW = 4,
  PURPLE = 5,
};

const QColor bg_colors [] = {
  [STATUS_DISENGAGED] = QColor(0x17, 0x33, 0x49, 0xc8),
  [STATUS_OVERRIDE] = QColor(0x91, 0x9b, 0x95, 0xf1),
  [STATUS_ENGAGED] = QColor(0x17, 0x86, 0x44, 0xf1),
};

static std::map<cereal::ControlsState::AlertStatus, QColor> alert_colors = {
  {cereal::ControlsState::AlertStatus::NORMAL, QColor(0x15, 0x15, 0x15, 0xf1)},
  {cereal::ControlsState::AlertStatus::USER_PROMPT, QColor(0xDA, 0x6F, 0x25, 0xf1)},
  {cereal::ControlsState::AlertStatus::CRITICAL, QColor(0xC9, 0x22, 0x31, 0xf1)},
};

typedef struct UIScene {
  bool calibration_valid = false;
  bool calibration_wide_valid  = false;
  bool wide_cam = true;
  mat3 view_from_calib = DEFAULT_CALIBRATION;
  mat3 view_from_wide_calib = DEFAULT_CALIBRATION;
  cereal::PandaState::PandaType pandaType;

  // modelV2
  float lane_line_probs[4];
  float road_edge_stds[2];
  QPolygonF track_vertices;
  QPolygonF lane_line_vertices[4];
  QPolygonF road_edge_vertices[2];

  // lead
  QPointF lead_vertices[2];
  bool lead_radar[2] = {false, false};

  // DMoji state
  float driver_pose_vals[3];
  float driver_pose_diff[3];
  float driver_pose_sins[3];
  float driver_pose_coss[3];
  vec3 face_kpts_draw[std::size(default_face_kpts_3d)];

  bool navigate_on_openpilot = false;

  float light_sensor;
  bool started, ignition, is_metric, map_on_left, longitudinal_control;
  bool world_objects_visible = false;
  uint64_t started_frame;
} UIScene;

class UIState : public QObject {
  Q_OBJECT

public:
  UIState(QObject* parent = 0);
  void updateStatus();
  inline bool engaged() const {
    return scene.started && (*sm)["controlsState"].getControlsState().getEnabled();
  }

  void setPrimeType(PrimeType type);
  inline PrimeType primeType() const { return prime_type; }
  inline bool hasPrime() const { return prime_type != PrimeType::UNKNOWN && prime_type != PrimeType::NONE; }

  int fb_w = 0, fb_h = 0;

  std::unique_ptr<SubMaster> sm;

  UIStatus status;
  UIScene scene = {};

  QString language;

  QTransform car_space_transform;

  bool recording = false;

signals:
  void uiUpdate(const UIState &s);
  void offroadTransition(bool offroad);
  void primeChanged(bool prime);
  void primeTypeChanged(PrimeType prime_type);

private slots:
  void update();

private:
  QTimer *timer;
  bool started_prev = false;
  PrimeType prime_type = PrimeType::UNKNOWN;
};

UIState *uiState();

// device management class
class Device : public QObject {
  Q_OBJECT

public:
  Device(QObject *parent = 0);
  bool isAwake() { return awake; }
  void setOffroadBrightness(int brightness) {
    offroad_brightness = std::clamp(brightness, 0, 100);
  }

private:
  bool awake = false;
  int interactive_timeout = 0;
  bool ignition_on = false;

  int offroad_brightness = BACKLIGHT_OFFROAD;
  int last_brightness = 0;
  FirstOrderFilter brightness_filter;
  QFuture<void> brightness_future;

  void updateBrightness(const UIState &s);
  void updateWakefulness(const UIState &s);
  void setAwake(bool on);

signals:
  void displayPowerChanged(bool on);
  void interactiveTimeout();

public slots:
  void resetInteractiveTimeout(int timeout = -1);
  void update(const UIState &s);
};

Device *device();

void ui_update_params(UIState *s);
int get_path_length_idx(const cereal::XYZTData::Reader &line, const float path_height);
void update_model(UIState *s,
                  const cereal::ModelDataV2::Reader &model,
                  const cereal::UiPlan::Reader &plan);
void update_dmonitoring(UIState *s, const cereal::DriverStateV2::Reader &driverstate, float dm_fade_state, bool is_rhd);
void update_leads(UIState *s, const cereal::RadarState::Reader &radar_state, const cereal::XYZTData::Reader &line);
void update_line_data(const UIState *s, const cereal::XYZTData::Reader &line,
                      float y_off, float z_off, QPolygonF *pvd, int max_idx, bool allow_invert);
