#!/usr/bin/env python3
import math
import os
from enum import IntEnum
from typing import Dict, Union, Callable, List, Optional

from cereal import log, car
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.locationd.calibrationd import MIN_SPEED_FILTER
from openpilot.system.version import get_short_branch

AlertSize = log.ControlsState.AlertSize
AlertStatus = log.ControlsState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
EventName = car.CarEvent.EventName


# Alert priorities
class Priority(IntEnum):
  LOWEST = 0
  LOWER = 1
  LOW = 2
  MID = 3
  HIGH = 4
  HIGHEST = 5


# Event types
class ET:
  ENABLE = 'enable'
  PRE_ENABLE = 'preEnable'
  OVERRIDE_LATERAL = 'overrideLateral'
  OVERRIDE_LONGITUDINAL = 'overrideLongitudinal'
  NO_ENTRY = 'noEntry'
  WARNING = 'warning'
  USER_DISABLE = 'userDisable'
  SOFT_DISABLE = 'softDisable'
  IMMEDIATE_DISABLE = 'immediateDisable'
  PERMANENT = 'permanent'


# get event name from enum
EVENT_NAME = {v: k for k, v in EventName.schema.enumerants.items()}


class Events:
  def __init__(self):
    self.events: List[int] = []
    self.static_events: List[int] = []
    self.events_prev = dict.fromkeys(EVENTS.keys(), 0)

  @property
  def names(self) -> List[int]:
    return self.events

  def __len__(self) -> int:
    return len(self.events)

  def add(self, event_name: int, static: bool=False) -> None:
    if static:
      self.static_events.append(event_name)
    self.events.append(event_name)

  def clear(self) -> None:
    self.events_prev = {k: (v + 1 if k in self.events else 0) for k, v in self.events_prev.items()}
    self.events = self.static_events.copy()

  def contains(self, event_type: str) -> bool:
    return any(event_type in EVENTS.get(e, {}) for e in self.events)

  def create_alerts(self, event_types: List[str], callback_args=None):
    if callback_args is None:
      callback_args = []

    ret = []
    for e in self.events:
      types = EVENTS[e].keys()
      for et in event_types:
        if et in types:
          alert = EVENTS[e][et]
          if not isinstance(alert, Alert):
            alert = alert(*callback_args)

          if DT_CTRL * (self.events_prev[e] + 1) >= alert.creation_delay:
            alert.alert_type = f"{EVENT_NAME[e]}/{et}"
            alert.event_type = et
            ret.append(alert)
    return ret

  def add_from_msg(self, events):
    for e in events:
      self.events.append(e.name.raw)

  def to_msg(self):
    ret = []
    for event_name in self.events:
      event = car.CarEvent.new_message()
      event.name = event_name
      for event_type in EVENTS.get(event_name, {}):
        setattr(event, event_type, True)
      ret.append(event)
    return ret


class Alert:
  def __init__(self,
               alert_text_1: str,
               alert_text_2: str,
               alert_status: log.ControlsState.AlertStatus,
               alert_size: log.ControlsState.AlertSize,
               priority: Priority,
               visual_alert: car.CarControl.HUDControl.VisualAlert,
               audible_alert: car.CarControl.HUDControl.AudibleAlert,
               duration: float,
               alert_rate: float = 0.,
               creation_delay: float = 0.):

    self.alert_text_1 = alert_text_1
    self.alert_text_2 = alert_text_2
    self.alert_status = alert_status
    self.alert_size = alert_size
    self.priority = priority
    self.visual_alert = visual_alert
    self.audible_alert = audible_alert

    self.duration = int(duration / DT_CTRL)

    self.alert_rate = alert_rate
    self.creation_delay = creation_delay

    self.alert_type = ""
    self.event_type: Optional[str] = None

  def __str__(self) -> str:
    return f"{self.alert_text_1}/{self.alert_text_2} {self.priority} {self.visual_alert} {self.audible_alert}"

  def __gt__(self, alert2) -> bool:
    if not isinstance(alert2, Alert):
      return False
    return self.priority > alert2.priority

# MJ Comments added :
# 한글화
class NoEntryAlert(Alert):
  def __init__(self, alert_text_2: str,
               alert_text_1: str = "openpilot Unavailable /\n 오픈파일럿 사용불가",
               visual_alert: car.CarControl.HUDControl.VisualAlert=VisualAlert.none):
    super().__init__(alert_text_1, alert_text_2, AlertStatus.normal,
                     AlertSize.mid, Priority.LOW, visual_alert,
                     AudibleAlert.refuse, 3.)

class SoftDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__("TAKE CONTROL IMMEDIATELY /\n 핸들을 즉시 잡아주세요", alert_text_2,
                     AlertStatus.userPrompt, AlertSize.full,
                     Priority.MID, VisualAlert.steerRequired,
                     AudibleAlert.warningSoft, 2.),


# less harsh version of SoftDisable, where the condition is user-triggered
class UserSoftDisableAlert(SoftDisableAlert):
  def __init__(self, alert_text_2: str):
    super().__init__(alert_text_2),
    self.alert_text_1 = "openpilot will disengage /\n 오픈파일럿이 해제됩니다"


class ImmediateDisableAlert(Alert):
  def __init__(self, alert_text_2: str):
    super().__init__("TAKE CONTROL IMMEDIATELY /\n 핸들을 즉시 잡아주세요", alert_text_2,
                     AlertStatus.critical, AlertSize.full,
                     Priority.HIGHEST, VisualAlert.steerRequired,
                     AudibleAlert.warningImmediate, 4.),


class EngagementAlert(Alert):
  def __init__(self, audible_alert: car.CarControl.HUDControl.AudibleAlert):
    super().__init__("", "",
                     AlertStatus.normal, AlertSize.none,
                     Priority.MID, VisualAlert.none,
                     audible_alert, .2),


class NormalPermanentAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "",
               duration: float = 0.2, priority: Priority = Priority.LOWER, creation_delay: float = 0.):
    super().__init__(alert_text_1, alert_text_2,
                     AlertStatus.normal, AlertSize.mid if len(alert_text_2) else AlertSize.small,
                     priority, VisualAlert.none, AudibleAlert.none, duration, creation_delay=creation_delay),


class StartupAlert(Alert):
  def __init__(self, alert_text_1: str, alert_text_2: str = "Always keep hands on wheel and eyes on road /\n 항상 핸들을 잡고 전방주시를 하세요",
               alert_status=AlertStatus.normal):
    super().__init__(alert_text_1, alert_text_2,
                     alert_status, AlertSize.mid,
                     Priority.LOWER, VisualAlert.none, AudibleAlert.none, 5.),


# ********** helper functions **********
def get_display_speed(speed_ms: float, metric: bool) -> str:
  speed = int(round(speed_ms * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH)))
  unit = 'km/h' if metric else 'mph'
  return f"{speed} {unit}"


# MJ
"""
AlertCallbackType는 파이썬 코드에서 
특정 인자를 받아 Alert 객체를 반환하는 콜백 함수의, 예상 형식을 지정하기 위한 타입 힌트입니다.
구체적으로, AlertCallbackType은 다음과 같이 정의되어 있습니다:
  Callable[[car.CarParams, car.CarState, messaging.SubMaster, bool, int], Alert]

여기서 Callable은 특정 시그니처를 가진 호출 가능한 객체를 나타내는 내장 클래스입니다. 
(즉, 함수나 람다 표현식과 같은 함수와 유사한 객체)

AlertCallbackType은 다음 인자를 받고 Alert 객체를 반환합니다:
  car.CarParams의 인스턴스
  car.CarState의 인스턴스
  messaging.SubMaster의 인스턴스
  불리언 값
  정수 값

따라서, AlertCallbackType을 별도의 변수로 정의하면, 코드의 가독성과 유지보수성을 높일 수 있습니다. 
개발자는 읽는 동안, 콜백 함수의 Expected Signature를 쉽게 이해할 수 있으며, 
모든 IDE와 텍스트 편집기는, 타입 힌트를 사용하여 더 나은 자동 완성 제안과 오류 검사를 제공할 수 있습니다.
"""
# ********** alert callback functions **********
AlertCallbackType = Callable[[car.CarParams, car.CarState, messaging.SubMaster, bool, int], Alert]


def soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return SoftDisableAlert(alert_text_2)
  return func
# MJ
def user_soft_disable_alert(alert_text_2: str) -> AlertCallbackType:
  def func(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
    if soft_disable_time < int(0.5 / DT_CTRL):
      return ImmediateDisableAlert(alert_text_2)
    return UserSoftDisableAlert(alert_text_2)
  return func

def startup_master_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  branch = get_short_branch("")  # Ensure get_short_branch is cached to avoid lags on startup
  if "REPLAY" in os.environ:
    branch = "replay"
  return StartupAlert("WARNING: This branch is not tested /\n 이 브랜치는 테스트되지 않았습니다", branch, alert_status=AlertStatus.userPrompt)

def below_engage_speed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NoEntryAlert(f"Drive above {get_display_speed(CP.minEnableSpeed, metric)} to engage /\n {get_display_speed(CP.minEnableSpeed, metric)} 이상으로 운전하여 오픈파일럿을 활성화하세요")

def below_steer_speed_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    f"Steer Unavailable Below {get_display_speed(CP.minSteerSpeed, metric)} / {get_display_speed(CP.minSteerSpeed, metric)} 이하에서는 조향이 불가능합니다",
    "",
    AlertStatus.userPrompt, AlertSize.small,
    Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, 0.4)


def calibration_incomplete_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  first_word = 'Recalibration / 재보정' if sm['liveCalibration'].calStatus == log.LiveCalibrationData.Status.recalibrating else 'Calibration / 보정'
  return Alert(
    f"{first_word} in Progress: {sm['liveCalibration'].calPerc:.0f}% 진행중",
    f"Drive Above {get_display_speed(MIN_SPEED_FILTER, metric)} /\n {get_display_speed(MIN_SPEED_FILTER, metric)} 이상 운전하세요",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2)


def no_gps_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return Alert(
    "Poor GPS reception / GPS 수신불량",
    "Hardware malfunctioning if sky is visible /\n 하늘이 보이는데도 GPS가 작동하지 않을 경우 하드웨어의 오작동일 수 있습니다",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=300.)

# *** debug alerts ***

def out_of_space_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  full_perc = round(100. - sm['deviceState'].freeSpacePercent)
  return NormalPermanentAlert("Out of Storage /\n 저장 공간 부족", f"{full_perc}% full / {full_perc}% 가득 찼습니다")

def posenet_invalid_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  mdl = sm['modelV2'].velocity.x[0] if len(sm['modelV2'].velocity.x) else math.nan
  err = CS.vEgo - mdl
  msg = f"Speed Error: {err:.1f} m/s /\n 속도 오류: {err:.1f} m/s"
  return NoEntryAlert(msg, alert_text_1="Posenet Speed Invalid /\n Posenet 속도 무효")

def process_not_running_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  not_running = [p.name for p in sm['managerState'].processes if not p.running and p.shouldBeRunning]
  msg = ', '.join(not_running)
  return NoEntryAlert(msg, alert_text_1="Process Not Running /\n 프로세스 실행 중지")

def comm_issue_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  bs = [s for s in sm.data.keys() if not sm.all_checks([s, ])]
  msg = ', '.join(bs[:4])  # can't fit too many on one line
  return NoEntryAlert(msg, alert_text_1="Communication Issue Between Processes /\n 프로세스 간 통신 문제")

def camera_malfunction_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  all_cams = ('roadCameraState', 'driverCameraState', 'wideRoadCameraState')
  bad_cams = [s.replace('State', '') for s in all_cams if s in sm.data.keys() and not sm.all_checks([s, ])]
  return NormalPermanentAlert("Camera Malfunction /\n 카메라 오작동", ', '.join(bad_cams))

def calibration_invalid_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  rpy = sm['liveCalibration'].rpyCalib
  yaw = math.degrees(rpy[2] if len(rpy) == 3 else math.nan)
  pitch = math.degrees(rpy[1] if len(rpy) == 3 else math.nan)
  angles = f"Remount Device (Pitch: {pitch:.1f}°, Yaw: {yaw:.1f}°) /\n 장치를 다시 설치하세요 (피치: {pitch:.1f}°, 요: {yaw:.1f}°)"
  return NormalPermanentAlert("Calibration Invalid /\n 캘리브레이션이 유효하지 않습니다", angles)

def overheat_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  cpu = max(sm['deviceState'].cpuTempC, default=0.)
  gpu = max(sm['deviceState'].gpuTempC, default=0.)
  temp = max((cpu, gpu, sm['deviceState'].memoryTempC))
  return NormalPermanentAlert("System Overheated /\n 시스템의 온도가 높습니다", f"{temp:.0f} °C")

def low_memory_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NormalPermanentAlert("Low Memory / 메모리 부족", f"{sm['deviceState'].memoryUsagePercent}% used / 사용 중")

def high_cpu_usage_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  x = max(sm['deviceState'].cpuUsagePercent, default=0.)
  return NormalPermanentAlert("High CPU Usage / CPU 사용량 높음", f"{x}% used / 사용 중")

def modeld_lagging_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  return NormalPermanentAlert("Driving Model Lagging /\n 주행 모델 지연", f"{sm['modelV2'].frameDropPerc:.1f}% frames dropped / 프레임 드롭")

def wrong_car_mode_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  text = "Enable Adaptive Cruise to Engage /\n 적응형 크루즈를 활성화하세요"
  if CP.carName == "honda":
    text = "Enable Main Switch to Engage /\n 메인 스위치를 활성화하세요"
  return NoEntryAlert(text)


def joystick_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int) -> Alert:
  axes = sm['testJoystick'].axes
  gb, steer = list(axes)[:2] if len(axes) else (0., 0.)
  vals = f"Gas: {round(gb * 100.)}%, Steer: {round(steer * 100.)}% /\n 가스: {round(gb * 100.)}%, 조향: {round(steer * 100.)}%"
  return NormalPermanentAlert("Joystick Mode /\n 조이스틱 모드", vals)


# ********** EVENTS dictionary **********

EVENTS: Dict[int, Dict[str, Union[Alert, AlertCallbackType]]] = {
  # ********** events with no alerts **********

  EventName.stockFcw: {},

  # ********** events only containing alerts displayed in all states **********

  EventName.joystickDebug: {
    ET.WARNING: joystick_alert,
    ET.PERMANENT: NormalPermanentAlert("Joystick Mode /\n 조이스틱 모드"),
  },

  EventName.controlsInitializing: {
    ET.NO_ENTRY: NoEntryAlert("System Initializing /\n 시스템 초기화 중"),
  },

  EventName.startup: {
    ET.PERMANENT: StartupAlert("Be ready to take over at any time /\n 언제든지 제어할 준비를 하세요")
  },

  EventName.startupMaster: {
    ET.PERMANENT: startup_master_alert,
  },

  # Car is recognized, but marked as dashcam only
  EventName.startupNoControl: {
    ET.PERMANENT: StartupAlert("Dashcam mode /\n 블랙박스 모드"),
    ET.NO_ENTRY: NoEntryAlert("Dashcam mode /\n 블랙박스 모드"),
  },

  # Car is not recognized
  EventName.startupNoCar: {
    ET.PERMANENT: StartupAlert("Dashcam mode for unsupported car /\n 지원되지 않는 차량의 블랙박스 모드"),
  },

  EventName.startupNoFw: {
    ET.PERMANENT: StartupAlert("Car Unrecognized /\n 자동차를 인식할 수 없습니다",
                               "Check comma power connections /\n 커뮤니티 설정에서 차량을 선택하세요",
                               alert_status=AlertStatus.userPrompt),
  },

  EventName.dashcamMode: {
    ET.PERMANENT: NormalPermanentAlert("Dashcam Mode /\n 블랙박스 모드",
                                       priority=Priority.LOWEST),
  },

  EventName.invalidLkasSetting: {
    ET.PERMANENT: NormalPermanentAlert("Stock LKAS is on /\n 차량 LKAS 버튼 상태 확인",
                                       "Turn off stock LKAS to engage /\n 차량 LKAS 버튼 OFF 후 활성화됩니다"),
  },

  EventName.cruiseMismatch: {
    #ET.PERMANENT: ImmediateDisableAlert("openpilot failed to cancel cruise / 오픈파일럿이 크루즈 취소에 실패했습니다"),
  },

  # openpilot doesn't recognize the car. This switches openpilot into a
  # read-only mode. This can be solved by adding your fingerprint.
  # See https://github.com/commaai/openpilot/wiki/Fingerprinting for more information
  EventName.carUnrecognized: {
    ET.PERMANENT: NormalPermanentAlert("Dashcam Mode /\n 블랙박스 모드",
                                       "Car Unrecognized /\n 커뮤니티 설정에서 차량을 선택하세요",
                                       priority=Priority.LOWEST),
  },
  
  EventName.stockAeb: {
    ET.PERMANENT: Alert(
      "BRAKE! /\n 브레이크!",
      "Stock AEB: Risk of Collision /\n 순정 AEB: 추돌 위험",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.none, 2.),
    ET.NO_ENTRY: NoEntryAlert("Stock AEB: Risk of Collision /\n 순정 AEB: 추돌 위험"),
  },
  
  EventName.fcw: {
    ET.PERMANENT: Alert(
      "BRAKE! /\n 브레이크!",
      "Risk of Collision /\n 추돌 위험",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGHEST, VisualAlert.fcw, AudibleAlert.warningSoft, 2.),
  },
  
  EventName.ldw: {
    ET.PERMANENT: Alert(
      "Lane Departure Detected /\n 핸들을 잡아주세요",
      "차선이탈 감지됨",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.ldw, AudibleAlert.prompt, 3.),
  },
  
  # ********** events only containing alerts that display while engaged **********
  
  EventName.steerTempUnavailableSilent: {
    ET.WARNING: Alert(
      "Steering Temporarily Unavailable /\n 일시적으로 조향을 사용할 수 없음",
      "",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.prompt, 1.8),
  },
  
  EventName.preDriverDistracted: {
    ET.WARNING: Alert(
      "Pay Attention /\n 도로를 주시하세요 : 운전자 전방주시 불안",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
  
  EventName.promptDriverDistracted: {
    ET.WARNING: Alert(
      "Pay Attention /\n 도로를 주시하세요",
      "Driver Distracted /\n 운전자 전방주시 불안",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },
  
  EventName.driverDistracted: {
    ET.WARNING: Alert(
      "DISENGAGE IMMEDIATELY /\n 조향제어가 강제로 해제됩니다",
      "Driver Distracted /\n 운전자 도로주시 불안",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },
  
  EventName.preDriverUnresponsive: {
    ET.WARNING: Alert(
      "Touch Steering Wheel: No Face Detected /\n 핸들을 잡아주세요 : 운전자 인식 불가",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.none, .1, alert_rate=0.75),
  },
  
  EventName.promptDriverUnresponsive: {
    ET.WARNING: Alert(
      "Touch Steering Wheel /\n 핸들을 잡아주세요",
      "Driver Unresponsive /\n 운전자가 응답하지 않음",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.MID, VisualAlert.steerRequired, AudibleAlert.promptDistracted, .1),
  },
  
  EventName.driverUnresponsive: {
    ET.WARNING: Alert(
      "DISENGAGE IMMEDIATELY /\n 조향제어가 강제로 해제됩니다",
      "Driver Unresponsive /\n 운전자가 응답하지 않음",
      AlertStatus.critical, AlertSize.full,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.warningImmediate, .1),
  },

  EventName.manualRestart: {
    ET.WARNING: Alert(
      "TAKE CONTROL /\n 핸들을 잡아주세요",
      "Resume Driving Manually /\n 수동으로 재활성화하세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .2),
  },
  
  EventName.resumeRequired: {
    ET.WARNING: Alert(
      "Press Resume to Exit Standstill /\n 앞차량 멈춤",
      "앞차가 출발하면 자동 재출발",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.MID, VisualAlert.none, AudibleAlert.none, .2),
  },
  
  EventName.belowSteerSpeed: {
    ET.WARNING: below_steer_speed_alert,
  },
  
  EventName.preLaneChangeLeft: {
    ET.WARNING: Alert(
      "Steer Left to Start Lane Change Once Safe /\n 좌측 차선으로 변경합니다",
      "좌측 차선의 차량을 확인하세요",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },
  
  EventName.preLaneChangeRight: {
    ET.WARNING: Alert(
      "Steer Right to Start Lane Change Once Safe /\n 우측 차선으로 변경합니다",
      "우측 차선의 차량을 확인하세요",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1, alert_rate=0.75),
  },
  
  EventName.laneChangeBlocked: {
    ET.WARNING: Alert(
      "Car Detected in Blindspot /\n 후측방 차량감지",
      "차선에 차량이 감지되니 대기하세요",
      AlertStatus.userPrompt, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.prompt, .1),
  },
  
  EventName.laneChange: {
    ET.WARNING: Alert(
      "Changing Lanes /\n 차선을 변경합니다",
      "후측방 차량에 주의하세요",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
  
  EventName.steerSaturated: {
    ET.WARNING: Alert(
      "Take Control /\n 핸들을 잡아주세요",
      "Turn Exceeds Steering Limit /\n 조향제어 제한을 초과함",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.LOW, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 2.),
  },

  # Thrown when the fan is driven at >50% but is not rotating
  EventName.fanMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("Fan Malfunction /\n 팬 오작동", "Likely Hardware Issue /\n 하드웨어를 점검하세요"),
  },

  # Camera is not outputting frames
  EventName.cameraMalfunction: {
    ET.PERMANENT: camera_malfunction_alert,
    ET.SOFT_DISABLE: soft_disable_alert("Camera Malfunction /\n 카메라 작동 오류"),
    ET.NO_ENTRY: NoEntryAlert("Camera Malfunction: Reboot Your Device /\n 하드웨어를 점검하세요 : 재부팅 하세요"),
  },
  # Camera framerate too low
  EventName.cameraFrameRate: {
    ET.PERMANENT: NormalPermanentAlert("Camera Frame Rate Low /\n 카메라 프레임 레이트가 낮습니다.", "Reboot your Device /\n 재부팅 하세요"),
    ET.SOFT_DISABLE: soft_disable_alert("Camera Frame Rate Low /\n 카메라 프레임 레이트가 낮습니다."),
    ET.NO_ENTRY: NoEntryAlert("Camera Frame Rate Low: Reboot Your Device /\n 카메라 프레임 레이트가 낮습니다. : 재부팅 하세요"),
  },

  # Unused
  EventName.gpsMalfunction: {
    ET.PERMANENT: NormalPermanentAlert("GPS Malfunction /\n GPS 작동 오류", "Likely Hardware Issue /\n 하드웨어를 점검하세요"),
  },

  EventName.locationdTemporaryError: {
    ET.NO_ENTRY: NoEntryAlert("locationd Temporary Error /\n 위치 정보 임시 오류"),
    ET.SOFT_DISABLE: soft_disable_alert("locationd Temporary Error /\n 위치 정보 임시 오류"),
  },
  
  EventName.locationdPermanentError: {
    ET.NO_ENTRY: NoEntryAlert("locationd Permanent Error /\n 위치 정보 영구 오류"),
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("locationd Permanent Error /\n 위치 정보 영구 오류"),
    ET.PERMANENT: NormalPermanentAlert("locationd Permanent Error /\n 위치 정보 영구 오류"),
  },

  # openpilot tries to learn certain parameters about your car by observing
  # how the car behaves to steering inputs from both human and openpilot driving.
  # This includes:
  # - steer ratio: gear ratio of the steering rack. Steering angle divided by tire angle
  # - tire stiffness: how much grip your tires have
  # - angle offset: most steering angle sensors are offset and measure a non zero angle when driving straight
  # This alert is thrown when any of these values exceed a sanity check. This can be caused by
  # bad alignment or bad sensor data. If this happens consistently consider creating an issue on GitHub

  EventName.paramsdTemporaryError: {
    ET.NO_ENTRY: NoEntryAlert("paramsd Temporary Error /\n paramsd 일시적 오류"),
    ET.SOFT_DISABLE: soft_disable_alert("paramsd Temporary Error /\n paramsd 일시적 오류"),
  },
  
  EventName.paramsdPermanentError: {
    ET.NO_ENTRY: NoEntryAlert("paramsd Permanent Error /\n paramsd 영구적 오류"),
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("paramsd Permanent Error /\n paramsd 영구적 오류"),
    ET.PERMANENT: NormalPermanentAlert("paramsd Permanent Error /\n paramsd 영구적 오류"),
  },
  
  # ********** events that affect controls state transitions **********
  
  EventName.pcmEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },
  
  EventName.buttonEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },
  
  EventName.pcmDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },
  
  EventName.buttonCancel: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Cancel Pressed /\n 취소 버튼 작동"),
  },
  
  EventName.brakeHold: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Active /\n 브레이크 감지됨"),
  },
  
  EventName.parkBrake: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Parking Brake Engaged /\n 주차 브레이크를 해제하세요"),
  },
  
  EventName.pedalPressed: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Pedal Pressed /\n 브레이크 감지됨",
                              visual_alert=VisualAlert.brakePressed),
  },
  
  EventName.preEnableStandstill: {
    ET.PRE_ENABLE: Alert(
      "Release Brake to Engage /\n 작동을 위해 브레이크를 해제하세요",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1, creation_delay=1.),
  },
  
  EventName.gasPressedOverride: {
    ET.OVERRIDE_LONGITUDINAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },
  
  EventName.steerOverride: {
    ET.OVERRIDE_LATERAL: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .1),
  },
  
  EventName.wrongCarMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: wrong_car_mode_alert,
  },
  
  EventName.resumeBlocked: {
    ET.NO_ENTRY: NoEntryAlert("Press Set to Engage /\n 재개 버튼을 누르세요"),
  },
  
  EventName.wrongCruiseMode: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
    ET.NO_ENTRY: NoEntryAlert("Adaptive Cruise Disabled /\n 적응형 크루즈 비활성화됨"),
  },
  
  EventName.steerTempUnavailable: {
    ET.SOFT_DISABLE: soft_disable_alert("Steering Temporarily Unavailable /\n 조향제어 일시적으로 사용불가"),
    ET.NO_ENTRY: NoEntryAlert("Steering Temporarily Unavailable /\n 조향제어 일시적으로 사용불가"),
  },
  
  EventName.steerTimeLimit: {
    ET.SOFT_DISABLE: soft_disable_alert("Vehicle Steering Time Limit /\n 조향 시간 제한"),
    ET.NO_ENTRY: NoEntryAlert("Vehicle Steering Time Limit /\n 조향 시간 제한"),
  },
  
  EventName.outOfSpace: {
    ET.PERMANENT: out_of_space_alert,
    ET.NO_ENTRY: NoEntryAlert("Out of Storage /\n 저장공간 부족"),
  },
  
  EventName.belowEngageSpeed: {
    ET.NO_ENTRY: below_engage_speed_alert,
  },
  
  EventName.sensorDataInvalid: {
    ET.PERMANENT: Alert(
      "Sensor Data Invalid /\n 센서 오류 입니다.",
      "Possible Hardware Issue /\n 장치를 재부팅 하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOWER, VisualAlert.none, AudibleAlert.none, .2, creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("Sensor Data Invalid /\n 센서의 데이터가 유효하지 않습니다"),
    ET.SOFT_DISABLE: soft_disable_alert("Sensor Data Invalid /\n 센서의 데이터가 유효하지 않습니다"),
  },

  EventName.noGps: {
    ET.PERMANENT: no_gps_alert,
  },
  
  EventName.soundsUnavailable: {
    ET.PERMANENT: NormalPermanentAlert("Speaker not found /\n 스피커가 감지되지않습니다", "Reboot your Device /\n 재부팅 해주세요"),
    ET.NO_ENTRY: NoEntryAlert("Speaker not found /\n 스피커가 감지되지않습니다"),
  },
  
  EventName.tooDistracted: {
    ET.NO_ENTRY: NoEntryAlert("Distraction Level Too High /\n 주의 산만 수준이 너무 높음"),
  },
  
  EventName.overheat: {
    ET.PERMANENT: overheat_alert,
    ET.SOFT_DISABLE: soft_disable_alert("System Overheated /\n 장치 과열됨"),
    ET.NO_ENTRY: NoEntryAlert("System Overheated /\n 장치 과열됨"),
  },
  
  EventName.wrongGear: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Gear not D /\n 기어를 [D]로 변경하세요"),
    ET.NO_ENTRY: NoEntryAlert("Gear not D /\n 기어를 [D]로 변경하세요"),
  },

  # This alert is thrown when the calibration angles are outside of the acceptable range.
  # For example if the device is pointed too much to the left or the right.
  # Usually this can only be solved by removing the mount from the windshield completely,
  # and attaching while making sure the device is pointed straight forward and is level.
  # See https://comma.ai/setup for more information
  EventName.calibrationInvalid: {
    ET.PERMANENT: calibration_invalid_alert,
    ET.SOFT_DISABLE: soft_disable_alert("Calibration Invalid: Remount Device & Recalibrate /\n 캘리브레이션 오류: 장치 위치변경후 캘리브레이션을 다시하세요"),
    ET.NO_ENTRY: NoEntryAlert("Calibration Invalid: Remount Device & Recalibrate /\n 캘리브레이션 오류: 장치 위치변경후 캘리브레이션을 다시하세요"),
  },
  
  EventName.calibrationIncomplete: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert("Calibration Incomplete /\n 캘리브레이션 맞지 않습니다."),
    ET.NO_ENTRY: NoEntryAlert("Calibration in Progress /\n 캘리브레이션 진행중입니다"),
  },
  
  EventName.calibrationRecalibrating: {
    ET.PERMANENT: calibration_incomplete_alert,
    ET.SOFT_DISABLE: soft_disable_alert("Device Remount Detected: Recalibrating /\n 본체의 위치가 변경되었습니다.: 캘리브레이션 재시작중"),
    ET.NO_ENTRY: NoEntryAlert("Remount Detected: Recalibrating /\n 본체의 위치가 변경되었습니다.: 캘리브레이션 재시작중"),
  },
  
  EventName.doorOpen: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Door Open / 도어 열림"),
    ET.NO_ENTRY: NoEntryAlert("Door Open / 도어 열림"),
  },
  # MJ
  EventName.seatbeltNotLatched: {
    ET.SOFT_DISABLE: user_soft_disable_alert("Seatbelt Unlatched /\n 안전벨트를 착용해주세요"),
    ET.NO_ENTRY: NoEntryAlert("Seatbelt Unlatched /\n 안전벨트를 착용해주세요"),
  },
  
  EventName.espDisabled: {
    ET.SOFT_DISABLE: soft_disable_alert("Electronic Stability Control Disabled /\n 차량 안정성 제어장치 비활성화"),
    ET.NO_ENTRY: NoEntryAlert("Electronic Stability Control Disabled /\n 차량 안정성 제어장치 비활성화"),
  },
  
  EventName.lowBattery: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Battery /\n 배터리를 충전하세요"),
    ET.NO_ENTRY: NoEntryAlert("Low Battery /\n 배터리를 충전하세요"),
  },

  # Different openpilot services communicate between each other at a certain
  # interval. If communication does not follow the regular schedule this alert
  # is thrown. This can mean a service crashed, did not broadcast a message for
  # ten times the regular interval, or the average interval is more than 10% too high.
  EventName.commIssue: {
    ET.SOFT_DISABLE: soft_disable_alert("Communication Issue between Processes /\n 장치 프로세스 통신오류"),
    ET.NO_ENTRY: comm_issue_alert,
  },

  EventName.commIssueAvgFreq: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Communication Rate between Processes /\n 장치 프로세스 레이트가 낮습니다"),
    ET.NO_ENTRY: NoEntryAlert("Low Communication Rate between Processes /\n 장치 프로세스 레이트가 낮습니다"),
  },
  
  EventName.controlsdLagging: {
    ET.SOFT_DISABLE: soft_disable_alert("Controls Lagging /\n 콘트롤이 지연되고 있습니다"),
    ET.NO_ENTRY: NoEntryAlert("Controls Process Lagging: Reboot Your Device /\n 컨트롤 프로세스가 지연되고 있습니다: 장치를 재부팅하세요"),
  },

  # Thrown when manager detects a service exited unexpectedly while driving
  EventName.processNotRunning: {
    ET.NO_ENTRY: process_not_running_alert,
    ET.SOFT_DISABLE: soft_disable_alert("Process Not Running /\n 프로세스가 작동되지 않고 있습니다"),
  },
  
  EventName.radarFault: {
    ET.SOFT_DISABLE: soft_disable_alert("Radar Error: Restart the Car /\n 레이더 오류 : 차량을 재가동하세요"),
    ET.NO_ENTRY: NoEntryAlert("Radar Error: Restart the Car /\n 레이더 오류 : 차량을 재가동하세요"),
  },

  # Every frame from the camera should be processed by the model. If modeld
  # is not processing frames fast enough they have to be dropped. This alert is
  # thrown when over 20% of frames are dropped.
  EventName.modeldLagging: {
    ET.SOFT_DISABLE: soft_disable_alert("Driving Model Lagging /\n 주행모델 지연됨"),
    ET.NO_ENTRY: NoEntryAlert("Driving Model Lagging /\n 주행모델 지연됨"),
    ET.PERMANENT: modeld_lagging_alert,
  },

  # Besides predicting the path, lane lines and lead car data the model also
  # predicts the current velocity and rotation speed of the car. If the model is
  # very uncertain about the current velocity while the car is moving, this
  # usually means the model has trouble understanding the scene. This is used
  # as a heuristic to warn the driver.
  EventName.posenetInvalid: {
    ET.SOFT_DISABLE: soft_disable_alert("Posenet Speed Invalid /\n Posenet 속도 무효"),
    ET.NO_ENTRY: posenet_invalid_alert,
  },

  # When the localizer detects an acceleration of more than 40 m/s^2 (~4G) we
  # alert the driver the device might have fallen from the windshield.
  EventName.deviceFalling: {
    ET.SOFT_DISABLE: soft_disable_alert("Device Fell Off Mount /\n 장치가 마운트에서 떨어짐"),
    ET.NO_ENTRY: NoEntryAlert("Device Fell Off Mount /\n 장치가 마운트에서 떨어짐"),
  },
   
  EventName.lowMemory: {
    ET.SOFT_DISABLE: soft_disable_alert("Low Memory: Reboot Your Device /\n 메모리 부족 : 장치를 재가동하세요"),
    ET.PERMANENT: low_memory_alert,
    ET.NO_ENTRY: NoEntryAlert("Low Memory: Reboot Your Device /\n 메모리 부족 : 장치를 재가동하세요"),
  },
  
  EventName.highCpuUsage: {
    ET.NO_ENTRY: high_cpu_usage_alert,
  },
  
  EventName.accFaulted: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Fault: Restart the Car /\n 크루즈 오류: 재시동 하세요"),
    ET.PERMANENT: NormalPermanentAlert("Cruise Fault: Restart the car to engage /\n 크루즈 오류: 재시동 하세요"),
    ET.NO_ENTRY: NoEntryAlert("Cruise Fault: Restart the Car /\n 크루즈 오류: 재시동 하세요"),
  },
  
  EventName.controlsMismatch: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Controls Mismatch /\n 컨트롤 불일치"),
    ET.NO_ENTRY: NoEntryAlert("Controls Mismatch /\n 컨트롤 불일치"),
  },
  
  EventName.roadCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera CRC Error - Road /\n 후면 카메라 오류",
                                       duration=1.,
                                       creation_delay=30.),
  },
  
  EventName.wideRoadCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera CRC Error - Road Fisheye /\n 후면 광각 카메라 오류",
                                       duration=1.,
                                       creation_delay=30.),
  },
  
  EventName.driverCameraError: {
    ET.PERMANENT: NormalPermanentAlert("Camera CRC Error - Driver /\n 전면 카메라 오류",
                                       duration=1.,
                                       creation_delay=30.),
  },

  # Sometimes the USB stack on the device can get into a bad state
  # causing the connection to the panda to be lost
  EventName.usbError: {
    ET.SOFT_DISABLE: soft_disable_alert("USB Error: Reboot Your Device /\n USB 에러 : 장치를 재시작 하세요"),
    ET.PERMANENT: NormalPermanentAlert("USB Error: Reboot Your Device /\n USB 에러 : 장치를 재시작 하세요", ""),
    ET.NO_ENTRY: NoEntryAlert("USB Error: Reboot Your Device /\n USB 에러 : 장치를 재시작 하세요"),
  },

  # This alert can be thrown for the following reasons:
  # - No CAN data received at all
  # - CAN data is received, but some message are not received at the right frequency
  # If you're not writing a new car port, this is usually cause by faulty wiring
  EventName.canError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("CAN Error /\n CAN 오류 : 하드웨어를 점검하세요"),
    ET.PERMANENT: Alert(
      "CAN Error: Check Connections /\n CAN 오류 : 하드웨어를 점검하세요",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("CAN Error: Check Connections /\n CAN 오류 : 하드웨어를 점검하세요"),
  },
  
  EventName.canBusMissing: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("CAN Bus Disconnected /\n CAN 버스 연결 끊김"),
    ET.PERMANENT: Alert(
      "CAN Bus Disconnected: Likely Faulty Cable /\n CAN 버스 연결 끊김 : 케이블 점검",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1., creation_delay=1.),
    ET.NO_ENTRY: NoEntryAlert("CAN Bus Disconnected: Check Connections /\n CAN 버스 연결 끊김 : 케이블 점검"),
  },
  
  EventName.steerUnavailable: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("LKAS Fault: Restart the Car /\n LKAS 오류 : 차량을 재가동하세요"),
    ET.PERMANENT: NormalPermanentAlert("LKAS Fault: Restart the car to engage /\n LKAS 오류 : 차량을 재가동하세요"),
    ET.NO_ENTRY: NoEntryAlert("LKAS Fault: Restart the Car /\n LKAS 오류 : 차량을 재가동하세요"),
  },
  
  EventName.reverseGear: {
    ET.PERMANENT: Alert(
      "Reverse Gear /\n 후진 기어",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.USER_DISABLE: SoftDisableAlert("Reverse Gear /\n 기어 [R] 상태"),
    ET.NO_ENTRY: NoEntryAlert("Reverse Gear /\n 기어 [R] 상태"),
  },

  # On cars that use stock ACC the car can decide to cancel ACC for various reasons.
  # When this happens we can no long control the car so the user needs to be warned immediately.
  EventName.cruiseDisabled: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Cruise Is Off /\n 크루즈 꺼짐"),
  },

  # For planning the trajectory Model Predictive Control (MPC) is used. This is
  # an optimization algorithm that is not guaranteed to find a feasible solution.
  # If no solution is found or the solution has a very high cost this alert is thrown.
  EventName.plannerError: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Planner Solution Error /\n 플래너 솔루션 오류"),
    ET.NO_ENTRY: NoEntryAlert("Planner Solution Error /\n 플래너 솔루션 오류"),
  },

  # When the relay in the harness box opens the CAN bus between the LKAS camera
  # and the rest of the car is separated. When messages from the LKAS camera
  # are received on the car side this usually means the relay hasn't opened correctly
  # and this alert is thrown.
  EventName.relayMalfunction: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Harness Relay Malfunction /\n 하네스 오작동"),
    ET.PERMANENT: NormalPermanentAlert("Harness Relay Malfunction /\n 하네스 오작동", "Check Hardware /\n 하드웨어를 점검하세요"),
    ET.NO_ENTRY: NoEntryAlert("Harness Relay Malfunction /\n 하네스 오작동"),
  },
  
  EventName.speedTooLow: {
    ET.IMMEDIATE_DISABLE: Alert(
      "openpilot Canceled /\n 오픈파일럿 사용불가",
      "Speed too low /\n 속도를 높이고 재가동하세요",
      AlertStatus.normal, AlertSize.mid,
      Priority.HIGH, VisualAlert.none, AudibleAlert.disengage, 3.),
  },

  # When the car is driving faster than most cars in the training data, the model outputs can be unpredictable.
  EventName.speedTooHigh: {
    ET.WARNING: Alert(
      "Speed Too High /\n 속도가 너무 높습니다",
      "Model uncertain at this speed /\n 속도를 줄여주세요",
      AlertStatus.userPrompt, AlertSize.mid,
      Priority.HIGH, VisualAlert.steerRequired, AudibleAlert.promptRepeat, 4.),
    ET.NO_ENTRY: NoEntryAlert("Slow down to engage /\n 속도를 줄여서 재가동하세요"),
  },
  
  EventName.lowSpeedLockout: {
    ET.PERMANENT: NormalPermanentAlert("Cruise Fault: Restart the car to engage /\n 크루즈 오류 : 차량을 재가동하세요"),
    ET.NO_ENTRY: NoEntryAlert("Cruise Fault: Restart the Car /\n 크루즈 오류 : 차량을 재가동하세요"),
  },
  
  EventName.lkasDisabled: {
    ET.PERMANENT: NormalPermanentAlert("LKAS Disabled: Enable LKAS to engage /\n LKAS 비활성화됨: LKAS를 활성화하여 재가동하세요"),
    ET.NO_ENTRY: NoEntryAlert("LKAS Disabled /\n LKAS 비활성화됨"),
  },
  
  EventName.vehicleSensorsInvalid: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Vehicle Sensors Invalid /\n 차량 센서 이상"),
    ET.PERMANENT: NormalPermanentAlert("Vehicle Sensors Calibrating /\n 차량 센서 캘리브레이션중", "Drive to Calibrate /\n 캘리브레이션을 위해 주행중"),
    ET.NO_ENTRY: NoEntryAlert("Vehicle Sensors Calibrating /\n 차량 센서 캘리브레이션중"),
  },
  
  EventName.slowingDownSpeed: {
    ET.PERMANENT: Alert("Slowing down /\n 속도를 조절합니다", "", AlertStatus.normal, AlertSize.small,
                        Priority.LOW, VisualAlert.none, AudibleAlert.none, .1),
  },
  
  EventName.slowingDownSpeedSound: {
    ET.PERMANENT: Alert("Slowing down /\n 속도를 조절합니다", "", AlertStatus.normal, AlertSize.small,
                        Priority.MID, VisualAlert.none, AudibleAlert.slowingDownSpeed, 2.),
  },
  
  EventName.cruiseOn: {
    ET.PERMANENT: Alert("", "", AlertStatus.normal, AlertSize.none,
                        Priority.MID, VisualAlert.none, AudibleAlert.cruiseOn, 1.),
  },
  
  EventName.cruiseOff: {
    ET.PERMANENT: Alert("", "", AlertStatus.normal, AlertSize.none,
                        Priority.MID, VisualAlert.none, AudibleAlert.cruiseOff, 1.),
  },


}


if __name__ == '__main__':
  # print all alerts by type and priority
  from cereal.services import SERVICE_LIST
  from collections import defaultdict

  event_names = {v: k for k, v in EventName.schema.enumerants.items()}
  alerts_by_type: Dict[str, Dict[Priority, List[str]]] = defaultdict(lambda: defaultdict(list))

  CP = car.CarParams.new_message()
  CS = car.CarState.new_message()
  sm = messaging.SubMaster(list(SERVICE_LIST.keys()))

  for i, alerts in EVENTS.items():
    for et, alert in alerts.items():
      if callable(alert):
        alert = alert(CP, CS, sm, False, 1)
      alerts_by_type[et][alert.priority].append(event_names[i])

  all_alerts: Dict[str, List[tuple[Priority, List[str]]]] = {}
  for et, priority_alerts in alerts_by_type.items():
    all_alerts[et] = sorted(priority_alerts.items(), key=lambda x: x[0], reverse=True)

  for status, evs in sorted(all_alerts.items(), key=lambda x: x[0]):
    print(f"**** {status} ****")
    for p, alert_list in evs:
      print(f"  {repr(p)}:")
      print("   ", ', '.join(alert_list), "\n")
