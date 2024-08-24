import random

import numpy as np
import logging

from common.numpy_fast import clip, interp
from cereal import car
from openpilot.common.conversions import Conversions as CV
from openpilot.selfdrive.controls.lib.drive_helpers import (VCruiseHelper, V_CRUISE_MIN, V_CRUISE_MAX, V_CRUISE_UNSET)
from openpilot.selfdrive.car.hyundai.values import Buttons
from openpilot.common.params import Params
from openpilot.selfdrive.controls.neokii.navi_controller import SpeedLimiter
from openpilot.selfdrive.modeld.constants import ModelConstants

SYNC_MARGIN = 3.
MIN_CURVE_SPEED = 32. * CV.KPH_TO_MS

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type

logger = logging.getLogger('SpeedControllerLogger')
logger.setLevel(logging.DEBUG)

console_handler = logging.StreamHandler()
console_handler.setLevel(logging.DEBUG)

file_handler = logging.FileHandler('speed_controller.log')
file_handler.setLevel(logging.DEBUG)

formatter = logging.Formatter('%(asctime)s %(levelname)s: %(message)s')
console_handler.setFormatter(formatter)
file_handler.setFormatter(formatter)

logger.addHandler(console_handler)
logger.addHandler(file_handler)

'''
这个代码片段是一个自动驾驶系统中的速度控制器类 (SpeedController) 的实现。该类主要用于在自动驾驶过程中，基于车辆状态和环境信息动态调整车辆的目标速度。以下是对该代码的主要功能进行逐步解析：

1. 初始化 (__init__ 方法)
初始化了控制器参数，如车辆参数 (CP) 和控制接口 (CI)。
初始化了速度控制的基本配置，如是否使用公制单位 (is_metric)、是否启用了实验模式 (experimental_mode)。
设置了一些初始变量，如目标速度、最大速度、曲线速度等。
2. 内部方法
_kph_to_clu: 将公里每小时 (KPH) 转换为车辆仪表盘使用的单位。
_get_alive_count 和 _get_wait_count: 这些方法用于获取调整速度的等待计数，用于在不同情况下调整速度的时间间隔。
reset: 重置控制器的状态，包括按钮状态、计时器、目标速度等。
inject_events: 在车辆巡航状态启用时，根据是否需要减速，触发对应的事件提醒（如减速警报）。
_cal_max_speed: 计算当前条件下车辆的最大允许速度，考虑了道路限速、曲线速度、前车速度等因素。
get_lead: 从雷达数据中获取前方车辆的信息（如果存在）。
_get_long_lead_speed: 计算前方车辆的速度，并根据距离和速度差调整目标速度。
_cal_curve_speed: 计算车辆在弯道中的安全速度，基于曲率和侧向加速度。
_cal_target_speed: 计算车辆的目标速度，考虑驾驶员输入（如油门）和车辆当前速度。
_update_max_speed: 更新车辆的最大速度设置。
_get_button: 根据当前的目标速度与设定速度的差异，决定是否调整巡航速度，并返回相应的按键指令（如加速或减速）。
update_v_cruise: 更新巡航速度，考虑了车辆是否启用了巡航控制、驾驶员输入以及系统设定。
3. 日志记录
通过日志系统记录了控制器的初始化状态和每个关键步骤的计算结果，这对于调试和性能分析非常有帮助。
4. 主要逻辑
该速度控制器在巡航状态下，会不断计算出合适的巡航速度，并根据环境信息（如弯道、前车）动态调整速度，确保车辆的安全与舒适。
总结
该类主要实现了基于环境和车辆状态的动态速度调整功能，是自动驾驶系统中关键的控制模块之一。代码通过详细的日志记录和多个辅助计算函数，确保了在不同的驾驶场景中都能准确地控制车辆速度。
'''

class SpeedController:
  def __init__(self, CP, CI):
    self.CP = CP
    self.CI = CI

    self.long_control = CP.openpilotLongitudinalControl
    self.pcmcruise = CP.pcmCruise

    self.is_metric = Params().get_bool('IsMetric')
    self.experimental_mode = Params().get_bool("ExperimentalMode") and self.long_control

    self.speed_conv_to_ms = CV.KPH_TO_MS if self.is_metric else CV.MPH_TO_MS
    self.speed_conv_to_clu = CV.MS_TO_KPH if self.is_metric else CV.MS_TO_MPH

    self.min_set_speed_clu = self._kph_to_clu(V_CRUISE_MIN)
    self.max_set_speed_clu = self._kph_to_clu(V_CRUISE_MAX)

    self.btn = Buttons.NONE

    self.target_speed = 0.
    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.
    self.cruise_speed_kph = 0.
    self.real_set_speed_kph = 0.
    self.v_cruise_kph_last = 0
    self.v_cruise_kph = V_CRUISE_UNSET
    self.v_cruise_cluster_kph = V_CRUISE_UNSET

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False
    self.active_cam = False
    self.prev_cruise_enabled = False
    self.limited_lead = False

    self.wait_timer = 0
    self.alive_timer = 0
    self.alive_index = 0
    self.wait_index = 0
    self.alive_count = 0
    self.wait_count_list, self.alive_count_list = CI.get_params_adjust_set_speed(CP)
    random.shuffle(self.wait_count_list)
    random.shuffle(self.alive_count_list)

    self.v_cruise_helper = VCruiseHelper(self.CP)

    logger.debug(f'SpeedController initialized with: is_metric={self.is_metric}, experimental_mode={self.experimental_mode},'
                 f'pcmcruise={self.pcmcruise}, longcontrol={self.long_control}')

  def _kph_to_clu(self, kph):
    return int(kph * CV.KPH_TO_MS * self.speed_conv_to_clu)

  def _get_alive_count(self):
    count = self.alive_count_list[self.alive_index]
    self.alive_index += 1
    if self.alive_index >= len(self.alive_count_list):
      self.alive_index = 0
    return count


  def _get_wait_count(self):
    count = self.wait_count_list[self.wait_index]
    self.wait_index += 1
    if self.wait_index >= len(self.wait_count_list):
      self.wait_index = 0
    return count


  def reset(self):
    self.btn = Buttons.NONE

    self.wait_timer = 0
    self.alive_timer = 0
    self.target_speed = 0.
    self.max_speed_clu = 0.
    self.curve_speed_ms = 0.

    self.slowing_down = False
    self.slowing_down_alert = False
    self.slowing_down_sound_alert = False

    logger.debug('SpeedController state has been reset.')

  def inject_events(self, CS, events):
    if CS.cruiseState.enabled:
      if self.slowing_down_sound_alert:
        self.slowing_down_sound_alert = False
        events.add(EventName.slowingDownSpeedSound)
      elif self.slowing_down_alert:
        events.add(EventName.slowingDownSpeed)

  def _cal_max_speed(self, CS, sm, clu_speed, v_cruise_kph):
    # kph
    apply_limit_speed, road_limit_speed, left_dist, first_started, cam_type, max_speed_log = \
      SpeedLimiter.instance().get_max_speed(clu_speed, self.is_metric)

    self._cal_curve_speed(sm, CS.vEgo, sm.frame)
    if self.curve_speed_ms >= MIN_CURVE_SPEED:
      max_speed_clu = min(v_cruise_kph * CV.KPH_TO_MS, self.curve_speed_ms) * self.speed_conv_to_clu
    else:
      max_speed_clu = self._kph_to_clu(v_cruise_kph)

    self.active_cam = road_limit_speed > 0 and left_dist > 0

    if apply_limit_speed >= self._kph_to_clu(10):
      if first_started:
        self.max_speed_clu = clu_speed

      max_speed_clu = min(max_speed_clu, apply_limit_speed)

      if clu_speed > apply_limit_speed:
        if not self.slowing_down_alert and not self.slowing_down:
          self.slowing_down_sound_alert = True
          self.slowing_down = True
        self.slowing_down_alert = True
      else:
        self.slowing_down_alert = False
    else:
      self.slowing_down_alert = False
      self.slowing_down = False

    lead_speed = self._get_long_lead_speed(clu_speed, sm)
    if lead_speed >= self.min_set_speed_clu:
      if lead_speed < max_speed_clu:
        max_speed_clu = lead_speed
        if not self.limited_lead:
          self.max_speed_clu = clu_speed + 3.
          self.limited_lead = True
    else:
      self.limited_lead = False

    self._update_max_speed(int(round(max_speed_clu)))

    logger.debug(f'Calculated max_speed_clu: {max_speed_clu}')

    return max_speed_clu

  def get_lead(self, sm):
    radar = sm['radarState']
    if radar.leadOne.status:
      return radar.leadOne
    return None

  def _get_long_lead_speed(self, clu_speed, sm):
    if self.long_control:
      lead = self.get_lead(sm)
      if lead is not None:
        d = lead.dRel - 5.
        if 0. < d < -lead.vRel * 11. * 2. and lead.vRel < -1.:
          t = d / lead.vRel
          accel = -(lead.vRel / t) * self.speed_conv_to_clu
          accel *= 1.5 #1.2

          if accel < 0.:
            target_speed = clu_speed + accel
            target_speed = max(target_speed, self.min_set_speed_clu)
            return target_speed
    return 0


  def _cal_curve_speed(self, sm, v_ego, frame):
    if frame % 20 == 0:
      model_msg = sm['modelV2']
      if len(model_msg.position.x) == ModelConstants.IDX_N and len(model_msg.position.y) == ModelConstants.IDX_N:
        x = model_msg.position.x
        y = model_msg.position.y
        dy = np.gradient(y, x)
        d2y = np.gradient(dy, x)
        curv = d2y / (1 + dy ** 2) ** 1.5

        start = int(interp(v_ego, [10., 27.], [10, ModelConstants.IDX_N - 10]))
        curv = curv[start:min(start + 10, ModelConstants.IDX_N)]
        a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
        v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
        model_speed = np.mean(v_curvature) * 0.85

        if model_speed < v_ego:
          self.curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
        else:
          self.curve_speed_ms = 255.

        if np.isnan(self.curve_speed_ms):
          self.curve_speed_ms = 255.
      else:
        self.curve_speed_ms = 255.

  def _cal_target_speed(self, CS, clu_speed, v_cruise_kph, cruise_btn_pressed):
    override_speed = -1
    if not self.long_control:
      if CS.gasPressed and not cruise_btn_pressed:
        if clu_speed + SYNC_MARGIN > self._kph_to_clu(v_cruise_kph):
          set_speed = clip(clu_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          v_cruise_kph = int(round(set_speed * self.speed_conv_to_ms * CV.MS_TO_KPH))
          override_speed = v_cruise_kph

      self.target_speed = self._kph_to_clu(v_cruise_kph)
      if self.max_speed_clu > self.min_set_speed_clu:
        self.target_speed = clip(self.target_speed, self.min_set_speed_clu, self.max_speed_clu)

    elif CS.cruiseState.enabled:
      if CS.gasPressed and not cruise_btn_pressed:
        if clu_speed + SYNC_MARGIN > self._kph_to_clu(v_cruise_kph):
          set_speed = clip(clu_speed + SYNC_MARGIN, self.min_set_speed_clu, self.max_set_speed_clu)
          self.target_speed = set_speed

    logger.debug(f'Target speed calculated: {self.target_speed}, override speed : {override_speed}')

    return override_speed

  def _update_max_speed(self, max_speed):
    if not self.long_control or self.max_speed_clu <= 0:
      self.max_speed_clu = max_speed
    else:
      kp = 0.01
      error = max_speed - self.max_speed_clu
      self.max_speed_clu = self.max_speed_clu + error * kp

    logger.debug(f'Updated max_speed_clu: {self.max_speed_clu}')

  def _get_button(self, current_set_speed):
    if self.target_speed < self.min_set_speed_clu:
      return Buttons.NONE
    error = self.target_speed - current_set_speed
    if abs(error) < 0.9:
      return Buttons.NONE
    return Buttons.RES_ACCEL if error > 0 else Buttons.SET_DECEL

  def update_v_cruise(self, CS, sm, enabled):
    self.v_cruise_kph_last = self.v_cruise_kph
    v_cruise_kph = self.v_cruise_kph

    manage_button = not self.CP.openpilotLongitudinalControl or not self.CP.pcmCruise

    if CS.cruiseState.enabled:
      if manage_button:
        v_cruise_kph = self.v_cruise_helper.update_v_cruise(CS, enabled, self.is_metric)
      else:
        v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH
    else:
      v_cruise_kph = V_CRUISE_UNSET

    if self.prev_cruise_enabled != CS.cruiseState.enabled:
      self.prev_cruise_enabled = CS.cruiseState.enabled

      if CS.cruiseState.enabled:
        if not self.CP.pcmCruise:
          v_cruise_kph = self.v_cruise_helper.initialize_v_cruise(CS, self.experimental_mode)
        else:
          v_cruise_kph = CS.cruiseState.speed * CV.MS_TO_KPH

    self.real_set_speed_kph = v_cruise_kph
    if CS.cruiseState.enabled:
      clu_speed = CS.vEgoCluster * self.speed_conv_to_clu
      self._cal_max_speed(CS, sm, clu_speed, v_cruise_kph)
      self.cruise_speed_kph = float(clip(v_cruise_kph, V_CRUISE_MIN, self.max_speed_clu * self.speed_conv_to_ms * CV.MS_TO_KPH))

      override_speed = self._cal_target_speed(CS, clu_speed, self.real_set_speed_kph, self.CI.CS.cruise_buttons[-1] != Buttons.NONE)
      if override_speed > 0:
        v_cruise_kph = override_speed
    else:
      self.reset()

    self.v_cruise_kph = v_cruise_kph

    logger.debug(f'v_cruise_kph: {self.v_cruise_kph}, real set speed : {self.real_set_speed_kph}')

    self._update_message(CS)

  def spam_message(self, CS, can_sends):
    ascc_enabled = CS.cruiseState.enabled and 1 < CS.cruiseState.speed < 255 and not CS.brakePressed
    btn_pressed = self.CI.CS.cruise_buttons[-1] != Buttons.NONE

    if not self.long_control:
      if not ascc_enabled or btn_pressed:
        self.reset()
        self.wait_timer = max(self.alive_count_list) + max(self.wait_count_list)
        return

    if not ascc_enabled:
      self.reset()

    if self.wait_timer > 0:
      self.wait_timer -= 1
    elif ascc_enabled and CS.vEgo > 0.1:
      if self.alive_timer == 0:
        current_set_speed_clu = int(round(CS.cruiseState.speed * self.speed_conv_to_clu))
        self.btn = self._get_button(current_set_speed_clu)
        self.alive_count = self._get_alive_count()

      if self.btn != Buttons.NONE:
        can = self.CI.create_buttons(self.btn)
        if can is not None:
          can_sends.append(can)

        self.alive_timer += 1

        if self.alive_timer >= self.alive_count:
          self.alive_timer = 0
          self.wait_timer = self._get_wait_count()
          self.btn = Buttons.NONE
      else:
        if self.long_control and self.target_speed >= self.min_set_speed_clu:
          self.target_speed = 0.
    else:
      if self.long_control:
        self.target_speed = 0.

    logger.debug(f'Spam message - wait_timer: {self.wait_timer}, alive_timer: {self.alive_timer}, btn: {self.btn}')


  def _update_message(self, CS):
    exState = CS.exState
    exState.vCruiseKph = self.v_cruise_kph
    exState.cruiseMaxSpeed = self.real_set_speed_kph
    exState.applyMaxSpeed = self.cruise_speed_kph

