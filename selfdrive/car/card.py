#!/usr/bin/env python3
import os
import time

import cereal.messaging as messaging

from cereal import car
from openpilot.selfdrive.controls.neokii.speed_controller import SpeedController

from panda import ALTERNATIVE_EXPERIENCE

from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper

from openpilot.selfdrive.pandad import can_list_to_can_capnp
from openpilot.selfdrive.car import DT_CTRL
from openpilot.selfdrive.car.car_helpers import get_car, get_one_can
from openpilot.selfdrive.car.interfaces import CarInterfaceBase
from openpilot.selfdrive.controls.lib.events import Events

REPLAY = "REPLAY" in os.environ

EventName = car.CarEvent.EventName

'''
这段代码是用于汽车控制的核心部分，涉及到汽车状态的更新、事件处理和控制命令的发布。它通过读取 CAN 总线的数据来更新汽车状态，并将控制命令发送回车辆。
Car 类是汽车控制的核心类，它初始化 CAN 总线通信、状态管理、控制参数等。可以通过 CI 参数传入 CarInterface 实例。
'''

class Car:
  CI: CarInterfaceBase

  '''
  这些初始化操作设置了与 CAN 总线的通信通道，并初始化了状态管理器 (SubMaster) 和发布管理器 (PubMaster)，用于订阅和发布消息。
  '''
  def __init__(self, CI=None):
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'modelV2', 'radarState'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False

    self.last_actuators_output = car.CarControl.Actuators.new_message()

    self.params = Params()

    '''
    等待CAN消息：当CI为空时，程序会调用get_one_can(self.can_sock)等待接收到一条CAN消息，这保证了在继续执行之前至少有一条消息到达。

    获取车辆信息：然后程序会调用get_car函数，传入can_sock和sendcan，这将初始化车辆接口(CarInterface)对象 CI 和车辆参数(CarParams)对象 CP。get_car函数会根据接收到的CAN消息和车辆的具体情况来决定使用哪种接口和参数。

    实验性纵向控制：experimental_long_allowed 是一个布尔值，决定了实验性纵向控制是否启用。

    '''
    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      get_one_can(self.can_sock)

      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)
      experimental_long_allowed = self.params.get_bool("ExperimentalLongitudinalEnabled")
      self.CI, self.CP = get_car(self.can_sock, self.pm.sock['sendcan'], experimental_long_allowed, num_pandas)
    else:
      self.CI, self.CP = CI, CI.CP

    # set alternative experiences from parameters
    self.disengage_on_accelerator = self.params.get_bool("DisengageOnAccelerator")
    self.disengage_on_brake = self.params.get_bool("DisengageOnBrake")

    self.CP.alternativeExperience = 0
    if not self.disengage_on_accelerator:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.DISABLE_DISENGAGE_ON_GAS

    if not self.disengage_on_brake:
      self.CP.alternativeExperience |= ALTERNATIVE_EXPERIENCE.ALLOW_AEB

    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")

    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly

    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = car.CarParams.SafetyConfig.new_message()
      safety_config.safetyModel = car.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes)
    self.params.put_nonblocking("CarParamsCache", cp_bytes)
    self.params.put_nonblocking("CarParamsPersistent", cp_bytes)

    self.events = Events()

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

    self.speed_controller = SpeedController(self.CP, self.CI)

  '''
  state_update 方法从 CAN 总线读取数据并更新汽车状态 (CarState)。

  接收CAN数据：state_update() 方法从CAN总线上接收数据，并更新车辆状态 (CarState)。

  处理事件：update_events() 方法根据接收到的数据更新事件 (Events)。

  发布状态：state_publish() 方法将车辆状态 (CarState) 和车辆参数 (CarParams) 发布给其他模块或组件。

  控制更新：如果控制已经初始化且车辆不是被动模式（passive），则执行controls_update()方法，通过CAN总线发送控制信号。

  这个循环会持续运行，不断接收、处理和发送数据，确保车辆控制系统实时响应CAN总线上的消息。
  '''
  def state_update(self) -> car.CarState:
    """carState update loop, driven by can"""

    # Update carState from CAN
    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    CS = self.CI.update(self.CC_prev, can_strs)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.speed_controller.update_v_cruise(CS, self.sm, self.sm['carControl'].enabled)

    return CS
  '''
  update_events 方法处理与车辆状态相关的事件，例如加速器或制动器的状态变化。
  '''
  def update_events(self, CS: car.CarState) -> car.CarState:
    self.events.clear()

    self.events.add_from_msg(CS.events)

    # Disable on rising edge of accelerator or brake. Also disable on brake when speed > 0
    #if (CS.gasPressed and not self.CS_prev.gasPressed and self.disengage_on_accelerator) or \
    #  (CS.brakePressed and (not self.CS_prev.brakePressed or not CS.standstill)) or \
    #  (CS.regenBraking and (not self.CS_prev.regenBraking or not CS.standstill)):
    #  self.events.add(EventName.pedalPressed)

    self.speed_controller.inject_events(CS, self.events)
    CS.events = self.events.to_msg()

  '''
  state_publish 方法发布汽车状态和参数信息，供其他模块使用。
  '''
  def state_publish(self, CS: car.CarState):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

  '''
  controls_update 方法负责将控制命令发送到车辆，并确保初始化完成。
  详细解读
  初始化CarInterface:

  在 controls_update 方法的最开始，如果 initialized_prev 还未被设置（即车辆控制还未初始化），就会调用 self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan']) 来初始化车辆接口。这一步可能包括配置CAN总线、安全配置等。

  初始化完成后，会通过 self.params.put_bool_nonblocking("ControlsReady", True) 将 ControlsReady 参数设为 True，告诉 pandad 切换到车辆安全模式。

  生成并发送CAN消息:

  当 carControl（车辆控制消息）模块都存活 (self.sm.all_alive(['carControl'])) 时，开始处理车辆控制。

  调用 self.CI.apply(CC, now_nanos) 生成车辆控制命令，其中 CC 是控制命令，now_nanos 是当前的时间戳。该方法会返回两个值：last_actuators_output 表示执行器的最新输出状态，can_sends 则是要发送的CAN消息列表。

  接下来，调用 self.speed_controller.spam_message(CS, can_sends) 方法，这个方法可能用于向CAN消息列表添加更多消息，或进一步修改这些消息。

  最后，使用 self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid)) 将CAN消息发送出去。这里的 can_list_to_can_capnp 方法将 can_sends 转换为 capnp 消息格式，以便通过 sendcan 通道发布。
  '''
  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, self.can_sock, self.pm.sock['sendcan'])
      # signal pandad to switch to car safety mode
      self.params.put_bool_nonblocking("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      # CC下面是使用汽车接口将计算结果转换为制造商特定的 CAN 消息的代码核心，CI其中包含有关汽车品牌/型号的信息：
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos)
      self.speed_controller.spam_message(CS, can_sends)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  '''
  接收CAN数据：state_update() 方法从CAN总线上接收数据，并更新车辆状态 (CarState)。

  处理事件：update_events() 方法根据接收到的数据更新事件 (Events)。

  发布状态：state_publish() 方法将车辆状态 (CarState) 和车辆参数 (CarParams) 发布给其他模块或组件。

  控制更新：如果控制已经初始化且车辆不是被动模式（passive），则执行controls_update()方法，通过CAN总线发送控制信号。

  这个循环会持续运行，不断接收、处理和发送数据，确保车辆控制系统实时响应CAN总线上的消息。
  '''
  def step(self):
    CS = self.state_update()

    self.update_events(CS)

    self.state_publish(CS)

    initialized = (not any(e.name == EventName.controlsInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'])

    self.initialized_prev = initialized
    self.CS_prev = CS.as_reader()

  '''
  card_thread 是主循环，它不断调用 step 方法更新状态、处理事件、发布状态和控制命令。
  '''
  def card_thread(self):
    while True:
      self.step()
      self.rk.monitor_time()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
