环境（Environment）
物理环境（Physical Environment）：包括道路条件和其他车辆等外部因素。
用户干预（human intervention）：用户可以通过改变速度限制或手动接管车辆进行干预。
汽车（Car）
Comma Two设备：主要负责控制汽车的单元。
CAN总线（Controller Area Network）：用于与汽车通信的特定格式的消息传递。
Panda：硬件适配器，用于连接CAN到Comma Two。
Comma Two传感器（Sensors）
相机：监控前方道路。
GPS：位置确定。
红外线：检测驾驶员分心。
软件组件（Software Components）
基础设施（Infrastructure）：安装/更新脚本、APK（Android应用文件）、phonelibs（导入的Android库）。
通信依赖项（Communication Dependencies）：ZMQ/msgg（发布订阅消息传递）、Cereal（内部消息和日志）、Cap'n Proto（消息类型）。
自驾核心（Self-drive Core）：boardd（与Panda通过USB通信）、camerad（读取道路摄像头图像）、radard（处理雷达消息）、modeld（应用机器学习模型）、plannerd（决定车辆应去的方向）、controlsd（创建特定车型的驱动指令）、opendbc（编码CAN消息）。


想象一下，我们正在为自动驾驶汽车设计非常简单的软件。为了让它对环境（其他汽车、车道分离）做出反应，我们需要能够读取传感器数据、根据这些数据采取行动并更新执行器，例如方向盘或油门。

从这个描述中，我们可以得出核心系统功能的一个非常简单的抽象实现：

while (true) { <br>
  read sensor data <br>
  compute adjustments using machine-learning model <br>
  apply adjustments to actuators <br>
}
虽然这看起来过于简单，但本质上这就是 openpilot 内部的工作方式。实际上，openpilot 使用 300 多个 Python 文件（分为各种子模块、依赖项和多个硬件组件）来运行该系统。

这篇文章旨在概述从传感器输入到执行器输出的路径上的主要架构组件和流程、它们的职责、组件内或组件之间应用的设计模式以及在该架构中遇到的权衡。

<div align="center" style="text-align: center;">

<h1>openpilot</h1>

<p>
  <b>openpilot is an operating system for robotics.</b>
  <br>
  Currently, it upgrades the driver assistance system in 275+ supported cars.
</p>

<h3>
  <a href="https://docs.comma.ai">Docs</a>
  <span> · </span>
  <a href="https://docs.comma.ai/contributing/roadmap/">Roadmap</a>
  <span> · </span>
  <a href="https://github.com/commaai/openpilot/blob/master/docs/CONTRIBUTING.md">Contribute</a>
  <span> · </span>
  <a href="https://discord.comma.ai">Community</a>
  <span> · </span>
  <a href="https://comma.ai/shop">Try it on a comma 3X</a>
</h3>

Quick start: `bash <(curl -fsSL openpilot.comma.ai)`

![openpilot tests](https://github.com/commaai/openpilot/actions/workflows/selfdrive_tests.yaml/badge.svg)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![X Follow](https://img.shields.io/twitter/follow/comma_ai)](https://x.com/comma_ai)
[![Discord](https://img.shields.io/discord/469524606043160576)](https://discord.comma.ai)

</div>

[![openpilot on the comma 3X](https://github.com/commaai/openpilot/assets/8762862/f09e6d29-db2d-4179-80c2-51e8d92bdb5c)](https://comma.ai/shop/comma-3x)

---
<table>
  <tr>
    <td><a href="https://youtu.be/NmBfgOanCyk" title="Video By Greer Viau"><img src="https://github.com/commaai/openpilot/assets/8762862/2f7112ae-f748-4f39-b617-fabd689c3772"></a></td>
    <td><a href="https://youtu.be/VHKyqZ7t8Gw" title="Video By Logan LeGrand"><img src="https://github.com/commaai/openpilot/assets/8762862/92351544-2833-40d7-9e0b-7ef7ae37ec4c"></a></td>
    <td><a href="https://youtu.be/SUIZYzxtMQs" title="A drive to Taco Bell"><img src="https://github.com/commaai/openpilot/assets/8762862/05ceefc5-2628-439c-a9b2-89ce77dc6f63"></a></td>
  </tr>
</table>

---

[![](https://i.imgur.com/TMtVMV8.png)](#)


openpilot by crwusiz branch
------
 * version 0.9.8 [ only comma3 or comma3x support ]
 * event message, qt ui kor translate ( language select )
 * brake, gps, wifi icon and wheel, N direction icon rotate
 * add community toggle / function / simple ui
 * autohold, turnsignal, tpms, roadlimitspeed display ( neokii )
 * select manufacturer and car
 * If you have any questions, please send DM to [crwusiz](https://discord.com/channels/@me) from discord.
 * It is open source and inherits MIT license.
 * By installing this software you accept all responsibility for anything that might occur while you use it.
 * All contributors to this fork are not liable. <b>Use at your own risk.</b>
 * if you like this Branch <b> [Donate](https://paypal.me/crwusiz) </b> for me

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/J3J4RXA92)

## Harness Cable and Car Year Compatible Chart

 * cable order -> https://smartstore.naver.com/jmtechn

[![](https://i.imgur.com/v3cvfeF.png)](#)
[![](https://i.imgur.com/uIH0dQA.png)](#)
[![](https://i.imgur.com/6rxOQbQ.jpg)](#)

## Source Code Reference
 * https://github.com/commaai/openpilot
 * https://github.com/xx979xx/openpilot
 * https://github.com/xps-genesis/openpilot
 * https://github.com/kegman/openpilot
 * https://github.com/dragonpilot-community/dragonpilot
 * https://github.com/wirelessnet2/openpilot
 * https://github.com/sshane/openpilot
 * https://github.com/arne182/ArnePilot
 * https://github.com/neokii/glidepilot
 * https://github.com/openpilotusers
 * https://github.com/Circuit-Pro/openpilot
 * https://github.com/sunnyhaibin/sunnypilot
 * https://github.com/ajouatom/carrotpilot


To start using openpilot in a car
------

To use openpilot in a car, you need four things:
1. **Supported Device:** a comma 3/3X, available at [comma.ai/shop](https://comma.ai/shop/comma-3x).
2. **Software:** The setup procedure for the comma 3/3X allows users to enter a URL for custom software. Use the URL `openpilot.comma.ai` to install the release version.
3. **Supported Car:** Ensure that you have one of [the 275+ supported cars](docs/CARS.md).
4. **Car Harness:** You will also need a [car harness](https://comma.ai/shop/car-harness) to connect your comma 3/3X to your car.

We have detailed instructions for [how to install the harness and device in a car](https://comma.ai/setup). Note that it's possible to run openpilot on [other hardware](https://blog.comma.ai/self-driving-car-for-free/), although it's not plug-and-play.

To start developing openpilot
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot).

* Join the [community Discord](https://discord.comma.ai)
* Check out [the contributing docs](docs/CONTRIBUTING.md)
* Check out the [openpilot tools](tools/)
* Read about the [development workflow](docs/WORKFLOW.md)
* Code documentation lives at https://docs.comma.ai
* Information about running openpilot lives on the [community wiki](https://github.com/commaai/openpilot/wiki)

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs#open-positions) and offers lots of [bounties](docs/BOUNTIES.md) for external contributors.

Safety and Testing
----

* openpilot observes [ISO26262](https://en.wikipedia.org/wiki/ISO_26262) guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software-in-the-loop [tests](.github/workflows/selfdrive_tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software-in-the-loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware-in-the-loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware-in-the-loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road-facing cameras, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver-facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.
