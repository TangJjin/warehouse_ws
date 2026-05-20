# 各功能包作用说明

## 1. 文档目的

本文档用于对当前工作区中的几个核心功能包做一个初步职责梳理，方便后续：

- 分包协作
- GitHub 仓库管理
- 控制程序、机载端、地面端职责划分
- 后续补充更细的代码级说明

当前工作区中涉及的功能包有：

- `drone_bringup`
- `drone_control`
- `drone_localization`
- `drone_msgs`
- `drone_perception`
- `drone_qt`
- `drone_qt_2`

---

## 2. 功能包整体关系概览

### 2.1 启动与系统组织
- `drone_bringup`

### 2.2 控制与任务执行
- `drone_control`

### 2.3 定位与感知
- `drone_localization`
- `drone_perception`

### 2.4 通信接口与界面
- `drone_msgs`
- `drone_qt`
- `drone_qt_2`

---

## 3. 各功能包作用说明


## 3.1 `drone_bringup`

这个包放：

- launch 文件
- 启动参数
- 多节点编排
- 不同模块的统一启动入口

有：
- 机载端某些链路的统一启动入口
- offboard 启动流程
- 其他系统级启动编排任务


## 3.1 `drone_control`

drone_control 是控制程序主逻辑包。



## 3.2 `drone_localization`

drone_localization 主要负责定位链路相关功能。



## 3.3 `drone_msgs`

drone_msgs 主要负责全系统公共消息定义。

这是整个多包系统里的公共接口层。



## 3.4 `drone_perception`

drone_perception 是感知识别包，主要负责“看到了什么、识别到了什么”。



## 3.5 `drone_qt`

作用概述

drone_qt 主要负责Qt 地面端界面程序。



## 3.6 `drone_qt_2`

drone_qt_2 主要负责Qt 机载端程序。

---

## 4. 当前职责分层

启动层

- drone_bringup

控制层

- drone_control

定位层

- drone_localization

感知层

- drone_perception

接口层

- drone_msgs

地面站界面层

- drone_qt

机载端中转层

- drone_qt_2
