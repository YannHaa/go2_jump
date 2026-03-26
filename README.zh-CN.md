# Go2 Jump Workspace 中文文档

[English](README.md)

## 项目定位

这个仓库是一个面向教学和实验迭代的 Go2 前跳项目。项目目标不是直接做一套可上机器人实机的完整最优控制器，而是先搭建一条清晰、可运行、可验证、可调参的最小闭环链路：

- 使用 `unitree_mujoco` 作为主仿真平台
- 使用 `unitree_ros2` 打通 `LowCmd` 和 `LowState`
- 在 sport mode 之下做低层关节控制

当前版本聚焦于平地、短距离、左右对称前跳。

## 你能从这个项目学到什么

- 如何组织一个基于 ROS 2 Humble 的 Go2 仿真项目
- 如何在 MuJoCo 中验证低层 `LowCmd` 是否真的生效
- 如何把“目标距离”映射到一个参数化的跳跃动作
- 如何用试验报告和扫参脚本逐步优化跳跃效果
- 如何把一个实验项目整理成可复现、可维护的工程仓库

## 建议阅读顺序

如果你是第一次接触这个项目，建议按下面顺序阅读和操作：

1. 先读本文件，完成第一次构建和第一次单次试验。
2. 再读 [控制链路说明](docs/control_stack.zh-CN.md)，理解运行时数据怎么流动。
3. 再读 [算法说明](algorithm.md)，理解 planner、controller、WBC 各自在算什么。
4. 接着读 [调参流程](docs/calibration_workflow.zh-CN.md)，理解怎么读报告、怎么做 sweep。
5. 如果准备改代码或改依赖，再读 [开发流程](docs/development_workflow.zh-CN.md)。

## 项目范围

### 当前已经包含

- Docker 化的 ROS 2 Humble 开发环境
- Go2 的 Unitree MuJoCo 仿真
- `LowCmd` / `LowState` 低层通信链路
- 参数化的前跳控制器
- 单次试验、日志归档、试验报告
- 起跳速度和空中前移能力的基础调参脚本

### 当前不包含

- sport mode 的 `frontJump`
- 直接用于实机的控制安全机制
- 接触丰富的最优控制
- 完整的状态估计与感知系统

## 已验证环境基线

当前仓库按照以下基线完成过验证：

- Ubuntu 22.04
- Docker 中运行 ROS 2 Humble
- `unitree_ros2` 与 `unitree_mujoco` 作为 submodule 管理
- 低层话题链路：
  `rt/lowcmd`、`rt/lowstate`、`/lowcmd`、`/lowstate`、`/sportmodestate`

项目采用 Docker 的目的很明确：保证 ROS 2、MuJoCo 和 Unitree SDK 的运行环境可复现。

## 目录说明

- `src/unitree_ros2`
  官方 Unitree ROS 2 仓库，作为 submodule 管理。
- `src/unitree_mujoco`
  官方 Unitree MuJoCo 仓库，作为 submodule 管理。
- `src/go2_jump_planner`
  负责跳跃参数生成和目标发布。
- `src/go2_jump_controller`
  负责低层控制、相位机和试验报告。
- `src/go2_jump_bringup`
  负责 launch 文件和公共参数。
- `scripts/`
  负责构建、启动、验证、扫参。
- `tools/`
  负责直接 DDS 验证工具。
- `patches/`
  保存对上游 submodule 的本地补丁。
- `docs/`
  保存架构、调参、开发相关文档。
- `algorithm.md`
  说明当前前跳算法的 planner、controller 和 WBC 分工。

## 快速开始

### 1. 克隆仓库

```bash
git clone --recurse-submodules https://github.com/matrix050628/go2_jump.git /home/hayan/go2_jump_ws
cd /home/hayan/go2_jump_ws
```

### 2. 初始化工作区

```bash
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
```

其中：

- `bootstrap_workspace_repo.sh` 会初始化 submodule，并自动应用
  `unitree_mujoco` 的兼容补丁
- `bootstrap_third_party.sh` 会准备第三方依赖缓存

### 3. 构建 Docker 镜像

```bash
./scripts/docker_build_image.sh
```

### 4. 构建工作区

```bash
./scripts/docker_build_workspace.sh
```

这一步会完成以下主要构建任务：

- `unitree_ros2` 相关依赖包
- `go2_jump_planner`
- `go2_jump_controller`
- `go2_jump_bringup`
- `tools/verify_rt_lowcmd`
- `unitree_mujoco`

### 5. 启动 MuJoCo 仿真

```bash
./scripts/docker_run_go2_mujoco.sh
```

如果宿主机没有图形桌面，会自动启动 `Xvfb`，保证 MuJoCo 的渲染循环和 DDS bridge 能稳定运行。

### 6. 启动跳跃控制栈

另开一个终端：

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_launch_jump_stack.sh 0.25
```

### 7. 直接运行一次完整试验

如果你只想快速确认项目能不能跑通，推荐直接用单次试验脚本：

```bash
cd /home/hayan/go2_jump_ws
./scripts/docker_run_single_jump_trial.sh 0.25
```

这个脚本会自动完成：

- 启动一个新的仿真实例
- 等待 `/lowstate` 稳定
- 启动 planner 和 controller
- 等待新的试验报告生成
- 归档日志、报告和试验上下文
- 关闭运行时容器

## 两种标准验证方式

### 方式一：直接验证低层 DDS

```bash
./scripts/docker_verify_rt_lowcmd.sh
```

这个脚本适合回答一个非常具体的问题：

“现在的 `LowCmd` 到底有没有真正驱动仿真中的关节？”

它会绕开 ROS 2 控制器，直接对 `rt/lowcmd` / `rt/lowstate` 做验证。

### 方式二：验证完整前跳闭环

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

这个脚本适合验证完整链路是否打通：

- MuJoCo 仿真
- `/lowstate`
- planner
- controller
- `/lowcmd`
- 报告生成

## 如何读试验报告

如果你只看一个指标，很容易误判“跳得更好了”。建议至少同时看下面四项：

- `final_forward_displacement_m`
  最终稳定后的前向位移。
- `landing_forward_displacement_m`
  落地检测时的前向位移。
- `airborne_forward_progress_m`
  从起跳到落地之间的空中前向位移。
- `post_landing_forward_gain_m`
  落地之后又向前补出来的位移。

对于这个项目，`airborne_forward_progress_m` 更能反映“是不是真的跳出去了”，而 `final_forward_displacement_m` 更像“最后停在了哪里”。

最近一轮控制器重构后，还有两个时间字段很值得关注：

- `push_extension_after_plan_s`
  名义上的 push 阶段结束后，到真正检测到起跳之间又额外持续了多久。
- `flight_extension_after_plan_s`
  相比抛体近似给出的 flight 时间，真正检测到落地又多飞了多久。

## 调参入口

### 0. 切换命名参数档位

当前仓库支持用 `GO2_JUMP_PROFILE` 选择一组命名参数档位。

例如，显式使用当前默认风格的保守档：

```bash
GO2_JUMP_PROFILE=conservative_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

切到更激进的空中探索档：

```bash
GO2_JUMP_PROFILE=aggressive_airborne \
./scripts/docker_run_single_jump_trial.sh 0.25
```

当前内置档位包括：

- `config_default`
- `conservative_airborne`
- `aggressive_airborne`

### 1. 标定起跳速度

```bash
./scripts/sweep_takeoff_speed_scale.sh 0.20,0.25,0.30 1.00,1.03,1.06 1
```

这个 sweep 解决的是“给定目标距离，起跳速度应该放大多少”。

如果你已经切到某个命名档位，想在保留该档位姿态/力矩特征的同时重新标定
`takeoff_speed_scale`，可以直接使用：

```bash
./scripts/sweep_profile_takeoff_speed_scale.sh aggressive_airborne 0.25 0.94,0.97,1.00,1.03 1
```

这个脚本特别适合处理“空中前移明显增强，但最终位移超了”的情况。

### 2. 优化空中前移能力

```bash
./scripts/sweep_airborne_push_pitch.sh 0.25 0.88,0.92,0.96 1.08,1.12,1.16 -8.0,-5.0,-2.0 -2.0,0.0 1
```

这个 sweep 解决的是“在目标距离固定时，如何让真正的空中前移更明显”。它会比较：

- `airborne_forward_progress_m`
- `airborne_completion_ratio`
- 最终位移误差

## 当前参考参数

### 起跳速度曲线

当前距离到 `takeoff_speed_scale` 的标定结果是：

- `0.20 m -> 1.09`
- `0.25 m -> 1.06`
- `0.30 m -> 1.06`

### 当前默认参数

当前默认值已经不只是单纯的 push / flight 参数组，而是一套完整的前跳折中配置：

- `takeoff_angle_deg = 35.0`
- `push_front_tau_scale = 0.96`
- `push_rear_tau_scale = 1.12`
- `push_pitch_target_deg = -5.0`
- `flight_pitch_target_deg = -2.0`
- `landing_support_blend = 0.40`
- `landing_touchdown_reference_blend = 0.80`

之所以把这组参数升成默认，是因为它比旧默认值更少依赖落地后的“补出来的位移”，
同时又没有退回到完全不稳定的 full touchdown-hold 路线。

在 2026 年 3 月 25 日最新几次默认 `0.25 m` 验证里，这组默认参数的结果大致落在：

- `final_forward_displacement_m ~= 0.33-0.35`
- `airborne_forward_progress_m ~= 0.11`
- `post_landing_forward_gain_m ~= 0.23`
- `final_pitch_deg ~= -28`

它仍然不是“主要靠腾空完成”的干净前跳，但作为当前工程默认值，比旧默认值更接近
这个目标。

### 当前激进探索参数

在 2026 年 3 月 25 日的 focused airborne sweep 中，空中前移最强的一组参数是：

- `push_front_tau_scale = 0.96`
- `push_rear_tau_scale = 1.12`
- `push_pitch_target_deg = -2.0`
- `flight_pitch_target_deg = 0.0`

这组参数可以把 `airborne_forward_progress_m` 提高到约 `0.0655 m`，但最终位移会超到约 `0.306 m`，因此更适合作为探索模式，而不是默认设置。

### 当前已验证的一组激进档重标点

2026 年 3 月 25 日做过一次单点验证，保留激进空中探索档，同时把
`takeoff_speed_scale` 降到 `1.00`，得到：

- `final_forward_displacement_m ~= 0.2593`
- `airborne_forward_progress_m ~= 0.0527`

这说明“保留激进档姿态整形，再单独把起跳速度往下拉”这条路线是有效的。它还不
足以直接升级成默认值，但已经是下一轮重复试验的好起点。

### 2026 年 3 月 25 日控制器阶段性结论

最新一轮控制器迭代加入了：

- 事件驱动的 `push` / `flight` 相位
- 可调的 landing / recovery 阻尼
- `crouch` / `push` 的前向几何偏置
- 一个带额外诊断指标的 support-hold 落地链路
- 可连续调节的 touchdown reference blend

当前得到的工程结论是：

- 控制器现在会把落地后的前移拆成 `support_hold_forward_gain_m` 和
  `release_to_complete_forward_gain_m`
- 部分使用 touchdown reference 之后，`0.25 m` 目标试验里的
  `post_landing_forward_gain_m` 已经可以压到 `0.20-0.24 m` 区间
- 完全偏向 touchdown 的实验参数可以把最终位移压得更接近目标，但落地后机身仍然会
  过于前俯
- 因此当前版本比之前更接近“真正前跳”，但还没有达到“绝大部分位移都靠腾空完成”
  的项目目标

## 当前局限

- `foot_force_est` 仍然为零，因此落地检测仍是启发式的
- 真正的空中前移仍明显小于最终稳定位置
- 即便是当前最好的默认值，在 `0.25 m` 目标下仍然有大约 `0.22 m` 的落地后前移
- 当前最接近目标距离的 landing-support 组合，在 touchdown 后机身姿态仍然偏前俯
- 当前版本是仿真优先版本，不应直接视为实机控制器

## 文档索引

- [English README](README.md)
- [Control Stack](docs/control_stack.md)
- [控制链路说明](docs/control_stack.zh-CN.md)
- [Calibration Workflow](docs/calibration_workflow.md)
- [调参流程](docs/calibration_workflow.zh-CN.md)
- [Development Workflow](docs/development_workflow.md)
- [开发流程](docs/development_workflow.zh-CN.md)
