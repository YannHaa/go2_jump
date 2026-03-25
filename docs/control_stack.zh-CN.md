# 控制链路说明

[English](control_stack.md)

## 文档目的

本文说明 Go2 前跳控制栈在仓库中的组织方式、运行时数据如何流动，以及在排查
问题或扩展控制器时应该优先使用哪些验证手段。

这套栈的目标刻意收得比较窄：它服务于低层前跳实验，不试图覆盖完整的四足行走
框架。

## 各包职责

- `src/unitree_mujoco`
  负责运行 Go2 仿真，并发布控制器需要的 bridge 话题。
- `src/unitree_ros2`
  提供本工作区使用的 ROS 2 消息定义和支持包。
- `src/go2_jump_planner`
  把目标前跳距离转换成跳跃计划摘要。
- `src/go2_jump_controller`
  实现低层相位机，并发布 `LowCmd`。
- `src/go2_jump_bringup`
  存放共享参数和 launch 描述。
- `tools/verify_rt_lowcmd.cpp`
  不经过 ROS 2 控制器逻辑，直接验证 DDS 控制链路是否能驱动仿真。

## 运行时接口

### 仿真器输出

MuJoCo bridge 会发布：

- `rt/lowstate`
- `/lowstate`
- `/sportmodestate`
- 工作区其他模块需要的兼容别名话题

### 控制器输入

控制器订阅：

- `/lowstate`
- `/sportmodestate`
- `/jump_target_distance`

### 控制器输出

控制器发布：

- `/lowcmd`

### 试验产物

每次运行结束后，控制器会写出：

- `reports/jump_metrics/latest_report.txt`
- `reports/jump_metrics/latest_trial_context.txt`

单次试验脚本还会把时间戳版本的报告和上下文一起归档到同一目录。上下文文件会
记录本次试验使用的 profile 以及最终生效的 launch 覆盖参数。

## 端到端数据流

1. `go2_jump_planner` 从 launch 参数获得目标距离。
2. planner 计算跳跃计划，并发布 `/jump_target_distance`。
3. `unitree_mujoco` 发布 `/lowstate` 和 bridge 状态话题。
4. `go2_jump_controller` 等待稳定的起始姿态。
5. 控制器进入前跳相位机，并发布 `/lowcmd`。
6. 仿真器执行这些低层指令。
7. 控制器在完整动作过程中持续记录指标。
8. 恢复阶段结束后写出试验报告。

## 控制器相位机

当前控制器使用固定的阶段序列：

- `stand`
- `crouch`
- `push`
- `flight`
- `landing`
- `recovery`

控制器采用“参数化相位动作”而不是“接触优化控制”。这样做的好处是每个阶段的
行为更容易理解，单个参数的影响也更容易通过 sweep 分析出来。

## 运行时参数配置层级

当前生效的控制参数按下面顺序解析：

1. `src/go2_jump_bringup/config/jump_params.yaml` 中的基础默认值
2. `scripts/jump_profiles.sh` 中的命名参数档位
3. `GO2_JUMP_*` 环境变量覆盖
4. `scripts/jump_launch_args.sh` 生成的最终 ROS 2 launch 参数

这样设计的目的，是在不改动仓库默认参数的前提下，让实验参数能够通过一条命令
稳定复现。

## 主要调参旋钮作用位置

- `takeoff_speed_scale`
  调整由抛体模型生成的规划起跳速度。
- `push_front_tau_scale` / `push_rear_tau_scale`
  调整前后腿在蹬地阶段的力矩分配。
- `push_pitch_target_deg`
  调整蹬地阶段机身俯仰目标。
- `flight_pitch_target_deg`
  调整腾空阶段机身俯仰目标。
- `landing_*`
  影响落地姿态以及落地后的恢复行为。

## 建议的验证层级

建议按下面顺序使用验证工具。

### 第一层：直接验证 DDS

```bash
./scripts/docker_verify_rt_lowcmd.sh
```

适合回答的问题是：

“仿真器现在到底会不会响应低层指令？”

这个测试会把 DDS bridge 和 ROS 2 控制器逻辑拆开看。

### 第二层：验证完整前跳闭环

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

适合回答的问题是：

“planner、controller、simulator 这一整条链路能不能完整跑完并产出报告？”

## 调参时优先关注的指标

报告里有很多字段，但下面这些最值得优先看：

- `final_forward_displacement_m`
  最终稳定后的结果。
- `landing_forward_displacement_m`
  落地检测时的前向位置。
- `airborne_forward_progress_m`
  真正发生在腾空阶段的前向进展。
- `post_landing_forward_gain_m`
  落地检测之后又补出来的前向位移。
- `max_abs_pitch_deg`
  用来识别俯仰整形是否过激。

实践里建议遵循三条判断规则：

- 先优化 `airborne_forward_progress_m`
- 把 `final_forward_displacement_m` 当作约束而不是唯一目标
- 用 `post_landing_forward_gain_m` 识别“落地后补出来的假增益”

## 当前已知限制

- 当前落地检测仍主要依赖高度和速度阈值
- 现有 MuJoCo bridge 路径里的 `foot_force_est` 仍为零
- 当前控制器更偏向短距离、可重复的小跳，而不是最大腾空距离
- 落地后的恢复动作仍会对最终前向位移贡献可观比例
