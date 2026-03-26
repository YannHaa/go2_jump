# Go2 Jump MPC 工作区

[English](README.md)

这个仓库围绕一个明确的工程目标组织：

基于 `LowCmd / LowState` 接口，为 Unitree Go2 构建一个按目标距离条件化的前跳控制器，
主仿真平台使用 `unitree_mujoco`，主控制方向选择 MuJoCo-native whole-body MPC。

旧的模板式前跳实验已经不再作为项目基线。当前主线强调的是仿真可信度、接触感知控制
和可重复的 backend 迭代流程。

## 主线方向

- 仿真平台：`unitree_mujoco`
- 通信链路：`unitree_ros2`
- 任务接口：`JumpTask`
- 低层控制链路：`LowCmd`
- 主控制包：`go2_jump_mpc`
- 长期目标：`接触感知规划 -> MuJoCo-native MPC -> 低层执行`

## 目录结构

- `src/unitree_ros2`
  上游 Unitree ROS 2 依赖。
- `src/unitree_mujoco`
  上游 Unitree MuJoCo 依赖，叠加了本地兼容修正。
- `src/go2_jump_msgs`
  jump task 和控制器诊断消息。
- `src/go2_jump_core`
  目标距离到跳跃任务的构造，以及参考量采样。
- `src/go2_jump_mpc`
  接触估计、阶段管理、preview 控制和 MuJoCo-native MPC backend。
- `src/go2_jump_bringup`
  launch 文件和共享参数。
- `scripts/`
  Docker 构建、启动和自检脚本。
- `docs/`
  架构说明和研究路线文档。

## 当前进展

当前工作区已经有一个可运行的最小闭环，同时已经接入了一个实验中的原生 backend。

已经验证：

- headless `unitree_mujoco` 现在能够正常推进并发布非零 `/lowstate`
- `/lowstate` 已经包含非零关节状态、IMU 状态和持续增长的 tick
- `foot_force` / `foot_force_est` 不再是占位量；在低层控制运行时，它们会反映基于 MuJoCo 接触解算得到的支撑载荷
- `go2_jump_mpc` 已经走通真实低层控制链路；在当前 native backend 配置下，`/lowcmd` 实测通常在 `80-100 Hz` 区间
- `reference_preview` 仍然是当前最稳定的基线 backend
- `mujoco_native_mpc` 已经接入、可编译、可启动，并且走通了同一条低层控制链路
- 已经可以直接从 Docker 流程生成单次试跳报告，输出保存在 `reports/trials/`

当前限制：

- `mujoco_native_mpc` 还不能算高质量前跳控制器
- 它已经能比早期模板式方案生成更干净的腾空前移，但距离跟踪仍明显低于目标值，而且不同试次之间的一致性还需要继续打磨

也就是说，这个项目已经过了单纯的 bring-up 阶段，但现在仍然处在控制器迭代阶段，
还不是最终的 benchmark 版本。

## 快速开始

构建工作区：

```bash
cd /home/hayan/go2_jump_ws
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

启动仿真：

```bash
./scripts/docker_run_go2_mujoco.sh
```

使用稳定 preview backend 启动跳跃栈：

```bash
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_launch_jump_mpc.sh 0.25
```

使用 MuJoCo-native backend 启动跳跃栈：

```bash
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_launch_jump_mpc.sh 0.25
```

运行自检：

```bash
./scripts/docker_smoke_test_stack.sh 0.25
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_smoke_test_stack.sh 0.25
GO2_JUMP_SOLVER_BACKEND=mujoco_native_mpc \
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true \
./scripts/docker_smoke_test_stack.sh 0.25
```

运行一次带自动记录的单次试跳：

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

脚本会在 `reports/trials/<timestamp>_d<distance>_mujoco_native_mpc/` 下生成
`summary.json`、`stack.log`、`sim.log` 和 `recorder.log`。

## 建议阅读顺序

1. 先读本文件，建立对仓库入口和当前状态的整体认识。
2. 再读 [算法说明](algorithm.zh-CN.md)，理解 planner / controller / backend 三层分工。
3. 再读 [架构说明](docs/architecture.zh-CN.md)，理解包结构和依赖关系。
4. 最后读 [研究路线](docs/research_program.zh-CN.md)，了解当前控制器之后该往哪里推进。

## 实用说明

- Docker 是当前参考环境。
- 如果目标是验证接口链路和传输稳定性，优先使用 `reference_preview`。
- 如果目标是继续开发主控制器，使用 `mujoco_native_mpc`。
- 目前最有价值的调试话题是 `/lowstate`、`/lowcmd` 和 `/go2_jump/controller_state`。
- 如果目标是调控制器，优先使用 `./scripts/docker_run_single_jump_trial.sh <distance>`，因为它会把阶段序列、腾空位移和 lowcmd 频率一起记录下来。
