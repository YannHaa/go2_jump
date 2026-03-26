# Go2 Jump MPC 工作区

[English](README.md)

这个仓库已经彻底切到一条新的主线，目标非常明确：

围绕 `Go2 + Unitree MuJoCo + LowCmd/LowState`，构建一个真正面向研究和论文产出的
高质量前跳控制器，新主线选择 `MuJoCo-native whole-body MPC`。

旧的模板式前跳控制实验已经从项目主结构中移除，不再继续作为主线维护。新的目录
组织从一开始就面向研究型开发、系统辨识、对比实验和论文叙事。

## 新主线方向

- 仿真平台：`unitree_mujoco`
- 通信链路：`unitree_ros2` + `LowCmd` / `LowState`
- 任务接口：按目标距离条件化的 jump task
- 控制主线：`MuJoCo-native whole-body MPC`
- 落地策略：接触感知 + reactive landing
- 核心指标：主要靠腾空完成前移，并且落地干净稳定

## 当前目录结构

- `src/unitree_ros2`
  上游 Unitree ROS 2 依赖。
- `src/unitree_mujoco`
  上游 Unitree MuJoCo 依赖。
- `src/go2_jump_msgs`
  jump task 相关的 ROS 2 接口。
- `src/go2_jump_core`
  负责目标距离到 jump task 的生成，以及参考量采样。
- `src/go2_jump_mpc`
  新的 whole-body MPC 主线包。
- `src/go2_jump_bringup`
  新主线的 launch 和共享参数。
- `docs/`
  架构设计和研究路线文档。
- `scripts/`
  新主线的 Docker 构建和运行脚本。
- `patches/`
  针对上游依赖的可复现补丁。

## 当前已经完成的事情

这次重构已经不只是“空骨架”，而是把最小闭环真的打通了：

- `unitree_ros2`、`go2_jump_*`、`unitree_mujoco` 可以统一通过 Docker 构建
- 建立了任务层 jump specification
- 建立了带接触感知阶段切换保护逻辑的 preview controller 骨架
- 打通了 `LowCmd` 下发链路
- 增加了 `JumpControllerState` 调试话题
- 为上游依赖保留了可复现 patch 机制

当前主线已经验证过：

- `unitree_mujoco` 能发布 `/lowstate`
- `enable_lowcmd_output=true` 时，控制器能以约 `200 Hz` 发布 `/lowcmd`
- `/go2_jump/controller_state` 可用于观察 phase、contact 和 backend 状态

需要明确的是：最终的 MuJoCo-native 优化求解器还没有接进来。现在的 backend 仍然是
`reference_preview`，它的作用是先把外层闭环、诊断接口和求解器边界稳定下来。

## 快速开始

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

启动新的 MPC 主线：

```bash
./scripts/docker_launch_jump_mpc.sh 0.25
```

运行最小闭环自检：

```bash
./scripts/docker_smoke_test_stack.sh 0.25
GO2_JUMP_ENABLE_LOWCMD_OUTPUT=true ./scripts/docker_smoke_test_stack.sh 0.20
```

## 建议阅读顺序

1. 先读本文件，了解为什么这次重构要直接切主线。
2. 再读 [算法说明](algorithm.zh-CN.md)，先把 planner、controller、backend 三层理顺。
3. 再读 [架构说明](docs/architecture.zh-CN.md)，理解新的包划分。
4. 再读 [研究路线](docs/research_program.zh-CN.md)，理解 MPC 主线后续怎么推进。

## 当前默认假设

- 仍然以 `unitree_mujoco` 和 `unitree_ros2` 为基础依赖。
- MuJoCo bridge 的本地兼容修改会通过 `patches/` 目录管理。
- Docker 继续作为主开发环境，原因很简单：ROS 2、MuJoCo 和 Unitree 依赖需要
  可复现。
