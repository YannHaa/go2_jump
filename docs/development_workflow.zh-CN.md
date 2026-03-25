# 开发流程

[English](development_workflow.md)

## 文档目的

本文说明仓库的结构组织方式，以及如何在不破坏可复现性的前提下继续开发。

这个仓库的核心原则很简单：

- 项目专有代码放在根仓库
- 上游依赖用固定 submodule 管理
- 对上游的本地修改用显式 patch 保存

## 仓库管理约定

### 根仓库负责什么

根仓库跟踪以下内容：

- 自定义 ROS 2 包
- 构建和运行脚本
- 文档
- 本地 patch 文件

### 上游 submodule 负责什么

下面两个目录作为 submodule 管理：

- `src/unitree_ros2`
- `src/unitree_mujoco`

这样做的好处是可以保留上游完整历史，也便于在可控流程下升级依赖。

### 本地 patch 约定

当前 `unitree_mujoco` 的兼容性修改保存在：

- [`patches/unitree_mujoco/0001-go2-sim-compat.patch`](../patches/unitree_mujoco/0001-go2-sim-compat.patch)

不要把重要修改只留在 submodule 的工作区里。如果某个修改是项目运行所必需的，
就应该同步保存到 `patches/`。

## 全新克隆后的初始化流程

```bash
cd /home/hayan/go2_jump_ws
git submodule update --init --recursive
./scripts/bootstrap_workspace_repo.sh
./scripts/bootstrap_third_party.sh
./scripts/docker_build_image.sh
./scripts/docker_build_workspace.sh
```

## 日常开发循环

### 代码改完后重新构建

```bash
./scripts/docker_build_workspace.sh
```

### 挑最窄的验证命令先跑

根据你改动的范围，优先选择最聚焦的验证：

- 只看 DDS bridge 是否正常

```bash
./scripts/docker_verify_rt_lowcmd.sh
```

- 看完整前跳闭环是否正常

```bash
./scripts/docker_run_single_jump_trial.sh 0.25
```

### 看报告时重点关注什么

优先检查：

- `airborne_forward_progress_m`
- `final_forward_displacement_m`
- `max_abs_pitch_deg`
- `max_joint_tracking_error_rad`

### 调整或新增命名参数档位

当前命名参数档位定义在：

- `scripts/jump_profiles.sh`

如果你要新增一个 profile，建议遵循下面三条：

1. 只在环境变量和 launch 覆盖层做改动，不直接改基础 YAML 默认值
2. 在 `README.md` 和 `docs/calibration_workflow.md` 里补上用途说明
3. 至少跑一次单次试验，通过后再把它当成可复用档位

## 更新 submodule 的流程

如果需要有意升级 `unitree_ros2` 或 `unitree_mujoco`：

1. 把 submodule 移到目标上游提交
2. 如果需要，本地 patch 重新应用
3. 重新构建工作区
4. 重新跑 DDS 验证
5. 至少再跑一次完整前跳试验

不要在更新 submodule 后跳过验证。bridge、消息定义和运行时库之间耦合很强。

## 构建与运行时说明

### 第三方缓存

可选的缓存依赖保存在 `third_party/`。构建脚本会优先复用本地缓存，如果没有，
再从官方上游下载。

### MuJoCo 运行时

在没有图形桌面的机器上，`docker_run_go2_mujoco.sh` 会自动拉起 `Xvfb`。这样
既能保持 MuJoCo 的渲染循环正常工作，又能在无桌面环境里稳定运行仿真。

### CycloneDDS 运行时

如果手动运行 Unitree SDK2 的二进制程序，要保证 `LD_LIBRARY_PATH` 中
`/opt/unitree_robotics/lib` 排在工作区 CycloneDDS 库之前。这样可以避免混用
不同 DDS 运行时导致 MuJoCo bridge 不稳定。

## 建议的提交习惯

- 基础设施改动和控制参数调优，尽量分开提交
- 只要改了默认参数，就在提交信息里写清楚原因
- 行为发生变化时，在提交信息或 PR 描述里注明验证命令
- 只有在工作区可构建，并且至少一条目标验证通过后再推送
