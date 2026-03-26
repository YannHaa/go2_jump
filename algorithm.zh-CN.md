# 算法说明

[English](algorithm.md)

这个仓库的新主线按三层来组织：

`planner -> controller -> WBC/MPC backend -> LowCmd`

这份文档的目的不是写概念宣传，而是把三件事说清楚：

- planner 到底在算什么
- controller 到底在算什么
- WBC / MPC backend 到底在算什么

同时也明确当前仓库已经实现到了哪一步，哪些部分还只是主线预留。

## 1. Planner 在算什么

planner 负责把“目标前跳距离”翻译成一个可执行的跳跃任务描述。

当前 `go2_jump_core` 已经实现的内容包括：

- 根据目标距离和起跳角，计算目标起跳速度
- 将起跳速度分解为前向分量和竖直分量
- 估计下蹲、蹬地、腾空、落地、收敛五个阶段的名义时长
- 生成机身俯仰、机身高度偏移、目标速度等高层参考量

现在这部分还是“任务层规划”，不是完整的 centroidal 优化器。它的职责是稳定地输出：

`distance request -> jump task specification`

这样后面无论接：

- centroidal jump planner
- MuJoCo-native MPC
- 用于对比实验的其他基线

上层任务接口都不用推倒重来。

## 2. Controller 在算什么

controller 的职责是：

给定当前观测和当前 jump task，决定下一小段时间内机器人应该跟踪什么参考，以及当前时刻应该下发什么低层命令。

当前 `go2_jump_mpc` 已经实现了四件真正有价值的事：

- 接收 `JumpTask`，构造短时域 preview
- 把阶段级参考量转换成关节级参考姿态
- 生成控制周期上的 `LowCmd`
- 根据接触观测修正阶段切换，而不是完全依赖时间表

最后一点很关键。真正的跳跃控制不能只靠固定时间推进，因为实际接触序列经常会偏离名义计划：

- 可能比预计更早离地
- 可能比预计更早触地
- 可能出现部分触地、非对称触地

所以当前控制器已经有一个最小但实用的“接触感知阶段管理器”：

- 在 push 阶段，如果支撑接触提前消失，可以提前切到 flight
- 在 flight 阶段，如果接触提前恢复，可以提前切到 landing
- 在 landing 阶段，如果四足接触已经稳定且竖直速度足够小，可以提前切到 settle

为了避免假接触量把阶段切乱，当前控制器只有在观测流里真正看到有效接触信号后，
才会启用这些接触驱动的阶段切换逻辑。

这仍然不是最终的 whole-body optimizer，但它已经把项目从“纯时间脚本”推进到了“带观测闭环的控制器”。

## 3. WBC / MPC Backend 在算什么

backend 才是最终要解决的核心问题：

如何在满足动力学、接触、力矩限制、关节限制和任务目标的同时，计算当前控制时刻的低层输出？

这条主线最终希望落到一个 MuJoCo-native whole-body optimization backend。这个 backend 需要同时考虑：

- 起跳阶段的冲量和速度目标
- 腾空阶段的姿态保持
- 落地阶段的接触恢复和姿态吸收
- 关节位置、速度、力矩约束
- 触地后的快速稳定

目标中的 backend 结构可以概括成五步：

1. 在短时域内滚动展开动力学。
2. 对起跳速度、机身姿态、落地状态等任务目标建立代价。
3. 对力矩、关节速度、姿态偏移建立正则项和约束。
4. 结合接触信息对起跳和落地过程做接触一致性处理。
5. 只执行当前时刻的第一步控制，再进入下一次优化。

当前仓库里，这一层还没有换成最终的优化求解器。现在用的是一个 `reference_preview` 后端，用来先把外部 ROS2 接口、LowCmd 链路、接触观测和调试通道稳定下来。

## 当前已经实现到哪一步

已经实现：

- 距离条件化 jump task
- 阶段参考量 preview
- 接触感知阶段切换
- `LowCmd` 下发链路
- `JumpControllerState` 调试话题
- 可复现的 MuJoCo + ROS2 + smoke test 工作流

还没有实现：

- centroidal jump optimization
- torque-level WBC / QP
- MuJoCo-native shooting / SQP / MPC solver
- 更细致的落地接触重分配
- 基于辨识的模型参数校准

## 下一步最值得做什么

下一步最值得投入的，不是继续在关节空间里“抠动作参数”，而是把当前 preview backend 替换成真正显式建模以下对象的优化 backend：

- centroidal momentum
- contact schedule hypothesis
- joint / torque constraints
- touchdown cost
- post-touchdown recovery

只有走到这一步，项目才会真正从：

`按任务生成的动作脚本`

走向：

`按任务求解的优化控制器`
