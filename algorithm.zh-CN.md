# 算法说明

[English](algorithm.md)

这个项目当前按三层控制栈组织：

`planner -> controller -> MuJoCo-native WBC/MPC backend -> LowCmd`

这份文档的目标是把三层分别在算什么、已经做到哪一步、还差什么，说清楚。

## 1. Planner 在算什么

planner 负责把“跳多远”翻译成“跳成什么事件”。

当前实现位置：`go2_jump_core`

输入：

- 目标前跳距离
- 起跳角
- 可选的起跳速度缩放
- 目标起跳俯仰角和落地俯仰角

输出：

- 基于抛体近似得到的目标起跳速度
- 前向和竖直方向的目标起跳速度分量
- 下蹲 / 蹬地 / 腾空 / 落地 / 收敛五个阶段的名义时长
- 阶段级参考量：
  - 目标前向速度
  - 目标竖直速度
  - 目标机身俯仰
  - 目标机身高度偏移
  - 收腿比例
  - 落地支撑系数

当前 planner 还没有做的事：

- 没有 centroidal 优化
- 没有显式搜索接触时序
- 没有地形感知
- 没有落地点优化

这一层为什么仍然重要：

它把任务接口稳定了下来。以后 backend 从 preview 控制换到 native MPC，
上层仍然可以维持同一个接口：

`distance request -> jump task specification`

## 2. Controller 在算什么

controller 负责把当前 jump task 和当前观测翻译成下一帧低层命令。

当前实现位置：`go2_jump_mpc`

### 2.1 观测处理

controller 当前读取：

- `/lowstate` 中的关节位置和关节速度
- `/lowstate` 中的 IMU 姿态和角速度
- `/sportmodestate` 中的机身位置和线速度
- `/lowstate.foot_force_est` 中的足端接触代理

当前接触链路已经不是简单的占位量，而是三段式：

1. `unitree_mujoco` 从 MuJoCo 的实际接触解算中重建支撑载荷。
2. `go2_jump_mpc_node` 对载荷做滤波、滞回和去抖。
3. controller 再从滤波后的载荷构造足端接触布尔量和接触数量。

这样做的原因很直接：

跳跃的阶段切换对接触信号非常敏感。接触量如果太弱、太抖或者语义不稳，
phase 机会被一帧一帧地打乱。

### 2.2 阶段管理

controller 不是纯时间驱动的。

它先从 `SampleJumpReference` 得到名义阶段，再根据当前接触状态做修正，例如：

- 提前离地
- 到了名义 flight 但还没真正离地时，继续延长 push
- 到了名义 landing 但还没真正触地时，继续维持 flight
- 提前触地
- 触地稳定且竖直速度足够小时，转入 settle

这部分的核心目标很明确：

让实际执行阶段尽可能跟随真实接触序列，而不是只跟随名义时钟。

### 2.3 当前时刻的命令生成

对当前控制周期，controller 会构造：

- 关节参考位姿 `q_ref`
- 近似为零的 `dq_ref`
- 随阶段变化的 PD 增益
- 随阶段变化的前馈力矩

这里的前馈项目前仍然是工程化的结构化项，不是完整刚体逆动力学。它的作用是
在 push 和 landing 阶段给低层控制增加足够的方向性，而不是让系统只做关节位置跟踪。

## 3. MuJoCo-native WBC / MPC Backend 在算什么

backend 负责回答最后一个问题：

当前这一帧到底应该下发什么动作，才能在短时域内兼顾动力学、接触和任务目标？

当前实现名称：`mujoco_native_mpc`

它还不是教科书意义上的 QP-WBC，但已经是一个 MuJoCo 原生 rollout MPC 骨架：

1. 根据当前观测重建内部 MuJoCo 状态。
2. 采样一组候选动作修正量。
3. 在 MuJoCo 内对每个候选动作做短时域 rollout。
4. 按阶段目标、接触行为、姿态和终端速度给 rollout 打分。
5. 选得分最好的候选动作，只执行第一步。

### 3.1 当前 backend 实际在优化什么

当前 rollout 代价已经考虑了：

- 目标前向速度
- 目标竖直速度
- 机身 pitch / roll
- crouch / push / flight / landing / settle 与接触数量的一致性
- 短时域前向位移
- 控制输入大小

当前候选动作会扰动：

- push 强度
- 机身 pitch 偏置
- flight 收腿程度
- landing 支撑强度

### 3.2 这个 backend 已经解决了什么

和纯 preview 基线相比，这个 native backend 已经有三件重要的事做对了：

- 它在和仿真相同的 MuJoCo 模型里评估动作
- 它不只看当前时刻，还会看短时域后果
- 它和项目现有的 `LowCmd` 低层链路完全共用一套执行通道

### 3.3 这个 backend 还缺什么

这条 backend 还远没有完成，当前主要短板是：

- 起跳时机还不够干净
- 内部预测阶段和真实接触仍然可能错位
- 还没有显式 centroidal momentum 状态
- 还没有把接触力作为优化变量
- 还没有完整的 whole-body QP 优先级结构
- 还没有足级 reactive landing 重分配

所以现在的 backend 已经是真的 backend，但还只是早期版本。

## 4. 三层现在是怎么串起来的

在每个控制周期里，当前闭环按下面的顺序工作：

1. planner 提供名义 jump task 和阶段参考。
2. controller 用真实观测和接触信息修正这个参考。
3. native backend 在 MuJoCo 里做短时域动作搜索。
4. 选中的第一步动作被转换成 `LowCmd`。
5. 仿真发布新的 `LowState`，然后进入下一次循环。

这就是当前仓库已经打通的最小闭环。

## 5. 下一步最该优先解决什么

下一步最值得继续投入的，不是继续抠几个姿态参数。

真正关键的是把下面三件事对齐：

- 真实接触估计
- 实际执行阶段逻辑
- native rollout 的内部预测

只有这三者对齐了，backend 才能从“局部上看起来合理”走到“全局时序上真正干净的前跳控制器”。
