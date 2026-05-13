# 场景能力映射

下表说明比赛场景中的主要驾驶能力，以及对应的实现策略和源码入口。

| 能力 | 策略 | 主要入口 |
| --- | --- | --- |
| 全局路径循迹 | 由起点和终点生成 SimOne 全局路径，再跟踪前方路径点。 | `Impl/app/autodrive.cpp`, `Impl/planning/PathUtils.cpp` |
| 限速响应 | 检测限速牌，并更新 PID 控制器的目标速度。 | `Impl/perception/PerceptionUtils.cpp`, `Impl/app/autodrive.cpp` |
| 有灯路口通行 | 检测停止线、交通灯和人行横道，再决定通行或停车。 | `Impl/decision/DecisionUtils.cpp`, `Impl/app/autodrive.cpp` |
| 无灯路口通行 | 在没有交通灯控制时检测停止线和人行横道占用情况。 | `Impl/decision/DecisionUtils.cpp` |
| 静态障碍避让 | 检测短期路径上的第一个静态障碍物，并在可换道时生成避障路径。 | `Impl/perception/PerceptionUtils.cpp`, `Impl/decision/DecisionUtils.cpp`, `Impl/planning/PathUtils.cpp` |
| 动态障碍处理 | 检测规划路径上的运动障碍物，并调整速度或触发换道行为。 | `Impl/perception/PerceptionUtils.cpp`, `Impl/app/autodrive.cpp` |
| 换道 | 检查相邻车道类型、道路标线和检测区域占用情况，再生成换道路径。 | `Impl/decision/DecisionUtils.cpp`, `Impl/planning/PathUtils.cpp` |
| 紧急制动 | 根据距离阈值和障碍物速度进入面向车辆或其他障碍物的 AEB 类状态。 | `Impl/app/autodrive.cpp` |
| 长距离场景 | 对长距离场景额外处理速度上限和路径重建。 | `Impl/app/autodrive.cpp` |

## 场景特化逻辑

原始比赛方案中包含少量 `caseInfo.caseName` 分支，用于处理特定场景 ID。
这些分支被刻意保留，以便追溯比赛时期的行为。它们应被视为场景特化策略，
而不是通用自动驾驶规则。

如果要迁移到新的场景集，建议优先检查这些分支：其中可能包含原比赛环境中的
路径长度假设、一次性限速策略或场景名称判断。
