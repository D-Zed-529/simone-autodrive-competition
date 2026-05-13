# 代码阅读路径

建议从 `Impl/app/autodrive.cpp` 开始阅读。这个文件包含程序入口、初始化流程、
路径生成、主帧循环、驾驶状态切换和最终控制输出，是理解项目行为的主线。

## 推荐阅读顺序

1. `Impl/app/autodrive.cpp`
   - 先理解帧循环和驾驶状态：`Start!`、`Follow`、`NearIntersection`、
     `ChangeLaneStart`、`ObstacleAvoid`、`InChangeLane`、`CarAEB` 和
     `ObstacleAEB`。
2. `Impl/perception/PerceptionUtils.cpp`
   - 查看障碍物列表、限速牌、路口障碍物和人行横道占用如何从 SimOne 数据中检测。
3. `Impl/decision/DecisionUtils.cpp`
   - 阅读换道可行性、交通灯决策、路口拥堵判断和横穿道路辅助逻辑。
4. `Impl/planning/PathUtils.cpp`
   - 查看车道采样、路径工具、检测区域生成和换道路径构造。
5. `Impl/control/pid.h`
   - 理解纵向速度 PID 控制器。
6. `Impl/util/`
   - 阅读主循环使用的数学、字符串和车辆控制小工具。

## 重要边界

- SimOne SDK 的类型、字段和 API 名称保持原样。
- 比赛场景特化逻辑主要集中在 `Impl/app/autodrive.cpp` 后半段。
- `Impl/common/types.h` 仍保留原始障碍物数据结构，供感知、决策、规划和主程序共享。
- `Impl/planning/bezier.h` 是一个小型 header-only 辅助文件，用于在
  `Impl/planning/PathUtils.cpp` 中生成换道路径采样点。
