# 2025 届中国大学生工程实践与创新能力大赛｜虚拟仿真赛道｜智能网联汽车设计赛项

## AutoDrive

AutoDrive 是一个基于 SimOne 仿真平台的 C++ 自动驾驶控制项目。项目通过 SimOne API 获取主车状态、高精地图、交通信号和障碍物真值数据，并在每一帧完成感知、决策、路径规划和车辆控制。

这个项目源自 **2025 届中国大学生工程实践与创新能力大赛** 的 **虚拟仿真赛道 - 智能网联汽车设计赛项**。仓库适合用来学习 SimOne API 的基本使用方式，以及一个规则驱动自动驾驶程序如何组织路径规划、换道避障、路口通行和 PID 控制。

省赛阶段取得 1W+ 分数。

## 项目背景

比赛场景要求车辆在 SimOne 虚拟环境中完成循迹、限速响应、路口通行、障碍物处理、换道避障和安全制动等任务。本项目采用规则驱动方案：使用 SimOne 提供的 HD Map、GPS、障碍物真值、交通灯和交通标志信息，通过状态机组织驾驶行为，再用 PID 控制纵向速度。

更完整的背景说明见 [docs/competition-background.md](docs/competition-background.md)。

## 功能概览

- 感知输入：通过 `GetGps`、`GetGroundTruth` 等接口读取车辆状态和障碍物信息。
- 地图与路径：加载 HD Map，使用 SimOne 路径生成接口获得全局路径，并在换道时生成局部路径。
- 决策状态机：围绕 `Follow`、`NearIntersection`、`ChangeLaneStart`、`ObstacleAvoid`、`InChangeLane`、`CarAEB` 等状态组织驾驶行为。
- 车辆控制：使用 PID 控制目标速度，通过 `UtilDriver::calculateSteering` 和 `UtilDriver::setDriver` 输出油门、刹车和转向。
- 仿真同步：在 SimOne 帧同步模式下循环执行感知、决策、控制，并调用 `NextFrame` 推进仿真。

## 方案亮点

- 状态机驱动：用明确状态表达跟车、路口、换道、避障和 AEB 行为。
- HD Map 辅助：基于车道、停止线、信号灯、道路标线和相邻车道信息做规则判断。
- 局部路径生成：在换道和避障时生成短期目标路径，并与后续全局路径拼接。
- 纵向 PID 控制：根据最大速度、限速牌、障碍物距离和场景状态动态更新目标速度。
- 比赛场景覆盖：保留部分 `caseInfo.caseName` 分支，用于复现原始比赛场景中的特定策略。

能力映射见 [docs/scenario-capabilities.md](docs/scenario-capabilities.md)，系统结构见 [docs/architecture.md](docs/architecture.md)。
命名约定见 [docs/dev-notes/2026-05-13-naming-optimization-direction.md](docs/dev-notes/2026-05-13-naming-optimization-direction.md)。

## 项目结构

```text
.
├── CMakeLists.txt          # CMake 构建入口
├── Impl/                   # 自动驾驶主要实现
│   ├── app/                # main 入口和主状态机
│   ├── common/             # 项目内部共享类型和函数声明
│   ├── control/            # PID 等控制模块
│   ├── decision/           # 换道、安全区域、交通规则决策工具
│   ├── perception/         # 障碍物、路口、信号相关感知工具
│   ├── planning/           # 路径截取、换道路径生成等工具
│   └── util/               # 通用数学、字符串和驾驶控制工具
├── include/                # SimOne SDK、Eigen、nlohmann/json 等头文件
└── WinLibs/                # SimOne SDK、SSD、FFmpeg 等 Windows 动态库和导入库
```

构建生成目录如 `build/`、`out/`、`cmake-build-*`、`Release/`、`.idea/`、`.vs/` 不属于源码内容，已经通过 `.gitignore` 排除。

## 环境要求

建议使用 Windows 环境运行本项目，因为项目依赖 SimOne Windows SDK 的 `.lib` 和 `.dll`。

- Windows 10/11
- SimOne 仿真平台
- Visual Studio 2017 或更高版本，安装 Desktop development with C++
- CMake 3.26 或更高版本
- CLion 或 Visual Studio

## 用 CLion 配置项目

CLion 是推荐给初学者的入口，因为它可以直接读取 `CMakeLists.txt`。

1. 安装 Visual Studio，并确认安装了 MSVC 编译工具链。
2. 打开 CLion，选择 `Open`，打开本仓库根目录。
3. 进入 `Settings | Build, Execution, Deployment | Toolchains`。
4. 新增或选择 Visual Studio toolchain，Architecture 建议选择 `amd64`。
5. 进入 `Settings | Build, Execution, Deployment | CMake`。
6. 添加一个 Profile，例如 `x64-Release`：
   - Build type: `Release`
   - Generator: `Ninja` 或 `Visual Studio`
   - Toolchain: 刚才配置的 Visual Studio toolchain
   - Build directory: `cmake-build-release`
7. 点击 Reload CMake Project。
8. 在右上角运行配置中选择 `AutoDrive`，先 Build，再运行。

如果 CLion 找不到 MSVC 或 Windows SDK，优先检查 Visual Studio Installer 中是否安装了 C++ workload，而不是修改项目源码。

更详细的 Windows 构建与运行说明见 [docs/build-and-run-windows.md](docs/build-and-run-windows.md)。

## 用命令行构建

在 Developer PowerShell for VS 或 Developer Command Prompt for VS 中执行：

```powershell
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

也可以使用 Visual Studio generator：

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

构建成功后，`CMakeLists.txt` 会把可执行文件输出到：

```text
Release/AutoDrive.exe
```

同时会把运行需要的 `SimOneAPI.dll`、`SSD.dll`、`HDMapModule.dll` 从 `WinLibs/` 复制到 `Release/`。

## 运行方式

1. 启动 SimOne 仿真平台，并打开对应场景。
2. 确认主车 ID 与代码中的 `kMainVehicleId` 一致，默认在 `Impl/app/autodrive.cpp` 中为 `0`。
3. 构建项目。
4. 运行生成的程序：

```powershell
.\Release\AutoDrive.exe
```

程序启动后会初始化 SimOne API，加载 HD Map，读取起终点并生成全局路径，然后进入帧同步主循环。

运行前需要 SimOne 场景已经加载，并由仿真环境提供 GPS、HD Map、障碍物真值、交通灯和交通标志等数据。单独运行可执行文件无法脱离 SimOne 环境完成仿真。

## 主流程说明

`Impl/app/autodrive.cpp` 是理解项目的最佳入口。

初始化阶段：

1. 设置主车 ID、目标点、PID 参数和运行状态。
2. 调用 `SimOneAPI::InitSimOneAPI` 连接仿真平台。
3. 调用 `SimOneAPI::LoadHDMap` 加载高精地图。
4. 读取 GPS，确认仿真数据可用。
5. 调用 `SimOneAPI::GenerateRoute` 生成全局参考路径。

主循环阶段：

1. 感知：读取 GPS、障碍物真值、当前车道、停止线、限速牌和信号灯。
2. 决策：根据障碍物距离、交通灯、路口拥堵、可换道区域等条件更新 `driveStateName`。
3. 规划：必要时生成换道路径或避障路径，并拼接后续全局路径。
4. 控制：计算目标速度、油门、刹车和方向盘角度。
5. 同步：调用 `SimOneAPI::NextFrame` 进入下一帧。

## 关键文件

- `Impl/app/autodrive.cpp`：主程序、状态机和控制闭环。
- `Impl/common/`：项目内部共享类型和工具函数声明。
- `Impl/perception/PerceptionUtils.cpp`：障碍物筛选、前方障碍检测、路口障碍检测。
- `Impl/decision/DecisionUtils.cpp`：换道可行性、安全区域占用、路口规则判断。
- `Impl/planning/PathUtils.cpp`：路径截取、贝塞尔换道路径和目标车道路径。
- `Impl/utilTargetLane.h`：车道查询、车道目标点和相邻车道处理。
- `Impl/utilTargetObstacle.h`：障碍物结构与目标障碍物辅助逻辑。
- `Impl/control/pid.h`：速度控制 PID。

如果你想按阅读路径理解代码，建议从 [docs/code-walkthrough.md](docs/code-walkthrough.md) 开始。

## 当前验证状态

- 已在 macOS 上完成 CMake 配置检查和核心 `.cpp` 的 `clang++ -fsyntax-only` 语法检查。
- 完整构建、链接和运行依赖 Windows + MSVC + SimOne 运行环境。
- 项目保留了比赛时期的场景特化逻辑，主要用于复现和学习，不代表通用量产级自动驾驶系统。

## 常见问题

### CLion 里 CMake 配置失败

通常是 MSVC toolchain 没有配置好。先确认 Visual Studio Installer 中安装了 C++ 桌面开发组件，再回到 CLion 的 Toolchains 页面检查 C Compiler、C++ Compiler、Debugger 是否都能识别。

### 编译成功但运行失败

检查 `Release/` 下是否存在 `SimOneAPI.dll`、`SSD.dll`、`HDMapModule.dll`。如果缺少这些文件，重新执行 CMake build，或者确认 `WinLibs/` 中 SDK 文件完整。

### 程序连接不上 SimOne

确认 SimOne 仿真平台已经启动，场景已经加载，代码中的主车 ID 与场景配置一致。默认连接地址来自 SimOne SDK 的 `InitSimOneAPI` 默认参数。

### 不应该提交哪些文件

不要提交 IDE 缓存、CMake 构建目录、`Release/` 编译输出、`.pdb`、`.obj`、日志和本机 `settings.json`。这些文件可以由本地环境重新生成，开源仓库只需要保留源码、CMake 配置、第三方 SDK 头文件和必要库文件。

## 许可证

本项目代码以 MIT License 发布，见 [LICENSE](LICENSE)。
