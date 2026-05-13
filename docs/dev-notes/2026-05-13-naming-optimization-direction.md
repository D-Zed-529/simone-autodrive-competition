# 命名约定

## 适用范围

这些约定用于描述 `Impl/` 下项目自有代码的命名风格。`include/` 中的第三方
SDK 名称，以及 `WinLibs/` 中的二进制库文件名称保持原样。

## 规则

- 类型和类使用 `PascalCase`。
- 项目函数使用 `PascalCase`，并优先使用清晰动词，例如 `Get`、`Detect`、
  `Is`、`Calculate` 和 `Build`。
- 局部变量和参数使用 `lowerCamelCase`。
- 常量使用 `kPascalCase`。
- 布尔变量优先使用 `is`、`has`、`can` 或 `needs` 前缀。
- ID 统一写作 `Id`，避免混用 `ID`、`id` 和 `Id`。
- 单位会在必要时体现在变量名中，例如 `M`、`Mps`、`Rad`。

## 示例

- `kMainVehicleId`：主车 ID 常量。
- `driveStateName`：主循环中使用的字符串驾驶状态。
- `maxSpeedMps`、`stopDistanceM`、`headingErrorRad`：在变量名中标出单位，降低阅读歧义。
- `BuildLaneChangePath`：用于生成局部换道路径采样点的辅助函数。

## 兼容性说明

- SimOne SDK 符号和 SDK 结构体字段保持不变。
- `caseInfo.caseName` 场景特化分支保持显式写法，便于追溯原始比赛行为。
- `obstaclestruct` 及其字段在没有更完整跨模块类型清理计划前保持稳定。
- 驾驶状态和转向方向字符串保持稳定，除非后续统一迁移为 `enum class`。
