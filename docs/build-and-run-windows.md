# Windows 构建与运行

AutoDrive 建议在 Windows + SimOne 环境中运行，因为项目会链接 Windows `.lib`
文件，并从 `WinLibs/` 复制 Windows `.dll` 运行库。

## 环境要求

- Windows 10 或 Windows 11
- SimOne 仿真平台及匹配的运行环境
- Visual Studio 2017 或更高版本，并安装 Desktop development with C++
- CMake 3.26 或更高版本
- CLion、Visual Studio，或 Developer PowerShell for VS

## CLion 配置

1. 在 CLion 中打开仓库根目录。
2. 在 `Settings | Build, Execution, Deployment | Toolchains` 中配置 Visual Studio toolchain。
3. 新建 CMake Profile：
   - Build type: `Release`
   - Generator: `Ninja` 或 Visual Studio
   - Build directory: `cmake-build-release`
4. Reload CMake Project。
5. 构建 `AutoDrive` target。
6. 启动 SimOne，并加载目标场景。
7. 运行 `Release/AutoDrive.exe`。

## 命令行构建

使用 Developer PowerShell for VS：

```powershell
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
.\Release\AutoDrive.exe
```

使用 Visual Studio generator：

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
.\Release\AutoDrive.exe
```

## 运行前提

- SimOne 已经启动。
- 已加载兼容的仿真场景。
- 主车 ID 默认为 `0`，除非在 `Impl/app/autodrive.cpp` 中修改。
- HD Map、GPS、障碍物真值、交通灯和交通标志由 SimOne 环境提供。

## macOS/Linux 说明

项目可以在 macOS/Linux 上阅读源码、执行 CMake 配置检查，也可以用
`clang++ -fsyntax-only` 对部分源码做语法检查。但完整链接和真实运行仍依赖
`WinLibs/` 中的 Windows SimOne 库、MSVC，以及正在运行的 SimOne 场景。
