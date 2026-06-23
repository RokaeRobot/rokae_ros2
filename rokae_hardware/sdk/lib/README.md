# xCore SDK 预编译库 / Prebuilt Libraries

本目录**不包含**预编译库。库文件从 [xCoreSDK-CPP Releases](https://github.com/RokaeRobot/xCoreSDK-CPP/releases) 下载，版本号见上级目录 [`VERSION`](../VERSION)。

This directory does **not** include prebuilt libraries. Download them from [xCoreSDK-CPP Releases](https://github.com/RokaeRobot/xCoreSDK-CPP/releases); the required version is in [`VERSION`](../VERSION).

**Release URL：**

`https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v{VERSION}`

例如当前 0.7.1：[Release v0.7.1](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)

## 获取步骤 / How to Obtain

1. 查看 `rokae_hardware/sdk/VERSION` 中的版本号
2. 打开对应 [Release 页面](https://github.com/RokaeRobot/xCoreSDK-CPP/releases/tag/v0.7.1)
3. 下载 Linux 库包，例如：
   - `xCoreSDK-0.7.1-linux-x86_64.tar.gz`
   - `xCoreSDK-0.7.1-linux-aarch64.tar.gz`
4. 解压 Release 包，将以下文件**复制到本目录** `rokae_hardware/sdk/lib/`：

### Linux x86_64 / aarch64

从 Release 包内 `lib/Linux/<arch>/` 复制：

```
rokae_hardware/sdk/lib/
  libxCoreSDK.a
  libxMateModel.a
  libxCoreSDK.so.0.7.1    # 可选，动态库
  libxCoreSDK.so          # 可选，若 Release 包提供符号链接
```

`rokae_hardware` 与 `rokae_example` 默认链接静态库 `libxCoreSDK.a` 与 `libxMateModel.a`。

### Windows（交叉开发参考）

从 Release 包内 `lib/Windows/Release/64bit/` 复制 `xCoreSDK_static.lib`、`xMateModel.lib` 等到本目录，并在 CMake 中按平台调整库名（当前 CMake 面向 Linux 静态库命名）。

## 验证 / Verify

在 ROS 2 工作空间根目录执行：

```bash
colcon build --packages-select rokae_hardware
```

若库缺失，CMake 会输出 WARNING 并提示下载地址。

## 维护者说明 / Maintainers

- 升级 SDK 时：同步更新 `sdk/include/` 头文件、`sdk/VERSION`，并在栈 CHANGELOG 中注明
- **禁止**将 `.a` / `.so` / `.dll` / `.lib` 提交到 Git
- 完整 xCore SDK 发版流程见 [xCoreSDK-CPP lib/README.md](https://github.com/RokaeRobot/xCoreSDK-CPP/blob/main/lib/README.md)
