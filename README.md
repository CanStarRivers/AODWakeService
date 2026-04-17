# Android Hardware Scheduler Framework

![Language](https://img.shields.io/badge/Language-Rust-orange.svg)
![Platform](https://img.shields.io/badge/Platform-Android%20%28Root%29-green.svg)
![License](https://img.shields.io/badge/License-MIT-blue.svg)

本仓库开源了一个基于 Rust 编写的高性能 Android 底层硬件级自动化调度框架。

**⚠️ 仓库结构说明**：
* **源码部分 (Framework)**：本仓库开源的源码（`main.rs`）是一个纯粹的**底层引擎框架**。它剥离了具体的手机型号与业务逻辑，封装了极低开销的底层硬件交互能力。
* **成品文件 (Complete Build)**：仓库中上传的成品文件（或 Release 产物）是基于该框架实现的**完整体程序**——专为 MIUI/HyperOS 定制的息屏显示临时接管服务 (`vendor.hyperfusion.aod.temporary.display-service`)。

---

## 🏗️ 框架核心能力 (面向开发者)

如果你是一名开发者，你可以直接使用本框架快速构建自己的系统级后台常驻服务：

* **控制反转设计**：实现特定的 `RuleHandler` 接口即可注入业务逻辑，无需处理死锁与线程调度。
* **配置热重载**：基于 Linux 原生 `inotify` 机制，实现配置变更毫秒级响应，告别低效轮询。
* **内核级输入拦截**：直接读取 `/dev/input/event*` 节点，精确解析并拦截底层硬件事件（如触控手势、按键）。
* **原生级环境光感知**：以极低延迟高频读取光线传感器 (Lux) 数据，为屏幕亮度的动态调节提供毫无卡顿的数据支撑。
* **安全的生命周期管理**：基于 `Condvar` 的并发状态机，自动申请/维持唤醒锁 (`WakeLock`)，并支持超时后的平滑调度与强制 `Deep Sleep`。

---

## ✨ 完整体程序说明 (面向使用者)

如果你使用的是仓库中提供的完整成品程序，它包含了以下针对 MIUI/HyperOS 深度定制的 AOD 业务逻辑：

1. **进程名防伪校验**：启动时强制校验进程名为 `vendor.hyperfusion.aod.temporary.display-service`，防止恶意重命名或被外部脚本错误拉起。
2. **特定手势唤醒**：精准匹配 `fts` 触控设备，识别特定底层硬件手势（如 KEY_GOTO 354, 338）点亮屏幕。
3. **FOD 防冲突避让**：实时监控屏下指纹 (`fod_press_status`) 状态，指纹识别期间主动放弃接管，防止背光冲突。
4. **智能环境光自适应**：结合高频拉取的环境光数据与多级映射表，动态计算并平滑过渡屏幕目标亮度。

### ⚙️ 环境要求

* **系统**: Android (针对 MIUI / HyperOS)
* **权限**: **必须具备 Root 权限**
* **架构**: `aarch64`

### 🚀 部署与运行

**极其重要**：程序内置了防伪校验，二进制文件名**必须**是 `vendor.hyperfusion.aod.temporary.display-service`，否则启动时将静默退出。

```bash
# 1. 确保文件名为指定的系统进程名
mv <下载的完整体文件> vendor.hyperfusion.aod.temporary.display-service

# 2. 推送至 Android 设备的临时目录
adb push vendor.hyperfusion.aod.temporary.display-service /data/local/tmp/

# 3. 赋予执行权限并以 Root 身份运行
adb shell
su
cd /data/local/tmp/
chmod +x vendor.hyperfusion.aod.temporary.display-service
./vendor.hyperfusion.aod.temporary.display-service
