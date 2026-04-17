//! Android 硬件级自动化调度框架 (Android Hardware Scheduler Framework)
//!
//! 本框架提供了一个极低开销的、基于事件驱动的底层硬件接管引擎。
//! 主要包含了基于 `inotify` 的配置热重载、Linux 输入子系统按键拦截、
//! 以及安全的 WakeLock 与状态机并发调度机制。
//!
//! 开发者只需提供硬件节点路径并实现 [`RuleHandler`] 接口，即可快速构建
//! 类似“息屏显示(AOD)”、“双击亮屏”、“防误触阻断”等系统级后台常驻服务。

use std::ffi::CString;
use std::fs;
use std::io::Read;
use std::mem;
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

// ============================================================================
// 模块一：框架核心配置与对外抽象接口
// ============================================================================

/// 框架运行所需的底层硬件节点与核心调优参数配置。
#[derive(Clone)]
pub struct HardwareConfig {
    /// 配置文件绝对路径（用于 `inotify` 监听服务开关）。
    pub pref_path: String,
    /// 配置文件中表示“功能开启”的匹配特征字符串。
    pub pref_enable_keyword: String,
    /// 背光控制节点路径（如 `/sys/class/leds/lcd-backlight/brightness`）。
    pub backlight_node: String,
    /// 系统挂起状态节点路径，用于判断屏幕是否真正休眠。
    pub suspend_state_node: String,
    /// 自定义阻断节点路径（可选）。例如 FOD 屏下指纹状态节点，值为 `1` 时将阻断唤醒。
    pub block_status_node: Option<String>,
    /// 申请的系统唤醒锁 (`WakeLock`) 标识名称。
    pub wake_lock_name: String,
    /// 渐变背光时的微秒级步进延迟，控制渐明渐暗的平滑度。
    pub fade_delay_us: u64,
    /// 单次触发动作后的维持活跃时长。
    pub active_timeout: Duration,
    /// 传感器数据异常或不可用时的兜底屏幕亮度。
    pub fallback_brightness: i32,
}

/// 标准 Linux 内核输入事件 (`input_event`) 内存对齐映射。
#[repr(C)]
pub struct InputEvent {
    pub sec: i64,
    pub usec: i64,
    pub ev_type: u16,
    pub code: u16,
    pub value: i32,
}

/// 业务触发规则与策略处理器 (`IoC` 边界)。
///
/// 开发者必须实现此 trait 并注入框架，由框架在底层的事件循环中主动回调。
pub trait RuleHandler: Send + Sync {
    /// 解析硬件输入事件，判定是否满足触发条件。
    ///
    /// # Arguments
    ///
    /// * `ev` - 从 `/dev/input/eventX` 读取的原始输入事件引用。
    ///
    /// 返回 `true` 表示命中触发条件（如特定的手势或按键组合被按下）。
    fn match_input_event(&self, ev: &InputEvent) -> bool;

    /// 动态计算目标背光亮度。
    ///
    /// # Arguments
    ///
    /// * `lux` - 后台传感器轮询获取的最新环境光照度值。
    /// * `fallback` - 配置中定义的兜底亮度值。
    fn calculate_target_brightness(&self, lux: f32, fallback: i32) -> i32;

    /// 验证配置文件内容是否表明功能已激活。
    ///
    /// 当框架初次启动或通过 `inotify` 监听到文件变更时触发此验证。
    fn is_feature_enabled_in_config(&self, config_content: &str) -> bool;
}

// ============================================================================
// 模块二：框架底层守护引擎实现
// ============================================================================

const WAKE_LOCK_NODE: &str = "/sys/power/wake_lock";
const WAKE_UNLOCK_NODE: &str = "/sys/power/wake_unlock";

const INOTIFY_HDR: usize = 16;
const INOTIFY_BUF: usize = 512;

/// 框架内部的状态机，由 `Mutex` 保护以确保跨线程的绝对安全。
struct EngineState {
    active: bool,
    deadline: Option<Instant>,
}

/// 核心守护进程引擎。
///
/// 封装了所有多线程调度、底层硬件读写与系统资源的并发安全控制。
pub struct HardwareEngine<T: RuleHandler> {
    config: HardwareConfig,
    handler: Arc<T>,
    is_enabled: AtomicBool,
    /// 用于跨线程无锁传递环境光传感器数据（IEEE 754 位模式存储）。
    latest_lux_bits: AtomicU32,
    state_mtx: Mutex<EngineState>,
    timer_cond: Condvar,
}

impl<T: RuleHandler + 'static> HardwareEngine<T> {
    /// 实例化硬件接管引擎。
    pub fn new(config: HardwareConfig, handler: T) -> Self {
        Self {
            config,
            handler: Arc::new(handler),
            is_enabled: AtomicBool::new(false),
            latest_lux_bits: AtomicU32::new(f32::NAN.to_bits()),
            state_mtx: Mutex::new(EngineState { active: false, deadline: None }),
            timer_cond: Condvar::new(),
        }
    }

    /// 启动引擎，接管当前主线程并进入守护模式。
    ///
    /// # Panics
    ///
    /// 若系统环境中缺失关键的底层驱动节点（如背光控制节点），该方法将抛出 panic。
    pub fn run(self) {
        if !self.path_exists(&self.config.backlight_node) {
            panic!("[FRAMEWORK-FATAL] 必备节点缺失: {}", self.config.backlight_node);
        }

        let engine = Arc::new(self);

        // 阶段一：初次同步配置状态
        let initial_enabled = engine.read_config_status();
        engine.is_enabled.store(initial_enabled, Ordering::Relaxed);

        if !initial_enabled {
            eprintln!("[FRAMEWORK-INFO] 功能当前未开启，挂起等待配置文件变更...");
            engine.watch_pref_loop(|enabled| !enabled);
        }

        // 定位触控输入节点
        let touch_node = match engine.find_input_node() {
            Some(n) => n,
            None => panic!("[FRAMEWORK-FATAL] 未能在 /dev/input/ 下找到适用的触控/手势节点"),
        };

        // 阶段二：派发底层并发控制线程矩阵
        {
            let e = engine.clone();
            thread::Builder::new()
                .name("hw-sensor-poll".into())
                .spawn(move || e.run_sensor_polling_loop())
                .expect("Failed to spawn sensor thread");
        }
        {
            let e = engine.clone();
            thread::Builder::new()
                .name("hw-timeout-mgr".into())
                .spawn(move || e.handle_timeout_loop())
                .expect("Failed to spawn timeout thread");
        }
        {
            let e = engine.clone();
            thread::Builder::new()
                .name("hw-input-monitor".into())
                .spawn(move || e.monitor_input_loop(&touch_node))
                .expect("Failed to spawn input thread");
        }

        eprintln!("[FRAMEWORK-INFO] 底层引擎全面就绪，主线程转入 inotify 热重载监听模式");
        engine.watch_pref_loop(|enabled| {
            engine.is_enabled.store(enabled, Ordering::Relaxed);
            true
        });
    }

    // ─── 核心生命周期与拦截调度 ───────────────────────────────────────────────

    /// 硬件唤醒触发器。处理环境拦截、动态背光计算与资源加锁。
    fn trigger_wakeup(&self) {
        // [拦截网 1] 配置文件功能总开关验证
        if !self.is_enabled.load(Ordering::Relaxed) { return; }
        
        // [拦截网 2] 系统层挂起状态验证
        if self.read_node_i32(&self.config.suspend_state_node) == 0 { return; }
        
        // [拦截网 3] 业务阻断节点验证（如 FOD 解锁中）
        if let Some(ref block_node) = self.config.block_status_node {
            if self.read_node_string(block_node).trim() == "1" { return; }
        }

        // [业务派发] 通过 trait 回调计算目标亮度
        let bits = self.latest_lux_bits.load(Ordering::Relaxed);
        let lux = f32::from_bits(bits);
        let target_b = self.handler.calculate_target_brightness(lux, self.config.fallback_brightness);
        let current_b = self.read_node_i32(&self.config.backlight_node);

        // [拦截网 4] 防背光冲突验证
        if current_b > target_b { return; }

        // [执行动作] 状态机刷新与资源加锁
        let mut s = self.state_mtx.lock().unwrap();
        s.deadline = Some(Instant::now() + self.config.active_timeout);
        self.timer_cond.notify_one();
        
        if !s.active {
            s.active = true;
            self.write_node(WAKE_LOCK_NODE, &self.config.wake_lock_name);
            self.fade_brightness(current_b, target_b);
        }
    }

    /// 生命周期超时与资源回收控制器。
    fn handle_timeout_loop(&self) {
        loop {
            let mut s = self.state_mtx.lock().unwrap();
            s = self.timer_cond.wait_while(s, |s| s.deadline.is_none()).unwrap();

            let fired = loop {
                let deadline = match s.deadline {
                    Some(d) => d,
                    None => break false,
                };
                let now = Instant::now();
                if now >= deadline {
                    s.deadline = None;
                    break true;
                }
                let (g, res) = self.timer_cond.wait_timeout(s, deadline - now).unwrap();
                s = g;
                if res.timed_out() && s.deadline.map_or(false, |d| Instant::now() >= d) {
                    s.deadline = None;
                    break true;
                }
            };

            if fired && s.active {
                // 判断系统框架是否已经主动亮屏接管
                if self.read_node_i32(&self.config.suspend_state_node) == 0 {
                    s.active = false;
                    self.write_node(WAKE_UNLOCK_NODE, &self.config.wake_lock_name);
                } else {
                    // 渐暗熄灭，强制系统进入 Deep Sleep
                    let cur = self.read_node_i32(&self.config.backlight_node);
                    self.fade_brightness(cur, 0);
                    s.active = false;
                    self.write_node(WAKE_UNLOCK_NODE, &self.config.wake_lock_name);
                    drop(s);
                    self.force_deep_sleep();
                }
            }
        }
    }

    // ─── 硬件外设与传感器轮询交互 ─────────────────────────────────────────────

    /// 后台环境光传感器 (Lux) 独立轮询线程。
    ///
    /// **[开源集成说明]**
    /// 为了保障框架的跨平台通用性，具体的 Android NDK 传感器拉取逻辑已剥离。
    /// 实际应用中，开发者可在此处通过 NDK `ASensorManager`、或直接读取 
    /// `/sys/class/i2c-dev/` 等节点来高频获取光线数据。
    fn run_sensor_polling_loop(&self) {
        loop {
            // [TODO]: 开发者应在此处实现具体的硬件数据读取逻辑。
            // 示例：获取到最新的环境光数值后，通过无锁原子操作共享给调度引擎。
            // 
            // let current_lux: f32 = fetch_hardware_lux();
            // self.latest_lux_bits.store(current_lux.to_bits(), Ordering::Relaxed);

            // 防 CPU 满载休眠控制 (默认 ~60Hz 采样率)
            thread::sleep(Duration::from_millis(16));
        }
    }

    /// 内核 `/dev/input/` 输入事件极速解析与拦截。
    fn monitor_input_loop(&self, device_path: &str) {
        let mut file = match fs::File::open(device_path) {
            Ok(f) => f,
            Err(e) => panic!("[FRAMEWORK-FATAL] 无法接管输入节点: {}", e),
        };
        let ev_size = mem::size_of::<InputEvent>();
        let mut buf = vec![0u8; ev_size];
        
        loop {
            if file.read_exact(&mut buf).is_err() {
                thread::sleep(Duration::from_millis(100));
                continue;
            }
            // 安全断言：依靠 #[repr(C)] 对齐内存，直接将字节流强转为结构体引用
            let ev: &InputEvent = unsafe { &*(buf.as_ptr() as *const InputEvent) };
            if self.handler.match_input_event(ev) {
                self.trigger_wakeup();
            }
        }
    }

    /// 基于 Linux 原生 `inotify` 机制的配置文件极低开销热重载。
    fn watch_pref_loop(&self, mut on_change: impl FnMut(bool) -> bool) {
        let pref_dir = match Path::new(&self.config.pref_path).parent().and_then(|p| p.to_str()) {
            Some(d) => d,
            None => return self.poll_pref_loop(on_change),
        };
        let target_file = Path::new(&self.config.pref_path).file_name().and_then(|n| n.to_str()).unwrap_or("");

        let dir_c = match CString::new(pref_dir) {
            Ok(c) => c,
            Err(_) => return self.poll_pref_loop(on_change),
        };

        let fd = unsafe { libc::inotify_init1(libc::IN_CLOEXEC) };
        if fd < 0 { return self.poll_pref_loop(on_change); }

        let wd = unsafe {
            libc::inotify_add_watch(fd, dir_c.as_ptr(), libc::IN_CLOSE_WRITE | libc::IN_MOVED_TO)
        };
        if wd < 0 {
            unsafe { libc::close(fd) };
            return self.poll_pref_loop(on_change);
        }

        let mut buf = [0u8; INOTIFY_BUF];
        'outer: loop {
            let n = unsafe { libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, INOTIFY_BUF) };
            if n <= 0 { continue; }
            let n = n as usize;
            let mut off = 0usize;
            while off + INOTIFY_HDR <= n {
                let name_len = u32::from_ne_bytes(buf[off + 12..off + 16].try_into().unwrap()) as usize;
                let next = off + INOTIFY_HDR + name_len;
                if name_len > 0 && next <= n {
                    let nb = &buf[off + INOTIFY_HDR..next];
                    let nul = nb.iter().position(|&b| b == 0).unwrap_or(nb.len());
                    let name = std::str::from_utf8(&nb[..nul]).unwrap_or("");
                    if name == target_file && !on_change(self.read_config_status()) {
                        break 'outer;
                    }
                }
                off = if next > off { next } else { off + INOTIFY_HDR };
            }
        }
        unsafe { libc::close(fd) };
    }

    /// `inotify` 初始化失败时的降级方案：常规轮询。
    fn poll_pref_loop(&self, mut on_change: impl FnMut(bool) -> bool) {
        loop {
            if !on_change(self.read_config_status()) { return; }
            thread::sleep(Duration::from_secs(5));
        }
    }

    // ─── 硬件控制工具集 ───────────────────────────────────────────────────────

    /// 平滑控制屏幕背光渐变。
    fn fade_brightness(&self, from: i32, to: i32) {
        if from == to { return; }
        let step: i32 = if from < to { 1 } else { -1 };
        let mut cur = from;
        loop {
            self.write_node(&self.config.backlight_node, &cur.to_string());
            thread::sleep(Duration::from_micros(self.config.fade_delay_us));
            if cur == to { break; }
            cur += step;
        }
    }

    /// 释放阻止睡眠的 Wakelock，强制系统回退至 Deep Sleep 状态。
    fn force_deep_sleep(&self) {
        for lock in ["PowerManagerService.noSuspend", "SensorsHAL_WAKEUP"] {
            self.write_node(WAKE_UNLOCK_NODE, lock);
        }
    }

    /// 动态寻找特定的硬件输入节点 (默认寻找 `fts` 前缀设备)。
    fn find_input_node(&self) -> Option<String> {
        for entry in fs::read_dir("/sys/class/input/").ok()?.filter_map(|e| e.ok()) {
            let path = entry.path();
            let fname = path.file_name()?.to_str()?;
            if !fname.starts_with("event") { continue; }
            if let Ok(data) = fs::read_to_string(path.join("device/name")) {
                if data.to_ascii_lowercase().contains("fts") {
                    return Some(format!("/dev/input/{}", fname));
                }
            }
        }
        None
    }

    // ─── IO 辅助方法 ──────────────────────────────────────────────────────────

    fn read_config_status(&self) -> bool {
        fs::read_to_string(&self.config.pref_path)
            .map_or(false, |data| self.handler.is_feature_enabled_in_config(&data))
    }

    fn read_node_i32(&self, path: &str) -> i32 {
        fs::read_to_string(path).ok().and_then(|s| s.trim().parse().ok()).unwrap_or(0)
    }

    fn read_node_string(&self, path: &str) -> String {
        fs::read_to_string(path).unwrap_or_default()
    }

    fn write_node(&self, path: &str, val: &str) {
        let _ = fs::write(path, val.as_bytes());
    }

    fn path_exists(&self, path: &str) -> bool {
        Path::new(path).exists()
    }
}
                }
            }
        }
    }
    
    fn monitor_input(&self, device_path: &str) {
        let mut file = match fs::File::open(device_path) {
            Ok(f) => f,
            Err(_) => return,
        };
        let ev_size = mem::size_of::<InputEvent>();
        let mut buf = vec![0u8; ev_size];
        loop {
            if file.read_exact(&mut buf).is_err() {
                thread::sleep(Duration::from_millis(100));
                continue;
            }
            // SAFETY:
            let ev: &InputEvent = unsafe { &*(buf.as_ptr() as *const InputEvent) };
            if ev.ev_type == EV_KEY
                && (ev.code == GESTURE_KEY_1 || ev.code == GESTURE_KEY_2)
                && ev.value == 1
            {
                self.trigger_wakeup();
            }
        }
    }
    
    fn run_config_watcher(&self) {
        watch_pref_loop(|enabled| {
            self.is_aod_enabled.store(enabled, Ordering::Relaxed);
            true
        });
    }
}

fn main() {
    if !path_exists(BACKLIGHT_NODE) || !path_exists(SUSPEND_STATE_NODE) {
        return;
    }

    if !read_aod_enabled() {
        // on_change 返回 false 时循环停止，即 enabled=true 时退出
        watch_pref_loop(|enabled| !enabled);
    }

    let touch_node = match find_fts_touch_node() {
        Some(n) => n,
        None => return,
    };

    let svc = Arc::new(AodService::new());

    {
        let s = svc.clone();
        thread::Builder::new()
            .name("aod-timeout".into())
            .spawn(move || s.handle_timeout())
            .expect("spawn timeout thread");
    }
    {
        let s = svc.clone();
        thread::Builder::new()
            .name("aod-input".into())
            .spawn(move || s.monitor_input(&touch_node))
            .expect("spawn input thread");
    }
    
    svc.run_config_watcher();
}

fn read_aod_enabled() -> bool {
    fs::read_to_string(PREF_PATH).map_or(false, |data| {
        data.lines()
            .any(|l| l.contains("aod_temporary_style") && l.contains("\"true\""))
    })
}

fn watch_pref_loop(mut on_change: impl FnMut(bool) -> bool) {
    let pref_dir = match Path::new(PREF_PATH).parent().and_then(|p| p.to_str()) {
        Some(d) => d,
        None => return poll_pref_loop(on_change),
    };
    let target_file = Path::new(PREF_PATH)
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("");

    let dir_c = match CString::new(pref_dir) {
        Ok(c) => c,
        Err(_) => return poll_pref_loop(on_change),
    };

    let fd = unsafe { libc::inotify_init1(libc::IN_CLOEXEC) };
    if fd < 0 {
        return poll_pref_loop(on_change);
    }

    let wd = unsafe {
        libc::inotify_add_watch(
            fd,
            dir_c.as_ptr(),
            libc::IN_CLOSE_WRITE | libc::IN_MOVED_TO,
        )
    };
    if wd < 0 {
        unsafe { libc::close(fd) };
        return poll_pref_loop(on_change);
    }

    let mut buf = [0u8; INOTIFY_BUF];

    'outer: loop {
        let n = unsafe {
            libc::read(fd, buf.as_mut_ptr() as *mut libc::c_void, INOTIFY_BUF)
        };
        if n <= 0 {
            continue;
        }
        let n = n as usize;
        let mut off = 0usize;

        while off + INOTIFY_HDR <= n {
            // name_len 位于事件头 [off+12 .. off+16]
            let name_len = u32::from_ne_bytes(
                buf[off + 12..off + 16].try_into().unwrap(),
            ) as usize;

            let next_off = off + INOTIFY_HDR + name_len;

            if name_len > 0 && next_off <= n {
                let name_bytes = &buf[off + INOTIFY_HDR..next_off];
                let nul = name_bytes
                    .iter()
                    .position(|&b| b == 0)
                    .unwrap_or(name_bytes.len());
                let name = std::str::from_utf8(&name_bytes[..nul]).unwrap_or("");

                if name == target_file && !on_change(read_aod_enabled()) {
                    break 'outer;
                }
            }

            // 防止格式异常的零长事件导致死循环
            off = if next_off > off { next_off } else { off + INOTIFY_HDR };
        }
    }

    unsafe { libc::close(fd) };
}

fn poll_pref_loop(mut on_change: impl FnMut(bool) -> bool) {
    loop {
        if !on_change(read_aod_enabled()) {
            return;
        }
        thread::sleep(Duration::from_secs(5));
    }
}

fn find_fts_touch_node() -> Option<String> {
    for entry in fs::read_dir("/sys/class/input/").ok()?.filter_map(|e| e.ok()) {
        let path = entry.path();
        let fname = path.file_name()?.to_str()?;
        if !fname.starts_with("event") {
            continue;
        }
        if let Ok(data) = fs::read_to_string(path.join("device/name")) {
            if data.to_ascii_lowercase().contains("fts") {
                return Some(format!("/dev/input/{}", fname));
            }
        }
    }
    None
}

fn get_suspend_state() -> i32 {
    fs::read_to_string(SUSPEND_STATE_NODE)
        .ok()
        .and_then(|s| s.trim().parse().ok())
        .unwrap_or(0)
}

fn path_exists(path: &str) -> bool {
    Path::new(path).exists()
}

fn write_node(path: &str, val: &str) {
    let _ = fs::write(path, val.as_bytes());
}

fn fade_brightness(from: i32, to: i32) {
    if from == to {
        return;
    }
    let step: i32 = if from < to { 1 } else { -1 };
    let mut cur = from;
    loop {
        write_node(BACKLIGHT_NODE, &cur.to_string());
        thread::sleep(Duration::from_micros(FADE_DELAY_US));
        if cur == to {
            break;
        }
        cur += step;
    }
}

fn force_deep_sleep() {
    for lock in ["PowerManagerService.noSuspend", "SensorsHAL_WAKEUP"] {
        write_node(WAKE_UNLOCK_NODE, lock);
    }
}
