use std::ffi::CString;
use std::fs;
use std::io::Read;
use std::mem;
use std::path::Path;
use std::ptr;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread;
use std::time::{Duration, Instant};

const PREF_PATH: &str = "/data/user_de/0/com.miui.aod/shared_prefs/com.miui.aod_preferences.xml";
// 不是标准 backlight，小米自己的 clone 节点
const BACKLIGHT_NODE: &str = "/sys/devices/virtual/mi_display/disp_feature/disp-DSI-0/brightness_clone";
const WAKE_LOCK_NODE: &str = "/sys/power/wake_lock";
const WAKE_UNLOCK_NODE: &str = "/sys/power/wake_unlock";
const DPMS_NODE: &str = "/sys/class/drm/card0-DSI-1/dpms";
const FOD_STATUS_NODE: &str = "/sys/devices/virtual/touch/touch_dev/fod_press_status";
const MY_AOD_WAKE_LOCK: &str = "HyperfusionAodService_Lock";

const AOD_TIMEOUT: Duration = Duration::from_secs(10);

// lux 从高到低，匹配第一个满足的档位
const LUX_LEVELS: &[(f32, i32)] = &[
    (1000.0, 8192),
    (200.0,  4096),
    (10.0,   350),
    (1.0,    100),
    (0.0,    50),
];

const FALLBACK_BRIGHTNESS: i32 = 50;
const FADE_DELAY_US: u64 = 2000;

const EV_KEY: u16 = 1;
// 双击手势 keycode，fuxi 实测
const GESTURE_KEY_1: u16 = 354;
const GESTURE_KEY_2: u16 = 338;
const BTN_TOUCH: u16 = 330;

// 必须和内核 input_event 内存布局一致
#[repr(C)]
struct InputEvent {
    sec: i64,
    usec: i64,
    ev_type: u16,
    code: u16,
    value: i32,
}

const INOTIFY_HDR: usize = 16;
const INOTIFY_BUF: usize = 512;

struct ScreenState {
    on: bool,
    deadline: Option<Instant>,
}

// lux 用 f32 bit pattern 塞进 AtomicU32，省一把锁
struct AodService {
    is_aod_enabled: AtomicBool,
    latest_lux_bits: AtomicU32,
    screen_mtx: Mutex<ScreenState>,
    timer_cond: Condvar,
}

impl AodService {
    fn new() -> Self {
        Self {
            is_aod_enabled: AtomicBool::new(true),
            latest_lux_bits: AtomicU32::new(f32::NAN.to_bits()), // NaN = 传感器还没数据
            screen_mtx: Mutex::new(ScreenState { on: false, deadline: None }),
            timer_cond: Condvar::new(),
        }
    }

    fn lux_to_brightness(&self) -> i32 {
        let bits = self.latest_lux_bits.load(Ordering::Relaxed);
        let lux = f32::from_bits(bits);
        if lux.is_nan() || lux < 0.0 {
            return FALLBACK_BRIGHTNESS;
        }
        for &(threshold, brightness) in LUX_LEVELS {
            if lux >= threshold {
                return brightness;
            }
        }
        FALLBACK_BRIGHTNESS
    }

    fn trigger_wakeup(&self) {
        if !self.is_aod_enabled.load(Ordering::Relaxed) {
            return;
        }
        if is_dpms_on() {
            return;
        }
        // fod 识别中，让它自己处理亮屏
        if is_fod_pressed() {
            return;
        }
        let target = self.lux_to_brightness();
        let current_b = get_current_brightness();
        if current_b > target {
            return;
        }

        let mut s = self.screen_mtx.lock().unwrap();
        s.deadline = Some(Instant::now() + AOD_TIMEOUT);
        self.timer_cond.notify_one();
        if !s.on {
            s.on = true;
            write_node(WAKE_LOCK_NODE, MY_AOD_WAKE_LOCK);
            fade_brightness(current_b, target);
        }
    }

    fn handle_timeout(&self) {
        loop {
            let mut s = self.screen_mtx.lock().unwrap();
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

            if fired && s.on {
                if is_dpms_on() {
                    s.on = false;
                    write_node(WAKE_UNLOCK_NODE, MY_AOD_WAKE_LOCK);
                } else {
                    let cur = get_current_brightness();
                    fade_brightness(cur, 0);
                    s.on = false;
                    write_node(WAKE_UNLOCK_NODE, MY_AOD_WAKE_LOCK);
                    drop(s); // force_deep_sleep 可能阻塞，先放锁
                    force_deep_sleep();
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
        // 初始化往前推 10 秒，保证启动后第一次触摸立即生效
        let mut last_trigger_time = Instant::now() - Duration::from_secs(10);

        loop {
            if file.read_exact(&mut buf).is_err() {
                thread::sleep(Duration::from_millis(100));
                continue;
            }
            let ev: &InputEvent = unsafe { &*(buf.as_ptr() as *const InputEvent) };

            if ev.ev_type == EV_KEY {
                if (ev.code == GESTURE_KEY_1 || ev.code == GESTURE_KEY_2) && ev.value == 1 {
                    self.trigger_wakeup();
                } else if ev.code == BTN_TOUCH && ev.value == 1 {
                    let now = Instant::now();
                    if now.duration_since(last_trigger_time) > Duration::from_millis(500) {
                        self.trigger_wakeup();
                        last_trigger_time = now;
                    }
                }
            }
        }
    }

    fn run_lux_sensor(&self) {
        unsafe {
            use ndk_sys::*;

            let sm = ASensorManager_getInstance();
            if sm.is_null() {
                return;
            }
            let sensor = ASensorManager_getDefaultSensor(sm, ASENSOR_TYPE_LIGHT as i32);
            if sensor.is_null() {
                return;
            }
            let looper = ALooper_prepare(0);
            let queue = ASensorManager_createEventQueue(
                sm, looper, 0, None, ptr::null_mut(),
            );
            ASensorEventQueue_enableSensor(queue, sensor);
            ASensorEventQueue_setEventRate(queue, sensor, 16_000);

            let mut event: ASensorEvent = mem::zeroed();
            loop {
                while ASensorEventQueue_getEvents(queue, &mut event, 1) > 0 {
                    let lux = event.__bindgen_anon_1.__bindgen_anon_1.data[0];
                    self.latest_lux_bits.store(lux.to_bits(), Ordering::Relaxed);
                }
                thread::sleep(Duration::from_millis(16));
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
    if !path_exists(BACKLIGHT_NODE) {
        return;
    }
    if !path_exists(DPMS_NODE) {
        return;
    }

    // aod 没开，等它开再继续
    if !read_aod_enabled() {
        watch_pref_loop(|enabled| !enabled);
    }

    let touch_node = match find_touch_node() {
        Some(n) => n,
        None => return,
    };

    let svc = Arc::new(AodService::new());

    {
        let s = svc.clone();
        thread::Builder::new()
            .name("aod-lux".into())
            .spawn(move || s.run_lux_sensor())
            .unwrap();
    }
    {
        let s = svc.clone();
        thread::Builder::new()
            .name("aod-timeout".into())
            .spawn(move || s.handle_timeout())
            .unwrap();
    }
    {
        let s = svc.clone();
        let node = touch_node.clone();
        thread::Builder::new()
            .name("aod-input".into())
            .spawn(move || s.monitor_input(&node))
            .unwrap();
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
        .file_name().and_then(|n| n.to_str()).unwrap_or("");

    let dir_c = match CString::new(pref_dir) {
        Ok(c) => c,
        Err(_) => return poll_pref_loop(on_change),
    };

    let fd = unsafe { libc::inotify_init1(libc::IN_CLOEXEC) };
    if fd < 0 { return poll_pref_loop(on_change); }

    let wd = unsafe {
        libc::inotify_add_watch(fd, dir_c.as_ptr(),
            libc::IN_CLOSE_WRITE | libc::IN_MOVED_TO)
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
        if n <= 0 { continue; }
        let n = n as usize;
        let mut off = 0usize;
        while off + INOTIFY_HDR <= n {
            let name_len = u32::from_ne_bytes(
                buf[off + 12..off + 16].try_into().unwrap()
            ) as usize;
            let next = off + INOTIFY_HDR + name_len;
            if name_len > 0 && next <= n {
                let nb = &buf[off + INOTIFY_HDR..next];
                let nul = nb.iter().position(|&b| b == 0).unwrap_or(nb.len());
                let name = std::str::from_utf8(&nb[..nul]).unwrap_or("");
                if name == target_file && !on_change(read_aod_enabled()) {
                    break 'outer;
                }
            }
            off = if next > off { next } else { off + INOTIFY_HDR };
        }
    }
    unsafe { libc::close(fd) };
}

fn poll_pref_loop(mut on_change: impl FnMut(bool) -> bool) {
    loop {
        if !on_change(read_aod_enabled()) { return; }
        thread::sleep(Duration::from_secs(5));
    }
}

fn find_touch_node() -> Option<String> {
    let touch_keywords = [
        "fts", "goodix", "synaptics", "novatek", "xiaomi", "touch", "sec", "gt9"
    ];

    for entry in fs::read_dir("/sys/class/input/").ok()?.filter_map(|e| e.ok()) {
        let path  = entry.path();
        let fname = path.file_name()?.to_str()?;
        if !fname.starts_with("event") { continue; }

        if let Ok(data) = fs::read_to_string(path.join("device/name")) {
            let name_lower = data.trim().to_ascii_lowercase();
            if touch_keywords.iter().any(|&kw| name_lower.contains(kw)) {
                return Some(format!("/dev/input/{}", fname));
            }
        }
    }
    None
}

fn is_dpms_on() -> bool {
    fs::read_to_string(DPMS_NODE)
        .map_or(false, |s| s.trim() == "On")
}

fn is_fod_pressed() -> bool {
    fs::read_to_string(FOD_STATUS_NODE)
        .map_or(false, |s| s.trim() == "1")
}

fn get_current_brightness() -> i32 {
    fs::read_to_string(BACKLIGHT_NODE)
        .ok().and_then(|s| s.trim().parse().ok()).unwrap_or(0)
}

fn path_exists(path: &str) -> bool {
    Path::new(path).exists()
}

fn write_node(path: &str, val: &str) {
    let _ = fs::write(path, val.as_bytes());
}

fn fade_brightness(from: i32, to: i32) {
    if from == to { return; }
    let step: i32 = if from < to { 1 } else { -1 };
    let mut cur = from;
    loop {
        write_node(BACKLIGHT_NODE, &cur.to_string());
        thread::sleep(Duration::from_micros(FADE_DELAY_US));
        if cur == to { break; }
        cur += step;
    }
}

fn force_deep_sleep() {
    for lock in ["PowerManagerService.noSuspend", "SensorsHAL_WAKEUP"] {
        write_node(WAKE_UNLOCK_NODE, lock);
    }
}
