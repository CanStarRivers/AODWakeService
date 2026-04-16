/*
Hyperfusion AOD Wake Service
License: Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International (CC BY-NC-ND 4.0)
You may use and share this code for personal, educational, or non-commercial purposes only.
Commercial use, selling, charging, modification, or redistribution of modified versions is strictly prohibited.
Full license: https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode.txt
*/

package main

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"sync"
	"syscall"
	"time"
	"unsafe"
)

const (
	PrefPath         = "/data/user_de/0/com.miui.aod/shared_prefs/com.miui.aod_preferences.xml"
	BacklightNode    = "/sys/class/backlight/panel0-backlight/brightness"
	WakeLockNode     = "/sys/power/wake_lock"
	WakeUnlockNode   = "/sys/power/wake_unlock"
	SuspendStateNode = "/sys/devices/virtual/touch/touch_dev/suspend_state"

	MyAodWakeLock    = "HyperfusionAodService_Lock"

	AODTimeout       = 10 * time.Second
	TargetBrightness = 200
	FadeDelayUs      = 2000

	EV_KEY        = 1
	GESTURE_KEY_1 = 354
	GESTURE_KEY_2 = 338
)

type InputEvent struct {
	Sec   int64
	Usec  int64
	Type  uint16
	Code  uint16
	Value int32
}

var (
	isScreenOn = false
	timerMutex sync.Mutex
	sleepTimer *time.Timer

	configMutex  sync.RWMutex
	isAodEnabled = false
)

func main() {
	if !pathExists(BacklightNode) {
		return
	}
	if !pathExists(SuspendStateNode) {
		return
	}

	go watchConfig()
	targetNode := findFtsTouchNode()
	if targetNode == "" {
		return
	}

	sleepTimer = time.NewTimer(AODTimeout)
	sleepTimer.Stop()
	go handleTimeout()

	go monitorInput(targetNode)
	select {}
}

func watchConfig() {
	updateConfigState()
	fd, err := syscall.InotifyInit()
	if err != nil {
		fallbackWatchConfig()
		return
	}
	prefDir := filepath.Dir(PrefPath)
	targetFile := filepath.Base(PrefPath)
	_, err = syscall.InotifyAddWatch(fd, prefDir, syscall.IN_CLOSE_WRITE|syscall.IN_MOVED_TO)
	if err != nil {
		fallbackWatchConfig()
		return
	}

	var buf [1024]byte
	for {
		n, err := syscall.Read(fd, buf[:])
		if err != nil {
			continue
		}
		offset := 0
		for offset <= n-syscall.SizeofInotifyEvent {
			event := (*syscall.InotifyEvent)(unsafe.Pointer(&buf[offset]))
			nameLen := uint32(event.Len)
			if nameLen > 0 {
				nameBytes := buf[offset+syscall.SizeofInotifyEvent : offset+syscall.SizeofInotifyEvent+int(nameLen)]
				name := string(bytes.TrimRight(nameBytes, "\x00"))
				if name == targetFile {
					updateConfigState()
				}
			}
			offset += syscall.SizeofInotifyEvent + int(nameLen)
		}
	}
}

func updateConfigState() {
	data, err := os.ReadFile(PrefPath)
	enabled := false
	if err == nil {
		content := string(data)
		lines := strings.Split(content, "\n")
		for _, line := range lines {
			if strings.Contains(line, "aod_temporary_style") && strings.Contains(line, "\"true\"") {
				enabled = true
				break
			}
		}
	}
	configMutex.Lock()
	isAodEnabled = enabled
	configMutex.Unlock()
}

func fallbackWatchConfig() {
	for {
		updateConfigState()
		time.Sleep(5 * time.Second)
	}
}

func findFtsTouchNode() string {
	dirs, err := filepath.Glob("/sys/class/input/event*")
	if err != nil {
		return ""
	}
	for _, dir := range dirs {
		namePath := filepath.Join(dir, "device", "name")
		data, err := os.ReadFile(namePath)
		if err != nil {
			continue
		}
		devName := strings.ToLower(strings.TrimSpace(string(data)))
		if strings.Contains(devName, "fts") {
			base := filepath.Base(dir)
			return "/dev/input/" + base
		}
	}
	return ""
}

func monitorInput(devicePath string) {
	file, err := os.Open(devicePath)
	if err != nil {
		return
	}
	defer file.Close()
	var event InputEvent
	for {
		err := binary.Read(file, binary.LittleEndian, &event)
		if err != nil {
			continue
		}
		if event.Type == EV_KEY && (event.Code == GESTURE_KEY_1 || event.Code == GESTURE_KEY_2) && event.Value == 1 {
			triggerWakeup()
		}
	}
}

func triggerWakeup() {
	configMutex.RLock()
	enabled := isAodEnabled
	configMutex.RUnlock()
	if !enabled {
		return
	}
	if getSuspendState() == 0 {
		return
	}
	timerMutex.Lock()
	defer timerMutex.Unlock()
	sleepTimer.Reset(AODTimeout)
	if !isScreenOn {
		writeNode(WakeLockNode, MyAodWakeLock)
		isScreenOn = true
		fadeBrightness(0, TargetBrightness)
	}
}

func handleTimeout() {
	for {
		<-sleepTimer.C
		timerMutex.Lock()
		if isScreenOn {
			if getSuspendState() == 0 {
				isScreenOn = false
				writeNode(WakeUnlockNode, MyAodWakeLock)
			} else {
				fadeBrightness(TargetBrightness, 0)
				writeNode(WakeUnlockNode, MyAodWakeLock)
				forceDeepSleep()
				isScreenOn = false
			}
		}
		timerMutex.Unlock()
	}
}

func fadeBrightness(from, to int) {
	if from == to {
		return
	}
	step := 1
	if from > to {
		step = -1
	}
	for i := from; i != to+step; i += step {
		writeNode(BacklightNode, fmt.Sprintf("%d", i))
		time.Sleep(time.Duration(FadeDelayUs) * time.Microsecond)
	}
}

func forceDeepSleep() {
	locks := []string{"PowerManagerService.noSuspend", "SensorsHAL_WAKEUP"}
	for _, lock := range locks {
		writeNode(WakeUnlockNode, lock)
	}
}

func pathExists(path string) bool {
	_, err := os.Stat(path)
	return err == nil || !os.IsNotExist(err)
}

func writeNode(path, val string) {
	os.WriteFile(path, []byte(val), 0644)
}

func getSuspendState() int {
	data, err := os.ReadFile(SuspendStateNode)
	if err != nil {
		return 0
	}
	val := strings.TrimSpace(string(data))
	state, _ := strconv.Atoi(val)
	return state
}
