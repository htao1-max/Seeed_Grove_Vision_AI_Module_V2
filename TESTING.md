# tflm_yolov8_od_sdlog — Testing Guide

## How the Program Works

### Boot Sequence (what happens on power-up / reset)

1. **main()** calls `tflm_yolov8_od_sdlog_app()`
2. **Pin mux** configured for SPI (camera + SD card share SPI bus on PB2-PB5)
3. **Flash/XIP** enabled to load the YOLOv8n model from flash address `0x3AB7B000`
4. **Memory manager** initialized for inference tensor allocation
5. **SD card mounted** via `sdlog_session_init()`:
   - Mounts FAT filesystem on SD card
   - Creates `SESSION_XXXX/` directory (auto-incrementing: 0000, 0001, 0002...)
   - Creates `SESSION_XXXX/ALL/` and `SESSION_XXXX/DETECT/` subdirectories
   - Opens `SESSION_XXXX/session.log` for text logging
6. **I2C slave registered** via `i2c_cmd_init()`:
   - Registers customer callback on I2C slave address **0x62**
   - Listens for feature=0x80, cmd=0x01 (start recording)
7. **YOLOv8n model loaded** from flash into TFLM interpreter with Ethos-U55 NPU
8. **Camera + datapath started** — begins capturing 640x480 frames continuously
9. **Event loop started** — dispatches frame-ready and I2C events

### Per-Frame Loop (runs continuously after boot)

Every captured frame triggers `dp_app_cv_yolov8n_ob_eventhdl_cb()`:

1. JPEG image captured from camera → stored in SRAM
2. YOLOv8n inference runs (image resized to 192x192 → NPU inference → NMS post-processing)
3. JSON detection results sent over UART (always, regardless of recording state)
4. **If `g_recording_active == 1`** (set by I2C command):
   - Save JPEG to `SESSION_XXXX/ALL/img_XXXX.jpg` (every frame)
   - Log `[ALL] img_XXXX.jpg (frame N)` to session.log
   - If any detection confidence >= `g_detect_threshold` (default 0.50):
     - Save JPEG to `SESSION_XXXX/DETECT/det_XXXX.jpg`
     - Log `[DETECT] det_XXXX.jpg score=X.XX class=N (frame N)` to session.log
5. **If `g_recording_active == 0`**: inference still runs, UART JSON still outputs, but nothing is saved to SD

### I2C Command Protocol

The STM32 master sends an I2C write to slave address **0x62** using the `i2ccomm` packet format:

| Field | Value |
|---|---|
| Feature byte | `0x80` (customer range) |
| Command byte | `0x01` (start recording) |
| Payload[0] (optional) | Threshold override: 0-100 (maps to 0.00-1.00) |

When received:
- `g_recording_active` is set to `1`
- If payload byte present: `g_detect_threshold = payload[0] / 100.0`
- Logged to UART: `[I2C_CMD] Recording started (threshold=X.XX)`
- Logged to SD: `[I2C] Recording started (threshold=X.XX)`

**Note:** There is currently NO stop-recording command — once started, recording continues until reset.

---

## Hardware Setup

### Required Connections

| Signal | Grove Vision AI V2 Pin | Notes |
|---|---|---|
| Camera | CSI connector | OV5647 (default) or IMX219 |
| SD card MOSI | PB2 | |
| SD card MISO | PB3 | |
| SD card CLK | PB4 | |
| SD card CS | PB5 (GPIO16) | |
| I2C SDA | I2C slave bus 0 | To STM32 master |
| I2C SCL | I2C slave bus 0 | To STM32 master |
| UART TX | USB-C / debug header | For serial monitor |

### SD Card Requirements

- FAT32 formatted
- Insert BEFORE powering on (mount happens at boot)
- Must be accessible via SPI on PB2-PB5

---

## Test Procedures

### Test 1: Boot & SD Card Initialization

**Goal:** Verify the firmware boots, mounts SD, and creates a session directory.

**Steps:**
1. Insert a FAT32-formatted SD card into the module
2. Connect USB-C to PC for serial monitor
3. Open PuTTY / TeraTerm at **921600 baud, 8N1, no flow control**
4. Power on or press reset

**Expected UART output (in order):**
```
wakeup_event=0x0,WakeupEvt1=0x0, freq=...
[SDLOG] SD card mounted
[SDLOG] Session SESSION_0000 initialised
[I2C_CMD] Customer I2C handler registered (addr=0x62, feature=0x80)
YOLOv8n object detection (sdlog)
```
Then continuous JSON detection output (this is normal — it's the UART_SEND_ALOGO_RESEULT stream).

**Verify on SD card:** Power off, remove SD card, check on PC:
- Directory `SESSION_0000/` exists
- Subdirectories `SESSION_0000/ALL/` and `SESSION_0000/DETECT/` exist (both empty)
- File `SESSION_0000/session.log` exists with: `[BOOT] Session SESSION_0000 started`

**If SD mount fails:**
- UART shows: `[SDLOG] f_mount failed: X — SD logging disabled`
- Check: SD card inserted? FAT32? Wiring on PB2-PB5 correct?

---

### Test 2: Session Auto-Increment

**Goal:** Verify each reboot creates a new session folder.

**Steps:**
1. With SD card containing `SESSION_0000/` from Test 1, reset the module
2. Check UART for: `[SDLOG] Session SESSION_0001 initialised`
3. Reset again → `SESSION_0002`, etc.

**Verify on SD card:** Multiple `SESSION_XXXX` folders exist with incrementing numbers.

---

### Test 3: I2C Start Recording (from STM32)

**Goal:** Verify the STM32 can trigger recording via I2C.

**Prerequisites:**
- STM32 connected to Grove AI V2 via I2C (SDA/SCL)
- STM32 firmware configured as I2C master

**STM32 Side — Send I2C Command:**

You need to send an `i2ccomm`-formatted packet to slave address **0x62**. The packet format used by the Himax I2C comm library is:

```
Byte 0: Feature (0x80)
Byte 1: Command (0x01)
Byte 2: Payload length MSB (0x00)
Byte 3: Payload length LSB (0x01 for 1 byte, or 0x00 for no payload)
Byte 4: Payload[0] = threshold (e.g., 50 for 0.50) — optional
Byte N: Checksum (per i2ccomm protocol)
```

> **Important:** The exact packet format depends on the `i2ccomm` library's `I2CFMT_*_OFFSET` definitions. Review `evt_i2ccomm.h` for the exact byte layout and checksum algorithm.

**Expected UART output on Grove AI V2:**
```
[I2C_CMD] Recording started (threshold=0.50)
```

**If checksum fails:**
```
[I2C_CMD] checksum error, ignoring
```

---

### Test 4: SD Card Recording — ALL Images

**Goal:** Verify every frame is saved to `ALL/` while recording is active.

**Steps:**
1. Boot with SD card, wait for init messages
2. Send I2C start-recording command from STM32
3. Let it run for ~10 seconds (should capture several frames)
4. Power off the module (this is the only way to stop — no stop command exists)
5. Remove SD card and inspect on PC

**Expected on SD card:**
```
SESSION_XXXX/
  ALL/
    img_0000.jpg
    img_0001.jpg
    img_0002.jpg
    ...
  DETECT/
    (may have det_XXXX.jpg files if objects were detected)
  session.log
```

**Verify:**
- `ALL/` contains sequential JPEG images (`img_0000.jpg`, `img_0001.jpg`, ...)
- Each JPEG is a valid image (open them on PC — should show camera captures)
- `session.log` contains `[ALL]` entries for each frame

---

### Test 5: SD Card Recording — Detection Filtering

**Goal:** Verify only frames with detections above threshold are saved to `DETECT/`.

**Steps:**
1. Boot, start recording via I2C (with default threshold 0.50 or send a specific threshold)
2. Point camera at a known COCO object (person, car, cat, etc.) for some frames
3. Point camera at a blank wall / ceiling for some frames
4. Power off, inspect SD card

**Expected:**
- `ALL/` has images for ALL frames (both with and without objects)
- `DETECT/` has images ONLY for frames where YOLOv8n detected something with confidence >= threshold
- `session.log` has `[ALL]` lines for every frame, but `[DETECT]` lines only for detection frames

**Example session.log:**
```
[BOOT] Session SESSION_0000 started
[I2C] Recording started (threshold=0.50)
[ALL] img_0000.jpg (frame 0)
[ALL] img_0001.jpg (frame 1)
[DETECT] det_0000.jpg score=0.78 class=0 (frame 1)
[ALL] img_0002.jpg (frame 2)
[ALL] img_0003.jpg (frame 3)
[DETECT] det_0001.jpg score=0.65 class=0 (frame 3)
[ALL] img_0004.jpg (frame 4)
```

---

### Test 6: Custom Detection Threshold via I2C

**Goal:** Verify the threshold override works.

**Steps:**
1. Send I2C start-recording with payload byte = `25` (→ threshold 0.25, very sensitive)
2. UART should show: `[I2C_CMD] Recording started (threshold=0.25)`
3. Expect more frames in `DETECT/` since the threshold is lower
4. Repeat with payload byte = `90` (→ threshold 0.90, very strict)
5. Expect fewer or no frames in `DETECT/`

---

### Test 7: No Recording Without I2C Trigger

**Goal:** Verify that no SD card writes happen until I2C command is received.

**Steps:**
1. Boot with SD card (do NOT send any I2C command)
2. Let it run for 30+ seconds — camera is running, UART JSON is streaming
3. Power off, inspect SD card

**Expected:**
- `SESSION_XXXX/ALL/` is empty (no images)
- `SESSION_XXXX/DETECT/` is empty (no images)
- `SESSION_XXXX/session.log` contains only: `[BOOT] Session SESSION_XXXX started`

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| No UART output at all | Wrong baud rate or port | Use 921600, 8N1, no flow control |
| `f_mount failed` | SD card not inserted, not FAT32, or wiring issue | Check SD card format and PB2-PB5 wiring |
| `CIS Init fail` | Camera not connected or wrong sensor selected | Check CSI cable; verify `CIS_SUPPORT_INAPP_MODEL` in `.mk` |
| `checksum error` from I2C | Wrong I2C packet format | Match the `i2ccomm` library's packet format exactly |
| No I2C response | I2C address mismatch or `evt_i2ccomm` not initialized | Confirm slave address 0x62; check `event_handler_init()` is called |
| Images are blank/corrupt | JPEG buffer not valid or D-cache issue | Check `jpeg_sz` in UART debug; ensure `SCB_InvalidateDCache_by_Addr` is working |
| `DETECT/` always empty | Threshold too high or model not detecting | Lower threshold to 0.10 via I2C; point camera at a person (COCO class 0) |
| SD card fills up fast | Every frame saved to ALL/ | Recording has no auto-stop — power cycle to stop |

## Known Limitations

1. **No stop-recording command** — once `g_recording_active = 1`, it stays on until reset
2. **No frame rate control** — every captured frame is saved during recording (can fill SD quickly)
3. **Power-loss risk** — `f_sync()` is called after each log write, but JPEG writes could be corrupted if power is lost mid-write
4. **SPI bus sharing** — SD card (PB2-PB5) and camera both use SPI; the pin mux is reconfigured at boot but conflicts are possible if the SD card path overlaps with camera SPI
5. **Session folder limit** — scans sequentially from SESSION_0000; will slow down if hundreds of sessions exist
