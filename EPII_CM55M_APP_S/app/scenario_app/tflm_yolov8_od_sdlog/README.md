# tflm_yolov8_od_sdlog — YOLOv8n object detection with I2C-triggered SD-card logging

Fork of Himax's `tflm_yolov8_od` scenario app. Runs YOLOv8n continuously on
the Grove Vision AI Module V2 (Himax WE2). Frames are **only** persisted to
the on-board microSD once an external I2C master (an STM32G431, in our
setup) sends a "start recording" i2ccomm packet. Every frame is saved under
`SESSION_XXXX/ALL/`, and any frame with a detection above the configured
threshold is also saved under `SESSION_XXXX/DETECT/` with a sidecar `.txt`
holding the class / confidence / bbox of every detection in that frame.

Paired with the STM32 trigger firmware in the `i2cScan` repo.

## What's different from stock `tflm_yolov8_od`

| Added | File(s) |
| ----- | ------- |
| I2C customer-cmd handler (`feature=0x80, cmd=0x01`) to arm recording | `i2c_cmd.c`, `i2c_cmd.h` |
| microSD session logger (auto-numbers `SESSION_XXXX/`, ALL/DETECT split, `session.log`) | `sdlog.c`, `sdlog.h` |
| Per-frame save + per-detection `det_XXXX.txt` annotation | `tflm_yolov8_od_sdlog.c` (see `EVT_INDEX_XDMA_FRAME_READY` handler) |
| Disabled `UART_SEND_ALOGO_RESEULT` (freed ~300 ms/frame of UART blocking) | `tflm_yolov8_od_sdlog.mk` |
| D-cache clean+invalidate before SD writes (was invalidate-only — lost DMA data) | `sdlog.c :: sdlog_write_image()` |
| Init order: `i2c_cmd_init → cv_yolov8n_ob_init → app_start_state → sdlog_session_init` (sensor init clobbered SPI pinmux otherwise) | `tflm_yolov8_od_sdlog.c` |

## Boot flow

1. `i2c_cmd_init()` — register the customer callback on DW I2C slave 0.
   The underlying `evt_i2ccomm_init()` is called later from
   `event_handler_init()` / `app_start_state()`.
2. `cv_yolov8n_ob_init()` — load the YOLOv8n tflite model from flash
   (default addr `0x3AB7B000`, see `common_config.h`).
3. `app_start_state()` — start sensor + datapath, begins firing frame events.
4. `sdlog_session_init()` — configure SPI pinmux (PB2 MOSI, PB3 MISO,
   PB4 SCLK, PB5 CS via GPIO16), mount FatFs, create the next
   `SESSION_XXXX/ALL/` and `SESSION_XXXX/DETECT/`, open `session.log`.

The order matters: the SPI pinmux must be applied **after** sensor init or
it gets overwritten and SD writes silently fail.

## Runtime

Every frame ready event:

- Run `cv_yolov8n_ob_run()` (YOLO inference + JPEG encode).
- If `g_recording_active == 0` → do nothing, just drop the frame.
- If `g_recording_active == 1`:
  - Save raw JPEG to `SESSION_XXXX/ALL/img_%04u.jpg`.
  - Walk all YOLO results. Build an annotation buffer with one line per
    detection. If any detection's confidence ≥ `g_detect_threshold`, also
    save the JPEG as `SESSION_XXXX/DETECT/det_%04u.jpg` and write the
    annotation buffer to `SESSION_XXXX/DETECT/det_%04u.txt`.
  - Append a summary line (and per-object details) to `session.log`.

There is **no** software FPS limiter. Effective capture rate ≈ sensor FPS −
SD write overhead − YOLO inference latency. Long SD write or
`session.log` flush = dropped frames.

## Annotation file format (`det_XXXX.txt`)

One line per detection in the matching `det_XXXX.jpg`:

```
<class_idx> <confidence> <x> <y> <width> <height>
```

- `class_idx` — integer class id from the tflite model.
- `confidence` — float, e.g. `0.87`.
- `x y width height` — pixel coordinates in the raw **320×240** image
  (scaled from the 192×192 model output inside `yolo_postprocessing.cc`).

Visualizer lives in `../../../../../i2cScan/plot_detections.py` (Pillow +
matplotlib).

## i2ccomm protocol (customer range)

The STM32 sends a single packet over I2C. Framing is the stock Himax
`i2c_comm` customer frame:

```
Byte 0     Feature              (0x80 = recorder, customer range)
Byte 1     Command              (0x01 = start recording)
Byte 2     Payload length LSB
Byte 3     Payload length MSB
Byte 4..   Payload              (optional, here: 1 byte threshold 0..100)
Byte N     CRC-16 CCITT LSB     poly 0x1021, init 0xFFFF, over bytes 0..N-1
Byte N+1   CRC-16 CCITT MSB
```

`i2c_customer_handler()` dumps the raw bytes, validates the checksum
(currently with a debug-only bypass — look for the `TODO` / bypass comment
before merging cleanly), then if `feature==0x80 && cmd==0x01`:

- If `payload_len >= 1` and `payload[0] <= 100`: `g_detect_threshold = payload[0] / 100.0f`.
- `g_recording_active = 1`.
- `xprintf` a "Recording started" banner; append the same to `session.log`.

The slave is configured as address `0x62` in firmware but shows up on the
physical bus at `0x28` — a known quirk of the DesignWare I2C slave driver's
address mapping. The STM32 talks to `0x28` and everything works; do not
change the firmware-side address.

## Public C API

`i2c_cmd.h`:
```c
extern volatile uint8_t g_recording_active;
extern volatile float   g_detect_threshold;    /* init: DETECT_CONF_THRESHOLD */
void i2c_cmd_init(void);                       /* register customer callback */
```

`sdlog.h`:
```c
void sdlog_session_init(void);
void sdlog_save_all   (uint32_t addr, uint32_t sz, const char *fname);
int  sdlog_save_detect(uint32_t addr, uint32_t sz, const char *fname);    /* 1=ok, 0=fail */
int  sdlog_save_detect_txt(const char *fname, const char *content);       /* 1=ok, 0=fail */
void sdlog_write(const char *fmt, ...);        /* printf → session.log, f_sync each call */
```

`common_config.h` knobs:
```c
#define DETECT_CONF_THRESHOLD              0.50f
#define YOLOV8_OBJECT_DETECTION_FLASH_ADDR 0x3AB7B000
```

## SD-card layout

```
SESSION_0000/
  ALL/
    img_0000.jpg
    img_0001.jpg
    ...
  DETECT/
    det_0000.jpg
    det_0000.txt
    det_0001.jpg
    det_0001.txt
    ...
  session.log
SESSION_0001/
  ...
```

Each power cycle allocates a new `SESSION_XXXX` — `sdlog_session_init()`
scans for the next unused index.

## Camera / resolution notes

The datapath is initialized at **`APP_DP_RES_RGB640x480_INP_SUBSAMPLE_2X`**,
which produces a 320×240 RGB frame (~230 KB buffer). Stepping up to
`_SUBSAMPLE_1X` (true 640×480 RGB, ~921 KB) **crashes with BusFault** on
the WE2's ~512 KB SRAM when YOLO is also resident. YUV 640×480 (~460 KB) is
a theoretical compromise but untested. See `OV5647_Camera_Controls.md` in
the repo root for the knobs available via register writes on the camera
itself.

## Build

From the repo root:

```bash
cd EPII_CM55M_APP_S
# ensure APP_TYPE = tflm_yolov8_od_sdlog is set in makefile
make clean && make
cd ../we2_image_gen_local
cp ../EPII_CM55M_APP_S/obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf \
   input_case1_secboot/
./we2_local_image_gen project_case1_blp_wlcsp.json    # → output_case1_sec_wlcsp/output.img
```

Flash the firmware + the bundled YOLOv8n tflite over XMODEM:

```bash
python xmodem/xmodem_send.py \
  --port=COMxx --baudrate=921600 --protocol=xmodem \
  --file=we2_image_gen_local/output_case1_sec_wlcsp/output.img \
  --model="model_zoo/tflm_yolov8_od/yolov8n_od_192_delete_transpose_0xB7B000.tflite 0xB7B000 0x00000"
```

## Custom 2-class model (BlueRing / RedRing)

The bundled tflite is Himax's stock 80-class COCO YOLOv8n. To flash the
custom 2-class model:

1. Train YOLOv8n (192×192, 2 classes) on Ultralytics Hub → download `exp.pt`.
2. Re-export ONNX with **`ultralytics==8.0.173` + `torch==2.0.1`**
   (newer versions produce a graph `onnx2tf`'s `-rtpo` cannot consume).
3. Build `replace_192_2cls_transpose_op.json` — copy from the Himax 80-class
   example, change every `144` to `66` (= 64 bbox regression + 2 class logits).
   Confirm node names in Netron: `/model.22/Reshape{,_1,_2}`,
   `/model.22/Concat_3`, `/model.22/dfl/Reshape`, `/model.22/dfl/Transpose`.
4. `onnx2tf -i exp-2.onnx -prf replace_192_2cls_transpose_op.json -rtpo`
   (Ubuntu 20.04 host, Python 3.10 venv — Windows not supported).
5. `vela --accelerator-config ethos-u55-64 --config himax_vela.ini ...`
   and verify the report shows `Total SRAM used < 1 MB`.
6. Flash via XMODEM as above, pointing `--model=` at the vela output.

## Expected console output (921600 baud)

```
[I2C_CMD] Handler registered (addr=0x28)
YOLOv8n object detection (sdlog)
[SDLOG] SD card mounted
[SDLOG] Session SESSION_0000 initialised
  ... frames processing, g_recording_active=0, nothing saved ...
****************************************************
***  [I2C_CMD] Recording started (thresh=0.50)  ***
****************************************************
[SDLOG] First frame: jpeg_addr=0x340XXXXX jpeg_sz=XXXXX
[ALL] img_0000.jpg (frame 0)
[DETECT] det_0000.jpg (2 objects, frame 12)
  obj[0] class=0 conf=0.87 bbox=(104,72,48,52)
  obj[1] class=1 conf=0.63 bbox=(210,110,44,46)
...
```
