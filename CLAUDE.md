# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is firmware for the **Seeed Grove Vision AI Module V2**, based on the Himax WiseEye2 (HX6538) chip with an ARM Cortex-M55 processor. The project runs TensorFlow Lite Micro (TFLM) models with optional Arm Ethos-U55 NPU acceleration.

## Build Commands

All build commands are run from the `EPII_CM55M_APP_S/` directory. Requires **ARM GNU Toolchain 13.2** (`arm-none-eabi-*`) on PATH.

```bash
cd EPII_CM55M_APP_S
make clean
make
```

**Output ELF:** `./obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf`

### Generate Flashable Image (step 2)

```bash
cd ../we2_image_gen_local/
cp ../EPII_CM55M_APP_S/obj_epii_evb_icv30_bdv10/gnu_epii_evb_WLCSP65/EPII_CM55M_gnu_epii_evb_WLCSP65_s.elf input_case1_secboot/
# Linux/Mac:
./we2_local_image_gen project_case1_blp_wlcsp.json
# Windows:
we2_local_image_gen project_case1_blp_wlcsp.json
```

**Output image:** `./output_case1_sec_wlcsp/output.img`

### Flash Firmware (via Python/xmodem)

```bash
# Install dependencies once:
pip install -r xmodem/requirements.txt

# Flash firmware + model (Windows):
python xmodem\xmodem_send.py --port=COM123 --baudrate=921600 --protocol=xmodem \
  --file=we2_image_gen_local\output_case1_sec_wlcsp\output.img \
  --model="model_zoo\tflm_yolov8_od\yolov8n_od_192_delete_transpose_0xB7B000.tflite 0xB7B000 0x00000"

# Flash firmware + model (Linux):
python3 xmodem/xmodem_send.py --port=/dev/ttyACM0 --baudrate=921600 --protocol=xmodem \
  --file=we2_image_gen_local/output_case1_sec_wlcsp/output.img \
  --model="model_zoo/tflm_yolov8_od/yolov8n_od_192_delete_transpose_0xB7B000.tflite 0xB7B000 0x00000"
```

After flashing, press the **reset** button on the device.

Serial monitor settings: **921600 baud, 8N1, no flow control** (TeraTerm on Windows, Minicom on Linux).

## Architecture

### Selecting a Scenario App

The active application is set by `APP_TYPE` in `EPII_CM55M_APP_S/makefile`:

```makefile
APP_TYPE = tflm_yolov8_od  # change this to select the app
```

Each scenario app lives in `EPII_CM55M_APP_S/app/scenario_app/<APP_TYPE>/`. The app-specific `.mk` file (e.g., `tflm_yolov8_od.mk`) overrides build settings like `LIB_SEL`, `CIS_SUPPORT_INAPP_MODEL`, linker script, and compiler defines.

Available apps: `allon_sensor_tflm`, `tflm_yolov8_od`, `tflm_yolov8_pose`, `tflm_yolov8_gender_cls`, `tflm_yolo11_od`, `tflm_fd_fm`, `tflm_peoplenet`, `tflm_mb_cls`, `pdm_record`, `kws_pdm_record`, `imu_read`, `allon_sensor_tflm_freertos`, `allon_sensor_tflm_cmsis_nn`, `allon_sensor_tflm_fatfs`, `allon_jpeg_encode`, `edge_impulse_firmware`, `ei_standalone_inferencing`, `ei_standalone_inferencing_camera`, `hello_world_cmsis_dsp`, `hello_world_cmsis_cv`.

### Camera Selection

Inside each scenario app's `.mk` file:

```makefile
CIS_SUPPORT_INAPP_MODEL = cis_ov5647    # default (OV5647 / OV5647 Pi Camera)
#CIS_SUPPORT_INAPP_MODEL = cis_imx219   # IMX219 Raspberry Pi Camera
#CIS_SUPPORT_INAPP_MODEL = cis_imx477   # IMX477
#CIS_SUPPORT_INAPP_MODEL = cis_hm0360   # HM0360
```

### Model Flash Address

Each scenario app's `common_config.h` defines the flash address where the `.tflite` model is stored (must be 4KB-aligned, after `0x200000` which is reserved for firmware):

```c
#define YOLOV8_OBJECT_DETECTION_FLASH_ADDR  0x3AB7B000
```

The `--model` argument to `xmodem_send.py` must match this address.

### Key Build Variables (makefile)

| Variable | Purpose |
|---|---|
| `APP_TYPE` | Selects scenario app |
| `LIB_CMSIS_NN_ENALBE` | Enable CMSIS-NN library (1=on) |
| `LIB_CMSIS_NN_VERSION` | `7_0_0` = newer tflmtag2412_u55tag2411, blank = tflmtag2209_u55tag2205 |
| `TOOLCHAIN` | `gnu` (default) or `arm` |
| `OS_SEL` | `freertos` or blank (bare-metal) |
| `TRUSTZONE` | `y` or `n` |

### Directory Structure

```
EPII_CM55M_APP_S/
  app/
    scenario_app/<app_name>/   # One folder per application
      <app>.c / <app>.h        # Main app entry point and datapath init
      cvapp*.cpp               # CV inference logic (TFLM model run)
      common_config.h          # Flash addresses, debug flags
      drv_user_defined.mk      # Driver selection for this app
      <app>.mk                 # Build config (LIB_SEL, CIS model, defines)
      <app>.ld                 # Linker script
  drivers/                     # Low-level peripheral drivers (hx_drv_*)
  library/
    inference/
      tflmtag2412_u55tag2411/  # TensorFlow Lite Micro + Ethos-U55 (newer)
      tflmtag2209_u55tag2205/  # TensorFlow Lite Micro + Ethos-U55 (older)
    cmsis_nn/                  # CMSIS-NN kernels
    img_proc/                  # Image resize/format conversion (Helium-optimized)
    sensordp/                  # Sensor datapath library
    pwrmgmt/                   # Power management
    spi_ptl/                   # SPI protocol for sending results to host
  external/cis/                # Camera sensor drivers (hm0360, imx219, ov5647, etc.)
  os/rtos2_freertos/           # FreeRTOS 10.5.1
  board/                       # Board-level init (epii_evb)
  options/                     # Build system include files
we2_image_gen_local/           # Firmware image packaging tool
model_zoo/                     # Pre-built Vela-compiled .tflite models
xmodem/                        # Python flash utility
swd_debugging/                 # VS Code SWD debug configs + pyOCD
```

### Typical Scenario App Data Flow

1. **App init** (`<app>.c`): initializes camera, SPI, power management, registers datapath callback
2. **Datapath callback** fires on each captured frame (via `sensor_dp_lib`)
3. **CV inference** (`cvapp*.cpp`): resizes image → runs TFLM interpreter with Ethos-U55 → post-processes outputs
4. **Send result** (`send_result.cpp`): serializes detection/classification results as JSON over UART at 921600 baud; also sends JPEG image frame over SPI

### Adding a New Scenario App

Copy an existing app folder, rename it, update its `.mk` and `common_config.h`, then set `APP_TYPE` in the top-level `makefile`.

### CMSIS-NN Usage

To use CMSIS-NN (non-Vela model path), set in `makefile`:
```makefile
LIB_CMSIS_NN_ENALBE = 1
APP_TYPE = allon_sensor_tflm_cmsis_nn
```

### SWD Debugging (Linux + VS Code)

See `swd_debugging/README.md`. Uses pyOCD (`pyocd_hx` fork) with CMSIS-DAP probe. VS Code tasks in `EPII_CM55M_APP_S/.vscode/` require updating toolchain and source paths before use.
