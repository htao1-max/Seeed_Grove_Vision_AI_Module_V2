# OV5647 Camera Controls — Grove AI Vision Module V2

All settings are configured via I2C register writes in the sensor init table:
`tflm_yolov8_od_sdlog/cis_sensor/cis_ov5647/OV5647_mipi_2lane_640x480.i`

Each entry follows the format:
```c
{HX_CIS_I2C_Action_W, <register_address>, <value>}
```

---

## 1. Exposure

### Auto Exposure (AEC) — On by Default

| Register | Current | Description |
|----------|---------|-------------|
| `0x3503` | `0x00` | AEC/AGC mode. `0x00` = fully auto. `0x01` = manual exposure only. `0x03` = manual exposure + manual AGC gain. `0x07` = manual exposure + manual gain + manual VTS |

### AEC Target Brightness

Controls how bright auto exposure tries to make the image.

| Register | Current | Description |
|----------|---------|-------------|
| `0x3a0f` | `0x58` | AEC stable range high (upper brightness target) |
| `0x3a10` | `0x50` | AEC stable range low (lower brightness target) |
| `0x3a1b` | `0x58` | AEC fast-zone high |
| `0x3a1e` | `0x50` | AEC fast-zone low |
| `0x3a11` | `0x60` | High limit of AEC step |
| `0x3a1f` | `0x28` | Low limit of AEC step |

**To make image brighter:** increase `0x3a0f` and `0x3a10` (e.g., `0x78` / `0x70`)
**To make image darker:** decrease them (e.g., `0x38` / `0x30`)
Keep `0x3a1b` = `0x3a0f` and `0x3a1e` = `0x3a10`.

### AEC Banding (Anti-Flicker)

Prevents banding under artificial lighting (50Hz or 60Hz).

| Register | Current | Description |
|----------|---------|-------------|
| `0x3a08` | `0x01` | 50Hz banding step [15:8] |
| `0x3a09` | `0x2e` | 50Hz banding step [7:0] |
| `0x3a0a` | `0x00` | 60Hz banding step [15:8] |
| `0x3a0b` | `0xfb` | 60Hz banding step [7:0] |
| `0x3a0d` | `0x02` | 50Hz max banding (max number of steps) |
| `0x3a0e` | `0x01` | 60Hz max banding |
| `0x3c01` | `0x80` | Band select: bit 7 = 1 → 50Hz. Set to `0x00` for 60Hz |

### Manual Exposure (when `0x3503` bit 0 = 1)

| Register | Description |
|----------|-------------|
| `0x3500` | Exposure [19:16] |
| `0x3501` | Exposure [15:8] |
| `0x3502` | Exposure [7:0] — units of 1/16 line |

Longer exposure = brighter image but more motion blur. Max value depends on VTS (frame height + blanking).

Example — add to init table:
```c
{HX_CIS_I2C_Action_W, 0x3503, 0x07},  // manual mode
{HX_CIS_I2C_Action_W, 0x3500, 0x00},
{HX_CIS_I2C_Action_W, 0x3501, 0x20},  // increase for longer exposure
{HX_CIS_I2C_Action_W, 0x3502, 0x00},
```

---

## 2. Gain (Sensitivity / ISO equivalent)

### Gain Ceiling (Auto Mode)

| Register | Current | Description |
|----------|---------|-------------|
| `0x3a18` | `0x00` | Max gain ceiling [9:8] |
| `0x3a19` | `0xf8` | Max gain ceiling [7:0]. `0xf8` = very high gain allowed (noisy). Lower to reduce noise, e.g., `0x40` |

### Manual Gain (when `0x3503` bit 1 = 1)

| Register | Description |
|----------|-------------|
| `0x350a` | AGC real gain [9:8] |
| `0x350b` | AGC real gain [7:0]. `0x10` = 1x, `0x20` = 2x, `0x40` = 4x |

---

## 3. Image Orientation (Mirror / Flip)

| Register | Current | Description |
|----------|---------|-------------|
| `0x3820` | `0x41` | Vertical flip. Bit 1: sensor flip, Bit 6: ISP flip |
| `0x3821` | `0x07` | Horizontal mirror. Bit 1: sensor mirror, Bit 6: ISP mirror |

| Desired Effect | `0x3820` | `0x3821` |
|----------------|----------|----------|
| Normal | `0x41` | `0x01` |
| Mirror only | `0x41` | `0x07` |
| Flip only | `0x47` | `0x01` |
| Mirror + Flip (180 rotate) | `0x47` | `0x07` |

**Current config:** mirrored horizontally, flipped vertically.

---

## 4. Frame Rate / Timing

### PLL Clock (determines sensor pixel clock)

| Register | Current | Description |
|----------|---------|-------------|
| `0x3034` | `0x1a` | PLL charge pump, MIPI bit mode. `0x1a` = 10-bit |
| `0x3035` | `0x21` | System clock divider. Bits [7:4] = sys divider, [3:0] = MIPI divider. `0x21` = slow. `0x11` = faster |
| `0x3036` | `0x46` | PLL multiplier. Higher = faster pixel clock = higher FPS. `0x46` = 70x |
| `0x303c` | `0x11` | PLL root divider |
| `0x3106` | `0xf5` | SRB clock |

### HTS / VTS (blanking — affects FPS directly)

| Register | Current | Description |
|----------|---------|-------------|
| `0x380c` | `0x07` | HTS (total horizontal size) [15:8] |
| `0x380d` | `0x3c` | HTS [7:0] → total = 1852 pixels/line |

VTS registers (`0x380e`, `0x380f`) are not in the current init table — they default to the sensor's reset value. Adding them lets you control FPS:

```
FPS = pixel_clock / (HTS × VTS)
```

Lower VTS = higher FPS (but reduces max exposure time).

---

## 5. Window / Crop (Output Region)

### Output Size

| Register | Current | Value | Description |
|----------|---------|-------|-------------|
| `0x3808` | `0x02` | | Output width [12:8] |
| `0x3809` | `0x80` | **640** | Output width [7:0] → 0x280 = 640 |
| `0x380a` | `0x01` | | Output height [11:8] |
| `0x380b` | `0xe0` | **480** | Output height [7:0] → 0x1E0 = 480 |

### ISP Input Window (crop from full sensor array)

| Register | Current | Description |
|----------|---------|-------------|
| `0x3800` | `0x00` | X start [12:8] |
| `0x3801` | `0x10` | X start [7:0] → 16 |
| `0x3802` | `0x00` | Y start [11:8] |
| `0x3803` | `0x00` | Y start [7:0] → 0 |
| `0x3804` | `0x0a` | X end [12:8] |
| `0x3805` | `0x2f` | X end [7:0] → 2607 |
| `0x3806` | `0x07` | Y end [11:8] |
| `0x3807` | `0x9f` | Y end [7:0] → 1951 |

Full sensor array is 2592x1944. The current config reads the full array and downscales to 640x480.

### Subsampling

| Register | Current | Description |
|----------|---------|-------------|
| `0x3814` | `0x35` | Horizontal subsample. `0x35` = skip 2 out of 3 (odd/even) |
| `0x3815` | `0x35` | Vertical subsample |

---

## 6. ISP Processing

| Register | Current | Description |
|----------|---------|-------------|
| `0x5000` | `0x06` | ISP control. Bit 2: black-level correction. Bit 1: white-pixel correction. Set `0x27` to also enable lens shading and auto-white-balance |
| `0x5003` | `0x08` | ISP misc control |
| `0x5a00` | `0x08` | Pattern/test control |

### Enable Additional ISP Features (add to init table)

```c
// Auto White Balance (AWB)
{HX_CIS_I2C_Action_W, 0x5001, 0x01},  // enable AWB
{HX_CIS_I2C_Action_W, 0x5180, 0xff},  // AWB B block
{HX_CIS_I2C_Action_W, 0x5181, 0xf2},  // AWB control
{HX_CIS_I2C_Action_W, 0x5182, 0x00},  // AWB control 2

// Manual White Balance gains (when AWB is off)
// 0x5186/87 = R gain, 0x5188/89 = G gain, 0x518a/8b = B gain
{HX_CIS_I2C_Action_W, 0x5186, 0x04},  // R gain [11:8]
{HX_CIS_I2C_Action_W, 0x5187, 0x00},  // R gain [7:0]
{HX_CIS_I2C_Action_W, 0x518a, 0x04},  // B gain [11:8]
{HX_CIS_I2C_Action_W, 0x518b, 0x00},  // B gain [7:0]

// Lens Shading Correction
{HX_CIS_I2C_Action_W, 0x5000, 0x26},  // bit 5: enable lens shading

// Sharpness / De-noise (limited on OV5647)
{HX_CIS_I2C_Action_W, 0x5300, 0x08},  // sharpness control
{HX_CIS_I2C_Action_W, 0x5301, 0x30},  // sharpness threshold
```

---

## 7. Test Pattern

Useful for debugging without a real scene:

```c
{HX_CIS_I2C_Action_W, 0x503d, 0x80},  // enable test pattern
// 0x80 = color bar, 0x82 = color bar with fading, 0x00 = off
```

---

## Quick Reference — Common Adjustments

| Goal | Registers to Change | Notes |
|------|---------------------|-------|
| Brighter image (auto) | `0x3a0f`/`0x3a10` → higher values | Keep auto mode |
| Darker image (auto) | `0x3a0f`/`0x3a10` → lower values | Keep auto mode |
| Less noise | `0x3a19` → lower (e.g., `0x40`) | Limits max gain |
| Fixed exposure | `0x3503`=`0x07`, set `0x3500-02` | Full manual |
| Mirror image | `0x3821` = `0x01` or `0x07` | Toggle bit 1 |
| Flip image | `0x3820` = `0x41` or `0x47` | Toggle bit 1 |
| Higher FPS | `0x3036` → higher, or `0x3035` → `0x11` | Faster PLL clock |
| 60Hz anti-flicker | `0x3c01` = `0x00` | Default is 50Hz |
| Color bar test | `0x503d` = `0x80` | Debug only |
