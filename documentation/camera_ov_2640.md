## Hardware Parameters

**Sensor Details:**
* **Format:** 1/4-inch CMOS
* **Pixel Size:** 2.2 µm x 2.2 µm
* **Active Array:** 1632 x 1220 pixels

**Supported Frame Rates & Resolutions:**
* **UXGA/SXGA (1600x1200 / 1280x1024):** 15 fps
* **SVGA (800x600):** 30 fps
* **CIF (400x296):** 60 fps

**Sensor Characteristics:**
* **Sensitivity:** 0.6 V/Lux-sec
* **S/N Ratio:** 40 dB
* **Dynamic Range:** 50 dB
* **Field of View (FOV):** Approximately 60° (30° to each side) with a standard lens.

## Optical Features & Filters

* **Color Filter Array:** RGB Bayer pattern (line-alternating BG/GR).
* **Spectral Response:** Good red sensitivity (peaks around 620-660 nm in RGB response curve). The raw Near-Infrared (NIR) response extends beyond 900 nm.
* **IR Filter:** Standard ESP32-CAM modules typically include a 650 nm IR-cut filter to ensure daytime color accuracy. Modules without this filter are required if tracking IR LEDs.

## ESP-IDF Configuration

The following parameters can be adjusted via the ESP-IDF camera driver to optimize the image for LED detection.

### Image Format Settings

When performing pixel-level processing (like applying a Gaussian filter), it is crucial to use a format that is efficient to compute.

* **Recommended Format:** `PIXFORMAT_GRAYSCALE` (1-byte per pixel, ideal for processing).
* **Other Options:** `PIXFORMAT_JPEG` (compressed), `PIXFORMAT_YUV422`, `PIXFORMAT_RGB565`.

```c
config.pixel_format = PIXFORMAT_GRAYSCALE;

```

### Frame Size Options

```c
FRAMESIZE_UXGA (1600 x 1200)
FRAMESIZE_SXGA (1280 x 1024)
FRAMESIZE_XGA  (1024 x 768)
FRAMESIZE_SVGA (800 x 600)
FRAMESIZE_VGA  (640 x 480)
FRAMESIZE_CIF  (352 x 288) // Recommended for high fps
FRAMESIZE_QVGA (320 x 240) // Recommended for high fps

```

### Camera Sensor Settings (`sensor_t`)

For LED detection, isolating bright points and minimizing noise is key. You will likely need to adjust auto-exposure and gain settings.

| Function | Description | Typical Values / Range |
| --- | --- | --- |
| `set_brightness()` | Adjust brightness | -2 to 2 |
| `set_contrast()` | Adjust contrast | -2 to 2 |
| `set_saturation()` | Adjust color saturation | -2 to 2 |
| `set_special_effect()` | Apply color effects | 0 (None), 1 (Negative), 2 (Grayscale), 3-5 (Tints), 6 (Sepia) |
| `set_whitebal()` | Auto White Balance | 0 (Disable), 1 (Enable) |
| `set_awb_gain()` | AWB Gain | 0 (Disable), 1 (Enable) |
| `set_wb_mode()` | White Balance Mode | 0 (Auto), 1-4 (Sunny, Cloudy, Office, Home) |
| `set_exposure_ctrl()` | Auto Exposure Control | 0 (Disable) *Recommended for LED tracking*, 1 (Enable) |
| `set_aec2()` | Auto Exposure Control 2 | 0 (Disable), 1 (Enable) |
| `set_ae_level()` | Auto Exposure Level | -2 to 2 |
| `set_aec_value()` | Manual Exposure Value | 0 to 1200 *(Tune this if auto-exposure is disabled)* |
| `set_gain_ctrl()` | Auto Gain Control | 0 (Disable) *Recommended for low noise*, 1 (Enable) |
| `set_agc_gain()` | Manual Gain Value | 0 to 30 |
| `set_gainceiling()` | Gain Ceiling | 0 to 6 |
| `set_bpc()` | Black Pixel Correction | 0 (Disable), 1 (Enable) |
| `set_wpc()` | White Pixel Correction | 0 (Disable), 1 (Enable) |
| `set_raw_gma()` | Raw Gamma | 0 (Disable), 1 (Enable) |
| `set_lenc()` | Lens Correction | 0 (Disable), 1 (Enable) |
| `set_hmirror()` | Horizontal Mirror | 0 (Disable), 1 (Enable) |
| `set_vflip()` | Vertical Flip | 0 (Disable), 1 (Enable) |
| `set_dcw()` | Downsize Enable | 0 (Disable), 1 (Enable) |
| `set_colorbar()` | Test Colorbar | 0 (Disable), 1 (Enable) |

**Example Implementation:**

```c
sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_special_effect(s, 0);
  s->set_whitebal(s, 0);       // Disable AWB for consistency
  s->set_awb_gain(s, 0);
  s->set_wb_mode(s, 0);
  s->set_exposure_ctrl(s, 0);  // Disable Auto Exposure to isolate bright LEDs
  s->set_aec2(s, 0);
  s->set_ae_level(s, 0);
  s->set_aec_value(s, 300);    // Set fixed exposure (tune as needed)
  s->set_gain_ctrl(s, 0);      // Disable Auto Gain to reduce noise
  s->set_agc_gain(s, 0);       // Set low fixed gain
  s->set_gainceiling(s, (gainceiling_t)0);
  s->set_bpc(s, 0);
  s->set_wpc(s, 1);
  s->set_raw_gma(s, 1);
  s->set_lenc(s, 1);
  s->set_hmirror(s, 0);
  s->set_vflip(s, 0);
  s->set_dcw(s, 1);
  s->set_colorbar(s, 0);

```

## GPIO Pinout & Clock Configuration

Standard GPIO assignments for AI-Thinker style ESP32-CAM modules.

```c
config.ledc_channel = LEDC_CHANNEL_0;
config.ledc_timer   = LEDC_TIMER_0;

// Data Pins
config.pin_d0 = Y2_GPIO_NUM;
config.pin_d1 = Y3_GPIO_NUM;
config.pin_d2 = Y4_GPIO_NUM;
config.pin_d3 = Y5_GPIO_NUM;
config.pin_d4 = Y6_GPIO_NUM;
config.pin_d5 = Y7_GPIO_NUM;
config.pin_d6 = Y8_GPIO_NUM;
config.pin_d7 = Y9_GPIO_NUM;

// Control Pins
config.pin_xclk  = XCLK_GPIO_NUM;
config.pin_pclk  = PCLK_GPIO_NUM;
config.pin_vsync = VSYNC_GPIO_NUM;
config.pin_href  = HREF_GPIO_NUM;

// I2C (SCCB) Pins
config.pin_sscb_sda = SIOD_GPIO_NUM;
config.pin_sscb_scl = SIOC_GPIO_NUM;

// Power & Reset
config.pin_pwdn  = PWDN_GPIO_NUM;
config.pin_reset = RESET_GPIO_NUM;

// Camera Clock Frequency
config.xclk_freq_hz = 20000000; // 20 MHz

```
