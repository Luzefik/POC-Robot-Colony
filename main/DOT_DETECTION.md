# Dot Detection Algorithm

Fast red blob detection for ESP32-CAM robot colony tracking.

## Overview

This module detects up to 3 red dots in real-time camera frames using chrominance-based filtering in YUV color space. Designed for ESP32-CAM with minimal memory footprint and low latency.

**Key Features:**
- Single-pass pixel scanning with spatial clustering
- YUV422 (YUYV) native format processing (no color conversion needed)
- Horizontal 5-tap Gaussian on the Y channel (noise suppression, frame kept intact)
- Geometric validation of the LED triple (horizontal, wide enough, symmetric)
- Temporal gating: the triple cannot teleport or change scale in one frame
- EMA smoothing (alpha = 0.8) against frame-to-frame jitter
- Tracking persistence across temporary detection failures
- Image-centered coordinate output for robot navigation

## Algorithm Pipeline

```
┌─────────────────────────────────────────────────────────────┐
│                 CAMERA FRAME (YUV422)                       │
│     ●              ●              ●                         │
│   (red)          (red)         (red)                        │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ PASS 1: Color Filtering                                     │
│   - Check each pixel: V ≥ 150, U < 100, Y > 50              │
│   - Cluster nearby red pixels using greedy assignment       │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ PASS 2: Blob Validation                                     │
│   - Filter out noise (blobs < 5 pixels)                     │
│   - Calculate centroid for each valid blob                  │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ PASS 3: Geometric Validation & Triple Selection             │
│   - Triple must be near-horizontal (Δy < 20 px)             │
│   - Wide enough (> 20 px), center dot near the middle       │
│   - While tracking: reject jumps > 80 px and scale changes  │
│     outside 0.6–1.6× (stops locking onto other red objects) │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ PASS 4: EMA Smoothing (α = 0.8) + Coordinate Transform      │
│   - Origin at image center                                  │
│   - +X right, -X left, +Y up, -Y down                       │
└─────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────┐
│ OUTPUT: BlobResult → FreeRTOS Queue                         │
│   LEFT: (-80, 40)  CENTER: (0, 35)  RIGHT: (75, 38)         │
└─────────────────────────────────────────────────────────────┘
```

## API Reference

### Main Function

```c
BlobResult process_image(camera_fb_t *fb);
```

**Parameters:**
| Name | Type | Description |
|------|------|-------------|
| `fb` | `camera_fb_t*` | ESP32-CAM frame buffer in YUV422 format |

**Returns:** `BlobResult` structure containing up to 3 detected blobs.

**Thread Safety:** Not thread-safe. Call from a single task only.

### Data Structures

#### `DetectedDot`
```c
typedef struct {
    float x;    // X coordinate (image-centered, right positive)
    float y;    // Y coordinate (image-centered, up positive)
    int count;  // Number of pixels in the blob
} DetectedDot;
```

#### `BlobResult`
```c
typedef struct {
    bool valid;           // true only for a geometrically consistent triple
    DetectedDot dots[3];  // [0]=LEFT, [1]=CENTER, [2]=RIGHT
    float spacing_px;     // dots[2].x - dots[0].x; grows as leader gets closer
} BlobResult;
```

**Dot ordering:** Sorted by X coordinate (leftmost first). Always check
`valid` before using the coordinates.

**Coordinate system:**
```
        +Y (up)
          │
          │
 -X ──────┼────── +X
  (left)  │        (right)
          │
        -Y (down)

Origin: Image center (width/2, height/2)
```

### External Dependencies

```c
extern QueueHandle_t dots_detection_queue;  // Define in main.c
```

Results are automatically sent to this FreeRTOS queue using `xQueueOverwrite()`.

## Configuration

All parameters are defined as macros in `dot_detection.c`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `RED_V_MIN` | 150 | Minimum V (Cr) channel value for red detection |
| `RED_U_MAX` | 100 | Maximum U (Cb) channel value (excludes blue/purple) |
| `LUMA_MIN` | 50 | Minimum brightness (Y channel) |
| `MAX_BLOBS` | 10 | Maximum simultaneous blob tracking |
| `MIN_BLOB_PIXELS` | 5 | Minimum pixels to consider valid blob |
| `CLUSTER_RADIUS` | 50 | Max distance (px) to merge pixels into same blob |
| `SCAN_STEP` | 2 | Pixel skip factor (1=full, 2=half resolution) |
| `GEO_MAX_DY` | 20 | Max vertical spread inside the triple (px) |
| `GEO_MIN_WIDTH` | 20 | Min horizontal width of the triple (px) |
| `GEO_SYM_TOL` | 0.35 | Center-dot symmetry tolerance (fraction of width) |
| `TRACK_MAX_JUMP_PX` | 80 | Max center displacement between frames (px) |
| `TRACK_SCALE_MIN/MAX` | 0.6 / 1.6 | Accepted width change between frames |
| `EMA_ALPHA` | 0.8 | Weight of the new measurement in the output |
| `MAX_BLIND_FRAMES` | 5 | Frames to retain last position on detection loss |

## Color Detection

### YUV422 Format

The ESP32-CAM outputs frames in YUYV (YUV422) packed format:

```
Byte:    [Y0] [U]  [Y1] [V]  [Y2] [U]  [Y3] [V]  ...
Pixel:    0         1         2         3
```

Each pair of horizontal pixels shares U and V chrominance values.

### Red Detection Logic

A pixel is classified as red when:

```c
(V >= 150) && (U < 100) && (Y > 50)
```

| Channel | Condition | Rationale |
|---------|-----------|-----------|
| V (Cr) | ≥ 150 | High Cr indicates red/orange hue |
| U (Cb) | < 100 | Low Cb excludes blue/purple tones |
| Y | > 50 | Ensures sufficient brightness (not black) |

### Tuning Color Thresholds

**Too few detections:** Lower `RED_V_MIN` or raise `RED_U_MAX`
**False positives (orange/pink):** Raise `RED_V_MIN` or lower `RED_U_MAX`
**Dark red not detected:** Lower `LUMA_MIN`

### Live tuning workflow (no cable needed)

1. Enable the debug stream (menuconfig → `UGV_ENABLE_WEB_STREAM` + Wi-Fi
   credentials) and open the robot's pages in a browser:
   - `http://ugv-XXXX.local/mask` — every pixel passing the thresholds is
     painted green. Lit LEDs must show as three solid green blobs; nothing
     else in the scene should be green.
   - `http://ugv-XXXX.local/` — green boxes around the locked triple; the
     small square top-left is the lock status (green = locked, red = lost).
   - `http://ugv-XXXX.local/log` — the live log.
2. If blobs are green but the status stays red, look for this line in the
   log — it says exactly which geometry check rejects the triple:
   ```
   W blob_detect: 4 blobs, no triple: rejects dy=2 width=0 sym=1 scale=0 jump=0
   W blob_detect:   blob 0: (152, 148) 214 px
   ```
   - `dy` — the triple is not horizontal enough → raise `GEO_MAX_DY_FRAC`
   - `sym` — center dot too far from the middle → raise `GEO_SYM_TOL`
   - `scale`/`jump` — tracking gates too strict → widen `TRACK_*`
3. Change the constant, rebuild and flash (`./ugv.sh`), repeat.

## Clustering Algorithm

### Greedy Assignment

For each red pixel found:
1. Calculate distance to centroid of each existing blob
2. If within `CLUSTER_RADIUS` of nearest blob → add to that blob
3. Otherwise → create new blob (if under `MAX_BLOBS` limit)

```
        CLUSTER_RADIUS
            ←───→
        ┌─────────┐
        │  blob   │  ← Red pixel inside radius joins this blob
        │   ●     │
        └─────────┘

          ✕ (x,y)   ← New pixel being evaluated
```

### Centroid Calculation

Blob position is the mean of all member pixels:

```
centroid_x = sum_x / count
centroid_y = sum_y / count
```

## Tracking Persistence

When fewer than 3 blobs are detected:

1. **If previously tracking:** Return last known position for up to `MAX_BLIND_FRAMES` frames
2. **If persistence exhausted:** Return zero result, log warning

This smooths temporary occlusions and lighting changes.

## Usage Example

### Basic Setup

```c
// main.c
#include "dot_detection.h"
#include "freertos/queue.h"

QueueHandle_t dots_detection_queue;

void app_main(void) {
    // Create queue (single item, overwrite mode)
    dots_detection_queue = xQueueCreate(1, sizeof(BlobResult));

    // Initialize camera...

    // Start detection task
    xTaskCreate(detection_task, "detect", 4096, NULL, 5, NULL);
}

void detection_task(void *arg) {
    while (1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            BlobResult result = process_image(fb);
            esp_camera_fb_return(fb);
        }
        vTaskDelay(pdMS_TO_TICKS(50));  // ~20 FPS
    }
}
```

### Reading Results

```c
void navigation_task(void *arg) {
    BlobResult result;

    while (1) {
        if (xQueueReceive(dots_detection_queue, &result, pdMS_TO_TICKS(50)) &&
            result.valid) {
            float heading_err = result.dots[1].x; // px, >0: leader to the right
            float spacing = result.spacing_px;    // grows as the leader closes in

            // Use for navigation... (see follower_loop in main.c)
        }
    }
}
```

## Performance

### Complexity

| Operation | Time Complexity |
|-----------|-----------------|
| Pixel scan | O(W × H / SCAN_STEP²) |
| Clustering | O(red_pixels × active_blobs) |
| Sorting | O(blobs²) |
| Total | O(W × H) typical |

### Benchmarks (640×480 frame)

| Metric | Value |
|--------|-------|
| Processing time | ~1-2 ms |
| Memory usage | ~200 bytes stack |
| Max sustainable FPS | 500+ (CPU-bound) |

### Optimization Tips

1. **Increase `SCAN_STEP`** to 4 for faster but coarser detection
2. **Reduce `MAX_BLOBS`** if detecting fewer targets
3. **Crop ROI** if dots appear in predictable region

## Troubleshooting

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| No blobs detected | Thresholds too strict | Lower `RED_V_MIN`, check lighting |
| Wrong colors detected | Thresholds too loose | Raise `RED_V_MIN`, lower `RED_U_MAX` |
| Blobs merge together | `CLUSTER_RADIUS` too large | Reduce to 30-40 |
| Blobs split apart | `CLUSTER_RADIUS` too small | Increase to 60-80 |
| Flickering detection | Noise or small blobs | Increase `MIN_BLOB_PIXELS` |
| Lag in tracking | Frame rate too low | Reduce `SCAN_STEP`, optimize camera |

## Testing

### Desktop Simulation

```bash
# Compile test harness
gcc -O2 -o test_algo test_algo.c -lm

# Run with test image
./test_algo test.jpeg 20 100
```

### Creating Test Images

Use any image editor to create a JPEG with 3 red circles (RGB ~255,0,0) on contrasting background.

## File Structure

```
main/
├── dot_detection.c    # Algorithm implementation
├── dot_detection.h    # Public API
└── DOT_DETECTION.md   # This documentation
```
