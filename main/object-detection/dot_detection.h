#pragma once

#include "esp_camera.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One detected LED marker, in image-centered coordinates:
 * x: left negative / right positive, y: down negative / up positive. */
typedef struct {
    float x;
    float y;
    int count; /* pixels in the blob */
} DetectedDot;

typedef struct {
    bool valid;
    DetectedDot dots[3]; /* ordered left, center, right */
    float spacing_px;    /* dots[2].x - dots[0].x; grows as the leader gets closer */
} BlobResult;

/* Detect the three-LED reference on a YUV422 (YUYV) frame and publish the
 * result to dots_detection_queue (single-slot, overwrite semantics). */
BlobResult process_image(camera_fb_t *fb);

/* Copy of the most recently published result, for consumers that must not
 * drain the control queue (e.g. the debug stream overlay). Thread-safe. */
BlobResult dot_detection_get_last(void);

/* Debug: paint every pixel that passes the red-detection thresholds bright
 * green, directly into a YUYV frame. Lets you SEE what the detector sees
 * (http://<robot>.local/mask) and tune the thresholds against reality. */
void dot_detection_paint_mask(camera_fb_t *fb);

/* ── Живі параметри детекції ─────────────────────────────────────────────
 * Всі пороги можна міняти на льоту зі сторінки /tune (без перепрошивки).
 * "Зберегти" на сторінці пише їх у NVS; при старті вони звантажуються
 * назад. Відтюнені значення варто перенести в код як нові дефолти
 * (dot_detection.c, блок DEF_*). */

typedef struct {
    /* пороги "червоності" (YUV) */
    int red_v_min;  /* V (Cr): високий = червоне */
    int red_u_max;  /* U (Cb): низький = не синє */
    int luma_min;   /* Y: мінімальна яскравість */
    int min_blob_px; /* менші плями - шум */
    /* геометрія трійки */
    float geo_max_dy;    /* мін. допуск по вертикалі, px */
    float geo_dy_frac;   /* + частка ширини (нахил планки) */
    float geo_min_width; /* мінімальна ширина трійки, px */
    float geo_sym_tol;   /* симетрія центру, частка ширини */
    /* трекінг */
    float trk_max_jump;  /* макс. стрибок центру за кадр, px */
    float trk_scale_min; /* межі зміни розміру за кадр */
    float trk_scale_max;
    float ema_alpha;     /* вага нового виміру в згладжуванні */
} detect_params_t;

detect_params_t dot_detection_get_params(void);
void dot_detection_set_params(const detect_params_t *p);
void dot_detection_params_reset(void); /* повернути дефолти з коду */
void dot_detection_params_load(void);  /* з NVS (кличеться при старті) */
esp_err_t dot_detection_params_save(void); /* у NVS */

#ifdef __cplusplus
}
#endif
