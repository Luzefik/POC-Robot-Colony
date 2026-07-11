#include "web_tune.h"

#include "dot_detection.h"
#include "esp_camera.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "web_tune";

/* ── /tune/get: поточні параметри як JSON ───────────────────────────────── */

static int params_to_json(char *out, size_t cap) {
    detect_params_t p = dot_detection_get_params();
    return snprintf(out, cap,
        "{\"red_v_min\":%d,\"red_u_max\":%d,\"luma_min\":%d,"
        "\"min_blob_px\":%d,\"geo_max_dy\":%.1f,\"geo_dy_frac\":%.2f,"
        "\"geo_min_width\":%.1f,\"geo_sym_tol\":%.2f,\"trk_max_jump\":%.0f,"
        "\"trk_scale_min\":%.2f,\"trk_scale_max\":%.2f,\"ema_alpha\":%.2f}",
        p.red_v_min, p.red_u_max, p.luma_min, p.min_blob_px, p.geo_max_dy,
        p.geo_dy_frac, p.geo_min_width, p.geo_sym_tol, p.trk_max_jump,
        p.trk_scale_min, p.trk_scale_max, p.ema_alpha);
}

static esp_err_t get_handler(httpd_req_t *req) {
    char json[320];
    params_to_json(json, sizeof(json));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

/* ── /tune/set?ім'я=значення&... ─────────────────────────────────────────── */

static void set_int(const char *q, const char *key, int *dst, int lo, int hi) {
    char v[16];
    if (httpd_query_key_value(q, key, v, sizeof(v)) == ESP_OK) {
        int n = atoi(v);
        if (n >= lo && n <= hi)
            *dst = n;
    }
}

static void set_f(const char *q, const char *key, float *dst, float lo,
                  float hi) {
    char v[16];
    if (httpd_query_key_value(q, key, v, sizeof(v)) == ESP_OK) {
        float n = strtof(v, NULL);
        if (n >= lo && n <= hi)
            *dst = n;
    }
}

static esp_err_t set_handler(httpd_req_t *req) {
    char q[256] = "";
    httpd_req_get_url_query_str(req, q, sizeof(q));

    detect_params_t p = dot_detection_get_params();
    set_int(q, "red_v_min", &p.red_v_min, 0, 255);
    set_int(q, "red_u_max", &p.red_u_max, 0, 255);
    set_int(q, "luma_min", &p.luma_min, 0, 255);
    set_int(q, "min_blob_px", &p.min_blob_px, 1, 100);
    set_f(q, "geo_max_dy", &p.geo_max_dy, 1, 200);
    set_f(q, "geo_dy_frac", &p.geo_dy_frac, 0, 1);
    set_f(q, "geo_min_width", &p.geo_min_width, 1, 300);
    set_f(q, "geo_sym_tol", &p.geo_sym_tol, 0.01f, 1);
    set_f(q, "trk_max_jump", &p.trk_max_jump, 5, 400);
    set_f(q, "trk_scale_min", &p.trk_scale_min, 0.1f, 1);
    set_f(q, "trk_scale_max", &p.trk_scale_max, 1, 5);
    set_f(q, "ema_alpha", &p.ema_alpha, 0.05f, 1);
    dot_detection_set_params(&p);

    return get_handler(req);
}

static esp_err_t save_handler(httpd_req_t *req) {
    bool ok = dot_detection_params_save() == ESP_OK;
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, ok ? "{\"saved\":true}" : "{\"saved\":false}",
                           HTTPD_RESP_USE_STRLEN);
}

static esp_err_t reset_handler(httpd_req_t *req) {
    dot_detection_params_reset();
    return get_handler(req);
}

/* ── /probe?x=..&y=.. : піпетка - YUV значення пікселя ──────────────────── */

static esp_err_t probe_handler(httpd_req_t *req) {
    char q[64] = "", v[8];
    int x = -1, y = -1;
    httpd_req_get_url_query_str(req, q, sizeof(q));
    if (httpd_query_key_value(q, "x", v, sizeof(v)) == ESP_OK)
        x = atoi(v);
    if (httpd_query_key_value(q, "y", v, sizeof(v)) == ESP_OK)
        y = atoi(v);

    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"error\":\"no frame\"}",
                               HTTPD_RESP_USE_STRLEN);
    }

    char json[192];
    if (fb->format == PIXFORMAT_YUV422 && x >= 0 && y >= 0 &&
        x < (int)fb->width && y < (int)fb->height) {
        size_t px = ((size_t)y * fb->width + x) * 2;
        size_t pair = px & ~(size_t)3;
        uint8_t Y = fb->buf[px];
        uint8_t U = fb->buf[pair + 1];
        uint8_t V = fb->buf[pair + 3];

        detect_params_t p = dot_detection_get_params();
        bool red = (V >= p.red_v_min) && (U < p.red_u_max) && (Y > p.luma_min);
        snprintf(json, sizeof(json),
                 "{\"x\":%d,\"y\":%d,\"Y\":%d,\"U\":%d,\"V\":%d,\"red\":%s,"
                 "\"w\":%u,\"h\":%u}",
                 x, y, Y, U, V, red ? "true" : "false", fb->width, fb->height);
    } else {
        snprintf(json, sizeof(json), "{\"error\":\"bad coords\"}");
    }
    esp_camera_fb_return(fb);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

/* ── /tune: сторінка зі слайдерами ──────────────────────────────────────── */

static const char TUNE_HTML[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>UGV tune</title><style>"
"body{background:#111;color:#ddd;font:14px/1.5 monospace;margin:0;padding:14px}"
"h2{margin:4px 0 10px;font-size:16px}"
"a{color:#7c7}.row{display:flex;align-items:center;gap:8px;margin:6px 0}"
".row label{width:120px}.row input[type=range]{flex:1}"
".row output{width:52px;text-align:right}"
"button{background:#274;color:#fff;border:0;padding:8px 14px;margin:8px 6px 0 0;"
"border-radius:4px;font:inherit;cursor:pointer}"
"button.grey{background:#444}#st{margin-left:8px;color:#9c9}"
"small{color:#888}</style></head><body>"
"<h2>Живий тюнінг детекції "
"<small><a href='/' target='_blank'>стрім</a> | "
"<a href='/mask' target='_blank'>маска</a> | "
"<a href='/log' target='_blank'>лог</a></small></h2>"
"<div id='rows'></div>"
"<button onclick='save()'>Зберегти в NVS</button>"
"<button class='grey' onclick='reset()'>Скинути до дефолтів</button>"
"<span id='st'></span>"
"<p><small>Міняй слайдери і дивись на /mask у сусідній вкладці: ліхтарики "
"мають бути зеленими плямами, все інше - ні. Значення застосовуються "
"миттєво; без \"Зберегти\" вони зникнуть після перезавантаження.</small></p>"
"<script>"
"const DEFS=["
"['red_v_min','V min (червоне)',0,255,1],"
"['red_u_max','U max (не синє)',0,255,1],"
"['luma_min','Y min (яскравість)',0,255,1],"
"['min_blob_px','мін. плями, px',1,60,1],"
"['geo_max_dy','dy допуск, px',1,100,1],"
"['geo_dy_frac','dy частка ширини',0,1,0.05],"
"['geo_min_width','мін. ширина, px',1,200,1],"
"['geo_sym_tol','симетрія',0.05,1,0.05],"
"['trk_max_jump','макс. стрибок, px',5,400,5],"
"['trk_scale_min','масштаб min',0.1,1,0.05],"
"['trk_scale_max','масштаб max',1,3,0.1],"
"['ema_alpha','EMA alpha',0.05,1,0.05]];"
"const rows=document.getElementById('rows'),st=document.getElementById('st');"
"let t=null;"
"function row(k,label,lo,hi,step,val){"
"const d=document.createElement('div');d.className='row';"
"d.innerHTML=`<label>${label}</label><input type=range id=${k} min=${lo} "
"max=${hi} step=${step} value=${val}><output id=o_${k}>${val}</output>`;"
"rows.appendChild(d);"
"d.querySelector('input').oninput=e=>{"
"document.getElementById('o_'+k).textContent=e.target.value;"
"clearTimeout(t);t=setTimeout(()=>apply(k,e.target.value),120);};}"
"async function apply(k,v){"
"await fetch(`/tune/set?${k}=${v}`);st.textContent='застосовано';"
"setTimeout(()=>st.textContent='',700);}"
"async function save(){const r=await(await fetch('/tune/save')).json();"
"st.textContent=r.saved?'збережено в NVS':'ПОМИЛКА збереження';}"
"async function reset(){const p=await(await fetch('/tune/reset')).json();"
"fill(p);st.textContent='дефолти';}"
"function fill(p){for(const[k]of DEFS){const i=document.getElementById(k);"
"if(i){i.value=p[k];document.getElementById('o_'+k).textContent=p[k];}}}"
"(async()=>{const p=await(await fetch('/tune/get')).json();"
"for(const[k,l,lo,hi,s]of DEFS)row(k,l,lo,hi,s,p[k]);})();"
"</script></body></html>";

static esp_err_t tune_page_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, TUNE_HTML, HTTPD_RESP_USE_STRLEN);
}

void web_tune_register(httpd_handle_t server) {
    const httpd_uri_t uris[] = {
        {.uri = "/tune", .method = HTTP_GET, .handler = tune_page_handler},
        {.uri = "/tune/get", .method = HTTP_GET, .handler = get_handler},
        {.uri = "/tune/set", .method = HTTP_GET, .handler = set_handler},
        {.uri = "/tune/save", .method = HTTP_GET, .handler = save_handler},
        {.uri = "/tune/reset", .method = HTTP_GET, .handler = reset_handler},
        {.uri = "/probe", .method = HTTP_GET, .handler = probe_handler},
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++)
        httpd_register_uri_handler(server, &uris[i]);
    ESP_LOGI(TAG, "Tune UI at /tune, pixel probe at /probe");
}
