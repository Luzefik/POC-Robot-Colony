// Example file - Public Domain
// Need help? https://tinyurl.com/bluepad32-help

#include <string.h>
#include <stdbool.h>
#include <driving.h>
#include "esp_log.h"

#include "../src/components/bluepad32/include/uni.h"

// External function declared in main.c
extern void set_gamepad_connected(bool connected);

// Custom "instance"
typedef struct my_platform_instance_s {
    uni_gamepad_seat_t gamepad_seat;  // which "seat" is being used
} my_platform_instance_t;

// Declarations
static void trigger_event_on_gamepad(uni_hid_device_t* d);
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d);

//
// Platform Overrides
//
static void my_platform_init(int argc, const char** argv) {
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    logi("custom: init()\n");
    logi("[GAMEPAD] Initializing gamepad platform...\n");

#if 0
    uni_gamepad_mappings_t mappings = GAMEPAD_DEFAULT_MAPPINGS;

    // Inverted axis with inverted Y in RY.
    mappings.axis_x = UNI_GAMEPAD_MAPPINGS_AXIS_RX;
    mappings.axis_y = UNI_GAMEPAD_MAPPINGS_AXIS_RY;
    mappings.axis_ry_inverted = true;
    mappings.axis_rx = UNI_GAMEPAD_MAPPINGS_AXIS_X;
    mappings.axis_ry = UNI_GAMEPAD_MAPPINGS_AXIS_Y;

    // Invert A & B
    mappings.button_a = UNI_GAMEPAD_MAPPINGS_BUTTON_B;
    mappings.button_b = UNI_GAMEPAD_MAPPINGS_BUTTON_A;

    uni_gamepad_set_mappings(&mappings);
#endif
    //    uni_bt_service_set_enabled(true);
}

static void my_platform_on_init_complete(void) {
    logi("custom: on_init_complete()\n");
    logi("[GAMEPAD] Initialization complete - Starting BT scanning...\n");
    logi("[GAMEPAD] IMPORTANT: Put your gamepad in PAIRING/DISCOVERY MODE!\n");

    // Safe to call "unsafe" functions since they are called from BT thread

    // Start scanning
    uni_bt_start_scanning_and_autoconnect_unsafe();
    uni_bt_allow_incoming_connections(true);
    logi("[GAMEPAD] Bluetooth scanning started - waiting for gamepad connection...\n");

    // Based on runtime condition, you can delete or list the stored BT keys.
    if (1)
        uni_bt_del_keys_unsafe();
    else
        uni_bt_list_keys_unsafe();
}

static uni_error_t my_platform_on_device_discovered(bd_addr_t addr, const char* name, uint16_t cod, uint8_t rssi) {
    // You can filter discovered devices here.
    // Just return any value different from UNI_ERROR_SUCCESS;
    // @param addr: the Bluetooth address
    // @param name: could be NULL, could be zero-length, or might contain the name.
    // @param cod: Class of Device. See "uni_bt_defines.h" for possible values.
    // @param rssi: Received Signal Strength Indicator (RSSI) measured in dBms. The higher (255) the better.


    // As an example, if you want to filter out keyboards, do:
    if (((cod & UNI_BT_COD_MINOR_MASK) & UNI_BT_COD_MINOR_KEYBOARD) == UNI_BT_COD_MINOR_KEYBOARD) {
        return UNI_ERROR_IGNORE_DEVICE;
    }

    return UNI_ERROR_SUCCESS;
}

static void my_platform_on_device_connected(uni_hid_device_t* d) {
    logi("custom: device connected: %p\n", d);
}

static void my_platform_on_device_disconnected(uni_hid_device_t* d) {
    logi("custom: device disconnected: %p\n", d);
    logi("[GAMEPAD] Disconnected - Device address: %02X:%02X:%02X:%02X:%02X:%02X\n",
         d->conn.btaddr[0], d->conn.btaddr[1], d->conn.btaddr[2],
         d->conn.btaddr[3], d->conn.btaddr[4], d->conn.btaddr[5]);
    logi("[GAMEPAD] Waiting for new connection...\n");

    // Signal that gamepad is no longer connected
    set_gamepad_connected(false);
}

static uni_error_t my_platform_on_device_ready(uni_hid_device_t* d) {
    logi("custom: device ready: %p\n", d);
    my_platform_instance_t* ins = get_my_platform_instance(d);
    ins->gamepad_seat = GAMEPAD_SEAT_A;

    trigger_event_on_gamepad(d);

    // Signal that gamepad is now connected and ready
    set_gamepad_connected(true);

    return UNI_ERROR_SUCCESS;
}

int16_t clamp(int x, int min, int max) {
    if (x < min) {
        return min;
    } else if (x >= max) {
        return max;
    }

    return x;
}



static void my_platform_on_controller_data(uni_hid_device_t* d, uni_controller_t* ctl) {
    static uni_controller_t prev = {0};
    uni_gamepad_t* gp;

    static int16_t speed = 0;
    static int8_t turn = 0;

    if (memcmp(&prev, ctl, sizeof(*ctl)) == 0) {
        return;
    }
    prev = *ctl;
    gp = &ctl->gamepad;

    int16_t left_y = (gp->axis_y == 4) ? 0 : (-gp->axis_y / 8)*1.125;
    int16_t right_x = (gp->axis_rx == 4) ? 0 : gp->axis_rx / 8;

    if (left_y > 0) {left_y += 335;}
    else if (left_y < 0) { left_y -= 336;}
    
    if (left_y > speed) {speed += (left_y - speed);}
    else if (left_y < speed) {speed -= (speed - left_y);}

    if (0 < speed && speed < 319) {speed = 319;}
    else if (-320 < speed && speed < 0) {speed = -320;}


    right_x = clamp(right_x, -32, 31);
    if (right_x > turn) {turn += 2;}
    else if (right_x < turn) {turn -= 2;}


    if (speed > 0) {
        speed = abs(speed);
        motor(0, speed - (turn / 2));
        motor(1, 0);

        motor(2, speed + (turn / 2));
        motor(3, 0);
    } else {
        speed = abs(speed);
        motor(1, speed - (turn / 2));
        motor(0, 0);

        motor(3, speed + (turn / 2));
        motor(2, 0);
    }
}

static const uni_property_t* my_platform_get_property(uni_property_idx_t idx) {
    ARG_UNUSED(idx);
    return NULL;
}

static void my_platform_on_oob_event(uni_platform_oob_event_t event, void* data) {
    switch (event) {
        case UNI_PLATFORM_OOB_GAMEPAD_SYSTEM_BUTTON: {
            uni_hid_device_t* d = data;

            if (d == NULL) {
                loge("ERROR: my_platform_on_oob_event: Invalid NULL device\n");
                return;
            }
            logi("custom: on_device_oob_event(): %d\n", event);

            my_platform_instance_t* ins = get_my_platform_instance(d);
            ins->gamepad_seat = ins->gamepad_seat == GAMEPAD_SEAT_A ? GAMEPAD_SEAT_B : GAMEPAD_SEAT_A;

            trigger_event_on_gamepad(d);
            break;
        }

        case UNI_PLATFORM_OOB_BLUETOOTH_ENABLED:
            logi("custom: Bluetooth enabled: %d\n", (bool)(data));
            break;

        default:
            logi("my_platform_on_oob_event: unsupported event: 0x%04x\n", event);
            break;
    }
}

//
// Helpers
//
static my_platform_instance_t* get_my_platform_instance(uni_hid_device_t* d) {
    return (my_platform_instance_t*)&d->platform_data[0];
}

static void trigger_event_on_gamepad(uni_hid_device_t* d) {
    my_platform_instance_t* ins = get_my_platform_instance(d);

    if (d->report_parser.play_dual_rumble != NULL) {
        d->report_parser.play_dual_rumble(d, 0 /* delayed start ms */, 150 /* duration ms */, 128 /* weak magnitude */,
                                          40 /* strong magnitude */);
    }

}

//
// Entry Point
//
struct uni_platform* get_my_platform(void) {
    static struct uni_platform plat = {
        .name = "custom",
        .init = my_platform_init,
        .on_init_complete = my_platform_on_init_complete,
        .on_device_discovered = my_platform_on_device_discovered,
        .on_device_connected = my_platform_on_device_connected,
        .on_device_disconnected = my_platform_on_device_disconnected,
        .on_device_ready = my_platform_on_device_ready,
        .on_oob_event = my_platform_on_oob_event,
        .on_controller_data = my_platform_on_controller_data,
        .get_property = my_platform_get_property,
    };

    return &plat;
}