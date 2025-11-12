#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

void wifi_init_sta(void);
void wifi_register_got_ip_cb(void (*cb)(void));

#endif
