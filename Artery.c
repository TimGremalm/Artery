#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "lwip/api.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "ssid_config.h"

#include "ws2812_i2s/ws2812_i2s.h"
#include "E131.h"
#include "hsl_rgb.h"

e131_packet_t pbuff; /* Packet buffer */
e131_packet_t *pwbuff; /* Pointer to working packet buffer */

void e131task(void *pvParameters) {
	printf("Open server.\r\n");
	vTaskDelay(1000);

	struct netconn *conn;
	err_t err;

	/* Create a new connection handle */
	conn = netconn_new(NETCONN_UDP);
	if(!conn) {
		printf("Error: Failed to allocate socket.\r\n");
		return;
	}

	/* Bind to port with default IP address */
	err = netconn_bind(conn, IP_ADDR_ANY, E131_DEFAULT_PORT);
	if(err != ERR_OK) {
		printf("Error: Failed to bind socket. err=%d\r\n", err);
		return;
	}

	ip4_addr_t multiaddr;
	IP4_ADDR(&multiaddr, 239, 255, 0, 1); //IPv4 local scope multicast

	err = netconn_join_leave_group(conn, &multiaddr, &netif_default->ip_addr, NETCONN_JOIN);
	if(err != ERR_OK) {
		printf("Error: Join Multicast Group. err=%d\r\n", err);
		return;
	}

	printf("Listening for connections.\r\n");

	while(1) {
		struct netbuf *buf;

		err = netconn_recv(conn, &buf);
		if(err != ERR_OK) {
			printf("Error: Failed to receive packet. err=%d\r\n", err);
			continue;
		}

		if(buf->p->tot_len == sizeof(pwbuff->raw)) {
			//If packet is 638 bytes we handle it as a correct package and copy it to a global struct
			if(netbuf_copy(buf, pwbuff->raw, sizeof(pwbuff->raw)) != buf->p->tot_len) {
				printf("Error: Couldn't copy buffer. err=%d\r\n", err);
			}
		} else {
			printf("Wrong packet size.\n\n");
		}

		netbuf_delete(buf);
	}
}

void lighttask(void *pvParameters) {
	//NodeMCU D9 GPIO3 I2S https://github.com/nodemcu/nodemcu-devkit-v1.0#pin-map
	uint32_t led_number = 12;
	ws2812_pixel_t pixels[led_number];
	ws2812_i2s_init(led_number, PIXEL_RGB);
	memset(pixels, 0, sizeof(ws2812_pixel_t) * led_number);

	while(1) {
		uint8_t lightness = pwbuff->property_values[1];
		printf("Channel 1: %d\n", lightness);
		for (int i = 0; i < led_number; i++) {
			pixels[i] = hslToRgb(0, 1.0, (float)lightness/255);
		}
		ws2812_i2s_update(pixels, PIXEL_RGB);
		vTaskDelay(50 / portTICK_PERIOD_MS);
	}
}

void user_init(void) {
	uart_set_baud(0, 115200);
	printf("SDK version:%s\n", sdk_system_get_sdk_version());

	struct sdk_station_config config = {
		.ssid = WIFI_SSID,
		.password = WIFI_PASS,
	};
	sdk_wifi_set_opmode(STATION_MODE);
	sdk_wifi_station_set_config(&config);

	memset(pbuff.raw, 0, sizeof(pbuff.raw));
	pwbuff = &pbuff;
	xTaskCreate(e131task, "e131task", 768, NULL, 8, NULL);
	xTaskCreate(lighttask, "lighttask", 256, NULL, 2, NULL);
}

