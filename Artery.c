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

#define min(x, y) (((x) < (y)) ? (x) : (y))
#define max(a,b) \
	({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a > _b ? _a : _b; })

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

typedef struct {
	uint8_t pulse_width;
	uint8_t pulse_gap;
	uint8_t pulse_bpm;
	uint8_t pulse_flow;
	float pulse_hue;
	float pulse_light_min;
	float pulse_light_max;
} beat_parameters_t;

enum DMX_CHANNEL {
	DMX_WIDTH,
	DMX_GAP,
	DMX_BPM,
	DMX_FLOW,
	DMX_HUE,
	DMX_LIGHT_MIN,
	DMX_LIGHT_MAX,
};

void init_dmx_values(int dmx_address) {
	pwbuff->property_values[dmx_address+DMX_LIGHT_MAX] = 10;
	pwbuff->property_values[dmx_address+DMX_LIGHT_MIN] = 10;
	pwbuff->property_values[dmx_address+DMX_HUE] = 0;
	pwbuff->property_values[dmx_address+DMX_WIDTH] = 4;
	pwbuff->property_values[dmx_address+DMX_GAP] = 10;
	pwbuff->property_values[dmx_address+DMX_BPM] = 128;
	pwbuff->property_values[dmx_address+DMX_FLOW] = 30;
}

beat_parameters_t readparametersfromdmx(int dmx_address) {
	beat_parameters_t paramout;
	// Read parameters from DMX values
	paramout.pulse_light_max = (float)pwbuff->property_values[dmx_address+DMX_LIGHT_MAX] / 255;
	paramout.pulse_light_min = ((float)pwbuff->property_values[dmx_address+DMX_LIGHT_MIN] / 255) * paramout.pulse_light_max;
	paramout.pulse_hue = (float)pwbuff->property_values[dmx_address+DMX_HUE] / 255;
	paramout.pulse_width = pwbuff->property_values[dmx_address+DMX_WIDTH];
	paramout.pulse_gap = pwbuff->property_values[dmx_address+DMX_GAP];
	paramout.pulse_bpm = pwbuff->property_values[dmx_address+DMX_BPM];
	paramout.pulse_flow = pwbuff->property_values[dmx_address+DMX_FLOW];

	// Make sure values are whithin ranges
	if (paramout.pulse_width < 1) {
		paramout.pulse_width = 1;
	}
	if (paramout.pulse_gap < 1) {
		paramout.pulse_gap = 1;
	}
	if (paramout.pulse_bpm < 1) {
		paramout.pulse_bpm = 1;
	}

	return paramout;
}

void lighttask(void *pvParameters) {
	//NodeMCU D9 GPIO3 I2S https://github.com/nodemcu/nodemcu-devkit-v1.0#pin-map
	uint32_t led_number = 110;
	int dmx_address = 1;
	init_dmx_values(dmx_address);
	ws2812_pixel_t pixels[led_number];
	ws2812_i2s_init(led_number, PIXEL_RGB);
	memset(pixels, 0, sizeof(ws2812_pixel_t) * led_number);
	beat_parameters_t beat_parameters = {0};
	uint8_t bpm_last = 0;
	int segment_pixels;
	int segment_index;
	float segment_offset = 0.0;
	float pixel_lightlevel;
	float pixel_rest;
	float beat_level;
	uint32_t beat_mid = 0;
	uint32_t beat_end = 0;
	uint32_t now;
	uint32_t beatms;
	uint32_t diff;

	while(1) {
		// Timing of beats
		bpm_last = beat_parameters.pulse_bpm;
		beat_parameters = readparametersfromdmx(dmx_address);
		now = xTaskGetTickCount()*portTICK_PERIOD_MS;
		beatms = (((float)60/beat_parameters.pulse_bpm)*1000)/2;
		// If beat pastt, or BPM value changed, calculate next beat
		if (now > beat_end || beat_parameters.pulse_bpm != bpm_last) {
			beat_mid = now + beatms;
			beat_end = beat_mid + beatms;
		}
		if (now < beat_mid) {
			// Start of beat, go up
			diff = beat_mid - now;
			beat_level = (float)diff / beatms;
		} else {
			// End of beat, go down
			diff = beat_end - now;
			beat_level = 1.0 - ((float)diff / beatms);
		}

		// By offseting segments a movement can be created
		segment_pixels = beat_parameters.pulse_width + beat_parameters.pulse_gap;
		segment_offset += (float)beat_parameters.pulse_flow / 100;
		if (segment_offset > segment_pixels || segment_offset < -segment_pixels) {
			segment_offset = 0;
		}

		// Render
		for (int i = 0; i < led_number; i++) {
			segment_index = (abs(i-(int)segment_offset+segment_pixels)) % segment_pixels;
			if (segment_index < beat_parameters.pulse_width) {
				pixel_lightlevel = beat_parameters.pulse_light_max;
				pixel_rest = segment_offset - (int)segment_offset;
				// Fade out/in first/last pixel in segment for smoother transition
				if (segment_index == 0) {
					pixel_lightlevel = pixel_lightlevel * (1-pixel_rest);
				}
				if (segment_index == beat_parameters.pulse_width-1) {
					pixel_lightlevel = pixel_lightlevel * pixel_rest;
				}
				// Light level with beat level, but cap it to light min level
				pixel_lightlevel = max(pixel_lightlevel * beat_level, beat_parameters.pulse_light_min);
			} else {
				pixel_lightlevel = 0;
			}
			pixels[i] = hslToRgb(beat_parameters.pulse_hue, 1.0, pixel_lightlevel);
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

