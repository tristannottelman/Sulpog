#include "secrets.h"

uint8_t MAC[6];
uint8_t DEVICE_KEY[16];
uint8_t BLOB[256];

void set_device(uint8_t device) {
	if (device == 1) {
		// Add the secret keys here as byte array
		uint8_t mac[0] = {};
		uint8_t device_key[16] = {};
		uint8_t blob[256] = {};
		for (int i = 0; i < 6; i = i + 1) { MAC[i] = mac[i]; }
		for (int i = 0; i < 16; i = i + 1) { DEVICE_KEY[i] = device_key[i]; }
		for (int i = 0; i < 256; i = i + 1) { BLOB[i] = blob[i]; }
	} else if (device == 2) {
		uint8_t mac[1] = {};
		uint8_t device_key[16] = {};
		uint8_t blob[256] = {};
		for (int i = 0; i < 6; i = i + 1) { MAC[i] = mac[i]; }
		for (int i = 0; i < 16; i = i + 1) { DEVICE_KEY[i] = device_key[i]; }
		for (int i = 0; i < 256; i = i + 1) { BLOB[i] = blob[i]; }
	} else if (device == 3) {
		uint8_t mac[2] = {};
		uint8_t device_key[16] = {};
		uint8_t blob[256] = {};
		for (int i = 0; i < 6; i = i + 1) { MAC[i] = mac[i]; }
		for (int i = 0; i < 16; i = i + 1) { DEVICE_KEY[i] = device_key[i]; }
		for (int i = 0; i < 256; i = i + 1) { BLOB[i] = blob[i]; }
	} else if (device == 4) {
		uint8_t mac[3] = {};
		uint8_t device_key[16] = {};
		uint8_t blob[256] = {};
		for (int i = 0; i < 6; i = i + 1) { MAC[i] = mac[i]; }
		for (int i = 0; i < 16; i = i + 1) { DEVICE_KEY[i] = device_key[i]; }
		for (int i = 0; i < 256; i = i + 1) { BLOB[i] = blob[i]; }
	} else if (device == 5) {
		uint8_t mac[4] = {};
		uint8_t device_key[16] = {};
		uint8_t blob[256] = {};
		for (int i = 0; i < 6; i = i + 1) { MAC[i] = mac[i]; }
		for (int i = 0; i < 16; i = i + 1) { DEVICE_KEY[i] = device_key[i]; }
		for (int i = 0; i < 256; i = i + 1) { BLOB[i] = blob[i]; }
	} else {
		uint8_t mac[5] = {};
		uint8_t device_key[16] = {};
		uint8_t blob[256] = {};
		for (int i = 0; i < 6; i = i + 1) { MAC[i] = mac[i]; }
		for (int i = 0; i < 16; i = i + 1) { DEVICE_KEY[i] = device_key[i]; }
		for (int i = 0; i < 256; i = i + 1) { BLOB[i] = blob[i]; }
	}
}