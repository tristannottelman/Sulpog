#ifndef DEVICE_KEY_H
#define DEVICE_KEY_H

#include <stdint.h>
#include <stdio.h>

extern uint8_t MAC[6];
extern uint8_t DEVICE_KEY[16];
extern uint8_t BLOB[256];

void set_device(uint8_t device);

#endif
