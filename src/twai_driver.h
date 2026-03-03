#pragma once

#include <cstdint>

bool twai_init();
bool twai_send(uint32_t id, uint8_t len, const uint8_t *data);
bool twai_receive(uint32_t *id, uint8_t *len, uint8_t *data);
void twai_set_silent_mode(bool silent);
