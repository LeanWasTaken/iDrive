#include "can_tx.h"
#include "can_protocol.h"
#include "idrive_controller.h"
#include "twai_driver.h"

#include <Arduino.h>

void sendKeepAlive() {
    twai_send(ID_KEEPALIVE, 8, KEEPALIVE_FRAME);
    state.lastKeepAliveTime = millis();
}
