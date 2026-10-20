// Host stub of the ULD platform layer, only used to prove that the vendored ULD
// sources (C) compile and link together with the component's platform.h.
#include "platform.h"
extern "C" {
uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *, uint16_t, uint8_t *) { return 0; }
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *, uint16_t, uint8_t) { return 0; }
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *, uint16_t, uint8_t *, uint32_t) { return 0; }
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *, uint16_t, uint8_t *, uint32_t) { return 0; }
uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *) { return 0; }
void VL53L5CX_SwapBuffer(uint8_t *, uint16_t) {}
uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *, uint32_t) { return 0; }
}
