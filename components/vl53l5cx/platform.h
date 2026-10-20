/*
 * ESPHome platform layer header for the STMicroelectronics VL53L5CX ULD.
 *
 * This file replaces the empty "platform.h" shell shipped with the ULD
 * (STSW-IMG023 2.0.1). The platform macros below are the only compile-time
 * configuration of the ULD used by this component.
 *
 * Must stay valid C and C++ (it is included by the ULD C sources).
 */
#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Platform structure, visible to the platform layer only. */
typedef struct {
  /* 8-bit I2C address as used by ST documentation (0x52). Informational. */
  uint16_t address;
  /* Pointer to the esphome::i2c::I2CDevice used for all transfers. */
  void *i2c_device;
  /* Scratch buffer of (chunk_size + 2) bytes used for chunked writes. */
  uint8_t *tx_buffer;
  /* Maximum number of payload bytes per I2C transaction (>= 16). */
  uint16_t chunk_size;
} VL53L5CX_Platform;

/*
 * Number of targets per zone (UM2884 4.10). Compile-time only. ESPHome sets
 * it from the YAML key `targets_per_zone` using a build flag.
 */
#ifndef VL53L5CX_NB_TARGET_PER_ZONE
#define VL53L5CX_NB_TARGET_PER_ZONE 4U
#endif
#if (VL53L5CX_NB_TARGET_PER_ZONE < 1) || (VL53L5CX_NB_TARGET_PER_ZONE > 4)
#error "VL53L5CX_NB_TARGET_PER_ZONE must be between 1 and 4"
#endif

/* VL53L5CX_USE_RAW_FORMAT is intentionally NOT defined (UM2884 5.4).
 * All VL53L5CX_DISABLE_* output macros are intentionally NOT defined:
 * every ULD output is used by this component. */

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value);
uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value);
uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size);
uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform);
void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size);
uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);

#ifdef __cplusplus
}
#endif

#endif /* _PLATFORM_H_ */
