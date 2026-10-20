// ESPHome implementation of the VL53L5CX ULD platform layer.
//
// Register addresses are 16-bit big-endian. Large transfers are split into
// chunks of at most `chunk_size` bytes; the register address is advanced by
// the chunk length for every chunk (the same approach used by ST reference
// ports). Every failure is reported with the ULD "major error" code (255).
#include "platform.h"

#include "esphome/components/i2c/i2c.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"

using esphome::i2c::ERROR_OK;
using esphome::i2c::I2CDevice;

namespace {

constexpr uint8_t PLATFORM_ERROR = 255;
// Feed the watchdog every N chunks during long transfers (firmware upload).
constexpr uint32_t WDT_FEED_EVERY_CHUNKS = 16;

inline I2CDevice *dev_of(VL53L5CX_Platform *p) { return static_cast<I2CDevice *>(p->i2c_device); }

uint8_t write_chunked(VL53L5CX_Platform *p, uint16_t reg, const uint8_t *data, uint32_t size) {
  I2CDevice *dev = dev_of(p);
  if (dev == nullptr || p->tx_buffer == nullptr || p->chunk_size == 0)
    return PLATFORM_ERROR;
  uint32_t done = 0;
  uint32_t chunks = 0;
  while (done < size) {
    uint32_t n = size - done;
    if (n > p->chunk_size)
      n = p->chunk_size;
    const uint16_t r = static_cast<uint16_t>(reg + done);
    p->tx_buffer[0] = static_cast<uint8_t>(r >> 8);
    p->tx_buffer[1] = static_cast<uint8_t>(r & 0xFF);
    memcpy(&p->tx_buffer[2], &data[done], n);
    if (dev->write(p->tx_buffer, n + 2) != ERROR_OK)
      return PLATFORM_ERROR;
    done += n;
    if ((++chunks % WDT_FEED_EVERY_CHUNKS) == 0)
      esphome::App.feed_wdt();
  }
  return 0;
}

}  // namespace

extern "C" {

uint8_t VL53L5CX_RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
  I2CDevice *dev = dev_of(p_platform);
  if (dev == nullptr || p_platform->chunk_size == 0)
    return PLATFORM_ERROR;
  uint32_t done = 0;
  uint32_t chunks = 0;
  while (done < size) {
    uint32_t n = size - done;
    if (n > p_platform->chunk_size)
      n = p_platform->chunk_size;
    const uint16_t r = static_cast<uint16_t>(RegisterAdress + done);
    const uint8_t addr[2] = {static_cast<uint8_t>(r >> 8), static_cast<uint8_t>(r & 0xFF)};
    if (dev->write_read(addr, 2, &p_values[done], n) != ERROR_OK)
      return PLATFORM_ERROR;
    done += n;
    if ((++chunks % WDT_FEED_EVERY_CHUNKS) == 0)
      esphome::App.feed_wdt();
  }
  return 0;
}

uint8_t VL53L5CX_RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_value) {
  return VL53L5CX_RdMulti(p_platform, RegisterAdress, p_value, 1);
}

uint8_t VL53L5CX_WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t *p_values, uint32_t size) {
  return write_chunked(p_platform, RegisterAdress, p_values, size);
}

uint8_t VL53L5CX_WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAdress, uint8_t value) {
  return write_chunked(p_platform, RegisterAdress, &value, 1);
}

uint8_t VL53L5CX_Reset_Sensor(VL53L5CX_Platform *p_platform) {
  (void) p_platform;  // Hardware reset is handled by the component (LPn pin).
  return 0;
}

void VL53L5CX_SwapBuffer(uint8_t *buffer, uint16_t size) {
  // Reverses the byte order of every 32-bit word (big-endian firmware format to
  // little-endian host words). Same behaviour as the ST reference platform.
  for (uint32_t i = 0; i + 3 < size; i += 4) {
    const uint32_t tmp = (static_cast<uint32_t>(buffer[i]) << 24) | (static_cast<uint32_t>(buffer[i + 1]) << 16) |
                         (static_cast<uint32_t>(buffer[i + 2]) << 8) | static_cast<uint32_t>(buffer[i + 3]);
    memcpy(&buffer[i], &tmp, 4);
  }
}

uint8_t VL53L5CX_WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs) {
  (void) p_platform;
  // Blocking by ULD design (used during init/start/stop only). Sliced so the
  // watchdog is fed.
  while (TimeMs > 0) {
    const uint32_t step = TimeMs > 10 ? 10 : TimeMs;
    esphome::delay(step);
    esphome::App.feed_wdt();
    TimeMs -= step;
  }
  return 0;
}

}  // extern "C"
