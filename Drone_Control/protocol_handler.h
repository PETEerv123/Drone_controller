#ifndef SRC_signal_H_
#define SRC_signal_H_

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

namespace bfs {

struct signalData {
  bool lost_frame;
  bool failsafe;
  bool ch17, ch18;
  static constexpr int8_t NUM_CH = 16;
  int16_t ch[NUM_CH];
};

class signalRx {
public:
#if defined(ESP32)
  signalRx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
           const bool inv)
    : uart_(bus), inv_(inv), rxpin_(rxpin), txpin_(txpin) {}
  signalRx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
           const bool inv, const bool fast)
    : uart_(bus), inv_(inv), fast_(fast),
      rxpin_(rxpin), txpin_(txpin) {}
#else
  explicit signalRx(HardwareSerial *bus)
    : uart_(bus) {}
  signalRx(HardwareSerial *bus, const bool inv)
    : uart_(bus), inv_(inv) {}
  signalRx(HardwareSerial *bus, const bool inv, const bool fast)
    : uart_(bus),
      inv_(inv),
      fast_(fast) {}
#endif
  void Begin();
  bool Read();
  inline signalData data() const {
    return data_;
  }

private:
  /* Communication */
  HardwareSerial *uart_;
  bool inv_ = true;
  bool fast_ = false;
#if defined(ESP32)
  int8_t rxpin_, txpin_;
#endif
  int32_t baud_ = 100000;
  /* Message len */
  static constexpr int8_t PAYLOAD_LEN_ = 23;
  static constexpr int8_t HEADER_LEN_ = 1;
  static constexpr int8_t FOOTER_LEN_ = 1;
  /* signal message defs */
  static constexpr int8_t NUM_signal_CH_ = 16;
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t FOOTER2_ = 0x04;
  static constexpr uint8_t CH17_MASK_ = 0x01;
  static constexpr uint8_t CH18_MASK_ = 0x02;
  static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
  static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
  /* Parsing state tracking */
  int8_t state_ = 0;
  uint8_t prev_byte_ = FOOTER_;
  uint8_t cur_byte_;
  /* Buffer for storing messages */
  uint8_t buf_[25];
  /* Data */
  bool new_data_;
  signalData data_;
  bool Parse();
};

class signalTx {
public:
#if defined(ESP32)
  signalTx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
           const bool inv)
    : uart_(bus), inv_(inv), rxpin_(rxpin), txpin_(txpin) {}
  signalTx(HardwareSerial *bus, const int8_t rxpin, const int8_t txpin,
           const bool inv, const bool fast)
    : uart_(bus), inv_(inv), fast_(fast),
      rxpin_(rxpin), txpin_(txpin) {}
#else
  explicit signalTx(HardwareSerial *bus)
    : uart_(bus) {}
  signalTx(HardwareSerial *bus, const bool inv)
    : uart_(bus), inv_(inv) {}
  signalTx(HardwareSerial *bus, const bool inv, const bool fast)
    : uart_(bus),
      inv_(inv),
      fast_(fast) {}
#endif
  void Begin();
  void Write();
  inline void data(const signalData &data) {
    data_ = data;
  }
  inline signalData data() const {
    return data_;
  }

private:
  /* Communication */
  HardwareSerial *uart_;
  bool inv_ = true;
  bool fast_ = false;
#if defined(ESP32)
  int8_t rxpin_, txpin_;
#endif
  int32_t baud_ = 100000;
  /* Message len */
  static constexpr int8_t BUF_LEN_ = 25;
  /* signal message defs */
  static constexpr int8_t NUM_signal_CH_ = 16;
  static constexpr uint8_t HEADER_ = 0x0F;
  static constexpr uint8_t FOOTER_ = 0x00;
  static constexpr uint8_t FOOTER2_ = 0x04;
  static constexpr uint8_t CH17_MASK_ = 0x01;
  static constexpr uint8_t CH18_MASK_ = 0x02;
  static constexpr uint8_t LOST_FRAME_MASK_ = 0x04;
  static constexpr uint8_t FAILSAFE_MASK_ = 0x08;
  /* Data */
  uint8_t buf_[BUF_LEN_];
  signalData data_;
};

}  // namespace bfs

#endif  // SRC_signal_H_
