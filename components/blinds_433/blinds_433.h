#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/cover/cover.h"
#include <map>

namespace esphome {
namespace blinds_433 {

class Blinds433Hub : public Component {
 public:
  void set_pin(GPIOPin *pin) { pin_ = pin; }
  void setup() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void send_command(uint32_t remote_id, uint8_t blind_id, uint8_t cmd, int repeats = 6);

 protected:
  void send_bit_(uint8_t b);
  void send_frame_(uint8_t *data, uint32_t frame_num);

  GPIOPin *pin_;
  std::map<uint32_t, uint16_t[16]> counters_;
};

class Blinds433Cover : public cover::Cover, public Component {
 public:
  void set_hub(Blinds433Hub *hub) { hub_ = hub; }
  void set_remote_id(uint32_t remote_id) { remote_id_ = remote_id; }
  void set_blind_id(uint8_t blind_id) { blind_id_ = blind_id; }

  void setup() override {}
  cover::CoverTraits get_traits() override;
  void control(const cover::CoverCall &call) override;

 protected:
  Blinds433Hub *hub_;
  uint32_t remote_id_;
  uint8_t blind_id_;
};

}  // namespace blinds_433
}  // namespace esphome
