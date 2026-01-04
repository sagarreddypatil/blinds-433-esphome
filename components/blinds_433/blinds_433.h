#pragma once

#include "esphome/components/cover/cover.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include <map>
#include <vector>

namespace esphome {
namespace blinds_433 {

struct QueuedCommand {
  uint32_t remote_id;
  uint8_t blind_id;
  uint8_t cmd;
  int repeats;
};

class Blinds433Hub : public Component {
public:
  void set_pin(GPIOPin *pin) { pin_ = pin; }
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void queue_command(uint32_t remote_id, uint8_t blind_id, uint8_t cmd,
                     int repeats = 6);

protected:
  void send_command_(const QueuedCommand &cmd);
  void send_bit_(uint8_t b);
  void send_frame_(uint8_t *data, uint32_t frame_num);

  GPIOPin *pin_;
  std::map<uint32_t, uint16_t[16]> counters_;
  std::vector<QueuedCommand> queue_;
  uint32_t last_send_time_{0};
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

} // namespace blinds_433
} // namespace esphome