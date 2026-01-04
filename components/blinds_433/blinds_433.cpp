#include "blinds_433.h"
#include "esphome/core/log.h"

namespace esphome {
namespace blinds_433 {

static const char *TAG = "blinds_433";

static constexpr uint32_t SYNC_HALF_US = 295;
static constexpr uint32_t HALF_BIT_US = 640;
static constexpr uint32_t PREAMBLE_US = 6800;
static constexpr uint32_t FRAME_GAP_US = 25000;

// Minimum time between commands (ms) - accounts for full transmission + margin
static constexpr uint32_t COMMAND_SPACING_MS = 500;

static constexpr uint8_t CMD_STOP = 0x01;
static constexpr uint8_t CMD_UP = 0x02;
static constexpr uint8_t CMD_DOWN = 0x04;

void Blinds433Hub::setup() {
  pin_->setup();
  pin_->digital_write(false);
  ESP_LOGCONFIG(TAG, "Blinds 433MHz hub initialized");
}

void Blinds433Hub::loop() {
  if (queue_.empty()) {
    return;
  }

  uint32_t now = millis();
  if (now - last_send_time_ < COMMAND_SPACING_MS) {
    return;
  }

  QueuedCommand cmd = queue_.front();
  queue_.erase(queue_.begin());

  ESP_LOGD(TAG, "Processing queued command (remaining: %d)", queue_.size());
  send_command_(cmd);
  last_send_time_ = millis();
}

void Blinds433Hub::queue_command(uint32_t remote_id, uint8_t blind_id, uint8_t cmd, int repeats) {
  QueuedCommand qc;
  qc.remote_id = remote_id;
  qc.blind_id = blind_id;
  qc.cmd = cmd;
  qc.repeats = repeats;

  queue_.push_back(qc);
  ESP_LOGD(TAG, "Queued command 0x%02X for remote 0x%06X blind %d (queue size: %d)",
           cmd, remote_id, blind_id, queue_.size());
}

void Blinds433Hub::send_command_(const QueuedCommand &cmd) {
  // Initialize counter for this remote if not exists
  if (counters_.find(cmd.remote_id) == counters_.end()) {
    for (int i = 0; i < 16; i++) {
      counters_[cmd.remote_id][i] = 0x0410;
    }
  }

  uint8_t data[8];
  data[0] = 0x81;
  data[1] = cmd.cmd;

  uint16_t counter = counters_[cmd.remote_id][cmd.blind_id];
  data[2] = counter >> 8;
  data[3] = counter & 0xFF;
  counters_[cmd.remote_id][cmd.blind_id]++;

  data[4] = ((cmd.remote_id >> 12) & 0xF0) | (cmd.blind_id & 0x0F);
  data[5] = (cmd.remote_id >> 8) & 0xFF;
  data[6] = cmd.remote_id & 0xFF;

  data[7] = (data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + 0x2B) & 0xFF;

  ESP_LOGD(TAG, "Sending command 0x%02X to remote 0x%06X blind %d", cmd.cmd, cmd.remote_id, cmd.blind_id);

  for (int i = 0; i < cmd.repeats; i++) {
    send_frame_(data, i);
    delayMicroseconds(FRAME_GAP_US);
  }
}

void Blinds433Hub::send_bit_(uint8_t b) {
  if (b) {  // 1 = LOW-HIGH
    pin_->digital_write(false);
    delayMicroseconds(HALF_BIT_US);
    pin_->digital_write(true);
    delayMicroseconds(HALF_BIT_US);
  } else {  // 0 = HIGH-LOW
    pin_->digital_write(true);
    delayMicroseconds(HALF_BIT_US);
    pin_->digital_write(false);
    delayMicroseconds(HALF_BIT_US);
  }
}

void Blinds433Hub::send_frame_(uint8_t *data, uint32_t frame_num) {
  const uint32_t sync_count = frame_num == 0 ? 434 : 59;

  for (uint32_t i = 0; i < sync_count; i++) {
    pin_->digital_write(true);
    delayMicroseconds(SYNC_HALF_US);
    pin_->digital_write(false);
    delayMicroseconds(SYNC_HALF_US);
  }

  pin_->digital_write(true);
  delayMicroseconds(PREAMBLE_US);

  pin_->digital_write(false);
  delayMicroseconds(HALF_BIT_US);

  for (int i = 0; i < 8; i++) {
    for (int j = 7; j >= 0; j--) {
      send_bit_((data[i] >> j) & 1);
    }
  }

  pin_->digital_write(false);
}

cover::CoverTraits Blinds433Cover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(true);
  traits.set_supports_position(false);
  traits.set_supports_tilt(false);
  traits.set_supports_stop(true);
  return traits;
}

void Blinds433Cover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    hub_->queue_command(remote_id_, blind_id_, CMD_STOP);
  }
  if (call.get_position().has_value()) {
    float pos = *call.get_position();
    if (pos == cover::COVER_OPEN) {
      hub_->queue_command(remote_id_, blind_id_, CMD_UP);
    } else if (pos == cover::COVER_CLOSED) {
      hub_->queue_command(remote_id_, blind_id_, CMD_DOWN);
    }
  }
}

}  // namespace blinds_433
}  // namespace esphome
