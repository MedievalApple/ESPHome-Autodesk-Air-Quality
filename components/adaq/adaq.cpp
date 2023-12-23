#include "adaq.h"
#include "esphome/core/log.h"

namespace esphome {
namespace adaq {

static const char *const TAG = "adaq";

static const uint8_t ADAQ_RESPONSE_HEADER[] = {0x16, 0x11, 0x0B};
static const uint8_t ADAQ_REQUEST[] = {0x11, 0x02, 0x0B, 0x01, 0xE1};

void ADAQComponent::setup() {
  // because this implementation is currently rx-only, there is nothing to setup
}

void ADAQComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "ADAQ:");
  LOG_SENSOR("  ", "PM2.5", this->pm_2_5_sensor_);
  LOG_UPDATE_INTERVAL(this);
  this->check_uart_settings(9600);
}

void ADAQComponent::update() {
  ESP_LOGV(TAG, "sending measurement request");
  this->write_array(ADAQ_REQUEST, sizeof(ADAQ_REQUEST));
}

void ADAQComponent::loop() {
  while (this->available() != 0) {
    this->read_byte(&this->data_[this->data_index_]);
    ESP_LOGV(TAG, "What is this: {this}");
    auto check = this->check_byte_();
    if (!check.has_value()) {
      // finished
      this->parse_data_();
      this->data_index_ = 0;
    } else if (!*check) {
      // wrong data
      ESP_LOGV(TAG, "Byte %i of received data frame is invalid.", this->data_index_);
      this->parse_data_();
      this->data_index_ = 0;
    } else {
      // next byte
      this->data_index_++;
    }
  }
}

float ADAQComponent::get_setup_priority() const { return setup_priority::DATA; }

uint8_t ADAQComponent::adaq_checksum_(const uint8_t *command_data, uint8_t length) const {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < length; i++) {
    sum += command_data[i];
  }
  return sum;
}

optional<bool> ADAQComponent::check_byte_() const {
  uint8_t index = this->data_index_;
  uint8_t byte = this->data_[index];

  // index 0..2 are the fixed header
  if (index < sizeof(ADAQ_RESPONSE_HEADER)) {
    return byte == ADAQ_RESPONSE_HEADER[index];
  }

  // just some additional notes here:
  // index 3..4 is unused
  // index 5..6 is our PM2.5 reading (3..6 is called DF1-DF4 in the datasheet at
  // http://www.jdscompany.co.kr/download.asp?gubun=07&filename=ADAQ_LED_PARTICLE_SENSOR_MODULE_SPECIFICATIONS.pdf
  // that datasheet goes on up to DF16, which is unused for ADAQ but used in ADAQK
  // so this code should be trivially extensible to support that one later
  if (index < (sizeof(ADAQ_RESPONSE_HEADER) + 16))
    return true;

  // checksum
  if (index == (sizeof(ADAQ_RESPONSE_HEADER) + 16)) {
    uint8_t checksum = adaq_checksum_(this->data_, sizeof(ADAQ_RESPONSE_HEADER) + 17);
    if (checksum != 0) {
      ESP_LOGW(TAG, "ADAQ checksum is wrong: %02x, expected zero", checksum);
      return false;
    }
    return {};
  }

  return false;
}

void ADAQComponent::parse_data_() {
  const int pm_2_5_concentration = this->get_16_bit_uint_(5);

  ESP_LOGD(TAG, "Got PM2.5 Concentration: %d µg/m³", pm_2_5_concentration);

  if (this->pm_2_5_sensor_ != nullptr) {
    this->pm_2_5_sensor_->publish_state(pm_2_5_concentration);
  }
}

uint16_t ADAQComponent::get_16_bit_uint_(uint8_t start_index) const {
  return encode_uint16(this->data_[start_index], this->data_[start_index + 1]);
}

}  // namespace adaq
}  // namespace esphome