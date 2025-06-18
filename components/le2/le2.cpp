#include "le2.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#define STATE_BOOTUP_WAIT "Waiting to boot up"
#define STATE_METER_NOT_FOUND "Meter not found"
#define STATE_METER_FOUND "Meter found"
#define STATE_OK "OK"
#define STATE_PARTIAL_OK "Read partial data"
#define STATE_DATA_FAIL "Unable to read data"

#define MASK_GOT_CONSUMPTION 0b001
#define MASK_GOT_GRID_DATA 0b010
#define MASK_GOT_METER_INFO 0b100

#define MESSAGE_CRC_IEC 0x0F47

namespace esphome {
namespace le2 {

static const char *TAG = "LE2";

static const uint8_t MAGIC = 0xAA;
static const uint8_t ESC_START = 0x55;
static const uint8_t ESC_AA = 0x01;
static const uint8_t ESC_55 = 0x02;

// // avoid communications until properly booted
// static constexpr uint8_t bootupWaitUpdate = 10;

static constexpr uint32_t NO_GOOD_READS_TIMEOUT_MS = 5 * 60 * 1000;  // 5 minutes

static constexpr size_t RX_BUFFER_SIZE = 256u;
static std::array<uint8_t, RX_BUFFER_SIZE> rxBuffer;

enum class ComType : uint8_t { Enq = 0x01 };

#pragma pack(1)
typedef struct {
  uint8_t magic;      // 0xAA
  uint8_t length;     // Length of the frame, including header and CRC16
  uint32_t address;   // Network address of the meter
  uint32_t password;  // Password for the meter, usually 0x00000000
  uint8_t com;        // Command type: 0x01 for enquiry, ...
  uint8_t function;   // Request function
} le2_frame_header_t;

typedef struct {
  le2_frame_header_t header;
  uint16_t crc16;
} le2_request_command_t;

// responses

typedef struct {
  le2_frame_header_t header;
  uint32_t production_date;
  uint32_t serial_number;
  uint32_t network_address;
  uint8_t type;
  uint8_t hw_ver;
  uint8_t fw_ver;
  uint8_t _unk;
  uint64_t error;
  uint16_t crc16;
} le2_response_meter_info_t;

typedef struct {
  le2_frame_header_t header;
  uint8_t currentTariff;
  float consumption[4][8];  // [consumption_type][tariff] for A+, A-, R+, R- (0-3)
  uint16_t crc16;
} le2_response_consumed_energy_t;

typedef struct {
  le2_frame_header_t header;
  uint32_t dtm;
  float u;
  float measurements[2][4];  // [phase][measurement]:  [phase,neutral][I, P_A, P_R, cos_phi]
  float freq;
  uint16_t crc16;
} le2_response_grid_parameters_t;

#pragma pack(0)

static le2_request_command_t txBuffer;

float LE2Component::get_setup_priority() const { return setup_priority::AFTER_WIFI; }

void LE2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "LE-2:");
  LOG_UPDATE_INTERVAL(this);
  ESP_LOGCONFIG(TAG, "  Meter address requested: %u", this->requested_meter_address_);
  ESP_LOGCONFIG(TAG, "  Receive timeout: %.1fs", this->receive_timeout_ / 1e3f);
  ESP_LOGCONFIG(TAG, "  Update interval: %.1fs", this->update_interval_ / 1e3f);
  LOG_PIN("  Flow Control Pin: ", this->flow_control_pin_);
  // LOG_SENSOR("  ", "Active power", this->active_power_);
  // LOG_SENSOR("  ", "Energy total", this->energy_total_);
  // LOG_SENSOR("  ", "Energy consumed total", this->energy_total_);
  // LOG_SENSOR("  ", "Energy consumed tariff 1", this->energy_t1_);
  // LOG_SENSOR("  ", "Energy consumed tariff 2", this->energy_t2_);
  // LOG_SENSOR("  ", "Energy consumed tariff 3", this->energy_t3_);
  // LOG_SENSOR("  ", "Energy consumed tariff 4", this->energy_t4_);
  // LOG_TEXT_SENSOR("  ", "Electricity tariff", this->tariff_);
  // LOG_TEXT_SENSOR("  ", "Date", this->date_);
  // LOG_TEXT_SENSOR("  ", "Time", this->time_);
  // LOG_TEXT_SENSOR("  ", "Network address", this->network_address_);
  // LOG_TEXT_SENSOR("  ", "Serial number", this->serial_nr_);
  //  this->check_uart_settings(9600, 1, uart::UART_CONFIG_PARITY_EVEN, 8);
  ESP_LOGCONFIG(TAG, "Data errors %d, proper reads %d", this->data_.readErrors, this->data_.properReads);
}

void LE2Component::set_tariff_consumption_sensor(uint8_t consumption_type, uint8_t tariff, sensor::Sensor *sensor) {
  if (consumption_type > 3 || tariff > 7) {
    ESP_LOGE(TAG, "Invalid consumption type %d or tariff %d", consumption_type, tariff);
    return;
  }
  this->tariff_consumption_[consumption_type][tariff] = sensor;
}

void LE2Component::set_phase_measurements_sensor(uint8_t phase, uint8_t measurement, sensor::Sensor *sensor) {
  if (phase > 1 || measurement > 3) {
    ESP_LOGE(TAG, "Invalid phase %d or measurement %d", phase, measurement);
    return;
  }
  this->phase_measurements_[phase][measurement] = sensor;
}

void LE2Component::setup() {
  if (this->reading_state_text_sensor_ != nullptr) {
    this->reading_state_text_sensor_->publish_state(STATE_BOOTUP_WAIT);
  }

  this->set_timeout(1000, [this]() { this->state_ = State::IDLE; });
}

void LE2Component::loop() {
  if (!this->is_ready())
    return;

  switch (this->state_) {
    case State::NOT_INITIALIZED: {
      this->log_state_();
    } break;

    case State::IDLE: {
      this->log_state_();
      uint32_t now = millis();
      if (now - this->data_.lastGoodRead_ms > NO_GOOD_READS_TIMEOUT_MS) {
        ESP_LOGE(TAG, "Rebooting due to no good reads from the meter for 5 minutes...");
        delay(1000);
        App.reboot();
      }
    } break;

    case State::WAITING_FOR_RESPONSE: {
      this->log_state_(&this->next_state_);
      process_response();
      // process_response will transition to next_state_ when done
    } break;

    case State::GET_METER_INFO: {
      this->log_state_();
      start_async_request(EnqCmd::MeterInfo, sizeof(le2_response_meter_info_t), State::GET_GRID_PARAMETERS);
    } break;

    case State::GET_GRID_PARAMETERS: {
      this->log_state_();
      start_async_request(EnqCmd::GridParameters, sizeof(le2_response_grid_parameters_t), State::GET_ENERGY_CONSUMED);
    } break;

    case State::GET_ENERGY_CONSUMED: {
      this->log_state_();
      start_async_request(EnqCmd::ConsumedEnergy, sizeof(le2_response_consumed_energy_t), State::PUBLISH_INFO);
    } break;

    case State::PUBLISH_INFO: {
      this->log_state_();
      // // Update status text sensor based on the data we were able to collect
      // if (this->data_.got == (MASK_GOT_DATE_TIME | MASK_GOT_ACTIVE_POWER | MASK_GOT_ENERGY)) {
      //   this->data_.failure = false;
      //   this->data_.initialized = true;
      //   if (this->reading_state_ != nullptr) {
      //     this->reading_state_->publish_state(STATE_OK);
      //   }
      //   this->data_.lastGoodRead_ms = millis();
      // } else {
      //   ESP_LOGW(TAG, "Got no or partial data %o", this->data_.got);
      //   this->data_.failure = true;
      //   if (this->reading_state_ != nullptr) {
      //     this->reading_state_->publish_state((this->data_.got == 0) ? STATE_DATA_FAIL : STATE_PARTIAL_OK);
      //   }
      // }
      if (this->data_.got) {
        this->data_.lastGoodRead_ms = millis();
      }

      if (this->data_.meterFound) {
        if (this->network_address_text_sensor_ != nullptr) {
          this->network_address_text_sensor_->publish_state(to_string(this->data_.meterInfo.network_address));
        }
        if (this->serial_nr_text_sensor_ != nullptr) {
          this->serial_nr_text_sensor_->publish_state(to_string(this->data_.meterInfo.serial_number));
        }
        if (this->error_code_text_sensor_ != nullptr) {
          this->error_code_text_sensor_->publish_state(to_string(this->data_.meterInfo.error_code));
        }
        if (this->reading_state_text_sensor_ != nullptr && !this->data_.initialized) {
          this->reading_state_text_sensor_->publish_state(STATE_METER_FOUND);
        }
        if (this->about_text_sensor_ != nullptr) {
          this->about_text_sensor_->publish_state(this->data_.meterInfo.about_str);
        }
      } else {
        if (this->reading_state_text_sensor_ != nullptr && !this->data_.initialized) {
          this->reading_state_text_sensor_->publish_state(STATE_METER_NOT_FOUND);
        }
      }

      if (this->data_.got & MASK_GOT_CONSUMPTION) {
        for (uint8_t consumption_type = 0; consumption_type < 4; ++consumption_type) {
          for (uint8_t tariff = 0; tariff < 8; ++tariff) {
            if (this->tariff_consumption_[consumption_type][tariff] != nullptr) {
              this->tariff_consumption_[consumption_type][tariff]->publish_state(
                  this->data_.energy.consumption[consumption_type][tariff]);
            }
          }
        }
        if (this->electricity_tariff_text_sensor_ != nullptr) {
          char tariff_str[3];
          tariff_str[0] = 'T';
          tariff_str[1] = '0' + (this->data_.energy.currentTariff & 0b111);
          tariff_str[2] = 0;
          this->electricity_tariff_text_sensor_->publish_state(tariff_str);
        }

        if (this->date_text_sensor_ != nullptr) {
          this->date_text_sensor_->publish_state(this->data_.dateStr);
        }
        if (this->time_text_sensor_ != nullptr) {
          this->time_text_sensor_->publish_state(this->data_.timeStr);
        }
        if (this->datetime_text_sensor_ != nullptr) {
          this->datetime_text_sensor_->publish_state(this->data_.dateTimeStr);
        }
      }

      if (this->data_.got & MASK_GOT_GRID_DATA) {
        if (this->frequency_sensor_ != nullptr) {
          this->frequency_sensor_->publish_state(this->data_.grid.freq);
        }
        if (this->voltage_sensor_ != nullptr) {
          this->voltage_sensor_->publish_state(this->data_.grid.u);
        }

        for (uint8_t phase = 0; phase < 2; ++phase) {
          for (uint8_t measurement = 0; measurement < 4; ++measurement) {
            if (this->phase_measurements_[phase][measurement] != nullptr) {
              this->phase_measurements_[phase][measurement]->publish_state(
                  this->data_.grid.measurements[phase][measurement]);
            }
          }
        }
      }

      ESP_LOGD(TAG, "Data errors %d, proper reads %d", this->data_.readErrors, this->data_.properReads);
      this->state_ = State::IDLE;
    } break;

    default:
      break;
  }
}

void LE2Component::update() {
  if (!this->is_ready() || this->state_ != State::IDLE) {
    ESP_LOGV(TAG, "Update: Component not ready yet");
    return;
  }
  ESP_LOGV(TAG, "Update: Initiating new data collection");
  this->data_.got = 0;
  this->request_tracker_.reset();
  this->state_ = State::GET_METER_INFO;
}

void LE2Component::send_enquiry_command(EnqCmd cmd) {
  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(true);

  txBuffer.header.magic = 0xAA;  // magic
  txBuffer.header.length = sizeof(le2_request_command_t) - 1;
  txBuffer.header.address = this->requested_meter_address_;
  txBuffer.header.password = this->password_;  // 0x0;
  txBuffer.header.com = static_cast<uint8_t>(ComType::Enq);
  txBuffer.header.function = static_cast<uint8_t>(cmd);

  // minus 2 bytes for CRC minus 1 for frame separator byte (0xAA)
  txBuffer.crc16 = crc_16_iec((const uint8_t *) &txBuffer + 1, sizeof(le2_request_command_t) - 3);

  // todo: SLIP encode! AA -> 55 01, 55 -> 55 02

  write_array((const uint8_t *) &txBuffer, sizeof(le2_request_command_t));
  flush();

  if (this->flow_control_pin_ != nullptr)
    this->flow_control_pin_->digital_write(false);

  ESP_LOGVV(TAG, "TX: %s", format_hex_pretty((const uint8_t *) &txBuffer, sizeof(le2_request_command_t)).c_str());
}

void LE2Component::start_async_request(EnqCmd cmd, uint16_t expected_size, State next_state) {
  this->request_tracker_.current_cmd = cmd;
  this->request_tracker_.expected_size = expected_size;
  this->request_tracker_.start_time = millis();
  this->request_tracker_.bytes_read = 0;
  this->next_state_ = next_state;

  send_enquiry_command(cmd);
  this->state_ = State::WAITING_FOR_RESPONSE;
}

bool LE2Component::process_response() {
  auto &tracker = this->request_tracker_;
  auto now = millis();

  static auto got = 0;
  if (got != tracker.bytes_read) {
    got = tracker.bytes_read;
    ESP_LOGVV(TAG, "process_resp(), read=%d, exp=%d", tracker.bytes_read, tracker.expected_size);
  }

  // Check timeout
  if (now - tracker.start_time > this->receive_timeout_) {
    ESP_LOGE(TAG, "Response timeout, most likely password is wrong or meter is not responding");
    this->data_.readErrors++;
    if (tracker.bytes_read > 0) {
      ESP_LOGVV(TAG, "RX: %s",
                format_hex_pretty(static_cast<const uint8_t *>(rxBuffer.data()), tracker.bytes_read).c_str());
    }
    // Return to main FSM with failure
    this->state_ = this->next_state_;
    return false;
  }

  // Read available data
  while (available() > 0 && tracker.bytes_read < tracker.expected_size) {
    int currentByte = read();
    if (currentByte >= 0) {
      rxBuffer[tracker.bytes_read++] = static_cast<uint8_t>(currentByte);
      if (currentByte == ESC_START) {
        tracker.escape_seq_found++;
        tracker.expected_size++;
      }
    }
    yield();
  }

  // ESP_LOGV(TAG, "Bytes read so far: %d of %d", tracker.bytes_read, tracker.expected_size);

  // Check if we have received all expected bytes
  if (tracker.bytes_read != tracker.expected_size) {
    // wait for more data
    return false;
  }

  ESP_LOGV(TAG, "Received all bytes, validating...");
  ESP_LOGVV(TAG, "RX: %s",
            format_hex_pretty(static_cast<const uint8_t *>(rxBuffer.data()), tracker.bytes_read).c_str());

  if (tracker.escape_seq_found > 0) {
    ESP_LOGD(TAG, "Found %d escape sequences", tracker.escape_seq_found);
    // Process escape sequences ESC_START+ESC_AA => MAGIC
    //

    // Input example  : AA 01 02 55 01 04 05
    // Output example : AA 01 02 AA 04 05

    uint8_t *data_ptr = rxBuffer.data();
    uint8_t *end_ptr = data_ptr + tracker.bytes_read;
    uint8_t *write_ptr = data_ptr;

    while (data_ptr < end_ptr) {
      if (*data_ptr == ESC_START) {
        // Found escape sequence, replace with MAGIC.
        *write_ptr = ESC_START;
        data_ptr++;
        if (data_ptr < end_ptr) {
          if (*data_ptr == ESC_AA) {
            *write_ptr = MAGIC;
          } else if (*data_ptr == ESC_55) {
            *write_ptr = ESC_START;  // keep the escape sequence as is
          } else {
            ESP_LOGE(TAG, "Unexpected escape sequence: 0x%02X", *data_ptr);
            // return false; // Invalid escape sequence
          }
          // Skip the next byte (ESC_AA)
          data_ptr++;
        }
        *write_ptr++;
      } else {
        // Copy the byte as is
        *write_ptr++ = *data_ptr++;
      }
    }
    tracker.bytes_read -= tracker.escape_seq_found;     // Adjust the size after processing escape sequences
    tracker.expected_size -= tracker.escape_seq_found;  // Adjust expected size
  }

  // CRC of message + its CRC = 0x0F47
  if (crc_16_iec(rxBuffer.data() + 1, tracker.expected_size - 1) != MESSAGE_CRC_IEC) {
    ESP_LOGE(TAG, "CRC check failed");
    this->data_.readErrors++;
    // Return to main FSM with failure
    this->state_ = this->next_state_;
    return false;
  }

  // Process the received data
  this->data_.properReads++;
  bool result = process_received_data();

  // Return to main FSM with success/failure flag
  this->state_ = this->next_state_;
  return result;
}

bool LE2Component::process_received_data() {
  bool success = true;

  switch (this->request_tracker_.current_cmd) {
    case EnqCmd::ConsumedEnergy: {
      le2_response_consumed_energy_t &res = *(le2_response_consumed_energy_t *) rxBuffer.data();
      this->data_.energy.currentTariff = res.currentTariff + 1;
      ESP_LOGI(TAG, "Cur tariff: T%d, T1 A+ =%f, T2 A+=%f", res.currentTariff + 1, res.consumption[0][0],
               res.consumption[0][1]);
      memcpy(this->data_.energy.consumption, res.consumption, sizeof(res.consumption));
      this->data_.got |= MASK_GOT_CONSUMPTION;
      break;
    }

    case EnqCmd::GridParameters: {
      le2_response_grid_parameters_t &grid = *(le2_response_grid_parameters_t *) rxBuffer.data();

      this->data_.grid.dtm = grid.dtm;
      this->data_.grid.u = grid.u;
      this->data_.grid.freq = grid.freq;
      memcpy(this->data_.grid.measurements, grid.measurements, sizeof(grid.measurements));
      time_t t = grid.dtm;
      strftime(this->data_.dateTimeStr, sizeof(this->data_.dateTimeStr), "%Y-%m-%d %H:%M:%S", localtime(&t));
      strftime(this->data_.dateStr, sizeof(this->data_.dateStr), "%Y-%m-%d", localtime(&t));
      strftime(this->data_.timeStr, sizeof(this->data_.timeStr), "%H:%M:%S", localtime(&t));

      ESP_LOGI(TAG, "Dtm = %s, U = %f, F = %f", this->data_.dateTimeStr, this->data_.grid.u, this->data_.grid.freq);

      this->data_.got |= MASK_GOT_GRID_DATA;
    } break;

    case EnqCmd::MeterInfo: {
      le2_response_meter_info_t &res = *(le2_response_meter_info_t *) rxBuffer.data();
      this->data_.meterInfo.production_date = res.production_date;
      this->data_.meterInfo.serial_number = res.serial_number;
      this->data_.meterInfo.network_address = res.network_address;
      this->data_.meterInfo.type = res.type;
      this->data_.meterInfo.fw_version = res.fw_ver;
      this->data_.meterInfo.hw_version = res.hw_ver;
      this->data_.meterInfo.error_code = res.error;

      time_t d = res.production_date;
      strftime(this->data_.meterInfo.production_date_str, sizeof(this->data_.meterInfo.production_date_str), "%Y-%m-%d",
               localtime(&d));
      snprintf(this->data_.meterInfo.about_str, sizeof(this->data_.meterInfo.about_str),
               "Typ: %u, HW: %u, FW: %u, Prod: %s", res.type, res.hw_ver, res.fw_ver,
               this->data_.meterInfo.production_date_str);

      ESP_LOGI(TAG,
               "Got reply from meter with s/n %u (0x%08X), network address %u, "
               "fw ver. %02X hw ver. %02X, type %02X, error code %llu, production date %s",
               res.serial_number, res.serial_number, res.network_address, res.fw_ver, res.hw_ver, res.type, res.error,
               this->data_.meterInfo.production_date_str);

      if (!this->data_.meterFound) {
        this->data_.meterFound = true;
        requested_meter_address_ = this->data_.meterInfo.network_address;
      }
      this->data_.got |= MASK_GOT_METER_INFO;
      break;
    }

    default:
      ESP_LOGE(TAG, "Unknown command type: %d", static_cast<int>(this->request_tracker_.current_cmd));
      success = false;
      break;
  }

  return success;
}

const char *LE2Component::state_to_string(State state) {
  switch (state) {
    case State::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case State::IDLE:
      return "IDLE";
    case State::WAITING_FOR_RESPONSE:
      return "WAITING_FOR_RESPONSE";
    case State::GET_METER_INFO:
      return "GET_METER_INFO";
    case State::GET_GRID_PARAMETERS:
      return "GET_GRID_PARAMETERS";
    case State::GET_ENERGY_CONSUMED:
      return "GET_ENERGY_CONSUMED";
    case State::PUBLISH_INFO:
      return "PUBLISH_INFO";
    default:
      return "UNKNOWN_STATE";
  }
}

void LE2Component::log_state_(State *next_state) {
  if (this->state_ != this->last_reported_state_) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", this->state_to_string(this->state_));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", this->state_to_string(this->state_), this->state_to_string(*next_state));
    }
    this->last_reported_state_ = this->state_;
  }
}
//---------------------------------------------------------------------------------------
//  The standard 16-bit CRC polynomial specified in ISO/IEC 3309 is used.
//             16   12   5
//  Which is: x  + x  + x + 1
//----------------------------------------------------------------------------
uint16_t LE2Component::crc_16_iec(const uint8_t *buffer, uint16_t len) {
  uint16_t crc = 0xffff;
  uint8_t d;
  do {
    d = *buffer++ ^ (crc & 0xFF);
    d ^= d << 4;
    crc = (d << 3) ^ (d << 8) ^ (crc >> 8) ^ (d >> 4);
  } while (--len);
  crc ^= 0xFFFF;
  return crc;
}

}  // namespace le2
}  // namespace esphome