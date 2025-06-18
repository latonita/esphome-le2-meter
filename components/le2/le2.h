#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace le2 {

struct InternalDataState {
  struct Readings {
    uint8_t currentTariff;
    float consumption[4][8];  // [consumption_type][tariff] for A+, A-, R+, R- (0-3)
  } energy;

  // grid parameters
  struct GridParameters {
    uint32_t dtm;
    float u;
    float measurements[2][4];  // [phase][measurement]
    float freq;
  } grid;

  char timeStr[9]{0};   // "23:59:99"
  char dateStr[11]{0};  // "30/08/2023"

  char dateTimeStr[25]{0};  // "30/08/2023 23:59:59"

  uint32_t serialNumber{0};
  uint32_t networkAddress{0};

  uint32_t properReads{0};
  uint32_t readErrors{0};
  bool meterFound{false};
  bool initialized{false};
  bool failure{false};
  uint8_t got{0};
  uint32_t lastGoodRead_ms{0};
};

enum class EnqCmd : uint8_t {
  ConsumedEnergy = 0x01,  // read current tariff and consumed energy A+, A-, R+, R- for 8 tariffs
  GridParameters = 0x05,  // read datetime and grid parameters - voltage, frequency, etc.
  MeterInfo = 0xF0,       // read meter info - serial number, network address, errors, etc.
};

class LE2Component : public PollingComponent, public uart::UARTDevice {
  SUB_SENSOR(frequency)
  SUB_SENSOR(voltage)

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(electricity_tariff)
  SUB_TEXT_SENSOR(date)
  SUB_TEXT_SENSOR(time)
  SUB_TEXT_SENSOR(datetime)
  SUB_TEXT_SENSOR(network_address)
  SUB_TEXT_SENSOR(serial_nr)
  SUB_TEXT_SENSOR(reading_state)
#endif

 public:
  LE2Component() = default;


  void set_tariff_consumption_sensor(uint8_t consumption_type, uint8_t tariff, sensor::Sensor *sensor);
  void set_phase_measurements_sensor(uint8_t phase, uint8_t measurement, sensor::Sensor *sensor);

  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_receive_timeout(uint32_t receive_timeout) { this->receive_timeout_ = receive_timeout; }
  void set_requested_meter_address(uint32_t address) { this->requested_meter_address_ = address; }
  void set_password(uint32_t password) { this->password_ = password; } 

  float get_setup_priority() const override;

  void dump_config() override;
  void setup() override;

  void loop() override;
  void update() override;

 protected:
  sensor::Sensor *tariff_consumption_[4][8] = {{nullptr}};
  sensor::Sensor *phase_measurements_[2][4] = {{nullptr}};  // [phase][measurement]

  GPIOPin *flow_control_pin_{nullptr};
  uint32_t receive_timeout_{2000};
  uint32_t requested_meter_address_{0};
  uint32_t password_{0};

  InternalDataState data_{};

  // Tracker for the current command and response
  struct {
    EnqCmd current_cmd{};
    uint16_t expected_size{0};
    uint8_t escape_seq_found{0};
    uint32_t start_time{0};
    uint16_t bytes_read{0};
    void reset() {
      current_cmd = EnqCmd::ConsumedEnergy;
      expected_size = 0;
      escape_seq_found = 0;
      start_time = 0;
      bytes_read = 0;
    }
  } request_tracker_{};

  enum class State : uint8_t {
    NOT_INITIALIZED,
    IDLE,
    WAITING_FOR_RESPONSE,
    GET_METER_INFO,
    GET_GRID_PARAMETERS,
    GET_ENERGY_CONSUMED,
    PUBLISH_INFO,
  } state_{State::NOT_INITIALIZED};

  State next_state_{State::IDLE};
  State last_reported_state_{State::NOT_INITIALIZED};

  void send_enquiry_command(EnqCmd cmd);
  void start_async_request(EnqCmd cmd, uint16_t expected_size, State next_state);
  bool process_response();
  bool process_received_data();

  uint16_t crc_16_iec(const uint8_t *buffer, uint16_t len);

  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);
};

}  // namespace le2
}  // namespace esphome