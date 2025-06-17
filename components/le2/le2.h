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
    float A[8];
    float A_returned[8];
    float R[8];
    float R_returned[8];
  } energy;

  // grid parameters
  struct GridParameters {
    time_t dtm;
    float u;
    float i_ph;
    float p_a_phase;
    float p_r_phase;
    float cos_pi_phase;
    float i_neutral;
    float p_a_neutral;
    float p_r_neutral;
    float cos_pi_neutral;
    float freq;
  } grid;

  char timeStr[9]{0};   // "23:59:99"
  char dateStr[11]{0};  // "30/08/2023"

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
  ConsumedEnergy = 0x01,  // read current tariff and consumed energy A+,A-,R+,R- for 8 tariffs
  GridParameters = 0x05,  // read datetime and grid parameters - voltage, frequency, etc.
  MeterInfo = 0xF0,       // read meter info - serial number, network address, errors, etc.
};

class LE2Component : public PollingComponent, public uart::UARTDevice {
 public:
  LE2Component() = default;

  //  replace with SUB_SENSOR()
  void set_active_power_sensor(sensor::Sensor *active_power) { this->active_power_ = active_power; }
  void set_energy_total_sensor(sensor::Sensor *energy_total) { this->energy_total_ = energy_total; }
  void set_energy_t1_sensor(sensor::Sensor *energy_t1) { this->energy_t1_ = energy_t1; }
  void set_energy_t2_sensor(sensor::Sensor *energy_t2) { this->energy_t2_ = energy_t2; }
  void set_energy_t3_sensor(sensor::Sensor *energy_t3) { this->energy_t3_ = energy_t3; }
  void set_energy_t4_sensor(sensor::Sensor *energy_t4) { this->energy_t4_ = energy_t4; }

  void set_electricity_tariff_text_sensor(text_sensor::TextSensor *tariff) { this->tariff_ = tariff; }
  void set_date_text_sensor(text_sensor::TextSensor *date) { this->date_ = date; }
  void set_time_text_sensor(text_sensor::TextSensor *time) { this->time_ = time; }
  void set_network_address_text_sensor(text_sensor::TextSensor *address) { this->network_address_ = address; }
  void set_serial_nr_text_sensor(text_sensor::TextSensor *serial_nr) { this->serial_nr_ = serial_nr; }
  void set_state_text_sensor(text_sensor::TextSensor *state) { this->reading_state_ = state; }

  void set_flow_control_pin(GPIOPin *flow_control_pin) { this->flow_control_pin_ = flow_control_pin; }
  void set_receive_timeout(uint32_t receive_timeout) { this->receive_timeout_ = receive_timeout; }
  void set_requested_meter_address(uint32_t address) { this->requested_meter_address_ = address; }

  float get_setup_priority() const override;

  void dump_config() override;
  void setup() override;

  void loop() override;
  void update() override;

 protected:
  //  replace with SUB_SENSOR()
  sensor::Sensor *active_power_{nullptr};
  sensor::Sensor *energy_total_{nullptr};
  sensor::Sensor *energy_t1_{nullptr};
  sensor::Sensor *energy_t2_{nullptr};
  sensor::Sensor *energy_t3_{nullptr};
  sensor::Sensor *energy_t4_{nullptr};

  text_sensor::TextSensor *tariff_{nullptr};
  text_sensor::TextSensor *date_{nullptr};
  text_sensor::TextSensor *time_{nullptr};
  text_sensor::TextSensor *network_address_{nullptr};
  text_sensor::TextSensor *serial_nr_{nullptr};
  text_sensor::TextSensor *reading_state_{nullptr};

  GPIOPin *flow_control_pin_{nullptr};
  uint32_t receive_timeout_{0};
  uint32_t requested_meter_address_{0};

  InternalDataState data_{};

  // Tracker for the current command and response
  struct {
    EnqCmd current_cmd{};
    uint16_t expected_size{0};
    uint8_t escape_seq_found{0};
    uint32_t start_time{0};
    uint16_t bytes_read{0};
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