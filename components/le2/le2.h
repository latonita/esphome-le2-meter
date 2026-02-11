#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace le2 {

enum PHASE : size_t {
  LINE = 0,     // Phase A
  NEUTRAL = 1,  // Phase R
};

enum MEAS : size_t {
  I = 0,        // Current
  P_A = 1,      // Active Power Phase A
  P_R = 2,      // Active Power Phase R
  COS_PHI = 3,  // Power Factor
  S = 4,        // Apparent Power
};

constexpr size_t PHASE_COUNT = 2;
constexpr size_t MEAS_COUNT = 4;
constexpr size_t MEAS_COUNT_EXTENDED = 5;     // Extended to include apparent power
constexpr size_t CONSUMPTION_TYPE_COUNT = 4;  // A+, A-, R+, R-
constexpr size_t TARIFF_COUNT = 8;            // Number of tariffs

struct InternalDataState {
  struct Readings {
    uint8_t current_tariff;
    float consumption[CONSUMPTION_TYPE_COUNT][TARIFF_COUNT];  // [consumption_type][tariff] for A+, A-, R+, R- (0-3)
  } energy;

  // grid parameters
  struct GridParameters {
    uint32_t dtm;
    float u;
    float measurements[PHASE_COUNT][MEAS_COUNT_EXTENDED];  // [phase][measurement+1]: [phase,neutral][I, P_A, P_R,
                                                           // cos_phi, + apparent power]
    float freq;
  } grid;

  char time_str[9]{0};   // "23:59:99"
  char date_str[11]{0};  // "30/08/2023"

  char datetime_str[25]{0};  // "30/08/2023 23:59:59"

  struct {
    uint32_t production_date{0};
    char production_date_str[11]{0};  // "30/08/2023"
    uint32_t serial_number{0};
    uint32_t network_address{0};
    uint8_t type{0};
    uint8_t hw_version{0};
    uint8_t fw_version{0};
    uint64_t error_code{0};

    char about_str[64]{0};  // "Type: 1, HW: 1, FW: 1, Date: 2025-01-28"
  } meter_info;

  uint32_t proper_reads{0};
  uint32_t read_errors{0};
  bool meter_found{false};
  uint8_t got{0};
  uint32_t last_good_read_ms{0};
};

enum class EnqCmd : uint8_t {
  ConsumedEnergy = 0x01,  // read current tariff and consumed energy A+, A-, R+, R- for 8 tariffs
  GridParameters = 0x05,  // read datetime and grid parameters - voltage, frequency, etc.
  MeterInfo = 0xF0,       // read meter info - serial number, network address, errors, etc.
};

class LE2Component : public PollingComponent, public uart::UARTDevice {
#ifdef USE_SENSOR
  SUB_SENSOR(frequency)
  SUB_SENSOR(voltage)
#endif

#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(electricity_tariff)
  SUB_TEXT_SENSOR(date)
  SUB_TEXT_SENSOR(time)
  SUB_TEXT_SENSOR(datetime)
  SUB_TEXT_SENSOR(network_address)
  SUB_TEXT_SENSOR(serial_nr)
  SUB_TEXT_SENSOR(reading_state)
  SUB_TEXT_SENSOR(error_code)
  SUB_TEXT_SENSOR(about)
#endif

 public:
  LE2Component() = default;

#ifdef USE_SENSOR
  void set_tariff_consumption_sensor(uint8_t consumption_type, uint8_t tariff, sensor::Sensor *sensor);
  void set_phase_measurements_sensor(uint8_t phase, uint8_t measurement, sensor::Sensor *sensor);
#endif

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
#ifdef USE_SENSOR
  sensor::Sensor *tariff_consumption_[4][8] = {{nullptr}};
  sensor::Sensor *phase_measurements_[2][5] = {{nullptr}};
#endif

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
  
  const char *state_to_string(State state);
  void log_state_(State *next_state = nullptr);

  uint16_t crc_16_iec(const uint8_t *buffer, uint16_t len);
  size_t slip_encode(uint8_t *data_out, const uint8_t *data_in, size_t size_in);
  size_t slip_decode_inplace(uint8_t *data, size_t size_in);
};

}  // namespace le2
}  // namespace esphome