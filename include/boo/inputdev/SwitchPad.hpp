#pragma once
#include "DeviceBase.hpp"
#include "boo/System.hpp"

#include <condition_variable>
#include <thread>

namespace boo {
struct SwitchPadState {
  // TODO
  bool operator==(const SwitchPadState& other) const;
  bool operator!=(const SwitchPadState& other) const;
};

class SwitchPad;
struct ISwitchProPadCallback {
  virtual void controllerDisconnected() {}
  virtual void controllerUpdate(const SwitchPadState& state) {}
};

enum joycon_msg_type {
  JOYCON_MSG_TYPE_NONE,
  JOYCON_MSG_TYPE_USB,
  JOYCON_MSG_TYPE_SUBCMD,
};

struct joycon_subcmd_request {
  uint8_t output_id;  /* must be 0x01 for subcommand, 0x10 for rumble only */
  uint8_t packet_num; /* incremented every send */
  uint8_t rumble_data[8];
  uint8_t subcmd_id;
  uint8_t data[1]; /* length depends on the subcommand */
} __attribute__((__packed__));

struct joycon_subcmd_reply {
  uint8_t ack; /* MSB 1 for ACK, 0 for NACK */
  uint8_t id; /* id of requested subcmd */
  uint8_t data[0]; /* will be at most 35 bytes */
} __attribute__((__packed__));

struct joycon_imu_data {
  uint16_t accel_x;
  uint16_t accel_y;
  uint16_t accel_z;
  uint16_t gyro_x;
  uint16_t gyro_y;
  uint16_t gyro_z;
} __attribute__((__packed__));

struct joycon_imu_report {
  struct joycon_imu_data data[3];
} __attribute__((__packed__));

/* Output Reports */
static const uint8_t JC_OUTPUT_RUMBLE_AND_SUBCMD = 0x01;
static const uint8_t JC_OUTPUT_FW_UPDATE_PKT = 0x03;
static const uint8_t JC_OUTPUT_RUMBLE_ONLY = 0x10;
static const uint8_t JC_OUTPUT_MCU_DATA = 0x11;
static const uint8_t JC_OUTPUT_USB_CMD = 0x80;

/* USB Commands */
static const uint8_t JC_USB_CMD_CONN_STATUS	= 0x01;
static const uint8_t JC_USB_CMD_HANDSHAKE	= 0x02;
static const uint8_t JC_USB_CMD_BAUDRATE_3M	= 0x03;
static const uint8_t JC_USB_CMD_NO_TIMEOUT	= 0x04;
static const uint8_t JC_USB_CMD_EN_TIMEOUT	= 0x05;
static const uint8_t JC_USB_RESET		= 0x06;
static const uint8_t JC_USB_PRE_HANDSHAKE	= 0x91;
static const uint8_t JC_USB_SEND_UART		= 0x92;

/* Input Reports */
static const uint8_t JC_INPUT_BUTTON_EVENT = 0x3F;
static const uint8_t JC_INPUT_SUBCMD_REPLY = 0x21;
static const uint8_t JC_INPUT_IMU_DATA = 0x30;
static const uint8_t JC_INPUT_MCU_DATA = 0x31;
static const uint8_t JC_INPUT_USB_RESPONSE = 0x81;

/* Subcommand IDs */
static const uint8_t JC_SUBCMD_STATE			= 0x00;
static const uint8_t JC_SUBCMD_MANUAL_BT_PAIRING	= 0x01;
static const uint8_t JC_SUBCMD_REQ_DEV_INFO		= 0x02;
static const uint8_t JC_SUBCMD_SET_REPORT_MODE	= 0x03;
static const uint8_t JC_SUBCMD_TRIGGERS_ELAPSED	= 0x04;
static const uint8_t JC_SUBCMD_GET_PAGE_LIST_STATE	= 0x05;
static const uint8_t JC_SUBCMD_SET_HCI_STATE		= 0x06;
static const uint8_t JC_SUBCMD_RESET_PAIRING_INFO	= 0x07;
static const uint8_t JC_SUBCMD_LOW_POWER_MODE	= 0x08;
static const uint8_t JC_SUBCMD_SPI_FLASH_READ	= 0x10;
static const uint8_t JC_SUBCMD_SPI_FLASH_WRITE	= 0x11;
static const uint8_t JC_SUBCMD_RESET_MCU		= 0x20;
static const uint8_t JC_SUBCMD_SET_MCU_CONFIG	= 0x21;
static const uint8_t JC_SUBCMD_SET_MCU_STATE		= 0x22;
static const uint8_t JC_SUBCMD_SET_PLAYER_LIGHTS	= 0x30;
static const uint8_t JC_SUBCMD_GET_PLAYER_LIGHTS	= 0x31;
static const uint8_t JC_SUBCMD_SET_HOME_LIGHT	= 0x38;
static const uint8_t JC_SUBCMD_ENABLE_IMU		= 0x40;
static const uint8_t JC_SUBCMD_SET_IMU_SENSITIVITY	= 0x41;
static const uint8_t JC_SUBCMD_WRITE_IMU_REG		= 0x42;
static const uint8_t JC_SUBCMD_READ_IMU_REG		= 0x43;
static const uint8_t JC_SUBCMD_ENABLE_VIBRATION	= 0x48;
static const uint8_t JC_SUBCMD_GET_REGULATED_VOLTAGE	= 0x50;

struct joycon_input_report {
  uint8_t id;
  uint8_t timer;
  uint8_t bat_con; /* battery and connection info */
  uint8_t button_status[3];
  uint8_t left_stick[3];
  uint8_t right_stick[3];
  uint8_t vibrator_report;

  union {
    struct joycon_subcmd_reply subcmd_reply;
    struct joycon_imu_report imu_report;
  };
} __attribute__((__packed__));

#define JC_MAX_RESP_SIZE	(sizeof(struct joycon_input_report) + 35)
#define JC_NUM_LEDS		4
#define JC_RUMBLE_DATA_SIZE	8
#define JC_RUMBLE_QUEUE_SIZE	8

class SwitchPad final : public TDeviceBase<ISwitchProPadCallback> {
  bool m_init;
  joycon_msg_type m_msgType;
  uint8_t m_usbAckMatch;
  uint8_t m_subcmdAckMatch;
  bool m_recievedResponse;
  uint8_t m_inputBuf[JC_MAX_RESP_SIZE];
  std::mutex m_output;
  std::condition_variable m_wait;
  std::thread m_handshakeThread;

  SwitchPadState m_last{};
  void deviceDisconnected() override;
  void initialCycle() override;
  void finalCycle() override;
  void receivedHIDReport(const uint8_t* data, size_t length, HIDReportType tp, uint32_t message) override;

  void performHandshake();
  bool sendHIDReportSync(const uint8_t* data, size_t length, HIDReportType tp, uint32_t message);
  void parseReport(struct joycon_input_report* report);

public:
  explicit SwitchPad(DeviceToken*);
  ~SwitchPad() override;
};
} // namespace boo
