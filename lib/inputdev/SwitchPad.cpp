#include "boo/inputdev/SwitchPad.hpp"

#include <array>
#include <cstring>

#include "boo/inputdev/DeviceSignature.hpp"

namespace boo {
SwitchPad::SwitchPad(DeviceToken* token) : TDeviceBase<ISwitchProPadCallback>(dev_typeid(SwitchPad), token) {}

SwitchPad::~SwitchPad() = default;

void SwitchPad::deviceDisconnected() {
  std::lock_guard lk{m_callbackLock};
  if (m_callback != nullptr) {
    m_callback->controllerDisconnected();
  }
}

void SwitchPad::initialCycle() {
  m_init = true;
  m_handshakeThread = std::thread{&SwitchPad::performHandshake, this};
}

void SwitchPad::finalCycle() {}

void SwitchPad::receivedHIDReport(const uint8_t* data, size_t length, HIDReportType tp, uint32_t message) {
  if (m_msgType != JOYCON_MSG_TYPE_NONE) {
    std::unique_lock<std::mutex> lk(m_output);
    joycon_input_report* report;
    bool match = false;

    switch (m_msgType) {
    case JOYCON_MSG_TYPE_USB:
      if (length < 2)
        break;
      if (data[0] == JC_INPUT_USB_RESPONSE && data[1] == m_usbAckMatch)
        match = true;
      break;
    case JOYCON_MSG_TYPE_SUBCMD:
      if (length < sizeof(struct joycon_input_report) || data[0] != JC_INPUT_SUBCMD_REPLY)
        break;
      report = (struct joycon_input_report*)data;
      if (report->subcmd_reply.id == m_subcmdAckMatch)
        match = true;
      break;
    default:
      break;
    }

    if (match) {
      memcpy(m_inputBuf, data, std::min(length, JC_MAX_RESP_SIZE));
      m_msgType = JOYCON_MSG_TYPE_NONE;
      m_recievedResponse = true;
      m_wait.notify_one();
      return;
    }
  }

  // Not in read state yet
  if (m_init)
    return;

  if (data[0] == JC_INPUT_SUBCMD_REPLY || data[0] == JC_INPUT_IMU_DATA || data[0] == JC_INPUT_MCU_DATA) {
    if (length >= 12) /* make sure it contains the input report */
      parseReport((struct joycon_input_report*)data);
  }
}

bool SwitchPad::sendHIDReportSync(const uint8_t* data, size_t length, HIDReportType tp, uint32_t message) {
  std::unique_lock<std::mutex> lk(m_output);
  switch (message) {
  case JC_OUTPUT_USB_CMD:
    m_msgType = JOYCON_MSG_TYPE_USB;
    m_usbAckMatch = data[1];
    break;
  case JC_OUTPUT_RUMBLE_AND_SUBCMD:
    m_msgType = JOYCON_MSG_TYPE_SUBCMD;
    m_subcmdAckMatch = ((struct joycon_subcmd_request*)data)->subcmd_id;
    break;
  }
  if (!sendHIDReport(data, length, tp, message)) {
    deviceError(fmt("Unable to send complete packet! Request size {:x}\n"), length);
    return false;
  }
  if (m_msgType != JOYCON_MSG_TYPE_NONE) {
    std::cv_status ret = m_wait.wait_for(lk, std::chrono::milliseconds(300));
    if (ret == std::cv_status::timeout) {
      m_msgType = JOYCON_MSG_TYPE_NONE;
      m_recievedResponse = false;
      return false;
    }
  }
  return true;
}

void SwitchPad::performHandshake() {
  // Start handshake
  uint8_t usbCommand[2] = {JC_OUTPUT_USB_CMD, JC_USB_CMD_HANDSHAKE};
  if (!sendHIDReportSync(usbCommand, sizeof(usbCommand), HIDReportType::Output, usbCommand[0])) {
    deviceError(fmt("Handshake start command failed! (Not USB?)\n"), sizeof(usbCommand));
  } else {
    // Assume USB device

    // Set baud rate
    usbCommand[1] = JC_USB_CMD_BAUDRATE_3M;
    if (!sendHIDReportSync(usbCommand, sizeof(usbCommand), HIDReportType::Output, usbCommand[0])) {
      deviceError(fmt("Baudrate command failed!\n"), sizeof(usbCommand));
      return;
    }

    // End handshake
    usbCommand[1] = JC_USB_CMD_HANDSHAKE;
    if (!sendHIDReportSync(usbCommand, sizeof(usbCommand), HIDReportType::Output, usbCommand[0])) {
      deviceError(fmt("Handshake end command failed!\n"), sizeof(usbCommand));
      return;
    }

    // Disable timeout, no sync
    usbCommand[1] = JC_USB_CMD_NO_TIMEOUT;
    if (!sendHIDReport(usbCommand, sizeof(usbCommand), HIDReportType::Output, usbCommand[0])) {
      deviceError(fmt("Timeout command failed!\n"), sizeof(usbCommand));
      return;
    }
    fmt::print(fmt("Handshake sent!\n"));
  }

  joycon_subcmd_request subcmdRequest = {
      .output_id = JC_OUTPUT_RUMBLE_AND_SUBCMD,
      .subcmd_id = JC_SUBCMD_SET_REPORT_MODE,
      .data = {0x30}, // Full report mode
  };
  if (!sendHIDReportSync((const uint8_t*)&subcmdRequest, sizeof(subcmdRequest), HIDReportType::Output,
                         subcmdRequest.output_id)) {
    deviceError(fmt("Failed to set report mode!\n"), sizeof(usbCommand));
    return;
  }

  subcmdRequest = {
      .output_id = JC_OUTPUT_RUMBLE_AND_SUBCMD,
      .subcmd_id = JC_SUBCMD_SET_PLAYER_LIGHTS,
      .data = {0x1}, // First LED
  };
  if (!sendHIDReportSync((const uint8_t*)&subcmdRequest, sizeof(subcmdRequest), HIDReportType::Output,
                         subcmdRequest.output_id)) {
    deviceError(fmt("Failed to set player LEDS!\n"), sizeof(usbCommand));
    return;
  }
}

void SwitchPad::parseReport(struct joycon_input_report* report) {
  //  SwitchPadState state = *reinterpret_cast<const SwitchPadState*>(data);
  fmt::print(fmt("New button data: {:x} {:x} {:x}\n"), report->button_status[0], report->button_status[1],
             report->button_status[2]);
}

bool SwitchPadState::operator==(const SwitchPadState& other) const {
  return std::memcmp(this, &other, sizeof(SwitchPadState)) == 0;
}

bool SwitchPadState::operator!=(const SwitchPadState& other) const { return !operator==(other); }

} // namespace boo
