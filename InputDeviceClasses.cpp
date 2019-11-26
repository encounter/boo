#include "boo/inputdev/DeviceSignature.hpp"
#include "boo/inputdev/DolphinSmashAdapter.hpp"
#include "boo/inputdev/DualshockPad.hpp"
#include "boo/inputdev/GenericPad.hpp"
#include "boo/inputdev/XInputPad.hpp"
#include "boo/inputdev/NintendoPowerA.hpp"
#include "boo/inputdev/SwitchPad.hpp"

namespace boo {

const DeviceSignature BOO_DEVICE_SIGS[] = {DEVICE_SIG(DolphinSmashAdapter, 0x57e, 0x337, DeviceType::USB),
                                           DEVICE_SIG(DualshockPad, 0x54c, 0x268, DeviceType::HID),
                                           DEVICE_SIG(GenericPad, 0, 0, DeviceType::HID),
                                           DEVICE_SIG(NintendoPowerA, 0x20D6, 0xA711, DeviceType::USB),
                                           DEVICE_SIG(SwitchPad, 0x57e, 0x2007, DeviceType::HID), // Joy-Con L
                                           DEVICE_SIG(SwitchPad, 0x57e, 0x2008, DeviceType::HID), // Joy-Con R
                                           DEVICE_SIG(SwitchPad, 0x57e, 0x2009, DeviceType::HID), // Pro Controller
                                           DEVICE_SIG(XInputPad, 0, 0, DeviceType::XInput),
                                           DEVICE_SIG_SENTINEL()};

}
