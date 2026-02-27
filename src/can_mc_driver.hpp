#pragma once

#include "mc_firmware/can_base.hpp"
#include "mc_firmware/mc_common.hpp"
#include "mc_firmware/status.hpp"
#include <bitset>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>

#include <memory>

#include "mcan_base_module_dummy.hpp"
#include "mcan_basic_module_types.hpp"

// #include <iomanip>
// #include <iostream>

#define LOG_MSG(x)
// #define LOG_MSG(msg)                                                                     \
//   do {                                                                                   \
//     auto now = std::chrono::system_clock::now();                                         \
//     auto duration = now.time_since_epoch();                                              \
//     auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);           \
//     auto fraction =                                                                      \
//       std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);          \
//     std::cout << "[" << seconds.count() << "." << std::setfill('0') << std::setw(9)      \
//               << fraction.count() << "] [CAN MC Driver] " << msg << std::endl;           \
//   } while (0);

namespace mcan {

using namespace mcan_basic_module;
using namespace mcan_base_module_dummy;

template<typename McCanSlaveInterface, typename Hardware>
class McSlaveDriver
{
 public:
  Result<std::shared_ptr<McSlaveDriver<McCanSlaveInterface, Hardware>>> static Make(
    std::shared_ptr<CanBase> can_interface,
    uint32_t uid_21_bit)
  {
    if (!can_interface) {
      return Status::Invalid("Can interface is null");
    }

    if (uid_21_bit > 0x1FFFFF) {
      return Status::Invalid("UID must be a 21-bit value");
    }

    return Result<std::shared_ptr<McSlaveDriver>>::OK(std::shared_ptr<McSlaveDriver>(
      new McSlaveDriver(std::move(can_interface), uid_21_bit)));
  }

  Status start_driver()
  {
    switch_to_state(DeviceMode::CONFIGURATION);
    return Status::OK();
  }

  Status control_loop()
  {
    if (execute_state() && _func_main_loop) {
      return _func_main_loop();
    }
    return Status::OK();
  }

  void switch_to_state(DeviceMode next_state)
  {
    LOG_MSG("Switching to state: " << static_cast<int>(next_state));

    if (_state == next_state) {
      return;
    }
    _next_state = next_state;
    _switch_state = true;
  }

  bool execute_state()
  {
    if (!_switch_state) {
      return true;
    }

    // Exit current state
    if (_func_exit_prev_state) {
      auto exit_status = _func_exit_prev_state();
      if (!exit_status.ok()) {
        LOG_MSG("Failed to exit state: " << exit_status.to_string());
        return false;
      }
    }

    DeviceMode next_state = _next_state;
    _switch_state = false;

    // Configure functions for next state
    switch (next_state) {
      case DeviceMode::NORMAL:
        _func_main_loop = std::bind(&McSlaveDriver::mode_normal_loop, this);
        _func_exit_prev_state = std::bind(&McSlaveDriver::mode_exit_normal, this);
        _func_enter_next_state = std::bind(&McSlaveDriver::mode_enter_normal, this);
        break;
      case DeviceMode::CONFIGURATION:
        _func_main_loop = std::bind(&McSlaveDriver::mode_configuration_loop, this);
        _func_exit_prev_state = std::bind(&McSlaveDriver::exit_configuration_mode, this);
        _func_enter_next_state =
          std::bind(&McSlaveDriver::enter_configuration_mode, this);
        break;
      default:
        LOG_MSG("Unknown state");
        return false;
    }

    // Update state
    _state = next_state;

    // Enter new state
    if (_func_enter_next_state) {
      auto enter_status = _func_enter_next_state();
      if (!enter_status.ok()) {
        LOG_MSG("Failed to enter state: " << enter_status.to_string());
        return false;
      }
      // Check if state changed during enter
      if (next_state != _next_state) {
        _func_exit_prev_state = nullptr;
      }
    }

    return next_state == _next_state;
  }

  McCanSlaveInterface& get_interface() { return _interface; }

 private:
  McSlaveDriver(std::shared_ptr<CanBase> can_interface, uint32_t uid_21_bit)
    : _can_interface(std::move(can_interface))
    , _uid_21_bit(uid_21_bit)
  {
  }

  Status mode_normal_loop()
  {
    // Main loop actions for NORMAL mode
    return Status::OK();
  }

  Status mode_enter_normal()
  {
    LOG_MSG("BEGIN Entering NORMAL mode")
    //  Actions to perform when entering NORMAL mode
    (void)_can_interface->close_can();
    ARI_RETURN_ON_ERROR(_can_interface->add_callback(
      mcan_connect_msg_id_with_node_id(
        configs::GetHardwareType::k_base_address, _node_id, true),
      std::bind(&McSlaveDriver::callback_get_hardware_type,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3),
      nullptr));
    ARI_RETURN_ON_ERROR(_can_interface->add_callback(
      mcan_connect_msg_id_with_node_id(
        configs::EnterConfigurationMode::k_base_address, 1, true),
      std::bind(&McSlaveDriver::callback_nm_enter_configuration_mode,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3),
      nullptr));
    ARI_RETURN_ON_ERROR(
      _can_interface->add_callback(mcan_connect_msg_id_with_node_id(
                                     configs::PingModule::k_base_address, _node_id, true),
                                   std::bind(&McSlaveDriver::callback_ping_module,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2,
                                             std::placeholders::_3),
                                   nullptr));

    ARI_RETURN_ON_ERROR(_can_interface->add_callback(
      mcan_connect_msg_id_with_node_id(configs::FlashIndicatorLed::k_base_address,
                                       _node_id),
      std::bind(&McSlaveDriver::callback_flash_indicator_led,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3),
      nullptr));

    std::apply(
      [&](auto&&... args) {
        (
          [&] {
            using MsgT = std::decay_t<decltype(this->_interface.*(args.second))>;
            auto msg_buffer = std::make_shared<CanMultiPackageFrame<MsgT>>();
            (void)_can_interface->add_callback(
              mcan_connect_msg_id_with_node_id(
                (this->_interface.*(args.second)).k_base_address, _node_id),
              [this, args, msg_buffer](CanBase& cd, const CanFrame& frame, void* ar) {
                // static CanMultiPackageFrame<MsgT> msg_buffer = {};
                Status status = mcan_unpack_msg(frame, *msg_buffer);
                if (status.status_code() == StatusCode::Cancelled) {
                  return; // wait for more frames
                } else if (!status.ok()) {
                  return; // error unpacking
                }
                auto& state_ver = this->_interface.*(args.second);
                state_ver.value = msg_buffer->value;
                (this->_interface.*(args.first))(state_ver);
                //  if constexpr(MsgT::k_group == "configs") {
                //    callback_save_configs();
                //  }
              });
          }(),
          ...);
      },
      _interface.get_write_callbacks());

    std::apply(
      [&](auto&&... args) {
        ((void)_can_interface->add_callback(
           mcan_connect_msg_id_with_node_id(
             (this->_interface.*(args)).k_base_address, _node_id, true),
           [this, args](CanBase& cd, const CanFrame& frame, void* ar) {
             (void)mcan_pack_send_msg(
               *_can_interface, this->_interface.*(args), _node_id);
           }),
         ...);
      },
      _interface.get_read_variables());

    _can_interface->open_can();
    LOG_MSG("EXIT Entered NORMAL mode")
    return Status::OK();
  }

  Status mode_exit_normal()
  {
    LOG_MSG("BEGIN Exiting NORMAL mode")
    _can_interface->close_can();
    //  Actions to perform when exiting NORMAL mode
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
      configs::GetHardwareType::k_base_address, _node_id, true)));
    // ARI_RETURN_ON_ERROR(_can_interface->remove_callback(
    //   connect_msg_id_with_node_id(configs::SetDeviceNodeId::k_base_address,
    //   _node_id)));
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
      configs::EnterConfigurationMode::k_base_address, 1, true)));
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
      configs::PingModule::k_base_address, _node_id, true)));

    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
      configs::FlashIndicatorLed::k_base_address, _node_id)));

    std::apply(
      [&](auto&&... args) {
        ((void)_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
           (this->_interface.*(args.second)).k_base_address, _node_id)),
         ...);
      },
      _interface.get_write_callbacks());

    std::apply(
      [&](auto&&... args) {
        ((void)_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
           (this->_interface.*(args)).k_base_address, _node_id, true)),
         ...);
      },
      _interface.get_read_variables());

    _can_interface->open_can();
    LOG_MSG("EXIT Exiting NORMAL mode")
    return Status::OK();
  }

  void callback_get_hardware_type(CanBase& cd, const CanFrame& frame, void* args)
  {
    LOG_MSG("Received GetHardwareType request")
    //  Handle get hardware type callback
    configs::GetHardwareType response;
    response.value.hw_revision = _hardware.k_hw_revision;
    response.value.fw_revision = _hardware.k_fw_revision;
    response.value.hw_time_stamp = _hardware.k_time_stamp;

    CanFrame response_frame;
    response_frame.id = mcan_connect_msg_id_with_node_id(
      configs::GetHardwareType::k_base_address, _node_id);
    response_frame.size = sizeof(response);
    response_frame.is_extended = true;
    response_frame.is_remote_request = false;
    std::memcpy(
      response_frame.data, reinterpret_cast<uint8_t*>(&response), sizeof(response));
    cd.send(response_frame);
  }

  void callback_nm_enter_configuration_mode(CanBase& cd,
                                            const CanFrame& frame,
                                            void* args)
  {
    LOG_MSG("Received request to enter CONFIGURATION mode in Normal Mode")
    switch_to_state(DeviceMode::CONFIGURATION);
  }

  void callback_ping_module(CanBase& cd, const CanFrame& frame, void* args)
  {
    LOG_MSG("Received PingModule request")
    //  Handle ping module callback
    if (!frame.is_remote_request) {
      return; // Invalid frame size
    }
    configs::PingModule response{ _ping_counter++ };
    (void)mcan_pack_send_msg(*_can_interface, response, _node_id);
  }

  void callback_flash_indicator_led(CanBase& cd, const CanFrame& frame, void* args)
  {
    LOG_MSG("Received FlashIndicatorLed request")
    //  Handle flash indicator LED callback
    static CanMultiPackageFrame<configs::FlashIndicatorLed> msg_buffer = {};
    if (!mcan_unpack_msg(frame, msg_buffer).ok()) {
      return; // error unpacking
    }
    _led_indicator_state = msg_buffer.value;
  }

  Status enter_configuration_mode()
  {
    LOG_MSG("BEGIN Entering CONFIGURATION mode")
    _can_interface->close_can();
    //  Actions to perform when entering CONFIGURATION mode
    _node_id = 0; // Unconfigured node ID
    ARI_RETURN_ON_ERROR(_can_interface->add_callback(
      mcan_connect_msg_id_with_node_id(configs::DiscoverDevices::k_base_address, 1, true),
      std::bind(&McSlaveDriver::callback_discover_devices,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3),
      nullptr));
    ARI_RETURN_ON_ERROR(
      _can_interface->add_callback(mcan_connect_msg_id_with_node_id(_uid_21_bit, 1),
                                   std::bind(&McSlaveDriver::callback_set_device_node_id,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2,
                                             std::placeholders::_3),
                                   nullptr));

    ARI_RETURN_ON_ERROR(_can_interface->add_callback(
      mcan_connect_msg_id_with_node_id(
        configs::EnterConfigurationMode::k_base_address, 1, true),
      std::bind(&McSlaveDriver::callback_enter_configuration_mode,
                this,
                std::placeholders::_1,
                std::placeholders::_2,
                std::placeholders::_3),
      nullptr));

    _can_interface->open_can();
    LOG_MSG("EXIT Entering CONFIGURATION mode")
    return Status::OK();
  }

  Status exit_configuration_mode()
  {
    LOG_MSG("BEGIN Exiting CONFIGURATION mode")
    _can_interface->close_can();
    //  Actions to perform when exiting CONFIGURATION mode
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
      configs::DiscoverDevices::k_base_address, 1, true)));
    ARI_RETURN_ON_ERROR(
      _can_interface->remove_callback(mcan_connect_msg_id_with_node_id(_uid_21_bit, 1)));
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(
      configs::EnterConfigurationMode::k_base_address, 1, true)));
    _can_interface->open_can();
    LOG_MSG("EXIT Exiting CONFIGURATION mode")
    return Status::OK();
  }

  Status mode_configuration_loop() { return Status::OK(); }

  void callback_discover_devices(CanBase& cd, const CanFrame& frame, void* args)
  {
    LOG_MSG("Received DiscoverDevices request")
    //  Handle discover devices callback
    //  Respond with unique ID
    configs::DiscoverDevices response;
    response.value = _hardware.k_unique_id;
    (void)mcan_pack_send_msg(
      *_can_interface, response, 0, mcan_connect_msg_id_with_node_id(_uid_21_bit, 0));
  }

  void callback_set_device_node_id(CanBase& cd, const CanFrame& frame, void* args)
  {
    // Handle set device node ID callback
    if (frame.size != 1) {
      return; // Invalid frame size
    }
    CanMultiPackageFrame<configs::SetDeviceNodeId> msg_buffer = {};
    if (!mcan_unpack_msg(frame, msg_buffer).ok()) {
      return; // error unpacking
    }
    _node_id = msg_buffer.value;
    LOG_MSG("Received SetDeviceNodeId request: " << std::to_string(_node_id))
    switch_to_state(DeviceMode::NORMAL);
  }

  void callback_enter_configuration_mode(CanBase& cd, const CanFrame& frame, void* args)
  {
    LOG_MSG("Received request to enter CONFIGURATION mode in Config Mode")
    switch_to_state(DeviceMode::CONFIGURATION);
  }

  std::shared_ptr<CanBase> _can_interface;
  Hardware _hardware;
  uint32_t _uid_21_bit;

  DeviceMode _state{ DeviceMode::UNDEFINED };
  DeviceMode _next_state{ DeviceMode::UNDEFINED };
  bool _switch_state{ false };
  std::function<Status()> _func_main_loop = nullptr;
  std::function<Status()> _func_exit_prev_state = nullptr;
  std::function<Status()> _func_enter_next_state = nullptr;

  McCanSlaveInterface _interface;

  uint16_t _node_id = 0;
  uint8_t _ping_counter = 0;
  bool _led_indicator_state = false;
};

} // namespace mcan