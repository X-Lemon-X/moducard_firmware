#pragma once


#include "can_base.hpp"
#include "status.hpp"
#include "mc_common.hpp"
#include <memory>
#include <cstdint>
#include <functional>
#include <cstring>
#include <bitset>

#include "basic_module_types.hpp"
#include "base_module_dummy.hpp"

namespace mcan {


using namespace basic_module;
using namespace base_module_dummy;

template <typename McCanSlaveInterface, typename Hardware> class McSlavePluginDriver {

public:
  Result<std::shared_ptr<McSlavePluginDriver<McCanSlaveInterface, Hardware>>> static Make(std::shared_ptr<CanBase> can_interface,
                                                                                          uint32_t uid_21_bit) {
    if(!can_interface) {
      return Status::Invalid("Can interface is null");
    }

    if(uid_21_bit > 0x1FFFFF) {
      return Status::Invalid("UID must be a 21-bit value");
    }

    return Result<std::shared_ptr<McSlavePluginDriver>>::OK(
    std::shared_ptr<McSlavePluginDriver>(new McSlavePluginDriver(std::move(can_interface), uid_21_bit)));
  }

  Status start_driver() {
    ARI_RETURN_ON_ERROR(generic_switch_mode(DeviceMode::CONFIGURATION));
    return Status::OK();
  }

  Status control_loop() {
    switch(_mode) {
    case DeviceMode::NORMAL: return mode_normal_loop(); break;
    case DeviceMode::CONFIGURATION: return mode_configuration_loop(); break;
    default: break;
    }
    return Status::OK();
  }

  McCanSlaveInterface &get_interface() {
    return _interface;
  }

private:
  McSlavePluginDriver(std::shared_ptr<CanBase> can_interface, uint32_t uid_21_bit)
  : _can_interface(std::move(can_interface)), _uid_21_bit(uid_21_bit) {
  }


  Status generic_switch_mode(DeviceMode new_mode) {
    if(new_mode == _mode) {
      return Status::OK(); // Already in the desired mode
    }

    // Exit current mode
    if(_mode_exit_mode_func) {
      ARI_RETURN_ON_ERROR(_mode_exit_mode_func());
    }

    switch(new_mode) {
    case DeviceMode::NORMAL:
      _mode_exit_mode_func  = std::bind(&McSlavePluginDriver::mode_exit_normal, this);
      _mode_enter_mode_func = std::bind(&McSlavePluginDriver::mode_enter_normal, this);
      break;
    case DeviceMode::CONFIGURATION:
      _mode_exit_mode_func  = std::bind(&McSlavePluginDriver::exit_configuration_mode, this);
      _mode_enter_mode_func = std::bind(&McSlavePluginDriver::enter_configuration_mode, this);
      break;
    default:
      // Handle unknown mode error
      break;
    }

    // Update mode
    _mode = new_mode;

    // Enter new mode
    if(_mode_enter_mode_func) {
      ARI_RETURN_ON_ERROR(_mode_enter_mode_func());
    }

    return Status::OK();
  }

  Status mode_normal_loop() {
    // Main loop actions for NORMAL mode
    return Status::OK();
  }

  Status mode_enter_normal() {
    // Actions to perform when entering NORMAL mode
    ARI_RETURN_ON_ERROR(
    _can_interface->add_callback(mcan_connect_msg_id_with_node_id(configs::GetHardwareType::k_base_address, _node_id, true),
                                 std::bind(&McSlavePluginDriver::callback_get_hardware_type, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                 nullptr));
    ARI_RETURN_ON_ERROR(
    _can_interface->add_callback(mcan_connect_msg_id_with_node_id(configs::EnterConfigurationMode::k_base_address, 1, true),
                                 std::bind(&McSlavePluginDriver::callback_nm_enter_configuration_mode, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                 nullptr));
    ARI_RETURN_ON_ERROR(
    _can_interface->add_callback(mcan_connect_msg_id_with_node_id(configs::PingModule::k_base_address, _node_id, true),
                                 std::bind(&McSlavePluginDriver::callback_ping_module, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                 nullptr));

    ARI_RETURN_ON_ERROR(
    _can_interface->add_callback(mcan_connect_msg_id_with_node_id(configs::FlashIndicatorLed::k_base_address, _node_id),
                                 std::bind(&McSlavePluginDriver::callback_flash_indicator_led, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                 nullptr));


    std::apply(
    [&](auto &&...args) {
      (
      [&] {
        using MsgT      = std::decay_t<decltype(this->_interface.*(args.second))>;
        auto msg_buffer = std::make_shared<CanMultiPackageFrame<MsgT>>();
        (void)_can_interface->add_callback(mcan_connect_msg_id_with_node_id((this->_interface.*(args.second)).k_base_address, _node_id),
                                           [this, args, msg_buffer](CanBase &cd, const CanFrame &frame, void *ar) {
                                             // static CanMultiPackageFrame<MsgT> msg_buffer = {};
                                             Status status = mcan_unpack_msg(frame, *msg_buffer);
                                             if(status.status_code() == StatusCode::Cancelled) {
                                               return; // wait for more frames
                                             } else if(!status.ok()) {
                                               return; // error unpacking
                                             }
                                             auto &state_ver = this->_interface.*(args.second);
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
    [&](auto &&...args) {
      ((void)_can_interface->add_callback(mcan_connect_msg_id_with_node_id((this->_interface.*(args)).k_base_address,
                                                                           _node_id, true),
                                          [this, args](CanBase &cd, const CanFrame &frame, void *ar) {
                                            (void)mcan_pack_send_msg(*_can_interface, this->_interface.*(args), _node_id);
                                          }),
       ...);
    },
    _interface.get_read_variables());


    return Status::OK();
  }

  Status mode_exit_normal() {
    // Actions to perform when exiting NORMAL mode
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(
    mcan_connect_msg_id_with_node_id(configs::GetHardwareType::k_base_address, _node_id, true)));
    // ARI_RETURN_ON_ERROR(_can_interface->remove_callback(
    // connect_msg_id_with_node_id(configs::SetDeviceNodeId::k_base_address, _node_id)));
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(
    mcan_connect_msg_id_with_node_id(configs::EnterConfigurationMode::k_base_address, 1, true)));
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(
    mcan_connect_msg_id_with_node_id(configs::PingModule::k_base_address, _node_id, true)));


    std::apply(
    [&](auto &&...args) {
      ((void)_can_interface->remove_callback(
       mcan_connect_msg_id_with_node_id((this->_interface.*(args.second)).k_base_address, _node_id)),
       ...);
    },
    _interface.get_write_callbacks());

    std::apply(
    [&](auto &&...args) {
      ((void)_can_interface->remove_callback(
       mcan_connect_msg_id_with_node_id((this->_interface.*(args)).k_base_address, _node_id, true)),
       ...);
    },
    _interface.get_read_variables());


    return Status::OK();
  }

  void callback_get_hardware_type(CanBase &cd, const CanFrame &frame, void *args) {
    // Handle get hardware type callback
    configs::GetHardwareType response;
    response.value.hw_revision   = _hardware.k_hw_revision;
    response.value.fw_revision   = _hardware.k_fw_revision;
    response.value.hw_time_stamp = _hardware.k_time_stamp;

    CanFrame response_frame;
    response_frame.id = mcan_connect_msg_id_with_node_id(configs::GetHardwareType::k_base_address, _node_id);
    response_frame.size              = sizeof(response);
    response_frame.is_extended       = true;
    response_frame.is_remote_request = false;
    std::memcpy(response_frame.data, reinterpret_cast<uint8_t *>(&response), sizeof(response));
    cd.send(response_frame);
  }

  // void callback_nm_set_device_node_id(CanBase &cd, const CanFrame &frame, void *args);
  void callback_nm_enter_configuration_mode(CanBase &cd, const CanFrame &frame, void *args) {
    (void)(generic_switch_mode(DeviceMode::CONFIGURATION));
  }

  void callback_ping_module(CanBase &cd, const CanFrame &frame, void *args) {
    // Handle ping module callback
    if(!frame.is_remote_request) {
      return; // Invalid frame size
    }
    configs::PingModule response{ _ping_counter++ };
    (void)mcan_pack_send_msg(*_can_interface, response, _node_id);
  }

  void callback_flash_indicator_led(CanBase &cd, const CanFrame &frame, void *args) {
    // Handle flash indicator LED callback
    static CanMultiPackageFrame<configs::FlashIndicatorLed> msg_buffer = {};
    if(!mcan_unpack_msg(frame, msg_buffer).ok()) {
      return; // error unpacking
    }
    _led_indicator_state = msg_buffer.value;
  }


  Status enter_configuration_mode() {
    // Actions to perform when entering CONFIGURATION mode
    _node_id     = 0; // Unconfigured node ID
    _new_node_id = 0;
    ARI_RETURN_ON_ERROR(
    _can_interface->add_callback(mcan_connect_msg_id_with_node_id(configs::DiscoverDevices::k_base_address, 1, true),
                                 std::bind(&McSlavePluginDriver::callback_discover_devices, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                 nullptr));
    ARI_RETURN_ON_ERROR(
    _can_interface->add_callback(mcan_connect_msg_id_with_node_id(_uid_21_bit, 1),
                                 std::bind(&McSlavePluginDriver::callback_set_device_node_id, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                 nullptr));

    ARI_RETURN_ON_ERROR(
    _can_interface->add_callback(mcan_connect_msg_id_with_node_id(configs::EnterConfigurationMode::k_base_address, 1, true),
                                 std::bind(&McSlavePluginDriver::callback_enter_configuration_mode, this,
                                           std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
                                 nullptr));

    return Status::OK();
  }


  Status exit_configuration_mode() {
    // Actions to perform when exiting CONFIGURATION mode
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(
    mcan_connect_msg_id_with_node_id(configs::DiscoverDevices::k_base_address, 1, true)));
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(mcan_connect_msg_id_with_node_id(_uid_21_bit, 1)));
    ARI_RETURN_ON_ERROR(_can_interface->remove_callback(
    mcan_connect_msg_id_with_node_id(configs::EnterConfigurationMode::k_base_address, 1, true)));
    _new_node_id = 0;
    return Status::OK();
  }

  Status mode_configuration_loop() {
    return Status::OK();
  }


  void callback_discover_devices(CanBase &cd, const CanFrame &frame, void *args) {
    // Handle discover devices callback
    // Respond with unique ID
    configs::DiscoverDevices response;
    response.value = _hardware.k_unique_id;
    (void)mcan_pack_send_msg(*_can_interface, response, 0);
  }

  void callback_set_device_node_id(CanBase &cd, const CanFrame &frame, void *args) {
    // Handle set device node ID callback
    if(frame.size != 1) {
      return; // Invalid frame size
    }
    CanMultiPackageFrame<configs::SetDeviceNodeId> msg_buffer = {};
    if(!mcan_unpack_msg(frame, msg_buffer).ok()) {
      return; // error unpacking
    }
    _node_id = msg_buffer.value;
    (void)generic_switch_mode(DeviceMode::NORMAL);
  }

  void callback_enter_configuration_mode(CanBase &cd, const CanFrame &frame, void *args) {
    (void)(generic_switch_mode(DeviceMode::CONFIGURATION));
  }


  std::shared_ptr<CanBase> _can_interface;
  Hardware _hardware;
  uint32_t _uid_21_bit;

  DeviceMode _mode{ DeviceMode::UNDEFINED };
  std::function<Status()> _mode_exit_mode_func  = nullptr;
  std::function<Status()> _mode_enter_mode_func = nullptr;

  McCanSlaveInterface _interface;


  uint16_t _node_id         = 0;
  uint8_t _new_node_id      = 0;
  uint8_t _ping_counter     = 0;
  bool _led_indicator_state = false;
};


} // namespace mcan