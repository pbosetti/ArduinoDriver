#include "arduino_driver_c.h"
#include "error_mapping.h"
#include "internal_types.h"

#include "arduino_driver/Enumerator.h"

#include <algorithm>
#include <memory>

namespace {

ArduinoDriver::PinMode to_cpp_pin_mode(adrv_pin_mode_t mode) {
  using ArduinoDriver::PinMode;
  switch (mode) {
  case ADRV_PIN_INPUT:
    return PinMode::Input;
  case ADRV_PIN_OUTPUT:
    return PinMode::Output;
  case ADRV_PIN_INPUT_PULLUP:
    return PinMode::InputPullup;
  case ADRV_PIN_INPUT_PULLDOWN:
    return PinMode::InputPulldown;
  case ADRV_PIN_ANALOG_IN:
    return PinMode::AnalogIn;
  case ADRV_PIN_PWM:
    return PinMode::Pwm;
  case ADRV_PIN_DAC:
    return PinMode::Dac;
  }
  return PinMode::Input; // unreachable for a valid adrv_pin_mode_t
}

adrv_info_t to_c_info(const ArduinoDriver::Info &info) {
  adrv_info_t out{};
  out.protocol_version = info.protocol_version;
  out.board_id = static_cast<uint16_t>(info.board_id);
  out.n_pins = info.n_pins;
  out.n_ain = info.n_ain;
  out.adc_bits = info.adc_bits;
  out.pwm_bits = info.pwm_bits;
  out.dac_bits = info.dac_bits;
  out.queue_depth = info.queue_depth;
  out.vref_mv = info.vref_mv;
  out.io_mv = info.io_mv;
  out.flags = info.flags;
  out.stream_max_channels = info.stream_max_channels;
  out.event_max_pins = info.event_max_pins;
  out.stream_min_period_us = info.stream_min_period_us;
  return out;
}

} // namespace

adrv_status_t adrv_device_open_first(adrv_context_t *ctx, adrv_device_t **out) {
  if (!ctx || !out) {
    return adrv_detail::invalid_argument(
        "adrv_device_open_first: null context or out-param");
  }
  return adrv_guard([&] {
    ArduinoDriver::Device device = ArduinoDriver::open_first(ctx->ctx);
    auto handle = std::make_unique<adrv_device>();
    handle->device = std::make_unique<ArduinoDriver::Device>(std::move(device));
    handle->context = ctx->ctx;
    *out = handle.release();
  });
}

adrv_status_t adrv_device_open_by_serial(adrv_context_t *ctx,
                                         const char *serial,
                                         adrv_device_t **out) {
  if (!ctx || !serial || !out) {
    return adrv_detail::invalid_argument(
        "adrv_device_open_by_serial: null context, serial or out-param");
  }
  return adrv_guard([&] {
    ArduinoDriver::Device device =
        ArduinoDriver::open_by_serial(ctx->ctx, serial);
    auto handle = std::make_unique<adrv_device>();
    handle->device = std::make_unique<ArduinoDriver::Device>(std::move(device));
    handle->context = ctx->ctx;
    *out = handle.release();
  });
}

void adrv_device_close(adrv_device_t *dev) { delete dev; }

adrv_status_t adrv_device_get_info(adrv_device_t *dev, adrv_info_t *out) {
  if (!dev || !dev->device || !out) {
    return adrv_detail::invalid_argument("adrv_device_get_info: null argument");
  }
  *out = to_c_info(dev->device->info());
  return ADRV_OK;
}

size_t adrv_pin_count(adrv_device_t *dev) {
  if (!dev || !dev->device) {
    return 0;
  }
  return dev->device->pin_count();
}

const char *adrv_board_name(uint16_t board_id) {
  return ArduinoDriver::board_name(
             static_cast<ArduinoDriver::BoardId>(board_id))
      .data();
}

adrv_status_t adrv_pin_mode(adrv_device_t *dev, uint8_t pin,
                            adrv_pin_mode_t mode) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_pin_mode: null device");
  }
  return adrv_guard([&] { dev->device->pin_mode(pin, to_cpp_pin_mode(mode)); });
}

adrv_status_t adrv_digital_write(adrv_device_t *dev, uint8_t pin, bool high) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_digital_write: null device");
  }
  return adrv_guard([&] { dev->device->digital_write(pin, high); });
}

adrv_status_t adrv_digital_read(adrv_device_t *dev, uint8_t pin, bool *out) {
  if (!dev || !dev->device || !out) {
    return adrv_detail::invalid_argument("adrv_digital_read: null argument");
  }
  return adrv_guard([&] { *out = dev->device->digital_read(pin); });
}

adrv_status_t adrv_read_all_digital(adrv_device_t *dev, bool *out,
                                    size_t out_len, size_t *n_written) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_read_all_digital: null device");
  }
  return adrv_guard([&] {
    std::vector<bool> bits = dev->device->read_all_digital();
    if (n_written) {
      *n_written = bits.size();
    }
    if (out) {
      const size_t n = std::min(out_len, bits.size());
      for (size_t i = 0; i < n; ++i) {
        out[i] = bits[i];
      }
    }
  });
}

adrv_status_t adrv_analog_read(adrv_device_t *dev, uint8_t pin, uint16_t *out) {
  if (!dev || !dev->device || !out) {
    return adrv_detail::invalid_argument("adrv_analog_read: null argument");
  }
  return adrv_guard([&] { *out = dev->device->analog_read(pin); });
}

adrv_status_t adrv_analog_read_volts(adrv_device_t *dev, uint8_t pin,
                                     double *out) {
  if (!dev || !dev->device || !out) {
    return adrv_detail::invalid_argument(
        "adrv_analog_read_volts: null argument");
  }
  return adrv_guard([&] { *out = dev->device->analog_read_volts(pin); });
}

adrv_status_t adrv_read_all_analog(adrv_device_t *dev, uint16_t *out,
                                   size_t out_len, size_t *n_written) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_read_all_analog: null device");
  }
  return adrv_guard([&] {
    std::vector<std::uint16_t> samples = dev->device->read_all_analog();
    if (n_written) {
      *n_written = samples.size();
    }
    if (out) {
      const size_t n = std::min(out_len, samples.size());
      std::copy_n(samples.begin(), n, out);
    }
  });
}

adrv_status_t adrv_to_volts(adrv_device_t *dev, uint16_t raw, double *out) {
  if (!dev || !dev->device || !out) {
    return adrv_detail::invalid_argument("adrv_to_volts: null argument");
  }
  *out = dev->device->to_volts(raw);
  return ADRV_OK;
}

adrv_status_t adrv_pwm_write(adrv_device_t *dev, uint8_t pin, uint16_t duty) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_pwm_write: null device");
  }
  return adrv_guard([&] { dev->device->pwm_write(pin, duty); });
}

adrv_status_t adrv_pwm_write_fraction(adrv_device_t *dev, uint8_t pin,
                                      double fraction) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument(
        "adrv_pwm_write_fraction: null device");
  }
  return adrv_guard([&] { dev->device->pwm_write_fraction(pin, fraction); });
}

adrv_status_t adrv_dac_write(adrv_device_t *dev, uint8_t pin, uint16_t value) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_dac_write: null device");
  }
  return adrv_guard([&] { dev->device->dac_write(pin, value); });
}

adrv_status_t adrv_dac_write_volts(adrv_device_t *dev, uint8_t pin,
                                   double volts) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_dac_write_volts: null device");
  }
  return adrv_guard([&] { dev->device->dac_write_volts(pin, volts); });
}

adrv_status_t adrv_device_status(adrv_device_t *dev, uint8_t *out_last_error,
                                 uint8_t *out_queue_pending) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_device_status: null device");
  }
  return adrv_guard([&] {
    uint8_t queue_pending = 0;
    ArduinoDriver::Status status = dev->device->status(&queue_pending);
    if (out_last_error) {
      *out_last_error = static_cast<uint8_t>(status);
    }
    if (out_queue_pending) {
      *out_queue_pending = queue_pending;
    }
  });
}

adrv_status_t adrv_sync(adrv_device_t *dev) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_sync: null device");
  }
  return adrv_guard([&] { dev->device->sync(); });
}

adrv_status_t adrv_reset(adrv_device_t *dev) {
  if (!dev || !dev->device) {
    return adrv_detail::invalid_argument("adrv_reset: null device");
  }
  return adrv_guard([&] { dev->device->reset(); });
}
