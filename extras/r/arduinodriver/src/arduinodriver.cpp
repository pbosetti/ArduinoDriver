// arduinodriver.cpp - .Call entry points wrapping arduino_driver_c (see
// extras/c_api/include/arduino_driver_c.h). This is a thin, mechanical
// translation: each entry point marshals R SEXPs to/from the C API's types
// and returns either a bare value or a two-element (status, value) list for
// calls that can fail, so R/errors.R can turn a non-OK status into a
// classed condition.
#define R_NO_REMAP
#include <R.h>
#include <Rinternals.h>

#include "arduino_driver_c.h"

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

namespace {

SEXP wrap_ptr(void *ptr, const char *tag) {
  if (ptr == nullptr) {
    return R_NilValue;
  }
  SEXP xp = PROTECT(R_MakeExternalPtr(ptr, Rf_install(tag), R_NilValue));
  UNPROTECT(1);
  return xp;
}

template <typename T> T *unwrap_ptr(SEXP xp) {
  if (xp == R_NilValue) {
    return nullptr;
  }
  return static_cast<T *>(R_ExternalPtrAddr(xp));
}

// A two-element named list: list(status = <int>, value = <value>).
SEXP status_result(int status, SEXP value) {
  SEXP result = PROTECT(Rf_allocVector(VECSXP, 2));
  SET_VECTOR_ELT(result, 0, Rf_ScalarInteger(status));
  SET_VECTOR_ELT(result, 1, value);
  SEXP names = PROTECT(Rf_allocVector(STRSXP, 2));
  SET_STRING_ELT(names, 0, Rf_mkChar("status"));
  SET_STRING_ELT(names, 1, Rf_mkChar("value"));
  Rf_setAttrib(result, R_NamesSymbol, names);
  UNPROTECT(2);
  return result;
}

SEXP named_list(std::initializer_list<const char *> names,
                std::initializer_list<SEXP> values) {
  const R_xlen_t n = static_cast<R_xlen_t>(names.size());
  SEXP result = PROTECT(Rf_allocVector(VECSXP, n));
  SEXP r_names = PROTECT(Rf_allocVector(STRSXP, n));
  R_xlen_t i = 0;
  auto name_it = names.begin();
  auto value_it = values.begin();
  for (; i < n; ++i, ++name_it, ++value_it) {
    SET_VECTOR_ELT(result, i, *value_it);
    SET_STRING_ELT(r_names, i, Rf_mkChar(*name_it));
  }
  Rf_setAttrib(result, R_NamesSymbol, r_names);
  UNPROTECT(2);
  return result;
}

} // namespace

extern "C" {

SEXP adrv_r_last_error_message(void) {
  return Rf_mkString(adrv_last_error_message());
}

// ---- context / device lifecycle --------------------------------------------

SEXP adrv_r_context_create(void) {
  return wrap_ptr(adrv_context_create(), "adrv_context_t");
}

SEXP adrv_r_context_destroy(SEXP ctx) {
  adrv_context_destroy(unwrap_ptr<adrv_context_t>(ctx));
  return R_NilValue;
}

SEXP adrv_r_device_open_first(SEXP ctx) {
  adrv_device_t *dev = nullptr;
  const adrv_status_t status =
      adrv_device_open_first(unwrap_ptr<adrv_context_t>(ctx), &dev);
  return status_result(status, wrap_ptr(dev, "adrv_device_t"));
}

SEXP adrv_r_device_open_by_serial(SEXP ctx, SEXP serial) {
  adrv_device_t *dev = nullptr;
  const adrv_status_t status = adrv_device_open_by_serial(
      unwrap_ptr<adrv_context_t>(ctx), CHAR(STRING_ELT(serial, 0)), &dev);
  return status_result(status, wrap_ptr(dev, "adrv_device_t"));
}

SEXP adrv_r_device_close(SEXP dev) {
  adrv_device_close(unwrap_ptr<adrv_device_t>(dev));
  return R_NilValue;
}

// ---- static information -----------------------------------------------------

SEXP adrv_r_device_get_info(SEXP dev) {
  adrv_info_t info{};
  const adrv_status_t status =
      adrv_device_get_info(unwrap_ptr<adrv_device_t>(dev), &info);
  SEXP value = R_NilValue;
  if (status == ADRV_OK) {
    value = named_list(
        {"protocol_version", "board_id", "n_pins", "n_ain", "adc_bits",
         "pwm_bits", "dac_bits", "queue_depth", "vref_mv", "io_mv", "flags",
         "stream_max_channels", "event_max_pins", "stream_min_period_us"},
        {Rf_ScalarInteger(info.protocol_version),
         Rf_ScalarInteger(info.board_id), Rf_ScalarInteger(info.n_pins),
         Rf_ScalarInteger(info.n_ain), Rf_ScalarInteger(info.adc_bits),
         Rf_ScalarInteger(info.pwm_bits), Rf_ScalarInteger(info.dac_bits),
         Rf_ScalarInteger(info.queue_depth), Rf_ScalarInteger(info.vref_mv),
         Rf_ScalarInteger(info.io_mv), Rf_ScalarInteger(info.flags),
         Rf_ScalarInteger(info.stream_max_channels),
         Rf_ScalarInteger(info.event_max_pins),
         Rf_ScalarInteger(info.stream_min_period_us)});
  }
  return status_result(status, value);
}

SEXP adrv_r_pin_count(SEXP dev) {
  return Rf_ScalarInteger(
      static_cast<int>(adrv_pin_count(unwrap_ptr<adrv_device_t>(dev))));
}

SEXP adrv_r_board_name(SEXP board_id) {
  return Rf_mkString(adrv_board_name(
      static_cast<uint16_t>(Rf_asInteger(board_id))));
}

// ---- configuration ------------------------------------------------------------

SEXP adrv_r_pin_mode(SEXP dev, SEXP pin, SEXP mode) {
  const adrv_status_t status = adrv_pin_mode(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      static_cast<adrv_pin_mode_t>(Rf_asInteger(mode)));
  return Rf_ScalarInteger(status);
}

// ---- digital I/O ----------------------------------------------------------------

SEXP adrv_r_digital_write(SEXP dev, SEXP pin, SEXP high) {
  const adrv_status_t status =
      adrv_digital_write(unwrap_ptr<adrv_device_t>(dev),
                         static_cast<uint8_t>(Rf_asInteger(pin)),
                         Rf_asLogical(high) == TRUE);
  return Rf_ScalarInteger(status);
}

SEXP adrv_r_digital_read(SEXP dev, SEXP pin) {
  bool value = false;
  const adrv_status_t status = adrv_digital_read(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      &value);
  return status_result(status, Rf_ScalarLogical(value ? TRUE : FALSE));
}

SEXP adrv_r_read_all_digital(SEXP dev) {
  adrv_device_t *handle = unwrap_ptr<adrv_device_t>(dev);
  const size_t n = adrv_pin_count(handle);
  auto buffer = std::make_unique<bool[]>(n);
  size_t n_written = 0;
  const adrv_status_t status =
      adrv_read_all_digital(handle, buffer.get(), n, &n_written);
  SEXP value = PROTECT(Rf_allocVector(LGLSXP, static_cast<R_xlen_t>(n_written)));
  for (size_t i = 0; i < n_written; ++i) {
    LOGICAL(value)[i] = buffer[i] ? TRUE : FALSE;
  }
  UNPROTECT(1);
  return status_result(status, value);
}

// ---- analog input -----------------------------------------------------------------

SEXP adrv_r_analog_read(SEXP dev, SEXP pin) {
  uint16_t value = 0;
  const adrv_status_t status = adrv_analog_read(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      &value);
  return status_result(status, Rf_ScalarInteger(value));
}

SEXP adrv_r_analog_read_volts(SEXP dev, SEXP pin) {
  double value = 0.0;
  const adrv_status_t status = adrv_analog_read_volts(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      &value);
  return status_result(status, Rf_ScalarReal(value));
}

SEXP adrv_r_read_all_analog(SEXP dev) {
  adrv_device_t *handle = unwrap_ptr<adrv_device_t>(dev);
  adrv_info_t info{};
  adrv_device_get_info(handle, &info);
  std::vector<uint16_t> buffer(info.n_ain);
  size_t n_written = 0;
  const adrv_status_t status =
      adrv_read_all_analog(handle, buffer.data(), buffer.size(), &n_written);
  SEXP value = PROTECT(Rf_allocVector(INTSXP, static_cast<R_xlen_t>(n_written)));
  for (size_t i = 0; i < n_written; ++i) {
    INTEGER(value)[i] = buffer[i];
  }
  UNPROTECT(1);
  return status_result(status, value);
}

// ---- PWM / DAC output ---------------------------------------------------------------

SEXP adrv_r_pwm_write(SEXP dev, SEXP pin, SEXP duty) {
  const adrv_status_t status = adrv_pwm_write(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      static_cast<uint16_t>(Rf_asInteger(duty)));
  return Rf_ScalarInteger(status);
}

SEXP adrv_r_pwm_write_fraction(SEXP dev, SEXP pin, SEXP fraction) {
  const adrv_status_t status = adrv_pwm_write_fraction(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      Rf_asReal(fraction));
  return Rf_ScalarInteger(status);
}

SEXP adrv_r_dac_write(SEXP dev, SEXP pin, SEXP value) {
  const adrv_status_t status = adrv_dac_write(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      static_cast<uint16_t>(Rf_asInteger(value)));
  return Rf_ScalarInteger(status);
}

SEXP adrv_r_dac_write_volts(SEXP dev, SEXP pin, SEXP volts) {
  const adrv_status_t status = adrv_dac_write_volts(
      unwrap_ptr<adrv_device_t>(dev), static_cast<uint8_t>(Rf_asInteger(pin)),
      Rf_asReal(volts));
  return Rf_ScalarInteger(status);
}

// ---- control -------------------------------------------------------------------------

SEXP adrv_r_device_status(SEXP dev) {
  uint8_t last_error = 0;
  uint8_t queue_pending = 0;
  const adrv_status_t status = adrv_device_status(
      unwrap_ptr<adrv_device_t>(dev), &last_error, &queue_pending);
  SEXP value = status == ADRV_OK
                   ? named_list({"last_error", "queue_pending"},
                                {Rf_ScalarInteger(last_error),
                                 Rf_ScalarInteger(queue_pending)})
                   : R_NilValue;
  return status_result(status, value);
}

SEXP adrv_r_sync(SEXP dev) {
  return Rf_ScalarInteger(adrv_sync(unwrap_ptr<adrv_device_t>(dev)));
}

SEXP adrv_r_reset(SEXP dev) {
  return Rf_ScalarInteger(adrv_reset(unwrap_ptr<adrv_device_t>(dev)));
}

// ---- streaming ------------------------------------------------------------------------

SEXP adrv_r_start_stream(SEXP dev, SEXP pins, SEXP period_us, SEXP flags,
                         SEXP queue_capacity) {
  const R_xlen_t n_pins = Rf_xlength(pins);
  std::vector<uint8_t> pins_buffer(n_pins);
  for (R_xlen_t i = 0; i < n_pins; ++i) {
    pins_buffer[i] = static_cast<uint8_t>(INTEGER(pins)[i]);
  }
  adrv_stream_config_t config{};
  config.pins = pins_buffer.data();
  config.n_pins = pins_buffer.size();
  config.period_us = static_cast<uint32_t>(Rf_asInteger(period_us));
  config.flags = static_cast<uint8_t>(Rf_asInteger(flags));
  config.queue_capacity = static_cast<size_t>(Rf_asInteger(queue_capacity));

  adrv_stream_t *stream = nullptr;
  const adrv_status_t status =
      adrv_start_stream(unwrap_ptr<adrv_device_t>(dev), &config, &stream);
  return status_result(status, wrap_ptr(stream, "adrv_stream_t"));
}

SEXP adrv_r_stream_read(SEXP stream, SEXP max_samples, SEXP timeout_ms) {
  const size_t n = static_cast<size_t>(Rf_asInteger(max_samples));
  std::vector<adrv_sample_t> buffer(n);
  size_t n_read = 0;
  const adrv_status_t status = adrv_stream_read(
      unwrap_ptr<adrv_stream_t>(stream), buffer.data(), n,
      static_cast<uint32_t>(Rf_asInteger(timeout_ms)), &n_read);

  SEXP pin = PROTECT(Rf_allocVector(INTSXP, static_cast<R_xlen_t>(n_read)));
  SEXP raw = PROTECT(Rf_allocVector(INTSXP, static_cast<R_xlen_t>(n_read)));
  SEXP volts = PROTECT(Rf_allocVector(REALSXP, static_cast<R_xlen_t>(n_read)));
  SEXP t_us = PROTECT(Rf_allocVector(REALSXP, static_cast<R_xlen_t>(n_read)));
  for (size_t i = 0; i < n_read; ++i) {
    INTEGER(pin)[i] = buffer[i].pin;
    INTEGER(raw)[i] = buffer[i].raw;
    REAL(volts)[i] = buffer[i].volts;
    REAL(t_us)[i] = static_cast<double>(buffer[i].t_us);
  }
  SEXP value = named_list({"pin", "raw", "volts", "t_us"},
                          {pin, raw, volts, t_us});
  UNPROTECT(4);
  return status_result(status, value);
}

SEXP adrv_r_stream_stats(SEXP stream) {
  adrv_stream_stats_t stats{};
  const adrv_status_t status =
      adrv_stream_stats(unwrap_ptr<adrv_stream_t>(stream), &stats);
  SEXP value =
      status == ADRV_OK
          ? named_list(
                {"device_overruns", "records_received", "seq_gaps",
                 "host_drops", "resyncs", "stale_records"},
                {Rf_ScalarInteger(stats.device_overruns),
                 Rf_ScalarReal(static_cast<double>(stats.records_received)),
                 Rf_ScalarReal(static_cast<double>(stats.seq_gaps)),
                 Rf_ScalarReal(static_cast<double>(stats.host_drops)),
                 Rf_ScalarReal(static_cast<double>(stats.resyncs)),
                 Rf_ScalarReal(static_cast<double>(stats.stale_records))})
          : R_NilValue;
  return status_result(status, value);
}

SEXP adrv_r_stream_running(SEXP stream) {
  return Rf_ScalarLogical(
      adrv_stream_running(unwrap_ptr<adrv_stream_t>(stream)) ? TRUE : FALSE);
}

SEXP adrv_r_stream_error(SEXP stream) {
  return Rf_mkString(adrv_stream_error(unwrap_ptr<adrv_stream_t>(stream)));
}

SEXP adrv_r_stream_stop(SEXP stream) {
  adrv_stream_stop(unwrap_ptr<adrv_stream_t>(stream));
  return R_NilValue;
}

SEXP adrv_r_stream_destroy(SEXP stream) {
  adrv_stream_destroy(unwrap_ptr<adrv_stream_t>(stream));
  return R_NilValue;
}

} // extern "C"
