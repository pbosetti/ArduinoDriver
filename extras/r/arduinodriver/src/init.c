// init.c - registers the .Call entry points in arduinodriver.cpp. Standard
// R extension boilerplate (see "Writing R Extensions" sec. 5.4).
#include <R.h>
#include <R_ext/Rdynload.h>
#include <Rinternals.h>

extern SEXP adrv_r_last_error_message(void);
extern SEXP adrv_r_context_create(void);
extern SEXP adrv_r_context_destroy(SEXP ctx);
extern SEXP adrv_r_device_open_first(SEXP ctx);
extern SEXP adrv_r_device_open_by_serial(SEXP ctx, SEXP serial);
extern SEXP adrv_r_device_close(SEXP dev);
extern SEXP adrv_r_device_get_info(SEXP dev);
extern SEXP adrv_r_pin_count(SEXP dev);
extern SEXP adrv_r_board_name(SEXP board_id);
extern SEXP adrv_r_pin_mode(SEXP dev, SEXP pin, SEXP mode);
extern SEXP adrv_r_digital_write(SEXP dev, SEXP pin, SEXP high);
extern SEXP adrv_r_digital_read(SEXP dev, SEXP pin);
extern SEXP adrv_r_read_all_digital(SEXP dev);
extern SEXP adrv_r_analog_read(SEXP dev, SEXP pin);
extern SEXP adrv_r_analog_read_volts(SEXP dev, SEXP pin);
extern SEXP adrv_r_read_all_analog(SEXP dev);
extern SEXP adrv_r_pwm_write(SEXP dev, SEXP pin, SEXP duty);
extern SEXP adrv_r_pwm_write_fraction(SEXP dev, SEXP pin, SEXP fraction);
extern SEXP adrv_r_dac_write(SEXP dev, SEXP pin, SEXP value);
extern SEXP adrv_r_dac_write_volts(SEXP dev, SEXP pin, SEXP volts);
extern SEXP adrv_r_device_status(SEXP dev);
extern SEXP adrv_r_sync(SEXP dev);
extern SEXP adrv_r_reset(SEXP dev);
extern SEXP adrv_r_start_stream(SEXP dev, SEXP pins, SEXP period_us,
                                SEXP flags, SEXP queue_capacity);
extern SEXP adrv_r_stream_read(SEXP stream, SEXP max_samples, SEXP timeout_ms);
extern SEXP adrv_r_stream_stats(SEXP stream);
extern SEXP adrv_r_stream_running(SEXP stream);
extern SEXP adrv_r_stream_error(SEXP stream);
extern SEXP adrv_r_stream_stop(SEXP stream);
extern SEXP adrv_r_stream_destroy(SEXP stream);

static const R_CallMethodDef CallEntries[] = {
    {"adrv_r_last_error_message", (DL_FUNC)&adrv_r_last_error_message, 0},
    {"adrv_r_context_create", (DL_FUNC)&adrv_r_context_create, 0},
    {"adrv_r_context_destroy", (DL_FUNC)&adrv_r_context_destroy, 1},
    {"adrv_r_device_open_first", (DL_FUNC)&adrv_r_device_open_first, 1},
    {"adrv_r_device_open_by_serial", (DL_FUNC)&adrv_r_device_open_by_serial, 2},
    {"adrv_r_device_close", (DL_FUNC)&adrv_r_device_close, 1},
    {"adrv_r_device_get_info", (DL_FUNC)&adrv_r_device_get_info, 1},
    {"adrv_r_pin_count", (DL_FUNC)&adrv_r_pin_count, 1},
    {"adrv_r_board_name", (DL_FUNC)&adrv_r_board_name, 1},
    {"adrv_r_pin_mode", (DL_FUNC)&adrv_r_pin_mode, 3},
    {"adrv_r_digital_write", (DL_FUNC)&adrv_r_digital_write, 3},
    {"adrv_r_digital_read", (DL_FUNC)&adrv_r_digital_read, 2},
    {"adrv_r_read_all_digital", (DL_FUNC)&adrv_r_read_all_digital, 1},
    {"adrv_r_analog_read", (DL_FUNC)&adrv_r_analog_read, 2},
    {"adrv_r_analog_read_volts", (DL_FUNC)&adrv_r_analog_read_volts, 2},
    {"adrv_r_read_all_analog", (DL_FUNC)&adrv_r_read_all_analog, 1},
    {"adrv_r_pwm_write", (DL_FUNC)&adrv_r_pwm_write, 3},
    {"adrv_r_pwm_write_fraction", (DL_FUNC)&adrv_r_pwm_write_fraction, 3},
    {"adrv_r_dac_write", (DL_FUNC)&adrv_r_dac_write, 3},
    {"adrv_r_dac_write_volts", (DL_FUNC)&adrv_r_dac_write_volts, 3},
    {"adrv_r_device_status", (DL_FUNC)&adrv_r_device_status, 1},
    {"adrv_r_sync", (DL_FUNC)&adrv_r_sync, 1},
    {"adrv_r_reset", (DL_FUNC)&adrv_r_reset, 1},
    {"adrv_r_start_stream", (DL_FUNC)&adrv_r_start_stream, 5},
    {"adrv_r_stream_read", (DL_FUNC)&adrv_r_stream_read, 3},
    {"adrv_r_stream_stats", (DL_FUNC)&adrv_r_stream_stats, 1},
    {"adrv_r_stream_running", (DL_FUNC)&adrv_r_stream_running, 1},
    {"adrv_r_stream_error", (DL_FUNC)&adrv_r_stream_error, 1},
    {"adrv_r_stream_stop", (DL_FUNC)&adrv_r_stream_stop, 1},
    {"adrv_r_stream_destroy", (DL_FUNC)&adrv_r_stream_destroy, 1},
    {NULL, NULL, 0}};

void R_init_arduinodriver(DllInfo *dll) {
  R_registerRoutines(dll, NULL, CallEntries, NULL, NULL);
  R_useDynamicSymbols(dll, FALSE);
}
