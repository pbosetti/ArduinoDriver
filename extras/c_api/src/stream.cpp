#include "arduino_driver_c.h"
#include "error_mapping.h"
#include "internal_types.h"

#include <chrono>
#include <memory>
#include <span>
#include <vector>

adrv_status_t adrv_start_stream(adrv_device_t *dev,
                                const adrv_stream_config_t *config,
                                adrv_stream_t **out) {
  if (!dev || !dev->device || !config || !out ||
      (config->n_pins > 0 && config->pins == nullptr)) {
    return adrv_detail::invalid_argument("adrv_start_stream: null argument");
  }
  return adrv_guard([&] {
    ArduinoDriver::StreamConfig cpp_config;
    cpp_config.pins.assign(config->pins, config->pins + config->n_pins);
    cpp_config.period = std::chrono::microseconds(config->period_us);
    cpp_config.flags = config->flags;
    if (config->queue_capacity != 0) {
      cpp_config.queue_capacity = config->queue_capacity;
    }
    ArduinoDriver::Stream stream = dev->device->start_stream(cpp_config);
    auto handle = std::make_unique<adrv_stream>();
    handle->stream = std::make_unique<ArduinoDriver::Stream>(std::move(stream));
    *out = handle.release();
  });
}

adrv_status_t adrv_stream_read(adrv_stream_t *stream, adrv_sample_t *out,
                               size_t out_len, uint32_t timeout_ms,
                               size_t *n_read) {
  if (!stream || !stream->stream || (out_len > 0 && !out)) {
    return adrv_detail::invalid_argument("adrv_stream_read: null argument");
  }
  return adrv_guard([&] {
    std::vector<ArduinoDriver::Sample> buffer(out_len);
    const size_t n =
        stream->stream->read(std::span<ArduinoDriver::Sample>(buffer),
                             std::chrono::milliseconds(timeout_ms));
    for (size_t i = 0; i < n; ++i) {
      out[i].pin = buffer[i].pin;
      out[i].raw = buffer[i].raw;
      out[i].volts = buffer[i].volts;
      out[i].t_us = buffer[i].t_us;
    }
    if (n_read) {
      *n_read = n;
    }
  });
}

adrv_status_t adrv_stream_stats(adrv_stream_t *stream,
                                adrv_stream_stats_t *out) {
  if (!stream || !stream->stream || !out) {
    return adrv_detail::invalid_argument("adrv_stream_stats: null argument");
  }
  ArduinoDriver::StreamStats stats = stream->stream->stats();
  out->device_overruns = stats.device_overruns;
  out->records_received = stats.records_received;
  out->seq_gaps = stats.seq_gaps;
  out->host_drops = stats.host_drops;
  out->resyncs = stats.resyncs;
  out->stale_records = stats.stale_records;
  return ADRV_OK;
}

bool adrv_stream_running(adrv_stream_t *stream) {
  if (!stream || !stream->stream) {
    return false;
  }
  return stream->stream->running();
}

const char *adrv_stream_error(adrv_stream_t *stream) {
  if (!stream || !stream->stream) {
    return "";
  }
  stream->last_error = stream->stream->error();
  return stream->last_error.c_str();
}

void adrv_stream_stop(adrv_stream_t *stream) {
  if (stream && stream->stream) {
    stream->stream->stop();
  }
}

void adrv_stream_destroy(adrv_stream_t *stream) { delete stream; }
