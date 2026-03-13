#include "camera_relay/gst_jpeg_decoder.hpp"

#include <cstring>
#include <sstream>

#include <gst/video/video.h>

GstJpegDecoder::GstJpegDecoder()
{
  gst_init(nullptr, nullptr);
}

GstJpegDecoder::~GstJpegDecoder()
{
  cleanup();
}

bool GstJpegDecoder::initialize(std::string & error_message)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (initialized_) {
    return true;
  }

  if (!build_pipeline(error_message)) {
    return false;
  }

  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    error_message = "Failed to set decoder pipeline to PLAYING state.";
    cleanup();
    return false;
  }

  initialized_ = true;
  return true;
}

bool GstJpegDecoder::build_pipeline(std::string & error_message)
{
  const char * pipeline_desc =
    "appsrc name=src is-live=false format=time block=true "
    "caps=image/jpeg ! "
    "nvjpegdec ! "
    "nvvidconv ! video/x-raw,format=BGRx ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true";

  GError * gerror = nullptr;
  pipeline_ = gst_parse_launch(pipeline_desc, &gerror);
  if (!pipeline_) {
    error_message = gerror ? gerror->message : "gst_parse_launch failed.";
    if (gerror) {
      g_error_free(gerror);
    }
    return false;
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");

  if (!appsrc_ || !appsink_) {
    error_message = "Failed to retrieve appsrc/appsink from pipeline.";
    cleanup();
    return false;
  }

  gst_app_sink_set_max_buffers(GST_APP_SINK(appsink_), 1);
  gst_app_sink_set_drop(GST_APP_SINK(appsink_), true);

  return true;
}

bool GstJpegDecoder::decode(
  const std::vector<uint8_t> & jpeg_bytes,
  DecodedFrame & frame,
  std::string & error_message)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!initialized_) {
    error_message = "Decoder is not initialized.";
    return false;
  }

  GstBuffer * buffer = gst_buffer_new_allocate(nullptr, jpeg_bytes.size(), nullptr);
  if (!buffer) {
    error_message = "Failed to allocate GstBuffer.";
    return false;
  }

  GstMapInfo map_info;
  if (!gst_buffer_map(buffer, &map_info, GST_MAP_WRITE)) {
    gst_buffer_unref(buffer);
    error_message = "Failed to map GstBuffer for write.";
    return false;
  }

  std::memcpy(map_info.data, jpeg_bytes.data(), jpeg_bytes.size());
  gst_buffer_unmap(buffer, &map_info);

  GstFlowReturn push_ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if (push_ret != GST_FLOW_OK) {
    std::ostringstream oss;
    oss << "gst_app_src_push_buffer failed with code " << push_ret;
    error_message = oss.str();
    return false;
  }

  GstSample * sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink_));
  if (!sample) {
    error_message = "Failed to pull decoded sample from appsink.";
    return false;
  }

  GstCaps * caps = gst_sample_get_caps(sample);
  if (!caps) {
    gst_sample_unref(sample);
    error_message = "Decoded sample has no caps.";
    return false;
  }

  GstStructure * structure = gst_caps_get_structure(caps, 0);
  int width = 0;
  int height = 0;
  if (!gst_structure_get_int(structure, "width", &width) ||
      !gst_structure_get_int(structure, "height", &height))
  {
    gst_sample_unref(sample);
    error_message = "Failed to read decoded frame width/height from caps.";
    return false;
  }

  GstBuffer * out_buffer = gst_sample_get_buffer(sample);
  if (!out_buffer) {
    gst_sample_unref(sample);
    error_message = "Decoded sample has no buffer.";
    return false;
  }

  GstMapInfo out_map;
  if (!gst_buffer_map(out_buffer, &out_map, GST_MAP_READ)) {
    gst_sample_unref(sample);
    error_message = "Failed to map decoded output buffer.";
    return false;
  }

  frame.width = static_cast<uint32_t>(width);
  frame.height = static_cast<uint32_t>(height);
  frame.encoding = "bgr8";
  frame.step = static_cast<uint32_t>(width * 3);
  frame.data.assign(out_map.data, out_map.data + out_map.size);

  gst_buffer_unmap(out_buffer, &out_map);
  gst_sample_unref(sample);

  return true;
}

void GstJpegDecoder::cleanup()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
  }

  if (appsrc_) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }

  if (appsink_) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }

  if (pipeline_) {
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }

  initialized_ = false;
}