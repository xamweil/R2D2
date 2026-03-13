#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

struct DecodedFrame
{
  uint32_t width{0};
  uint32_t height{0};
  std::string encoding;              // e.g. "bgr8"
  std::vector<uint8_t> data;         // tightly packed
  uint32_t step{0};
};

class GstJpegDecoder
{
  public:
    GstJpegDecoder();
    ~GstJpegDecoder();

    GstJpegDecoder(const GstJpegDecoder &) = delete;
    GstJpegDecoder & operator=(const GstJpegDecoder &) = delete;

    bool initialize(std::string & error_message);
    bool decode(
      const std::vector<uint8_t> & jpeg_bytes,
      DecodedFrame & frame,
      std::string & error_message);

  private:
    bool build_pipeline(std::string & error_message);
    void cleanup();

    std::mutex mutex_;
    bool initialized_{false};

    GstElement * pipeline_{nullptr};
    GstElement * appsrc_{nullptr};
    GstElement * appsink_{nullptr};
};