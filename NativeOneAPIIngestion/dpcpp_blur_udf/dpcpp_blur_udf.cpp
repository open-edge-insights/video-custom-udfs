// Copyright (c) 2021 Intel Corporation.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM,OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

/**
 * @file
 * @brief OneAPI ingestion UDF Implementation.
 */


#include "dpcpp_blur_udf.h"
#include <eii/utils/logger.h>

class sycl_inverse_kernel;

using namespace eii::custom_udfs;
using namespace eii::msgbus;
using namespace cl::sycl;

dpcpp_blur_udf::dpcpp_blur_udf(config_t *config) : BaseUdf(config) {
  LOG_DEBUG_0("Initializing UDF...");

  height = config_get(config, "height");
  if (height != NULL) {
    if (height->type != CVT_INTEGER) {
      const char *err = "height type must be a integer";
      LOG_ERROR("%s", err);
      config_value_destroy(height);
      throw err;
    }
  } else {
      const char *err = "height NULL config recieved";
      LOG_ERROR("%s", err);
      throw err;
  }

  width = config_get(config, "width");
  if (width != NULL) {
    if (width->type != CVT_INTEGER) {
      const char *err = "width type must be a integer";
      LOG_ERROR("%s", err);
      config_value_destroy(width);
      throw err;
    }
  } else {
      const char *err = "width NULL config recieved";
      LOG_ERROR("%s", err);
      throw err;
  }

  sycl_queue = queue(cpu_selector());
  LOG_DEBUG_0("Device selected is CPU...");
}

dpcpp_blur_udf::~dpcpp_blur_udf() {}

UdfRetCode dpcpp_blur_udf::process(cv::Mat &frame, cv::Mat &output,
                                   msg_envelope_t *meta) {
  CV_Assert(frame.isContinuous());
  buffer<uint8_t, 2> frame_buf(frame.data, range<2>(frame.rows, frame.cols));

  // sycl_queue is submiting its items to the handler context for the job to be done
  // Please refer sycl_queue documentation for more info:
  // https://docs.oneapi.io/versions/latest/dpcpp/iface/queue.html
  sycl_queue.submit([&](handler& h) {
    auto pixels = frame_buf.get_access<access::mode::read_write>(h);
    // sycl_inverse_class is used to inverse each pixel which is used
    // in the parallel execution.
    h.parallel_for<class sycl_inverse_kernel>(range<2>(1, 1), [=](item<2> item) {
        // These data is written back to the same frame buffer where actual frame data resides
        uint8_t v = pixels[item];
        pixels[item] = ~v;
    });

    // We just reusing OpenCL context/device/queue from SYCL here
    // and utilizing the same queue for blurring
    cv::blur(frame, output, cv::Size(height->body.integer,
                                     width->body.integer));
  });
  sycl_queue.wait_and_throw();

  return UdfRetCode::UDF_OK;
}
extern "C" {
/**
 * ease the process of finding UDF symbol from shared object.
 *
 * @return void*
 */
    void *initialize_udf(config_t *config) {
        eii::custom_udfs::dpcpp_blur_udf* udf = new eii::custom_udfs::dpcpp_blur_udf(config);
        return (void *)udf;
    }
};  // extern "C"
