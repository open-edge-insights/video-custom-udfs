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
 * @brief PCL ingestion UDF Implementation.
 */

#include "rs2pcl.h"

using namespace eii::custom_udfs;
using namespace eii::msgbus;

rs2pcl::rs2pcl(config_t *config) : RawBaseUdf(config) {
    m_frame_number = 0;
}

rs2pcl::~rs2pcl() {}

UdfRetCode rs2pcl::process(Frame* frame) {

    try {
        set_rs2_intrinsics_and_extrinsics(frame->get_meta_data());

        void* color_frame = frame->get_data(RGB_FRAME_INDEX);
        if (color_frame == NULL) {
            LOG_ERROR_0("color_frame is NULL");
        }

        void* depth_frame = frame->get_data(DEPTH_FRAME_INDEX);
        if (depth_frame == NULL) {
            LOG_ERROR_0("depth_frame is NULL");
        }

        construct_rs2_frameset(color_frame, depth_frame);

        ++m_frame_number;

        rs2::video_frame rs2_color = color_queue.wait_for_frame();
        if (rs2_color == NULL) {
            const char* err = "rs2 color frame is NULL";
            LOG_ERROR("%s", err);
            return UdfRetCode::UDF_DROP_FRAME;
        }

        rs2::depth_frame rs2_depth = depth_queue.wait_for_frame();
        if (rs2_depth == NULL) {
            const char* err = "rs2 depth frame is NULL";
            LOG_ERROR("%s", err);
            return UdfRetCode::UDF_DROP_FRAME;
        }

        m_points = m_pc.calculate(rs2_depth);
        m_pc.map_to(rs2_color);

        /*
         * Generating opensource PCL complaint point cloud.
         * The points are of type XYZ. This is a place holder call for user
         * if XYZ formatted point cloud is required.
         */

        // auto pcl_points = rs2pcl::points_to_pcl(m_points);

        /*
         * Generating point cloud formatted as per PCL opensource lib.
         * The points are of type XYZRGB type. Below lines are commented as
         * this is a reference code for manipulating PCL cloud from realsense.
         * User is expcted to write his own PCL transformation logic.
         */

        // Uncomment the below lines to do the below pcl transformation and save the pcd file
        /*
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud_pointer cloud = PCL_Conversion(m_points, rs2_color);

        pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
        Cloud_Filter.setInputCloud (cloud);           // Input generated cloud to filter
        Cloud_Filter.setFilterFieldName ("z");        // Set field name to Z-coordinate
        Cloud_Filter.setFilterLimits (0.0, 1.0);      // Set accepted interval values
        Cloud_Filter.filter (*newCloud);              // Filtered Cloud Outputted

        std::string cloudFile;
        if (m_frame_number == 200) {
            LOG_DEBUG_0("Generating PCD Point Cloud File... ");
            cloudFile = "/var/tmp/PCL_Frame" + std::to_string(m_frame_number) + ".pcd";
            pcl::io::savePCDFileASCII(cloudFile, *cloud); // Input cloud to be saved to .pcd
            usleep(1000*1000);
        }

        LOG_DEBUG("%s file generated", cloudFile.c_str());
        */
        return UdfRetCode::UDF_OK;
    } catch (const rs2::error &e) {
        LOG_ERROR("RealSense error calling %s( %s ):'\n%s",
                  e.get_failed_function(), e.get_failed_args(), e.what());
        throw e;
    } catch (...) {
        LOG_ERROR("Exception occured in native pcl udf process function");
        throw;
    }
}

extern "C" {
/**
 * ease the process of finding UDF symbol from shared object.
 *
 * @return void*
 */
    void *initialize_udf(config_t *config) {
        eii::custom_udfs::rs2pcl* udf = new eii::custom_udfs::rs2pcl(config);
        return (void *)udf;
    }
};  // extern "C"
