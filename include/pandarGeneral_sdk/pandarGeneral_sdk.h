/******************************************************************************
 * Copyright 2019 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef INCLUDE_PANDAR40P_SDK_PANDAR40P_SDK_H_
#define INCLUDE_PANDAR40P_SDK_PANDAR40P_SDK_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <boost/function.hpp>
#include <string>
#include <vector>

#include "pandarGeneral/pandarGeneral.h"
#include "pandarGeneral/point_types.h"

// class PandarGeneralSDK_Internal;

class PandarGeneralSDK {
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        gps_port   				The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        start_angle       The start angle of every point cloud
   *                          should be <real angle> * 100.
   *        lidar_type        The model of the lidar
   *        frame_id          The id of the point cloud data published to ROS
   */
  PandarGeneralSDK(
      std::string device_ip, const uint16_t lidar_port, uint16_t lidar_algorithm_port, const uint16_t gps_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double)>
          pcl_callback,
      boost::function<void(HS_Object3D_Object_List *)> algorithm_callback,
      boost::function<void(double)> gps_callback, uint16_t start_angle,
      int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,  // the default timestamp type is LiDAR time
      std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag);
  /**
   * @brief Constructor
   * @param pcap_path         The path of pcap file
   *        pcl_callback      The callback of PCL data structure
   *        start_angle       The start angle of every point cloud
   *                          should be <real angle> * 100.
   *        tz                The timezone
   *        lidar_type        The model of the lidar
   *        frame_id          The id of the point cloud data published to ROS
   */
  PandarGeneralSDK(std::string pcap_path,
                   boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
                   uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id,
                   std::string timestampType, bool coordinate_correction_flag);  // the default timestamp type is LiDAR time
  ~PandarGeneralSDK();

  /**
   * @brief load the correction file
   * @param file The path of correction file
   */
  int LoadLidarCorrectionFile(std::string correction_content);
  void ResetLidarStartAngle(uint16_t start_angle);
  std::string GetLidarCalibration();
  void GetCalibrationFromDevice();
  void Start();
  void Stop();

  /** @brief get major version.
   * @Return   ： major version
   */
  int getMajorVersion();

  /**
   * @brief get minor version.
   * @Return   ： minor version
   */
  int getMinorVersion();

 private:
  PandarGeneral *pandarGeneral_;
  void *tcp_command_client_;
  boost::thread *get_calibration_thr_;
  bool enable_get_calibration_thr_;
  bool got_lidar_calibration_;
  std::string correction_content_;
  std::string correction_file_path_;
};

#endif  // INCLUDE_PANDAR40P_SDK_PANDAR40P_SDK_H_
