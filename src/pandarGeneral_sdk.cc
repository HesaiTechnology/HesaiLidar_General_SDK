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

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"
#include "pandar_log.h"
#include "src/tcp_command_client.h"
#include "version.h"
#include "yaml-cpp/yaml.h"
#include <fstream>

#define PANDARGENERALSDK_TCP_COMMAND_PORT (9347)

PandarGeneralSDK::PandarGeneralSDK(
    std::string device_ip, const uint16_t lidar_port,
    uint16_t lidar_algorithm_port, uint16_t gps_port,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    boost::function<void(HS_Object3D_Object_List *)> algorithm_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
    int pcl_type, std::string lidar_type, std::string frame_id,
    std::string timestampType, std::string lidar_correction_file,
    std::string multicast_ip, bool coordinate_correction_flag) {
  printVersion();
  pandarGeneral_ = NULL;
  // LOG_FUNC();

  pandarGeneral_ = new PandarGeneral(
      device_ip, lidar_port, lidar_algorithm_port, gps_port, pcl_callback,
      algorithm_callback, gps_callback, start_angle, tz, pcl_type, lidar_type,
      frame_id, timestampType, lidar_correction_file, multicast_ip,
      coordinate_correction_flag);

  tcp_command_client_ =
      TcpCommandClientNew(device_ip.c_str(), PANDARGENERALSDK_TCP_COMMAND_PORT);
  if (!tcp_command_client_) {
    std::cout << "Init TCP Command Client Failed" << std::endl;
  }
  get_calibration_thr_ = NULL;
  enable_get_calibration_thr_ = false;
  got_lidar_calibration_ = false;
  correction_file_path_ = lidar_correction_file;
}

PandarGeneralSDK::PandarGeneralSDK(
    std::string pcap_path,
    boost::function<void(boost::shared_ptr<PPointCloud>, double)> pcl_callback,
    uint16_t start_angle, int tz, int pcl_type, std::string lidar_type,
    std::string frame_id, std::string timestampType,
    bool coordinate_correction_flag) {
  printVersion();
  pandarGeneral_ = NULL;

  pandarGeneral_ = new PandarGeneral(pcap_path, pcl_callback, start_angle, tz,
                                     pcl_type, lidar_type, frame_id,
                                     timestampType, coordinate_correction_flag);

  get_calibration_thr_ = NULL;
  tcp_command_client_ = NULL;
  enable_get_calibration_thr_ = false;
  got_lidar_calibration_ = false;
}

PandarGeneralSDK::~PandarGeneralSDK() {
  Stop();
  if (pandarGeneral_) {
    delete pandarGeneral_;
  }
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int PandarGeneralSDK::LoadLidarCorrectionFile(std::string correction_content) {
  return pandarGeneral_->LoadCorrectionFile(correction_content);
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void PandarGeneralSDK::ResetLidarStartAngle(uint16_t start_angle) {
  if (!pandarGeneral_)
    return;
  pandarGeneral_->ResetStartAngle(start_angle);
}

std::string PandarGeneralSDK::GetLidarCalibration() {
  return correction_content_;
}

void PandarGeneralSDK::Start() {
  // LOG_FUNC();
  Stop();

  if (pandarGeneral_) {
    pandarGeneral_->Start();
  }

  enable_get_calibration_thr_ = true;
  get_calibration_thr_ = new boost::thread(
      boost::bind(&PandarGeneralSDK::GetCalibrationFromDevice, this));
}

void PandarGeneralSDK::Stop() {
  if (pandarGeneral_)
    pandarGeneral_->Stop();

  enable_get_calibration_thr_ = false;
  if (get_calibration_thr_) {
    get_calibration_thr_->join();
  }
}

void PandarGeneralSDK::GetCalibrationFromDevice() {
  // LOG_FUNC();
  if (!tcp_command_client_) {
    return;
  }
  std::cout << "Load correction file from lidar" << std::endl;
  int32_t ret = 0;
  // get lidar calibration.
  char *buffer = NULL;
  uint32_t len = 0;

  ret = TcpCommandGetLidarCalibration(tcp_command_client_, &buffer, &len);
  if (ret == 0 && buffer) {
    // success;
    correction_content_ = std::string(buffer);
    if (pandarGeneral_) {
      ret = pandarGeneral_->LoadCorrectionFile(correction_content_);
      if (ret != 0) {
        std::cout << "Load correction file from lidar failed" << std::endl;
      } else {
        std::cout << "Load correction file from lidar succeed" << std::endl;
        pandarGeneral_->SetCorrectionFileFlag(true);
      }
    }
    free(buffer);
  }
  if (!pandarGeneral_->GetCorrectionFileFlag()) {
    std::ifstream fin(correction_file_path_);
    if (fin.is_open()) {
      std::cout << "Open correction file " << correction_file_path_
                << " succeed" << std::endl;
    } else {
      std::cout << "Open correction file " << correction_file_path_ << " failed"
                << std::endl;
      return;
    }
    int length = 0;
    std::string strlidarCalibration;
    fin.seekg(0, std::ios::end);
    length = fin.tellg();
    fin.seekg(0, std::ios::beg);
    char *buffer = new char[length];
    fin.read(buffer, length);
    fin.close();
    strlidarCalibration = buffer;
    ret = pandarGeneral_->LoadCorrectionFile(strlidarCalibration);
    if (ret != 0) {
      std::cout << "Load correction file from " << correction_file_path_
                << " failed" << std::endl;
    } else {
      std::cout << "Load correction file from " << correction_file_path_
                << " succeed" << std::endl;
      pandarGeneral_->SetCorrectionFileFlag(true);
    }
  }
}

int PandarGeneralSDK::getMajorVersion() {
  if (pandarGeneral_) {
    return pandarGeneral_->getMajorVersion();
  }
  return 0;
}

int PandarGeneralSDK::getMinorVersion() {
  if (pandarGeneral_) {
    return pandarGeneral_->getMinorVersion();
  }
  return 0;
}
