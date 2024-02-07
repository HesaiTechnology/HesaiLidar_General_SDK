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

#include "pandarGeneral_internal.h"

#include <sstream>

#include "input.h"
#include "log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double degreeToRadian(double degree) { return degree * M_PI / 180; }

// elevation angle of each line for HS Line 40 Lidar, Line 1 - Line 40
static const float pandar40p_elev_angle_map[] = {
    15.0f, 11.0f, 8.0f, 5.0f, 3.0f, 2.0f, 1.67f, 1.33f,
    1.0f, 0.67f, 0.33f, 0.0f, -0.33f, -0.67f, -1.0f, -1.33f,
    -1.66f, -2.0f, -2.33f, -2.67f, -3.0f, -3.33f, -3.67f, -4.0f,
    -4.33f, -4.67f, -5.0f, -5.33f, -5.67f, -6.0f, -7.0f, -8.0f,
    -9.0f, -10.0f, -11.0f, -12.0f, -13.0f, -14.0f, -19.0f, -25.0f};

static const float pandarGeneral_elev_angle_map[] = {
    14.882f, 11.032f, 8.059f, 5.057f, 3.04f, 2.028f, 1.86f, 1.688f,
    1.522f, 1.351f, 1.184f, 1.013f, 0.846f, 0.675f, 0.508f, 0.337f,
    0.169f, 0.0f, -0.169f, -0.337f, -0.508f, -0.675f, -0.845f, -1.013f,
    -1.184f, -1.351f, -1.522f, -1.688f, -1.86f, -2.028f, -2.198f, -2.365f,
    -2.536f, -2.7f, -2.873f, -3.04f, -3.21f, -3.375f, -3.548f, -3.712f,
    -3.884f, -4.05f, -4.221f, -4.385f, -4.558f, -4.72f, -4.892f, -5.057f,
    -5.229f, -5.391f, -5.565f, -5.726f, -5.898f, -6.061f, -7.063f, -8.059f,
    -9.06f, -9.885f, -11.032f, -12.006f, -12.974f, -13.93f, -18.889f, -24.897f};

// Line 40 Lidar azimuth Horizatal offset ,  Line 1 - Line 40
static const float pandar40p_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 3.125f, -5.208f,
    -1.042f, 3.125f, -5.208f, -1.042f, 3.125f, -5.208f, -1.042f, 3.125f,
    -5.208f, -1.042f, 3.125f, -5.208f, -1.042f, 3.125f, -5.208f, -1.042f,
    3.125f, -5.208f, -1.042f, 3.125f, -5.208f, -1.042f, -1.042f, -1.042f,
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};

static const float pandarGeneral_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, 1.042f, 3.125f,
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f,
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f,
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f,
    5.208f, -5.208f, -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f,
    -3.125f, -1.042f, 1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f,
    1.042f, 3.125f, 5.208f, -5.208f, -3.125f, -1.042f, -1.042f, -1.042f,
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f};

static const float pandar20_elev_angle_map[] = {
    8.0f, 5.0f, 3.0f, 2.0f, 1.67f, 1.0f, 0.33f, -0.33f,
    -1.0f, -2.0f, -3.0f, -4.0f, -5.0f, -6.0f, -8.0f, -10.0f,
    -12.0f, -14.0f, -19.0f, -25.0f};

static const float pandar20_horizatal_azimuth_offset_map[] = {
    -1.042f, -1.042f, -1.042f, -1.042f, 3.125f, -1.042f, -5.208f, 3.125f,
    -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f, -1.042f,
    -1.042f, -1.042f, -1.042f, -1.042f};

PandarGeneral_Internal::PandarGeneral_Internal(
    std::string device_ip, uint16_t lidar_port, uint16_t lidar_algorithm_port, uint16_t gps_port,
    boost::function<void(boost::shared_ptr<SVPointCloud>, double)> pcl_callback,
    boost::function<void(HS_Object3D_Object_List *)> algorithm_callback,
    boost::function<void(double)> gps_callback, uint16_t start_angle, int tz,
    int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,
    std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag) {
  // LOG_FUNC();
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_ = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  input_.reset(new Input(device_ip, lidar_port, gps_port, multicast_ip));

  start_angle_ = start_angle;
  pcl_callback_ = pcl_callback;
  gps_callback_ = gps_callback;
  last_azimuth_ = 0;
  last_timestamp_ = 0;
  m_sLidarType = lidar_type;
  frame_id_ = frame_id;
  tz_second_ = tz * 3600;
  pcl_type_ = pcl_type;
  connect_lidar_ = true;
  pcap_reader_ = NULL;
  m_sTimestampType = timestampType;
  m_dPktTimestamp = 0.0f;

  pthread_mutex_init(&m_mutexAlgorithmListLock, NULL);
  sem_init(&m_semAlgorithmList, 0, 0);
  m_threadLidarAlgorithmRecv = NULL;
  m_threadLidarAlgorithmProcess = NULL;
  m_fAlgorithmCallback = NULL;
  m_bEnableLidarAlgorithmRecvThread = false;
  m_bEnableLidarAlgorithmProcessThread = false;
  m_bGetVersion = false;
  m_iMajorVersion = 0;
  m_iMinorVersion = 0;
  m_iHeaderSize = 0;
  m_iRegularInfoLen = 0;
  m_u16LidarAlgorithmPort = 0;
  correction_file_path_ = lidar_correction_file;
  m_bCoordinateCorrectionFlag = coordinate_correction_flag;
  m_iAzimuthRange = MAX_AZIMUTH_DEGREE_NUM;
  if (0 != lidar_algorithm_port) {
    m_u16LidarAlgorithmPort = lidar_algorithm_port;
    m_spAlgorithmPktInput.reset(new Input(device_ip, m_u16LidarAlgorithmPort, 0));
  }
  if (NULL != algorithm_callback) {
    m_fAlgorithmCallback = algorithm_callback;
  }
  Init();
}

PandarGeneral_Internal::PandarGeneral_Internal(std::string pcap_path,
                                               boost::function<void(boost::shared_ptr<SVPointCloud>, double)>
                                                   pcl_callback,
                                               uint16_t start_angle, int tz, int pcl_type,
                                               std::string lidar_type, std::string frame_id,
                                               std::string timestampType, bool coordinate_correction_flag) {
  pthread_mutex_init(&lidar_lock_, NULL);
  sem_init(&lidar_sem_, 0, 0);

  lidar_recv_thr_ = NULL;
  lidar_process_thr_ = NULL;

  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;

  pcap_reader_ = new PcapReader(pcap_path, lidar_type);

  start_angle_ = start_angle;
  pcl_callback_ = pcl_callback;
  gps_callback_ = NULL;
  last_azimuth_ = 0;
  last_timestamp_ = 0;
  m_sLidarType = lidar_type;
  frame_id_ = frame_id;
  tz_second_ = tz * 3600;
  pcl_type_ = pcl_type;
  connect_lidar_ = false;
  m_sTimestampType = timestampType;
  m_iAzimuthRange = MAX_AZIMUTH_DEGREE_NUM;
  m_dPktTimestamp = 0.0f;
  m_bCoordinateCorrectionFlag = coordinate_correction_flag;

  Init();
}

PandarGeneral_Internal::~PandarGeneral_Internal() {
  Stop();
  sem_destroy(&lidar_sem_);
  pthread_mutex_destroy(&lidar_lock_);

  if (pcap_reader_ != NULL) {
    delete pcap_reader_;
    pcap_reader_ = NULL;
  }
}

void PandarGeneral_Internal::Init() {
  m_iPointCloudIndex = 0;
  m_vPointCloudList.resize(MAX_LASER_NUM);
  m_vPointCloud.resize(MAX_POINT_CLOUD_NUM);
  for (uint16_t rotIndex = 0; rotIndex < ROTATION_MAX_UNITS; ++rotIndex) {
    float rotation = degreeToRadian(0.01 * static_cast<double>(rotIndex));
    cos_lookup_table_[rotIndex] = cosf(rotation);
    sin_lookup_table_[rotIndex] = sinf(rotation);
  }
  m_sin_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_.resize(MAX_AZIMUTH_DEGREE_NUM);
  for (int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    m_sin_azimuth_map_[i] = sinf(i * M_PI / 18000);
    m_cos_azimuth_map_[i] = cosf(i * M_PI / 18000);
  }
  m_sin_azimuth_map_h.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_h.resize(MAX_AZIMUTH_DEGREE_NUM);
  for (int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    if (m_sLidarType == "PandarXTM") {
      m_sin_azimuth_map_h[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_H;
      m_cos_azimuth_map_h[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_H;
    } else {
      m_sin_azimuth_map_h[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_H;
      m_cos_azimuth_map_h[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_H;
    }
  }
  m_sin_azimuth_map_b.resize(MAX_AZIMUTH_DEGREE_NUM);
  m_cos_azimuth_map_b.resize(MAX_AZIMUTH_DEGREE_NUM);
  for (int i = 0; i < MAX_AZIMUTH_DEGREE_NUM; ++i) {
    if (m_sLidarType == "PandarXTM") {
      m_sin_azimuth_map_b[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_B;
      m_cos_azimuth_map_b[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XTM_COORDINATE_CORRECTION_B;
    } else {
      m_sin_azimuth_map_b[i] = sinf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_B;
      m_cos_azimuth_map_b[i] = cosf(i * M_PI / 18000) * HS_LIDAR_XT_COORDINATE_CORRECTION_B;
    }
  }
  if (pcl_type_) {
    for (int i = 0; i < MAX_LASER_NUM; i++) {
      m_vPointCloudList[i].reserve(MAX_POINT_CLOUD_NUM_PER_CHANNEL);
    }
  }

  // laser40
  //  init the block time offset, us
  block40OffsetSingle_[9] = 55.56f * 0.0f + 28.58f;
  block40OffsetSingle_[8] = 55.56f * 1.0f + 28.58f;
  block40OffsetSingle_[7] = 55.56f * 2.0f + 28.58f;
  block40OffsetSingle_[6] = 55.56f * 3.0f + 28.58f;
  block40OffsetSingle_[5] = 55.56f * 4.0f + 28.58f;
  block40OffsetSingle_[4] = 55.56f * 5.0f + 28.58f;
  block40OffsetSingle_[3] = 55.56f * 6.0f + 28.58f;
  block40OffsetSingle_[2] = 55.56f * 7.0f + 28.58f;
  block40OffsetSingle_[1] = 55.56f * 8.0f + 28.58f;
  block40OffsetSingle_[0] = 55.56f * 9.0f + 28.58f;

  block40OffsetDual_[9] = 55.56f * 0.0f + 28.58f;
  block40OffsetDual_[8] = 55.56f * 0.0f + 28.58f;
  block40OffsetDual_[7] = 55.56f * 1.0f + 28.58f;
  block40OffsetDual_[6] = 55.56f * 1.0f + 28.58f;
  block40OffsetDual_[5] = 55.56f * 2.0f + 28.58f;
  block40OffsetDual_[4] = 55.56f * 2.0f + 28.58f;
  block40OffsetDual_[3] = 55.56f * 3.0f + 28.58f;
  block40OffsetDual_[2] = 55.56f * 3.0f + 28.58f;
  block40OffsetDual_[1] = 55.56f * 4.0f + 28.58f;
  block40OffsetDual_[0] = 55.56f * 4.0f + 28.58f;

  // init the laser shot time offset, us
  laser40Offset_[3] = 3.62f;
  laser40Offset_[39] = 3.62f;
  laser40Offset_[35] = 4.92f;
  laser40Offset_[27] = 6.23f;
  laser40Offset_[11] = 8.19f;
  laser40Offset_[15] = 8.19f;
  laser40Offset_[31] = 9.5f;
  laser40Offset_[23] = 11.47f;
  laser40Offset_[28] = 12.77f;
  laser40Offset_[16] = 14.74f;
  laser40Offset_[2] = 16.04f;
  laser40Offset_[38] = 16.04f;
  laser40Offset_[34] = 17.35f;
  laser40Offset_[24] = 18.65f;
  laser40Offset_[8] = 20.62f;
  laser40Offset_[12] = 20.62f;
  laser40Offset_[30] = 21.92f;
  laser40Offset_[20] = 23.89f;
  laser40Offset_[25] = 25.19f;
  laser40Offset_[13] = 27.16f;
  laser40Offset_[1] = 28.47f;
  laser40Offset_[37] = 28.47f;
  laser40Offset_[33] = 29.77f;
  laser40Offset_[5] = 31.74f;
  laser40Offset_[21] = 31.7447f;
  laser40Offset_[9] = 33.71f;
  laser40Offset_[29] = 35.01f;
  laser40Offset_[17] = 36.98f;
  laser40Offset_[22] = 38.95f;
  laser40Offset_[10] = 40.91f;
  laser40Offset_[0] = 42.22f;
  laser40Offset_[36] = 42.22f;
  laser40Offset_[32] = 43.52f;
  laser40Offset_[4] = 45.49f;
  laser40Offset_[18] = 45.49f;
  laser40Offset_[6] = 47.46f;
  laser40Offset_[26] = 48.76f;
  laser40Offset_[14] = 50.73f;
  laser40Offset_[19] = 52.7f;
  laser40Offset_[7] = 54.67f;

  // laser64 init the laser shot time offset, us
  //  init the block time offset, us
  block64OffsetSingle_[5] = 55.56f * 0.0f + 42.58f;
  block64OffsetSingle_[4] = 55.56f * 1.0f + 42.58f;
  block64OffsetSingle_[3] = 55.56f * 2.0f + 42.58f;
  block64OffsetSingle_[2] = 55.56f * 3.0f + 42.58f;
  block64OffsetSingle_[1] = 55.56f * 4.0f + 42.58f;
  block64OffsetSingle_[0] = 55.56f * 5.0f + 42.58f;

  block64OffsetDual_[5] = 55.56f * 0.0f + 42.58f;
  block64OffsetDual_[4] = 55.56f * 0.0f + 42.58f;
  block64OffsetDual_[3] = 55.56f * 1.0f + 42.58f;
  block64OffsetDual_[2] = 55.56f * 1.0f + 42.58f;
  block64OffsetDual_[1] = 55.56f * 2.0f + 42.58f;
  block64OffsetDual_[0] = 55.56f * 2.0f + 42.58f;

  laser64Offset_[50] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[60] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[44] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[59] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[38] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[56] = 1.304f * 2.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[8] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[54] = 1.304f * 3.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[48] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[62] = 1.304f * 4.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[42] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[58] = 1.304f * 5.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[6] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[55] = 1.304f * 6.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[52] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[63] = 1.304f * 7.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[46] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[61] = 1.304f * 8.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[40] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[57] = 1.304f * 9.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[5] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[53] = 1.304f * 10.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[4] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[47] = 1.304f * 11.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[3] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[49] = 1.304f * 12.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[2] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[51] = 1.304f * 13.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[1] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[45] = 1.304f * 14.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[0] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[43] = 1.304f * 15.0f + 1.968f * 0.0f + 3.62f;
  laser64Offset_[23] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
  laser64Offset_[32] = 1.304f * 15.0f + 1.968f * 1.0f + 3.62f;
  laser64Offset_[26] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
  laser64Offset_[41] = 1.304f * 15.0f + 1.968f * 2.0f + 3.62f;
  laser64Offset_[20] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
  laser64Offset_[35] = 1.304f * 15.0f + 1.968f * 3.0f + 3.62f;
  laser64Offset_[14] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
  laser64Offset_[29] = 1.304f * 15.0f + 1.968f * 4.0f + 3.62f;
  laser64Offset_[21] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
  laser64Offset_[36] = 1.304f * 15.0f + 1.968f * 5.0f + 3.62f;
  laser64Offset_[15] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
  laser64Offset_[30] = 1.304f * 15.0f + 1.968f * 6.0f + 3.62f;
  laser64Offset_[9] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
  laser64Offset_[24] = 1.304f * 15.0f + 1.968f * 7.0f + 3.62f;
  laser64Offset_[18] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
  laser64Offset_[33] = 1.304f * 15.0f + 1.968f * 8.0f + 3.62f;
  laser64Offset_[12] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
  laser64Offset_[27] = 1.304f * 15.0f + 1.968f * 9.0f + 3.62f;
  laser64Offset_[19] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
  laser64Offset_[34] = 1.304f * 15.0f + 1.968f * 10.0f + 3.62f;
  laser64Offset_[13] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
  laser64Offset_[28] = 1.304f * 15.0f + 1.968f * 11.0f + 3.62f;
  laser64Offset_[7] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
  laser64Offset_[22] = 1.304f * 15.0f + 1.968f * 12.0f + 3.62f;
  laser64Offset_[16] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser64Offset_[31] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser64Offset_[10] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
  laser64Offset_[25] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;
  laser64Offset_[17] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
  laser64Offset_[37] = 1.304f * 15.0f + 1.968f * 15.0f + 3.62f;
  laser64Offset_[11] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;
  laser64Offset_[39] = 1.304f * 15.0f + 1.968f * 16.0f + 3.62f;

  for (int i = 0; i < HS_LIDAR_L20_BLOCK_NUMBER; i++) {
    block20OffsetSingle_[i] = 28.58f + (HS_LIDAR_L20_BLOCK_NUMBER - 1 - i) * 55.56f;
    block20OffsetDual_[i] = 28.58f +
                            static_cast<int>((HS_LIDAR_L20_BLOCK_NUMBER - 1 - i) / 2) * 55.56f;
  }

  laser20AOffset_[1] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser20AOffset_[19] = 1.304f * 0.0f + 1.968f * 0.0f + 3.62f;
  laser20AOffset_[16] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser20AOffset_[14] = 1.304f * 3.0f + 1.968f * 1.0f + 3.62f;
  laser20AOffset_[11] = 1.304f * 3.0f + 1.968f * 2.0f + 3.62f;
  laser20AOffset_[0] = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
  laser20AOffset_[18] = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
  laser20AOffset_[5] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20AOffset_[7] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20AOffset_[10] = 1.304f * 8.0f + 1.968f * 5.0f + 3.62f;
  laser20AOffset_[17] = 1.304f * 10.0f + 1.968f * 6.0f + 3.62f;
  laser20AOffset_[15] = 1.304f * 11.0f + 1.968f * 6.0f + 3.62f;
  laser20AOffset_[3] = 1.304f * 11.0f + 1.968f * 7.0f + 3.62f;
  laser20AOffset_[13] = 1.304f * 12.0f + 1.968f * 8.0f + 3.62f;
  laser20AOffset_[9] = 1.304f * 12.0f + 1.968f * 9.0f + 3.62f;
  laser20AOffset_[6] = 1.304f * 12.0f + 1.968f * 11.0f + 3.62f;
  laser20AOffset_[2] = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
  laser20AOffset_[4] = 1.304f * 14.0f + 1.968f * 13.0f + 3.62f;
  laser20AOffset_[12] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser20AOffset_[8] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;

  laser20BOffset_[17] = 1.304f * 1.0f + 1.968f * 0.0f + 3.62f;
  laser20BOffset_[5] = 1.304f * 2.0f + 1.968f * 1.0f + 3.62f;
  laser20BOffset_[15] = 1.304f * 3.0f + 1.968f * 1.0f + 3.62f;
  laser20BOffset_[11] = 1.304f * 3.0f + 1.968f * 2.0f + 3.62f;
  laser20BOffset_[8] = 1.304f * 4.0f + 1.968f * 3.0f + 3.62f;
  laser20BOffset_[19] = 1.304f * 5.0f + 1.968f * 3.0f + 3.62f;
  laser20BOffset_[3] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20BOffset_[6] = 1.304f * 7.0f + 1.968f * 4.0f + 3.62f;
  laser20BOffset_[14] = 1.304f * 8.0f + 1.968f * 4.0f + 3.62f;
  laser20BOffset_[10] = 1.304f * 8.0f + 1.968f * 5.0f + 3.62f;
  laser20BOffset_[18] = 1.304f * 10.0f + 1.968f * 6.0f + 3.62f;
  laser20BOffset_[16] = 1.304f * 11.0f + 1.968f * 6.0f + 3.62f;
  laser20BOffset_[1] = 1.304f * 11.0f + 1.968f * 7.0f + 3.62f;
  laser20BOffset_[13] = 1.304f * 12.0f + 1.968f * 8.0f + 3.62f;
  laser20BOffset_[4] = 1.304f * 12.0f + 1.968f * 11.0f + 3.62f;
  laser20BOffset_[0] = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
  laser20BOffset_[9] = 1.304f * 14.0f + 1.968f * 12.0f + 3.62f;
  laser20BOffset_[2] = 1.304f * 14.0f + 1.968f * 13.0f + 3.62f;
  laser20BOffset_[12] = 1.304f * 15.0f + 1.968f * 13.0f + 3.62f;
  laser20BOffset_[7] = 1.304f * 15.0f + 1.968f * 14.0f + 3.62f;

  // QT
  blockQTOffsetSingle_[0] = 25.71f;
  blockQTOffsetSingle_[1] = 25.71f + 166.67f;
  blockQTOffsetSingle_[2] = 25.71f + 333.33f;
  blockQTOffsetSingle_[3] = 25.71f + 500.00f;

  blockQTOffsetDual_[0] = 25.71f;
  blockQTOffsetDual_[1] = 25.71f;
  blockQTOffsetDual_[2] = 25.71f + 166.67f;
  blockQTOffsetDual_[3] = 25.71f + 166.67f;

  laserQTOffset_[0] = 10.0f + 2.31f;
  laserQTOffset_[1] = 10.0f + 4.37f;
  laserQTOffset_[2] = 10.0f + 6.43f;
  laserQTOffset_[3] = 10.0f + 8.49f;
  laserQTOffset_[4] = 10.0f + 10.54f;
  laserQTOffset_[5] = 10.0f + 12.60f;
  laserQTOffset_[6] = 10.0f + 14.66f;
  laserQTOffset_[7] = 10.0f + 16.71f;
  laserQTOffset_[8] = 10.0f + 19.16f;
  laserQTOffset_[9] = 10.0f + 21.22f;
  laserQTOffset_[10] = 10.0f + 23.28f;
  laserQTOffset_[11] = 10.0f + 25.34f;
  laserQTOffset_[12] = 10.0f + 27.39f;
  laserQTOffset_[13] = 10.0f + 29.45f;
  laserQTOffset_[14] = 10.0f + 31.50f;
  laserQTOffset_[15] = 10.0f + 33.56f;

  laserQTOffset_[16] = 10.0f + 36.61f;
  laserQTOffset_[17] = 10.0f + 38.67f;
  laserQTOffset_[18] = 10.0f + 40.73f;
  laserQTOffset_[19] = 10.0f + 42.78f;
  laserQTOffset_[20] = 10.0f + 44.84f;
  laserQTOffset_[21] = 10.0f + 46.90f;
  laserQTOffset_[22] = 10.0f + 48.95f;
  laserQTOffset_[23] = 10.0f + 51.01f;
  laserQTOffset_[24] = 10.0f + 53.45f;
  laserQTOffset_[25] = 10.0f + 55.52f;
  laserQTOffset_[26] = 10.0f + 57.58f;
  laserQTOffset_[27] = 10.0f + 59.63f;
  laserQTOffset_[28] = 10.0f + 61.69f;
  laserQTOffset_[29] = 10.0f + 63.74f;
  laserQTOffset_[30] = 10.0f + 65.80f;
  laserQTOffset_[31] = 10.0f + 67.86f;

  laserQTOffset_[32] = 10.0f + 70.90f;
  laserQTOffset_[33] = 10.0f + 72.97f;
  laserQTOffset_[34] = 10.0f + 75.02f;
  laserQTOffset_[35] = 10.0f + 77.08f;
  laserQTOffset_[36] = 10.0f + 79.14f;
  laserQTOffset_[37] = 10.0f + 81.19f;
  laserQTOffset_[38] = 10.0f + 83.25f;
  laserQTOffset_[39] = 10.0f + 85.30f;
  laserQTOffset_[40] = 10.0f + 87.75f;
  laserQTOffset_[41] = 10.0f + 89.82f;
  laserQTOffset_[42] = 10.0f + 91.87f;
  laserQTOffset_[43] = 10.0f + 93.93f;
  laserQTOffset_[44] = 10.0f + 95.98f;
  laserQTOffset_[45] = 10.0f + 98.04f;
  laserQTOffset_[46] = 10.0f + 100.10f;
  laserQTOffset_[47] = 10.0f + 102.15f;

  laserQTOffset_[48] = 10.0f + 105.20f;
  laserQTOffset_[49] = 10.0f + 107.26f;
  laserQTOffset_[50] = 10.0f + 109.32f;
  laserQTOffset_[51] = 10.0f + 111.38f;
  laserQTOffset_[52] = 10.0f + 113.43f;
  laserQTOffset_[53] = 10.0f + 115.49f;
  laserQTOffset_[54] = 10.0f + 117.54f;
  laserQTOffset_[55] = 10.0f + 119.60f;
  laserQTOffset_[56] = 10.0f + 122.05f;
  laserQTOffset_[57] = 10.0f + 124.11f;
  laserQTOffset_[58] = 10.0f + 126.17f;
  laserQTOffset_[59] = 10.0f + 128.22f;
  laserQTOffset_[60] = 10.0f + 130.28f;
  laserQTOffset_[61] = 10.0f + 132.34f;
  laserQTOffset_[62] = 10.0f + 134.39f;
  laserQTOffset_[63] = 10.0f + 136.45f;

  if (m_sLidarType == "Pandar40P" || m_sLidarType == "Pandar40M") {
    m_sin_elevation_map_.resize(LASER_COUNT);
    m_cos_elevation_map_.resize(LASER_COUNT);
    for (int i = 0; i < LASER_COUNT; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandar40p_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandar40p_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandar40p_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] =
          pandar40p_horizatal_azimuth_offset_map[i];
    }
  }

  if (m_sLidarType == "Pandar64") {
    m_sin_elevation_map_.resize(HS_LIDAR_L64_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_L64_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_L64_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarGeneral_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarGeneral_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandarGeneral_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] =
          pandarGeneral_horizatal_azimuth_offset_map[i];
    }
  }

  if (m_sLidarType == "Pandar20A" || m_sLidarType == "Pandar20B") {
    m_sin_elevation_map_.resize(HS_LIDAR_L20_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_L20_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_L20_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandar20_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandar20_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandar20_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] = pandar20_horizatal_azimuth_offset_map[i];
    }
  }

  if (m_sLidarType == "PandarQT") {
    m_sin_elevation_map_.resize(HS_LIDAR_QT_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_QT_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_QT_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarQT_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarQT_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandarQT_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] = pandarQT_horizatal_azimuth_offset_map[i];
    }
  }

  if (m_sLidarType == "PandarXT-32") {
    m_sin_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_XT_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarXT_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarXT_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandarXT_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i];
      laserXTOffset_[i] = laserXTOffset[i];
    }
  }

  if (m_sLidarType == "PandarXT-16") {
    m_sin_elevation_map_.resize(HS_LIDAR_XT16_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_XT16_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_XT16_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarXT_elev_angle_map[i * 2]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarXT_elev_angle_map[i * 2]));
      General_elev_angle_map_[i] = pandarXT_elev_angle_map[i * 2];
      General_horizatal_azimuth_offset_map_[i] = pandarXT_horizatal_azimuth_offset_map[i * 2];
      laserXTOffset_[i] = laserXTOffset[i * 2];
    }
  }

  for (int i = 0; i < HS_LIDAR_XT_BLOCK_NUMBER; i++) {
    blockXTOffsetSingle_[i] = blockXTOffsetSingle[i];
    blockXTOffsetDual_[i] = blockXTOffsetDual[i];
  }

  if (m_sLidarType == "PandarXTM") {
    m_sin_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    m_cos_elevation_map_.resize(HS_LIDAR_XT_UNIT_NUM);
    for (int i = 0; i < HS_LIDAR_XT_UNIT_NUM; i++) {
      m_sin_elevation_map_[i] = sinf(degreeToRadian(pandarXTM_elev_angle_map[i]));
      m_cos_elevation_map_[i] = cosf(degreeToRadian(pandarXTM_elev_angle_map[i]));
      General_elev_angle_map_[i] = pandarXTM_elev_angle_map[i];
      General_horizatal_azimuth_offset_map_[i] = pandarXTM_horizatal_azimuth_offset_map[i];
      laserXTOffset_[i] = laserXTMOffset[i];
    }
    for (int i = 0; i < HS_LIDAR_XT_BLOCK_NUMBER; i++) {
      blockXTOffsetSingle_[i] = blockXTMOffsetSingle[i];
      blockXTOffsetDual_[i] = blockXTMOffsetDual[i];
      blockXTOffsetTriple_[i] = blockXTMOffsetTriple[i];
    }
  }

  SetEnvironmentVariableTZ();
}

/**
 * @brief load the correction file
 * @param file The path of correction file
 */
int PandarGeneral_Internal::LoadCorrectionFile(std::string correction_content) {
  // LOG_FUNC();
  // LOG_D("stirng:[%s]",correction_content.c_str());
  std::istringstream ifs(correction_content);

  std::string line;
  if (std::getline(ifs, line)) {  // first line "Laser id,Elevation,Azimuth"
    std::cout << "Parse Lidar Correction..." << std::endl;
  }

  double azimuthOffset[HS_LIDAR_L64_UNIT_NUM];
  double elev_angle[HS_LIDAR_L64_UNIT_NUM];

  int lineCounter = 0;
  while (std::getline(ifs, line)) {
    // correction file has 3 columns, min length is len(0,0,0)
    if (line.length() < 5) {
      break;
    }
    lineCounter++;

    int lineId = 0;
    double elev, azimuth;

    std::stringstream ss(line);
    std::string subline;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> lineId;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> elev;
    std::getline(ss, subline, ',');
    std::stringstream(subline) >> azimuth;

    if (lineId != lineCounter) {
      break;
    }

    elev_angle[lineId - 1] = elev;
    azimuthOffset[lineId - 1] = azimuth;
  }
  m_sin_elevation_map_.resize(lineCounter);
  m_cos_elevation_map_.resize(lineCounter);
  for (int i = 0; i < lineCounter; ++i) {
    /* for all the laser offset */
    General_elev_angle_map_[i] = elev_angle[i];
    m_sin_elevation_map_[i] = sinf(degreeToRadian(General_elev_angle_map_[i]));
    m_cos_elevation_map_[i] = cosf(degreeToRadian(General_elev_angle_map_[i]));
    General_horizatal_azimuth_offset_map_[i] = azimuthOffset[i];
  }

  return 0;
}

/**
 * @brief load the correction file
 * @param angle The start angle
 */
void PandarGeneral_Internal::ResetStartAngle(uint16_t start_angle) {
  start_angle_ = start_angle;
}

void PandarGeneral_Internal::Start() {
  // LOG_FUNC();
  Stop();
  enable_lidar_recv_thr_ = true;
  enable_lidar_process_thr_ = true;
  lidar_process_thr_ = new boost::thread(
      boost::bind(&PandarGeneral_Internal::ProcessLiarPacket, this));

  if (connect_lidar_) {
    lidar_recv_thr_ =
        new boost::thread(boost::bind(&PandarGeneral_Internal::RecvTask, this));
  } else {
    pcap_reader_->start(boost::bind(&PandarGeneral_Internal::FillPacket, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  if (0 != m_u16LidarAlgorithmPort) {
    m_bEnableLidarAlgorithmRecvThread = true;
    m_bEnableLidarAlgorithmProcessThread = true;
    m_threadLidarAlgorithmProcess = new boost::thread(boost::bind(&PandarGeneral_Internal::ProcessAlgorithmPacket, this));
    m_threadLidarAlgorithmRecv = new boost::thread(boost::bind(&PandarGeneral_Internal::recvAlgorithmPacket, this));
  }
}

void PandarGeneral_Internal::Stop() {
  enable_lidar_recv_thr_ = false;
  enable_lidar_process_thr_ = false;
  m_bEnableLidarAlgorithmRecvThread = false;
  m_bEnableLidarAlgorithmProcessThread = false;

  if (lidar_process_thr_) {
    lidar_process_thr_->interrupt();
    lidar_process_thr_->join();
    delete lidar_process_thr_;
    lidar_process_thr_ = NULL;
  }

  if (lidar_recv_thr_) {
    lidar_recv_thr_->interrupt();
    lidar_recv_thr_->join();
    delete lidar_recv_thr_;
    lidar_recv_thr_ = NULL;
  }

  if (pcap_reader_ != NULL) {
    pcap_reader_->stop();
  }

  if (m_bEnableLidarAlgorithmRecvThread) {
    m_threadLidarAlgorithmRecv->interrupt();
    m_threadLidarAlgorithmRecv->join();
    delete m_threadLidarAlgorithmRecv;
    m_threadLidarAlgorithmRecv = NULL;
  }

  if (m_bEnableLidarAlgorithmProcessThread) {
    m_threadLidarAlgorithmProcess->interrupt();
    m_threadLidarAlgorithmProcess->join();
    delete m_threadLidarAlgorithmProcess;
    m_threadLidarAlgorithmProcess = NULL;
  }
  m_listAlgorithmPacket.clear();

  return;
}

void PandarGeneral_Internal::RecvTask() {
  // LOG_FUNC();
  int ret = 0;
  sched_param param;
  int ret_policy;
  // SCHED_FIFO和SCHED_RR
  param.sched_priority = 99;
  int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &param);
  printf("publishRawDataThread:set result [%d]\n", rc);
  pthread_getschedparam(pthread_self(), &ret_policy, &param);
  printf("publishRawDataThread:get thead %lu, policy %d and priority %d\n",
         pthread_self(), ret_policy, param.sched_priority);
  while (enable_lidar_recv_thr_) {
    boost::this_thread::interruption_point();
    PandarPacket pkt;
    int rc = input_->getPacket(&pkt);
    if (rc == -1) {
      continue;
    }

    if (pkt.size == GPS_PACKET_SIZE) {
      PandarGPS gpsMsg;
      ret = ParseGPS(&gpsMsg, pkt.data, pkt.size);
      if (ret == 0) {
        ProcessGps(gpsMsg);
      }
      continue;
    }
    PushLiDARData(pkt);
  }
}

void PandarGeneral_Internal::FillPacket(const uint8_t *buf, const int len, double timestamp) {
  if (len != GPS_PACKET_SIZE) {
    PandarPacket pkt;
    memcpy(pkt.data, buf, len);
    pkt.size = len;
    pkt.stamp = timestamp;
    PushLiDARData(pkt);
  }
}

void PandarGeneral_Internal::ProcessLiarPacket() {
  // LOG_FUNC();
  double lastTimestamp = 0.0f;
  struct timespec ts;
  int ret = 0;

  boost::shared_ptr<SVPointCloud> outMsg(new SVPointCloud());

  while (enable_lidar_process_thr_) {
    boost::this_thread::interruption_point();
    if (!m_PacketsBuffer.hasEnoughPackets()) {
      usleep(1000);
      continue;
    }
    PandarPacket packet = *(m_PacketsBuffer.getIterCalc());
    m_PacketsBuffer.moveIterCalc();
    m_dPktTimestamp = packet.stamp;

    if (packet.size == PACKET_SIZE || packet.size == PACKET_SIZE + SEQ_NUM_SIZE) {
      Pandar40PPacket pkt;
      ret = ParseRawData(&pkt, packet.data, packet.size);

      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < BLOCKS_PER_PACKET; ++i) {
        int azimuthGap = 0;      /* To do */
        double timestampGap = 0; /* To do */

        if (last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (MAX_AZIMUTH_DEGREE_NUM - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }
        timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
        if (last_azimuth_ != pkt.blocks[i].azimuth &&
            (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && (m_iPointCloudIndex > 0 || m_vPointCloudList[0].size() > 0)) {
              EmitBackMessege(LASER_COUNT, outMsg);
            }
          }
        }
        CalcPointXYZIT(&pkt, i, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
        last_timestamp_ = pkt.timestamp_point;
      }
    } else if (packet.size == HS_LIDAR_L64_6PACKET_SIZE ||
               packet.size == HS_LIDAR_L64_7PACKET_SIZE ||
               packet.size == HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE ||
               packet.size == HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE) {
      HS_LIDAR_L64_Packet pkt;
      ret = ParseL64Data(&pkt, packet.data, packet.size);

      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
        int azimuthGap = 0;      /* To do */
        double timestampGap = 0; /* To do */
        if (last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (MAX_AZIMUTH_DEGREE_NUM - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }
        timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
        if (last_azimuth_ != pkt.blocks[i].azimuth &&
            (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && (m_iPointCloudIndex > 0 || m_vPointCloudList[0].size() > 0)) {
              EmitBackMessege(pkt.header.chLaserNumber, outMsg);
            }
          }
        } else {
          // printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth, azimuthGap);
        }

        CalcL64PointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
        last_timestamp_ = pkt.timestamp_point;
      }
    } else if (HS_LIDAR_L20_PACKET_SIZE == packet.size) {
      HS_LIDAR_L20_Packet pkt;
      ret = ParseL20Data(&pkt, packet.data, packet.size);

      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
        int azimuthGap = 0;      /* To do */
        double timestampGap = 0; /* To do */
        if (last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (MAX_AZIMUTH_DEGREE_NUM - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }
        timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
        if (last_azimuth_ != pkt.blocks[i].azimuth &&
            (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && (m_iPointCloudIndex > 0 || m_vPointCloudList[0].size() > 0)) {
              EmitBackMessege(pkt.header.chLaserNumber, outMsg);
            }
          }
        } else {
          // printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth, azimuthGap);
        }
        CalcL20PointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
        last_timestamp_ = pkt.timestamp_point;
      }
    } else if (packet.size == HS_LIDAR_QT_PACKET_SIZE ||
               packet.size == HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE) {
      HS_LIDAR_QT_Packet pkt;
      ret = ParseQTData(&pkt, packet.data, packet.size);
      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
        int azimuthGap = 0;      /* To do */
        double timestampGap = 0; /* To do */
        if (last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (MAX_AZIMUTH_DEGREE_NUM - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }
        timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
        if (last_azimuth_ != pkt.blocks[i].azimuth &&
            (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && (m_iPointCloudIndex > 0 || m_vPointCloudList[0].size() > 0)) {
              EmitBackMessege(pkt.header.chLaserNumber, outMsg);
            }
          }
        } else {
          // printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth, azimuthGap);
        }
        CalcQTPointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
        last_timestamp_ = pkt.timestamp_point;
      }
    } else if ((packet.size == HS_LIDAR_XT_PACKET_SIZE && (m_sLidarType == "XT" || m_sLidarType == "PandarXT-32")) ||
               (packet.size == HS_LIDAR_XT16_PACKET_SIZE && (m_sLidarType == "PandarXT-16")) ||
               (packet.size == HS_LIDAR_XTM_PACKET_SIZE && (m_sLidarType == "PandarXTM"))) {
      HS_LIDAR_XT_Packet pkt;
      ret = ParseXTData(&pkt, packet.data, packet.size);
      if (ret != 0) {
        continue;
      }

      for (int i = 0; i < pkt.header.chBlockNumber; ++i) {
        int azimuthGap = 0;      /* To do */
        double timestampGap = 0; /* To do */
        if (last_azimuth_ > pkt.blocks[i].azimuth) {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) + (MAX_AZIMUTH_DEGREE_NUM - static_cast<int>(last_azimuth_));
        } else {
          azimuthGap = static_cast<int>(pkt.blocks[i].azimuth) - static_cast<int>(last_azimuth_);
        }
        timestampGap = pkt.timestamp_point - last_timestamp_ + 0.001;
        if (last_azimuth_ != pkt.blocks[i].azimuth &&
            (azimuthGap / timestampGap) < MAX_AZIMUTH_DEGREE_NUM * 100) {
          /* for all the blocks */
          if ((last_azimuth_ > pkt.blocks[i].azimuth &&
               start_angle_ <= pkt.blocks[i].azimuth) ||
              (last_azimuth_ < start_angle_ &&
               start_angle_ <= pkt.blocks[i].azimuth)) {
            if (pcl_callback_ && (m_iPointCloudIndex > 0 || m_vPointCloudList[0].size() > 0)) {
              EmitBackMessege(pkt.header.chLaserNumber, outMsg);
            }
          }
        } else {
          // printf("last_azimuth_:%d pkt.blocks[i].azimuth:%d  *******azimuthGap:%d\n", last_azimuth_, pkt.blocks[i].azimuth, azimuthGap);
        }
        CalcXTPointXYZIT(&pkt, i, pkt.header.chLaserNumber, outMsg);
        last_azimuth_ = pkt.blocks[i].azimuth;
        last_timestamp_ = pkt.timestamp_point;
      }
    } else {
      continue;
    }

    outMsg->header.frame_id = frame_id_;
    outMsg->height = 1;
  }
}

void PandarGeneral_Internal::PushLiDARData(PandarPacket packet) {
  m_PacketsBuffer.push_back(packet);
}

void PandarGeneral_Internal::ProcessGps(const PandarGPS &gpsMsg) {
  struct tm t = {0};
  t.tm_sec = gpsMsg.second;
  t.tm_min = gpsMsg.minute;

  t.tm_hour = gpsMsg.hour;
  t.tm_mday = gpsMsg.day;

  // UTC's month start from 1, but mktime only accept month from 0.
  t.tm_mon = gpsMsg.month - 1;
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  t.tm_year = gpsMsg.year + 100;
  t.tm_isdst = 0;

  if (gps_callback_) {
    gps_callback_(static_cast<double>(mktime(&t) + tz_second_));
  }
}

int PandarGeneral_Internal::ParseRawData(Pandar40PPacket *packet,
                                         const uint8_t *buf, const int len) {
  if (len != PACKET_SIZE && len != PACKET_SIZE + SEQ_NUM_SIZE) {
    std::cout << "packet size mismatch PandarGeneral_Internal " << len << ","
              << PACKET_SIZE << std::endl;
    return -1;
  }

  int index = 0;
  // 10 BLOCKs
  for (int i = 0; i < BLOCKS_PER_PACKET; i++) {
    Pandar40PBlock &block = packet->blocks[i];

    block.sob = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);
    block.azimuth = (buf[index + 2] & 0xff) | ((buf[index + 3] & 0xff) << 8);
    index += SOB_ANGLE_SIZE;
    // 40x units
    for (int j = 0; j < LASER_COUNT; j++) {
      Pandar40PUnit &unit = block.units[j];
      uint32_t range = (buf[index] & 0xff) | ((buf[index + 1] & 0xff) << 8);

      // distance is M.
      unit.distance =
          (static_cast<double>(range)) * LASER_RETURN_TO_DISTANCE_RATE;
      unit.intensity = (buf[index + 2] & 0xff);

      // TODO(Philip.Pi): Filtering wrong data for LiDAR.
      if ((unit.distance == 0x010101 && unit.intensity == 0x0101) ||
          unit.distance > (200 * 1000 / 2 /* 200m -> 2mm */)) {
        unit.distance = 0;
        unit.intensity = 0;
      }

      index += RAW_MEASURE_SIZE;
    }
  }

  index += RESERVE_SIZE;  // skip reserved bytes
  packet->spin_speed = (float)((buf[index] & 0xff) | (buf[index + 1] & 0xff) << 8);
  // printf("%f\n", packet->spin_speed);

  index += REVOLUTION_SIZE;

  packet->usec = (buf[index] & 0xff) |
                 (buf[index + 1] & 0xff) << 8 | ((buf[index + 2] & 0xff) << 16) |
                 ((buf[index + 3] & 0xff) << 24);
  packet->usec %= 1000000;

  index += TIMESTAMP_SIZE;
  packet->echo = buf[index] & 0xff;

  index += FACTORY_INFO_SIZE + ECHO_SIZE;

  // parse the UTC Time.

  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  packet->t.tm_year = (buf[index + 0] & 0xff) + 100;

  // in case of time error
  if (packet->t.tm_year >= 200) {
    packet->t.tm_year -= 100;
  }

  // UTC's month start from 1, but mktime only accept month from 0.
  packet->t.tm_mon = (buf[index + 1] & 0xff) - 1;
  packet->t.tm_mday = buf[index + 2] & 0xff;
  packet->t.tm_hour = buf[index + 3] & 0xff;
  packet->t.tm_min = buf[index + 4] & 0xff;
  packet->t.tm_sec = buf[index + 5] & 0xff;
  packet->t.tm_isdst = 0;
  packet->timestamp_point = mktime(&packet->t) + static_cast<double>(packet->usec) / 1000000.0;

  return 0;
}

int PandarGeneral_Internal::ParseL64Data(HS_LIDAR_L64_Packet *packet,
                                         const uint8_t *recvbuf, const int len) {
  if (len != HS_LIDAR_L64_6PACKET_SIZE &&
      len != HS_LIDAR_L64_7PACKET_SIZE &&
      len != HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE &&
      len != HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE &&
      len != 1270) {
    std::cout << "packet size mismatch PandarGeneral_Internal " << len << "," << len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  // Parse 8 Bytes Header
  packet->header.sob = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
  packet->header.chLaserNumber = recvbuf[index + 2] & 0xff;
  packet->header.chBlockNumber = recvbuf[index + 3] & 0xff;
  packet->header.chReturnType = recvbuf[index + 4] & 0xff;
  packet->header.chDisUnit = recvbuf[index + 5] & 0xff;
  index += HS_LIDAR_L64_HEAD_SIZE;

  if (packet->header.sob != 0xEEFF) {
    printf("Error Start of Packet!\n");
    return -1;
  }

  for (block = 0; block < packet->header.chBlockNumber; block++) {
    packet->blocks[block].azimuth = (recvbuf[index] & 0xff) |
                                    ((recvbuf[index + 1] & 0xff) << 8);
    index += HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH;

    int unit;

    for (unit = 0; unit < packet->header.chLaserNumber; unit++) {
      unsigned int unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

      packet->blocks[block].units[unit].distance =
          (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
      packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
      index += HS_LIDAR_L64_UNIT_SIZE;
    }
  }

  index += HS_LIDAR_L64_RESERVED_SIZE;  // skip reserved bytes
  packet->spin_speed = (float)((recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8);
  index += HS_LIDAR_L64_ENGINE_VELOCITY;

  packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
  // printf("timestamp %u \n", packet->timestamp);
  index += HS_LIDAR_L64_TIMESTAMP_SIZE;

  packet->echo = recvbuf[index] & 0xff;

  index += HS_LIDAR_L64_ECHO_SIZE;
  index += HS_LIDAR_L64_FACTORY_SIZE;

  packet->addtime[0] = recvbuf[index] & 0xff;
  packet->addtime[1] = recvbuf[index + 1] & 0xff;
  packet->addtime[2] = recvbuf[index + 2] & 0xff;
  packet->addtime[3] = recvbuf[index + 3] & 0xff;
  packet->addtime[4] = recvbuf[index + 4] & 0xff;
  packet->addtime[5] = recvbuf[index + 5] & 0xff;

  index += HS_LIDAR_TIME_SIZE;
  struct tm tTm = {0};
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  tTm.tm_year = packet->addtime[0] + 100;

  // in case of time error
  if (tTm.tm_year >= 200) {
    tTm.tm_year -= 100;
  }

  // UTC's month start from 1, but mktime only accept month from 0.
  tTm.tm_mon = packet->addtime[1] - 1;
  tTm.tm_mday = packet->addtime[2];
  tTm.tm_hour = packet->addtime[3];
  tTm.tm_min = packet->addtime[4];
  tTm.tm_sec = packet->addtime[5];
  tTm.tm_isdst = 0;
  packet->timestamp_point = mktime(&tTm) + static_cast<double>(packet->timestamp) / 1000000.0;

  return 0;
}

int PandarGeneral_Internal::ParseL20Data(HS_LIDAR_L20_Packet *packet,
                                         const uint8_t *recvbuf, const int len) {
  if (len != HS_LIDAR_L20_PACKET_SIZE) {
    std::cout << "packet size mismatch Pandar20A/B " << len << "," << len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  // Parse 8 Bytes Header
  packet->header.sob = (recvbuf[index] & 0xff) << 8 |
                       ((recvbuf[index + 1] & 0xff));
  packet->header.chLaserNumber = recvbuf[index + 2] & 0xff;
  packet->header.chBlockNumber = recvbuf[index + 3] & 0xff;
  packet->header.chReturnType = recvbuf[index + 4] & 0xff;
  packet->header.chDisUnit = recvbuf[index + 5] & 0xff;
  index += HS_LIDAR_L20_HEAD_SIZE;

  if (packet->header.sob != 0xEEFF) {
    printf("Error Start of Packet!\n");
    return -1;
  }

  for (block = 0; block < packet->header.chBlockNumber; block++) {
    packet->blocks[block].azimuth = (recvbuf[index] & 0xff) |
                                    ((recvbuf[index + 1] & 0xff) << 8);
    index += HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH;

    int unit;
    for (unit = 0; unit < packet->header.chLaserNumber; unit++) {
      unsigned int unRange = (recvbuf[index] & 0xff) |
                             ((recvbuf[index + 1] & 0xff) << 8);

      packet->blocks[block].units[unit].distance =
          (static_cast<double>(unRange * packet->header.chDisUnit)) / 1000.0;
      packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
      index += HS_LIDAR_L20_UNIT_SIZE;
    }
  }

  index += HS_LIDAR_L20_RESERVED_SIZE;  // skip reserved bytes
  packet->spin_speed = (float)((recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8);
  // printf("%f\n", packet->spin_speed);
  index += HS_LIDAR_L20_ENGINE_VELOCITY;

  packet->timestamp = (recvbuf[index] & 0xff) |
                      (recvbuf[index + 1] & 0xff) << 8 | ((recvbuf[index + 2] & 0xff) << 16) |
                      ((recvbuf[index + 3] & 0xff) << 24);
  index += HS_LIDAR_L20_TIMESTAMP_SIZE;

  packet->echo = recvbuf[index] & 0xff;

  index += HS_LIDAR_L20_ECHO_SIZE;
  index += HS_LIDAR_L20_FACTORY_SIZE;

  packet->addtime[0] = recvbuf[index] & 0xff;
  packet->addtime[1] = recvbuf[index + 1] & 0xff;
  packet->addtime[2] = recvbuf[index + 2] & 0xff;
  packet->addtime[3] = recvbuf[index + 3] & 0xff;
  packet->addtime[4] = recvbuf[index + 4] & 0xff;
  packet->addtime[5] = recvbuf[index + 5] & 0xff;

  index += HS_LIDAR_TIME_SIZE;
  struct tm tTm = {0};
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  tTm.tm_year = packet->addtime[0] + 100;

  // in case of time error
  if (tTm.tm_year >= 200) {
    tTm.tm_year -= 100;
  }

  // UTC's month start from 1, but mktime only accept month from 0.
  tTm.tm_mon = packet->addtime[1] - 1;
  tTm.tm_mday = packet->addtime[2];
  tTm.tm_hour = packet->addtime[3];
  tTm.tm_min = packet->addtime[4];
  tTm.tm_sec = packet->addtime[5];
  tTm.tm_isdst = 0;
  packet->timestamp_point = mktime(&tTm) + static_cast<double>(packet->timestamp) / 1000000.0;

  return 0;
}

/**
 * Pandar QT
 */
int PandarGeneral_Internal::ParseQTData(HS_LIDAR_QT_Packet *packet,
                                        const uint8_t *recvbuf, const int len) {
  if (len != HS_LIDAR_QT_PACKET_SIZE &&
      len != HS_LIDAR_QT_PACKET_WITHOUT_UDPSEQ_SIZE) {
    std::cout << "packet size mismatch PandarQT " << len << "," << len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  // Parse 12 Bytes Header
  packet->header.sob = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
  packet->header.chProtocolMajor = recvbuf[index + 2] & 0xff;
  packet->header.chProtocolMinor = recvbuf[index + 3] & 0xff;
  packet->header.chLaserNumber = recvbuf[index + 6] & 0xff;
  packet->header.chBlockNumber = recvbuf[index + 7] & 0xff;
  packet->header.chReturnType = recvbuf[index + 8] & 0xff;
  packet->header.chDisUnit = recvbuf[index + 9] & 0xff;
  index += HS_LIDAR_QT_HEAD_SIZE;

  if (packet->header.sob != 0xEEFF) {
    printf("Error Start of Packet!\n");
    return -1;
  }

  for (block = 0; block < packet->header.chBlockNumber; block++) {
    packet->blocks[block].azimuth = (recvbuf[index] & 0xff) |
                                    ((recvbuf[index + 1] & 0xff) << 8);
    index += HS_LIDAR_QT_BLOCK_HEADER_AZIMUTH;

    int unit;

    for (unit = 0; unit < packet->header.chLaserNumber; unit++) {
      unsigned int unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

      packet->blocks[block].units[unit].distance =
          (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
      packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
      packet->blocks[block].units[unit].confidence = (recvbuf[index + 3] & 0xff);
      index += HS_LIDAR_QT_UNIT_SIZE;
    }
  }

  index += HS_LIDAR_QT_RESERVED_SIZE;  // skip reserved bytes
  packet->spin_speed = (float)((recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8);
  // printf("%f\n", packet->spin_speed);
  index += HS_LIDAR_QT_ENGINE_VELOCITY;

  packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
  // printf("timestamp %u \n", packet->timestamp);
  index += HS_LIDAR_QT_TIMESTAMP_SIZE;

  packet->echo = recvbuf[index] & 0xff;

  index += HS_LIDAR_QT_ECHO_SIZE;
  index += HS_LIDAR_QT_FACTORY_SIZE;

  packet->addtime[0] = recvbuf[index] & 0xff;
  packet->addtime[1] = recvbuf[index + 1] & 0xff;
  packet->addtime[2] = recvbuf[index + 2] & 0xff;
  packet->addtime[3] = recvbuf[index + 3] & 0xff;
  packet->addtime[4] = recvbuf[index + 4] & 0xff;
  packet->addtime[5] = recvbuf[index + 5] & 0xff;

  index += HS_LIDAR_TIME_SIZE;
  struct tm tTm = {0};
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  tTm.tm_year = packet->addtime[0] + 100;

  // in case of time error
  if (tTm.tm_year >= 200) {
    tTm.tm_year -= 100;
  }

  // UTC's month start from 1, but mktime only accept month from 0.
  tTm.tm_mon = packet->addtime[1] - 1;
  tTm.tm_mday = packet->addtime[2];
  tTm.tm_hour = packet->addtime[3];
  tTm.tm_min = packet->addtime[4];
  tTm.tm_sec = packet->addtime[5];
  tTm.tm_isdst = 0;
  packet->timestamp_point = mktime(&tTm) + static_cast<double>(packet->timestamp) / 1000000.0;

  return 0;
}

int PandarGeneral_Internal::ParseXTData(HS_LIDAR_XT_Packet *packet,
                                        const uint8_t *recvbuf, const int len) {
  if (len != HS_LIDAR_XT_PACKET_SIZE && len != HS_LIDAR_XT16_PACKET_SIZE && len != HS_LIDAR_XTM_PACKET_SIZE) {
    std::cout << "packet size mismatch PandarXT " << len << "," << len << std::endl;
    return -1;
  }

  int index = 0;
  int block = 0;
  // Parse 12 Bytes Header
  packet->header.sob = (recvbuf[index] & 0xff) << 8 | ((recvbuf[index + 1] & 0xff));
  packet->header.chProtocolMajor = recvbuf[index + 2] & 0xff;
  packet->header.chProtocolMinor = recvbuf[index + 3] & 0xff;
  packet->header.chLaserNumber = recvbuf[index + 6] & 0xff;
  packet->header.chBlockNumber = recvbuf[index + 7] & 0xff;
  packet->header.chReturnType = recvbuf[index + 8] & 0xff;
  packet->header.chDisUnit = recvbuf[index + 9] & 0xff;
  index += HS_LIDAR_XT_HEAD_SIZE;

  if (packet->header.sob != 0xEEFF) {
    printf("Error Start of Packet!\n");
    return -1;
  }

  for (block = 0; block < packet->header.chBlockNumber; block++) {
    packet->blocks[block].azimuth = (recvbuf[index] & 0xff) |
                                    ((recvbuf[index + 1] & 0xff) << 8);
    index += HS_LIDAR_XT_BLOCK_HEADER_AZIMUTH;

    int unit;

    for (unit = 0; unit < packet->header.chLaserNumber; unit++) {
      unsigned int unRange = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);

      packet->blocks[block].units[unit].distance =
          (static_cast<double>(unRange * packet->header.chDisUnit)) / (double)1000;
      packet->blocks[block].units[unit].intensity = (recvbuf[index + 2] & 0xff);
      packet->blocks[block].units[unit].confidence = (recvbuf[index + 3] & 0xff);
      index += HS_LIDAR_XT_UNIT_SIZE;
    }
  }

  index += HS_LIDAR_XT_RESERVED_SIZE;  // skip reserved bytes

  packet->echo = recvbuf[index] & 0xff;

  index += HS_LIDAR_XT_ECHO_SIZE;
  packet->spin_speed = (float)((recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8);
  // printf("%f\n", packet->spin_speed);
  index += HS_LIDAR_XT_ENGINE_VELOCITY;

  packet->addtime[0] = recvbuf[index] & 0xff;
  packet->addtime[1] = recvbuf[index + 1] & 0xff;
  packet->addtime[2] = recvbuf[index + 2] & 0xff;
  packet->addtime[3] = recvbuf[index + 3] & 0xff;
  packet->addtime[4] = recvbuf[index + 4] & 0xff;
  packet->addtime[5] = recvbuf[index + 5] & 0xff;

  index += HS_LIDAR_XT_UTC_SIZE;

  packet->timestamp = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
                      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
  // printf("timestamp %u \n", packet->timestamp);
  struct tm tTm = {0};
  // UTC's year only include 0 - 99 year , which indicate 2000 to 2099.
  // and mktime's year start from 1900 which is 0. so we need add 100 year.
  tTm.tm_year = packet->addtime[0] + 100;

  // in case of time error
  if (tTm.tm_year >= 200) {
    tTm.tm_year -= 100;
  }

  // UTC's month start from 1, but mktime only accept month from 0.
  tTm.tm_mon = packet->addtime[1] - 1;
  tTm.tm_mday = packet->addtime[2];
  tTm.tm_hour = packet->addtime[3];
  tTm.tm_min = packet->addtime[4];
  tTm.tm_sec = packet->addtime[5];
  tTm.tm_isdst = 0;
  packet->timestamp_point = mktime(&tTm) + static_cast<double>(packet->timestamp) / 1000000.0;
  return 0;
}

int PandarGeneral_Internal::ParseGPS(PandarGPS *packet, const uint8_t *recvbuf,
                                     const int size) {
  if (size != GPS_PACKET_SIZE) {
    return -1;
  }
  int index = 0;
  packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
  index += GPS_PACKET_FLAG_SIZE;
  packet->year =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_YEAR_SIZE;
  packet->month =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MONTH_SIZE;
  packet->day =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_DAY_SIZE;
  packet->second =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_SECOND_SIZE;
  packet->minute =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_MINUTE_SIZE;
  packet->hour =
      (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
  index += GPS_PACKET_HOUR_SIZE;
  packet->fineTime =
      (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
      ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);

  return 0;
}

void PandarGeneral_Internal::CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                                            boost::shared_ptr<SVPointCloud> cld) {
  Pandar40PBlock *block = &pkt->blocks[blockid];

  for (int i = 0; i < LASER_COUNT; ++i) {
    /* for all the units in a block */
    Pandar40PUnit &unit = block->units[i];
    SVPoint point;

    /* skip wrong points */
    // if (unit.distance <= 0.1 || unit.distance > 200.0) {
    //   continue;
    // }

    int azimuth = static_cast<int>(General_horizatal_azimuth_offset_map_[i] * 100 + block->azimuth - GetFiretimeOffset(pkt->spin_speed, laser40Offset_[i]) * 100);
    if (azimuth < 0)
      azimuth += 36000;
    if (azimuth >= 36000)
      azimuth -= 36000;

    point.angle = block->azimuth;

    float xyDistance = unit.distance * m_cos_elevation_map_[i];
    point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
    point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
    point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
    point.intensity = unit.intensity;

    if ("realtime" == m_sTimestampType) {
      point.timestamp = m_dPktTimestamp;
    } else {
      point.timestamp = pkt->timestamp_point + tz_second_;

      if (pkt->echo == 0x39) {
        // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
        point.timestamp = point.timestamp -
                          (static_cast<double>(block40OffsetDual_[blockid] +
                                               laser40Offset_[i]) /
                           1000000.0f);
      } else {
        point.timestamp = point.timestamp -
                          (static_cast<double>(block40OffsetSingle_[blockid] +
                                               laser40Offset_[i]) /
                           1000000.0f);
      }
    }

    point.ring = i;

    if (pcl_type_) {
      m_vPointCloudList[i].push_back(point);
    } else {
      m_vPointCloud[m_iPointCloudIndex] = point;
      m_iPointCloudIndex++;
    }
  }
}

void PandarGeneral_Internal::CalcL64PointXYZIT(HS_LIDAR_L64_Packet *pkt, int blockid,
                                               char chLaserNumber, boost::shared_ptr<SVPointCloud> cld) {
  HS_LIDAR_L64_Block *block = &pkt->blocks[blockid];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    HS_LIDAR_L64_Unit &unit = block->units[i];
    SVPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    int azimuth = static_cast<int>(General_horizatal_azimuth_offset_map_[i] * 100 + block->azimuth - GetFiretimeOffset(pkt->spin_speed, laser64Offset_[i]) * 100);
    if (azimuth < 0)
      azimuth += 36000;
    if (azimuth >= 36000)
      azimuth -= 36000;

    point.angle = block->azimuth;

    float xyDistance = unit.distance * m_cos_elevation_map_[i];
    point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
    point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
    point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);

    point.intensity = unit.intensity;

    if ("realtime" == m_sTimestampType) {
      point.timestamp = m_dPktTimestamp;
    } else {
      point.timestamp = pkt->timestamp_point + tz_second_;

      if (pkt->echo == 0x39) {
        // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
        point.timestamp =
            point.timestamp - (static_cast<double>(block64OffsetDual_[blockid] +
                                                   laser64Offset_[i]) /
                               1000000.0f);
      } else {
        point.timestamp = point.timestamp -
                          (static_cast<double>(block64OffsetSingle_[blockid] + laser64Offset_[i]) /
                           1000000.0f);
      }
    }

    point.ring = i;

    if (pcl_type_) {
      m_vPointCloudList[i].push_back(point);
    } else {
      m_vPointCloud[m_iPointCloudIndex] = point;
      m_iPointCloudIndex++;
    }
  }
}

void PandarGeneral_Internal::CalcL20PointXYZIT(HS_LIDAR_L20_Packet *pkt, int blockid,
                                               char chLaserNumber, boost::shared_ptr<SVPointCloud> cld) {
  HS_LIDAR_L20_Block *block = &pkt->blocks[blockid];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    HS_LIDAR_L20_Unit &unit = block->units[i];
    SVPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    int azimuth = static_cast<int>(General_horizatal_azimuth_offset_map_[i] * 100 + block->azimuth + GetFiretimeOffset(pkt->spin_speed, laser20BOffset_[i]) * 100);
    if (azimuth < 0)
      azimuth += 36000;
    if (azimuth >= 36000)
      azimuth -= 36000;

    point.angle = block->azimuth;

    float xyDistance = unit.distance * m_cos_elevation_map_[i];
    point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
    point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
    point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);

    point.intensity = unit.intensity;

    if ("realtime" == m_sTimestampType) {
      point.timestamp = m_dPktTimestamp;
    } else {
      point.timestamp = pkt->timestamp_point + tz_second_;

      if (pkt->echo == 0x39) {
        // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
        if (strcmp(m_sLidarType.c_str(), "Pandar20A") == 0) {
          point.timestamp = point.timestamp -
                            (static_cast<double>(block20OffsetDual_[blockid] +
                                                 laser20AOffset_[i]) /
                             1000000.0f);
        } else if (strcmp(m_sLidarType.c_str(), "Pandar20B") == 0) {
          point.timestamp = point.timestamp -
                            (static_cast<double>(block20OffsetDual_[blockid] +
                                                 laser20BOffset_[i]) /
                             1000000.0f);
        }
      } else {
        if (strcmp(m_sLidarType.c_str(), "Pandar20A") == 0) {
          point.timestamp = point.timestamp -
                            (static_cast<double>(block20OffsetSingle_[blockid] +
                                                 laser20AOffset_[i]) /
                             1000000.0f);
        } else if (strcmp(m_sLidarType.c_str(), "Pandar20B") == 0) {
          point.timestamp = point.timestamp -
                            (static_cast<double>(block20OffsetSingle_[blockid] +
                                                 laser20BOffset_[i]) /
                             1000000.0f);
        }
      }
    }

    point.ring = i;

    if (pcl_type_) {
      m_vPointCloudList[i].push_back(point);
    } else {
      m_vPointCloud[m_iPointCloudIndex] = point;
      m_iPointCloudIndex++;
    }
  }
}

// QT
void PandarGeneral_Internal::CalcQTPointXYZIT(HS_LIDAR_QT_Packet *pkt, int blockid,
                                              char chLaserNumber, boost::shared_ptr<SVPointCloud> cld) {
  HS_LIDAR_QT_Block *block = &pkt->blocks[blockid];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    HS_LIDAR_QT_Unit &unit = block->units[i];
    SVPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    int azimuth = static_cast<int>(General_horizatal_azimuth_offset_map_[i] * 100 + block->azimuth + GetFiretimeOffset(pkt->spin_speed, laserQTOffset_[i]) * 100);
    if (azimuth < 0)
      azimuth += 36000;
    if (azimuth >= 36000)
      azimuth -= 36000;

    point.angle = block->azimuth;

    if (m_bCoordinateCorrectionFlag) {
      if (m_sin_elevation_map_[i] != 0) {
        float c = (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG +
                   HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT * HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT -
                   unit.distance * unit.distance) *
                  m_sin_elevation_map_[i] * m_sin_elevation_map_[i];
        float b = 2 * m_sin_elevation_map_[i] * m_cos_elevation_map_[i] * (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * m_cos_azimuth_map_[azimuth] - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT * m_sin_azimuth_map_[azimuth]);
        point.z = (-b + sqrt(b * b - 4 * c)) / 2;
        point.x = point.z * m_sin_azimuth_map_[azimuth] * m_cos_elevation_map_[i] / m_sin_elevation_map_[i] - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT;
        point.y = point.z * m_cos_azimuth_map_[azimuth] * m_cos_elevation_map_[i] / m_sin_elevation_map_[i] + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
        if (((point.x + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT) * m_cos_elevation_map_[i] * m_sin_azimuth_map_[azimuth] +
             (point.y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) * m_cos_elevation_map_[i] * m_cos_azimuth_map_[azimuth] +
             point.z * m_sin_elevation_map_[i]) <= 0) {
          point.z = (-b - sqrt(b * b - 4 * c)) / 2;
          point.x = point.z * m_sin_azimuth_map_[azimuth] * m_cos_elevation_map_[i] / m_sin_elevation_map_[i] - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT;
          point.y = point.z * m_cos_azimuth_map_[azimuth] * m_cos_elevation_map_[i] / m_sin_elevation_map_[i] + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
        }
      } else if (m_cos_azimuth_map_[azimuth] != 0) {
        float tan_azimuth = m_sin_azimuth_map_[azimuth] / m_cos_azimuth_map_[azimuth];
        float c = (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * tan_azimuth + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT) *
                      (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * tan_azimuth + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT) -
                  unit.distance * unit.distance;
        float a = 1 + tan_azimuth * tan_azimuth;
        float b = -2 * tan_azimuth * (HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * tan_azimuth + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT);
        point.z = 0;
        point.y = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
        point.x = (point.y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) * tan_azimuth - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT;
        if (((point.x + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT) * m_cos_elevation_map_[i] * m_sin_azimuth_map_[azimuth] +
             (point.y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) * m_cos_elevation_map_[i] * m_cos_azimuth_map_[azimuth] +
             point.z * m_sin_elevation_map_[i]) <= 0) {
          point.z = 0;
          point.y = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
          point.x = (point.y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) * tan_azimuth - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT;
        }
      } else {
        point.x = sqrt(unit.distance * unit.distance - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG);
        point.y = HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
        point.z = 0;
        if (((point.x + HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT) * m_cos_elevation_map_[i] * m_sin_azimuth_map_[azimuth] +
             (point.y - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG) * m_cos_elevation_map_[i] * m_cos_azimuth_map_[azimuth] +
             point.z * m_sin_elevation_map_[i]) <= 0) {
          point.x = -sqrt(unit.distance * unit.distance - HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG * HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG);
          point.y = HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG;
          point.z = 0;
        }
      }
      if (COORDINATE_CORRECTION_CHECK) {
        float xyDistance = unit.distance * m_cos_elevation_map_[i];
        float point_x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);  // without coordinate correction
        float point_y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
        float point_z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
        printf("distance = %f; elevation = %f; azimuth = %f; delta X = %f; delta Y = %f; delta Z = %f; \n",
               unit.distance, pandarGeneral_elev_angle_map[i], float(azimuth / 100), point.x - point_x, point.y - point_y, point.z - point_z);
      }

    } else {
      float xyDistance = unit.distance * m_cos_elevation_map_[i];
      point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);  // without coordinate correction
      point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
      point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
    }
    point.intensity = unit.intensity;

    if ("realtime" == m_sTimestampType) {
      point.timestamp = m_dPktTimestamp;
    } else {
      point.timestamp = pkt->timestamp_point + tz_second_;

      if (pkt->echo == 0x05) {
        // dual return, block 0&1 (2&3 , 4*5 ...)'s timestamp is the same.
        // dual:0x05, single:0x00
        point.timestamp =
            point.timestamp + (static_cast<double>(blockQTOffsetDual_[blockid] +
                                                   laserQTOffset_[i]) /
                               1000000.0f);
      } else {
        point.timestamp = point.timestamp +
                          (static_cast<double>(blockQTOffsetSingle_[blockid] + laserQTOffset_[i]) /
                           1000000.0f);
      }
    }

    point.ring = i;

    if (pcl_type_)
      m_vPointCloudList[i].push_back(point);
    else
      m_vPointCloud[m_iPointCloudIndex] = point;
    m_iPointCloudIndex++;
  }
}

void PandarGeneral_Internal::CalcXTPointXYZIT(HS_LIDAR_XT_Packet *pkt, int blockid,
                                              char chLaserNumber, boost::shared_ptr<SVPointCloud> cld) {
  HS_LIDAR_XT_Block *block = &pkt->blocks[blockid];

  for (int i = 0; i < chLaserNumber; ++i) {
    /* for all the units in a block */
    HS_LIDAR_XT_Unit &unit = block->units[i];
    SVPoint point;

    /* skip wrong points */
    if (unit.distance <= 0.1 || unit.distance > 200.0) {
      continue;
    }

    int azimuth = static_cast<int>(General_horizatal_azimuth_offset_map_[i] * 100 + block->azimuth + GetFiretimeOffset(pkt->spin_speed, laserXTOffset_[i]) * 100);
    if (azimuth < 0)
      azimuth += 36000;
    if (azimuth >= 36000)
      azimuth -= 36000;

    point.angle = block->azimuth;

    if (m_bCoordinateCorrectionFlag) {
      float distance = unit.distance - (m_cos_azimuth_map_h[abs(int(General_horizatal_azimuth_offset_map_[i] * 100))] * m_cos_elevation_map_[i] -
                                        m_sin_azimuth_map_b[abs(int(General_horizatal_azimuth_offset_map_[i] * 100))] * m_cos_elevation_map_[i]);
      float xyDistance = distance * m_cos_elevation_map_[i];
      point.x = xyDistance * m_sin_azimuth_map_[azimuth] - m_cos_azimuth_map_b[azimuth] + m_sin_azimuth_map_h[azimuth];
      point.y = xyDistance * m_cos_azimuth_map_[azimuth] + m_sin_azimuth_map_b[azimuth] + m_cos_azimuth_map_h[azimuth];
      point.z = distance * m_sin_elevation_map_[i];

      if (COORDINATE_CORRECTION_CHECK) {
        float xyDistance = unit.distance * m_cos_elevation_map_[i];
        float point_x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
        float point_y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
        float point_z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
        printf("distance = %f; elevation = %f; azimuth = %f; delta X = %f; delta Y = %f; delta Z = %f; \n",
               unit.distance, pandarGeneral_elev_angle_map[i], float(azimuth / 100), point.x - point_x, point.y - point_y, point.z - point_z);
      }
    } else {
      float xyDistance = unit.distance * m_cos_elevation_map_[i];
      point.x = static_cast<float>(xyDistance * m_sin_azimuth_map_[azimuth]);
      point.y = static_cast<float>(xyDistance * m_cos_azimuth_map_[azimuth]);
      point.z = static_cast<float>(unit.distance * m_sin_elevation_map_[i]);
    }
    point.intensity = unit.intensity;

    if ("realtime" == m_sTimestampType) {
      point.timestamp = m_dPktTimestamp;
    } else {
      point.timestamp = pkt->timestamp_point + tz_second_;

      if (pkt->echo == 0x3d) {
        point.timestamp =
            point.timestamp + (static_cast<double>(blockXTOffsetTriple_[blockid] +
                                                   laserXTOffset_[i]) /
                               1000000.0f);
      } else if (pkt->echo == 0x39 || pkt->echo == 0x3b || pkt->echo == 0x3c) {
        point.timestamp =
            point.timestamp + (static_cast<double>(blockXTOffsetDual_[blockid] +
                                                   laserXTOffset_[i]) /
                               1000000.0f);
      } else {
        point.timestamp = point.timestamp +
                          (static_cast<double>(blockXTOffsetSingle_[blockid] + laserXTOffset_[i]) /
                           1000000.0f);
      }
    }

    point.ring = i;
    if (pcl_type_) {
      m_vPointCloudList[i].push_back(point);
    } else {
      m_vPointCloud[m_iPointCloudIndex] = point;
      m_iPointCloudIndex++;
    }
  }
}

void PandarGeneral_Internal::EmitBackMessege(char chLaserNumber, boost::shared_ptr<SVPointCloud> cld) {
  if (pcl_type_) {
    for (int i = 0; i < chLaserNumber; i++) {
      for (int j = 0; j < m_vPointCloudList[i].size(); j++) {
        cld->push_back(m_vPointCloudList[i][j]);
      }
    }
  } else {
    cld->points.assign(m_vPointCloud.begin(), m_vPointCloud.begin() + m_iPointCloudIndex);
    cld->width = (uint32_t)cld->points.size();
    cld->height = 1;
    m_iPointCloudIndex = 0;
  }
  pcl_callback_(cld, cld->points[0].timestamp);
  if (pcl_type_) {
    for (int i = 0; i < chLaserNumber; i++) {
      m_vPointCloudList[i].clear();
      m_vPointCloudList[i].reserve(MAX_POINT_CLOUD_NUM_PER_CHANNEL);
      cld->points.clear();
      cld->width = (uint32_t)cld->points.size();
      cld->height = 1;
    }
  }
}

void PandarGeneral_Internal::recvAlgorithmPacket() {
  while (m_bEnableLidarAlgorithmRecvThread) {
    boost::this_thread::interruption_point();
    PandarPacket pkt;
    int rc = m_spAlgorithmPktInput->getPacket(&pkt);
    if (0 == rc) {
      pushAlgorithmData(pkt);
    }
  }
}

void PandarGeneral_Internal::ProcessAlgorithmPacket() {
  getProtocolVersion();
  initOffsetByProtocolVersion();
  HS_Object3D_Object_List objectRecvList;
  int totalPacketOfOneFrame = 0;
  int packetNumber = 0;
  while (m_threadLidarAlgorithmProcess) {
    boost::this_thread::interruption_point();

    PandarPacket packet;
    if (0 != popAlgorithmData(&packet)) {
      continue;
    }
    totalPacketOfOneFrame = packet.data[30];
    packetNumber = packet.data[31];
    // printf("totalpacket: %d,packetnumber: %d\n",totalPacketOfOneFrame, packetNumber);
    DecodeUdpData(packet.data, packet.size, &objectRecvList);
    if (((packetNumber + 1) == totalPacketOfOneFrame) || (totalPacketOfOneFrame == 0)) {  // enough packet or empty packet
      m_fAlgorithmCallback(&objectRecvList);
      objectRecvList.valid_size = 0;
      objectRecvList.data.clear();
    }
  }
}

void PandarGeneral_Internal::getProtocolVersion() {
  while (!m_bGetVersion) {
    PandarPacket packet;
    if (0 != popAlgorithmData(&packet)) {
      continue;
    }
    m_iMajorVersion = packet.data[3];
    m_iMinorVersion = packet.data[2];
    m_bGetVersion = true;
    printf("protocol version: %d.%d \n", m_iMajorVersion, m_iMinorVersion);
  }
}

int PandarGeneral_Internal::getMajorVersion() {
  return m_iMajorVersion;
}

int PandarGeneral_Internal::getMinorVersion() {
  return m_iMinorVersion;
}

void PandarGeneral_Internal::initOffsetByProtocolVersion() {
  if (0 == (m_iMinorVersion % 2)) {
    m_iHeaderSize = HEADER_EXTERNAL_LEN;
  } else {
    m_iHeaderSize = HEADER_INTERNAL_LEN;
  }

  if ((2 == m_iMajorVersion) && ((0 == m_iMinorVersion) || (1 == m_iMinorVersion))) {
    m_iRegularInfoLen = REGULAR_INFO_LEN - WGS84_LEN;
  } else if ((2 == m_iMajorVersion) && ((2 == m_iMinorVersion) || (3 == m_iMinorVersion) || (4 == m_iMinorVersion) || (5 == m_iMinorVersion))) {
    m_iRegularInfoLen = REGULAR_INFO_LEN;
  } else {
    printf("version error: %d.%d \n", m_iMajorVersion, m_iMinorVersion);
  }
}

void PandarGeneral_Internal::pushAlgorithmData(PandarPacket packet) {
  pthread_mutex_lock(&m_mutexAlgorithmListLock);
  m_listAlgorithmPacket.push_back(packet);
  sem_post(&m_semAlgorithmList);
  pthread_mutex_unlock(&m_mutexAlgorithmListLock);
}

int PandarGeneral_Internal::popAlgorithmData(PandarPacket *packet) {
  struct timespec ts;
  if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
    printf("get time error\n");
    return -1;
  }
  ts.tv_sec += 1;
  if (sem_timedwait(&m_semAlgorithmList, &ts) == -1) {
    return -1;
  }
  pthread_mutex_lock(&m_mutexAlgorithmListLock);
  *packet = m_listAlgorithmPacket.front();
  m_listAlgorithmPacket.pop_front();
  pthread_mutex_unlock(&m_mutexAlgorithmListLock);
  return 0;
}

int PandarGeneral_Internal::DecodeUdpData(unsigned char *app_data_buff, int data_length, HS_Object3D_Object_List *Object_Recv_Sample) {
  // 1.Null pointer exception or data corruption
  if (NULL == app_data_buff || NULL == Object_Recv_Sample || data_length < m_iHeaderSize + CRC_LEN) {
    printf("Empty pointer or broken data error.\n");
    return -1;
  }

  // 2.Handling empty packages, only including the header
  if (data_length == m_iHeaderSize + CRC_LEN) {
    Object_Recv_Sample->valid_size = 0;
    // printf("Empty package detected\n");
    return 0;
  }

  // 3. Less than one data in the packet
  if (data_length < m_iHeaderSize + INFO_HEAD_LEN + m_iRegularInfoLen + INFO_TAIL_LEN + CRC_LEN) {
    printf("Data length error\n");
    return -2;
  }

  // 4.Extract the data in the package
  data_length = data_length - CRC_LEN - INFO_TAIL_LEN;  // Ignore the last 8bytes of data

  int type = 0;
  int index = m_iHeaderSize + INFO_HEAD_LEN;
  int object_num = Object_Recv_Sample->valid_size;
  HS_Object3D_Data *help_ptr = NULL;
  struct tm *tm_now = {0};
  time_t time_tmp = 0;

  while (index < data_length) {
    type = app_data_buff[index];
    index += 1;
    Object_Recv_Sample->valid_size = app_data_buff[index];
    index += 2;  // Skip the single box attribute size field

    for (size_t i = object_num; i < Object_Recv_Sample->valid_size + object_num; i++) {
      // printf("total: %d,object_num: %d\n",Object_Recv_Sample->valid_size,object_num);
      HS_Object3D_Object info_detection;
      memcpy(&info_detection.data.id, app_data_buff + index, 4);  // index += 4;
      info_detection.type = OBJ_TYPE(type);
      help_ptr = reinterpret_cast<HS_Object3D_Data *>(app_data_buff + index);
      // Timestamp processing, convert the 8-byte timestamp in help_ptr into a structure of year, month, day, hour, minute and second
      // memcpy(&time_tmp, app_data_buff+index, 8);
      // time_tmp /= 1000000000;
      // tm_now = localtime(&time_tmp);
      // help_ptr->timestamp.year = tm_now->tm_year + 1900;
      // help_ptr->timestamp.month = tm_now->tm_mon + 1;
      // help_ptr->timestamp.day = tm_now->tm_mday;
      // help_ptr->timestamp.hour = tm_now->tm_hour;
      // help_ptr->timestamp.minute = tm_now->tm_min;
      // help_ptr->timestamp.second = tm_now->tm_sec;

      info_detection.data.timestamp = help_ptr->timestamp;
      info_detection.data.rect_x = help_ptr->rect_x;
      info_detection.data.rect_y = help_ptr->rect_y;
      info_detection.data.rect_z = help_ptr->rect_z;
      info_detection.data.rect_center_x = help_ptr->rect_center_x;
      info_detection.data.rect_center_y = help_ptr->rect_center_y;
      info_detection.data.rect_center_z = help_ptr->rect_center_z;
      for (size_t i = 0; i < 4; i++) {
        if (i == 3) {
          info_detection.data.yaw[i] = help_ptr->yaw[i];
          break;
        }
        info_detection.data.yaw[i] = help_ptr->yaw[i];
        info_detection.data.relative_velocity[i] = help_ptr->relative_velocity[i];
        info_detection.data.absolute_velocity[i] = help_ptr->absolute_velocity[i];
        info_detection.data.acceleration[i] = help_ptr->acceleration[i];
      }
      for (size_t i = 0; i < 9; i++) {
        info_detection.data.acceleration_cov[i] = help_ptr->acceleration_cov[i];
        info_detection.data.location_cov[i] = help_ptr->location_cov[i];
        info_detection.data.velocity_cov[i] = help_ptr->velocity_cov[i];
      }
      info_detection.data.tracking_confidence = help_ptr->tracking_confidence;
      info_detection.data.is_detection = help_ptr->is_detection;
      if ((2 == m_iMajorVersion) && ((2 == m_iMinorVersion) || (3 == m_iMinorVersion) || (4 == m_iMinorVersion) || (5 == m_iMinorVersion))) {
        info_detection.data.utm_heading = help_ptr->utm_heading;
        info_detection.data.rect_center_WGS84_lon = help_ptr->rect_center_WGS84_lon;
        info_detection.data.rect_center_WGS84_lat = help_ptr->rect_center_WGS84_lat;
        info_detection.data.rect_center_WGS84_ele = help_ptr->rect_center_WGS84_ele;
      }
      index = index + m_iRegularInfoLen;  // Excluding the id (4 bytes), the single box attribute length is 196 or 212 bytes
      Object_Recv_Sample->data.push_back(info_detection);

      // The data length does not meet the requirements, and there is a problem with the incoming data
      if (index > data_length) {
        printf("Data length error\n");
        return -2;
      }
    }
    // Box count
    object_num += Object_Recv_Sample->valid_size;
  }
  Object_Recv_Sample->valid_size = object_num;
  return object_num;
}

void PandarGeneral_Internal::SetEnvironmentVariableTZ() {
  char *TZ;
  if ((TZ = getenv("TZ"))) {
    printf("TZ=%s\n", TZ);
    return;
  }
  unsigned int timezone = 0;
  time_t t1, t2;
  struct tm *tm_local, *tm_utc;
  time(&t1);
  t2 = t1;
  tm_local = localtime(&t1);
  t1 = mktime(tm_local);
  tm_utc = gmtime(&t2);
  t2 = mktime(tm_utc);
  timezone = 0;
  std::string data = "TZ=UTC" + std::to_string(timezone);
  int len = data.length();
  TZ = (char *)malloc((len + 1) * sizeof(char));
  data.copy(TZ, len, 0);
  if (putenv(TZ) == 0) {
    printf("set environment %s\n", TZ);
  } else {
    printf("set environment fail\n");
  }
}

bool PandarGeneral_Internal::GetCorrectionFileFlag() {
  return got_lidar_correction_flag;
}

void PandarGeneral_Internal::SetCorrectionFileFlag(bool flag) {
  got_lidar_correction_flag = flag;
}

float PandarGeneral_Internal::GetFiretimeOffset(float spinSpeed, float deltMicrosecond) {
  return spinSpeed * deltMicrosecond * 0.000006;
}
