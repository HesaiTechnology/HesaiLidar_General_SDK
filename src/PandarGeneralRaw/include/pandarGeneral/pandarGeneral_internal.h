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

#ifndef SRC_PANDARGENERAL_INTERNAL_H_
#define SRC_PANDARGENERAL_INTERNAL_H_

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pthread.h>
#include <semaphore.h>

#include <list>
#include <string>
#include <mutex>

#include <boost/function.hpp>
#include <condition_variable>

#include "pandarGeneral/point_types.h"
#include "input.h"
#include "pandarQT.h"
#include "pandarXT.h"
#include "pcap_reader.h"

#define SOB_ANGLE_SIZE (4)
#define RAW_MEASURE_SIZE (3)
#define LASER_COUNT (40)
#define BLOCKS_PER_PACKET (10)
#define BLOCK_SIZE (RAW_MEASURE_SIZE * LASER_COUNT + SOB_ANGLE_SIZE)
#define TIMESTAMP_SIZE (4)
#define FACTORY_INFO_SIZE (1)
#define ECHO_SIZE (1)
#define RESERVE_SIZE (8)
#define REVOLUTION_SIZE (2)
#define INFO_SIZE (TIMESTAMP_SIZE + FACTORY_INFO_SIZE + ECHO_SIZE + \
    RESERVE_SIZE + REVOLUTION_SIZE)
#define UTC_TIME (6)
#define PACKET_SIZE (BLOCK_SIZE * BLOCKS_PER_PACKET + INFO_SIZE + UTC_TIME)
#define LASER_RETURN_TO_DISTANCE_RATE (0.004)
#define SEQ_NUM_SIZE (4)

/**
 * Pandar 64
 */
#define HS_LIDAR_TIME_SIZE (6)
// Each Packet have 8 byte 
#define HS_LIDAR_L64_HEAD_SIZE (8)
// Block number 6 or 7
#define HS_LIDAR_L64_BLOCK_NUMBER_6 (6)
#define HS_LIDAR_L64_BLOCK_NUMBER_7 (7)

// each block first 2 byte  is azimuth
#define HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH (2)
// each block have  64 Unit
#define HS_LIDAR_L64_UNIT_NUM (64)
// each Unit have 3 byte :2 bytes(distance) + 1 byte(intensity)
#define HS_LIDAR_L64_UNIT_SIZE (3)
// total block size 194
#define HS_LIDAR_L64_BLOCK_SIZE (HS_LIDAR_L64_UNIT_SIZE * \
  HS_LIDAR_L64_UNIT_NUM + HS_LIDAR_L64_BLOCK_HEADER_AZIMUTH)

// Block tail = timestamp ( 4 bytes ) + factory num (2 bytes)
#define HS_LIDAR_L64_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L64_ECHO_SIZE (1)
#define HS_LIDAR_L64_FACTORY_SIZE (1)
#define HS_LIDAR_L64_RESERVED_SIZE (8)
#define HS_LIDAR_L64_ENGINE_VELOCITY (2)

// packet body size two type
#define HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE (HS_LIDAR_L64_BLOCK_SIZE * \
  HS_LIDAR_L64_BLOCK_NUMBER_6)
#define HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE (HS_LIDAR_L64_BLOCK_SIZE * \
HS_LIDAR_L64_BLOCK_NUMBER_7)

// packet tail size 
#define HS_LIDAR_L64_PACKET_TAIL_SIZE (26)
#define HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE (22)

//total packet size two type,length: 1198 and 1392
#define HS_LIDAR_L64_6PACKET_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_SIZE)
#define HS_LIDAR_L64_7PACKET_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_SIZE)

#define HS_LIDAR_L64_6PACKET_WITHOUT_UDPSEQ_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_6_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)
#define HS_LIDAR_L64_7PACKET_WITHOUT_UDPSEQ_SIZE ( HS_LIDAR_L64_HEAD_SIZE + \
HS_LIDAR_L64_7_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L64_PACKET_TAIL_WITHOUT_UDPSEQ_SIZE)

/**
 * Pandar20
 */
#define HS_LIDAR_L20_HEAD_SIZE (8)
#define HS_LIDAR_L20_BLOCK_NUMBER (20)
#define HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH (2)
#define HS_LIDAR_L20_UNIT_NUM (20)
#define HS_LIDAR_L20_UNIT_SIZE (3)
#define HS_LIDAR_L20_BLOCK_SIZE (HS_LIDAR_L20_UNIT_SIZE * \
    HS_LIDAR_L20_UNIT_NUM + HS_LIDAR_L20_BLOCK_HEADER_AZIMUTH)
#define HS_LIDAR_L20_TIMESTAMP_SIZE (4)
#define HS_LIDAR_L20_ECHO_SIZE (1)
#define HS_LIDAR_L20_FACTORY_SIZE (1)
#define HS_LIDAR_L20_RESERVED_SIZE (8)
#define HS_LIDAR_L20_ENGINE_VELOCITY (2)
#define HS_LIDAR_L20_BLOCK_PACKET_BODY_SIZE (HS_LIDAR_L20_BLOCK_SIZE * \
    HS_LIDAR_L20_BLOCK_NUMBER)
#define HS_LIDAR_L20_PACKET_TAIL_SIZE (22)
#define HS_LIDAR_L20_PACKET_SIZE ( HS_LIDAR_L20_HEAD_SIZE + \
    HS_LIDAR_L20_BLOCK_PACKET_BODY_SIZE + HS_LIDAR_L20_PACKET_TAIL_SIZE)


#define GPS_PACKET_SIZE (512)
#define GPS_PACKET_FLAG_SIZE (2)
#define GPS_PACKET_YEAR_SIZE (2)
#define GPS_PACKET_MONTH_SIZE (2)
#define GPS_PACKET_DAY_SIZE (2)
#define GPS_PACKET_HOUR_SIZE (2)
#define GPS_PACKET_MINUTE_SIZE (2)
#define GPS_PACKET_SECOND_SIZE (2)
#define GPS_ITEM_NUM (7)

#define HesaiLidarSDK_DEFAULT_LIDAR_RECV_PORT 8080
#define HesaiLidarSDK_DEFAULT_GPS_RECV_PORT 10110

#define MAX_LASER_NUM (256)
#define MAX_POINT_CLOUD_NUM (1000000)
#define MAX_POINT_CLOUD_NUM_PER_CHANNEL (10000)
#define MAX_AZIMUTH_DEGREE_NUM (36000)
#define HS_LIDAR_XT_COORDINATE_CORRECTION_H (0.0315)
#define HS_LIDAR_XT_COORDINATE_CORRECTION_B (0.013)
#define HS_LIDAR_XTM_COORDINATE_CORRECTION_H (0.0305)
#define HS_LIDAR_XTM_COORDINATE_CORRECTION_B (0.013)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_ODOG (0.0298)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_ODOT (0.0072)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_F (0.0295)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_I0 (0.0006)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_S0 (0.00017)
#define HS_LIDAR_QT_COORDINATE_CORRECTION_D0 (20)
#define COORDINATE_CORRECTION_CHECK (false)

#define PKT_ARRAY_SIZE 36000

struct Pandar40PUnit_s {
  uint8_t intensity;
  double distance;
};
typedef struct Pandar40PUnit_s Pandar40PUnit;

struct Pandar40PBlock_s {
  uint16_t azimuth;
  uint16_t sob;
  Pandar40PUnit units[LASER_COUNT];
};
typedef struct Pandar40PBlock_s Pandar40PBlock;

struct Pandar40PPacket_s {
  Pandar40PBlock blocks[BLOCKS_PER_PACKET];
  struct tm t;
  uint32_t usec;
  int echo;
  double timestamp_point;
};
typedef struct Pandar40PPacket_s Pandar40PPacket;

/************Pandar64*******************************/
typedef struct HS_LIDAR_L64_Header_s{
    unsigned short sob;     // 0xFFEE 2bytes
    char chLaserNumber;     // laser number 1byte
    char chBlockNumber;     //block number 1byte
    char chReturnType;      // return mode 1 byte  when dual return 0-Single Return 
                            // 1-The first block is the 1 st return. 
                            // 2-The first block is the 2 nd return
    char chDisUnit;         // Distance unit, 6mm/5mm/4mm
    public:
    HS_LIDAR_L64_Header_s() {
        sob = 0;
        chLaserNumber = 0;
        chBlockNumber = 0;
        chReturnType = 0;
        chDisUnit = 0;
    }
} HS_LIDAR_L64_Header;

typedef struct HS_LIDAR_L64_Unit_s{
    double distance;
    unsigned short intensity;
} HS_LIDAR_L64_Unit;


typedef struct HS_LIDAR_L64_Block_s{
    unsigned short azimuth; // packet angle  ,Azimuth = RealAzimuth * 100
    HS_LIDAR_L64_Unit units[HS_LIDAR_L64_UNIT_NUM];
} HS_LIDAR_L64_Block;

typedef struct HS_LIDAR_L64_Packet_s{
    HS_LIDAR_L64_Header header;
    HS_LIDAR_L64_Block blocks[HS_LIDAR_L64_BLOCK_NUMBER_7];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
    double timestamp_point;
} HS_LIDAR_L64_Packet;
/***************Pandar64****************************/

/************Pandar20A/B*******************************/
typedef struct HS_LIDAR_L20_Header_s{
    unsigned short sob;     // 0xFFEE 2bytes
    char chLaserNumber;     // laser number 1byte
    char chBlockNumber;     //block number 1byte
    char chReturnType;      // return mode 1 byte  when dual return 0-Single Return 
                            // 1-The first block is the 1 st return. 
                            // 2-The first block is the 2 nd return
    char chDisUnit;         // Distance unit, 6mm/5mm/4mm
    public:
    HS_LIDAR_L20_Header_s() {
        sob = 0;
        chLaserNumber = 0;
        chBlockNumber = 0;
        chReturnType = 0;
        chDisUnit = 0;
    }
} HS_LIDAR_L20_Header;

typedef struct HS_LIDAR_L20_Unit_s{
    double distance;
    unsigned short intensity;
} HS_LIDAR_L20_Unit;


typedef struct HS_LIDAR_L20_Block_s{
    unsigned short azimuth;
    HS_LIDAR_L20_Unit units[HS_LIDAR_L20_UNIT_NUM];
} HS_LIDAR_L20_Block;

typedef struct HS_LIDAR_L20_Packet_s{
    HS_LIDAR_L20_Header header;
    HS_LIDAR_L20_Block blocks[HS_LIDAR_L20_BLOCK_NUMBER];
    unsigned int timestamp; // ms
    unsigned int echo;
    unsigned char addtime[6];
    double timestamp_point;
} HS_LIDAR_L20_Packet;
/************Pandar20A/B*******************************/


struct PandarGPS_s {
  uint16_t flag;
  uint16_t year;
  uint16_t month;
  uint16_t day;
  uint16_t second;
  uint16_t minute;
  uint16_t hour;
  uint32_t fineTime;
};
typedef struct PandarGPS_s PandarGPS;

#define ROTATION_MAX_UNITS (36001)

#define HEADER_EXTERNAL_LEN (88) //Publish UDP header length externally
#define HEADER_INTERNAL_LEN (108) //Internal test UDP header length
#define CRC_LEN (4) //UDP packet end is 4, crc32 check bit
#define INFO_HEAD_LEN (4) //Information body start tag, length 4
#define INFO_TAIL_LEN (4) //The end of the message body tag, length 4
#define REGULAR_INFO_LEN (216) // Single box attribute size
#define WGS84_LEN (16) //WGS84 data structure size

//Target object type enum constant
typedef enum {
     SMALLCAR = 0, //Car
     PEOPLE = 1, //Pedestrian
     NonMobile = 2, //Non-motorized vehicles
     LARGECAR = 3, //Large car
     Unknown = 4 //other
}OBJ_TYPE;

#pragma pack(1) 
typedef struct{
    short year;
    char month;
    char day;
    char hour;
    char minute;
    char second;
    char microsecond;
}Utc_Time;
#pragma pack()

#pragma pack(4)
typedef struct {
    unsigned int id; //!< Detection object identification id
    uint64_t timestamp; //!< The timestamp of the end of object detection
    float yaw[4]; // Quadruple of object pose
    float rect_x; //!< Object detection frame size xyz(m)
    float rect_y; //!< Object detection frame size xyz(m)
    float rect_z; //!< Object detection frame size xyz(m)

    float rect_center_x; //!< Center of object detection frame xyz(m)
    float rect_center_y; //!< Center of object detection frame xyz(m)
    float rect_center_z; //!< Center of object detection frame xyz(m)

    float relative_velocity[3]; //!< Object relative velocity xyz component (m/s)
    float absolute_velocity[3]; //!< Object absolute velocity xyz component (m/s)
    float acceleration[3]; //!< Object relative acceleration xyz component (m/s^2)

    float location_cov[9]; //!< Object position uncertainty matrix
    float velocity_cov[9]; //!< Object velocity uncertainty matrix
    float acceleration_cov[9]; //!< Object acceleration uncertainty matrix
    char tracking_confidence; //!< Object tracking confidence
    char is_detection; //!< Whether it is a real detection target, 0 means complement object, 1 means real object
    char reserved[2]; // fill in 　4bytes
    float utm_heading; // WGS84 detects object heading angle (angle with true north)
    float rect_center_WGS84_lon; //!< WGS84 detection center longitude
    float rect_center_WGS84_lat; //!< WGS84 detection center latitude
    float rect_center_WGS84_ele; //!< WGS84 detection center altitude
} HS_Object3D_Data;
#pragma pack()

typedef struct {
    OBJ_TYPE type;
    HS_Object3D_Data  data;
} HS_Object3D_Object;

typedef struct {
    int valid_size;                //!< Total number of objects
    std::vector<HS_Object3D_Object> data;  //!< Object data array
} HS_Object3D_Object_List;

typedef std::array<PandarPacket, PKT_ARRAY_SIZE> PktArray;

typedef struct PacketsBuffer_s {
private:
  std::mutex mutex;
  PktArray m_buffers{};
  PktArray::iterator m_iterPush;
  PktArray::iterator m_iterPop;
  bool m_full;
  std::condition_variable buffer_not_full;
  std::condition_variable buffer_not_empty;
public:
  inline PacketsBuffer_s() {
    m_iterPush = m_buffers.begin();
    m_iterPop = m_buffers.begin();
    m_full = false;
  }
  inline int push_back(const PandarPacket& pkt) {
    std::unique_lock<std::mutex> lock(mutex);

    //    if (m_full) {
    //      printf("Pandar: buffer doesn't have space!\n");
    //    return 0;
    //    }

    while(m_full)
    {
      buffer_not_full.wait(lock);
    }

    *m_iterPush = pkt;
    ++m_iterPush;
    if (m_iterPush == m_buffers.end()) {
      m_iterPush = m_buffers.begin();
    }

    if (m_iterPush == m_iterPop) {
      m_full = true;
    }

    buffer_not_empty.notify_all();

    return 1;

  }

  inline bool pop(PandarPacket& packet) {
    std::unique_lock<std::mutex> lock(mutex);

    //    if (!m_full && m_iterPush == m_iterPop) {
    //      return false;
    //    }

    while(!m_full && m_iterPush == m_iterPop)
    {
      buffer_not_empty.wait(lock);
    }

    packet = *m_iterPop;

    m_iterPop++;
    if (m_buffers.end() == m_iterPop) {
      m_iterPop = m_buffers.begin();
    }
    m_full = false;
    buffer_not_full.notify_all();

    return true;
  }
} PacketsBuffer;


class PandarGeneral_Internal {
 public:
  /**
   * @brief Constructor
   * @param device_ip  				The ip of the device
   *        lidar_port 				The port number of lidar data
   *        gps_port   				The port number of gps data
   *        pcl_callback      The callback of PCL data structure
   *        gps_callback      The callback of GPS structure
   *        type       				The device type
   */
  PandarGeneral_Internal(
      std::string device_ip, uint16_t lidar_port, uint16_t lidar_algorithm_port, uint16_t gps_port,
      boost::function<void(boost::shared_ptr<PPointCloud>, double)>
          pcl_callback,
          boost::function<void(HS_Object3D_Object_List*)> algorithm_callback,
          boost::function<void(double)> gps_callback, 
          uint16_t start_angle, int tz, int pcl_type, std::string lidar_type, std::string frame_id, std::string timestampType,
          std::string lidar_correction_file, std::string multicast_ip, bool coordinate_correction_flag);

  /**
   * @brief Constructor
   * @param pcap_path         The path of pcap file
   *        pcl_callback      The callback of PCL data structure
   *        start_angle       The start angle of frame
   *        tz                The timezone
   *        pcl_type          Structured Pointcloud
   *        frame_id          The frame id of pcd
   */
  PandarGeneral_Internal(
      std::string pcap_path, \
      boost::function<void(boost::shared_ptr<PPointCloud>, double)> \
      pcl_callback, uint16_t start_angle, int tz, int pcl_type, \
      std::string lidar_type, std::string frame_id, std::string timestampType, bool coordinate_correction_flag);// the default timestamp type is LiDAR time
  ~PandarGeneral_Internal();

  /**
   * @brief load the correction file
   * @param correction The path of correction file
   */
  int LoadCorrectionFile(std::string correction);

  /**
   * @brief load the correction file
   * @param angle The start angle
   */
  void ResetStartAngle(uint16_t start_angle);

  void Start();
  void Stop();
  bool GetCorrectionFileFlag();
  void SetCorrectionFileFlag(bool flag);

    /*
    @Description：Udp byte stream analysis function
    @Params     ：app_data_buff, Byte stream received by udp packet
    @Params     ：data_length，  the total length of the byte stream
    @Params     ：Object_Recv_Sample， the structure used to store data
    @Return     ： 0 -> parsed successfully
    @Return     ：-1 -> Null pointer or UDP packet is abnormal
    @Return     ：-2 ->The data length does not meet the requirements
    */
    int DecodeUdpData(unsigned char* app_data_buff, int data_length, HS_Object3D_Object_List* Object_Recv_Sample);

        /**
    * @brief get major version.
    * @Return   ： major version
    */    
    int getMajorVersion();

    /**
    * @brief get minor version.
    * @Return   ： minor version
    */    
    int getMinorVersion();
    
    private:

    /**
    * @brief get protocol version from LiDAR.
    */
    void getProtocolVersion();

    /**
    * @brief init offset by protocol version.
    */
    void initOffsetByProtocolVersion();

    /**
    * @brief start receive packet from LiDAR.
    */
    void recvAlgorithmPacket();

    /**
    * @brief start parse packet from LiDAR.
    */
    void ProcessAlgorithmPacket();

        /**
    * @brief push algorithm packet data to list.
    * @Params     ：packet, data packet
    */
    void pushAlgorithmData(PandarPacket packet);

    /**
    * @brief pop algorithm packet data from list.
    * @Params     ：packet, data packet
    */
    int popAlgorithmData(PandarPacket *packet);

 private:
  void Init();
  void RecvTask();
  void ProcessGps(const PandarGPS &gpsMsg);
  void ProcessLiarPacket();
  void PushLiDARData(PandarPacket packet);
  int ParseRawData(Pandar40PPacket *packet, const uint8_t *buf, const int len);
  int ParseL64Data(HS_LIDAR_L64_Packet *packet, const uint8_t *recvbuf, const int len);
  int ParseL20Data(HS_LIDAR_L20_Packet *packet, const uint8_t *recvbuf, const int len);
  int ParseQTData(HS_LIDAR_QT_Packet *packet, const uint8_t *recvbuf, const int len);
  int ParseXTData(HS_LIDAR_XT_Packet *packet, const uint8_t *recvbuf, const int len);

  int ParseGPS(PandarGPS *packet, const uint8_t *recvbuf, const int size);
  void CalcPointXYZIT(Pandar40PPacket *pkt, int blockid,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcL64PointXYZIT(HS_LIDAR_L64_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcL20PointXYZIT(HS_LIDAR_L20_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcQTPointXYZIT(HS_LIDAR_QT_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void CalcXTPointXYZIT(HS_LIDAR_XT_Packet *pkt, int blockid, char chLaserNumber,
                      boost::shared_ptr<PPointCloud> cld);
  void FillPacket(const uint8_t *buf, const int len, double timestamp);

  void EmitBackMessege(char chLaserNumber, boost::shared_ptr<PPointCloud> cld);
  void SetEnvironmentVariableTZ();
  pthread_mutex_t lidar_lock_;
  sem_t lidar_sem_;
  boost::thread *lidar_recv_thr_;
  boost::thread *lidar_process_thr_;
  bool enable_lidar_recv_thr_;
  bool enable_lidar_process_thr_;
  int start_angle_;
  std::string m_sTimestampType;
  double m_dPktTimestamp;
  uint16_t m_u16LidarAlgorithmPort;
  pthread_mutex_t m_mutexAlgorithmListLock;
  sem_t m_semAlgorithmList;
  boost::thread *m_threadLidarAlgorithmRecv;
  boost::thread *m_threadLidarAlgorithmProcess;
  bool m_bEnableLidarAlgorithmRecvThread;
  bool m_bEnableLidarAlgorithmProcessThread;
  bool m_bGetVersion;
  int m_iMajorVersion;
  int m_iMinorVersion;
  int m_iHeaderSize;
  int m_iRegularInfoLen;
  std::list<PandarPacket> m_listAlgorithmPacket;
  boost::shared_ptr<Input> m_spAlgorithmPktInput;
  boost::function<void(HS_Object3D_Object_List*)> m_fAlgorithmCallback;

  std::list<struct PandarPacket_s> lidar_packets_;

  boost::shared_ptr<Input> input_;
  boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)>
      pcl_callback_;
  boost::function<void(double timestamp)> gps_callback_;

  float sin_lookup_table_[ROTATION_MAX_UNITS];
  float cos_lookup_table_[ROTATION_MAX_UNITS];

  uint16_t last_azimuth_;
  double last_timestamp_;

  float elev_angle_map_[LASER_COUNT];
  float horizatal_azimuth_offset_map_[LASER_COUNT];

  float General_elev_angle_map_[MAX_LASER_NUM];
  float General_horizatal_azimuth_offset_map_[MAX_LASER_NUM];

  float Pandar20_elev_angle_map_[HS_LIDAR_L20_UNIT_NUM];
  float Pandar20_horizatal_azimuth_offset_map_[HS_LIDAR_L20_UNIT_NUM];

  float PandarQT_elev_angle_map_[HS_LIDAR_QT_UNIT_NUM];
  float PandarQT_horizatal_azimuth_offset_map_[HS_LIDAR_QT_UNIT_NUM];

  float PandarXT_elev_angle_map_[HS_LIDAR_XT_UNIT_NUM];
  float PandarXT_horizatal_azimuth_offset_map_[HS_LIDAR_XT_UNIT_NUM];

  float block64OffsetSingle_[HS_LIDAR_L64_BLOCK_NUMBER_6];
  float block64OffsetDual_[HS_LIDAR_L64_BLOCK_NUMBER_6];
  float laser64Offset_[HS_LIDAR_L64_UNIT_NUM];

  float block40OffsetSingle_[BLOCKS_PER_PACKET];
  float block40OffsetDual_[BLOCKS_PER_PACKET];
  float laser40Offset_[LASER_COUNT];

  float block20OffsetSingle_[HS_LIDAR_L20_BLOCK_NUMBER];
  float block20OffsetDual_[HS_LIDAR_L20_BLOCK_NUMBER];
  float laser20AOffset_[HS_LIDAR_L20_UNIT_NUM];
  float laser20BOffset_[HS_LIDAR_L20_UNIT_NUM];

  float blockQTOffsetSingle_[HS_LIDAR_QT_BLOCK_NUMBER];
  float blockQTOffsetDual_[HS_LIDAR_QT_BLOCK_NUMBER];
  float laserQTOffset_[HS_LIDAR_QT_UNIT_NUM];

  float blockXTOffsetSingle_[HS_LIDAR_XT_BLOCK_NUMBER];
  float blockXTOffsetDual_[HS_LIDAR_XT_BLOCK_NUMBER];
  float blockXTOffsetTriple_[HS_LIDAR_XT_BLOCK_NUMBER];
  float laserXTOffset_[HS_LIDAR_XT_UNIT_NUM];

  int tz_second_;
  std::string frame_id_;
  int pcl_type_;
  PcapReader *pcap_reader_;
  bool connect_lidar_;
  std::string m_sLidarType;
  std::vector<float> m_sin_azimuth_map_;
  std::vector<float> m_cos_azimuth_map_;
  std::vector<float> m_sin_elevation_map_;
  std::vector<float> m_cos_elevation_map_;
  std::vector<float> m_sin_azimuth_map_h;
  std::vector<float> m_cos_azimuth_map_h;
  std::vector<float> m_sin_azimuth_map_b;
  std::vector<float> m_cos_azimuth_map_b;
  bool got_lidar_correction_flag;
  std::string correction_file_path_;
  PacketsBuffer m_PacketsBuffer;
  bool m_bCoordinateCorrectionFlag;
  uint16_t m_iAzimuthRange;

};

#endif  // SRC_PANDARGENERAL_INTERNAL_H_
